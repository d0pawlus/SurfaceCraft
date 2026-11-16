//! The viewport camera on the desktop control channel (see [`crate::control`]): `ui.camera_get`
//! reads it and `ui.camera_set` changes it, so that an agent can take comparable `ui.screenshot`s
//! (a screenshot draws the camera of the app) and put a view back exactly.
//!
//! These are UI requests like `ui.screenshot`, not engine commands (`view.*`): the camera is state
//! of this crate's viewport, not of the document or the session, the engine (layer L4) cannot see
//! the UI (layer L5), and a headless session, the CLI and scripts have no viewport to move.
//!
//! # The camera
//!
//! Both requests speak the same object; the reply of either is `{"ok": camera}`, or
//! `{"err": "message"}` when the request is refused (nothing changes then).
//!
//! | field | meaning |
//! |---|---|
//! | `projection` | `"perspective"` (the only one the viewport has) |
//! | `target` | `[x, y, z]`, the point the camera orbits and looks at, in model units (mm), each coordinate within 1e12 |
//! | `distance` | from the target to the eye, model units, in 1e-3 to 1e7 (the limits of zooming) |
//! | `yaw_rad` | azimuth around +Z in radians, in -1e6 to 1e6 (a camera turned further than that is reported wrapped into (-pi, pi], the same view): the eye is at `target + distance * (cos(pitch) cos(yaw), cos(pitch) sin(yaw), sin(pitch))`, Z is up |
//! | `pitch_rad` | elevation in radians, within the poles (at most `POLE_PITCH` either way, a thousandth of a radian short of pi/2) |
//! | `fov_rad` | vertical field of view in radians, in 0.01 to 3 |
//!
//! # `ui.camera_set`
//!
//! Every field is optional (`null` counts as absent); a partial request changes only the given
//! fields. Besides them it takes `view`, a standard view of the view cube (`front`, `back`, `left`,
//! `right`, `top`, `bottom`, `iso`), and `fit`, a boolean. They apply in this order:
//!
//! 1. `view` turns the orientation (`yaw_rad`, `pitch_rad`) to the standard view at once, without
//!    the animation of the view cube; target, distance and field of view stay;
//! 2. the explicit fields (`target`, `distance`, `yaw_rad`, `pitch_rad`, `fov_rad`) are written;
//! 3. `fit` frames the document as `ui.screenshot` draws it: the bounding box of the document, even
//!    while the auto-align tool shows a proposed pose (a screenshot does not draw the proposal, so the
//!    navigation bar's fit-all, which frames the proposal, would frame empty space). It replaces
//!    `target` and `distance` (the distance stays within the limits of zooming) and keeps the
//!    orientation and the field of view as they are by then. On an empty document it changes nothing.
//!
//! The whole request is checked before anything changes: a number that is not finite or out of its
//! range, a wrong type, an unknown field, an unknown view or `projection` (`orthographic` is
//! refused: the viewport has no such camera) is an error and leaves the camera as it was.
//!
//! Both requests first complete what the app has pending for the camera: a running view animation
//! jumps to its end and a fit-all requested by the UI (after an import, say) is carried out, so the
//! answer is the camera the next frame draws. `ui.screenshot` does the same first, so it draws the
//! camera `ui.camera_get` reports. A fit-all the UI requested is carried out as the frame carries it
//! out (a proposed alignment is framed in its proposed pose).
//!
//! What `ui.camera_get` reports is always accepted by `ui.camera_set`: the ranges above hold every
//! camera the UI can produce (zoom, pan, orbit, view animation, fit), which is why fit keeps the
//! distance within the zoom limits.
use crate::app::SurfaceCraftApp;
use crate::camera::{ANIM_SECONDS, Camera, MAX_DISTANCE, MIN_DISTANCE, POLE_PITCH, StandardView};
use serde::Deserialize;
use serde_json::{Value, json};
use surfacecraft_geom::{Aabb, Point3};

/// Largest magnitude of a target coordinate (model units).
const MAX_COORD: f64 = 1e12;
/// Range of the distance from target to eye (model units): the range zooming keeps it in.
const DISTANCE: (f64, f64) = (MIN_DISTANCE, MAX_DISTANCE);
/// Range of the vertical field of view (radians).
const FOV: (f64, f64) = (0.01, 3.0);
/// Largest magnitude of the azimuth (radians): thousands of turns, far beyond any orbiting.
const MAX_YAW: f64 = 1e6;
/// How far past the poles a pitch may be and still count as the pole (a pole view written to JSON
/// and read back may differ in the last digit).
const PITCH_SLACK: f64 = 1e-6;
/// Longest error message (characters): a hostile name is not echoed in full.
const MAX_MESSAGE_CHARS: usize = 300;
/// Longest name quoted in an error (characters).
const MAX_NAME_CHARS: usize = 32;

/// The standard views by the name the request uses.
const VIEWS: [(&str, StandardView); 7] = [
    ("front", StandardView::Front),
    ("back", StandardView::Back),
    ("left", StandardView::Left),
    ("right", StandardView::Right),
    ("top", StandardView::Top),
    ("bottom", StandardView::Bottom),
    ("iso", StandardView::Iso),
];

/// The parameters of `ui.camera_set` as they arrive.
#[derive(Default, Deserialize)]
#[serde(deny_unknown_fields)]
struct SetParams {
    view: Option<String>,
    fit: Option<bool>,
    projection: Option<String>,
    target: Option<[f64; 3]>,
    distance: Option<f64>,
    yaw_rad: Option<f64>,
    pitch_rad: Option<f64>,
    fov_rad: Option<f64>,
}

/// `ui.camera_set` after validation: everything in it is safe to write.
#[derive(Default)]
struct Update {
    view: Option<StandardView>,
    fit: bool,
    target: Option<Point3>,
    distance: Option<f64>,
    yaw: Option<f64>,
    pitch: Option<f64>,
    fov: Option<f64>,
}

/// `ui.camera_get`: the viewport camera. Takes no parameters.
pub fn camera_get(app: &mut SurfaceCraftApp, params: &Value) -> Value {
    let none = match params {
        Value::Null => true,
        Value::Object(fields) => fields.is_empty(),
        _ => false,
    };
    if !none {
        return json!({"err": "ui.camera_get takes no parameters"});
    }
    settle(app);
    json!({"ok": camera_json(&app.camera)})
}

/// `ui.camera_set`: changes the viewport camera (see the module documentation for the order) and
/// answers with the camera it has become.
pub fn camera_set(app: &mut SurfaceCraftApp, params: &Value) -> Value {
    let update = match parse_set(params) {
        Ok(update) => update,
        Err(message) => return json!({"err": message}),
    };
    settle(app);
    apply(app, &update);
    json!({"ok": camera_json(&app.camera)})
}

/// The camera as both requests show it.
fn camera_json(c: &Camera) -> Value {
    json!({
        "projection": "perspective",
        "target": [c.target.x, c.target.y, c.target.z],
        "distance": c.distance,
        "yaw_rad": reported_yaw(c.yaw),
        "pitch_rad": c.pitch,
        "fov_rad": c.fov,
    })
}

/// The azimuth as reported: as it is, or, past the range `ui.camera_set` takes (a hundred thousand
/// turns of dragging), the same direction wrapped into (-pi, pi].
fn reported_yaw(yaw: f64) -> f64 {
    if yaw.abs() <= MAX_YAW {
        return yaw;
    }
    let turned = yaw.rem_euclid(std::f64::consts::TAU);
    if turned > std::f64::consts::PI { turned - std::f64::consts::TAU } else { turned }
}

/// Completes what the app has pending for the camera, as the next frame would: a running view
/// animation jumps to its end and a requested fit-all is carried out (a proposed alignment is
/// framed in its proposed pose, otherwise the document). `ui.screenshot` calls it too, so that it
/// draws the camera `ui.camera_get` reports.
pub(crate) fn settle(app: &mut SurfaceCraftApp) {
    app.tick_view_animation(ANIM_SECONDS);
    if app.fit_pending {
        let shown = crate::align_proposal::framed_box(app).unwrap_or_else(|| app.session.doc.bbox());
        frame(app, &shown);
        app.fit_pending = false;
    }
}

/// `ui.camera_set` with `fit`: frames the document, which is what `ui.screenshot` draws (never the
/// box of a proposed alignment, which a screenshot does not show).
fn fit_document(app: &mut SurfaceCraftApp) {
    let shown = app.session.doc.bbox();
    frame(app, &shown);
}

/// Frames `shown`. The distance stays within the limits of zooming and a result that is not a
/// camera `ui.camera_set` takes (a box at the end of the number range) leaves the camera as it was.
fn frame(app: &mut SurfaceCraftApp, shown: &Aabb) {
    let before = app.camera.clone();
    app.camera.fit(shown);
    let cam = &mut app.camera;
    let near_enough = cam.target.coords.iter().all(|v| v.abs() <= MAX_COORD);
    if near_enough && cam.distance.is_finite() {
        cam.distance = cam.distance.clamp(MIN_DISTANCE, MAX_DISTANCE);
    } else {
        *cam = before;
    }
}

/// Writes a validated update: the view, then the explicit fields, then the fit.
fn apply(app: &mut SurfaceCraftApp, update: &Update) {
    let cam = &mut app.camera;
    if let Some(view) = update.view {
        cam.set_standard(view);
    }
    if let Some(target) = update.target {
        cam.target = target;
    }
    if let Some(distance) = update.distance {
        cam.distance = distance;
    }
    if let Some(yaw) = update.yaw {
        cam.yaw = yaw;
    }
    if let Some(pitch) = update.pitch {
        cam.pitch = pitch;
    }
    if let Some(fov) = update.fov {
        cam.fov = fov;
    }
    if update.fit {
        fit_document(app);
    }
}

/// Reads and checks the parameters of `ui.camera_set`; the error says what to change.
fn parse_set(params: &Value) -> Result<Update, String> {
    let raw = match params {
        Value::Null => SetParams::default(),
        Value::Object(_) => SetParams::deserialize(params)
            .map_err(|e| clip(&format!("ui.camera_set: invalid parameters: {e}"), MAX_MESSAGE_CHARS))?,
        _ => return Err("ui.camera_set: the parameters must be a JSON object".into()),
    };
    check_projection(raw.projection.as_deref())?;
    Ok(Update {
        view: raw.view.as_deref().map(parse_view).transpose()?,
        fit: raw.fit.unwrap_or(false),
        target: raw.target.map(check_target).transpose()?,
        distance: raw.distance.map(|d| ranged("distance", d, DISTANCE)).transpose()?,
        yaw: raw.yaw_rad.map(|y| ranged("yaw_rad", y, (-MAX_YAW, MAX_YAW))).transpose()?,
        pitch: raw.pitch_rad.map(check_pitch).transpose()?,
        fov: raw.fov_rad.map(|f| ranged("fov_rad", f, FOV)).transpose()?,
    })
}

/// `v` when it is a finite number within `range`.
fn ranged(name: &str, v: f64, (lo, hi): (f64, f64)) -> Result<f64, String> {
    if (lo..=hi).contains(&v) {
        return Ok(v);
    }
    Err(format!(
        "ui.camera_set: `{name}` must be a finite number from {} to {}, got {}",
        shown(lo),
        shown(hi),
        shown(v)
    ))
}

/// A number for an error message: plain when moderate, in scientific notation when huge or tiny
/// (a plain 1e300 would be 300 digits).
fn shown(v: f64) -> String {
    if v == 0.0 || (1e-3..1e6).contains(&v.abs()) { v.to_string() } else { format!("{v:e}") }
}

/// The target as a point, when every coordinate is finite and not absurdly far.
fn check_target(target: [f64; 3]) -> Result<Point3, String> {
    for (axis, v) in target.iter().enumerate() {
        ranged(&format!("target[{axis}]"), *v, (-MAX_COORD, MAX_COORD))?;
    }
    let [x, y, z] = target;
    Ok(Point3::new(x, y, z))
}

/// A pitch within the poles; one a rounding error past a pole is the pole.
fn check_pitch(pitch: f64) -> Result<f64, String> {
    let limit = POLE_PITCH + PITCH_SLACK;
    ranged("pitch_rad", pitch, (-limit, limit)).map(|p| p.clamp(-POLE_PITCH, POLE_PITCH))
}

/// The projection, which can only be the perspective one the viewport has.
fn check_projection(projection: Option<&str>) -> Result<(), String> {
    match projection {
        None | Some("perspective") => Ok(()),
        Some("orthographic") => {
            Err("ui.camera_set: `projection` orthographic is not supported, the viewport camera is perspective".into())
        }
        Some(other) => {
            Err(format!("ui.camera_set: `projection` must be \"perspective\", got \"{}\"", clip(other, MAX_NAME_CHARS)))
        }
    }
}

/// A standard view by name (case does not matter).
fn parse_view(name: &str) -> Result<StandardView, String> {
    let wanted = name.trim().to_ascii_lowercase();
    VIEWS.iter().find(|(known, _)| *known == wanted).map(|(_, view)| *view).ok_or_else(|| {
        let known: Vec<&str> = VIEWS.iter().map(|(known, _)| *known).collect();
        format!("ui.camera_set: unknown view \"{}\"; views: {}", clip(name, MAX_NAME_CHARS), known.join(", "))
    })
}

/// The first `max` characters of `text`, with an ellipsis when it was longer.
fn clip(text: &str, max: usize) -> String {
    let mut out: String = text.chars().take(max).collect();
    if text.chars().nth(max).is_some() {
        out.push('…');
    }
    out
}
