//! The deviation readings in the viewport. The reading under the pointer is the deviation of the
//! scan triangle the pointer is on. Alt-click on the mapped scan pins the reading of its triangle;
//! pinned readings stay until the panel removes them or their map goes.
//!
//! Readings come from `deviation.at` through `Session::query`, which logs nothing. They are cached:
//! the engine is asked only when the map, the selection, the pointer or the view changed, and not
//! at all on idle frames.
use crate::app::SurfaceCraftApp;
use crate::deviation_panel::current;
use crate::i18n::{t, tf};
use crate::theme::Token;
use egui::{Align2, FontId, Pos2, Rect, Shape, Vec2};
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::EngineError;
use surfacecraft_geom::Point3;

/// Most pinned readings kept: pinning another drops the oldest.
pub const MAX_PINS: usize = 8;
/// Gap (px) between the pointer and the label of its reading.
const POINTER_GAP: f32 = 14.0;
/// Radius (px) of a pinned reading's marker.
const MARKER_RADIUS: f32 = 4.0;
/// Padding (px) around a label's text, and its corner radius.
const LABEL_PAD: f32 = 4.0;
const LABEL_RADIUS: f32 = 3.0;
const LABEL_FONT_SIZE: f32 = 12.0;
/// Decimals of a reading (mm).
const READING_DECIMALS: usize = 3;
/// Values of the view that a frame key holds: orbit target, distance, angles, field of view and the
/// size of the viewport.
const VIEW_KEY_LEN: usize = 9;

/// A triangle of the scan and the point it was read at (model mm).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spot {
    /// The scan.
    pub object: ObjectId,
    /// Its triangle.
    pub triangle: u32,
    /// The point of the pick on the triangle (mm).
    pub point: [f64; 3],
}

/// What the engine says about a triangle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Reading {
    /// The mean signed deviation of its counted corners (mm).
    Measured(f64),
    /// Not measured, or not facing the reference: the grey of the colour map.
    NotMeasured,
    /// The scan has no current deviation map, so there is nothing to read.
    NoMap,
}

/// What the readout drew last frame. When it changes, the reading under the pointer is asked again.
#[derive(Clone, Copy, Debug, PartialEq)]
struct FrameKey {
    epoch: u64,
    selected: Option<ObjectId>,
    pointer: Option<[f32; 2]>,
    view: [u64; VIEW_KEY_LEN],
}

/// The reading under the pointer, with the epoch it was read at.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Hover {
    spot: Spot,
    reading: Reading,
    epoch: u64,
}

/// The readings of the pinned triangles, read at one epoch.
#[derive(Clone, Debug, PartialEq)]
struct PinReadings {
    epoch: u64,
    readings: Vec<(Spot, Reading)>,
}

/// The readings of the Deviation panel in the viewport.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct State {
    /// Pinned readings, oldest first.
    pub pins: Vec<Spot>,
    /// The readings of `pins`, as read at the last epoch.
    pin_readings: Option<PinReadings>,
    /// The inputs of the last frame's hover reading.
    frame: Option<FrameKey>,
    /// The reading under the pointer.
    hover: Option<Hover>,
    /// Engine queries made so far: idle frames make none.
    pub queries: usize,
}

/// `pins` with `spot` added. A triangle is pinned once; past `MAX_PINS` the oldest pin goes.
#[must_use]
pub fn with_pin(pins: &[Spot], spot: Spot) -> Vec<Spot> {
    if pins.iter().any(|p| p.object == spot.object && p.triangle == spot.triangle) {
        return pins.to_vec();
    }
    let mut next = pins.to_vec();
    next.push(spot);
    let excess = next.len().saturating_sub(MAX_PINS);
    next.into_iter().skip(excess).collect()
}

/// `pins` without the pin at `index` (nothing is removed when there is none there).
#[must_use]
pub fn without_pin(pins: &[Spot], index: usize) -> Vec<Spot> {
    pins.iter().enumerate().filter(|(i, _)| *i != index).map(|(_, p)| *p).collect()
}

/// The reading of an answer of `deviation.at`: its deviation, or why there is none.
#[must_use]
pub fn reading_of(answer: Result<Value, EngineError>) -> Reading {
    match answer {
        Ok(out) => out.get("deviation").and_then(Value::as_f64).map_or(Reading::NotMeasured, Reading::Measured),
        Err(_) => Reading::NoMap,
    }
}

/// The reading at `spot`, asked of the engine without a log entry.
pub fn read(app: &mut SurfaceCraftApp, spot: Spot) -> Reading {
    app.deviation.queries += 1;
    reading_of(app.session.query("deviation.at", json!({"object": spot.object.0, "triangle": spot.triangle})))
}

/// The text of a reading: its signed value in mm, or why there is none.
#[must_use]
pub fn label(reading: Reading) -> String {
    match reading {
        Reading::Measured(d) => tf("deviation.readout", &[("value", &format!("{d:+.prec$}", prec = READING_DECIMALS))]),
        Reading::NotMeasured => t("deviation.readout_none").to_string(),
        Reading::NoMap => t("deviation.none").to_string(),
    }
}

/// The triangle of `object` under viewport pixel `(x, y)` of a `w × h` viewport, if the pick hits
/// that object.
pub fn spot_at(app: &mut SurfaceCraftApp, object: ObjectId, x: f64, y: f64, w: f64, h: f64) -> Option<Spot> {
    let pick = app.ray_hit(x, y, w, h)?;
    (pick.object == object).then(|| Spot {
        object,
        triangle: pick.triangle,
        point: [pick.point.x, pick.point.y, pick.point.z],
    })
}

/// A click at viewport pixel `(x, y)` pins the reading of the mapped scan's triangle under it, but
/// only with Alt held: a plain click only selects, as before.
pub fn pin_click(app: &mut SurfaceCraftApp, x: f64, y: f64, w: f64, h: f64, alt: bool) {
    if !alt {
        return;
    }
    let Some(object) = current(app).map(|c| c.object) else { return };
    if let Some(spot) = spot_at(app, object, x, y, w, h) {
        app.deviation.pins = with_pin(&app.deviation.pins, spot);
    }
}

/// Keeps the pinned readings current. A pin whose map is gone is dropped. The readings are read
/// again only when the epoch moved or the pins changed; a pin read at this epoch keeps its reading.
pub(crate) fn sync_pins(app: &mut SurfaceCraftApp) {
    app.deviation.pins.retain(|pin| app.session.live_map(pin.object).is_some());
    let epoch = app.session.epoch();
    let previous = match app.deviation.pin_readings.take() {
        Some(cached) if cached.epoch == epoch => cached.readings,
        _ => Vec::new(),
    };
    let unchanged = previous.len() == app.deviation.pins.len()
        && previous.iter().map(|(spot, _)| *spot).eq(app.deviation.pins.iter().copied());
    if unchanged {
        app.deviation.pin_readings = Some(PinReadings { epoch, readings: previous });
        return;
    }
    let pins = app.deviation.pins.clone();
    let mut readings = Vec::with_capacity(pins.len());
    for pin in pins {
        let reading = match previous.iter().find(|(spot, _)| *spot == pin) {
            Some((_, reading)) => *reading,
            None => read(app, pin),
        };
        readings.push((pin, reading));
    }
    app.deviation.pin_readings = Some(PinReadings { epoch, readings });
}

/// Paints the readings over the viewport: the reading under the pointer, then each pinned reading at
/// its point. The engine is asked only when the inputs of the readout changed.
pub(crate) fn paint(app: &mut SurfaceCraftApp, ui: &egui::Ui, rect: Rect) {
    let painter = ui.painter_at(rect);
    let pointer = ui.input(|i| i.pointer.hover_pos()).filter(|p| rect.contains(*p));
    update_hover(app, pointer, rect);
    sync_pins(app);
    if let (Some(hover), Some(pointer)) = (app.deviation.hover, pointer) {
        label_at(app, &painter, pointer + Vec2::splat(POINTER_GAP), &label(hover.reading));
    }
    paint_pins(app, &painter, rect);
}

/// Sets the reading under the pointer when the map, the selection, the pointer or the view changed;
/// otherwise the last reading stands.
fn update_hover(app: &mut SurfaceCraftApp, pointer: Option<Pos2>, rect: Rect) {
    let key = frame_key(app, pointer, rect);
    if app.deviation.frame == Some(key) {
        return;
    }
    app.deviation.frame = Some(key);
    let object = current(app).map(|c| c.object);
    let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
    let spot = match (pointer, object) {
        (Some(p), Some(object)) => {
            let local = p - rect.min;
            spot_at(app, object, f64::from(local.x), f64::from(local.y), w, h)
        }
        _ => None,
    };
    let hover = spot.map(|spot| hover_for(app, spot));
    app.deviation.hover = hover;
}

/// The hover reading of `spot`: the last one is kept while the pointer stays on the same triangle
/// of the same map, otherwise the engine is asked.
fn hover_for(app: &mut SurfaceCraftApp, spot: Spot) -> Hover {
    let epoch = app.session.epoch();
    let kept = app
        .deviation
        .hover
        .filter(|h| h.epoch == epoch && h.spot.object == spot.object && h.spot.triangle == spot.triangle);
    match kept {
        Some(h) => Hover { spot, reading: h.reading, epoch },
        None => Hover { spot, reading: read(app, spot), epoch },
    }
}

fn frame_key(app: &SurfaceCraftApp, pointer: Option<Pos2>, rect: Rect) -> FrameKey {
    let c = &app.camera;
    let view = [
        c.target.x,
        c.target.y,
        c.target.z,
        c.distance,
        c.yaw,
        c.pitch,
        c.fov,
        f64::from(rect.width()),
        f64::from(rect.height()),
    ]
    .map(f64::to_bits);
    FrameKey { epoch: app.session.epoch(), selected: app.selected_object, pointer: pointer.map(|p| [p.x, p.y]), view }
}

/// Marks each pinned reading at its point and labels it.
fn paint_pins(app: &SurfaceCraftApp, painter: &egui::Painter, rect: Rect) {
    let Some(cached) = &app.deviation.pin_readings else { return };
    let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
    for (pin, reading) in &cached.readings {
        let [x, y, z] = pin.point;
        let Some([sx, sy, _]) = app.camera.project(&Point3::new(x, y, z), w, h) else { continue };
        let at = rect.min + Vec2::new(sx as f32, sy as f32);
        painter.circle_filled(at, MARKER_RADIUS, app.token_color(Token::Highlight));
        label_at(app, painter, at + Vec2::splat(POINTER_GAP / 2.0), &label(*reading));
    }
}

/// A label at `anchor` on a raised background, so that it reads over the mesh.
fn label_at(app: &SurfaceCraftApp, painter: &egui::Painter, anchor: Pos2, text: &str) {
    let background = painter.add(Shape::Noop);
    let font = FontId::proportional(LABEL_FONT_SIZE);
    let text_rect = painter.text(anchor, Align2::LEFT_TOP, text, font, app.token_color(Token::TextPrimary));
    let fill = app.token_color(Token::SurfaceRaised);
    painter.set(background, Shape::rect_filled(text_rect.expand(LABEL_PAD), LABEL_RADIUS, fill));
}

/// The pinned readings of the panel: each with its triangle, its reading and a Remove button.
pub(crate) fn pins_section(app: &mut SurfaceCraftApp, ui: &mut egui::Ui) {
    sync_pins(app);
    ui.strong(t("deviation.pinned"));
    let rows = app.deviation.pin_readings.as_ref().map(|cached| cached.readings.clone()).unwrap_or_default();
    if rows.is_empty() {
        ui.weak(t("deviation.pin_hint"));
        return;
    }
    let mut remove = None;
    for (index, (pin, reading)) in rows.iter().enumerate() {
        let text = label(*reading);
        let triangle = tf("deviation.triangle", &[("n", &pin.triangle.to_string())]);
        ui.horizontal(|ui| {
            ui.monospace(triangle);
            ui.label(text);
            if ui.small_button(t("deviation.unpin")).clicked() {
                remove = Some(index);
            }
        });
    }
    if let Some(index) = remove {
        app.deviation.pins = without_pin(&app.deviation.pins, index);
    }
}
