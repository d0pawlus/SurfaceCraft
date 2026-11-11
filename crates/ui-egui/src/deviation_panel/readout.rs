//! The deviation readings in the viewport. The reading under the pointer is the deviation of the
//! scan triangle the pointer is on. A click on the mapped scan pins the reading of its triangle;
//! pinned readings stay until the panel removes them. Both come from `deviation.at`, asked of the
//! engine without a log entry (see `Session::query`), and only when the pointer moves to another
//! triangle, so hovering does not flood the command log.
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
    /// The scan has no deviation map (any more), so there is nothing to read.
    NoMap,
}

/// The readings of the Deviation panel in the viewport.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct State {
    /// Pinned readings, oldest first.
    pub pins: Vec<Spot>,
    /// The triangle under the pointer last time, with its reading.
    hover: Option<(Spot, Reading)>,
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

/// A click at viewport pixel `(x, y)` on the mapped scan pins the reading of the triangle under
/// it. The click's own pick (selection) happens as before.
pub fn pin_click(app: &mut SurfaceCraftApp, x: f64, y: f64, w: f64, h: f64) {
    let Some(object) = current(app).map(|c| c.object) else { return };
    if let Some(spot) = spot_at(app, object, x, y, w, h) {
        app.deviation.pins = with_pin(&app.deviation.pins, spot);
    }
}

/// Paints the readings over the viewport: the one under the pointer, then each pinned reading at
/// its point.
pub(crate) fn paint(app: &mut SurfaceCraftApp, ui: &egui::Ui, rect: Rect) {
    let painter = ui.painter_at(rect);
    update_hover(app, ui, rect);
    if let (Some((_, reading)), Some(pointer)) = (app.deviation.hover, ui.input(|i| i.pointer.hover_pos())) {
        label_at(app, &painter, pointer + Vec2::splat(POINTER_GAP), &label(reading));
    }
    paint_pins(app, &painter, rect);
}

/// Sets the reading under the pointer while it is on the mapped scan, and clears it otherwise.
fn update_hover(app: &mut SurfaceCraftApp, ui: &egui::Ui, rect: Rect) {
    let pointer = ui.input(|i| i.pointer.hover_pos()).filter(|p| rect.contains(*p));
    let object = current(app).map(|c| c.object);
    let spot = match (pointer, object) {
        (Some(p), Some(object)) => {
            let local = p - rect.min;
            let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
            spot_at(app, object, f64::from(local.x), f64::from(local.y), w, h)
        }
        _ => None,
    };
    app.deviation.hover = spot.map(|spot| hover_reading(app, spot));
}

/// The reading of `spot`, kept when the pointer is still on the same triangle.
fn hover_reading(app: &mut SurfaceCraftApp, spot: Spot) -> (Spot, Reading) {
    match app.deviation.hover {
        Some((last, reading)) if last.object == spot.object && last.triangle == spot.triangle => (spot, reading),
        _ => (spot, read(app, spot)),
    }
}

/// Marks each pinned reading at its point and labels it.
fn paint_pins(app: &mut SurfaceCraftApp, painter: &egui::Painter, rect: Rect) {
    let (w, h) = (f64::from(rect.width()), f64::from(rect.height()));
    let pins = app.deviation.pins.clone();
    for pin in pins {
        let [x, y, z] = pin.point;
        let Some([sx, sy, _]) = app.camera.project(&Point3::new(x, y, z), w, h) else { continue };
        let at = rect.min + Vec2::new(sx as f32, sy as f32);
        let reading = read(app, pin);
        painter.circle_filled(at, MARKER_RADIUS, app.token_color(Token::Highlight));
        label_at(app, painter, at + Vec2::splat(POINTER_GAP / 2.0), &label(reading));
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
    ui.strong(t("deviation.pinned"));
    if app.deviation.pins.is_empty() {
        ui.weak(t("deviation.pin_hint"));
        return;
    }
    let pins = app.deviation.pins.clone();
    let mut remove = None;
    for (index, pin) in pins.iter().enumerate() {
        let text = label(read(app, *pin));
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
