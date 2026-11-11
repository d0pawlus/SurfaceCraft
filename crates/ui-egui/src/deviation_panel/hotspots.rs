//! The hotspots of the current map: the groups of scan triangles above its tolerance, largest
//! first, from `deviation.hotspots`. The panel runs the command on request; clicking a group
//! centres the view on it.
use crate::app::SurfaceCraftApp;
use crate::deviation_panel::{Current, num};
use crate::i18n::{t, tf};
use serde_json::{Map, Value, json};
use surfacecraft_deviation::{Hotspot, Sign};
use surfacecraft_engine::cmd::deviation::HotspotsOut;
use surfacecraft_geom::Point3;

/// Most groups the command returns and the panel lists (largest first).
pub const LISTED_GROUPS: usize = 20;

/// Parameters of `deviation.hotspots` for the map `cur`: its scan, reference, region and facing,
/// at its tolerance. Only the fields the command accepts are copied (the command refuses others).
#[must_use]
pub fn params(cur: &Current) -> Value {
    let mut out: Map<String, Value> = ["against", "region", "max_distance"]
        .into_iter()
        .filter_map(|key| cur.params.get(key).map(|v| (key.to_string(), v.clone())))
        .collect();
    let facing = cur.params.get("facing").cloned().unwrap_or(Value::Bool(true));
    out.insert("object".into(), json!(cur.object.0));
    out.insert("tolerance".into(), json!(cur.stats.tolerance));
    out.insert("limit".into(), json!(LISTED_GROUPS));
    out.insert("facing".into(), facing);
    Value::Object(out)
}

/// The latest successful `deviation.hotspots` run of the log with the parameters of map `cur`.
#[must_use]
pub fn latest(app: &SurfaceCraftApp, cur: &Current) -> Option<HotspotsOut> {
    let want = params(cur);
    app.session.log.iter().rev().filter(|e| e.ok && e.command == "deviation.hotspots").find_map(|e| {
        if e.params != want {
            return None;
        }
        serde_json::from_value::<HotspotsOut>(e.outcome.clone()).ok()
    })
}

/// The Hotspots section: the Find button runs the command, and its groups are listed below.
pub(crate) fn section(app: &mut SurfaceCraftApp, ui: &mut egui::Ui, cur: &Current) {
    ui.strong(t("deviation.hotspots"));
    if ui.button(t("deviation.find_hotspots")).clicked() {
        app.launch("deviation.hotspots", params(cur), crate::jobs::After::Nothing);
    }
    if let Some(found) = latest(app, cur) {
        listing(app, ui, &found);
    }
}

/// One button per group, largest first; a click focuses the view on it.
fn listing(app: &mut SurfaceCraftApp, ui: &mut egui::Ui, found: &HotspotsOut) {
    if found.hotspots.is_empty() {
        ui.weak(t("deviation.hotspots_none"));
        return;
    }
    let shown = found.hotspots.len().to_string();
    let total = found.total.to_string();
    ui.weak(tf("deviation.hotspots_shown", &[("shown", &shown), ("total", &total)]));
    for hotspot in &found.hotspots {
        if ui.button(row_text(hotspot)).on_hover_text(t("deviation.focus_hint")).clicked() {
            focus(app, hotspot);
        }
    }
}

/// The text of one group's row: its side, its footprint, its largest deviation and its size.
#[must_use]
pub fn row_text(h: &Hotspot) -> String {
    let side = match h.sign {
        Sign::Positive => t("deviation.excess"),
        Sign::Negative => t("deviation.missing"),
    };
    tf(
        "deviation.hotspot_row",
        &[("sign", side), ("area", &num(h.area, 2)), ("max", &num(h.max, 3)), ("n", &h.triangles.to_string())],
    )
}

/// The centroid of a group as a model point.
#[must_use]
pub fn centroid(h: &Hotspot) -> Point3 {
    let [x, y, z] = h.centroid;
    Point3::new(x, y, z)
}

/// Centres the view on group `h`: the orbit target moves to its centroid and the zoom stays.
pub fn focus(app: &mut SurfaceCraftApp, h: &Hotspot) {
    app.camera.target = centroid(h);
}
