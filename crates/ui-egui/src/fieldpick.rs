//! Picking the value of a form field with the mouse in the viewport.
//!
//! A form field that takes geometry (the 3-2-1 datums of `align.datum`, lists of points, the
//! `edges` of a fillet or chamfer) gets a "Pick in viewport" button. While a field is being
//! picked a click in the viewport ray-casts onto the scan (points) or looks for the nearest body
//! edge within a pixel tolerance (edges), writes the result into the command's JSON parameters
//! (the typed field stays editable and shows the same value) and highlights what was picked.
//! Datums go in order: three points make the primary plane, two the secondary axis, one the
//! tertiary point, and the pick moves on by itself. Esc or a right click ends picking.
//!
//! Everything works on the shared parameter text (`SurfaceCraftApp::palette_params`), so the
//! forms, the JSON mode, the live preview and the tests all see the same state.
use crate::app::SurfaceCraftApp;
use crate::brep::{BrepHit, BrepSelection, BrepTarget};
use crate::i18n::{t, tf};
use serde_json::{Map, Value, json};
use std::collections::BTreeSet;
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::Vec3;

/// The datum parameters of `align.datum`, in order, with the number of points each one takes: a
/// plane from three points, an axis from two, a point from one.
pub const DATUM_FIELDS: [(&str, usize); 3] = [("primary", 3), ("secondary", 2), ("tertiary", 1)];

/// The parameters that name the body whose edges are picked (`body.fillet` calls it `object`,
/// `feature.fillet` calls it `body`).
const BODY_KEYS: [&str; 2] = ["object", "body"];

/// A point closer than this to collinear gives no plane.
const DEGENERATE: f64 = 1e-9;

/// What a field takes from the viewport.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PickKind {
    /// A datum made of `need` clicked points: 3 give a plane, 2 an axis, 1 a point.
    Datum {
        /// How many points the datum takes.
        need: usize,
    },
    /// A list of points; every click appends one.
    Points,
    /// A list of edge indices of a body; every click toggles one.
    Edges,
}

/// The number of points the datum parameter called `name` takes (`None` for other parameters).
#[must_use]
pub fn datum_need(name: &str) -> Option<usize> {
    DATUM_FIELDS.iter().find(|(n, _)| *n == name).map(|(_, k)| *k)
}

/// What the datum parameter `name` starts as: the world plane, axis or point of its role (a 3-2-1
/// alignment with these moves nothing).
#[must_use]
pub fn datum_default(name: &str) -> Option<Value> {
    match name {
        "primary" => Some(json!({"plane": {"point": [0.0, 0.0, 0.0], "normal": [0.0, 0.0, 1.0]}})), // not a colour
        "secondary" => Some(json!({"axis": {"point": [0.0, 0.0, 0.0], "direction": [1.0, 0.0, 0.0]}})), // not a colour
        "tertiary" => Some(json!({"point": {"point": [0.0, 0.0, 0.0]}})),                           // not a colour
        _ => None,
    }
}

/// A form's wish to start or stop picking a field (the form never acts on it itself).
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PickRequest {
    /// The parameter to fill.
    pub field: String,
    /// What it takes.
    pub kind: PickKind,
    /// Start picking (true) or stop (false).
    pub start: bool,
}

/// The state of a field being picked.
#[derive(Clone, Debug)]
pub struct FieldPick {
    /// The command the form belongs to (the pick ends when another one is chosen).
    pub command: &'static str,
    /// The fields still to pick, the current one first.
    queue: Vec<(String, PickKind)>,
    /// Points clicked for the current datum.
    pub buffer: Vec<[f64; 3]>,
    /// Every point clicked in this pick (drawn as numbered markers).
    pub marks: Vec<[f64; 3]>,
    /// The point on the scan under the pointer.
    pub hover: Option<[f64; 3]>,
}

impl FieldPick {
    /// The parameter being picked.
    #[must_use]
    pub fn field(&self) -> Option<&str> {
        self.queue.first().map(|(f, _)| f.as_str())
    }

    /// What the parameter being picked takes.
    #[must_use]
    pub fn kind(&self) -> Option<PickKind> {
        self.queue.first().map(|(_, k)| *k)
    }
}

/// The datum `points` define: a plane through three points (normal by the right-hand rule), an
/// axis through two, a point. `None` when they do not define one (collinear or coinciding).
#[must_use]
pub fn datum_value(points: &[[f64; 3]]) -> Option<Value> {
    let v = |a: [f64; 3], b: [f64; 3]| Vec3::new(b[0] - a[0], b[1] - a[1], b[2] - a[2]);
    match points {
        [p] => Some(json!({"point": {"point": p}})),
        [a, b] => {
            let d = v(*a, *b);
            (d.norm() > DEGENERATE).then(|| json!({"axis": {"point": a, "direction": [d.x, d.y, d.z]}}))
        }
        [a, b, c] => {
            let n = v(*a, *b).cross(&v(*a, *c));
            let scale = v(*a, *b).norm() * v(*a, *c).norm();
            (scale > DEGENERATE && n.norm() > DEGENERATE * scale)
                .then(|| json!({"plane": {"point": a, "normal": [n.x, n.y, n.z]}}))
        }
        _ => None,
    }
}

impl SurfaceCraftApp {
    fn params_object(&self) -> Option<Map<String, Value>> {
        match serde_json::from_str::<Value>(&self.palette_params) {
            Ok(Value::Object(m)) => Some(m),
            _ => None,
        }
    }

    fn store_params(&mut self, map: Map<String, Value>) {
        self.palette_params = serde_json::to_string_pretty(&Value::Object(map)).unwrap_or_else(|_| "{}".into());
    }

    /// The object an `ObjRef` value (id or name) names.
    fn object_of(&self, v: &Value) -> Option<ObjectId> {
        match v {
            Value::Number(n) => n.as_u64().map(ObjectId),
            Value::String(name) => self.session.doc.summaries().into_iter().find(|s| &s.name == name).map(|s| s.id),
            _ => None,
        }
    }

    /// The edge indices in the field `field` of the parameters.
    fn edges_of(map: &Map<String, Value>, field: &str) -> Vec<u64> {
        map.get(field)
            .and_then(Value::as_array)
            .map(|a| a.iter().filter_map(Value::as_u64).collect())
            .unwrap_or_default()
    }

    /// Starts picking `field` (for a datum, the datums after it follow). The command of the
    /// palette is the one the pick belongs to.
    pub fn start_field_pick(&mut self, field: &str, kind: PickKind) {
        let Some(command) = self.palette_command else { return };
        let queue: Vec<(String, PickKind)> = match kind {
            PickKind::Datum { .. } => DATUM_FIELDS
                .iter()
                .skip_while(|(n, _)| *n != field)
                .map(|(n, k)| ((*n).to_string(), PickKind::Datum { need: *k }))
                .collect(),
            other => vec![(field.to_string(), other)],
        };
        if queue.is_empty() {
            return;
        }
        self.field_pick = Some(FieldPick { command, queue, buffer: Vec::new(), marks: Vec::new(), hover: None });
        self.tools.hover = None;
        self.tools.touch();
        if let Some((_, hint)) = self.field_pick_hint() {
            self.status = hint;
        }
    }

    /// Ends picking (what was picked stays in the parameters).
    pub fn stop_field_pick(&mut self) {
        if self.field_pick.take().is_some() {
            self.tools.hover = None;
            self.tools.touch();
            self.status = t("fieldpick.done").to_string();
        }
    }

    /// Acts on a form's request.
    pub fn handle_pick_request(&mut self, req: PickRequest) {
        if req.start {
            self.start_field_pick(&req.field, req.kind);
        } else {
            self.stop_field_pick();
        }
    }

    /// The field being picked and what to tell the user about it, in the current language.
    #[must_use]
    pub fn field_pick_hint(&self) -> Option<(String, String)> {
        let fp = self.field_pick.as_ref()?;
        let field = fp.field()?.to_string();
        let hint = match fp.kind()? {
            PickKind::Datum { need } => tf(
                "fieldpick.datum",
                &[("field", &field), ("n", &(fp.buffer.len() + 1).to_string()), ("need", &need.to_string())],
            ),
            PickKind::Points => {
                let n = self
                    .params_object()
                    .and_then(|m| m.get(&field).and_then(Value::as_array).map(Vec::len))
                    .unwrap_or_default();
                tf("fieldpick.points", &[("field", &field), ("n", &n.to_string())])
            }
            PickKind::Edges => {
                let n = self.params_object().map_or(0, |m| Self::edges_of(&m, &field).len());
                tf("fieldpick.edges", &[("field", &field), ("n", &n.to_string())])
            }
        };
        Some((field, hint))
    }

    /// A click at viewport pixel `(x, y)` of a `w × h` viewport while a field is being picked.
    /// Returns whether it picked something.
    pub fn field_pick_click(&mut self, x: f64, y: f64, w: f64, h: f64) -> bool {
        let Some(kind) = self.field_pick.as_ref().and_then(FieldPick::kind) else { return false };
        let result = match kind {
            PickKind::Edges => self.pick_edge_into_field(x, y, w, h),
            PickKind::Datum { .. } | PickKind::Points => self.pick_point_into_field(kind, x, y, w, h),
        };
        match result {
            Ok(()) => {
                if let Some((_, hint)) = self.field_pick_hint() {
                    self.status = hint;
                }
                true
            }
            Err(key) => {
                self.status = t(key).to_string();
                false
            }
        }
    }

    /// The pointer moved to viewport pixel `p` while a field is being picked: what a click would
    /// pick is highlighted.
    pub(crate) fn field_pick_hover(&mut self, p: [f64; 2], w: f64, h: f64) {
        match self.field_pick.as_ref().and_then(FieldPick::kind) {
            Some(PickKind::Edges) => self.tools.hover = self.brep_hit_at_target(p[0], p[1], w, h, BrepTarget::Edges),
            Some(_) => {
                let hit = self.ray_hit(p[0], p[1], w, h).map(|p| [p.point.x, p.point.y, p.point.z]);
                if let Some(fp) = self.field_pick.as_mut() {
                    fp.hover = hit;
                }
            }
            None => {}
        }
    }

    /// The pointer left the viewport.
    pub(crate) fn field_pick_unhover(&mut self) {
        self.tools.hover = None;
        if let Some(fp) = self.field_pick.as_mut() {
            fp.hover = None;
        }
    }

    fn pick_point_into_field(&mut self, kind: PickKind, x: f64, y: f64, w: f64, h: f64) -> Result<(), &'static str> {
        let Some(hit) = self.ray_hit(x, y, w, h) else { return Err("fieldpick.miss") };
        let p = [hit.point.x, hit.point.y, hit.point.z];
        let Some(mut fp) = self.field_pick.take() else { return Err("fieldpick.miss") };
        let Some(mut map) = self.params_object() else {
            self.field_pick = Some(fp);
            return Err("form.err.json");
        };
        self.selected_object = Some(hit.object);
        fp.marks.push(p);
        let field = fp.field().unwrap_or_default().to_string();
        let mut finished = false;
        match kind {
            PickKind::Datum { need } => {
                fp.buffer.push(p);
                if fp.buffer.len() >= need {
                    let points = std::mem::take(&mut fp.buffer);
                    match datum_value(&points) {
                        Some(datum) => {
                            map.insert(field, datum);
                            self.store_params(map);
                            if !fp.queue.is_empty() {
                                fp.queue.remove(0);
                            }
                            finished = fp.queue.is_empty();
                        }
                        None => {
                            // collinear or coinciding points: start this datum again
                            let keep = fp.marks.len().saturating_sub(points.len());
                            fp.marks.truncate(keep);
                            self.field_pick = Some(fp);
                            return Err(if need == 3 {
                                "fieldpick.degenerate_plane"
                            } else {
                                "fieldpick.degenerate_axis"
                            });
                        }
                    }
                }
            }
            PickKind::Edges => {}
            PickKind::Points => {
                let mut list = match map.remove(&field) {
                    Some(Value::Array(a)) => a,
                    _ => Vec::new(),
                };
                list.push(json!(p));
                map.insert(field, Value::Array(list));
                self.store_params(map);
            }
        }
        if finished {
            self.tools.hover = None;
            self.tools.touch();
            self.status = t("fieldpick.done").to_string();
        } else {
            self.field_pick = Some(fp);
        }
        Ok(())
    }

    fn pick_edge_into_field(&mut self, x: f64, y: f64, w: f64, h: f64) -> Result<(), &'static str> {
        let Some((body, BrepHit::Edge(edge))) = self.brep_hit_at_target(x, y, w, h, BrepTarget::Edges) else {
            return Err("fieldpick.miss");
        };
        let Some(field) = self.field_pick.as_ref().and_then(|f| f.field()).map(str::to_string) else {
            return Err("fieldpick.miss");
        };
        let Some(mut map) = self.params_object() else { return Err("form.err.json") };
        let key = BODY_KEYS.iter().copied().find(|k| map.contains_key(*k));
        // edges of another body start a new list: edge indices only mean something per body
        let same_body = match key.and_then(|k| map.get(k)) {
            Some(v) => self.object_of(v) == Some(body),
            None => true,
        };
        let mut edges = if same_body { Self::edges_of(&map, &field) } else { Vec::new() };
        let e = edge as u64;
        match edges.iter().position(|x| *x == e) {
            Some(i) => {
                edges.remove(i);
            }
            None => edges.push(e),
        }
        if let Some(k) = key {
            map.insert(k.to_string(), json!(body.0));
        }
        let set: BTreeSet<usize> = edges.iter().map(|e| *e as usize).collect();
        map.insert(field, json!(edges));
        self.store_params(map);
        self.tools.brep = BrepSelection { body: Some(body), faces: BTreeSet::new(), edges: set };
        self.selected_object = Some(body);
        self.tools.touch();
        self.refresh_highlights();
        Ok(())
    }

    /// The body and the edges to draw as selected: while an edge field is picked, those of the
    /// field (so a typed index is highlighted too), else the viewport's own selection.
    #[must_use]
    pub(crate) fn highlighted_edges(&self) -> (Option<ObjectId>, BTreeSet<usize>) {
        if let Some(fp) = &self.field_pick
            && fp.kind() == Some(PickKind::Edges)
            && let Some(field) = fp.field()
            && let Some(map) = self.params_object()
        {
            let body = BODY_KEYS.iter().find_map(|k| map.get(*k)).and_then(|v| self.object_of(v));
            return (body, Self::edges_of(&map, field).into_iter().map(|e| e as usize).collect());
        }
        (self.tools.brep.body, self.tools.brep.edges.clone())
    }

    /// Points to mark in the viewport while a field is being picked: the ones clicked so far
    /// (numbered from 1), then the point under the pointer.
    #[must_use]
    pub(crate) fn field_pick_marks(&self) -> (Vec<[f64; 3]>, Option<[f64; 3]>) {
        match &self.field_pick {
            Some(fp) => (fp.marks.clone(), fp.hover),
            None => (Vec::new(), None),
        }
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)]
mod tests {
    use super::*;

    #[test]
    fn datums_come_from_one_two_or_three_points() {
        let o = [0.0, 0.0, 0.0];
        let (x, y) = ([10.0, 0.0, 0.0], [0.0, 10.0, 0.0]);
        assert_eq!(datum_value(&[o, x, y]), Some(json!({"plane": {"point": o, "normal": [0.0, 0.0, 100.0]}})));
        assert_eq!(datum_value(&[o, x]), Some(json!({"axis": {"point": o, "direction": [10.0, 0.0, 0.0]}})));
        assert_eq!(datum_value(&[y]), Some(json!({"point": {"point": y}})));
        assert_eq!(datum_value(&[o, x, [20.0, 0.0, 0.0]]), None, "collinear points give no plane");
        assert_eq!(datum_value(&[o, o]), None, "coinciding points give no axis");
        assert_eq!(datum_value(&[]), None);
    }

    #[test]
    fn the_datum_parameters_are_in_3_2_1_order() {
        assert_eq!(datum_need("primary"), Some(3));
        assert_eq!(datum_need("secondary"), Some(2));
        assert_eq!(datum_need("tertiary"), Some(1));
        assert_eq!(datum_need("object"), None);
    }
}
