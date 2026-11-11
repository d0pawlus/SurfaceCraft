//! Picking the value of a form field with the mouse in the viewport.
//!
//! A form field that takes geometry gets a "Pick in viewport" button. Which fields do is said by
//! the `x-pick` marker in the command's JSON Schema (the 3-2-1 datums of `align.datum`, lists of
//! points, the `edges` of a fillet or chamfer), never by the field's name. While a field is being
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

/// The schema extension that marks a parameter the viewport can fill, with what is picked for it
/// (`"plane"`, `"axis"`, `"point"`, `"points"` or `"edges"`).
pub const PICK_MARKER: &str = "x-pick";

/// The schema extension that marks the parameter naming the body whose edges are picked.
pub const PICK_BODY_MARKER: &str = "x-pick-body";

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

impl PickKind {
    /// The points a datum takes (`None` for lists).
    #[must_use]
    pub fn need(self) -> Option<usize> {
        match self {
            Self::Datum { need } => Some(need),
            Self::Points | Self::Edges => None,
        }
    }
}

/// What a marked parameter takes, as its schema marker names it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PickMode {
    /// A plane datum from three points (the primary datum of a 3-2-1 alignment).
    Plane,
    /// An axis datum from two points (the secondary datum).
    Axis,
    /// A point datum from one point (the tertiary datum).
    Point,
    /// A list of points.
    Points,
    /// A list of edge indices of a body.
    Edges,
}

impl PickMode {
    /// The mode a marker names (`None` for an unknown marker).
    #[must_use]
    pub fn from_marker(marker: &str) -> Option<Self> {
        match marker {
            "plane" => Some(Self::Plane),
            "axis" => Some(Self::Axis),
            "point" => Some(Self::Point),
            "points" => Some(Self::Points),
            "edges" => Some(Self::Edges),
            _ => None,
        }
    }

    /// The mode of a property schema (`None` when the property carries no marker).
    #[must_use]
    pub fn of(prop: &Value) -> Option<Self> {
        Self::from_marker(prop.get(PICK_MARKER)?.as_str()?)
    }

    /// What a field of this mode takes from the viewport.
    #[must_use]
    pub fn kind(self) -> PickKind {
        match self {
            Self::Plane => PickKind::Datum { need: 3 },
            Self::Axis => PickKind::Datum { need: 2 },
            Self::Point => PickKind::Datum { need: 1 },
            Self::Points => PickKind::Points,
            Self::Edges => PickKind::Edges,
        }
    }

    /// The position of a datum in the 3-2-1 order (plane, axis, point); `None` for lists.
    #[must_use]
    pub fn datum_rank(self) -> Option<u8> {
        match self {
            Self::Plane => Some(0),
            Self::Axis => Some(1),
            Self::Point => Some(2),
            Self::Points | Self::Edges => None,
        }
    }

    /// What a datum parameter of this mode starts as: the world plane, axis or point (a 3-2-1
    /// alignment with these moves nothing). `None` for lists.
    #[must_use]
    pub fn default_value(self) -> Option<Value> {
        match self {
            Self::Plane => Some(json!({"plane": {"point": [0.0, 0.0, 0.0], "normal": [0.0, 0.0, 1.0]}})), // not a colour
            Self::Axis => Some(json!({"axis": {"point": [0.0, 0.0, 0.0], "direction": [1.0, 0.0, 0.0]}})), // not a colour
            Self::Point => Some(json!({"point": {"point": [0.0, 0.0, 0.0]}})), // not a colour
            Self::Points | Self::Edges => None,
        }
    }
}

/// The parameters of a command's input schema that carry a pick marker, in property order.
#[must_use]
pub fn marked_fields(input: &Value) -> Vec<(String, PickMode)> {
    let Some(props) = input.get("properties").and_then(Value::as_object) else { return Vec::new() };
    props.iter().filter_map(|(name, prop)| Some((name.clone(), PickMode::of(prop)?))).collect()
}

/// The parameter of a command's input schema that names the body whose edges are picked.
#[must_use]
pub fn body_field(input: &Value) -> Option<String> {
    let props = input.get("properties").and_then(Value::as_object)?;
    props.iter().find(|(_, p)| p.get(PICK_BODY_MARKER).and_then(Value::as_bool) == Some(true)).map(|(n, _)| n.clone())
}

/// The JSON Schema of the parameters of the command `id` (`None` for an unknown command).
#[must_use]
pub fn input_schema(id: &str) -> Option<Value> {
    surfacecraft_engine::cmd::find(id).map(|spec| (spec.input)())
}

/// The parameters a pick fills, in order: the field itself, and for a 3-2-1 datum the datums after
/// it in the schema.
#[must_use]
pub fn pick_queue(input: &Value, field: &str, kind: PickKind) -> Vec<(String, PickKind)> {
    let marked = marked_fields(input);
    if !matches!(kind, PickKind::Datum { .. }) {
        return vec![(field.to_string(), kind)];
    }
    let Some(from) = marked.iter().find(|(n, _)| n == field).and_then(|(_, m)| m.datum_rank()) else {
        return Vec::new();
    };
    let mut datums: Vec<(u8, String, PickMode)> = marked
        .into_iter()
        .filter_map(|(n, m)| Some((m.datum_rank()?, n, m)))
        .filter(|(rank, _, _)| *rank >= from)
        .collect();
    datums.sort_by_key(|(rank, _, _)| *rank);
    datums.into_iter().map(|(_, n, m)| (n, m.kind())).collect()
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
    /// The parameter naming the body whose edges are picked (`x-pick-body`), if the command has one.
    body: Option<String>,
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

/// One coordinate of a datum's summary: two decimals at most, no trailing zeros.
fn summary_number(x: f64) -> String {
    let s = format!("{x:.2}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() || s == "-0" || s == "-" { "0".to_string() } else { s.to_string() }
}

fn summary_vector(v: [f64; 3]) -> String {
    format!("({}, {}, {})", summary_number(v[0]), summary_number(v[1]), summary_number(v[2]))
}

/// The three numbers of a JSON array (`None` for anything else).
fn vector_of(v: Option<&Value>) -> Option<[f64; 3]> {
    let a = v?.as_array()?;
    match a.as_slice() {
        [x, y, z] => Some([x.as_f64()?, y.as_f64()?, z.as_f64()?]),
        _ => None,
    }
}

/// A direction as a unit vector, so a normal of any length reads the same.
fn unit(v: [f64; 3]) -> [f64; 3] {
    let n = Vec3::new(v[0], v[1], v[2]).norm();
    if n > DEGENERATE { [v[0] / n, v[1] / n, v[2] / n] } else { v }
}

/// A datum in plain words ("Plane through (0, 0, 0), facing (0, 0, 1)"), what the form shows in
/// place of the datum's JSON. A value that is not a datum the form knows says so.
#[must_use]
pub fn datum_summary(value: &Value) -> String {
    let part = |kind: &str, key: &str| vector_of(value.get(kind).and_then(|d| d.get(key)));
    if let (Some(point), Some(normal)) = (part("plane", "point"), part("plane", "normal")) {
        let (point, normal) = (summary_vector(point), summary_vector(unit(normal)));
        return tf("form.datum.plane", &[("point", &point), ("normal", &normal)]);
    }
    if let (Some(point), Some(direction)) = (part("axis", "point"), part("axis", "direction")) {
        let (point, direction) = (summary_vector(point), summary_vector(unit(direction)));
        return tf("form.datum.axis", &[("point", &point), ("direction", &direction)]);
    }
    if let Some(point) = part("point", "point") {
        return tf("form.datum.point", &[("point", &summary_vector(point))]);
    }
    if let Some(triangles) = value.get("plane_from_triangles").and_then(Value::as_array) {
        return tf("form.datum.triangles", &[("n", &triangles.len().to_string())]);
    }
    t("form.datum.custom").to_string()
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
        let Some(input) = input_schema(command) else { return };
        let queue = pick_queue(&input, field, kind);
        if queue.is_empty() {
            return;
        }
        let body = body_field(&input);
        self.field_pick = Some(FieldPick { command, queue, body, buffer: Vec::new(), marks: Vec::new(), hover: None });
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
        let key = self.field_pick.as_ref().and_then(|f| f.body.clone());
        // edges of another body start a new list: edge indices only mean something per body
        let same_body = match key.as_ref().and_then(|k| map.get(k)) {
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
            let body = fp.body.as_ref().and_then(|k| map.get(k)).and_then(|v| self.object_of(v));
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
    fn a_datum_is_summarised_in_plain_words() {
        crate::i18n::set_lang(crate::i18n::Lang::En);
        let plane = json!({"plane": {"point": [0.0, 0.0, 0.0], "normal": [0.0, 0.0, 100.0]}}); // not a colour
        assert_eq!(datum_summary(&plane), "Plane through (0, 0, 0), facing (0, 0, 1)");
        let axis = json!({"axis": {"point": [1.5, 2.0, -0.004], "direction": [10.0, 0.0, 0.0]}});
        assert_eq!(datum_summary(&axis), "Axis through (1.5, 2, 0), along (1, 0, 0)");
        assert_eq!(datum_summary(&json!({"point": {"point": [1.0, 2.25, 3.0]}})), "Point (1, 2.25, 3)");
        assert_eq!(datum_summary(&json!({"plane_from_triangles": [1, 2, 3]})), "Plane fitted to 3 triangles");
        // hostile or unknown values never panic and say what they are
        for odd in [json!(null), json!(7), json!({"plane": {"point": [1, 2]}}), json!({"axis": "x"})] {
            assert_eq!(datum_summary(&odd), "Custom value", "{odd}");
        }
    }

    /// The schema extensions the engine puts on the fields the viewport fills, as (command,
    /// parameter, extension). These are the fields that had a "Pick in viewport" button before the
    /// markers: the 3-2-1 datums, the lists of points, the edge lists, and the bodies owning them.
    const MARKED: &[(&str, &str, &str)] = &[
        ("align.n_point", "from", "x-pick"),
        ("align.n_point", "to", "x-pick"),
        ("align.datum", "primary", "x-pick"),
        ("align.datum", "secondary", "x-pick"),
        ("align.datum", "tertiary", "x-pick"),
        ("body.fillet", "edges", "x-pick"),
        ("body.fillet", "object", "x-pick-body"),
        ("body.chamfer", "edges", "x-pick"),
        ("body.chamfer", "object", "x-pick-body"),
        ("feature.fillet", "edges", "x-pick"),
        ("feature.fillet", "body", "x-pick-body"),
        ("feature.chamfer", "edges", "x-pick"),
        ("feature.chamfer", "body", "x-pick-body"),
    ];

    fn input_of(id: &str) -> Value {
        surfacecraft_engine::cmd::find(id).map(|spec| (spec.input)()).expect("a registered command")
    }

    #[test]
    fn every_marker_resolves_and_the_engine_marks_exactly_the_fields_that_had_a_button() {
        let mut found: Vec<(String, String, String)> = Vec::new();
        for spec in surfacecraft_engine::cmd::registry() {
            let input = (spec.input)();
            let Some(props) = input.get("properties").and_then(Value::as_object) else { continue };
            for (name, prop) in props {
                for ext in [PICK_MARKER, PICK_BODY_MARKER] {
                    if prop.get(ext).is_some() {
                        found.push((spec.id.to_string(), name.clone(), ext.to_string()));
                    }
                }
                if prop.get(PICK_MARKER).is_some() {
                    assert!(PickMode::of(prop).is_some(), "{}.{name}: unknown pick mode {prop}", spec.id);
                }
            }
        }
        let mut want: Vec<(String, String, String)> =
            MARKED.iter().map(|(c, f, e)| ((*c).to_string(), (*f).to_string(), (*e).to_string())).collect();
        found.sort();
        want.sort();
        assert_eq!(found, want);
    }

    #[test]
    fn each_marked_field_takes_the_pick_its_mode_names() {
        let modes = [
            ("align.n_point", "from", PickMode::Points),
            ("align.n_point", "to", PickMode::Points),
            ("align.datum", "primary", PickMode::Plane),
            ("align.datum", "secondary", PickMode::Axis),
            ("align.datum", "tertiary", PickMode::Point),
            ("body.fillet", "edges", PickMode::Edges),
            ("feature.chamfer", "edges", PickMode::Edges),
        ];
        for (command, field, mode) in modes {
            let input = input_of(command);
            let prop = input.get("properties").and_then(|p| p.get(field)).expect("the field exists");
            assert_eq!(PickMode::of(prop), Some(mode), "{command}.{field}");
        }
        assert_eq!(body_field(&input_of("body.fillet")).as_deref(), Some("object"));
        assert_eq!(body_field(&input_of("feature.fillet")).as_deref(), Some("body"));
        assert_eq!(body_field(&input_of("align.datum")), None);
    }

    /// A pick is chosen by its marker, never by the parameter's name: a parameter named like a
    /// datum, an edge list or a body does nothing without a marker.
    #[test]
    fn a_field_is_picked_by_its_marker_not_by_its_name() {
        let input = json!({"properties": {
            "primary": {"type": "array"},
            "edges": {"type": "array"},
            "body": {"type": "string"},
            "from": {"type": "array", "x-pick": "edges"},
            "anything": {"type": "array", "x-pick": "points"},
        }});
        let mut marked = marked_fields(&input);
        marked.sort_by(|a, b| a.0.cmp(&b.0));
        assert_eq!(marked, vec![("anything".to_string(), PickMode::Points), ("from".to_string(), PickMode::Edges)]);
        assert_eq!(body_field(&input), None);
        assert!(pick_queue(&input, "primary", PickKind::Datum { need: 3 }).is_empty());
        assert_eq!(
            pick_queue(&input, "from", PickKind::Edges),
            vec![("from".to_string(), PickKind::Edges)],
            "a marked field picks what its marker names, whatever its name"
        );
    }

    #[test]
    fn a_datum_pick_goes_on_through_the_datums_after_it_in_3_2_1_order() {
        let input = input_of("align.datum");
        let datum = |n: &str, need: usize| (n.to_string(), PickKind::Datum { need });
        let from = |f: &str| pick_queue(&input, f, PickKind::Datum { need: 3 });
        assert_eq!(from("primary"), vec![datum("primary", 3), datum("secondary", 2), datum("tertiary", 1)]);
        assert_eq!(from("secondary"), vec![datum("secondary", 2), datum("tertiary", 1)]);
        assert_eq!(from("tertiary"), vec![datum("tertiary", 1)]);
        assert!(from("object").is_empty(), "an unmarked field starts no pick");
    }

    #[test]
    fn datum_modes_start_as_the_world_plane_axis_and_point() {
        assert_eq!(PickMode::Plane.kind(), PickKind::Datum { need: 3 });
        assert_eq!(PickMode::Axis.kind(), PickKind::Datum { need: 2 });
        assert_eq!(PickMode::Point.kind(), PickKind::Datum { need: 1 });
        assert_eq!(PickMode::Points.kind(), PickKind::Points);
        assert_eq!(PickMode::Edges.kind(), PickKind::Edges);
        assert_eq!(
            PickMode::Plane.default_value(),
            Some(json!({"plane": {"point": [0.0, 0.0, 0.0], "normal": [0.0, 0.0, 1.0]}}))
        );
        assert_eq!(
            PickMode::Axis.default_value(),
            Some(json!({"axis": {"point": [0.0, 0.0, 0.0], "direction": [1.0, 0.0, 0.0]}}))
        );
        assert_eq!(PickMode::Point.default_value(), Some(json!({"point": {"point": [0.0, 0.0, 0.0]}})));
        assert_eq!(PickMode::Edges.default_value(), None);
        assert_eq!(PickMode::from_marker("face"), None, "faces are not picked as a field yet");
    }
}
