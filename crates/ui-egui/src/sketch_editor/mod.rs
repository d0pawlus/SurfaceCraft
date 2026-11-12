//! The sketch editing mode (G5): draw on the sketch plane with the mouse, constrain by clicking,
//! dimension by typing a value, and see the solver's status live.
//!
//! The editor is a plain state machine over sketch coordinates ([`SketchEditor`]): a click, a
//! typed value or a key goes in, a list of [`Cmd`]s comes out. Every [`Cmd`] is one `sketch.*`
//! engine command (`sketch.add`, `sketch.constrain`, `sketch.edit_constraint`), run by the app
//! like any other action, so one gesture is one command is one undo step; `Esc` cancels the
//! gesture in progress without a command. The state machine knows nothing about egui or the GPU
//! and is tested without them; [`ui`] draws the overlay and feeds the pointer in.
pub mod drag;
pub mod snap;
mod ui;
pub mod view;

pub use snap::{Hit, Snap, SnapConfig, SnapKind, Want};

use serde_json::{Value, json};
use snap::{arc_from_centre, arc_geom, arc_through, circle_geom, dist, pick};
use surfacecraft_doc::ObjectId;
use surfacecraft_sketch::{Constraint, Entity, Sketch, SolveReport, SolveStatus};

use crate::theme::Token;

/// One engine command the editor wants run.
#[derive(Clone, Debug, PartialEq)]
pub struct Cmd {
    /// The command id (`sketch.add`, `sketch.constrain`, `sketch.edit_constraint`).
    pub id: &'static str,
    /// Its parameters.
    pub params: Value,
}

/// A sketch tool.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum Tool {
    /// No drawing: pick and double-click dimensions.
    Select,
    /// A line from two clicks.
    Line,
    /// A chain of lines (Enter, right click or a click on the last point ends it; a click on
    /// the first point closes it).
    Polyline,
    /// An arc from start, end and a point on it.
    Arc3,
    /// An arc from centre, start and end direction.
    ArcCentre,
    /// A circle from centre and a point on it.
    Circle,
    /// A rectangle from two corners.
    Rect,
    /// Horizontal line.
    Horizontal,
    /// Vertical line.
    Vertical,
    /// Two parallel lines.
    Parallel,
    /// Two perpendicular lines.
    Perpendicular,
    /// A line tangent to an arc or circle.
    Tangent,
    /// Two coincident points.
    Coincident,
    /// Two lines of equal length.
    Equal,
    /// A point fixed where it is.
    Fix,
    /// A length, radius or distance, typed after the click.
    Dimension,
    /// The angle between two lines, typed after the clicks.
    Angle,
}

impl Tool {
    /// The drawing tools.
    pub const DRAW: [Tool; 6] = [Tool::Line, Tool::Polyline, Tool::Arc3, Tool::ArcCentre, Tool::Circle, Tool::Rect];
    /// The constraint tools.
    pub const CONSTRAIN: [Tool; 10] = [
        Tool::Horizontal,
        Tool::Vertical,
        Tool::Parallel,
        Tool::Perpendicular,
        Tool::Tangent,
        Tool::Coincident,
        Tool::Equal,
        Tool::Fix,
        Tool::Dimension,
        Tool::Angle,
    ];

    /// The catalogue key of the tool's name.
    #[must_use]
    pub fn label_key(self) -> &'static str {
        match self {
            Tool::Select => "sketch.tool.select",
            Tool::Line => "sketch.tool.line",
            Tool::Polyline => "sketch.tool.polyline",
            Tool::Arc3 => "sketch.tool.arc3",
            Tool::ArcCentre => "sketch.tool.arc_centre",
            Tool::Circle => "sketch.tool.circle",
            Tool::Rect => "sketch.tool.rect",
            Tool::Horizontal => "sketch.tool.horizontal",
            Tool::Vertical => "sketch.tool.vertical",
            Tool::Parallel => "sketch.tool.parallel",
            Tool::Perpendicular => "sketch.tool.perpendicular",
            Tool::Tangent => "sketch.tool.tangent",
            Tool::Coincident => "sketch.tool.coincident",
            Tool::Equal => "sketch.tool.equal",
            Tool::Fix => "sketch.tool.fix",
            Tool::Dimension => "sketch.tool.dimension",
            Tool::Angle => "sketch.tool.angle",
        }
    }

    /// The icon of the tool in the sketch toolbar.
    #[must_use]
    pub fn icon(self) -> &'static str {
        match self {
            Tool::Select => "tool-navigate",
            Tool::Line => "sk-line",
            Tool::Polyline => "sk-polyline",
            Tool::Arc3 => "sk-arc3",
            Tool::ArcCentre => "sk-arc-centre",
            Tool::Circle => "sk-circle",
            Tool::Rect => "sk-rect",
            Tool::Horizontal => "sk-horizontal",
            Tool::Vertical => "sk-vertical",
            Tool::Parallel => "sk-parallel",
            Tool::Perpendicular => "sk-perpendicular",
            Tool::Tangent => "sk-tangent",
            Tool::Coincident => "sk-coincident",
            Tool::Equal => "sk-equal",
            Tool::Fix => "sk-fix",
            Tool::Dimension => "sk-dimension",
            Tool::Angle => "sk-angle",
        }
    }

    /// The catalogue key of the hint shown while the tool is active.
    #[must_use]
    pub fn hint_key(self) -> &'static str {
        match self {
            Tool::Select => "sketch.hint.select",
            Tool::Line => "sketch.hint.line",
            Tool::Polyline => "sketch.hint.polyline",
            Tool::Arc3 => "sketch.hint.arc3",
            Tool::ArcCentre => "sketch.hint.arc_centre",
            Tool::Circle => "sketch.hint.circle",
            Tool::Rect => "sketch.hint.rect",
            Tool::Horizontal | Tool::Vertical | Tool::Fix => "sketch.hint.one",
            Tool::Parallel | Tool::Perpendicular | Tool::Equal => "sketch.hint.two_lines",
            Tool::Tangent => "sketch.hint.tangent",
            Tool::Coincident => "sketch.hint.two_points",
            Tool::Dimension => "sketch.hint.dimension",
            Tool::Angle => "sketch.hint.angle",
        }
    }

    /// Does the tool draw geometry?
    #[must_use]
    pub fn draws(self) -> bool {
        Tool::DRAW.contains(&self)
    }

    /// What the first click of a constraint tool looks for.
    #[must_use]
    pub fn want(self) -> Want {
        match self {
            Tool::Horizontal | Tool::Vertical | Tool::Parallel | Tool::Perpendicular | Tool::Equal | Tool::Angle => {
                Want::Line
            }
            Tool::Tangent => Want::LineOrCircular,
            Tool::Coincident | Tool::Fix => Want::Point,
            _ => Want::PointOrEntity,
        }
    }
}

/// A dimension: what it measures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DimKind {
    /// Length of a line.
    Length {
        /// Line entity.
        line: usize,
    },
    /// Radius of an arc or circle.
    Radius {
        /// Arc or circle entity.
        entity: usize,
    },
    /// Distance between two points.
    Distance {
        /// First point.
        a: usize,
        /// Second point.
        b: usize,
    },
    /// Angle from line `a` to line `b`, degrees.
    Angle {
        /// First line.
        a: usize,
        /// Second line.
        b: usize,
    },
}

impl DimKind {
    /// The dimension a constraint is, if it is one.
    #[must_use]
    pub fn of(c: &Constraint) -> Option<(DimKind, f64)> {
        match *c {
            Constraint::Length { line, value } => Some((DimKind::Length { line }, value)),
            Constraint::Radius { entity, value } => Some((DimKind::Radius { entity }, value)),
            Constraint::Distance { a, b, value } => Some((DimKind::Distance { a, b }, value)),
            Constraint::Angle { a, b, degrees } => Some((DimKind::Angle { a, b }, degrees)),
            _ => None,
        }
    }

    /// The constraint with value `v`.
    #[must_use]
    pub fn build(self, v: f64) -> Constraint {
        match self {
            DimKind::Length { line } => Constraint::Length { line, value: v },
            DimKind::Radius { entity } => Constraint::Radius { entity, value: v },
            DimKind::Distance { a, b } => Constraint::Distance { a, b, value: v },
            DimKind::Angle { a, b } => Constraint::Angle { a, b, degrees: v },
        }
    }

    /// The value now in the sketch.
    #[must_use]
    pub fn measure(self, sk: &Sketch) -> Option<f64> {
        let line_dir = |e: usize| match sk.entities.get(e)? {
            Entity::Line { a, b } => {
                let (pa, pb) = (sk.points.get(*a)?, sk.points.get(*b)?);
                Some([pb[0] - pa[0], pb[1] - pa[1]])
            }
            _ => None,
        };
        match self {
            DimKind::Length { line } => line_dir(line).map(|d| d[0].hypot(d[1])),
            DimKind::Radius { entity } => circle_geom(sk, entity).map(|(_, r)| r),
            DimKind::Distance { a, b } => Some(dist(*sk.points.get(a)?, *sk.points.get(b)?)),
            DimKind::Angle { a, b } => {
                let (da, db) = (line_dir(a)?, line_dir(b)?);
                let cross = da[0] * db[1] - da[1] * db[0];
                let dot = da[0] * db[0] + da[1] * db[1];
                Some(cross.atan2(dot).to_degrees())
            }
        }
    }

    /// The catalogue key of the dimension's name.
    #[must_use]
    pub fn label_key(self) -> &'static str {
        match self {
            DimKind::Length { .. } => "sketch.dim.length",
            DimKind::Radius { .. } => "sketch.dim.radius",
            DimKind::Distance { .. } => "sketch.dim.distance",
            DimKind::Angle { .. } => "sketch.dim.angle",
        }
    }

    fn positive_only(self) -> bool {
        !matches!(self, DimKind::Angle { .. })
    }

    fn indices_exist(self, sk: &Sketch) -> bool {
        match self {
            DimKind::Length { line } => matches!(sk.entities.get(line), Some(Entity::Line { .. })),
            DimKind::Radius { entity } => {
                matches!(sk.entities.get(entity), Some(Entity::Arc { .. } | Entity::Circle { .. }))
            }
            DimKind::Distance { a, b } => a < sk.points.len() && b < sk.points.len(),
            DimKind::Angle { a, b } => {
                matches!(sk.entities.get(a), Some(Entity::Line { .. }))
                    && matches!(sk.entities.get(b), Some(Entity::Line { .. }))
            }
        }
    }
}

/// A dimension whose value is being typed: new (`index` is `None`) or an existing constraint.
#[derive(Clone, Debug, PartialEq)]
pub struct Pending {
    /// What it measures.
    pub kind: DimKind,
    /// The value text being edited.
    pub text: String,
    /// The constraint being edited (`None`: a new dimension).
    pub index: Option<usize>,
}

/// The editor's live read-out while a gesture has an anchor: length and angle of the rubber
/// band, or the radius of a circle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Readout {
    /// Length or radius, sketch units.
    pub length: f64,
    /// Direction angle, degrees (counter-clockwise from x).
    pub angle: f64,
    /// A radius (circle) rather than a length.
    pub radius: bool,
}

/// The state of the sketch editing mode.
#[derive(Clone, Debug)]
pub struct SketchEditor {
    /// The sketch being edited (`None`: not editing).
    pub target: Option<ObjectId>,
    /// The active tool.
    pub tool: Tool,
    /// Which snaps are on.
    pub snap: SnapConfig,
    /// Points placed by the gesture in progress (sketch coordinates).
    pub gesture: Vec<[f64; 2]>,
    /// Entities and points picked by the constraint tool in progress.
    pub picks: Vec<Hit>,
    /// A dimension waiting for its value.
    pub pending: Option<Pending>,
    /// A number typed while drawing (dynamic input).
    pub typed: String,
    /// The snapped cursor position.
    pub hover: Option<Snap>,
    /// The catalogue key of the last problem (nothing drawn: collinear arc, bad value).
    pub notice: Option<&'static str>,
    /// Where the dimension labels were drawn this frame (screen points), by constraint index.
    pub labels: Vec<(usize, [f32; 4])>,
    /// Where the editor's own controls were drawn this frame (screen points, `[x0, y0, x1, y1]`):
    /// clicks there are not clicks on the sketch.
    pub chrome: Vec<[f32; 4]>,
    /// The strip's details (snap options, hints, the constraint list) are shown.
    pub details_open: bool,
    /// A point being dragged with the mouse (see [`drag`]).
    pub drag: Option<drag::Drag>,
}

impl Default for SketchEditor {
    fn default() -> Self {
        Self {
            target: None,
            tool: Tool::Select,
            snap: SnapConfig::default(),
            gesture: Vec::new(),
            picks: Vec::new(),
            pending: None,
            typed: String::new(),
            hover: None,
            notice: None,
            labels: Vec::new(),
            chrome: Vec::new(),
            details_open: false,
            drag: None,
        }
    }
}

/// Formats a value without trailing zeros (`12.5`, `3`).
#[must_use]
pub fn fmt_value(v: f64) -> String {
    let s = format!("{v:.4}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() || s == "-" || s == "-0" { "0".into() } else { s.into() }
}

/// Parses a typed number (a comma works as the decimal point).
#[must_use]
pub fn parse_value(text: &str) -> Option<f64> {
    let v: f64 = text.trim().replace(',', ".").parse().ok()?;
    v.is_finite().then_some(v)
}

impl SketchEditor {
    /// Starts editing sketch `id`.
    pub fn begin(&mut self, id: ObjectId) {
        *self = Self { target: Some(id), snap: self.snap, ..Self::default() };
    }

    /// Leaves the editing mode.
    pub fn end(&mut self) {
        *self = Self { snap: self.snap, ..Self::default() };
    }

    /// Is a sketch being edited?
    #[must_use]
    pub fn active(&self) -> bool {
        self.target.is_some()
    }

    /// Chooses a tool (the gesture in progress is dropped).
    pub fn set_tool(&mut self, tool: Tool) {
        self.cancel_gesture();
        self.tool = tool;
    }

    fn cancel_gesture(&mut self) {
        self.gesture.clear();
        self.picks.clear();
        self.pending = None;
        self.typed.clear();
        self.notice = None;
    }

    /// `Esc`: cancels the gesture in progress; with none, drops back to the select tool.
    /// Never issues a command.
    pub fn escape(&mut self) {
        if self.drag.is_some() {
            self.drag_cancel();
            return;
        }
        let busy =
            !self.gesture.is_empty() || !self.picks.is_empty() || self.pending.is_some() || !self.typed.is_empty();
        if busy {
            self.cancel_gesture();
        } else {
            self.tool = Tool::Select;
            self.notice = None;
        }
    }

    /// Forgets picks and pending values that point at things the sketch no longer has (after an
    /// undo, say).
    pub fn sanitize(&mut self, sk: &Sketch) {
        let ok = |h: &Hit| match *h {
            Hit::Point(i) => i < sk.points.len(),
            Hit::Entity(e) => e < sk.entities.len(),
        };
        self.picks.retain(ok);
        if let Some(p) = &self.pending
            && (!p.kind.indices_exist(sk) || p.index.is_some_and(|i| i >= sk.constraints.len()))
        {
            self.pending = None;
        }
    }

    fn sketch_ref(&self) -> Option<u64> {
        self.target.map(|id| id.0)
    }

    fn add_cmd(&self, entity: Value) -> Vec<Cmd> {
        match self.sketch_ref() {
            Some(id) => vec![Cmd { id: "sketch.add", params: json!({"sketch": id, "entities": [entity]}) }],
            None => Vec::new(),
        }
    }

    fn constrain_cmd(&self, c: &Constraint) -> Vec<Cmd> {
        match (self.sketch_ref(), serde_json::to_value(c)) {
            (Some(id), Ok(v)) => {
                vec![Cmd { id: "sketch.constrain", params: json!({"sketch": id, "constraints": [v]}) }]
            }
            _ => Vec::new(),
        }
    }

    /// The last point placed by the gesture (the anchor of the rubber band and the inference).
    #[must_use]
    pub fn anchor(&self) -> Option<[f64; 2]> {
        if self.tool.draws() { self.gesture.last().copied() } else { None }
    }

    /// A click at snapped position `at` (`tol`: the pick radius in sketch units). Returns the
    /// commands to run.
    pub fn click(&mut self, sk: &Sketch, at: Snap, tol: f64) -> Vec<Cmd> {
        self.notice = None;
        if self.target.is_none() {
            return Vec::new();
        }
        match self.tool {
            Tool::Select => Vec::new(),
            t if t.draws() => self.click_draw(at.pos),
            _ => self.click_constrain(sk, at.pos, tol),
        }
    }

    fn click_draw(&mut self, p: [f64; 2]) -> Vec<Cmd> {
        match self.tool {
            Tool::Line => {
                let Some(&a) = self.gesture.first() else {
                    self.gesture.push(p);
                    return Vec::new();
                };
                if dist(a, p) < 1e-9 {
                    return Vec::new();
                }
                self.gesture.clear();
                self.add_cmd(json!({"line": {"from": a, "to": p}}))
            }
            Tool::Polyline => self.click_polyline(p),
            Tool::Circle => {
                let Some(&c) = self.gesture.first() else {
                    self.gesture.push(p);
                    return Vec::new();
                };
                let r = dist(c, p);
                if r < 1e-9 {
                    return Vec::new();
                }
                self.gesture.clear();
                self.add_cmd(json!({"circle": {"center": c, "radius": r}}))
            }
            Tool::Rect => {
                let Some(&a) = self.gesture.first() else {
                    self.gesture.push(p);
                    return Vec::new();
                };
                if (a[0] - p[0]).abs() < 1e-9 || (a[1] - p[1]).abs() < 1e-9 {
                    return Vec::new();
                }
                self.gesture.clear();
                self.add_cmd(json!({"polygon": {"points": [[a[0], a[1]], [p[0], a[1]], [p[0], p[1]], [a[0], p[1]]]}}))
            }
            Tool::Arc3 | Tool::ArcCentre => {
                if self.gesture.len() < 2 {
                    if self.gesture.last().is_some_and(|q| dist(*q, p) < 1e-9) {
                        return Vec::new();
                    }
                    self.gesture.push(p);
                    return Vec::new();
                }
                let (g0, g1) = (self.gesture[0], self.gesture[1]);
                let arc = if self.tool == Tool::Arc3 { arc_through(g0, p, g1) } else { arc_from_centre(g0, g1, p) };
                self.gesture.clear();
                match arc {
                    Some((c, a, b)) => self.add_cmd(json!({"arc": {"center": c, "start": a, "end": b}})),
                    None => {
                        self.notice = Some("sketch.notice.arc_degenerate");
                        Vec::new()
                    }
                }
            }
            _ => Vec::new(),
        }
    }

    fn click_polyline(&mut self, p: [f64; 2]) -> Vec<Cmd> {
        if let Some(&last) = self.gesture.last()
            && dist(last, p) < 1e-9
        {
            // a click on the last point (a double click) ends the chain
            return self.finish();
        }
        if self.gesture.len() >= 3
            && let Some(&first) = self.gesture.first()
            && dist(first, p) < 1e-9
        {
            let pts = std::mem::take(&mut self.gesture);
            return self.add_cmd(json!({"polygon": {"points": pts}}));
        }
        self.gesture.push(p);
        Vec::new()
    }

    /// `Enter` / right click: ends a polyline (one `sketch.add` with all its lines).
    pub fn finish(&mut self) -> Vec<Cmd> {
        if self.tool != Tool::Polyline || self.gesture.len() < 2 {
            self.gesture.clear();
            return Vec::new();
        }
        let pts = std::mem::take(&mut self.gesture);
        let Some(id) = self.sketch_ref() else { return Vec::new() };
        let entities: Vec<Value> = pts.windows(2).map(|w| json!({"line": {"from": w[0], "to": w[1]}})).collect();
        vec![Cmd { id: "sketch.add", params: json!({"sketch": id, "entities": entities}) }]
    }

    /// A number typed while drawing (dynamic input): places the next point `value` away from the
    /// anchor towards the cursor (a circle: the radius).
    pub fn type_value(&mut self, sk: &Sketch, value: f64, cursor: [f64; 2]) -> Vec<Cmd> {
        let Some(a) = self.anchor() else { return Vec::new() };
        self.typed.clear();
        if !(value.is_finite() && value > 0.0) || !matches!(self.tool, Tool::Line | Tool::Polyline | Tool::Circle) {
            self.notice = Some("sketch.notice.value");
            return Vec::new();
        }
        let (dx, dy) = (cursor[0] - a[0], cursor[1] - a[1]);
        let l = dx.hypot(dy);
        let (ux, uy) = if l < 1e-12 { (1.0, 0.0) } else { (dx / l, dy / l) };
        let end = [a[0] + ux * value, a[1] + uy * value];
        self.click(sk, Snap::free(end), 0.0)
    }

    fn click_constrain(&mut self, sk: &Sketch, p: [f64; 2], tol: f64) -> Vec<Cmd> {
        let tool = self.tool;
        let Some(hit) = pick(sk, p, tol, tool.want()) else { return Vec::new() };
        match tool {
            Tool::Horizontal | Tool::Vertical => {
                let Hit::Entity(line) = hit else { return Vec::new() };
                let c = if tool == Tool::Horizontal {
                    Constraint::Horizontal { line }
                } else {
                    Constraint::Vertical { line }
                };
                self.constrain_cmd(&c)
            }
            Tool::Fix => {
                let Hit::Point(point) = hit else { return Vec::new() };
                let Some(at) = sk.points.get(point).copied() else { return Vec::new() };
                self.constrain_cmd(&Constraint::Fix { point, at })
            }
            Tool::Parallel | Tool::Perpendicular | Tool::Equal => {
                let Hit::Entity(b) = hit else { return Vec::new() };
                let Some(Hit::Entity(a)) = self.picks.first().copied() else {
                    self.picks = vec![hit];
                    return Vec::new();
                };
                if a == b {
                    return Vec::new();
                }
                self.picks.clear();
                let c = match tool {
                    Tool::Parallel => Constraint::Parallel { a, b },
                    Tool::Perpendicular => Constraint::Perpendicular { a, b },
                    _ => Constraint::Equal { a, b },
                };
                self.constrain_cmd(&c)
            }
            Tool::Tangent => {
                let Hit::Entity(e) = hit else { return Vec::new() };
                let is_line = |i: usize| matches!(sk.entities.get(i), Some(Entity::Line { .. }));
                match self.picks.first().copied() {
                    Some(Hit::Entity(f)) if is_line(f) != is_line(e) => {
                        self.picks.clear();
                        let (line, entity) = if is_line(e) { (e, f) } else { (f, e) };
                        self.constrain_cmd(&Constraint::Tangent { line, entity })
                    }
                    _ => {
                        self.picks = vec![hit];
                        Vec::new()
                    }
                }
            }
            Tool::Coincident => {
                let Hit::Point(b) = hit else { return Vec::new() };
                match self.picks.first().copied() {
                    Some(Hit::Point(a)) if a != b => {
                        self.picks.clear();
                        self.constrain_cmd(&Constraint::Coincident { a, b })
                    }
                    Some(Hit::Point(_)) => Vec::new(),
                    _ => {
                        self.picks = vec![hit];
                        Vec::new()
                    }
                }
            }
            Tool::Dimension => self.click_dimension(sk, hit),
            Tool::Angle => {
                let Hit::Entity(b) = hit else { return Vec::new() };
                match self.picks.first().copied() {
                    Some(Hit::Entity(a)) if a != b => {
                        self.picks.clear();
                        self.begin_pending(sk, DimKind::Angle { a, b }, None);
                        Vec::new()
                    }
                    Some(Hit::Entity(_)) => Vec::new(),
                    _ => {
                        self.picks = vec![hit];
                        Vec::new()
                    }
                }
            }
            _ => Vec::new(),
        }
    }

    fn click_dimension(&mut self, sk: &Sketch, hit: Hit) -> Vec<Cmd> {
        match hit {
            Hit::Point(b) => match self.picks.first().copied() {
                Some(Hit::Point(a)) if a != b => {
                    self.picks.clear();
                    self.begin_pending(sk, DimKind::Distance { a, b }, None);
                }
                Some(Hit::Point(_)) => {}
                _ => self.picks = vec![hit],
            },
            Hit::Entity(e) => {
                self.picks.clear();
                let kind = match sk.entities.get(e) {
                    Some(Entity::Line { .. }) => Some(DimKind::Length { line: e }),
                    Some(Entity::Arc { .. } | Entity::Circle { .. }) => Some(DimKind::Radius { entity: e }),
                    None => None,
                };
                if let Some(k) = kind {
                    self.begin_pending(sk, k, None);
                }
            }
        }
        Vec::new()
    }

    fn begin_pending(&mut self, sk: &Sketch, kind: DimKind, index: Option<usize>) {
        let text = kind.measure(sk).map_or_else(String::new, fmt_value);
        self.pending = Some(Pending { kind, text, index });
    }

    /// Opens constraint `index` for editing when it is a dimension (double click on its label).
    pub fn begin_edit(&mut self, sk: &Sketch, index: usize) -> bool {
        let Some((kind, value)) = sk.constraints.get(index).and_then(DimKind::of) else { return false };
        self.cancel_gesture();
        self.pending = Some(Pending { kind, text: fmt_value(value), index: Some(index) });
        true
    }

    /// Enter in the value field: the new dimension (`sketch.constrain`) or the edited one
    /// (`sketch.edit_constraint`). A value that is not a number (or not positive for a length)
    /// keeps the field open and sets a notice.
    pub fn commit_pending(&mut self) -> Vec<Cmd> {
        let Some(p) = self.pending.clone() else { return Vec::new() };
        let Some(v) = parse_value(&p.text).filter(|v| !p.kind.positive_only() || *v > 0.0) else {
            self.notice = Some("sketch.notice.value");
            return Vec::new();
        };
        self.pending = None;
        self.notice = None;
        let c = p.kind.build(v);
        match p.index {
            None => self.constrain_cmd(&c),
            Some(index) => match (self.sketch_ref(), serde_json::to_value(&c)) {
                (Some(id), Ok(cv)) => vec![Cmd {
                    id: "sketch.edit_constraint",
                    params: json!({"sketch": id, "index": index, "constraint": cv}),
                }],
                _ => Vec::new(),
            },
        }
    }

    /// Removes constraint `index` (`sketch.edit_constraint` without a replacement).
    #[must_use]
    pub fn delete_constraint(&self, index: usize) -> Vec<Cmd> {
        match self.sketch_ref() {
            Some(id) => vec![Cmd { id: "sketch.edit_constraint", params: json!({"sketch": id, "index": index}) }],
            None => Vec::new(),
        }
    }

    /// The dimension label under screen point `(x, y)`, from the labels of the last frame.
    #[must_use]
    pub fn dimension_at(&self, x: f32, y: f32) -> Option<usize> {
        self.labels.iter().find(|(_, r)| x >= r[0] && x <= r[2] && y >= r[1] && y <= r[3]).map(|(i, _)| *i)
    }

    /// The rubber band of the gesture in progress, as polylines in sketch coordinates, for the
    /// cursor at `hover`.
    #[must_use]
    pub fn preview(&self, hover: [f64; 2]) -> Vec<Vec<[f64; 2]>> {
        let g = &self.gesture;
        match (self.tool, g.len()) {
            (Tool::Line, 1) => vec![vec![g[0], hover]],
            (Tool::Polyline, n) if n >= 1 => {
                let mut pts = g.clone();
                pts.push(hover);
                vec![pts]
            }
            (Tool::Circle, 1) => {
                vec![snap::sample_arc(g[0], dist(g[0], hover), 0.0, std::f64::consts::TAU, 72)]
            }
            (Tool::Rect, 1) => {
                vec![vec![g[0], [hover[0], g[0][1]], hover, [g[0][0], hover[1]], g[0]]]
            }
            (Tool::Arc3, 1) => vec![vec![g[0], hover]],
            (Tool::Arc3, 2) => arc_through(g[0], hover, g[1]).map_or_else(
                || vec![vec![g[0], hover, g[1]]],
                |(c, a, b)| {
                    vec![snap::sample_arc(
                        c,
                        dist(c, a),
                        (a[1] - c[1]).atan2(a[0] - c[0]),
                        snap::ccw_angle(c, a, b),
                        48,
                    )]
                },
            ),
            (Tool::ArcCentre, 1) => vec![vec![g[0], hover]],
            (Tool::ArcCentre, 2) => arc_from_centre(g[0], g[1], hover).map_or_else(
                || vec![vec![g[0], g[1]]],
                |(c, a, b)| {
                    vec![
                        vec![c, a],
                        snap::sample_arc(c, dist(c, a), (a[1] - c[1]).atan2(a[0] - c[0]), snap::ccw_angle(c, a, b), 48),
                    ]
                },
            ),
            _ => Vec::new(),
        }
    }

    /// The live length/angle (or radius) for the cursor at `hover`.
    #[must_use]
    pub fn readout(&self, hover: [f64; 2]) -> Option<Readout> {
        let a = self.anchor()?;
        let l = dist(a, hover);
        let angle = (hover[1] - a[1]).atan2(hover[0] - a[0]).to_degrees();
        Some(Readout { length: l, angle, radius: self.tool == Tool::Circle })
    }
}

/// What the solver says, as a catalogue key, a theme token and the numbers to show.
#[derive(Clone, Debug, PartialEq)]
pub struct StatusView {
    /// The catalogue key of the status line (`{dof}` and `{n}` placeholders).
    pub key: &'static str,
    /// The token of its colour.
    pub token: Token,
    /// Remaining degrees of freedom.
    pub dof: usize,
    /// Constraints that contradict the others (indices).
    pub conflicting: Vec<usize>,
    /// Constraints that add nothing (indices; includes the conflicting).
    pub redundant: Vec<usize>,
}

/// The status of the last solve (`None`: the sketch was never solved).
#[must_use]
pub fn status_view(report: Option<&SolveReport>) -> StatusView {
    let Some(r) = report else {
        return StatusView {
            key: "sketch.status.unsolved",
            token: Token::TextMuted,
            dof: 0,
            conflicting: Vec::new(),
            redundant: Vec::new(),
        };
    };
    let (key, token) = match r.status {
        SolveStatus::FullyDefined => ("sketch.status.fully", Token::StatusOk),
        SolveStatus::UnderDefined => ("sketch.status.under", Token::StatusWarning),
        SolveStatus::OverDefined => ("sketch.status.over", Token::StatusError),
        SolveStatus::Failed => ("sketch.status.failed", Token::StatusError),
    };
    StatusView { key, token, dof: r.dof, conflicting: r.conflicting.clone(), redundant: r.redundant.clone() }
}

/// The short mark drawn next to a constraint in the viewport.
#[must_use]
pub fn constraint_mark(c: &Constraint) -> String {
    match c {
        Constraint::Fix { .. } => "Fix".into(),
        Constraint::Coincident { .. } => "C".into(),
        Constraint::Horizontal { .. } => "H".into(),
        Constraint::Vertical { .. } => "V".into(),
        Constraint::Parallel { .. } => "//".into(),
        Constraint::Perpendicular { .. } => "_|_".into(),
        Constraint::Equal { .. } => "=".into(),
        Constraint::PointOnLine { .. } | Constraint::PointOnCircle { .. } => "On".into(),
        Constraint::Tangent { .. } => "T".into(),
        Constraint::Midpoint { .. } => "Mid".into(),
        Constraint::Length { value, .. } | Constraint::Distance { value, .. } => fmt_value(*value),
        Constraint::Radius { value, .. } => format!("R{}", fmt_value(*value)),
        Constraint::Angle { degrees, .. } => format!("{}\u{b0}", fmt_value(*degrees)),
    }
}

/// Where a constraint's mark is drawn (sketch coordinates): the points or entities it ties,
/// one anchor per thing (a dimension: a single one).
#[must_use]
pub fn constraint_anchors(sk: &Sketch, c: &Constraint) -> Vec<[f64; 2]> {
    let pt = |i: usize| sk.points.get(i).copied();
    let mid = |e: usize| match sk.entities.get(e)? {
        Entity::Line { a, b } => {
            let (pa, pb) = (sk.points.get(*a)?, sk.points.get(*b)?);
            Some([(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0])
        }
        _ => None,
    };
    let on_circle = |e: usize| {
        let (c, r) = circle_geom(sk, e)?;
        let a = arc_geom(sk, e).map_or(std::f64::consts::FRAC_PI_4, |g| g.a0 + g.sweep / 2.0);
        Some([c[0] + r * a.cos(), c[1] + r * a.sin()])
    };
    let v: Vec<Option<[f64; 2]>> = match *c {
        Constraint::Fix { point, .. } => vec![pt(point)],
        Constraint::Coincident { a, b } => {
            vec![pt(a).zip(pt(b)).map(|(p, q)| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0])]
        }
        Constraint::Horizontal { line } | Constraint::Vertical { line } | Constraint::Length { line, .. } => {
            vec![mid(line)]
        }
        Constraint::Distance { a, b, .. } => {
            vec![pt(a).zip(pt(b)).map(|(p, q)| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0])]
        }
        Constraint::Parallel { a, b } | Constraint::Perpendicular { a, b } | Constraint::Equal { a, b } => {
            vec![mid(a), mid(b)]
        }
        Constraint::Angle { a, b, .. } => {
            vec![mid(a).zip(mid(b)).map(|(p, q)| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0])]
        }
        Constraint::Radius { entity, .. } => vec![on_circle(entity)],
        Constraint::PointOnLine { point, .. }
        | Constraint::PointOnCircle { point, .. }
        | Constraint::Midpoint { point, .. } => {
            vec![pt(point)]
        }
        Constraint::Tangent { line, entity } => vec![mid(line).or_else(|| on_circle(entity))],
    };
    v.into_iter().flatten().collect()
}

/// The entities a constraint ties (points excluded).
#[must_use]
pub fn constraint_entities(c: &Constraint) -> Vec<usize> {
    match *c {
        Constraint::Horizontal { line }
        | Constraint::Vertical { line }
        | Constraint::Length { line, .. }
        | Constraint::PointOnLine { line, .. }
        | Constraint::Midpoint { line, .. } => vec![line],
        Constraint::Parallel { a, b }
        | Constraint::Perpendicular { a, b }
        | Constraint::Equal { a, b }
        | Constraint::Angle { a, b, .. } => {
            vec![a, b]
        }
        Constraint::Radius { entity, .. } | Constraint::PointOnCircle { entity, .. } => vec![entity],
        Constraint::Tangent { line, entity } => vec![line, entity],
        Constraint::Fix { .. } | Constraint::Coincident { .. } | Constraint::Distance { .. } => Vec::new(),
    }
}
