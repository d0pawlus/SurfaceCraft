//! Drawing gestures: the clicks of the drawing tools, the polyline and the typed length, and the
//! state the editor keeps between them.
use super::snap::{arc_from_centre, arc_through, dist};
use super::tool::*;
use super::*;
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_sketch::{Constraint, Sketch};

impl SketchEditor {
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

    pub(super) fn cancel_gesture(&mut self) {
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

    pub(super) fn sketch_ref(&self) -> Option<u64> {
        self.target.map(|id| id.0)
    }

    fn add_cmd(&self, entity: Value) -> Vec<Cmd> {
        match self.sketch_ref() {
            Some(id) => vec![Cmd { id: "sketch.add", params: json!({"sketch": id, "entities": [entity]}) }],
            None => Vec::new(),
        }
    }

    pub(super) fn constrain_cmd(&self, c: &Constraint) -> Vec<Cmd> {
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
}
