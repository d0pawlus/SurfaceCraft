//! Dragging a sketch point with the mouse (P8-H7). In the select tool a press on a point and a
//! move drag it: the point follows the pointer, the solver moves the constrained rest of the
//! sketch with it, and a point the constraints fix stays where they put it. A release commits one
//! `sketch.move_point` (one undo step) if anything moved; a release where nothing moved issues no
//! command. `Esc` cancels the drag without a command. The solve is the same one the engine runs,
//! so the preview is the committed result.
use super::snap::{Hit, Want, pick};
use super::{Cmd, SketchEditor, Tool};
use serde_json::json;
use surfacecraft_doc::ObjectId;
use surfacecraft_sketch::{Sketch, move_point};

/// Smallest move (sketch units) that counts as a change.
pub const NO_MOVE: f64 = 1e-9;

/// A point being dragged.
#[derive(Clone, Debug, PartialEq)]
pub struct Drag {
    /// The sketch the point belongs to.
    pub sketch: ObjectId,
    /// Index of the point.
    pub point: usize,
    /// The sketch before the drag: the baseline the drag is valid against.
    pub start: Sketch,
    /// Where the pointer is now (sketch coordinates).
    pub target: [f64; 2],
    /// The last pointer position the constraints could hold: what a release commits.
    pub valid_target: [f64; 2],
    /// The sketch the solver leaves with the point at `valid_target`: drawn while dragging.
    pub preview: Sketch,
}

/// The sketch with point `point` set to `to` and solved (the move the engine commits); `None`
/// when the point does not exist or the constraints cannot be satisfied.
#[must_use]
pub fn moved(sk: &Sketch, point: usize, to: [f64; 2]) -> Option<Sketch> {
    move_point(sk, point, to).ok().map(|m| m.sketch)
}

/// Did any point end up somewhere else?
#[must_use]
pub fn changed(before: &Sketch, after: &Sketch) -> bool {
    before.points.len() != after.points.len()
        || before.points.iter().zip(&after.points).any(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]) > NO_MOVE)
}

impl SketchEditor {
    /// Starts dragging point `point` of `sk`, which the pointer is on (select tool only).
    pub fn drag_begin(&mut self, sk: &Sketch, point: usize, at: [f64; 2]) -> bool {
        let Some(sketch) = self.target else { return false };
        if self.tool != Tool::Select || self.drag.is_some() || point >= sk.points.len() {
            return false;
        }
        self.drag = Some(Drag { sketch, point, start: sk.clone(), target: at, valid_target: at, preview: sk.clone() });
        self.drag_update(at)
    }

    /// Starts a drag from the point under the PRESS (`press`, within `tol`), with the pointer now at
    /// `current`: egui reports a drag only after the pointer has passed its threshold, so the
    /// pointer may already be far from, or over another, point.
    pub fn drag_press(&mut self, sk: &Sketch, press: [f64; 2], current: [f64; 2], tol: f64) -> bool {
        match pick(sk, press, tol, Want::Point) {
            Some(Hit::Point(i)) => self.drag_begin(sk, i, current),
            _ => false,
        }
    }

    /// Cancels the drag when `sk` (sketch `id`, as the document has it now) is not the baseline
    /// the drag started from: an undo, a redo or any other change made while the button is held.
    pub fn drag_check(&mut self, id: ObjectId, sk: &Sketch) {
        if self.drag.as_ref().is_some_and(|d| d.sketch != id || d.start != *sk) {
            self.drag_cancel();
        }
    }

    /// Moves the dragged point's target to `to` and recomputes the preview from the start. A
    /// position the constraints cannot hold leaves the preview at its last valid state.
    pub fn drag_update(&mut self, to: [f64; 2]) -> bool {
        let Some(d) = self.drag.as_mut() else { return false };
        d.target = to;
        if let Some(next) = moved(&d.start, d.point, to) {
            d.preview = next;
            d.valid_target = to;
        }
        true
    }

    /// Ends the drag: the one command that moves the point, or none if nothing would move.
    pub fn drag_end(&mut self) -> Vec<Cmd> {
        let Some(d) = self.drag.take() else { return Vec::new() };
        let Some(id) = self.target else { return Vec::new() };
        if id != d.sketch || !changed(&d.start, &d.preview) {
            return Vec::new();
        }
        vec![Cmd { id: "sketch.move_point", params: json!({"sketch": id.0, "point": d.point, "to": d.valid_target}) }]
    }

    /// Drops the drag in progress, if any, without a command.
    pub fn drag_cancel(&mut self) {
        self.drag = None;
    }
}
