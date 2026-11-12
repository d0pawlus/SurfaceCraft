//! Dragging a sketch point with the mouse (P8-H7). In the select tool a press on a point and a
//! move drag it: the point follows the pointer, the solver moves the constrained rest of the
//! sketch with it, and a point the constraints fix stays where they put it. A release commits one
//! `sketch.move_point` (one undo step) if anything moved; a release where nothing moved issues no
//! command. `Esc` cancels the drag without a command. The solve is the same one the engine runs,
//! so the preview is the committed result.
use super::{Cmd, SketchEditor, Tool};
use serde_json::json;
use surfacecraft_sketch::{Sketch, solve};

/// Smallest move (sketch units) that counts as a change.
pub const NO_MOVE: f64 = 1e-9;

/// A point being dragged.
#[derive(Clone, Debug, PartialEq)]
pub struct Drag {
    /// Index of the point.
    pub point: usize,
    /// The sketch before the drag.
    pub start: Sketch,
    /// Where the pointer is now (sketch coordinates).
    pub target: [f64; 2],
    /// The sketch the solver leaves with the point at the target: drawn while dragging.
    pub preview: Sketch,
}

/// The sketch with point `point` set to `to` and solved; `None` when the point does not exist or
/// the solver refuses.
#[must_use]
pub fn moved(sk: &Sketch, point: usize, to: [f64; 2]) -> Option<Sketch> {
    let mut next = sk.clone();
    *next.points.get_mut(point)? = to;
    solve(&mut next).ok()?;
    Some(next)
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
        if self.tool != Tool::Select || self.target.is_none() || self.drag.is_some() || point >= sk.points.len() {
            return false;
        }
        self.drag = Some(Drag { point, start: sk.clone(), target: at, preview: sk.clone() });
        self.drag_update(at)
    }

    /// Moves the dragged point's target to `to` and recomputes the preview from the start.
    pub fn drag_update(&mut self, to: [f64; 2]) -> bool {
        let Some(d) = self.drag.as_mut() else { return false };
        d.target = to;
        d.preview = moved(&d.start, d.point, to).unwrap_or_else(|| d.start.clone());
        true
    }

    /// Ends the drag: the one command that moves the point, or none if nothing would move.
    pub fn drag_end(&mut self) -> Vec<Cmd> {
        let Some(d) = self.drag.take() else { return Vec::new() };
        let Some(id) = self.target else { return Vec::new() };
        match moved(&d.start, d.point, d.target) {
            Some(after) if changed(&d.start, &after) => {
                vec![Cmd { id: "sketch.move_point", params: json!({"sketch": id.0, "point": d.point, "to": d.target}) }]
            }
            _ => Vec::new(),
        }
    }

    /// Drops the drag in progress, if any, without a command.
    pub fn drag_cancel(&mut self) {
        self.drag = None;
    }
}
