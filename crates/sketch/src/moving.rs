//! Moving one sketch point under the constraints: the move the mouse drag and `sketch.move_point`
//! share, so the preview is the committed result.
use crate::{Sketch, SketchError, SolveReport, solve};

/// Waypoints of the fallback path from the old position to the target (the last one is the
/// target, tried again from the last valid state after the walk).
const WAYPOINTS: usize = 8;

/// Distance (sketch units) within which the point counts as having reached the target.
pub const REACH_TOL: f64 = 1e-6;

/// A solved move.
#[derive(Clone, Debug, PartialEq)]
pub struct Moved {
    /// The sketch after the move, every constraint satisfied.
    pub sketch: Sketch,
    /// The solve of that sketch.
    pub report: SolveReport,
    /// Where the moved point ended up.
    pub position: [f64; 2],
    /// Distance from `position` to the requested target.
    pub remaining: f64,
}

impl Moved {
    /// Did the point reach the target (within [`REACH_TOL`])? A point the constraints fix, or a
    /// target they cannot hold, does not.
    #[must_use]
    pub fn reached(&self) -> bool {
        self.remaining <= REACH_TOL
    }
}

/// The sketch with `point` set to `to` and solved, when the constraints hold afterwards.
fn attempt(sk: &Sketch, point: usize, to: [f64; 2]) -> Result<Moved, SketchError> {
    let mut next = sk.clone();
    let slot = next.points.get_mut(point).ok_or_else(|| SketchError::Index(format!("point {point} does not exist")))?;
    *slot = to;
    let report = solve(&mut next)?;
    if !report.is_satisfied() {
        return Err(SketchError::Value("the constraints cannot be satisfied with the point there".into()));
    }
    let position = next.points.get(point).copied().unwrap_or(to);
    Ok(Moved { sketch: next, report, position, remaining: 0.0 })
}

fn lerp(a: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

/// The attempt with the distance left to the requested target filled in.
fn measured(attempt: Moved, target: [f64; 2]) -> Moved {
    let remaining = (attempt.position[0] - target[0]).hypot(attempt.position[1] - target[1]);
    Moved { remaining, ..attempt }
}

/// The error of a move that failed: a sketch whose constraints already fail before the move says
/// so (and what to do), any other failure is the move's own.
fn failure(sk: &Sketch, point: usize, own: SketchError) -> SketchError {
    let here = sk.points.get(point).copied().unwrap_or([0.0, 0.0]);
    match attempt(sk, point, here) {
        Err(SketchError::Value(_)) => SketchError::Value(
            "the sketch's constraints are already unsatisfied: edit or remove the conflicting constraint, then move the point"
                .into(),
        ),
        _ => own,
    }
}

/// Moves `point` to `to` and solves: the constraints pull the rest of the sketch along, and a
/// point they fix goes back to where they put it.
///
/// The result always satisfies the constraints (redundant but satisfied ones are fine). When the
/// target itself is a position the solver cannot leave (a line collapsed to a point has zero
/// derivatives, so a length cannot pull it open), the point walks to the target in small steps,
/// the target is tried once more from the last valid state, and otherwise that state is kept: a
/// drag through such a position gives the nearest valid sketch. [`Moved::reached`] says whether
/// the point is at the target. Fails when not even the first step can be satisfied.
///
/// # Errors
/// The point does not exist, the sketch is malformed, or the constraints cannot be satisfied.
pub fn move_point(sk: &Sketch, point: usize, to: [f64; 2]) -> Result<Moved, SketchError> {
    let first = match attempt(sk, point, to) {
        Ok(done) => return Ok(measured(done, to)),
        Err(e) => e,
    };
    let from = *sk.points.get(point).ok_or(first.clone())?;
    let mut last: Option<Moved> = None;
    for k in 1..WAYPOINTS {
        let base = last.as_ref().map_or(sk, |m| &m.sketch);
        match attempt(base, point, lerp(from, to, k as f64 / WAYPOINTS as f64)) {
            Ok(step) => last = Some(step),
            Err(_) => break,
        }
    }
    let Some(walked) = last else { return Err(failure(sk, point, first)) };
    Ok(measured(attempt(&walked.sketch, point, to).unwrap_or(walked), to))
}
