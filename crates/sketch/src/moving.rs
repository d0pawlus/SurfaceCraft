//! Moving one sketch point under the constraints: the move the mouse drag and `sketch.move_point`
//! share, so the preview is the committed result.
use crate::{Sketch, SketchError, SolveReport, solve};

/// Waypoints of the fallback path from the old position to the target (the last one is the
/// target, which the direct attempt already tried).
const WAYPOINTS: usize = 8;

/// The sketch with `point` set to `to` and solved, when the constraints hold afterwards.
fn attempt(sk: &Sketch, point: usize, to: [f64; 2]) -> Result<(Sketch, SolveReport), SketchError> {
    let mut next = sk.clone();
    let slot = next.points.get_mut(point).ok_or_else(|| SketchError::Index(format!("point {point} does not exist")))?;
    *slot = to;
    let report = solve(&mut next)?;
    if report.is_satisfied() {
        Ok((next, report))
    } else {
        Err(SketchError::Value("the constraints cannot be satisfied with the point there".into()))
    }
}

fn lerp(a: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

/// Moves `point` to `to` and solves: the constraints pull the rest of the sketch along, and a
/// point they fix goes back to where they put it.
///
/// The result always satisfies the constraints (redundant but satisfied ones are fine). When the
/// target itself is a position the solver cannot leave (a line collapsed to a point has zero
/// derivatives, so a length cannot pull it open), the point walks to the target in small steps
/// and the last satisfied state is kept: a drag through such a position gives the nearest valid
/// sketch. Fails when not even the first step can be satisfied.
///
/// # Errors
/// The point does not exist, the sketch is malformed, or the constraints cannot be satisfied.
pub fn move_point(sk: &Sketch, point: usize, to: [f64; 2]) -> Result<(Sketch, SolveReport), SketchError> {
    let first = match attempt(sk, point, to) {
        Ok(done) => return Ok(done),
        Err(e) => e,
    };
    let from = *sk.points.get(point).ok_or(first.clone())?;
    let mut last: Option<(Sketch, SolveReport)> = None;
    for k in 1..WAYPOINTS {
        let base = last.as_ref().map_or(sk, |(s, _)| s);
        match attempt(base, point, lerp(from, to, k as f64 / WAYPOINTS as f64)) {
            Ok(step) => last = Some(step),
            Err(_) => break,
        }
    }
    last.ok_or(first)
}
