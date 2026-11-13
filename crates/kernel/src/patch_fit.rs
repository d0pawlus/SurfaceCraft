//! Driver of `surface.patch`: one trimmed bicubic B-spline patch over a closed loop of sides, with
//! G0/G1/G2 to neighbouring faces, interior points and curves, and scan data (design doc
//! `2026-11-13-surface-patch-design.md` §4.2 to §4.5).
//!
//! The driver orchestrates and measures; the numbers are done elsewhere: [`crate::vpatch`] solves
//! the constrained variational problem and evaluates the patch, [`crate::nurbs`] projects onto it
//! and builds the trimmed sheet, solidcore evaluates the reference faces. A round states the
//! conditions (hard rows at collocation points, soft rows between them), solves, re-projects the
//! interior points and measures densely; the net grows where the misses are until every target holds,
//! the worst miss stops improving, or six rounds are done.
use crate::nurbs::trimmed_sheet;
use crate::record::brep::{Brep, Face, Surface};
use crate::{KernelError, Result, poll};
use geom::{V2, thin_indices};
use rows::Zone;
use surfacecraft_geom::Cancel;

mod curve;
mod geom;
mod measure;
mod reference;
mod report;
mod rounds;
mod rows;
mod start;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_cost;
#[cfg(test)]
mod tests_g2;
#[cfg(test)]
mod tests_points;
#[cfg(test)]
mod tests_report;
#[cfg(test)]
mod tests_rounds;
#[cfg(test)]
mod tests_soft;
#[cfg(test)]
mod tests_strict;

pub use reference::face_normal_curvature;

/// Continuity to the neighbouring face along a side.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Continuity {
    /// Positional.
    G0,
    /// Tangent plane.
    G1,
    /// Tangent plane and normal curvature.
    G2,
}

/// What to do where two sides ask for tangent planes that do not agree at their corner.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Conflicts {
    /// Relax the G1/G2 rows near the corner (the report lists it).
    Relax,
    /// Refuse.
    Fail,
}

/// One side of the loop, running from its first point to its last (which is the next side's first).
#[derive(Clone, Debug)]
pub struct Side {
    /// Dense polyline of the side, at least 2 points.
    pub points: Vec<[f64; 3]>,
    /// Continuity to the reference face.
    pub continuity: Continuity,
    /// The face the patch must continue (G1/G2): its surface, as stored in the body record.
    pub reference: Option<Face>,
    /// Parameters of `points` when the caller has a flattening (same length as `points`).
    pub uv: Option<Vec<[f64; 2]>>,
    /// The side is approximate (traced on a scan): the patch follows the data and comes as close to
    /// the side as the data allow; its gap is reported but is not a target.
    pub soft: bool,
}

/// What to fit.
#[derive(Clone, Debug)]
pub struct Request {
    /// Ordered sides, closed (each ends where the next starts, within 1e-7 of the loop's size).
    pub sides: Vec<Side>,
    /// Scan data (soft), thinned deterministically above 50 000 points.
    pub data: Vec<[f64; 3]>,
    /// Parameters of `data` with a flattening of the sides.
    pub data_uv: Option<Vec<[f64; 2]>>,
    /// Points the patch must pass through.
    pub hard_points: Vec<[f64; 3]>,
    /// Points the patch should pass near, with a weight relative to a data point's 1 (at most 2000).
    /// They are soft position conditions, not scan data: they never count toward the minimum number
    /// of data points, are never thinned, and a point outside the outline is an error.
    pub soft_points: Vec<([f64; 3], f64)>,
    /// Interior curves the patch must contain (hard G0).
    pub curves: Vec<Vec<[f64; 3]>>,
    /// Target of the data deviation (max), mm.
    pub tolerance: f64,
    /// Relative stiffness of the bending energy against the data (default 1e-3).
    pub smoothing: Option<f64>,
    /// Start control net; default from the number of data points.
    pub control: Option<[usize; 2]>,
    /// Most control points per direction, at most [`crate::vpatch::MAX_CONTROL`].
    pub max_control: usize,
    /// What to do at incompatible corners.
    pub conflicts: Conflicts,
    /// The caller's number of every side, named in messages; empty: the position in `sides`.
    pub side_ids: Vec<usize>,
}

/// How one side fared, measured on dense samples (at least 20 per knot interval).
#[derive(Clone, Debug)]
pub struct SideReport {
    /// The continuity asked for.
    pub continuity: Continuity,
    /// The side was soft: its numbers are reported, not targets.
    pub soft: bool,
    /// Largest distance from a side sample to the patch, mm.
    pub gap_max: f64,
    /// Largest angle between the reference normal and the patch normal, degrees (G1/G2).
    pub angle_max_deg: Option<f64>,
    /// Largest normal curvature error against the reference, relative (G2).
    pub curvature_error_max: Option<f64>,
    /// Samples measured.
    pub samples: usize,
    /// The patch folds back over the reference face.
    pub folded: bool,
    /// Largest distance a point of the side was moved onto its reference face (G1/G2 sides), mm.
    pub snap_max: Option<f64>,
    /// The side meets its targets: gap, and for G1/G2 angle, curvature and no fold. Always true for
    /// a soft side, which has none.
    pub met: bool,
}

/// A corner whose sides ask for tangent planes that disagree.
#[derive(Clone, Debug)]
pub struct CornerReport {
    /// Where.
    pub at: [f64; 3],
    /// Angle between the two tangent planes, degrees.
    pub mismatch_deg: f64,
    /// Length of the side stretches (both sides together) whose G1/G2 rows were relaxed, mm.
    pub relaxed_length: f64,
}

/// What the fit achieved.
#[derive(Clone, Debug)]
pub struct Report {
    /// Final control net.
    pub control: [usize; 2],
    /// Rounds run.
    pub rounds: u32,
    /// Root-mean-square distance of the data from the patch, mm.
    pub rms: f64,
    /// 99th percentile of that distance, mm.
    pub p99: f64,
    /// Largest of that distance, mm.
    pub max: f64,
    /// Data points measured.
    pub points: usize,
    /// Data points that lie outside the outline and were ignored.
    pub outside: usize,
    /// The data's largest distance is within the request's tolerance.
    pub within_tolerance: bool,
    /// Every hard side meets every target (gap, angle, curvature, not folded), the interior points
    /// and curves lie on the patch, the patch does not fold inside the outline, the hard conditions
    /// do not contradict each other, the data are within the tolerance (`within_tolerance`) and no
    /// corner was relaxed. A side whose gap stopped improving is not excused: it is reported with a
    /// warning instead.
    pub within_targets: bool,
    /// The patch folds: back over a reference face along a side, or over itself inside the outline.
    pub folded: bool,
    /// One entry per side.
    pub sides: Vec<SideReport>,
    /// Corners that were relaxed.
    pub corners: Vec<CornerReport>,
    /// Largest distance of a soft point from the patch, mm (`None` without soft points).
    pub soft_points_max: Option<f64>,
    /// Hard scalar rows found dependent and dropped by the last solve.
    pub dropped: usize,
    /// Largest miss of a hard row after the solve, as the control point motion that would satisfy it
    /// (mm): about 0 for consistent rows, the size of the contradiction otherwise.
    pub hard_residual: f64,
    /// The net stopped growing because the solve of the next net exceeds the cost bound
    /// ([`crate::vpatch::MAX_SOLVE_SECONDS`]); the report is that of the net reached.
    pub cost_limited: bool,
    /// Things the caller should know.
    pub warnings: Vec<String>,
}

/// The fitted patch, its trimmed sheet and the report.
#[derive(Clone, Debug)]
pub struct Patch {
    /// The untrimmed bicubic surface.
    pub surface: Surface,
    /// A sheet of one face on `surface`, trimmed to the loop.
    pub sheet: Brep,
    /// What was achieved.
    pub report: Report,
}

/// Target of the gap along a side, mm (measured densely).
pub const G0_TARGET: f64 = 1e-3;
/// Target of the angle between normals along a G1/G2 side, degrees.
pub const G1_TARGET_DEG: f64 = 0.05;
/// Target of the relative normal curvature error along a G2 side.
pub const G2_TARGET_REL: f64 = 0.05;
/// Corners whose tangent planes differ by more than this (degrees) are in conflict.
pub const CORNER_MISMATCH_DEG: f64 = 0.5;

const MAX_ROUNDS: u32 = 6;
/// A hard row missed by more than this share of [`G0_TARGET`] (as control point motion, mm) means the
/// hard conditions contradict each other.
const HARD_RESIDUAL_SHARE: f64 = 0.1;
/// The default start net of a call with G1/G2 sides (coupled solve) stays at this size; growth goes on
/// within the cost bound of [`crate::vpatch::MAX_SOLVE_SECONDS`].
const COUPLED_START_MAX: usize = 16;
const DEFAULT_SMOOTHING: f64 = 1e-3;
/// Share of a side's length around a conflicting corner where its G1/G2 rows are relaxed.
const RELAX_SPAN: f64 = 0.15;
/// A round must lower the worst miss by this share to be followed by another; a side whose gap
/// the growth lowered by less is as close as its own accuracy allows.
const MIN_IMPROVEMENT: f64 = 0.05;
/// Growth never replaces the kept round by one whose data deviation is this share larger.
const MAX_DATA_GROWTH: f64 = 0.10;
/// Corners of the trimming outline (the trimmer takes 8000).
const MAX_TRIM_CORNERS: usize = 6_000;

const WARN_NO_DATA: &str = "no data: the interior is shaped by the boundary and the bending energy";
const WARN_MAX_CONTROL: &str = "the control net reached max_control";
const WARN_FEW_DATA: &str = "too few data points for the control net";
const WARN_TRIM_START: &str = "the trim outline uses the start parameters of the sides";

/// What decides which round is kept.
#[derive(Clone, Copy, Debug)]
struct Summary {
    /// The data's largest distance is within the tolerance (or there are no data).
    data_ok: bool,
    /// The data's largest distance, mm.
    data_max: f64,
    /// Worst miss of the hard conditions in units of their targets (at most 1 when all are met).
    side_miss: f64,
}

/// One solved and measured round.
struct Round {
    surface: Surface,
    measure: measure::Measure,
    net: [usize; 2],
    dropped: usize,
    hard_residual: f64,
    summary: Summary,
}

/// Does `new` replace the kept round `kept`? A round within the data tolerance beats one outside
/// it. Among rounds on the same side of the tolerance the smaller miss of the hard conditions wins
/// (rounds that meet them all tie and the smaller data deviation wins), but a round whose data
/// deviation is more than [`MAX_DATA_GROWTH`] larger never does, unless it brings the hard
/// conditions from missed to met.
fn prefer(new: &Summary, kept: &Summary) -> bool {
    if new.data_ok != kept.data_ok {
        return new.data_ok;
    }
    let brings_met = kept.side_miss > 1.0 && new.side_miss <= 1.0;
    if new.data_max > (1.0 + MAX_DATA_GROWTH) * kept.data_max && !brings_met {
        return false;
    }
    let (n, k) = (new.side_miss.max(1.0), kept.side_miss.max(1.0));
    if n == k { new.data_max < kept.data_max } else { n < k }
}

/// A round as the tests see it.
#[cfg(test)]
#[derive(Clone, Debug)]
pub(crate) struct RoundTrace {
    /// Control net of the round.
    pub net: [usize; 2],
    /// Root-mean-square data distance, mm.
    pub rms: f64,
    /// Largest data distance, mm.
    pub max: f64,
    /// Largest gap of a hard side, mm.
    pub gap_max: f64,
    /// Wall-clock seconds of the solve.
    pub solve_secs: f64,
    /// Hard scalar rows of the solve.
    pub hard_rows: usize,
}

/// One direction's size after growing by 1.5×, capped.
fn grown(n: usize, max: usize) -> usize {
    (n * 3).div_ceil(2).max(n + 1).min(max)
}

/// The net after a round that missed: the direction(s) with the larger miss grow by 1.5×, capped.
fn grow(net: [usize; 2], miss: [f64; 2], max: usize) -> [usize; 2] {
    let both = (miss[0] - miss[1]).abs() <= 0.05 * miss[0].max(miss[1]);
    let mut order = [0usize, 1];
    if miss[1] > miss[0] {
        order = [1, 0];
    }
    let mut out = net;
    for d in order {
        if out[d] < max {
            out[d] = grown(out[d], max);
            if !both {
                break;
            }
        }
    }
    out
}

/// The net after a round whose growth (`from` to `net`) did not help: the directions that did
/// not grow in that step grow now. Unchanged when every direction grew or is at `max`.
fn grow_other(net: [usize; 2], from: [usize; 2], max: usize) -> [usize; 2] {
    let mut out = net;
    for d in 0..2 {
        if out[d] == from[d] && out[d] < max {
            out[d] = grown(out[d], max);
        }
    }
    out
}

/// The trimming outline: corners from the dense side samples, thinned. A hard side takes the closest
/// points of its samples on the patch (`own` false), a soft side (or every side, `own` true) the
/// samples' own parameters, which come from the flattening and cannot cross.
fn trim_outline(lp: &start::Loop, m: &measure::Measure, own: bool) -> Vec<V2> {
    let mut outer: Vec<V2> = Vec::new();
    let cap = (MAX_TRIM_CORNERS / lp.sides.len().max(1)).max(8);
    for (k, side) in lp.sides.iter().enumerate() {
        let source = if own || side.soft { m.starts.get(k) } else { m.feet.get(k) };
        let Some(pts) = source else { continue };
        let body = pts.get(..pts.len().saturating_sub(1)).unwrap_or_default();
        outer.extend(thin_indices(body.len(), cap).into_iter().filter_map(|i| body.get(i).copied()));
    }
    outer
}

fn conflict_list(corners: &[start::Corner]) -> String {
    corners
        .iter()
        .map(|c| format!("({:.4}, {:.4}, {:.4}): {:.2}°", c.at[0], c.at[1], c.at[2], c.mismatch_deg))
        .collect::<Vec<_>>()
        .join("; ")
}

/// Fits one trimmed bicubic patch to the loop of sides under the request's conditions.
///
/// Errors (`KernelError::Argument`) name the reason: non-finite or oversized input, an open loop,
/// G1/G2 without a reference face, an outline that is not a height field (give a flattening), a
/// reference normal more than 75° off the outline's plane, a G1/G2 side more than 0.1 mm off its
/// reference face (its points are first moved onto the face), incompatible corners with
/// [`Conflicts::Fail`], hard points outside the parameter domain; and a kernel error where the
/// solve or the trimming fails. Data outside the outline are ignored and counted; soft sides
/// ([`Side::soft`]) are followed as far as the data allow and do not steer the fit's targets.
/// The points of a hard side of three or more points are samples of a smooth curve: its dense
/// samples follow a cubic interpolant through them, not the chords (a warning names a side that
/// could not be interpolated). A round is kept by `prefer`; a side whose gap a growth step along it
/// no longer lowers is reported (with a warning) as being as close as its own accuracy allows, but
/// is not excused from `within_targets`. G2 rows are linearised about the previous round, so with
/// a G2 side a net that cannot grow still gets a second round, and more while the curvature error
/// falls by 5 %. Soft points are soft position rows that need an outline to lie inside (`Err`
/// otherwise); the largest distance of one from the patch is reported. When the trimmer refuses
/// the outline of closest points, it is tried with the start parameters of the sides (a warning).
pub fn fit_patch(req: &Request) -> Result<Patch> {
    fit_patch_cancellable(req, &Cancel::new())
}

/// [`fit_patch`] that gives up with [`KernelError::Cancelled`] when `cancel` is set: checked between
/// the stages, every round, and inside the loops over the samples, the rows and the data, so a stop
/// waits for one dense factorisation at most. Nothing is produced for a cancelled call.
///
/// The cost of a round is bounded: a round whose solve is estimated above
/// [`crate::vpatch::MAX_SOLVE_SECONDS`] (see [`crate::vpatch::solve_seconds`]) is not run. A net that would have to
/// grow past it stops at the net reached (`Report::cost_limited`, a warning); a start net over the
/// bound is an error.
pub fn fit_patch_cancellable(req: &Request, cancel: &Cancel) -> Result<Patch> {
    fit(
        req,
        cancel,
        #[cfg(test)]
        &mut Vec::new(),
    )
}

/// [`fit_patch`] with what every round measured.
#[cfg(test)]
pub(crate) fn fit_patch_rounds(req: &Request) -> Result<(Patch, Vec<RoundTrace>)> {
    let mut trace = Vec::new();
    let patch = fit(req, &Cancel::new(), &mut trace)?;
    Ok((patch, trace))
}

/// The corners whose tangent planes disagree: refused, or the zones where the rows are relaxed.
fn corner_zones(req: &Request, lp: &start::Loop, corners: &[start::Corner]) -> Result<(Vec<Zone>, Vec<CornerReport>)> {
    let conflicts: Vec<start::Corner> =
        corners.iter().filter(|c| c.mismatch_deg > CORNER_MISMATCH_DEG).copied().collect();
    if !conflicts.is_empty() && req.conflicts == Conflicts::Fail {
        return Err(KernelError::Argument(format!(
            "the tangent planes of neighbouring sides disagree at {} corner(s) (limit {CORNER_MISMATCH_DEG}°): {}",
            conflicts.len(),
            conflict_list(&conflicts)
        )));
    }
    let mut zones: Vec<Zone> = Vec::new();
    let mut reports: Vec<CornerReport> = Vec::new();
    for c in &conflicts {
        let len = |k: usize| lp.sides.get(k).map_or(0.0, |s| s.length());
        let (zb, za) = (RELAX_SPAN * len(c.before), RELAX_SPAN * len(c.after));
        zones.push(Zone { side: c.before, at_end: true, length: zb });
        zones.push(Zone { side: c.after, at_end: false, length: za });
        reports.push(CornerReport { at: c.at, mismatch_deg: c.mismatch_deg, relaxed_length: zb + za });
    }
    Ok((zones, reports))
}

/// The start net and the fixed parameter domain: the loop's box grown by one control spacing.
fn start_net_and_domain(req: &Request, lp: &start::Loop, interior: &start::Interior) -> ([usize; 2], [[f64; 2]; 2]) {
    let max = req.max_control;
    let n_data = interior.data.iter().filter(|p| !p.out).count();
    let coupled = lp.sides.iter().any(|s| s.continuity != Continuity::G0);
    let start_net = req.control.unwrap_or_else(|| {
        let k = ((n_data as f64 / 12.0).sqrt().round() as usize).max(6);
        let k = std::cmp::min(max, k);
        let k = if coupled { k.min(COUPLED_START_MAX) } else { k };
        [k, k]
    });
    let margin = |d: usize| (lp.bbox[d][1] - lp.bbox[d][0]) / (start_net[d] as f64 - 1.0);
    let domain = [
        [lp.bbox[0][0] - margin(0), lp.bbox[0][1] + margin(0)],
        [lp.bbox[1][0] - margin(1), lp.bbox[1][1] + margin(1)],
    ];
    (start_net, domain)
}

/// The trimmed sheet: the outline of the closest points of the dense samples of the hard sides on
/// the final patch, the samples' own parameters for soft sides (which may lie off the patch, where
/// the feet clamp or cross); when the trimmer refuses it, once more with the own parameters of every
/// side. The flag says the fallback was used.
fn trim(lp: &start::Loop, best: &Round) -> Result<(Brep, bool)> {
    let m = &best.measure;
    match trimmed_sheet(&best.surface, &trim_outline(lp, m, false), &[]) {
        Ok(sheet) => Ok((sheet, false)),
        Err(first) if lp.sides.iter().any(|s| !s.soft) => {
            let sheet = trimmed_sheet(&best.surface, &trim_outline(lp, m, true), &[]).map_err(|_| first)?;
            Ok((sheet, true))
        }
        Err(first) => Err(first),
    }
}

fn fit(req: &Request, cancel: &Cancel, #[cfg(test)] trace: &mut Vec<RoundTrace>) -> Result<Patch> {
    let start::Prepared { lp, mut interior, corners, warnings: notes } = start::prepare(req, cancel)?;
    let (zones, corner_reports) = corner_zones(req, &lp, &corners)?;
    let (start_net, domain) = start_net_and_domain(req, &lp, &interior);
    start::check_hard_inside(&interior, &domain)?;
    let cx = rounds::Context {
        req,
        lp: &lp,
        zones: &zones,
        domain,
        start_net,
        smoothing: req.smoothing.unwrap_or(DEFAULT_SMOOTHING),
        cancel,
    };
    let outcome = cx.run(
        &mut interior,
        #[cfg(test)]
        trace,
    )?;
    poll(cancel)?;
    let (sheet, trim_fallback) = trim(&lp, &outcome.best)?;
    let report = report::build(report::Inputs {
        lp: &lp,
        outcome: &outcome,
        tolerance: req.tolerance,
        max_control: req.max_control,
        start_net,
        corners: corner_reports,
        notes,
        trim_fallback,
    });
    Ok(Patch { surface: outcome.best.surface, sheet, report })
}
