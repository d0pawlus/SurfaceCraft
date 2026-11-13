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
use crate::nurbs::{SurfaceProbe, trimmed_sheet};
use crate::record::brep::{Brep, Face, Surface};
use crate::vpatch;
use crate::{KernelError, Result};
use geom::{V2, thin_indices};
use rows::{Inputs, Layout, SideSamples, Zone};

mod geom;
mod measure;
mod reference;
mod rows;
mod start;
#[cfg(test)]
mod tests;

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
    /// Points the patch should pass near, with a weight relative to a data point's 1.
    pub soft_points: Vec<([f64; 3], f64)>,
    /// Interior curves the patch must contain (hard G0).
    pub curves: Vec<Vec<[f64; 3]>>,
    /// Target of the data deviation (max), mm.
    pub tolerance: f64,
    /// Relative stiffness of the bending energy against the data (default 1e-3).
    pub smoothing: Option<f64>,
    /// Start control net; default from the number of data points.
    pub control: Option<[usize; 2]>,
    /// Most control points per direction, at most [`vpatch::MAX_CONTROL`].
    pub max_control: usize,
    /// What to do at incompatible corners.
    pub conflicts: Conflicts,
}

/// How one side fared, measured on dense samples (at least 20 per knot interval).
#[derive(Clone, Debug)]
pub struct SideReport {
    /// The continuity asked for.
    pub continuity: Continuity,
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
    /// The data's largest distance is within the request's tolerance.
    pub within_tolerance: bool,
    /// Every side within its targets, nothing folded, no corner relaxed.
    pub within_targets: bool,
    /// One entry per side.
    pub sides: Vec<SideReport>,
    /// Corners that were relaxed.
    pub corners: Vec<CornerReport>,
    /// Hard scalar rows found dependent and dropped by the last solve.
    pub dropped: usize,
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
const DEFAULT_SMOOTHING: f64 = 1e-3;
/// Share of a side's length around a conflicting corner where its G1/G2 rows are relaxed.
const RELAX_SPAN: f64 = 0.15;
/// A round must lower the worst miss by this share to be followed by another.
const MIN_IMPROVEMENT: f64 = 0.05;
/// Corners of the trimming outline (the trimmer takes 8000).
const MAX_TRIM_CORNERS: usize = 6_000;

const WARN_NO_DATA: &str = "no data: the interior is shaped by the boundary and the bending energy";
const WARN_MAX_CONTROL: &str = "the control net reached max_control";
const WARN_FEW_DATA: &str = "too few data points for the control net";

/// One solved and measured round.
struct Round {
    surface: Surface,
    measure: measure::Measure,
    net: [usize; 2],
    dropped: usize,
    ignored: usize,
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
            out[d] = (out[d] * 3).div_ceil(2).clamp(out[d] + 1, max);
            if !both {
                break;
            }
        }
    }
    out
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
/// reference normal more than 75° off the outline's plane, incompatible corners with
/// [`Conflicts::Fail`], hard points outside the parameter domain; and a kernel error where the
/// solve or the trimming fails.
pub fn fit_patch(req: &Request) -> Result<Patch> {
    let start::Prepared { lp, mut interior, corners } = start::prepare(req)?;

    // corners whose tangent planes disagree: refuse, or relax the rows near them
    let conflicts: Vec<start::Corner> =
        corners.iter().filter(|c| c.mismatch_deg > CORNER_MISMATCH_DEG).copied().collect();
    let mut zones: Vec<Zone> = Vec::new();
    let mut corner_reports: Vec<CornerReport> = Vec::new();
    if !conflicts.is_empty() && req.conflicts == Conflicts::Fail {
        return Err(KernelError::Argument(format!(
            "the tangent planes of neighbouring sides disagree at {} corner(s) (limit {CORNER_MISMATCH_DEG}°): {}",
            conflicts.len(),
            conflict_list(&conflicts)
        )));
    }
    for c in &conflicts {
        let len = |k: usize| lp.sides.get(k).map_or(0.0, |s| s.length());
        let (zb, za) = (RELAX_SPAN * len(c.before), RELAX_SPAN * len(c.after));
        zones.push(Zone { side: c.before, at_end: true, length: zb });
        zones.push(Zone { side: c.after, at_end: false, length: za });
        corner_reports.push(CornerReport { at: c.at, mismatch_deg: c.mismatch_deg, relaxed_length: zb + za });
    }

    // start net and the fixed parameter domain: the loop's box grown by one control spacing
    let max = req.max_control;
    let n_data = interior.data.len();
    let start_net = req.control.unwrap_or_else(|| {
        let k = ((n_data as f64 / 12.0).sqrt().round() as usize).max(6);
        let k = std::cmp::min(max, k);
        [k, k]
    });
    let margin = |d: usize| (lp.bbox[d][1] - lp.bbox[d][0]) / (start_net[d] as f64 - 1.0);
    let domain = [
        [lp.bbox[0][0] - margin(0), lp.bbox[0][1] + margin(0)],
        [lp.bbox[1][0] - margin(1), lp.bbox[1][1] + margin(1)],
    ];
    start::check_hard_inside(&interior, &domain)?;
    let smoothing = req.smoothing.unwrap_or(DEFAULT_SMOOTHING);

    let mut net = start_net;
    let mut best: Option<Round> = None;
    let mut cache: Option<(Layout, Vec<SideSamples>)> = None;
    let mut prev: Option<Surface> = None;
    let mut prev_worst = f64::INFINITY;
    let mut rounds = 0u32;
    for round in 1..=MAX_ROUNDS {
        rounds = round;
        let layout = Layout::new(domain, net);
        if cache.as_ref().is_none_or(|(l, _)| l.net != net) {
            cache = Some((layout, rows::sample_sides(&lp, &layout)?));
        }
        let Some((_, samples)) = cache.as_ref() else { break };
        let built = rows::build_problem(&Inputs {
            lp: &lp,
            layout: &layout,
            sides: samples,
            interior: &interior,
            zones: &zones,
            prev: prev.as_ref(),
            smoothing,
        })?;
        let solution = vpatch::solve(&built.problem)?;
        let probe = SurfaceProbe::new(&solution.surface)?;
        let m = measure::measure(&solution.surface, &probe, &lp, &layout, samples, &mut interior, req.tolerance)?;

        let done = m.sides_ok && (m.points == 0 || m.max <= req.tolerance);
        let worst = m.worst;
        let miss = m.miss;
        let improved = round == 1 || prev_worst - worst >= MIN_IMPROVEMENT * prev_worst;
        if best.as_ref().is_none_or(|b| worst < b.measure.worst) {
            best = Some(Round {
                surface: solution.surface.clone(),
                measure: m,
                net,
                dropped: solution.dropped,
                ignored: built.ignored,
            });
        }
        if done || !improved || round == MAX_ROUNDS {
            break;
        }
        prev_worst = worst;
        prev = Some(solution.surface);
        net = grow(net, miss, max);
    }
    let Some(best) = best else {
        return Err(KernelError::Argument("the patch could not be fitted".into()));
    };

    // trimming outline: the closest points of the dense side samples on the final patch
    let mut outer: Vec<V2> = Vec::new();
    let cap = (MAX_TRIM_CORNERS / lp.sides.len().max(1)).max(8);
    for feet in &best.measure.feet {
        let body = &feet[..feet.len().saturating_sub(1)];
        outer.extend(thin_indices(body.len(), cap).into_iter().filter_map(|i| body.get(i).copied()));
    }
    let sheet = trimmed_sheet(&best.surface, &outer, &[])?;

    let m = &best.measure;
    let within_tolerance = m.points == 0 || m.max <= req.tolerance;
    let within_targets = m.sides_ok && corner_reports.is_empty();
    let mut warnings = Vec::new();
    if interior.data.is_empty() {
        warnings.push(WARN_NO_DATA.to_string());
    } else if interior.data.len() < 3 * best.net[0] * best.net[1] {
        warnings.push(WARN_FEW_DATA.to_string());
    }
    if (best.net[0] >= max || best.net[1] >= max) && (best.net != start_net || !(within_tolerance && within_targets)) {
        warnings.push(WARN_MAX_CONTROL.to_string());
    }
    if best.ignored > 0 {
        warnings.push(format!("{} data points lie outside the parameter domain and were ignored", best.ignored));
    }
    if m.hard_max > G0_TARGET {
        warnings.push(format!("interior points or curves miss the patch by up to {:.4} mm", m.hard_max));
    }
    let report = Report {
        control: best.net,
        rounds,
        rms: m.rms,
        p99: m.p99,
        max: m.max,
        points: m.points,
        within_tolerance,
        within_targets,
        sides: m.sides.clone(),
        corners: corner_reports,
        dropped: best.dropped,
        warnings,
    };
    Ok(Patch { surface: best.surface, sheet, report })
}
