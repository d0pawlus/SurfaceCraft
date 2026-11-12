//! The rounds of `surface.wrap`: how the coarse mesh the patches are laid on is refined until the
//! network is within the tolerance of the scan.
//!
//! Two layouts. The *uniform* one halves the budget of the whole scan each round. The *adaptive*
//! one keeps the budget where the patches already fit and halves it only around the scan points
//! the patches miss (per-vertex detail levels, [`reduce_to_tolerance_scaled`]).
//!
//! Both refine the coarse *triangle mesh*, never single patches: every round lays the patches on a
//! fresh, manifold, conforming mesh (`fit::patch_g1::wrap_patches_g1`), so neighbouring patches
//! always share a whole boundary curve and no T-junction can open a gap, whatever the refinement.
//!
//! The layout is searched with unconstrained patches, which follow the scan best; the best layout is
//! then wrapped once more with patches that are tangent across the smooth edges, and the search is
//! repeated with tangent patches only when those lose the tolerance.
use super::wrap::WrapDeviation;
use super::{CmdError, CmdResult};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use surfacecraft_fit::mls::ScanSurface;
use surfacecraft_fit::patch::{PatchSet, tessellate};
use surfacecraft_fit::patch_g1::{G1Options, wrap_patches_g1};
use surfacecraft_geom::edit::{DecimateOptions, decimate, remove_non_manifold};
use surfacecraft_geom::reduce::{ToleranceOptions, raise_levels, reduce_to_tolerance, reduce_to_tolerance_scaled};
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3};

/// Patches per coarse triangle.
const PATCHES_PER_TRIANGLE: usize = 3;
/// The coarse mesh may leave the scan by this many tolerances in the first round.
const FIRST_BUDGET: f64 = 4.0;
/// The adaptive layout starts coarser (in tolerances): a patch follows a smooth surface much better
/// than a flat triangle, and the rounds refine only where it does not.
const ADAPTIVE_FIRST_BUDGET: f64 = 8.0;
/// Rounds at most of the uniform layout; each one halves the budget.
const UNIFORM_ROUNDS: usize = 3;
/// Rounds at most of the adaptive layout.
const ADAPTIVE_ROUNDS: usize = 7;
/// Most times the budget around a vertex is halved by the adaptive layout.
const MAX_LEVEL: u8 = 4;
/// Rings of scan vertices around a missed scan point that are refined with it.
const LEVEL_RINGS: usize = 2;
/// A patch point is looked for within this many budgets of its first guess.
const REACH_BUDGETS: f64 = 6.0;
/// Scan points measured at most (evenly strided).
const MAX_SAMPLES: usize = 200_000;
/// Times the budget is doubled to bring the coarse mesh under the patch limit.
const CAP_DOUBLINGS: usize = 4;
/// Tessellation of a patch for the measurement (quads per side).
const MEASURE_GRID: usize = 6;

/// How the coarse mesh is refined.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Refine {
    /// Detail only where the patches miss the tolerance.
    Adaptive,
    /// The budget of the whole scan is halved.
    Uniform,
}

impl Refine {
    pub(super) fn parse(name: Option<&str>) -> CmdResult<Self> {
        match name {
            None | Some("adaptive") => Ok(Self::Adaptive),
            Some("uniform") => Ok(Self::Uniform),
            Some(other) => {
                Err(CmdError::Params(format!("refine must be \"adaptive\" or \"uniform\", not \"{other}\"")))
            }
        }
    }

    pub(super) fn name(self) -> &'static str {
        match self {
            Self::Adaptive => "adaptive",
            Self::Uniform => "uniform",
        }
    }
}

/// What one refinement round produced.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct WrapLevel {
    /// Round, from 1.
    pub round: usize,
    /// Distance the coarse mesh was allowed from the scan where it is coarsest (mm).
    pub budget: f64,
    /// Triangles of the coarse mesh.
    pub coarse_triangles: usize,
    /// Patches.
    pub patches: usize,
    /// Share of the scan within the tolerance (0..1).
    pub within: f64,
    /// Scan vertices asked to keep more detail than in the first round (adaptive layout).
    pub refined_vertices: usize,
}

/// One round: the coarse mesh, its patches and their deviation.
pub(super) struct Round {
    pub(super) set: PatchSet,
    /// The coarse mesh the patches were laid on.
    coarse: Mesh,
    /// Layout edges of the patches that are creases.
    pub(super) creases: BTreeSet<(u32, u32)>,
    /// Coarse edges that are creases / smooth.
    pub(super) crease_edges: usize,
    pub(super) smooth_edges: usize,
    /// Patches built without the tangent construction (it folded them).
    pub(super) g0_patches: usize,
    pub(super) coarse_triangles: usize,
    pub(super) budget: f64,
    pub(super) capped: bool,
    pub(super) deviation: WrapDeviation,
}

/// The scan, its acceleration structure and the points the patches are measured at.
pub(super) struct Target<'a> {
    pub(super) scan: &'a Mesh,
    bvh: Bvh,
    surface: ScanSurface,
    options: G1Options,
    stride: usize,
    samples: Vec<Point3>,
    pub(super) tolerance: f64,
    pub(super) min_within: f64,
    max_triangles: usize,
}

impl<'a> Target<'a> {
    pub(super) fn new(scan: &'a Mesh, tolerance: f64, min_within: f64, max_patches: usize, options: G1Options) -> Self {
        let normals: Vec<Vec3> = scan.vertex_normals().into_iter().map(|n| n.try_normalize(0.0).unwrap_or(n)).collect();
        let stride = scan.positions.len().div_ceil(MAX_SAMPLES).max(1);
        Self {
            scan,
            bvh: Bvh::build(scan),
            surface: ScanSurface::oriented(&scan.positions, &normals),
            options,
            stride,
            samples: scan.positions.iter().step_by(stride).copied().collect(),
            tolerance,
            min_within,
            max_triangles: (max_patches / PATCHES_PER_TRIANGLE).max(4),
        }
    }
}

/// The result of all rounds.
pub(super) struct Wrapped {
    pub(super) best: Round,
    pub(super) rounds: usize,
    pub(super) levels: Vec<WrapLevel>,
}

fn decimate_options() -> DecimateOptions {
    DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true }
}

/// Brings `coarse` under `max_triangles`: the budget is doubled (the reduction is still measured
/// against the surface, so the shape survives) and only as a last resort the mesh is cut to the
/// count. The mesh is made manifold (a reduction can pinch the surface; the network needs at most
/// two triangles on an edge).
fn fit_to_limit(mut coarse: Mesh, budget: f64, max_triangles: usize) -> Mesh {
    let opts = decimate_options();
    let mut wider = budget;
    for _ in 0..CAP_DOUBLINGS {
        if coarse.triangle_count() <= max_triangles {
            break;
        }
        wider *= 2.0;
        let _report = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(wider));
    }
    if coarse.triangle_count() > max_triangles {
        decimate(&mut coarse, &DecimateOptions { target_triangles: max_triangles, ..opts });
    }
    remove_non_manifold(&mut coarse);
    coarse
}

/// The scan reduced within `budget`, more finely where `scale` is below 1. An invalid mesh is left
/// as it is; the caller has validated the scan.
fn reduce_scan(scan: &Mesh, budget: f64, scale: &[f64]) -> Mesh {
    let mut coarse = scan.clone();
    let _report = reduce_to_tolerance_scaled(&mut coarse, &decimate_options(), &ToleranceOptions::new(budget), scale);
    coarse
}

/// Distances of the sample points from the patches (`None` where no patch is found).
fn distances(set: &PatchSet, samples: &[Point3]) -> Vec<f64> {
    let bvh = Bvh::build(&tessellate(set, MEASURE_GRID));
    samples.iter().map(|p| bvh.closest(p, f64::INFINITY).map_or(f64::INFINITY, |h| h.distance)).collect()
}

fn deviation_of(d: &[f64], tolerance: f64) -> WrapDeviation {
    let mut d: Vec<f64> = d.iter().copied().filter(|x| x.is_finite()).collect();
    if d.is_empty() {
        return WrapDeviation::default();
    }
    d.sort_by(f64::total_cmp);
    let count = d.len();
    let within = d.iter().filter(|x| **x <= tolerance).count() as f64 / count as f64;
    let rms = (d.iter().map(|x| x * x).sum::<f64>() / count as f64).sqrt();
    let p99 = d.get(((count - 1) as f64 * 0.99) as usize).copied().unwrap_or(0.0);
    WrapDeviation { count, within, rms, p99, max: d.last().copied().unwrap_or(0.0) }
}

/// Lays the patches on `coarse` and measures them; also returns the distance of every sample.
fn build_round(t: &Target, coarse: &Mesh, budget: f64, capped: bool, g1: bool) -> (Round, Vec<f64>) {
    let options = G1Options { g1, ..t.options };
    let wrap = wrap_patches_g1(coarse, &t.bvh, &t.surface, REACH_BUDGETS * budget, &options);
    let d = distances(&wrap.set, &t.samples);
    let deviation = deviation_of(&d, t.tolerance);
    let (crease_edges, smooth_edges, g0_patches) = (wrap.crease_edges, wrap.smooth_edges, wrap.fallbacks);
    let (set, creases) = (wrap.set, wrap.creases);
    (
        Round {
            set,
            coarse: coarse.clone(),
            creases,
            crease_edges,
            smooth_edges,
            g0_patches,
            coarse_triangles: coarse.triangle_count(),
            budget,
            capped,
            deviation,
        },
        d,
    )
}

fn level_of(round: usize, r: &Round, refined_vertices: usize) -> WrapLevel {
    WrapLevel {
        round,
        budget: r.budget,
        coarse_triangles: r.coarse_triangles,
        patches: r.set.nets.len(),
        within: r.deviation.within,
        refined_vertices,
    }
}

/// Keeps `round` when it fits the scan better than `best`.
fn keep_better(best: Option<Round>, round: Round) -> Option<Round> {
    match best {
        Some(b) if b.deviation.within >= round.deviation.within => Some(b),
        _ => Some(round),
    }
}

fn failed() -> CmdError {
    CmdError::Failed("the scan could not be wrapped".into())
}

/// The uniform layout: the budget of the whole scan is halved each round.
fn uniform_rounds(t: &Target, g1: bool) -> CmdResult<Wrapped> {
    let mut best: Option<Round> = None;
    let mut levels = Vec::new();
    let mut budget = FIRST_BUDGET * t.tolerance;
    for round in 1..=UNIFORM_ROUNDS {
        let raw = reduce_scan(t.scan, budget, &[]);
        let capped = raw.triangle_count() > t.max_triangles;
        let coarse = fit_to_limit(raw, budget, t.max_triangles);
        let (r, _) = build_round(t, &coarse, budget, capped, g1);
        levels.push(level_of(round, &r, 0));
        let done = capped || r.deviation.within >= t.min_within;
        best = keep_better(best, r);
        if done {
            break;
        }
        budget *= 0.5;
    }
    let rounds = levels.len();
    best.map(|best| Wrapped { best, rounds, levels }).ok_or_else(failed)
}

/// Scan vertices whose sample is farther from the patches than the tolerance.
fn missed_vertices(t: &Target, d: &[f64]) -> Vec<u32> {
    d.iter()
        .enumerate()
        .filter(|(_, x)| **x > t.tolerance)
        .map(|(k, _)| u32::try_from(k * t.stride).unwrap_or(u32::MAX))
        .collect()
}

fn scale_of(levels: &[u8]) -> Vec<f64> {
    levels.iter().map(|&l| 0.5f64.powi(i32::from(l))).collect()
}

/// The adaptive layout: detail levels are raised around the scan points the patches miss, the
/// coarse mesh is reduced again from the scan with the new levels. A round whose mesh would not
/// fit `max_patches` is not built: the best earlier round is returned, flagged as capped.
fn adaptive_rounds(t: &Target, g1: bool) -> CmdResult<Wrapped> {
    let mut best: Option<Round> = None;
    let mut log = Vec::new();
    let mut levels: Vec<u8> = vec![0; t.scan.positions.len()];
    let budget = ADAPTIVE_FIRST_BUDGET * t.tolerance;
    for round in 1..=ADAPTIVE_ROUNDS {
        let raw = reduce_scan(t.scan, budget, &scale_of(&levels));
        let over = raw.triangle_count() > t.max_triangles;
        if over && round > 1 {
            best = best.map(|b| Round { capped: true, ..b });
            break;
        }
        let coarse = if over { fit_to_limit(raw, budget, t.max_triangles) } else { raw };
        let (r, d) = build_round(t, &coarse, budget, over, g1);
        let refined = levels.iter().filter(|&&l| l > 0).count();
        log.push(level_of(round, &r, refined));
        let done = over || r.deviation.within >= t.min_within;
        best = keep_better(best, r);
        let seeds = missed_vertices(t, &d);
        if done || seeds.is_empty() {
            break;
        }
        let next = raise_levels(t.scan, &seeds, &levels, LEVEL_RINGS, MAX_LEVEL);
        if next == levels {
            break;
        }
        levels = next;
    }
    let rounds = log.len();
    best.map(|best| Wrapped { best, rounds, levels: log }).ok_or_else(failed)
}

fn search(t: &Target, refine: Refine, g1: bool) -> CmdResult<Wrapped> {
    match refine {
        Refine::Adaptive => adaptive_rounds(t, g1),
        Refine::Uniform => uniform_rounds(t, g1),
    }
}

/// Wraps the scan until `min_within` of the samples are within the tolerance, or the rounds or the
/// patch limit run out.
///
/// The layout is searched with the unconstrained patches (they follow the scan best, and are cheapest);
/// the best layout is then wrapped once more with tangent patches. When those meet the tolerance too
/// (or lose nothing the unconstrained ones had) they are the result; otherwise the search is repeated
/// with tangent patches, so the refinement absorbs what the tangent condition costs.
pub(super) fn wrap_rounds(t: &Target, refine: Refine) -> CmdResult<Wrapped> {
    let mut found = search(t, refine, false)?;
    if !t.options.g1 {
        return Ok(found);
    }
    let best = &found.best;
    let (tangent, _) = build_round(t, &best.coarse, best.budget, best.capped, true);
    let kept = tangent.deviation.within >= t.min_within.min(best.deviation.within);
    if !kept {
        return search(t, refine, true);
    }
    let refined = found.levels.last().map_or(0, |l| l.refined_vertices);
    found.levels.push(level_of(found.rounds + 1, &tangent, refined));
    Ok(Wrapped { best: tangent, rounds: found.rounds + 1, levels: found.levels })
}
