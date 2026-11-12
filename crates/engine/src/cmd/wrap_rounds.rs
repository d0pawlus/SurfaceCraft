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
//!
//! `max_patches` is a hard bound for every round (three patches per coarse triangle). A first round
//! that does not fit is cut to the cap ([`trim_to_cap`]). A later round that would not fit is not
//! dropped: the largest refinement that does fit is searched for ([`search_cap`]) — the worst
//! missed regions first in the adaptive layout (how many of the scan vertices ranked by their
//! deviation get more detail), the budget in the uniform one — so the whole cap is spent.
use super::wrap::WrapDeviation;
use super::wrap_cap::{End, decimate_options, search_cap, trim_to_cap};
use super::{CmdError, CmdResult};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use surfacecraft_fit::mls::ScanSurface;
use surfacecraft_fit::patch::{PatchSet, tessellate};
use surfacecraft_fit::patch_g1::{G1Options, wrap_patches_g1};
use surfacecraft_geom::reduce::{ToleranceOptions, raise_levels, reduce_to_tolerance_scaled};
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
/// Tessellation of a patch for the measurement (quads per side).
const MEASURE_GRID: usize = 6;
/// Smallest change of the logarithm of the number of vertices refined the adaptive search tells
/// apart (5 %).
const VERTEX_STEP: f64 = 0.05;
/// Smallest change of the logarithm of the budget the uniform search tells apart (2 %).
const UNIFORM_STEP: f64 = 0.02;

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

/// Why a round was cut to the patch cap instead of being built as the budget asked.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum PartialReason {
    /// The first coarse mesh was bigger than `max_patches` allows and was cut to it.
    FirstRoundOverCap,
    /// The next refinement was bigger than `max_patches` allows: the largest one that fits was built
    /// (adaptive: the worst missed regions first; uniform: the budget between the two rounds).
    NextRoundOverCap,
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
    /// Set when `max_patches` cut this round; absent when the round is what the budget asked for.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub partial: Option<PartialReason>,
    /// Reductions of the scan spent searching for the largest refinement that fits the cap.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub probes: usize,
}

fn is_zero(n: &usize) -> bool {
    *n == 0
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
    partial: Option<PartialReason>,
    probes: usize,
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
    max_patches: usize,
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
            max_patches,
        }
    }
}

/// The result of all rounds.
pub(super) struct Wrapped {
    pub(super) best: Round,
    pub(super) rounds: usize,
    pub(super) levels: Vec<WrapLevel>,
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

/// A coarse mesh ready to be wrapped, and how it came about.
struct Laid {
    mesh: Mesh,
    budget: f64,
    partial: Option<PartialReason>,
    probes: usize,
    refined_vertices: usize,
}

fn over_the_cap() -> CmdError {
    CmdError::Failed("the scan cannot be represented within max_patches: raise it".into())
}

/// Lays the patches on `coarse` and measures them; also returns the distance of every sample.
fn build_round(t: &Target, coarse: &Mesh, budget: f64, g1: bool) -> (Round, Vec<f64>) {
    let options = G1Options { g1, ..t.options };
    let wrap = wrap_patches_g1(coarse, &t.bvh, &t.surface, REACH_BUDGETS * budget, &options);
    let d = distances(&wrap.set, &t.samples);
    let deviation = deviation_of(&d, t.tolerance);
    let (crease_edges, smooth_edges, g0_patches) = (wrap.crease_edges, wrap.smooth_edges, wrap.fallbacks);
    let (set, creases) = (wrap.set, wrap.creases);
    let round = Round {
        set,
        coarse: coarse.clone(),
        creases,
        crease_edges,
        smooth_edges,
        g0_patches,
        coarse_triangles: coarse.triangle_count(),
        budget,
        capped: false,
        deviation,
        partial: None,
        probes: 0,
    };
    (round, d)
}

fn level_of(round: usize, r: &Round, refined_vertices: usize) -> WrapLevel {
    WrapLevel {
        round,
        budget: r.budget,
        coarse_triangles: r.coarse_triangles,
        patches: r.set.nets.len(),
        within: r.deviation.within,
        refined_vertices,
        partial: r.partial,
        probes: r.probes,
    }
}

/// The rounds so far.
struct Run {
    /// Tangent patches (the layout search itself runs with unconstrained ones).
    g1: bool,
    best: Option<Round>,
    log: Vec<WrapLevel>,
    /// Did `max_patches` cut a round, or stop the refinement?
    hit_cap: bool,
}

impl Run {
    fn new(g1: bool) -> Self {
        Self { g1, best: None, log: Vec::new(), hit_cap: false }
    }

    /// Lays the patches on `laid`, logs the round and keeps it when it fits the scan better than the
    /// best so far (an equal share keeps the earlier, smaller one). Returns the distance of every
    /// sample and whether the round reached `min_within`.
    fn add(&mut self, t: &Target, laid: Laid) -> (Vec<f64>, bool) {
        let (round, d) = build_round(t, &laid.mesh, laid.budget, self.g1);
        let round = Round { partial: laid.partial, probes: laid.probes, ..round };
        self.log.push(level_of(self.log.len() + 1, &round, laid.refined_vertices));
        self.hit_cap |= round.partial.is_some();
        let met = round.deviation.within >= t.min_within;
        // never more patches than the cap, whatever the layout did (three per triangle by construction)
        let fits = round.set.nets.len() <= t.max_patches;
        self.best = match self.best.take() {
            Some(b) if !fits || b.deviation.within >= round.deviation.within => Some(b),
            None if !fits => None,
            _ => Some(round),
        };
        (d, met && fits)
    }

    /// The best round, flagged as capped when the cap cut a round or stopped the refinement and the
    /// tolerance was still missed.
    fn finish(self, t: &Target) -> CmdResult<Wrapped> {
        let rounds = self.log.len();
        let capped = self.hit_cap;
        let best = self.best.ok_or_else(over_the_cap)?;
        let capped = capped && best.deviation.within < t.min_within;
        Ok(Wrapped { best: Round { capped, ..best }, rounds, levels: self.log })
    }
}

/// The first round, bigger than the cap: cut to it.
fn first_round_over(t: &Target, raw: Mesh, budget: f64) -> CmdResult<Laid> {
    let mesh = trim_to_cap(raw, t.max_triangles, true).ok_or_else(over_the_cap)?;
    Ok(Laid { mesh, budget, partial: Some(PartialReason::FirstRoundOverCap), probes: 0, refined_vertices: 0 })
}

/// The round the budget asked for, bigger than the cap, cut to it by the decimation: detail stays
/// where the quadric error is highest. Costs no reduction of the scan.
fn trimmed(t: &Target, raw: Mesh, budget: f64, refined_vertices: usize) -> Option<Laid> {
    let mesh = trim_to_cap(raw, t.max_triangles, false)?;
    Some(Laid { mesh, budget, partial: Some(PartialReason::NextRoundOverCap), probes: 0, refined_vertices })
}

/// A uniform round bigger than the cap: the round cut to the cap is built first; when it misses the
/// tolerance the budget between the last round and this one that just fits is searched for (the
/// better of the two is kept). Returns that second round, or `None` when there is none to build.
fn uniform_over(t: &Target, run: &mut Run, raw: Mesh, budget: f64, last: usize) -> Option<Laid> {
    let ends = (End { x: -(2.0 * budget).ln(), triangles: last }, End { x: -budget.ln(), triangles: raw.triangle_count() });
    if let Some(cut) = trimmed(t, raw, budget, 0)
        && run.add(t, cut).1
    {
        return None;
    }
    let (found, probes) = search_cap(t.max_triangles, ends, UNIFORM_STEP, |x| reduce_scan(t.scan, (-x).exp(), &[]));
    let f = found?;
    Some(Laid { mesh: f.mesh, budget: (-f.x).exp(), partial: Some(PartialReason::NextRoundOverCap), probes, refined_vertices: 0 })
}

/// The uniform layout: the budget of the whole scan is halved each round. A round that would not
/// fit the cap is replaced by rounds that do (see [`uniform_over`]).
fn uniform_rounds(t: &Target, g1: bool) -> CmdResult<Wrapped> {
    let mut run = Run::new(g1);
    let mut budget = FIRST_BUDGET * t.tolerance;
    let mut last = 0;
    for round in 1..=UNIFORM_ROUNDS {
        let raw = reduce_scan(t.scan, budget, &[]);
        let laid = if raw.triangle_count() <= t.max_triangles {
            Laid { mesh: raw, budget, partial: None, probes: 0, refined_vertices: 0 }
        } else if round == 1 {
            first_round_over(t, raw, budget)?
        } else {
            run.hit_cap = true;
            let Some(l) = uniform_over(t, &mut run, raw, budget, last) else { break };
            l
        };
        let partial = laid.partial.is_some();
        last = laid.mesh.triangle_count();
        let (_, met) = run.add(t, laid);
        if partial || met {
            break;
        }
        budget *= 0.5;
    }
    run.finish(t)
}

/// Scan vertices whose sample is farther from the patches than the tolerance, the farthest first.
fn missed_vertices(t: &Target, d: &[f64]) -> Vec<u32> {
    let mut missed: Vec<(f64, u32)> = d
        .iter()
        .enumerate()
        .filter(|(_, x)| **x > t.tolerance)
        .map(|(k, x)| (*x, u32::try_from(k * t.stride).unwrap_or(u32::MAX)))
        .collect();
    missed.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
    missed.into_iter().map(|(_, v)| v).collect()
}

fn scale_of(levels: &[u8]) -> Vec<f64> {
    levels.iter().map(|&l| 0.5f64.powi(i32::from(l))).collect()
}

fn refined_count(levels: &[u8]) -> usize {
    levels.iter().filter(|&&l| l > 0).count()
}

/// The levels with detail raised around the `count` worst missed vertices of `ranked`.
fn raised(t: &Target, levels: &[u8], ranked: &[u32], count: usize) -> Vec<u8> {
    raise_levels(t.scan, ranked.get(..count).unwrap_or(ranked), levels, LEVEL_RINGS, MAX_LEVEL)
}

/// What the adaptive layout has when it asks for the next round.
struct Adaptive<'a> {
    levels: &'a [u8],
    /// Missed vertices of the last round, the worst first.
    ranked: &'a [u32],
    /// Triangles of the last round.
    last: usize,
}

/// The next adaptive round, bigger than the cap: first the round cut to the cap (see [`trimmed`]),
/// then the number of worst missed vertices that get more detail is searched so that the mesh just
/// fits; the better of the two is kept. Returns the second round, or `None` when there is none to
/// build (the first met the tolerance, or no refinement fits).
fn next_round_over(t: &Target, run: &mut Run, a: &Adaptive, raw: Mesh, budget: f64) -> Option<Laid> {
    let raw_count = raw.triangle_count();
    let all = refined_count(&raised(t, a.levels, a.ranked, a.ranked.len()));
    if let Some(cut) = trimmed(t, raw, budget, all)
        && run.add(t, cut).1
    {
        return None;
    }
    // the count rises steeply with the first vertices refined: the search runs on ln(1 + count)
    let ends = (End { x: 0.0, triangles: a.last }, End { x: (a.ranked.len() as f64).ln_1p(), triangles: raw_count });
    let levels_of = |x: f64| raised(t, a.levels, a.ranked, x.exp_m1().round() as usize);
    let probe = |x: f64| reduce_scan(t.scan, budget, &scale_of(&levels_of(x)));
    let (found, probes) = search_cap(t.max_triangles, ends, VERTEX_STEP, probe);
    let f = found?;
    let refined_vertices = refined_count(&levels_of(f.x));
    Some(Laid { mesh: f.mesh, budget, partial: Some(PartialReason::NextRoundOverCap), probes, refined_vertices })
}

/// The adaptive layout: detail levels are raised around the scan points the patches miss, the
/// coarse mesh is reduced again from the scan with the new levels. A round whose mesh would not
/// fit `max_patches` is built with as many of the worst missed vertices refined as fit.
fn adaptive_rounds(t: &Target, g1: bool) -> CmdResult<Wrapped> {
    let mut run = Run::new(g1);
    let mut levels: Vec<u8> = vec![0; t.scan.positions.len()];
    // the levels of the last round (those the next round raises)
    let mut base = levels.clone();
    let mut ranked: Vec<u32> = Vec::new();
    let mut last = 0;
    let budget = ADAPTIVE_FIRST_BUDGET * t.tolerance;
    for round in 1..=ADAPTIVE_ROUNDS {
        let raw = reduce_scan(t.scan, budget, &scale_of(&levels));
        let laid = if raw.triangle_count() <= t.max_triangles {
            Laid { mesh: raw, budget, partial: None, probes: 0, refined_vertices: refined_count(&levels) }
        } else if round == 1 {
            first_round_over(t, raw, budget)?
        } else {
            run.hit_cap = true;
            let a = Adaptive { levels: &base, ranked: &ranked, last };
            let Some(l) = next_round_over(t, &mut run, &a, raw, budget) else { break };
            l
        };
        let partial = laid.partial.is_some();
        last = laid.mesh.triangle_count();
        let (d, met) = run.add(t, laid);
        ranked = missed_vertices(t, &d);
        if partial || met || ranked.is_empty() {
            break;
        }
        let next = raise_levels(t.scan, &ranked, &levels, LEVEL_RINGS, MAX_LEVEL);
        if next == levels {
            break;
        }
        base = levels;
        levels = next;
    }
    run.finish(t)
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
    let (tangent, _) = build_round(t, &best.coarse, best.budget, true);
    let tangent = Round { capped: best.capped, partial: best.partial, probes: best.probes, ..tangent };
    let kept = tangent.deviation.within >= t.min_within.min(best.deviation.within);
    if !kept {
        return search(t, refine, true);
    }
    let refined = found.levels.last().map_or(0, |l| l.refined_vertices);
    found.levels.push(level_of(found.rounds + 1, &tangent, refined));
    Ok(Wrapped { best: tangent, rounds: found.rounds + 1, levels: found.levels })
}
