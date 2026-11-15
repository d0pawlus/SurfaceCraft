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
//! dropped. In the adaptive layout it is cut to the cap by the decimation (detail stays where the
//! quadric error is highest, which is where the levels were raised), and while such rounds gain the
//! rounds go on at the cap, the detail raised around the misses of the best round: at the cap a
//! round costs time, not patches, so they go on past `min_within` too and leave a margin. When the
//! first such round gains nothing, the largest refinement that fits is searched for ([`search_cap`]:
//! how many of the scan vertices ranked by their deviation get more detail); in the uniform layout
//! the budget between the two rounds is. Either way the whole cap is spent.
use super::wrap::WrapDeviation;
use super::wrap_cap::{End, Trimmed, decimate_options, search_cap, trim_to_cap};
use super::wrap_cluster::ClusterError;
use super::{CmdError, CmdResult};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::cell::RefCell;
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
/// With the coarse start the first round aims at this share of the cap...
const COARSE_SHARE: f64 = 0.5;
/// ... with a budget at most this many times the usual first one.
const MAX_COARSE: f64 = 4.0;
/// Rounds at most of the uniform layout; each one halves the budget.
const UNIFORM_ROUNDS: usize = 3;
/// Rounds at most of the adaptive layout.
const ADAPTIVE_ROUNDS: usize = 7;
/// The adaptive rounds stop (below the cap) once the share within the tolerance reaches
/// `min_within` plus this share of what `min_within` leaves out (99.75 % for 99 %): the wrap measures
/// a stride of the scan against a coarse tessellation of the patches, so a network that only just
/// meets `min_within` there can miss it against the scan. Only the stop uses it; whether the
/// tolerance is met is still judged by `min_within`.
const STOP_MARGIN: f64 = 0.75;
/// A round cut to the cap must raise the best share within the tolerance by this much for the
/// rounds to go on at the cap (0.1 percentage points of the samples).
const MIN_GAIN_AT_CAP: f64 = 0.001;
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
    /// Was the coarse mesh made by grid clustering?
    pub(super) clustered: bool,
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
    /// Share at which the adaptive rounds stop below the cap (see [`STOP_MARGIN`]).
    stop_within: f64,
    max_triangles: usize,
    max_patches: usize,
    /// Start coarser when the first round would not fit the cap (see [`first_round`]).
    coarse_start: bool,
    /// The clustered first round (keyed by the triangles of the mesh it was made from and the cap): the clustering
    /// is paid once per call, however many searches ask for the first round.
    cluster_cache: RefCell<Option<((usize, usize), Mesh)>>,
}

impl<'a> Target<'a> {
    /// The target with the coarse start on or off (see [`first_round`]).
    pub(super) fn with_coarse_start(self, on: bool) -> Self {
        Self { coarse_start: on, ..self }
    }

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
            stop_within: if std::env::var("TMP_STOP").is_ok_and(|v| v == "0") { min_within } else { min_within + STOP_MARGIN * (1.0 - min_within) },
            max_triangles: (max_patches / PATCHES_PER_TRIANGLE).max(4),
            coarse_start: false,
            max_patches,
            cluster_cache: RefCell::new(None),
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
    /// Was the mesh made by grid clustering (edge collapse stalled)?
    clustered: bool,
}

fn cap_error(e: ClusterError) -> CmdError {
    CmdError::Failed(
        match e {
            ClusterError::Over => "the scan cannot be represented within max_patches: raise it",
            ClusterError::Thin => {
                "the scan is too thin or too fine to be reduced to max_patches without losing its surface: raise it"
            }
        }
        .into(),
    )
}

fn over_the_cap() -> CmdError {
    cap_error(ClusterError::Over)
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
        clustered: false,
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
    /// The distance of every sample from the best round.
    best_d: Vec<f64>,
    log: Vec<WrapLevel>,
    /// Did `max_patches` cut a round, or stop the refinement?
    hit_cap: bool,
}

impl Run {
    fn new(g1: bool) -> Self {
        Self { g1, best: None, best_d: Vec::new(), log: Vec::new(), hit_cap: false }
    }

    /// Lays the patches on `laid`, logs the round and keeps it when it fits the scan better than the
    /// best so far (an equal share keeps the earlier, smaller one). Returns the distance of every
    /// sample and whether the round reached `min_within`.
    fn add(&mut self, t: &Target, laid: &Laid) -> (Vec<f64>, bool) {
        let (round, d) = build_round(t, &laid.mesh, laid.budget, self.g1);
        let round = Round { partial: laid.partial, probes: laid.probes, clustered: laid.clustered, ..round };
        self.log.push(level_of(self.log.len() + 1, &round, laid.refined_vertices));
        self.hit_cap |= round.partial.is_some();
        let met = round.deviation.within >= t.min_within;
        // never more patches than the cap, whatever the layout did (three per triangle by construction)
        let fits = round.set.nets.len() <= t.max_patches;
        self.best = match self.best.take() {
            Some(b) if !fits || b.deviation.within >= round.deviation.within => Some(b),
            None if !fits => None,
            _ => {
                self.best_d.clone_from(&d);
                Some(round)
            }
        };
        (d, met && fits)
    }

    /// Share within the tolerance of the best round so far (0 before any).
    fn best_within(&self) -> f64 {
        self.best.as_ref().map_or(0.0, |b| b.deviation.within)
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
    let key = (raw.triangle_count(), t.max_triangles);
    let cached = t.cluster_cache.borrow().as_ref().filter(|(k, _)| *k == key).map(|(_, m)| m.clone());
    let cut = match cached {
        Some(mesh) => Trimmed { mesh, clustered: true },
        None => trim_to_cap(raw, t.max_triangles, true).map_err(cap_error)?,
    };
    if cut.clustered {
        *t.cluster_cache.borrow_mut() = Some((key, cut.mesh.clone()));
    }
    let (mesh, clustered) = (cut.mesh, cut.clustered);
    Ok(Laid {
        mesh,
        budget,
        partial: Some(PartialReason::FirstRoundOverCap),
        probes: 0,
        refined_vertices: 0,
        clustered,
    })
}

/// The round the budget asked for, bigger than the cap, cut to it by the decimation: detail stays
/// where the quadric error is highest. Costs no reduction of the scan.
fn trimmed(t: &Target, raw: Mesh, budget: f64, refined_vertices: usize) -> Option<Laid> {
    let mesh = trim_to_cap(raw, t.max_triangles, false).ok()?.mesh;
    Some(Laid {
        mesh,
        budget,
        partial: Some(PartialReason::NextRoundOverCap),
        probes: 0,
        refined_vertices,
        clustered: false,
    })
}

/// A uniform round bigger than the cap: the round cut to the cap is built first; when it misses the
/// tolerance the budget between the last round and this one that just fits is searched for (the
/// better of the two is kept). Returns that second round, or `None` when there is none to build.
fn uniform_over(t: &Target, run: &mut Run, raw: Mesh, budget: f64, last: usize) -> Option<Laid> {
    let ends =
        (End { x: -(2.0 * budget).ln(), triangles: last }, End { x: -budget.ln(), triangles: raw.triangle_count() });
    if let Some(cut) = trimmed(t, raw, budget, 0)
        && run.add(t, &cut).1
    {
        return None;
    }
    let (found, probes) = search_cap(t.max_triangles, ends, UNIFORM_STEP, |x| reduce_scan(t.scan, (-x).exp(), &[]));
    let f = found?;
    Some(Laid {
        mesh: f.mesh,
        budget: (-f.x).exp(),
        partial: Some(PartialReason::NextRoundOverCap),
        probes,
        refined_vertices: 0,
        clustered: false,
    })
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
            Laid { mesh: raw, budget, partial: None, probes: 0, refined_vertices: 0, clustered: false }
        } else if round == 1 {
            first_round_over(t, raw, budget)?
        } else {
            run.hit_cap = true;
            let Some(l) = uniform_over(t, &mut run, raw, budget, last) else { break };
            l
        };
        let partial = laid.partial.is_some();
        last = laid.mesh.triangle_count();
        let (_, met) = run.add(t, &laid);
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

/// The next adaptive round, bigger than the cap, with the number of worst missed vertices that get
/// more detail searched so that the mesh just fits (`raw_count`: triangles of the round the
/// budget asked for). `None` when no refinement fits.
fn searched_round(t: &Target, a: &Adaptive, raw_count: usize, budget: f64) -> Option<Laid> {
    // the count rises steeply with the first vertices refined: the search runs on ln(1 + count)
    let ends = (End { x: 0.0, triangles: a.last }, End { x: (a.ranked.len() as f64).ln_1p(), triangles: raw_count });
    let levels_of = |x: f64| raised(t, a.levels, a.ranked, x.exp_m1().round() as usize);
    let probe = |x: f64| reduce_scan(t.scan, budget, &scale_of(&levels_of(x)));
    let (found, probes) = search_cap(t.max_triangles, ends, VERTEX_STEP, probe);
    let f = found?;
    let refined_vertices = refined_count(&levels_of(f.x));
    Some(Laid {
        mesh: f.mesh,
        budget,
        partial: Some(PartialReason::NextRoundOverCap),
        probes,
        refined_vertices,
        clustered: false,
    })
}

/// With the coarse start, a first round bigger than the cap is reduced again with a larger budget
/// (proportional to how far over it is, at most [`MAX_COARSE`] times) so that it takes about
/// [`COARSE_SHARE`] of the cap and the adaptive rounds can spend the rest where the patches miss. The
/// round as it was when the start is off, it already fits, or the coarser one does not fit either.
fn first_round(t: &Target, raw: Mesh, budget: f64) -> (Mesh, f64) {
    let count = raw.triangle_count();
    if !t.coarse_start || count <= t.max_triangles {
        return (raw, budget);
    }
    let factor = (count as f64 / (COARSE_SHARE * t.max_triangles as f64)).clamp(1.0, MAX_COARSE);
    let coarser = budget * factor;
    let again = reduce_scan(t.scan, coarser, &[]);
    if again.triangle_count() <= t.max_triangles { (again, coarser) } else { (raw, budget) }
}

/// The levels of the adaptive layout: those of the round being laid and of the last one.
struct Levels {
    now: Vec<u8>,
    /// The levels of the last round (those the next round raises).
    base: Vec<u8>,
    /// Missed vertices that the next round is refined around, the worst first.
    ranked: Vec<u32>,
    /// Triangles of the last round.
    last: usize,
    /// Has a round already been cut to the cap after the first?
    at_cap: bool,
}

/// The coarse mesh of an adaptive round and the triangles the reduction gave before any cut: the
/// scan reduced with the levels, cut to the cap when it does not fit (the first round to the cap,
/// a later one by the decimation, see [`trimmed`]; when that fails, the refinement that fits is
/// searched for). `None` ends the rounds.
fn lay_adaptive(t: &Target, lv: &Levels, round: usize, budget: &mut f64) -> CmdResult<Option<(Laid, usize)>> {
    let mut raw = reduce_scan(t.scan, *budget, &scale_of(&lv.now));
    if round == 1 {
        (raw, *budget) = first_round(t, raw, *budget);
    }
    let (raw_count, refined_vertices) = (raw.triangle_count(), refined_count(&lv.now));
    if raw_count <= t.max_triangles {
        let laid = Laid { mesh: raw, budget: *budget, partial: None, probes: 0, refined_vertices, clustered: false };
        return Ok(Some((laid, raw_count)));
    }
    if round == 1 {
        return first_round_over(t, raw, *budget).map(|l| Some((l, raw_count)));
    }
    if let Some(cut) = trimmed(t, raw, *budget, refined_vertices) {
        return Ok(Some((cut, raw_count)));
    }
    if lv.at_cap {
        return Ok(None);
    }
    let a = Adaptive { levels: &lv.base, ranked: &lv.ranked, last: lv.last };
    Ok(searched_round(t, &a, raw_count, *budget).map(|l| (l, raw_count)))
}

/// Has the best share within the tolerance risen by [`MIN_GAIN_AT_CAP`] since `before`?
fn gained(run: &Run, before: f64) -> bool {
    run.best_within() >= before + MIN_GAIN_AT_CAP
}

/// After a round cut to the cap that missed `min_within`: may the rounds go on at the cap? They
/// may when it raised the best share by [`MIN_GAIN_AT_CAP`]. When it did not, and it is the first
/// such round and not already the searched one, the refinement that just fits is searched for and
/// built as well (the better round is kept) before the rounds end.
fn go_on_at_cap(t: &Target, run: &mut Run, lv: &Levels, cut: (&Laid, usize), before: f64) -> bool {
    if gained(run, before) {
        return true;
    }
    let (laid, raw_count) = cut;
    if !lv.at_cap && laid.probes == 0 {
        let a = Adaptive { levels: &lv.base, ranked: &lv.ranked, last: lv.last };
        if let Some(found) = searched_round(t, &a, raw_count, laid.budget) {
            run.add(t, &found);
        }
    }
    false
}

/// The adaptive layout: detail levels are raised around the scan points the patches miss, the
/// coarse mesh is reduced again from the scan with the new levels. A round whose mesh would not
/// fit `max_patches` is cut to it by the decimation; while such rounds gain at least
/// [`MIN_GAIN_AT_CAP`] the rounds go on at the cap, the levels raised around the misses of the best
/// round (see [`go_on_at_cap`] for what happens when the first one does not gain).
fn adaptive_rounds(t: &Target, g1: bool) -> CmdResult<Wrapped> {
    let mut run = Run::new(g1);
    let n = t.scan.positions.len();
    let mut lv = Levels { now: vec![0; n], base: vec![0; n], ranked: Vec::new(), last: 0, at_cap: false };
    let mut budget = ADAPTIVE_FIRST_BUDGET * t.tolerance;
    for round in 1..=ADAPTIVE_ROUNDS {
        let before = run.best_within();
        let Some((laid, raw_count)) = lay_adaptive(t, &lv, round, &mut budget)? else { break };
        let reason = laid.partial;
        let (d, met) = run.add(t, &laid);
        let stop = run.log.last().is_some_and(|l| l.within >= t.stop_within);
        if reason == Some(PartialReason::FirstRoundOverCap) || (stop && reason.is_none()) {
            break;
        }
        if reason.is_some() {
            run.hit_cap = true;
            // at the cap a round costs no patches: it goes on while it gains, even past `min_within`
            let go_on =
                if met { gained(&run, before) } else { go_on_at_cap(t, &mut run, &lv, (&laid, raw_count), before) };
            if !go_on {
                break;
            }
            lv.at_cap = true;
        }
        lv.last = laid.mesh.triangle_count();
        lv.ranked = missed_vertices(t, if lv.at_cap { &run.best_d } else { &d });
        let next = raise_levels(t.scan, &lv.ranked, &lv.now, LEVEL_RINGS, MAX_LEVEL);
        if lv.ranked.is_empty() || next == lv.now {
            break;
        }
        lv.base = std::mem::replace(&mut lv.now, next);
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
    let tangent =
        Round { capped: best.capped, partial: best.partial, probes: best.probes, clustered: best.clustered, ..tangent };
    let kept = tangent.deviation.within >= t.min_within.min(best.deviation.within);
    if !kept {
        return search(t, refine, true);
    }
    let refined = found.levels.last().map_or(0, |l| l.refined_vertices);
    found.levels.push(level_of(found.rounds + 1, &tangent, refined));
    Ok(Wrapped { best: tangent, rounds: found.rounds + 1, levels: found.levels })
}
