//! `pair_coarse`: brings two scans of the same part into one pose without picked points, then
//! refines with ICP (design doc §6.1, P9-M2).
//!
//! Method (all samples are area-uniform and seeded, so the result is deterministic and does not
//! depend on the triangulation or the thread count):
//! 1. Candidates: the 24 signed permutations of the two principal frames (whole parts), plus poses
//!    from FPFH descriptor matches and RANSAC (partial overlap, ambiguous axes).
//! 2. A shortlist of the best distinct candidates by a truncated distance score on a subsample
//!    through the BVH of the fixed scan; each is refined by two short ICP passes.
//! 3. A tolerance from the best candidate's residual; every refined candidate is re-scored under
//!    it, equal poses merged. The best one is polished by a full ICP.
//! 4. Verdict: no match when too little of either scan lies on the other; ambiguous when a
//!    different pose explains the moving scan about as well; otherwise unique.
//!
//! Cost: `O(T)` to sample (T triangles), two BVH builds, about 3000 samples per scan for the
//! coarse stage (descriptors and matching: a few hundred milliseconds), at most 20 000 vertices in
//! the final ICP. It stays usable on scans of a million triangles.
mod candidates;
mod fpfh;
mod ransac;
mod sample;
mod score;
mod verdict;

use crate::{AlignError, IcpOptions, icp_cancellable};
use sample::Samples;
use score::{Fit, Target, corners, fit_of, median, pose_gap};
use surfacecraft_geom::{Cancel, Mesh, Point3, Transform};

pub use verdict::{AMBIGUITY_MARGIN, CONFIDENCE_THRESHOLD, CoarseMethod, MIN_OVERLAP, PairCandidate, PairVerdict};

/// Fewest triangles a scan may have.
const MIN_TRIANGLES: usize = 4;
/// Most samples per scan in the coarse stage.
const SAMPLE_CAP: usize = 3000;
/// Moving samples used to score the unrefined candidates.
const SHORTLIST_SAMPLES: usize = 1500;
/// Candidates refined by ICP.
const SHORTLIST: usize = 8;
/// Alternatives listed.
const MAX_ALTERNATIVES: usize = 7;
/// Tolerance of the shortlist score, as a fraction of the diagonal of the fixed scan.
const COARSE_TAU: f64 = 0.06;
/// Cut distances of the two refinement passes, as fractions of the diagonal.
const REFINE_CUTS: [f64; 2] = [0.05, 0.015];
/// Search range for the tolerance (fractions of the diagonal) and its factor over the median
/// residual.
const TAU_RANGE: (f64, f64) = (0.0015, 0.02);
const TAU_FACTOR: f64 = 5.0;
/// Poses closer than this fraction of the diagonal (largest displacement of the corners of the
/// moving scan's box) are one pose.
const SAME_POSE_BEFORE: f64 = 0.03;
const SAME_POSE_AFTER: f64 = 0.02;

/// Settings of [`pair_coarse`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PairOptions {
    /// Distance (mm) within which a point of one scan counts as lying on the other; `None` takes
    /// it from the residual of the best pose (between 0.15 % and 2 % of the diagonal).
    pub tolerance: Option<f64>,
}

/// What [`pair_coarse`] found.
#[derive(Clone, Debug)]
pub struct PairResult {
    /// The best pose, moving → fixed, refined by ICP. A proposal also when the verdict is not
    /// [`PairVerdict::Unique`].
    pub transform: Transform,
    /// Can the pose be trusted?
    pub verdict: PairVerdict,
    /// 0..1: how far the best pose explains the moving scan better than any different pose; 0 for
    /// no match. Certain from [`CONFIDENCE_THRESHOLD`].
    pub confidence: f64,
    /// Tolerance (mm) used.
    pub tolerance: f64,
    /// Share (0..1) of the moving scan's area on the fixed scan.
    pub overlap_moving: f64,
    /// Share (0..1) of the fixed scan's area on the moving scan.
    pub overlap_fixed: f64,
    /// RMS distance (mm) of the matched part of the moving scan (0 when nothing matched).
    pub rms: f64,
    /// Stage that proposed the best pose.
    pub method: CoarseMethod,
    /// Different poses that fit about as well as the best one, best first.
    pub alternatives: Vec<PairCandidate>,
    /// Candidate poses scored.
    pub candidates: usize,
    /// Iterations of the final ICP (0 when it did not run).
    pub icp_iterations: usize,
    /// Did the final ICP converge?
    pub icp_converged: bool,
}

#[derive(Clone, Copy)]
struct Cand {
    t: Transform,
    method: CoarseMethod,
}

struct Ranked {
    cand: Cand,
    fit: Fit,
}

struct Scene<'a> {
    moving: &'a Mesh,
    mov: Samples,
    fix: Samples,
    fixed: Target<'a>,
    /// The moving samples as a point-only mesh, the source of the short ICP passes.
    mov_cloud: Mesh,
    diag: f64,
    spacing: f64,
}

fn check_scan(m: &Mesh, what: &str) -> Result<f64, AlignError> {
    if m.triangle_count() < MIN_TRIANGLES {
        let n = m.triangle_count();
        return Err(AlignError::NotEnough(format!(
            "the {what} scan has {n} triangles; at least {MIN_TRIANGLES} are needed"
        )));
    }
    m.validate().map_err(|e| AlignError::Degenerate(format!("the {what} scan is invalid: {e}")))?;
    let area = m.area();
    if !(area.is_finite() && area > 0.0) {
        return Err(AlignError::Degenerate(format!("the {what} scan has no finite surface area")));
    }
    Ok(area)
}

impl<'a> Scene<'a> {
    fn new(moving: &'a Mesh, fixed: &'a Mesh, cancel: &Cancel) -> Result<Self, AlignError> {
        let (am, af) = (check_scan(moving, "moving")?, check_scan(fixed, "fixed")?);
        let diag = fixed.bbox().diagonal();
        if !(diag.is_finite() && diag > 0.0) {
            return Err(AlignError::Degenerate("the fixed scan has no extent".into()));
        }
        let spacing = (am.max(af) / SAMPLE_CAP as f64).sqrt();
        let few = || AlignError::NotEnough("a scan has too few usable triangles to sample".into());
        let mov =
            sample::sample(moving, sample::count_for(am, spacing, SAMPLE_CAP), spacing, 1, cancel).ok_or_else(few)?;
        let fix =
            sample::sample(fixed, sample::count_for(af, spacing, SAMPLE_CAP), spacing, 2, cancel).ok_or_else(few)?;
        let mov_cloud = Mesh { positions: mov.pts.clone(), triangles: Vec::new() };
        Ok(Scene { moving, mov, fix, fixed: Target::new(fixed, diag), mov_cloud, diag, spacing })
    }

    fn hypotheses(&self, cancel: &Cancel) -> Result<Vec<Cand>, AlignError> {
        let mut out: Vec<Cand> = candidates::principal_candidates(&self.mov.pts, &self.fix.pts)
            .into_iter()
            .map(|t| Cand { t, method: CoarseMethod::PrincipalAxes })
            .collect();
        let poses = ransac::descriptor_poses(&self.mov, &self.fix, self.spacing, cancel)?;
        out.extend(poses.into_iter().map(|t| Cand { t, method: CoarseMethod::Descriptors }));
        Ok(out)
    }

    /// The best distinct candidates under a loose tolerance, best first.
    fn shortlist(&self, hyps: Vec<Cand>) -> Vec<Cand> {
        let pts: Vec<Point3> = self.mov.pts.iter().take(SHORTLIST_SAMPLES).copied().collect();
        let tau = COARSE_TAU * self.diag;
        let mut scored: Vec<(f64, usize, Cand)> = hyps
            .into_iter()
            .enumerate()
            .map(|(i, c)| (fit_of(&self.fixed, &pts, &c.t, tau, false).msac, i, c))
            .collect();
        scored.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
        let probes = corners(&self.mov.pts);
        let mut kept: Vec<Cand> = Vec::new();
        for (_, _, c) in scored {
            if kept.len() < SHORTLIST
                && kept.iter().all(|k| pose_gap(&k.t, &c.t, &probes) > SAME_POSE_BEFORE * self.diag)
            {
                kept.push(c);
            }
        }
        kept
    }

    /// Two ICP passes from `c`; `None` when ICP cannot run from there (the candidate is dropped).
    fn refine(&self, c: &Cand, cancel: &Cancel) -> Result<Option<Cand>, AlignError> {
        let mut t = c.t;
        for cut in REFINE_CUTS {
            let opts = IcpOptions {
                max_iterations: 25,
                tolerance: 1e-3,
                max_distance: Some(cut * self.diag),
                max_samples: SAMPLE_CAP,
            };
            match icp_cancellable(&self.mov_cloud, &self.fixed.bvh, &t, &opts, cancel) {
                Ok(rep) => t = rep.transform,
                Err(AlignError::Cancelled) => return Err(AlignError::Cancelled),
                Err(_) => return Ok(None),
            }
        }
        Ok(Some(Cand { t, method: c.method }))
    }

    /// A user tolerance must lie between a billionth and half of the target's diagonal.
    fn check_tolerance(&self, user: Option<f64>) -> Result<(), AlignError> {
        let Some(t) = user else { return Ok(()) };
        let (lo, hi) = (1e-9 * self.diag, 0.5 * self.diag);
        if t.is_finite() && t >= lo && t <= hi {
            return Ok(());
        }
        Err(AlignError::Degenerate(format!(
            "the tolerance must be between {lo:e} and {hi:e} mm (a billionth to half of the target's diagonal)"
        )))
    }

    fn msac_at(&self, t: &Transform, tau: f64) -> f64 {
        fit_of(&self.fixed, &self.mov.pts, t, tau, false).msac
    }

    /// The tolerance: the user's, or five times the median residual of the best refined pose.
    fn tolerance(&self, refined: &[Cand], user: Option<f64>) -> f64 {
        let (lo, hi) = (TAU_RANGE.0 * self.diag, TAU_RANGE.1 * self.diag);
        if let Some(u) = user {
            return u;
        }
        let tau0 = hi;
        let Some(best) = refined.iter().min_by(|a, b| self.msac_at(&a.t, tau0).total_cmp(&self.msac_at(&b.t, tau0)))
        else {
            return lo;
        };
        let dist: Vec<f64> = surfacecraft_geom::par::map_slice(&self.mov.pts, |p| {
            self.fixed.bvh.closest(&best.t.apply(p), tau0).map_or(f64::NAN, |h| h.distance)
        });
        let inside: Vec<f64> = dist.into_iter().filter(|d| d.is_finite()).collect();
        (TAU_FACTOR * median(inside)).clamp(lo, hi)
    }

    /// Refined candidates scored under `tau`, best first, equal poses merged.
    fn rank(&self, refined: Vec<Cand>, tau: f64) -> Vec<Ranked> {
        let mut all: Vec<(usize, Ranked)> = refined
            .into_iter()
            .enumerate()
            .map(|(i, cand)| (i, Ranked { fit: fit_of(&self.fixed, &self.mov.pts, &cand.t, tau, true), cand }))
            .collect();
        all.sort_by(|a, b| {
            b.1.fit.score().total_cmp(&a.1.fit.score()).then(a.1.fit.msac.total_cmp(&b.1.fit.msac)).then(a.0.cmp(&b.0))
        });
        let probes = corners(&self.mov.pts);
        let mut kept: Vec<Ranked> = Vec::new();
        for (_, r) in all {
            if kept.iter().all(|k| pose_gap(&k.cand.t, &r.cand.t, &probes) > SAME_POSE_AFTER * self.diag) {
                kept.push(r);
            }
        }
        kept
    }

    /// Runs the near-tied candidates (the best and everything within the ambiguity margin of it)
    /// to convergence and ranks again: a slide that only looked as good because ICP crawls along a
    /// poorly constrained direction ends in the same pose as the best one and merges with it.
    fn settle(&self, ranked: Vec<Ranked>, tau: f64, cancel: &Cancel) -> Result<Vec<Ranked>, AlignError> {
        let Some(top) = ranked.first().map(|r| r.fit.score()) else { return Ok(ranked) };
        let opts =
            IcpOptions { max_iterations: 100, tolerance: 1e-4, max_distance: Some(3.0 * tau), max_samples: SAMPLE_CAP };
        let mut again = Vec::with_capacity(ranked.len());
        for r in ranked {
            let mut cand = r.cand;
            if r.fit.score() >= top - AMBIGUITY_MARGIN {
                match icp_cancellable(&self.mov_cloud, &self.fixed.bvh, &cand.t, &opts, cancel) {
                    Ok(rep) => cand.t = rep.transform,
                    Err(AlignError::Cancelled) => return Err(AlignError::Cancelled),
                    Err(_) => {}
                }
            }
            again.push(cand);
        }
        Ok(self.rank(again, tau))
    }

    /// The best pose polished by an ICP on the whole moving scan (kept only when it scores no
    /// worse), with the iterations it took and whether it converged.
    fn polish(&self, best: &Ranked, tau: f64, cancel: &Cancel) -> Result<(Ranked, usize, bool), AlignError> {
        let opts = IcpOptions { max_distance: Some(3.0 * tau), ..IcpOptions::default() };
        let rep = match icp_cancellable(self.moving, &self.fixed.bvh, &best.cand.t, &opts, cancel) {
            Ok(rep) => rep,
            Err(AlignError::Cancelled) => return Err(AlignError::Cancelled),
            Err(_) => return Ok((Ranked { cand: best.cand, fit: best.fit }, 0, false)),
        };
        let fit = fit_of(&self.fixed, &self.mov.pts, &rep.transform, tau, true);
        if fit.score() >= best.fit.score() - 1e-9 {
            Ok((
                Ranked { cand: Cand { t: rep.transform, method: best.cand.method }, fit },
                rep.iterations,
                rep.converged,
            ))
        } else {
            Ok((Ranked { cand: best.cand, fit: best.fit }, rep.iterations, rep.converged))
        }
    }

    /// Share of the fixed scan's samples within `tau` of the moving scan placed by `t`.
    fn overlap_fixed(&self, t: &Transform, tau: f64) -> f64 {
        fit_of(&Target::plain(self.moving), &self.fix.pts, &t.inverse(), tau, false).share
    }
}

fn finite_or_zero(x: f64) -> f64 {
    if x.is_finite() { x } else { 0.0 }
}

fn alternative(r: &Ranked) -> PairCandidate {
    PairCandidate {
        transform: r.cand.t,
        overlap_moving: r.fit.share,
        rms: finite_or_zero(r.fit.rms),
        score: r.fit.score(),
        method: r.cand.method,
    }
}

/// Brings `moving` onto `fixed` without picked points: the motion (moving → fixed), how well the
/// two scans then coincide and how far the answer can be trusted. See the module docs for the
/// method. Input is checked (finite, with area); an error means no pose could be tried, a
/// [`PairVerdict::NoMatch`] means the best one found is not a match.
pub fn pair_coarse(moving: &Mesh, fixed: &Mesh, opts: &PairOptions, cancel: &Cancel) -> Result<PairResult, AlignError> {
    let scene = Scene::new(moving, fixed, cancel)?;
    scene.check_tolerance(opts.tolerance)?;
    let hyps = scene.hypotheses(cancel)?;
    let candidates = hyps.len();
    let mut refined = Vec::new();
    for c in scene.shortlist(hyps) {
        refined.extend(scene.refine(&c, cancel)?);
    }
    if refined.is_empty() {
        return Err(AlignError::Degenerate("no candidate pose could be refined".into()));
    }
    let tau = scene.tolerance(&refined, opts.tolerance);
    let ranked = scene.rank(refined, tau);
    let mut ranked = scene.settle(ranked, tau, cancel)?;
    if ranked.is_empty() {
        return Err(AlignError::Degenerate("no candidate pose survived".into()));
    }
    let first = ranked.remove(0);
    let (best, icp_iterations, icp_converged) = scene.polish(&first, tau, cancel)?;
    let overlap_fixed = scene.overlap_fixed(&best.cand.t, tau);
    let mut result = conclude(&best, &ranked, tau, overlap_fixed)?;
    result.candidates = candidates;
    result.icp_iterations = icp_iterations;
    result.icp_converged = icp_converged;
    Ok(result)
}

/// The verdict and the result from the best pose and the other distinct ones.
fn conclude(best: &Ranked, others: &[Ranked], tau: f64, overlap_fixed: f64) -> Result<PairResult, AlignError> {
    if !best.cand.t.to_rows().iter().flatten().all(|v| v.is_finite()) {
        return Err(AlignError::Degenerate("the pose is not finite".into()));
    }
    let overlap = best.fit.share.max(overlap_fixed);
    let runner_up =
        others.iter().map(|r| r.fit.score()).fold(None, |m: Option<f64>, s| Some(m.map_or(s, |x| x.max(s))));
    let (verdict, confidence) = verdict::judge(&best.fit, overlap, tau, runner_up);
    let alternatives: Vec<PairCandidate> = others
        .iter()
        .filter(|r| verdict == PairVerdict::Ambiguous && r.fit.score() >= best.fit.score() - AMBIGUITY_MARGIN)
        .take(MAX_ALTERNATIVES)
        .map(alternative)
        .collect();
    Ok(PairResult {
        transform: best.cand.t,
        verdict,
        confidence,
        tolerance: tau,
        overlap_moving: best.fit.share,
        overlap_fixed,
        rms: finite_or_zero(best.fit.rms),
        method: best.cand.method,
        alternatives,
        candidates: 0,
        icp_iterations: 0,
        icp_converged: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, Vec3, synth};

    fn bracket(block: usize, seed: u64) -> Mesh {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), block);
        m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
        m.append(&synth::sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 32, 16));
        synth::add_noise(&mut m, 0.05, &mut Rng::new(seed));
        m
    }

    fn part(m: &Mesh, keep: impl Fn(&Point3) -> bool) -> Mesh {
        let ids: Vec<u32> = (0..m.triangle_count())
            .filter(|&i| m.face_centroid(i).is_some_and(|c| keep(&c)))
            .map(|i| i as u32)
            .collect();
        m.submesh(&ids)
    }

    /// Two partial scans share half the part: no principal frame lines them up (their centroids
    /// and axes differ), so a candidate near the truth can only come from the descriptors.
    #[test]
    fn only_the_descriptors_find_the_pose_of_partial_scans() {
        let fixed = part(&bracket(12, 1), |c| c.z > 1.0 && c.x <= 54.0);
        let moving = part(&bracket(15, 2), |c| c.z > 1.0 && c.x >= 6.0);
        let pose = Transform::rotation_axis_angle(&Vec3::new(0.2, 0.7, -0.4), 2.0)
            .then_after(&Transform::translation(Vec3::new(60.0, 10.0, -30.0)));
        let moved = moving.transformed(&pose);
        let scene = Scene::new(&moved, &fixed, &Cancel::new()).unwrap();
        let hyps = scene.hypotheses(&Cancel::new()).unwrap();
        let truth = pose.inverse();
        let miss = |method: CoarseMethod| {
            hyps.iter()
                .filter(|c| c.method == method)
                .map(|c| c.t.max_displacement(&truth, &moved.positions))
                .fold(f64::INFINITY, f64::min)
        };
        let (descriptors, principal) = (miss(CoarseMethod::Descriptors), miss(CoarseMethod::PrincipalAxes));
        assert!(descriptors < 3.0, "descriptor candidates miss the truth by {descriptors} mm");
        assert!(
            principal > 8.0,
            "the principal axes should not find it ({principal} mm): the test would prove nothing"
        );
    }

    #[test]
    fn a_whole_part_is_found_from_its_principal_axes_alone() {
        let fixed = bracket(12, 1);
        let moving = bracket(15, 2);
        let pose = Transform::rotation_axis_angle(&Vec3::new(1.0, 0.1, 0.3), 2.8)
            .then_after(&Transform::translation(Vec3::new(-50.0, 20.0, 90.0)));
        let moved = moving.transformed(&pose);
        let scene = Scene::new(&moved, &fixed, &Cancel::new()).unwrap();
        let hyps = scene.hypotheses(&Cancel::new()).unwrap();
        let near = hyps
            .iter()
            .filter(|c| c.method == CoarseMethod::PrincipalAxes)
            .map(|c| c.t.max_displacement(&pose.inverse(), &moved.positions))
            .fold(f64::INFINITY, f64::min);
        assert!(near < 3.0, "{near}");
    }
}
