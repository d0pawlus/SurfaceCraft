//! `pair_coarse`: brings two scans of the same part into one pose without picked points, then
//! refines with ICP (design doc §6.1, P9-M2).
//!
//! Method (all samples are area-uniform and seeded, so the result is deterministic and does not
//! depend on the triangulation or the thread count):
//! 1. Candidates: the 24 signed permutations of the two principal frames (whole parts), plus poses
//!    from FPFH descriptor matches and RANSAC (partial overlap, ambiguous axes).
//! 2. A shortlist of the best distinct candidates by a truncated distance score on a subsample
//!    through the BVH of the fixed scan; each is refined by two short ICP passes.
//! 3. A tolerance from the best candidate's residual, capped by the noise of the two scans; every
//!    refined candidate is re-scored under it, equal poses merged. The best one is polished by a full ICP.
//! 4. Verdict: no match when too little of either scan lies on the other; ambiguous when a
//!    different pose explains the moving scan about as well; otherwise unique.
//!
//! Cost: `O(T)` to sample (T triangles), two BVH builds, about 3000 samples per scan for the
//! coarse stage (descriptors and matching: a few hundred milliseconds), at most 20 000 vertices in
//! the final ICP. It stays usable on scans of a million triangles.
mod candidates;
mod evaluate;
mod fpfh;
mod noise;
mod orientation;
mod probe;
mod ransac;
mod sample;
mod score;
mod verdict;

use crate::{AlignError, IcpOptions, IcpReport, icp_with_normals};
use sample::Samples;
use score::{Fit, Target, clearance, corners, fit_of, free_space_conflicts, median, pose_gap};
use surfacecraft_geom::{Cancel, Mesh, Point3, Rng, SurfaceHit, Transform};

pub use evaluate::{PoseFit, pose_fit};
pub use noise::ScanNoise;
pub use verdict::{
    AMBIGUITY_MARGIN, CONFIDENCE_THRESHOLD, CoarseMethod, MIN_OVERLAP, PairCandidate, PairVerdict, UNIQUE_MAX_CONFLICT,
};

/// A triangle and a fixed sample are on the same face when their normals agree to this cosine (30 degrees).
const SAME_FACE_COS: f64 = 0.866;
/// Fewest triangles a scan may have.
const MIN_TRIANGLES: usize = 4;
/// Most samples per scan in the coarse stage.
const SAMPLE_CAP: usize = 3000;
/// Samples of each scan for the conflict test, and the seed of their stream.
const CONFLICT_SAMPLES: usize = 3000;
const CONFLICT_SEED: u64 = 0xC0F1_1C75;
/// Fewest conflict samples a scan must yield (`Mesh::sample_points` can return fewer than asked when
/// a picked triangle has no normal); with fewer the conflicts are unknown (NaN), never 0.
const MIN_CONFLICT_SAMPLES: usize = CONFLICT_SAMPLES / 2;
/// Moving samples used to score the unrefined candidates.
const SHORTLIST_SAMPLES: usize = 1500;
/// Candidates refined by ICP: the best 8 distinct ones, and up to 16 when more score within
/// `SHORTLIST_SLACK` of the eighth (a near-tie must not be cut by the order of a sort).
const SHORTLIST: usize = 8;
const SHORTLIST_MAX: usize = 16;
const SHORTLIST_SLACK: f64 = 1.25;
/// Alternatives listed.
const MAX_ALTERNATIVES: usize = 7;
/// Tolerance of the shortlist score, as a fraction of the diagonal of the fixed scan.
const COARSE_TAU: f64 = 0.06;
/// Cut distances of the two refinement passes, as fractions of the diagonal.
const REFINE_CUTS: [f64; 2] = [0.05, 0.015];
/// Range of the tolerance (fractions of the diagonal).
const TAU_RANGE: (f64, f64) = (0.0015, 0.02);
/// The pose's own tolerance: this many times the median distance of its matched points.
const TAU_FACTOR: f64 = 5.0;
/// The tolerance never exceeds this many times the noise of the pair (the noise estimates of the two
/// scans combined): a tolerance made of the residual of the pose alone is passed by that pose by
/// construction (a pose on other regions of a lumpy surface has a residual of 0.6 mm among scans
/// of noise 0.05 mm, and a tolerance five times that).
const TAU_SIGMAS: f64 = 4.0;
/// With no trustworthy noise estimate (a coarse mesh) the cap is this share of the mean edge length
/// of the coarser scan: the chord error between two tessellations of a surface is a small part of the
/// edge (sag = edge squared over eight radii of curvature, the radius being several edges at least).
const TAU_EDGE: f64 = 0.05;
/// Poses closer than this fraction of the diagonal (largest displacement of the corners of the
/// moving scan's box) are one pose.
const SAME_POSE_BEFORE: f64 = 0.03;
const SAME_POSE_AFTER: f64 = 0.02;

/// Settings of [`pair_coarse`].
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PairOptions {
    /// Distance (mm) within which a point of one scan counts as lying on the other; `None` takes
    /// it from the residual of the best pose, at most four times the noise of the two scans (between 0.15 % and 2 % of the diagonal).
    pub tolerance: Option<f64>,
    /// How far the probes displace a pose, as a fraction of the extent of the moving scan (default
    /// 0.2). For tests: a start displaced far beyond reach has no pairs and counts as failed.
    pub probe_step: Option<f64>,
    /// A pose to start from (moving -> fixed): the descriptor stage is skipped and only this pose
    /// and the 24 principal-axes poses are tried, so a roughly placed pair costs a fraction of a
    /// search. `None` (the default) is the full search.
    pub given: Option<Transform>,
    /// The noise estimates of the moving and of the fixed scan, when the caller has them already
    /// (`align.global` estimates each scan once, not once per pair); `None` estimates them.
    pub noises: Option<(ScanNoise, ScanNoise)>,
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
    /// Noise (mm) of the pair: the noise estimates of the two scans combined, 0 when either is not
    /// trustworthy (see [`ScanNoise`]: the default tolerance then has no noise cap); otherwise the
    /// default tolerance is at most four times it.
    pub noise: f64,
    /// Share (0..1) of the moving scan's area on the fixed scan.
    pub overlap_moving: f64,
    /// Share (0..1) of the fixed scan's area on the moving scan.
    pub overlap_fixed: f64,
    /// Share (0..1) of the moving scan's area that lies where the fixed scan is, but off it.
    pub conflict_moving: f64,
    /// Share (0..1) of the fixed scan's area that lies where the moving scan is, but off it.
    pub conflict_fixed: f64,
    /// Does either scan contradict the best pose by more than [`UNIQUE_MAX_CONFLICT`]?
    pub contradicted: bool,
    /// Why the orientation of the triangles is not trusted (a scan wound inconsistently or turned
    /// inside out, or the two scans facing opposite ways where they meet), `None` when it is; the
    /// pose is then never `unique`. See `orientation` for what cannot be detected.
    pub orientation_issue: Option<String>,
    /// RMS distance (mm) of the matched part of the moving scan (0 when nothing matched).
    pub rms: f64,
    /// Stage that proposed the best pose.
    pub method: CoarseMethod,
    /// Different poses that fit about as well as the best one, best first.
    pub alternatives: Vec<PairCandidate>,
    /// Candidate poses scored.
    pub candidates: usize,
    /// Candidates that ICP could refine and that were compared (fewer than two: never certain).
    pub refined: usize,
    /// How firmly the matched part pins the pose, 0 (it can slide or turn freely) and up.
    pub stability: f64,
    /// Displaced starts tried for a weakly pinned pose (0 for a pinned or a free one).
    pub probed: usize,
    /// Of those, the ones whose refinement could not run (no pairs within the trimming distance); a pose with any is never `unique`.
    pub probes_failed: usize,
    /// Of the completed ones, the starts that ended in a different pose that fits as well (they are alternatives).
    pub slides: usize,
    /// Spread of the conflicts among the starts that came back (shares of the moving scan's area).
    pub probe_noise: f64,
    /// Clearance below which a start that settled elsewhere counts as a slide: the larger of 0.02 and 8 times the spread.
    pub probe_threshold: f64,
    /// Largest clearance among the starts counted as slides (0 when none).
    pub probe_slide_clearance: f64,
    /// Smallest clearance of the pose over the displaced starts that ended elsewhere; the
    /// ambiguity margin is the bound (`None`: all came back).
    pub probe_margin: Option<f64>,
    /// Largest distance (mm) a displaced start ended from the pose, among those that came back.
    pub probe_return_gap: f64,
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
    /// A pose a displaced start of the best one settled in, as good as it: an alternative whatever its clearance.
    slide: bool,
}

struct Scene<'a> {
    moving: &'a Mesh,
    mov: Samples,
    fix: Samples,
    fixed: Target<'a>,
    /// The moving scan as a target (border-aware), for the fixed scan's side of every measure.
    moving_target: Target<'a>,
    /// Area-uniform samples of each scan for the conflict test: as many for a small scan as for a
    /// large one (the coarse samples are as dense as the larger scan allows, a small patch gets a few
    /// hundred of them and a share of them is a few samples).
    conflict_mov: Vec<Point3>,
    conflict_fix: Vec<Point3>,
    /// What the triangles of each scan say about their orientation.
    winding_moving: orientation::Winding,
    winding_fixed: orientation::Winding,
    /// The moving samples as a point-only mesh, the source of the short ICP passes.
    mov_cloud: Mesh,
    diag: f64,
    spacing: f64,
    /// Noise (mm) of the pair: the estimates of the two scans combined as a root of squares; `None`
    /// when an estimate is not trustworthy (the tolerance then has no noise cap).
    noise: Option<f64>,
    /// Mean edge length (mm) of the coarser scan, taken as an equilateral triangle of the mean area.
    edge: f64,
}

/// Mean edge length of a mesh of `area` (mm): that of the equilateral triangle of the mean area.
fn mean_edge(m: &Mesh, area: f64) -> f64 {
    (4.0 * area / (m.triangle_count().max(1) as f64 * 3.0_f64.sqrt())).sqrt()
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
    fn new(
        moving: &'a Mesh,
        fixed: &'a Mesh,
        noises: Option<(ScanNoise, ScanNoise)>,
        cancel: &Cancel,
    ) -> Result<Self, AlignError> {
        let (am, af) = (check_scan(moving, "moving")?, check_scan(fixed, "fixed")?);
        let diag = fixed.bbox().diagonal();
        if !(diag.is_finite() && diag > 0.0) {
            return Err(AlignError::Degenerate("the fixed scan has no extent".into()));
        }
        let spacing = (am.max(af) / SAMPLE_CAP as f64).sqrt();
        let few = || {
            if cancel.is_cancelled() {
                AlignError::Cancelled
            } else {
                AlignError::NotEnough("a scan has too few usable triangles to sample".into())
            }
        };
        let mov =
            sample::sample(moving, sample::count_for(am, spacing, SAMPLE_CAP), spacing, 1, cancel).ok_or_else(few)?;
        let fix =
            sample::sample(fixed, sample::count_for(af, spacing, SAMPLE_CAP), spacing, 2, cancel).ok_or_else(few)?;
        let mov_cloud = Mesh { positions: mov.pts.clone(), triangles: Vec::new() };
        let (nm, nf) = match noises {
            Some(n) => n,
            None => {
                let m = ScanNoise::of(moving);
                if cancel.is_cancelled() {
                    return Err(AlignError::Cancelled);
                }
                (m, ScanNoise::of(fixed))
            }
        };
        if cancel.is_cancelled() {
            return Err(AlignError::Cancelled);
        }
        let noise = noise::pair_noise(&nm, &nf);
        let conflict_samples = |m: &Mesh, stream: u64| -> Vec<Point3> {
            m.sample_points(CONFLICT_SAMPLES, &mut Rng::new(CONFLICT_SEED ^ stream))
                .into_iter()
                .map(|(p, _)| p)
                .collect()
        };
        Ok(Scene {
            moving,
            mov,
            fix,
            fixed: Target::new(fixed, diag),
            moving_target: Target::new(moving, moving.bbox().diagonal()),
            conflict_mov: conflict_samples(moving, 3),
            conflict_fix: conflict_samples(fixed, 4),
            winding_moving: orientation::winding(moving, moving.bbox().diagonal()),
            winding_fixed: orientation::winding(fixed, diag),
            mov_cloud,
            diag,
            spacing,
            noise,
            edge: mean_edge(moving, am).max(mean_edge(fixed, af)),
        })
    }

    /// The given pose and the principal-axes poses (no descriptors).
    fn given_hypotheses(&self, given: &Transform) -> Vec<Cand> {
        let mut out = vec![Cand { t: *given, method: CoarseMethod::Given }];
        out.extend(
            candidates::principal_candidates(&self.mov.pts, &self.fix.pts)
                .into_iter()
                .map(|t| Cand { t, method: CoarseMethod::PrincipalAxes }),
        );
        out
    }

    fn hypotheses(&self, cancel: &Cancel) -> Result<Vec<Cand>, AlignError> {
        let mut out: Vec<Cand> = candidates::principal_candidates(&self.mov.pts, &self.fix.pts)
            .into_iter()
            .map(|t| Cand { t, method: CoarseMethod::PrincipalAxes })
            .collect();
        out.extend(self.on_surface_variants(&out));
        let poses = ransac::descriptor_poses(&self.mov, &self.fix, self.spacing, cancel)?;
        out.extend(poses.into_iter().map(|t| Cand { t, method: CoarseMethod::Descriptors }));
        Ok(out)
    }

    /// Each candidate again with the moving centroid moved from the fixed centroid onto the fixed
    /// surface (to the point of it nearest to that centroid): a patch of a thick or a thin part has
    /// its centroid inside the whole part, out of reach of ICP or midway between two faces, where
    /// it hovers; on the surface it is within reach and flush.
    fn on_surface_variants(&self, cands: &[Cand]) -> Vec<Cand> {
        let n = self.fix.pts.len().max(1) as f64;
        let centroid = Point3::from(self.fix.pts.iter().map(|p| p.coords).sum::<surfacecraft_geom::Vec3>() / n);
        let Some(hit) = self.fixed.bvh.closest(&centroid, f64::INFINITY) else { return Vec::new() };
        if hit.distance <= TAU_RANGE.0 * self.diag {
            return Vec::new();
        }
        let shift = Transform::translation(hit.point - centroid);
        cands.iter().map(|c| Cand { t: shift.then_after(&c.t), method: c.method }).collect()
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
        let mut cut = f64::INFINITY;
        for (msac, _, c) in scored {
            let room = kept.len() < SHORTLIST || (kept.len() < SHORTLIST_MAX && msac <= cut);
            if room && kept.iter().all(|k| pose_gap(&k.t, &c.t, &probes) > SAME_POSE_BEFORE * self.diag) {
                kept.push(c);
                if kept.len() == SHORTLIST {
                    cut = msac * SHORTLIST_SLACK + 1e-12;
                }
            }
        }
        kept
    }

    /// ICP of `source` onto the fixed scan with smoothed normals: the plane-fitted normal of the
    /// nearest fixed sample where it lies on the same face as the triangle hit (their normals within
    /// 30 degrees), the triangle's own normal elsewhere (at an edge). On a fine noisy fixed scan the
    /// raw normals tilt by about noise over edge length, and a patch that nothing holds then walks
    /// off by the tilts alone.
    fn icp(&self, source: &Mesh, t: &Transform, opts: &IcpOptions, cancel: &Cancel) -> Result<IcpReport, AlignError> {
        let normal = |h: &SurfaceHit| {
            let raw = self.fixed.bvh.triangle_normal(h.triangle)?;
            let near =
                self.fix.tree.nearest(&h.point).and_then(|(j, _)| Some((self.fix.nrm.get(j)?, self.fix.fit.get(j)?)));
            Some(match near {
                Some((smooth, fit)) if raw.dot(smooth) > SAME_FACE_COS => *fit,
                _ => raw,
            })
        };
        icp_with_normals(source, &self.fixed.bvh, t, opts, cancel, &normal)
    }

    /// Two ICP passes from `c`; `None` when ICP cannot run from there (the candidate is dropped).
    fn refine(&self, c: &Cand, cancel: &Cancel) -> Result<Option<Cand>, AlignError> {
        self.refine_cuts(c, REFINE_CUTS.map(|f| f * self.diag), cancel)
    }

    /// [`Scene::refine`] with the cut distances (mm) of the two passes given.
    fn refine_cuts(&self, c: &Cand, cuts: [f64; 2], cancel: &Cancel) -> Result<Option<Cand>, AlignError> {
        let mut t = c.t;
        for cut in cuts {
            let opts =
                IcpOptions { max_iterations: 25, tolerance: 1e-3, max_distance: Some(cut), max_samples: SAMPLE_CAP };
            match self.icp(&self.mov_cloud, &t, &opts, cancel) {
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

    /// The tolerance of pose `t`: the user's, or the smaller of five times the median residual of
    /// the pose and `TAU_SIGMAS` times the noise of the pair (the pose may tighten the tolerance but
    /// never loosen it past what the data allow), within the range.
    fn tolerance_at(&self, t: &Transform, user: Option<f64>) -> f64 {
        let (lo, hi) = (TAU_RANGE.0 * self.diag, TAU_RANGE.1 * self.diag);
        if let Some(u) = user {
            return u;
        }
        let dist: Vec<f64> = surfacecraft_geom::par::map_slice(&self.mov.pts, |p| {
            self.fixed.bvh.closest(&t.apply(p), hi).map_or(f64::NAN, |h| h.distance)
        });
        let inside: Vec<f64> = dist.into_iter().filter(|d| d.is_finite()).collect();
        let from_pose = TAU_FACTOR * median(inside);
        from_pose.min(self.noise.map_or(TAU_EDGE * self.edge, |n| TAU_SIGMAS * n)).clamp(lo, hi)
    }

    /// The tolerance of the best refined candidate (see [`Scene::tolerance_at`]).
    fn tolerance(&self, refined: &[Cand], user: Option<f64>) -> f64 {
        let tau0 = TAU_RANGE.1 * self.diag;
        let best = refined.iter().min_by(|a, b| self.msac_at(&a.t, tau0).total_cmp(&self.msac_at(&b.t, tau0)));
        match best {
            Some(b) => self.tolerance_at(&b.t, user),
            None => user.unwrap_or(TAU_RANGE.0 * self.diag),
        }
    }

    /// Refined candidates scored under `tau`, best first, equal poses merged.
    fn rank(&self, refined: Vec<Cand>, tau: f64) -> Vec<Ranked> {
        let mut all: Vec<(usize, Ranked)> = refined
            .into_iter()
            .enumerate()
            .map(|(i, cand)| {
                (i, Ranked { fit: fit_of(&self.fixed, &self.mov.pts, &cand.t, tau, true), cand, slide: false })
            })
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
        let Some(top) = ranked.first().map(|r| r.fit) else { return Ok(ranked) };
        let mut again = Vec::with_capacity(ranked.len());
        for r in ranked {
            let mut cand = r.cand;
            if clearance(&top, &r.fit) < AMBIGUITY_MARGIN {
                cand.t = self.converge(&cand.t, tau, cancel)?;
            }
            again.push(cand);
        }
        Ok(self.rank(again, tau))
    }

    /// `t` run to convergence by ICP on the moving samples (unchanged when ICP cannot run).
    fn converge(&self, t: &Transform, tau: f64, cancel: &Cancel) -> Result<Transform, AlignError> {
        let opts =
            IcpOptions { max_iterations: 100, tolerance: 1e-4, max_distance: Some(3.0 * tau), max_samples: SAMPLE_CAP };
        match self.icp(&self.mov_cloud, t, &opts, cancel) {
            Ok(rep) => Ok(rep.transform),
            Err(AlignError::Cancelled) => Err(AlignError::Cancelled),
            Err(_) => Ok(*t),
        }
    }

    /// The best pose polished by an ICP on the whole moving scan (kept only when it scores no
    /// worse), with the iterations it took and whether it converged.
    fn polish(&self, best: &Ranked, tau: f64, cancel: &Cancel) -> Result<(Ranked, usize, bool), AlignError> {
        let opts = IcpOptions { max_distance: Some(3.0 * tau), ..IcpOptions::default() };
        let rep = match self.icp(self.moving, &best.cand.t, &opts, cancel) {
            Ok(rep) => rep,
            Err(AlignError::Cancelled) => return Err(AlignError::Cancelled),
            Err(_) => return Ok((Ranked { cand: best.cand, fit: best.fit, slide: false }, 0, false)),
        };
        let fit = fit_of(&self.fixed, &self.mov.pts, &rep.transform, tau, true);
        if fit.score() >= best.fit.score() - 1e-9 {
            Ok((
                Ranked { cand: Cand { t: rep.transform, method: best.cand.method }, fit, slide: false },
                rep.iterations,
                rep.converged,
            ))
        } else {
            Ok((Ranked { cand: best.cand, fit: best.fit, slide: false }, rep.iterations, rep.converged))
        }
    }

    /// Why the orientation of the triangles is not trusted at pose `t` (see `orientation`), if it is
    /// not: either scan's own winding, or the normals of the two opposing where the scans meet.
    fn orientation_issue(&self, t: &Transform, tau: f64) -> Option<String> {
        if let Some(why) = self.winding_moving.problem() {
            return Some(format!("the moving scan: {why}"));
        }
        if let Some(why) = self.winding_fixed.problem() {
            return Some(format!("the fixed scan: {why}"));
        }
        let opposed = orientation::opposed_share(&self.fixed.bvh, &self.mov.pts, &self.mov.nrm, t, tau);
        (!(opposed <= orientation::MAX_OPPOSED))
            .then(|| "the two scans face opposite ways where they meet: one is turned inside out".to_string())
    }

    /// Per pose, how the fixed scan's samples fit the moving scan placed there (the other side of
    /// [`fit_of`] on the moving samples): the share within `tau` and the conflicts.
    fn fits_fixed(&self, poses: &[Transform], tau: f64) -> Vec<Fit> {
        poses.iter().map(|t| fit_of(&self.moving_target, &self.fix.pts, &t.inverse(), tau, true)).collect()
    }

    /// The conflicts of both scans at pose `t` on the dedicated samples: (moving, fixed). A point
    /// of one scan is in conflict when it lies in the free space of the other scan: the other scan
    /// is there (the closest point of it is not at its border) but more than twice `tau` away and
    /// the point is in front of it. Surface beyond the border of the other scan, and surface behind
    /// it, is neutral: both numbers are measured inside the footprint of the other scan.
    fn conflicts_at(&self, t: &Transform, tau: f64) -> (f64, f64) {
        if self.conflict_mov.len() < MIN_CONFLICT_SAMPLES || self.conflict_fix.len() < MIN_CONFLICT_SAMPLES {
            return (f64::NAN, f64::NAN);
        }
        let moving = free_space_conflicts(&self.fixed, &self.conflict_mov, t, tau);
        let fixed = free_space_conflicts(&self.moving_target, &self.conflict_fix, &t.inverse(), tau);
        (moving, fixed)
    }
}

fn finite_or_zero(x: f64) -> f64 {
    if x.is_finite() { x } else { 0.0 }
}

fn alternative(r: &Ranked, other: &Fit) -> PairCandidate {
    PairCandidate {
        transform: r.cand.t,
        overlap_moving: r.fit.share,
        overlap_fixed: other.share,
        confidence: 0.0,
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
    let scene = Scene::new(moving, fixed, opts.noises, cancel)?;
    scene.check_tolerance(opts.tolerance)?;
    let hyps = match &opts.given {
        Some(g) => scene.given_hypotheses(g),
        None => scene.hypotheses(cancel)?,
    };
    solve(&scene, hyps, opts, cancel)
}

/// Everything after the candidates: shortlist, refinement, ranking, verdict. A separate step so
/// that a test can hand it a chosen set of candidates.
fn solve(scene: &Scene, hyps: Vec<Cand>, opts: &PairOptions, cancel: &Cancel) -> Result<PairResult, AlignError> {
    let candidates = hyps.len();
    let mut refined = Vec::new();
    for c in scene.shortlist(hyps) {
        refined.extend(scene.refine(&c, cancel)?);
    }
    if refined.is_empty() {
        return Err(AlignError::Degenerate("no candidate pose could be refined".into()));
    }
    let refined_count = refined.len();
    let tau = scene.tolerance(&refined, opts.tolerance);
    let ranked = scene.rank(refined, tau);
    let mut ranked = scene.settle(ranked, tau, cancel)?;
    if ranked.is_empty() {
        return Err(AlignError::Degenerate("no candidate pose survived".into()));
    }
    let first = ranked.remove(0);
    let (best, icp_iterations, icp_converged) = scene.polish(&first, tau, cancel)?;
    let probe = scene.probe_at(&best, tau, cancel, opts.probe_step.unwrap_or(probe::PROBE_STEP))?;
    let probes = corners(&scene.mov.pts);
    for r in probe.slides.iter() {
        if ranked.iter().all(|k| pose_gap(&k.cand.t, &r.cand.t, &probes) > SAME_POSE_AFTER * scene.diag) {
            ranked.push(Ranked { cand: r.cand, fit: r.fit, slide: true });
        }
    }
    let poses: Vec<Transform> = std::iter::once(best.cand.t).chain(ranked.iter().map(|r| r.cand.t)).collect();
    let shares = scene.fits_fixed(&poses, tau);
    let checks =
        verdict::ProbeFacts { attempted: probe.attempted, completed: probe.completed, slides: probe.slides.len() };
    let conflicts = scene.conflicts_at(&best.cand.t, tau);
    let orientation = scene.orientation_issue(&best.cand.t, tau);
    let mut result =
        conclude(&best, &ranked, tau, &shares, (conflicts, orientation), refined_count, probe.stability, checks)?;
    result.probed = probe.attempted;
    result.probes_failed = probe.attempted - probe.completed;
    result.slides = probe.slides.len();
    result.probe_noise = probe.noise;
    result.probe_threshold = probe.threshold;
    result.probe_slide_clearance = probe.slide_clearance;
    result.probe_margin = probe.margin;
    result.probe_return_gap = probe.returned_gap;
    result.candidates = candidates;
    result.noise = scene.noise.unwrap_or(0.0);
    result.icp_iterations = icp_iterations;
    result.icp_converged = icp_converged;
    Ok(result)
}

/// The verdict and the result from the best pose and the other distinct ones; `fixed[0]` is how the
/// fixed scan fits the moving one at the best pose, `fixed[k]` the same at `others[k - 1]`.
fn conclude(
    best: &Ranked,
    others: &[Ranked],
    tau: f64,
    fixed: &[Fit],
    evidence: ((f64, f64), Option<String>),
    refined: usize,
    stable: f64,
    probes: verdict::ProbeFacts,
) -> Result<PairResult, AlignError> {
    if !best.cand.t.to_rows().iter().flatten().all(|v| v.is_finite()) {
        return Err(AlignError::Degenerate("the pose is not finite".into()));
    }
    let fixed_fit = fixed.first().copied().unwrap_or(Fit { share: 0.0, conflict: 0.0, rms: f64::INFINITY, msac: 0.0 });
    let (conflicts, orientation) = evidence;
    let overlap_fixed = fixed_fit.share;
    let overlap = best.fit.share.max(overlap_fixed);
    let nearest = others
        .iter()
        .map(|r| clearance(&best.fit, &r.fit))
        .fold(None, |m: Option<f64>, c| Some(m.map_or(c, |x| x.min(c))));
    let facts = verdict::Facts {
        best: &best.fit,
        overlap,
        tau,
        clearance: nearest,
        refined,
        stable,
        conflict: conflicts.0.max(conflicts.1),
        orientation_ok: orientation.is_none(),
        probes,
    };
    let (verdict, confidence) = verdict::judge(&facts);
    let alternatives: Vec<PairCandidate> = others
        .iter()
        .zip(fixed.iter().skip(1))
        .filter(|(r, _)| {
            verdict == PairVerdict::Ambiguous && (r.slide || clearance(&best.fit, &r.fit) < AMBIGUITY_MARGIN)
        })
        .take(MAX_ALTERNATIVES)
        .map(|(r, s)| alternative(r, s))
        .collect();
    Ok(PairResult {
        transform: best.cand.t,
        verdict,
        confidence,
        tolerance: tau,
        noise: 0.0,
        overlap_moving: best.fit.share,
        overlap_fixed,
        conflict_moving: conflicts.0,
        conflict_fixed: conflicts.1,
        contradicted: !(conflicts.0.max(conflicts.1) <= UNIQUE_MAX_CONFLICT),
        orientation_issue: orientation,
        rms: finite_or_zero(best.fit.rms),
        method: best.cand.method,
        alternatives,
        candidates: 0,
        refined,
        stability: stable,
        probed: 0,
        probes_failed: 0,
        slides: 0,
        probe_noise: 0.0,
        probe_threshold: 0.0,
        probe_slide_clearance: 0.0,
        probe_margin: None,
        probe_return_gap: 0.0,
        icp_iterations: 0,
        icp_converged: false,
    })
}

#[cfg(test)]
mod tests;
