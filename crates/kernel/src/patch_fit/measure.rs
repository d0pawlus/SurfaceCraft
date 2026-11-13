//! Measurement of a fitted patch (design doc §4.3): the gap, angle and curvature error along every
//! side on the dense samples, the fold test, the deviation of the data, and where the misses lie.
//! Also re-projects the interior points onto the patch for the next round.
use super::geom::{V2, add, cross, dot, scale, sub, unit};
use super::reference::RefSample;
use super::rows::{Layout, MIN_DATA, Sample, SideSamples};
use super::start::{Interior, Loop, Pt, SideData};
use super::{Continuity, G0_TARGET, G1_TARGET_DEG, G2_TARGET_REL, SideReport};
use crate::nurbs::SurfaceProbe;
use crate::record::brep::Surface;
use crate::vpatch::derivatives;
use crate::{Result, poll};
use surfacecraft_geom::Cancel;

/// `S_d` pointing back over the reference face at this cosine or more counts as a fold.
const FOLD_COS: f64 = 0.1;
/// Samples (closest-point queries) between two looks at the cancellation flag.
const POLL_EVERY: usize = 256;
/// The patch normal at this cosine or less against the outline's plane normal counts as turned over
/// (the patch is no longer a graph over the plane of the outline: it folds).
const INTERIOR_FOLD_COS: f64 = 0.0;
/// Interior check: grid points per direction over the outline's box, at least and at most.
const INTERIOR_GRID: [usize; 2] = [16, 40];

/// What a round is measured against.
pub(super) struct Goal<'a> {
    /// Target of the data deviation (max), mm.
    pub tolerance: f64,
    /// Sides whose gap is as small as their own accuracy allows: it does not steer the growth.
    pub frozen: &'a [bool],
    /// The caller's cancellation flag.
    pub cancel: &'a Cancel,
}

/// Everything measured of one round's surface.
pub(super) struct Measure {
    pub sides: Vec<SideReport>,
    /// Parameters of the closest points of every dense side sample.
    pub feet: Vec<Vec<V2>>,
    /// The samples' own (fixed) parameters, from the flattening of the sides.
    pub starts: Vec<Vec<V2>>,
    pub rms: f64,
    pub p99: f64,
    pub max: f64,
    pub points: usize,
    /// Data points that lie outside the outline (not measured, not fitted).
    pub outside: usize,
    /// Largest distance of a hard point or curve sample from the patch.
    pub hard_max: f64,
    /// Hard points and curves on the patch.
    pub hard_ok: bool,
    /// Largest distance of a soft point from the patch, mm (`None` without soft points).
    pub soft_max: Option<f64>,
    /// Per side: its angle, curvature and fold targets are met (true for a soft side).
    pub rest_ok: Vec<bool>,
    /// Per side: the gap and the larger of the other misses, in units of their targets (a fold
    /// counts as 2); `[0, 0]` for a soft side.
    pub side_terms: Vec<[f64; 2]>,
    /// Miss of the hard points and curves in units of their target.
    pub hard_term: f64,
    /// Largest data distance in units of the tolerance.
    pub data_term: f64,
    /// Misses by direction: the largest root-mean-square miss over slabs of the parameter domain.
    pub miss: [f64; 2],
    /// The patch turns over inside the outline: its normal leaves the half space of the outline's
    /// plane normal on the interior grid.
    pub interior_folded: bool,
}

impl Measure {
    /// Largest miss of the hard conditions (sides, interior points and curves) in units of their
    /// targets, every gap counted: at most 1 when all are met.
    pub fn side_miss(&self) -> f64 {
        let fold = if self.interior_folded { FOLD_TERM } else { 0.0 };
        self.side_terms.iter().flatten().fold(self.hard_term.max(fold), |a, b| a.max(*b))
    }

    /// Worst miss, the data included, in units of the targets; the gap of a `frozen` side (as close
    /// to its polyline as its own accuracy allows) is left out.
    pub fn worst(&self, frozen: &[bool]) -> f64 {
        let mut w = self.hard_term.max(self.data_term);
        if self.interior_folded {
            w = w.max(FOLD_TERM);
        }
        for (k, t) in self.side_terms.iter().enumerate() {
            let gap = if frozen.get(k).copied().unwrap_or(false) { 0.0 } else { t[0] };
            w = w.max(gap).max(t[1]);
        }
        w
    }

    /// Largest relative normal curvature error of the hard G2 sides (0 without one).
    pub fn curvature_miss(&self) -> f64 {
        self.sides.iter().filter(|s| !s.soft).filter_map(|s| s.curvature_error_max).fold(0.0, f64::max)
    }

    /// Every hard side meets every target (gap, angle, curvature, no fold) and the interior points
    /// and curves lie on the patch: no allowance for sides whose gap stopped improving.
    pub fn targets_met(&self) -> bool {
        self.hard_ok
            && !self.interior_folded
            && self
                .sides
                .iter()
                .zip(&self.rest_ok)
                .all(|(s, rest)| s.soft || (*rest && !s.folded && s.gap_max <= G0_TARGET))
    }

    /// Whether the rounds can stop: every side within its targets and the interior on the patch,
    /// where the gap of a `frozen` side (as close as its own accuracy allows) may be up to ten times
    /// the target. This steers the stopping only; [`Measure::targets_met`] is what is reported.
    pub fn sides_ok(&self, frozen: &[bool]) -> bool {
        self.hard_ok
            && !self.interior_folded
            && self.sides.iter().zip(&self.rest_ok).enumerate().all(|(k, (s, rest))| {
                let lenient = frozen.get(k).copied().unwrap_or(false) && s.gap_max < 10.0 * G0_TARGET;
                s.soft || (*rest && (s.gap_max <= G0_TARGET || lenient))
            })
    }
}

/// A fold counts as this many targets missed.
const FOLD_TERM: f64 = 2.0;

/// Per side, the parameter direction (0 for u, 1 for v) it mostly runs along: the net is refined
/// along it to follow the side more closely.
pub(super) fn main_directions(lp: &Loop) -> Vec<usize> {
    lp.sides
        .iter()
        .map(|s| {
            let extent = |d: usize| s.uv.windows(2).map(|w| (w[1][d] - w[0][d]).abs()).sum::<f64>();
            usize::from(extent(1) > extent(0))
        })
        .collect()
}

/// Misses (in units of their targets) collected in slabs of the parameter domain.
struct Slabs {
    cells: [usize; 2],
    domain: [[f64; 2]; 2],
    sum: [Vec<f64>; 2],
    count: [Vec<f64>; 2],
}

impl Slabs {
    fn new(layout: &Layout) -> Self {
        let cells = [layout.net[0].saturating_sub(3).max(1), layout.net[1].saturating_sub(3).max(1)];
        Self {
            cells,
            domain: layout.domain,
            sum: [vec![0.0; cells[0]], vec![0.0; cells[1]]],
            count: [vec![0.0; cells[0]], vec![0.0; cells[1]]],
        }
    }

    fn add(&mut self, uv: V2, miss: f64) {
        if !miss.is_finite() {
            return;
        }
        for d in 0..2 {
            let w = self.domain[d][1] - self.domain[d][0];
            let last = self.cells[d].saturating_sub(1);
            let k = ((uv[d] - self.domain[d][0]) / w * self.cells[d] as f64).floor();
            let k = (k.max(0.0) as usize).min(last);
            if let (Some(s), Some(c)) = (self.sum[d].get_mut(k), self.count[d].get_mut(k)) {
                *s += miss * miss;
                *c += 1.0;
            }
        }
    }

    fn score(&self) -> [f64; 2] {
        let one = |d: usize| {
            self.sum[d]
                .iter()
                .zip(&self.count[d])
                .filter(|(_, c)| **c > 0.0)
                .map(|(s, c)| (s / c).sqrt())
                .fold(0.0_f64, f64::max)
        };
        [one(0), one(1)]
    }
}

/// Re-projects `pts` onto the patch and returns their distances; points outside the parameter
/// domain (ignored by the solver) are left alone.
fn reproject(
    probe: &SurfaceProbe,
    layout: &Layout,
    pts: &mut [Pt],
    slabs: &mut Slabs,
    target: f64,
    cancel: &Cancel,
) -> Result<Vec<f64>> {
    let mut out = Vec::with_capacity(pts.len());
    for (i, p) in pts.iter_mut().enumerate() {
        if i % POLL_EVERY == 0 {
            poll(cancel)?;
        }
        if !layout.inside(p.uv) {
            continue;
        }
        let (d, _, f) = probe.closest_uv(p.xyz)?;
        p.uv = f;
        slabs.add(f, d / target);
        out.push(d);
    }
    Ok(out)
}

/// Re-projects the data onto the patch: the distances of those whose foot lies inside the outline
/// (the others are marked `out`, and counted), and their misses go to `slabs`.
fn reproject_data(
    probe: &SurfaceProbe,
    lp: &Loop,
    pts: &mut [Pt],
    slabs: &mut Slabs,
    target: f64,
    cancel: &Cancel,
) -> Result<(Vec<f64>, usize)> {
    let mut dists = Vec::with_capacity(pts.len());
    let mut outside = 0usize;
    for (i, p) in pts.iter_mut().enumerate() {
        if i % POLL_EVERY == 0 {
            poll(cancel)?;
        }
        let (d, _, f) = probe.closest_uv(p.xyz)?;
        p.uv = f;
        p.out = !lp.contains(f);
        if p.out {
            outside += 1;
            continue;
        }
        slabs.add(f, d / target);
        dists.push(d);
    }
    Ok((dists, outside))
}

/// The running maxima of one side's samples.
#[derive(Default)]
struct SideAcc {
    gap_max: f64,
    angle_max: f64,
    curv_max: f64,
    folded: bool,
}

/// Everything measured of one side.
struct SideResult {
    report: SideReport,
    feet: Vec<V2>,
    starts: Vec<V2>,
    rest_ok: bool,
    terms: [f64; 2],
}

/// What measuring a side's samples needs besides the sample.
struct SampleCx<'a> {
    surface: &'a Surface,
    lp: &'a Loop,
    /// Smallest reference curvature the relative error is taken against.
    floor: f64,
}

/// What a sample of a G1/G2 side adds: the angle against the reference normal, the fold and, for
/// G2, the normal curvature error; returns the miss in units of the targets.
fn sample_reference(
    cx: &SampleCx,
    acc: &mut SideAcc,
    sd: &SideData,
    smp: &Sample,
    r: &RefSample,
    f: V2,
) -> Result<f64> {
    let der = derivatives(cx.surface, f)?;
    let (su, sv) = (der[1], der[2]);
    let angle = match unit(cross(su, sv)) {
        Some(n) => dot(n, r.normal).abs().min(1.0).acos().to_degrees(),
        None => 90.0,
    };
    acc.angle_max = acc.angle_max.max(angle);
    let mut miss = angle / G1_TARGET_DEG;
    let sd_vec = add(scale(su, smp.dir[0]), scale(sv, smp.dir[1]));
    let (Some(w), Some(inward)) = (unit(sd_vec), cx.lp.affine.lift(smp.dir)) else { return Ok(miss) };
    // the in-face direction away from the edge
    let away = scale(inward, -1.0);
    if let Some(a) = unit(sub(away, scale(r.normal, dot(away, r.normal)))) {
        acc.folded |= dot(w, a) > FOLD_COS;
    }
    if sd.continuity == Continuity::G2 {
        let (d0, d1) = (smp.dir[0], smp.dir[1]);
        let sdd = add(add(scale(der[3], d0 * d0), scale(der[4], 2.0 * d0 * d1)), scale(der[5], d1 * d1));
        let kappa_patch = dot(sdd, r.normal) / dot(sd_vec, sd_vec);
        if let Some(kappa_ref) = r.curvature(w) {
            let rel = (kappa_patch - kappa_ref).abs() / kappa_ref.abs().max(cx.floor);
            if rel.is_finite() {
                acc.curv_max = acc.curv_max.max(rel);
                miss = miss.max(rel / G2_TARGET_REL);
            }
        }
    }
    Ok(miss)
}

fn measure_side(
    k: usize,
    surface: &Surface,
    probe: &SurfaceProbe,
    lp: &Loop,
    ss: &SideSamples,
    slabs: &mut Slabs,
    goal: &Goal,
) -> Result<SideResult> {
    let Some(sd) = lp.sides.get(k) else { return Err(crate::KernelError::Argument("a side is missing".into())) };
    let is_frozen = goal.frozen.get(k).copied().unwrap_or(false);
    let g1 = sd.continuity != Continuity::G0;
    let g2 = sd.continuity == Continuity::G2;
    let cx = SampleCx { surface, lp, floor: 1.0 / (4.0 * lp.diag.max(1e-9)) };
    let mut acc = SideAcc::default();
    let mut feet = Vec::with_capacity(ss.samples.len());
    for (i, smp) in ss.samples.iter().enumerate() {
        if i % POLL_EVERY == 0 {
            poll(goal.cancel)?;
        }
        let (d, _, f) = probe.closest_uv(smp.xyz)?;
        feet.push(f);
        acc.gap_max = acc.gap_max.max(d);
        let mut miss = if is_frozen { 0.0 } else { d / G0_TARGET };
        if let Some(r) = smp.refs.as_ref().filter(|_| g1) {
            miss = miss.max(sample_reference(&cx, &mut acc, sd, smp, r, f)?);
        }
        // a soft side is reported but does not steer the growth of the net
        if !sd.soft {
            slabs.add(f, miss);
        }
    }
    let rest_ok = (!g1 || acc.angle_max <= G1_TARGET_DEG) && (!g2 || acc.curv_max <= G2_TARGET_REL) && !acc.folded;
    let rest_term = (if g1 { acc.angle_max / G1_TARGET_DEG } else { 0.0 })
        .max(if g2 { acc.curv_max / G2_TARGET_REL } else { 0.0 })
        .max(if acc.folded { FOLD_TERM } else { 0.0 });
    let report = SideReport {
        continuity: sd.continuity,
        soft: sd.soft,
        gap_max: acc.gap_max,
        angle_max_deg: g1.then_some(acc.angle_max),
        curvature_error_max: g2.then_some(acc.curv_max),
        samples: ss.samples.len(),
        folded: acc.folded,
        snap_max: sd.snap,
        met: sd.soft || (rest_ok && acc.gap_max <= G0_TARGET),
    };
    Ok(SideResult {
        report,
        feet,
        starts: ss.samples.iter().map(|smp| smp.uv).collect(),
        rest_ok: rest_ok || sd.soft,
        terms: if sd.soft { [0.0, 0.0] } else { [acc.gap_max / G0_TARGET, rest_term] },
    })
}

/// Does the patch turn over inside the outline? The unit normal `S_u × S_v` is compared with the
/// normal of the outline's plane (oriented like the parameters) on a grid of the trimmed domain:
/// a point where they are at 90° or more apart, or where the patch has no normal, is a fold.
fn interior_folded(surface: &Surface, lp: &Loop, layout: &Layout, cancel: &Cancel) -> Result<bool> {
    let Some(n0) = lp.affine.normal() else { return Ok(false) };
    let cells = layout.net[0].max(layout.net[1]).saturating_sub(3).max(1);
    let n = (4 * cells).clamp(INTERIOR_GRID[0], INTERIOR_GRID[1]);
    for i in 0..n {
        poll(cancel)?;
        for j in 0..n {
            let t = |k: usize| (k as f64 + 0.5) / n as f64;
            let uv = [
                lp.bbox[0][0] + t(i) * (lp.bbox[0][1] - lp.bbox[0][0]),
                lp.bbox[1][0] + t(j) * (lp.bbox[1][1] - lp.bbox[1][0]),
            ];
            if !lp.contains(uv) || !layout.inside(uv) {
                continue;
            }
            let der = derivatives(surface, uv)?;
            match unit(cross(der[1], der[2])) {
                Some(nrm) if dot(nrm, n0) > INTERIOR_FOLD_COS => {}
                _ => return Ok(true),
            }
        }
    }
    Ok(false)
}

/// Distance statistics of the data: root-mean-square, 99th percentile and largest.
fn data_stats(dists: &mut Vec<f64>) -> (usize, f64, f64, f64) {
    if dists.len() < MIN_DATA {
        dists.clear();
    }
    dists.sort_by(|a, b| a.total_cmp(b));
    let points = dists.len();
    if points == 0 {
        return (0, 0.0, 0.0, 0.0);
    }
    let rms = (dists.iter().map(|d| d * d).sum::<f64>() / points as f64).sqrt();
    let k = ((0.99 * points as f64).ceil() as usize).clamp(1, points) - 1;
    (points, rms, dists.get(k).copied().unwrap_or(0.0), dists.last().copied().unwrap_or(0.0))
}

/// Re-projects the interior points and curves; returns the largest distance of a hard one and of a soft one.
fn measure_interior(
    probe: &SurfaceProbe,
    layout: &Layout,
    interior: &mut Interior,
    slabs: &mut Slabs,
    cancel: &Cancel,
) -> Result<(f64, Option<f64>)> {
    let mut hard_max = 0.0_f64;
    let hard = reproject(probe, layout, &mut interior.hard, slabs, G0_TARGET, cancel)?;
    hard_max = hard.iter().fold(hard_max, |m, d| m.max(*d));
    for c in interior.curves.iter_mut() {
        let d = reproject(probe, layout, c, slabs, G0_TARGET, cancel)?;
        hard_max = d.iter().fold(hard_max, |m, x| m.max(*x));
    }
    // soft points: re-projected for the next round and reported, they steer no growth
    let mut soft_max: Option<f64> = None;
    for p in interior.soft.iter_mut() {
        let (d, _, f) = probe.closest_uv(p.xyz)?;
        p.uv = f;
        soft_max = Some(soft_max.map_or(d, |m| m.max(d)));
    }
    Ok((hard_max, soft_max))
}

/// Measures `surface` against the sides, the interior and the tolerance; the interior's parameters
/// become the closest points on `surface`. The gap of a frozen side does not steer the growth.
pub(super) fn measure(
    surface: &Surface,
    probe: &SurfaceProbe,
    lp: &Loop,
    layout: &Layout,
    sides: &[SideSamples],
    interior: &mut Interior,
    goal: &Goal,
) -> Result<Measure> {
    let mut slabs = Slabs::new(layout);
    let n = sides.len();
    let (mut reports, mut feet, mut starts) = (Vec::with_capacity(n), Vec::with_capacity(n), Vec::with_capacity(n));
    let (mut rest_ok, mut side_terms) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for (k, ss) in sides.iter().enumerate() {
        let r = measure_side(k, surface, probe, lp, ss, &mut slabs, goal)?;
        reports.push(r.report);
        feet.push(r.feet);
        starts.push(r.starts);
        rest_ok.push(r.rest_ok);
        side_terms.push(r.terms);
    }
    let (hard_max, soft_max) = measure_interior(probe, layout, interior, &mut slabs, goal.cancel)?;
    let (mut dists, outside) = reproject_data(probe, lp, &mut interior.data, &mut slabs, goal.tolerance, goal.cancel)?;
    let (points, rms, p99, max) = data_stats(&mut dists);
    Ok(Measure {
        sides: reports,
        feet,
        starts,
        rms,
        p99,
        max,
        points,
        outside,
        hard_max,
        hard_ok: hard_max <= G0_TARGET,
        soft_max,
        rest_ok,
        side_terms,
        hard_term: hard_max / G0_TARGET,
        data_term: max / goal.tolerance,
        miss: slabs.score(),
        interior_folded: interior_folded(surface, lp, layout, goal.cancel)?,
    })
}
