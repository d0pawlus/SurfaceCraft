//! Measurement of a fitted patch (design doc §4.3): the gap, angle and curvature error along every
//! side on the dense samples, the fold test, the deviation of the data, and where the misses lie.
//! Also re-projects the interior points onto the patch for the next round.
use super::geom::{V2, add, cross, dot, scale, sub, unit};
use super::rows::{Layout, MIN_DATA, SideSamples};
use super::start::{Interior, Loop, Pt};
use super::{Continuity, G0_TARGET, G1_TARGET_DEG, G2_TARGET_REL, SideReport};
use crate::Result;
use crate::nurbs::SurfaceProbe;
use crate::record::brep::Surface;
use crate::vpatch::derivatives;

/// `S_d` pointing back over the reference face at this cosine or more counts as a fold.
const FOLD_COS: f64 = 0.1;

/// What a round is measured against.
pub(super) struct Goal<'a> {
    /// Target of the data deviation (max), mm.
    pub tolerance: f64,
    /// Sides whose gap is as small as their own accuracy allows: it does not steer the growth.
    pub frozen: &'a [bool],
}

/// Everything measured of one round's surface.
pub(super) struct Measure {
    pub sides: Vec<SideReport>,
    /// Parameters of the closest points of every dense side sample.
    pub feet: Vec<Vec<V2>>,
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
}

impl Measure {
    /// Largest miss of the hard conditions (sides, interior points and curves) in units of their
    /// targets, every gap counted: at most 1 when all are met.
    pub fn side_miss(&self) -> f64 {
        self.side_terms.iter().flatten().fold(self.hard_term, |a, b| a.max(*b))
    }

    /// Worst miss, the data included, in units of the targets; the gap of a `frozen` side (as close
    /// to its polyline as its own accuracy allows) is left out.
    pub fn worst(&self, frozen: &[bool]) -> f64 {
        let mut w = self.hard_term.max(self.data_term);
        for (k, t) in self.side_terms.iter().enumerate() {
            let gap = if frozen.get(k).copied().unwrap_or(false) { 0.0 } else { t[0] };
            w = w.max(gap).max(t[1]);
        }
        w
    }

    /// Every side within its targets and the interior on the patch. The gap of a `frozen` side may
    /// be up to ten times the target.
    pub fn sides_ok(&self, frozen: &[bool]) -> bool {
        self.hard_ok
            && self.sides.iter().zip(&self.rest_ok).enumerate().all(|(k, (s, rest))| {
                let lenient = frozen.get(k).copied().unwrap_or(false) && s.gap_max < 10.0 * G0_TARGET;
                s.soft || (*rest && (s.gap_max <= G0_TARGET || lenient))
            })
    }
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
            let k = ((uv[d] - self.domain[d][0]) / w * self.cells[d] as f64).floor();
            let k = (k.max(0.0) as usize).min(self.cells[d] - 1);
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
) -> Result<Vec<f64>> {
    let mut out = Vec::with_capacity(pts.len());
    for p in pts.iter_mut() {
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
) -> Result<(Vec<f64>, usize)> {
    let mut dists = Vec::with_capacity(pts.len());
    let mut outside = 0usize;
    for p in pts.iter_mut() {
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
    let (tolerance, frozen) = (goal.tolerance, goal.frozen);
    let mut slabs = Slabs::new(layout);
    let floor = 1.0 / (4.0 * lp.diag.max(1e-9));
    let mut reports = Vec::with_capacity(sides.len());
    let mut all_feet = Vec::with_capacity(sides.len());
    let mut rest_ok_all = Vec::with_capacity(sides.len());
    let mut side_terms = Vec::with_capacity(sides.len());

    for (k, (sd, ss)) in lp.sides.iter().zip(sides).enumerate() {
        let is_frozen = frozen.get(k).copied().unwrap_or(false);
        let g1 = sd.continuity != Continuity::G0;
        let g2 = sd.continuity == Continuity::G2;
        let (mut gap_max, mut angle_max, mut curv_max) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut folded = false;
        let mut feet = Vec::with_capacity(ss.samples.len());
        for smp in &ss.samples {
            let (d, _, f) = probe.closest_uv(smp.xyz)?;
            feet.push(f);
            gap_max = gap_max.max(d);
            let mut miss = if is_frozen { 0.0 } else { d / G0_TARGET };
            if let Some(r) = smp.refs.as_ref().filter(|_| g1) {
                let der = derivatives(surface, f)?;
                let (su, sv) = (der[1], der[2]);
                let angle = match unit(cross(su, sv)) {
                    Some(n) => dot(n, r.normal).abs().min(1.0).acos().to_degrees(),
                    None => 90.0,
                };
                angle_max = angle_max.max(angle);
                miss = miss.max(angle / G1_TARGET_DEG);
                let sd_vec = add(scale(su, smp.dir[0]), scale(sv, smp.dir[1]));
                if let (Some(w), Some(inward)) = (unit(sd_vec), lp.affine.lift(smp.dir)) {
                    // the in-face direction away from the edge
                    let away = scale(inward, -1.0);
                    if let Some(a) = unit(sub(away, scale(r.normal, dot(away, r.normal)))) {
                        folded |= dot(w, a) > FOLD_COS;
                    }
                    if g2 {
                        let (d0, d1) = (smp.dir[0], smp.dir[1]);
                        let sdd =
                            add(add(scale(der[3], d0 * d0), scale(der[4], 2.0 * d0 * d1)), scale(der[5], d1 * d1));
                        let kappa_patch = dot(sdd, r.normal) / dot(sd_vec, sd_vec);
                        if let Some(kappa_ref) = r.curvature(w) {
                            let rel = (kappa_patch - kappa_ref).abs() / kappa_ref.abs().max(floor);
                            if rel.is_finite() {
                                curv_max = curv_max.max(rel);
                                miss = miss.max(rel / G2_TARGET_REL);
                            }
                        }
                    }
                }
            }
            // a soft side is reported but does not steer the growth of the net
            if !sd.soft {
                slabs.add(f, miss);
            }
        }
        let rest_ok = (!g1 || angle_max <= G1_TARGET_DEG) && (!g2 || curv_max <= G2_TARGET_REL) && !folded;
        let rest_term = (if g1 { angle_max / G1_TARGET_DEG } else { 0.0 })
            .max(if g2 { curv_max / G2_TARGET_REL } else { 0.0 })
            .max(if folded { 2.0 } else { 0.0 });
        rest_ok_all.push(rest_ok || sd.soft);
        side_terms.push(if sd.soft { [0.0, 0.0] } else { [gap_max / G0_TARGET, rest_term] });
        reports.push(SideReport {
            continuity: sd.continuity,
            soft: sd.soft,
            gap_max,
            angle_max_deg: g1.then_some(angle_max),
            curvature_error_max: g2.then_some(curv_max),
            samples: ss.samples.len(),
            folded,
            snap_max: sd.snap,
        });
        all_feet.push(feet);
    }

    let mut hard_max = 0.0_f64;
    let hard = reproject(probe, layout, &mut interior.hard, &mut slabs, G0_TARGET)?;
    hard_max = hard.iter().fold(hard_max, |m, d| m.max(*d));
    for c in interior.curves.iter_mut() {
        let d = reproject(probe, layout, c, &mut slabs, G0_TARGET)?;
        hard_max = d.iter().fold(hard_max, |m, x| m.max(*x));
    }
    let hard_term = hard_max / G0_TARGET;

    let (mut dists, outside) = reproject_data(probe, lp, &mut interior.data, &mut slabs, tolerance)?;
    if dists.len() < MIN_DATA {
        dists.clear();
    }
    dists.sort_by(|a, b| a.total_cmp(b));
    let points = dists.len();
    let (rms, p99, max) = if points == 0 {
        (0.0, 0.0, 0.0)
    } else {
        let rms = (dists.iter().map(|d| d * d).sum::<f64>() / points as f64).sqrt();
        let k = ((0.99 * points as f64).ceil() as usize).clamp(1, points) - 1;
        (rms, dists.get(k).copied().unwrap_or(0.0), dists.last().copied().unwrap_or(0.0))
    };

    Ok(Measure {
        sides: reports,
        feet: all_feet,
        rms,
        p99,
        max,
        points,
        outside,
        hard_max,
        hard_ok: hard_max <= G0_TARGET,
        rest_ok: rest_ok_all,
        side_terms,
        hard_term,
        data_term: max / tolerance,
        miss: slabs.score(),
    })
}
