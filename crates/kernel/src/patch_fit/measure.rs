//! Measurement of a fitted patch (design doc §4.3): the gap, angle and curvature error along every
//! side on the dense samples, the fold test, the deviation of the data, and where the misses lie.
//! Also re-projects the interior points onto the patch for the next round.
use super::geom::{V2, add, cross, dot, scale, sub, unit};
use super::rows::{Layout, SideSamples};
use super::start::{Interior, Loop, Pt};
use super::{Continuity, G0_TARGET, G1_TARGET_DEG, G2_TARGET_REL, SideReport};
use crate::Result;
use crate::nurbs::SurfaceProbe;
use crate::record::brep::Surface;
use crate::vpatch::derivatives;

/// `S_d` pointing back over the reference face at this cosine or more counts as a fold.
const FOLD_COS: f64 = 0.1;

/// Everything measured of one round's surface.
pub(super) struct Measure {
    pub sides: Vec<SideReport>,
    /// Parameters of the closest points of every dense side sample.
    pub feet: Vec<Vec<V2>>,
    pub rms: f64,
    pub p99: f64,
    pub max: f64,
    pub points: usize,
    /// Largest distance of a hard point or curve sample from the patch.
    pub hard_max: f64,
    /// Every side within its targets (and nothing folded), hard points and curves on the patch.
    pub sides_ok: bool,
    /// Worst miss in units of its target (a fold counts as 2).
    pub worst: f64,
    /// Misses by direction: the largest root-mean-square miss over slabs of the parameter domain.
    pub miss: [f64; 2],
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

/// Measures `surface` against the sides, the interior and the tolerance; the interior's parameters
/// become the closest points on `surface`.
pub(super) fn measure(
    surface: &Surface,
    probe: &SurfaceProbe,
    lp: &Loop,
    layout: &Layout,
    sides: &[SideSamples],
    interior: &mut Interior,
    tolerance: f64,
) -> Result<Measure> {
    let mut slabs = Slabs::new(layout);
    let floor = 1.0 / (4.0 * lp.diag.max(1e-9));
    let mut reports = Vec::with_capacity(sides.len());
    let mut all_feet = Vec::with_capacity(sides.len());
    let mut sides_ok = true;
    let mut worst = 0.0_f64;

    for (sd, ss) in lp.sides.iter().zip(sides) {
        let g1 = sd.continuity != Continuity::G0;
        let g2 = sd.continuity == Continuity::G2;
        let (mut gap_max, mut angle_max, mut curv_max) = (0.0_f64, 0.0_f64, 0.0_f64);
        let mut folded = false;
        let mut feet = Vec::with_capacity(ss.samples.len());
        for smp in &ss.samples {
            let (d, _, f) = probe.closest_uv(smp.xyz)?;
            feet.push(f);
            gap_max = gap_max.max(d);
            let mut miss = d / G0_TARGET;
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
            slabs.add(f, miss);
        }
        let ok = gap_max <= G0_TARGET
            && (!g1 || angle_max <= G1_TARGET_DEG)
            && (!g2 || curv_max <= G2_TARGET_REL)
            && !folded;
        sides_ok &= ok;
        worst = worst
            .max(gap_max / G0_TARGET)
            .max(if g1 { angle_max / G1_TARGET_DEG } else { 0.0 })
            .max(if g2 { curv_max / G2_TARGET_REL } else { 0.0 })
            .max(if folded { 2.0 } else { 0.0 });
        reports.push(SideReport {
            continuity: sd.continuity,
            gap_max,
            angle_max_deg: g1.then_some(angle_max),
            curvature_error_max: g2.then_some(curv_max),
            samples: ss.samples.len(),
            folded,
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
    if hard_max > G0_TARGET {
        sides_ok = false;
    }
    worst = worst.max(hard_max / G0_TARGET);

    let mut dists = reproject(probe, layout, &mut interior.data, &mut slabs, tolerance)?;
    dists.sort_by(|a, b| a.total_cmp(b));
    let points = dists.len();
    let (rms, p99, max) = if points == 0 {
        (0.0, 0.0, 0.0)
    } else {
        let rms = (dists.iter().map(|d| d * d).sum::<f64>() / points as f64).sqrt();
        let k = ((0.99 * points as f64).ceil() as usize).clamp(1, points) - 1;
        (rms, dists.get(k).copied().unwrap_or(0.0), dists.last().copied().unwrap_or(0.0))
    };
    worst = worst.max(max / tolerance);

    Ok(Measure {
        sides: reports,
        feet: all_feet,
        rms,
        p99,
        max,
        points,
        hard_max,
        sides_ok,
        worst,
        miss: slabs.score(),
    })
}
