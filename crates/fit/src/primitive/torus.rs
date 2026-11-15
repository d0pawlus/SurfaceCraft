//! The torus fit: tube radius scan and refinement, with its penalty bounds.

use super::estimate::*;
use super::model::*;
use super::*;

/// A trial of the tube radius scan: (cost of the planar circle, circle centre, axis, major radius,
/// unit direction from the centre to the middle of the arc the tube centres cover).
type TubeCircle = (f64, Vec3, Vec3, f64, Vec3);

/// A start of the torus refinement: (true torus residual, tube radius, circle centre, axis, major
/// radius).
type TorusStart = (f64, f64, Vec3, Vec3, f64);

/// A trial tube radius with the circle its tube centres lie on.
struct TubeTrial {
    s: f64,
    c0: Vec3,
    a: Vec3,
    big0: f64,
    dir: Vec3,
}

/// Fits a torus (needs outward normals: tube centres are `p − r·n`).
#[must_use]
pub fn fit_torus(pts: &[Point3], normals: &[Vec3]) -> Option<PrimitiveFit> {
    if pts.len() < 10 || normals.len() != pts.len() {
        return None;
    }
    let sub = subset(pts);
    let nsub: Vec<Vec3> = {
        let stride = pts.len().div_ceil(MAX_LM_POINTS).max(1);
        normals.iter().step_by(stride).copied().collect()
    };
    let ext = surfacecraft_geom::Aabb::from_points(&sub).diagonal().max(1e-9);
    let starts = torus_starts(&sub, &nsub, ext);
    let mut best_fit: Option<(f64, Primitive)> = None;
    for start in &starts {
        let Some((cost, prim)) = refine_torus(&sub, start) else { continue };
        if prim.is_valid() && cost.is_finite() && best_fit.as_ref().is_none_or(|b| cost < b.0) {
            best_fit = Some((cost, prim));
        }
        // the first start usually converges to the right torus: the others only run when the fit
        // is poor relative to the size of the patch, or is held at the penalty bound (tube
        // radius = 0.95 major radius), which is where a stalled fit ends, not a minimum
        let good = (TORUS_GOOD_RMS * ext).powi(2) * sub.len() as f64;
        if best_fit.as_ref().is_some_and(|b| b.0 <= good && !at_penalty_bound(&b.1)) {
            break;
        }
    }
    stats(best_fit?.1, pts)
}

/// Scores trial tube radius `s`: the tube centres lie on a planar circle, so `s` is scored by that
/// circle. The sign of the scan normals is not known: they point away from the tube centre on a
/// convex patch and towards it on a concave one, so `side` = ±1 tries both.
fn tube_circle_score(sub: &[Point3], nsub: &[Vec3], s: f64, side: f64) -> Option<TubeCircle> {
    let cs: Vec<Point3> = sub.iter().zip(nsub).map(|(p, n)| p - n * (s * side)).collect();
    let pl = crate::plane::fit_plane(&cs, None)?;
    let (u, w) = basis(&pl.normal);
    let o = pl.point.coords;
    let xy: Vec<(f64, f64)> = cs.iter().map(|c| ((c.coords - o).dot(&u), (c.coords - o).dot(&w))).collect();
    let (cx, cy, r) = circle_2d(&xy)?;
    let mut sq = 0.0;
    for &(x, y) in &xy {
        sq += ((x - cx).hypot(y - cy) - r).powi(2);
    }
    let cost = pl.rms * pl.rms + sq / xy.len() as f64;
    let n = xy.len() as f64;
    let (mx, my) = (xy.iter().map(|q| q.0).sum::<f64>() / n - cx, xy.iter().map(|q| q.1).sum::<f64>() / n - cy);
    let dir = (u * mx + w * my).try_normalize(1e-12).unwrap_or_else(Vec3::zeros);
    Some((cost, o + u * cx + w * cy, pl.normal, r, dir))
}

/// Scores the enlarged circles of one trial by their true torus residual and keeps the best of
/// each factor, for side `si` of the normals.
fn scan_factors(sub: &[Point3], best_per_factor: &mut [Option<TorusStart>], si: usize, t: &TubeTrial) {
    let arc_mid = t.c0 + t.dir * t.big0;
    for (fi, f) in TORUS_FACTORS.iter().enumerate() {
        let big = t.big0 * f;
        let c = arc_mid - t.dir * big;
        let geo = sub
            .iter()
            .map(|q| {
                let x = q.coords - c;
                let h = x.dot(&t.a);
                let rho = (x - t.a * h).norm();
                ((rho - big).hypot(h) - t.s).powi(2)
            })
            .sum::<f64>()
            / sub.len() as f64;
        if let Some(slot) = best_per_factor.get_mut(fi * 2 + si)
            && geo.is_finite()
            && slot.as_ref().is_none_or(|b| geo < b.0)
        {
            *slot = Some((geo, t.s, c, t.a, big));
        }
    }
}

/// The starts of the refinement, the most promising first.
///
/// On a partial torus patch the tube centres cover a short arc, whose circle is ill-determined:
/// (tube radius, major radius) lie along a nearly flat valley of the planar-circle cost, and a
/// start from the best circle can end in a poor local minimum of the final fit (a spindle-like
/// torus) while the true torus is several times larger. So each trial tube radius is also tried
/// with larger circles through the same arc, every trial is scored by its true torus residual,
/// and the few best distinct trials start the fit.
fn torus_starts(sub: &[Point3], nsub: &[Vec3], ext: f64) -> Vec<TorusStart> {
    let steps = 80;
    // per factor and per side of the normals
    let mut best_per_factor: Vec<Option<TorusStart>> = vec![None; TORUS_FACTORS.len() * 2];
    for k in 1..=steps {
        // geometric from 1 % to 400 % of the extent: the tube of a partial torus patch can be
        // larger than the patch, so a linear scan up to half the extent never reaches its radius
        let s = ext * 0.01 * 400f64.powf(k as f64 / steps as f64);
        for (si, side) in [1.0, -1.0].into_iter().enumerate() {
            let Some((_, c0, a, big0, dir)) = tube_circle_score(sub, nsub, s, side) else { continue };
            scan_factors(sub, &mut best_per_factor, si, &TubeTrial { s, c0, a, big0, dir });
        }
    }
    let mut starts: Vec<TorusStart> = best_per_factor.into_iter().flatten().collect();
    // most promising first (smallest residual of the trial torus before the fit): the first start
    // decides whether the others run at all, and the trials of the wrong side of the normals or of
    // a wrong circle size start far from the surface
    starts.sort_by(|x, y| x.0.total_cmp(&y.0));
    starts
}

/// Refines one start by Levenberg–Marquardt; the cost and the torus it ends in.
fn refine_torus(sub: &[Point3], start: &TorusStart) -> Option<(f64, Primitive)> {
    let &(_, s, c, a, big) = start;
    let (u, w) = basis(&a);
    // A patch of a torus with a large tube is nearly as well described by a spindle torus (tube
    // reaching past the axis, minor radius above the major one) or by a negative major radius,
    // and the fit drifts along that flat valley to a surface the auto fit discards. A penalty
    // residual keeps the tube inside the ring: minor <= 0.95 major.
    let n_pen = (sub.len() as f64).sqrt() * 10.0;
    let res = lm::minimize(
        DVector::from_vec(vec![0.0, 0.0, c.x, c.y, c.z, big.max(s / 0.9), s]),
        sub.len() + 1,
        150,
        |p, out| {
            let d = axis_from(&a, &u, &w, p[0], p[1]);
            let cc = Vec3::new(p[2], p[3], p[4]);
            for (o, q) in out.iter_mut().zip(sub) {
                let x = q.coords - cc;
                let h = x.dot(&d);
                let rho = (x - d * h).norm();
                *o = (rho - p[5]).hypot(h) - p[6];
            }
            if let Some(o) = out.get_mut(sub.len()) {
                *o = n_pen * (p[6].abs() - 0.95 * p[5]).max(0.0);
            }
        },
    )?;
    let p = res.params;
    let d = axis_from(&a, &u, &w, p[0], p[1]);
    let prim =
        Primitive::Torus { center: [p[2], p[3], p[4]], axis: d.into(), major_radius: p[5], minor_radius: p[6].abs() };
    Some((res.cost, prim))
}

/// RMS (relative to the patch extent) below which [`fit_torus`] stops trying further starts.
const TORUS_GOOD_RMS: f64 = 0.003;

/// Whether a torus sits at the bound `minor <= 0.95 major` that [`fit_torus`] enforces (a small
/// margin for the penalty's softness).
fn at_penalty_bound(p: &Primitive) -> bool {
    matches!(p, Primitive::Torus { major_radius, minor_radius, .. } if *minor_radius >= 0.94 * *major_radius)
}

/// Factors by which [`fit_torus`] enlarges the circle of tube centres of each trial tube radius;
/// the best trial of each factor starts one fit.
const TORUS_FACTORS: [f64; 4] = [1.0, 2.5, 6.0, 15.0];
