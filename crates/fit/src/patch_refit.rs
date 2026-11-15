//! Least-squares refit of the curves and patches of `surface.wrap` to the scan.
//!
//! The first construction of a wrap curve or patch interpolates a few scan points: the points
//! under the chord of the coarse edge at 1/3 and 2/3 (a curve), under the first guess of the patch
//! at the four inner thirds (a patch). Two things go wrong with that on a coarse layout over a
//! curved, noisy scan: the points are taken under the chord, not under the curve, so where the
//! chord is far from the scan they are spaced unlike the curve's parameter and the curve swings
//! between them; and the scan noise at those few points goes straight into the control points.
//! Here the result is sampled at more points, the samples are projected onto the scan, and the
//! inner control points are fitted to those projections by least squares (the ends and the
//! boundary stay, so neighbours still share their curves). A curve gives its projected samples
//! the parameters of their spacing along it (chord length), so the refit also corrects where along
//! the curve a point lies, not only how far off it is (a 100 degree arc: largest gap 0.52 mm by
//! the chord interpolant, 0.05 mm refitted); a patch keeps the parameters of its samples.
//!
//! A sample with no scan point within the reach is left out, of the fit and of the measurement;
//! too few valid samples leave the input as it is. Each refit is measured against the best so far
//! over the samples valid in both, and kept only when its samples lie closer to the scan there. So
//! the claim is about the samples only: between them nothing is measured, which is why a refitted
//! patch must also face like its input everywhere on a grid that includes its border (no fold, no
//! patch turned against the scan). On a real organic scan held at 60 000 patches the refit about
//! halves the points out of 0.2 mm for the same number of patches.
//!
//! Evaluation goes through [`crate::patch`] (the listed APP-GEOMETRY file).
use crate::patch::{Net, bern, eval, normal, tensor_weights};
use surfacecraft_geom::Vec3;

/// A projection onto the scan: `None` where no scan point is within the reach.
pub(crate) type Project<'a> = &'a dyn Fn(Vec3) -> Option<Vec3>;

/// Parameters a curve is sampled at when it is refitted.
const CURVE_T: [f64; 6] = [0.07, 0.22, 0.4, 0.6, 0.78, 0.93];
/// Parameters (in u and in v) a patch is sampled at when its inner points are refitted.
const NET_T: [f64; 6] = [0.07, 0.22, 0.4, 0.6, 0.78, 0.93];
/// Least-squares refits of a curve or a patch (each one measured; the best measured is kept).
const PASSES: usize = 2;
/// The inner control points of a patch.
const INNER: [(usize, usize); 4] = [(1, 1), (1, 2), (2, 1), (2, 2)];
/// A 2 × 2 system with a smaller determinant than this (relative) is singular.
const SINGULAR: f64 = 1e-12;
/// Fewest valid samples (of 6) a curve is refitted with: its two inner control points need two,
/// and a third makes the fit average instead of pass through the points; with fewer the plain
/// curve stays.
const MIN_CURVE_SAMPLES: usize = 3;
/// Fewest valid samples (of 36) a patch is refitted with: half the grid. Its four inner control
/// points need four, but when the samples of one side are lost (past a hole or beyond the reach)
/// nothing holds that side and the fit could swing it; with fewer the plain patch stays.
const MIN_NET_SAMPLES: usize = 18;
/// A refitted patch must face like its input to within this cosine (about 25 degrees) at every point
/// of [`GUARD_T`]: a refit corrects where the surface lies, it does not turn it. A looser limit (the
/// wrap's facing limit, 0.3) let patches turn by up to 72 degrees, and the hybrid method then found
/// patches disagreeing with the analytic face by more than its 60 degrees and kept a whole wall of an
/// open part as patches (`p9_hybrid_scan`, on Windows).
const MIN_TURN_COS: f64 = 0.9;
/// Where (in u and in v) a refitted patch's normals are checked against its input, borders included.
const GUARD_T: [f64; 7] = [0.0, 1.0 / 6.0, 1.0 / 3.0, 0.5, 2.0 / 3.0, 5.0 / 6.0, 1.0];

/// The scan point under a sample and its squared distance from the sample; `None` when there is
/// no scan point within the reach.
type Sample = Option<(Vec3, f64)>;

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

fn sample(p: Vec3, project: Project) -> Sample {
    let q = project(p)?;
    let d = (q - p).norm_squared();
    (d.is_finite() && q.iter().all(|x| x.is_finite())).then_some((q, d))
}

fn valid(s: &[Sample]) -> usize {
    s.iter().flatten().count()
}

/// Is `new` closer to the scan than `old` over the samples valid in both (at least `min` of them)?
fn closer(old: &[Sample], new: &[Sample], min: usize) -> bool {
    let both: Vec<(f64, f64)> = old.iter().zip(new).filter_map(|(a, b)| Some(((*a)?.1, (*b)?.1))).collect();
    let (o, n) = both.iter().fold((0.0, 0.0), |(o, n), (a, b)| (o + a, n + b));
    both.len() >= min && n < o
}

fn curve_point(c: &[Vec3; 4], t: f64) -> Vec3 {
    let b = bern(t);
    c[0] * b[0] + c[1] * b[1] + c[2] * b[2] + c[3] * b[3]
}

/// The samples of `c` at [`CURVE_T`].
fn curve_samples(c: &[Vec3; 4], project: Project) -> Vec<Sample> {
    CURVE_T.iter().map(|&t| sample(curve_point(c, t), project)).collect()
}

/// Parameters of the points `q` between the ends of `c` by the length of the polyline through them
/// (the points of a curve that follows the scan are spaced like its parameter); `None` for a
/// polyline of no length.
fn chord_parameters(c: &[Vec3; 4], q: &[Vec3]) -> Option<Vec<f64>> {
    let mut t = Vec::with_capacity(q.len());
    let mut prev = c[0];
    let mut run = 0.0;
    for p in q {
        run += (p - prev).norm();
        t.push(run);
        prev = *p;
    }
    let total = run + (c[3] - prev).norm();
    (total.is_finite() && total > 0.0).then(|| t.iter().map(|x| x / total).collect())
}

/// The cubic with the ends of `c` closest (least squares) to the valid samples `s` at their
/// chord-length parameters; `None` with fewer than [`MIN_CURVE_SAMPLES`], or when the system is
/// singular or the result is not finite.
fn curve_through(c: &[Vec3; 4], s: &[Sample]) -> Option<[Vec3; 4]> {
    let q: Vec<Vec3> = s.iter().flatten().map(|(q, _)| *q).collect();
    if q.len() < MIN_CURVE_SAMPLES {
        return None;
    }
    let params = chord_parameters(c, &q)?;
    let (mut m11, mut m12, mut m22) = (0.0, 0.0, 0.0);
    let (mut r1, mut r2) = (Vec3::zeros(), Vec3::zeros());
    for (&t, p) in params.iter().zip(&q) {
        let b = bern(t);
        let res = p - c[0] * b[0] - c[3] * b[3];
        m11 += b[1] * b[1];
        m12 += b[1] * b[2];
        m22 += b[2] * b[2];
        r1 += res * b[1];
        r2 += res * b[2];
    }
    let det = m11 * m22 - m12 * m12;
    if !(det.is_finite() && det.abs() > SINGULAR * (m11 * m22).abs().max(SINGULAR)) {
        return None;
    }
    let c1 = (r1 * m22 - r2 * m12) / det;
    let c2 = (r2 * m11 - r1 * m12) / det;
    c1.iter().chain(c2.iter()).all(|x| x.is_finite()).then_some([c[0], c1, c2, c[3]])
}

/// `c` refitted to the scan (see the module documentation); `c` itself when fewer than
/// [`MIN_CURVE_SAMPLES`] samples reach the scan or no refit lies closer.
pub(crate) fn refit_curve(c: [Vec3; 4], project: Project) -> [Vec3; 4] {
    let first = curve_samples(&c, project);
    if valid(&first) < MIN_CURVE_SAMPLES {
        return c;
    }
    let (mut best, mut best_s) = (c, first.clone());
    let (mut cur, mut cur_s) = (c, first);
    for _ in 0..PASSES {
        let Some(next) = curve_through(&cur, &cur_s) else { break };
        let next_s = curve_samples(&next, project);
        if closer(&best_s, &next_s, MIN_CURVE_SAMPLES) {
            (best, best_s) = (next, next_s.clone());
        }
        (cur, cur_s) = (next, next_s);
    }
    best
}

/// The samples of `net` at [`NET_T`] in u and v (u outer).
fn net_samples(net: &Net, project: Project) -> Vec<Sample> {
    let params = NET_T.iter().flat_map(|&u| NET_T.iter().map(move |&v| (u, v)));
    params.map(|(u, v)| sample(eval(net, u, v).0, project)).collect()
}

/// The part of the point of `net` at weights `w` that the boundary control points give.
fn boundary_part(net: &Net, w: &[[f64; 4]; 4]) -> Vec3 {
    let mut fixed = Vec3::zeros();
    for (i, (row, wrow)) in net.iter().zip(w).enumerate() {
        for (j, (p, wij)) in row.iter().zip(wrow).enumerate() {
            if !INNER.contains(&(i, j)) {
                fixed += v3(*p) * *wij;
            }
        }
    }
    fixed
}

/// `net` with its four inner control points fitted (least squares) to the valid samples `s` of
/// [`net_samples`]; `None` with fewer than [`MIN_NET_SAMPLES`], or when the system is singular or
/// the result is not finite.
fn net_through(net: &Net, s: &[Sample]) -> Option<Net> {
    if valid(s) < MIN_NET_SAMPLES {
        return None;
    }
    let mut m = nalgebra::Matrix4::<f64>::zeros();
    let mut r = nalgebra::Matrix4x3::<f64>::zeros();
    let params = NET_T.iter().flat_map(|&u| NET_T.iter().map(move |&v| (u, v)));
    for ((u, v), target) in params.zip(s).filter_map(|(uv, s)| Some((uv, (*s)?.0))) {
        let w = tensor_weights(u, v);
        let res = target - boundary_part(net, &w);
        let a: [f64; 4] = INNER.map(|(i, j)| w.get(i).and_then(|row| row.get(j)).copied().unwrap_or(0.0));
        for x in 0..4 {
            for y in 0..4 {
                m[(x, y)] += a[x] * a[y];
            }
            for d in 0..3 {
                r[(x, d)] += a[x] * res[d];
            }
        }
    }
    let sol = m.lu().solve(&r)?;
    let mut out = *net;
    for (x, &(i, j)) in INNER.iter().enumerate() {
        let p = [sol[(x, 0)], sol[(x, 1)], sol[(x, 2)]];
        if !p.iter().all(|c| c.is_finite()) {
            return None;
        }
        if let Some(slot) = out.get_mut(i).and_then(|row| row.get_mut(j)) {
            *slot = p;
        }
    }
    Some(out)
}

/// Does `new` face like `old` everywhere on [`GUARD_T`] × [`GUARD_T`] (normals within
/// [`MIN_TURN_COS`]; where `old` has a normal, `new` must have one)? A refit that folds or turns a
/// patch fails it.
fn keeps_facing(old: &Net, new: &Net) -> bool {
    GUARD_T.iter().all(|&u| {
        GUARD_T.iter().all(|&v| match (normal(old, u, v), normal(new, u, v)) {
            (Some(a), Some(b)) => a.dot(&b) >= MIN_TURN_COS,
            (None, _) => true,
            (Some(_), None) => false,
        })
    })
}

/// `net` with its inner control points refitted to the scan like [`refit_curve`] refits a curve
/// (its boundary is left as it is), keeping only refits that face like `net` ([`keeps_facing`]);
/// `net` itself when fewer than [`MIN_NET_SAMPLES`] samples reach the scan or no refit qualifies.
pub(crate) fn refit_net(net: Net, project: Project) -> Net {
    let first = net_samples(&net, project);
    if valid(&first) < MIN_NET_SAMPLES {
        return net;
    }
    let (mut best, mut best_s) = (net, first.clone());
    let (mut cur, mut cur_s) = (net, first);
    for _ in 0..PASSES {
        let Some(next) = net_through(&cur, &cur_s) else { break };
        let next_s = net_samples(&next, project);
        if closer(&best_s, &next_s, MIN_NET_SAMPLES) && keeps_facing(&net, &next) {
            (best, best_s) = (next, next_s.clone());
        }
        (cur, cur_s) = (next, next_s);
    }
    best
}

#[cfg(test)]
#[path = "patch_refit_tests.rs"]
mod tests;
