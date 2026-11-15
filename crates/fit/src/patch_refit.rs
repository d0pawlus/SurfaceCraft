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
//! the chord interpolant, 0.05 mm refitted); a patch keeps the parameters of its samples. This is
//! repeated, and of all the curves (or patches) measured the one whose samples lie closest to the
//! scan is kept, so a refit never makes the fit at the samples worse. On a real organic scan held
//! at 60 000 patches it about halves the points out of 0.2 mm for the same number of patches.
//!
//! Evaluation goes through [`crate::patch`] (the listed APP-GEOMETRY file).
use crate::patch::{Net, bern, eval, tensor_weights};
use surfacecraft_geom::Vec3;

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

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

fn curve_point(c: &[Vec3; 4], t: f64) -> Vec3 {
    let b = bern(t);
    c[0] * b[0] + c[1] * b[1] + c[2] * b[2] + c[3] * b[3]
}

/// The scan points under the samples of `c` and their mean squared distance from them.
fn curve_targets(c: &[Vec3; 4], project: &dyn Fn(Vec3) -> Vec3) -> ([Vec3; 6], f64) {
    let mut q = [Vec3::zeros(); 6];
    let mut sum = 0.0;
    for (slot, &t) in q.iter_mut().zip(&CURVE_T) {
        let p = curve_point(c, t);
        *slot = project(p);
        sum += (*slot - p).norm_squared();
    }
    (q, sum / CURVE_T.len() as f64)
}

/// Parameters of the points `q` between the ends of `c` by the length of the polyline through them
/// (the points of a curve that follows the scan are spaced like its parameter); `None` for a
/// polyline of no length.
fn chord_parameters(c: &[Vec3; 4], q: &[Vec3; 6]) -> Option<[f64; 6]> {
    let mut t = [0.0; 6];
    let mut prev = c[0];
    let mut run = 0.0;
    for (slot, p) in t.iter_mut().zip(q) {
        run += (p - prev).norm();
        *slot = run;
        prev = *p;
    }
    let total = run + (c[3] - prev).norm();
    if !(total.is_finite() && total > 0.0) {
        return None;
    }
    Some(t.map(|x| x / total))
}

/// The cubic with the ends of `c` closest (least squares) to the points `q` at their chord-length
/// parameters; `None` when the system is singular or the result is not finite.
fn curve_through(c: &[Vec3; 4], q: &[Vec3; 6]) -> Option<[Vec3; 4]> {
    let params = chord_parameters(c, q)?;
    let (mut m11, mut m12, mut m22) = (0.0, 0.0, 0.0);
    let (mut r1, mut r2) = (Vec3::zeros(), Vec3::zeros());
    for (&t, p) in params.iter().zip(q) {
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

/// `c` refitted to the scan (see the module documentation); `c` itself when no refit lies closer.
pub(crate) fn refit_curve(c: [Vec3; 4], project: &dyn Fn(Vec3) -> Vec3) -> [Vec3; 4] {
    let (mut q, mut best_err) = curve_targets(&c, project);
    let (mut best, mut cur) = (c, c);
    for _ in 0..PASSES {
        let Some(next) = curve_through(&cur, &q) else { break };
        let (nq, err) = curve_targets(&next, project);
        if err < best_err {
            (best, best_err) = (next, err);
        }
        (cur, q) = (next, nq);
    }
    best
}

/// The scan points under the samples of `net` (at [`NET_T`] in u and v) and their mean squared
/// distance from them.
fn net_targets(net: &Net, project: &dyn Fn(Vec3) -> Vec3) -> (Vec<Vec3>, f64) {
    let mut q = Vec::with_capacity(NET_T.len() * NET_T.len());
    let mut sum = 0.0;
    for &u in &NET_T {
        for &v in &NET_T {
            let (p, _, _) = eval(net, u, v);
            let at = project(p);
            sum += (at - p).norm_squared();
            q.push(at);
        }
    }
    let n = q.len().max(1) as f64;
    (q, sum / n)
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

/// `net` with its four inner control points fitted (least squares) to `q`, the targets of
/// [`net_targets`]; `None` when the system is singular or the result is not finite.
fn net_through(net: &Net, q: &[Vec3]) -> Option<Net> {
    let mut m = nalgebra::Matrix4::<f64>::zeros();
    let mut r = nalgebra::Matrix4x3::<f64>::zeros();
    let params = NET_T.iter().flat_map(|&u| NET_T.iter().map(move |&v| (u, v)));
    for ((u, v), target) in params.zip(q) {
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

/// `net` with its inner control points refitted to the scan like [`refit_curve`] refits a curve
/// (its boundary is left as it is); `net` itself when no refit lies closer.
pub(crate) fn refit_net(net: Net, project: &dyn Fn(Vec3) -> Vec3) -> Net {
    let (mut q, mut best_err) = net_targets(&net, project);
    let (mut best, mut cur) = (net, net);
    for _ in 0..PASSES {
        let Some(next) = net_through(&cur, &q) else { break };
        let (nq, err) = net_targets(&next, project);
        if err < best_err {
            (best, best_err) = (next, err);
        }
        (cur, q) = (next, nq);
    }
    best
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::patch::{chord_curve, interpolated_net};

    const R: f64 = 10.0;
    /// Points along a curve or across a patch at which the true surface is checked.
    const CHECK: usize = 60;

    /// The closest point on the cylinder of radius [`R`] about the z axis.
    fn on_cylinder(p: Vec3) -> Vec3 {
        let r = p.x.hypot(p.y);
        Vec3::new(p.x * R / r, p.y * R / r, p.z)
    }

    /// The point on the sphere of radius [`R`] about the origin under `p`, moved along the normal by
    /// a deterministic noise of amplitude `sigma` (it varies over a fraction of a millimetre).
    fn on_noisy_sphere(p: Vec3, sigma: f64) -> Vec3 {
        let n = p.normalize();
        let noise = (37.1 * p.x + 11.3 * p.y + 5.7 * p.z).sin() * (23.9 * p.y - 13.1 * p.z).cos();
        n * (R + sigma * noise * std::f64::consts::SQRT_2)
    }

    fn curve_gap(c: &[Vec3; 4]) -> f64 {
        (0..=CHECK).map(|k| (curve_point(c, k as f64 / CHECK as f64).xy().norm() - R).abs()).fold(0.0, f64::max)
    }

    fn net_rms(net: &Net) -> f64 {
        let mut sum = 0.0;
        for i in 0..=CHECK {
            for j in 0..=CHECK {
                let (p, _, _) = eval(net, i as f64 / CHECK as f64, j as f64 / CHECK as f64);
                sum += (p.norm() - R).powi(2);
            }
        }
        (sum / ((CHECK + 1) * (CHECK + 1)) as f64).sqrt()
    }

    fn at_angle(deg: f64, z: f64) -> Vec3 {
        let a = deg.to_radians();
        Vec3::new(R * a.cos(), R * a.sin(), z)
    }

    #[test]
    fn a_curve_over_a_strongly_curved_arc_is_refitted_much_closer_to_it() {
        // Arrange: a 100 degree arc of a cylinder of radius 10, rising 3 mm
        let (a, b) = (at_angle(0.0, 0.0), at_angle(100.0, 3.0));

        // Act
        let chord = chord_curve(a, b, &on_cylinder);
        let refit = refit_curve(chord, &on_cylinder);

        // Assert
        let (before, after) = (curve_gap(&chord), curve_gap(&refit));
        eprintln!("largest gap from the arc: chord interpolant {before:.4} mm, refitted {after:.4} mm");
        assert!(after < 0.25 * before, "{before} -> {after}");
        assert_eq!((refit[0], refit[3]), (a, b), "the ends stay where they are");
    }

    fn sphere_corner(lat: f64, lon: f64) -> Vec3 {
        let (la, lo) = (lat.to_radians(), lon.to_radians());
        Vec3::new(R * la.cos() * lo.cos(), R * la.cos() * lo.sin(), R * la.sin())
    }

    #[test]
    fn a_patch_over_a_noisy_sphere_is_refitted_closer_to_the_sphere() {
        // Arrange: a 40 x 40 degree patch whose boundary lies on the sphere, the inside taken from
        // a scan with noise 0.05 mm
        let c = [
            sphere_corner(-20.0, -20.0),
            sphere_corner(-20.0, 20.0),
            sphere_corner(20.0, 20.0),
            sphere_corner(20.0, -20.0),
        ];
        let exact = |p: Vec3| on_noisy_sphere(p, 0.0);
        let curve = |x: Vec3, y: Vec3| refit_curve(chord_curve(x, y, &exact), &exact);
        let (bot, right, top, left) = (curve(c[0], c[1]), curve(c[1], c[2]), curve(c[3], c[2]), curve(c[0], c[3]));
        let noisy = |p: Vec3| on_noisy_sphere(p, 0.05);

        // Act
        let interpolated = interpolated_net(&bot, &right, &top, &left, &noisy);
        let refit = refit_net(interpolated, &noisy);

        // Assert
        let (before, after) = (net_rms(&interpolated), net_rms(&refit));
        eprintln!("RMS distance from the sphere: interpolated {before:.4} mm, refitted {after:.4} mm");
        assert!(after < 0.6 * before, "{before} -> {after}");
        let boundary_kept = (0..4).all(|k| refit[k][0] == interpolated[k][0] && refit[3][k] == interpolated[3][k]);
        assert!(boundary_kept, "the boundary is shared with the neighbours and stays");
    }

    #[test]
    fn a_projection_that_gives_nothing_usable_leaves_the_curve_and_the_patch_as_they_were() {
        // Arrange
        let (a, b) = (at_angle(0.0, 0.0), at_angle(60.0, 1.0));
        let c = chord_curve(a, b, &on_cylinder);
        let net = interpolated_net(&c, &c, &c, &c, &on_cylinder);

        for project in [&(|_: Vec3| Vec3::repeat(f64::NAN)) as &dyn Fn(Vec3) -> Vec3, &|p: Vec3| p] {
            // Act
            let curve = refit_curve(c, project);
            let patch = refit_net(net, project);

            // Assert
            assert_eq!(curve, c);
            assert_eq!(patch, net);
        }
    }

    #[test]
    fn a_refit_never_puts_its_samples_farther_from_the_scan() {
        for (deg, sigma) in [(20.0, 0.0), (90.0, 0.02), (150.0, 0.1)] {
            // Arrange
            let project = |p: Vec3| on_noisy_sphere(p, sigma);
            let c = chord_curve(sphere_corner(0.0, 0.0), sphere_corner(10.0, deg), &project);

            // Act
            let refit = refit_curve(c, &project);

            // Assert
            assert!(curve_targets(&refit, &project).1 <= curve_targets(&c, &project).1, "{deg} {sigma}");
        }
    }
}
