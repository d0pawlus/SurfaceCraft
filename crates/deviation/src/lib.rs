//! SurfaceCraft deviation analysis (layer L2): how far scan points are from a reference, as
//! statistics and a colour map (design doc §7: "deviation as constant feedback").
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Bvh, KdTree, Mesh, Point3, Vec3};

/// Deviation statistics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct DeviationStats {
    /// Points measured.
    pub count: usize,
    /// Largest |deviation| (mm).
    pub max_abs: f64,
    /// Mean signed deviation (mm).
    pub mean: f64,
    /// RMS deviation (mm).
    pub rms: f64,
    /// Tolerance used for the colour map and the in-tolerance share (mm).
    pub tolerance: f64,
    /// Share of points within ±tolerance (0..1).
    pub within: f64,
    /// Ten histogram bins of signed deviation over [−3·tol, 3·tol] (outside counts in the end bins).
    pub histogram: [usize; 10],
}

/// Statistics of signed deviations.
#[must_use]
pub fn stats(values: &[f64], tolerance: f64) -> DeviationStats {
    let tol = if tolerance.is_finite() && tolerance > 0.0 { tolerance } else { 0.1 };
    let vals: Vec<f64> = values.iter().copied().filter(|v| v.is_finite()).collect();
    let n = vals.len();
    let mut h = [0usize; 10];
    let (mut sum, mut sq, mut max, mut within) = (0.0, 0.0, 0.0f64, 0usize);
    for &d in &vals {
        sum += d;
        sq += d * d;
        max = max.max(d.abs());
        if d.abs() <= tol {
            within += 1;
        }
        let bin = (((d / (3.0 * tol)) + 1.0) * 5.0).floor().clamp(0.0, 9.0) as usize;
        if let Some(b) = h.get_mut(bin) {
            *b += 1;
        }
    }
    let nf = n.max(1) as f64;
    DeviationStats {
        count: n,
        max_abs: max,
        mean: sum / nf,
        rms: (sq / nf).sqrt(),
        tolerance: tol,
        within: within as f64 / nf,
        histogram: h,
    }
}

/// Signed deviation of each point from a primitive.
#[must_use]
pub fn against_primitive(points: &[Point3], prim: &Primitive) -> Vec<f64> {
    points.iter().map(|p| prim.distance(p)).collect()
}

/// Signed deviation of each point from a mesh surface (sign from the triangle normal; NaN when
/// farther than `max_distance`).
#[must_use]
pub fn against_mesh(points: &[Point3], target: &Bvh, max_distance: f64) -> Vec<f64> {
    surfacecraft_geom::par::map_slice(points, |p| match target.closest(p, max_distance) {
        Some(h) => {
            let sign = target.triangle_normal(h.triangle).map_or(1.0, |n| (p - h.point).dot(&n).signum());
            h.distance * if sign == 0.0 { 1.0 } else { sign }
        }
        None => f64::NAN,
    })
}

/// Colour per value: green within ±tol, through yellow/orange to red above, through cyan to
/// blue below; grey for NaN (not measured).
#[must_use]
pub fn colorize(values: &[f64], tolerance: f64) -> Vec<[f32; 4]> {
    let tol = if tolerance.is_finite() && tolerance > 0.0 { tolerance } else { 0.1 };
    values
        .iter()
        .map(|&d| {
            if !d.is_finite() {
                return [0.45, 0.45, 0.45, 1.0];
            }
            let t = (d / tol) as f32;
            if t.abs() <= 1.0 {
                [0.2, 0.8, 0.3, 1.0]
            } else if t > 0.0 {
                let k = ((t - 1.0) / 2.0).clamp(0.0, 1.0);
                [1.0, 0.85 * (1.0 - k), 0.1, 1.0]
            } else {
                let k = ((-t - 1.0) / 2.0).clamp(0.0, 1.0);
                [0.1, 0.75 * (1.0 - k), 1.0, 1.0]
            }
        })
        .collect()
}

/// Noise estimate of a scan (σ, mm): median distance of vertices to a local quadric through
/// their neighbours, scaled to a standard deviation (design doc §10: warn when a tolerance is
/// below the scanner noise). Samples whose neighbourhood spans a sharp edge (neighbour normals
/// more than 30° off the local plane) are left out when enough smooth samples remain, so edges
/// and corners do not inflate the estimate.
#[must_use]
pub fn estimate_noise(mesh: &Mesh) -> f64 {
    let n = mesh.vertex_count();
    if n < 16 {
        return 0.0;
    }
    let tree = KdTree::build(&mesh.positions);
    let stride = n.div_ceil(4000).max(1);
    let normals = mesh.vertex_normals();
    let cos_sharp = 30f64.to_radians().cos();
    let mut d: Vec<f64> = Vec::new();
    let mut d_smooth: Vec<f64> = Vec::new();
    for p in mesh.positions.iter().step_by(stride) {
        let near = tree.knn(p, 17);
        // neighbours without the point itself (leave-one-out), local quadric height field
        let nb: Vec<Point3> =
            near.iter().filter(|(_, d2)| *d2 > 0.0).filter_map(|(i, _)| mesh.positions.get(*i).copied()).collect();
        let Some(f) = surfacecraft_fit::plane::fit_plane(&nb, None) else { continue };
        let z = f.normal;
        let sharp =
            near.iter().filter_map(|(i, _)| normals.get(*i)).any(|nn| nn.norm() > 0.5 && nn.dot(&z).abs() < cos_sharp);
        let x = surfacecraft_geom::any_perpendicular(&z);
        let y = z.cross(&x);
        let o = f.point.coords;
        let mut ata = nalgebra::Matrix6::<f64>::zeros();
        let mut atb = nalgebra::Vector6::<f64>::zeros();
        for q in &nb {
            let w = q.coords - o;
            let (u, v, h) = (w.dot(&x), w.dot(&y), w.dot(&z));
            let row = nalgebra::Vector6::new(u * u, u * v, v * v, u, v, 1.0);
            ata += row * row.transpose();
            atb += row * h;
        }
        let Some(c) = ata.lu().solve(&atb) else { continue };
        let w = p.coords - o;
        let (u, v, h) = (w.dot(&x), w.dot(&y), w.dot(&z));
        let pred = c[0] * u * u + c[1] * u * v + c[2] * v * v + c[3] * u + c[4] * v + c[5];
        d.push((h - pred).abs());
        if !sharp {
            d_smooth.push((h - pred).abs());
        }
    }
    if d_smooth.len() >= 50 && d_smooth.len() * 5 >= d.len() {
        d = d_smooth;
    }
    if d.is_empty() {
        return 0.0;
    }
    d.sort_by(f64::total_cmp);
    // median |x| of a normal variable is 0.6745 σ; the prediction's own error adds ≈ 10 %
    d.get(d.len() / 2).copied().unwrap_or(0.0) / 0.6745 / 1.1
}

/// Unused helper kept for symmetry with primitives: the outward normal of a primitive at `p`.
#[must_use]
pub fn primitive_normal(prim: &Primitive, p: &Point3) -> Vec3 {
    prim.normal(p)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    #[test]
    fn stats_colors_and_noise() {
        let s = stats(&[0.0, 0.05, -0.05, 0.3, f64::NAN], 0.1);
        assert_eq!(s.count, 4);
        assert!((s.within - 0.75).abs() < 1e-12);
        assert!((s.max_abs - 0.3).abs() < 1e-12);
        let c = colorize(&[0.0, 0.3, -0.3, f64::NAN], 0.1);
        assert_eq!(c[0], [0.2, 0.8, 0.3, 1.0]);
        assert!(c[1][0] > 0.9 && c[2][2] > 0.9);
        let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 50.0, 50.0, 60, 60);
        synth::add_noise(&mut m, 0.05, &mut Rng::new(1));
        let sigma = estimate_noise(&m);
        assert!((0.03..0.08).contains(&sigma), "noise estimate {sigma}");
        let flat = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 50.0, 50.0, 60, 60);
        let dev = against_mesh(&m.positions, &Bvh::build(&flat), 1.0);
        assert!((stats(&dev, 0.1).rms - 0.05).abs() < 0.01);
    }

    /// P1: edges and corners do not inflate the estimate (was 0.078 for σ = 0.05 on the bracket),
    /// nor does the density ramp of a scan.
    #[test]
    fn noise_estimate_is_robust_to_edges_and_density() {
        let mut m = synth::bracket();
        synth::add_noise(&mut m, 0.05, &mut Rng::new(4));
        let sigma = estimate_noise(&m);
        eprintln!("bracket σ=0.05 → {sigma:.4}");
        assert!((0.04..0.062).contains(&sigma), "bracket σ estimate {sigma}");
        let mut b = synth::cuboid(Point3::origin(), Vec3::new(30.0, 20.0, 10.0), 1);
        b.weld(1e-9);
        let d = synth::ScanDefects { noise: 0.03, holes: 0, spikes: 0.0, debris: 0, ..synth::ScanDefects::default() };
        let s = synth::scanify(&b, &d, &mut Rng::new(2));
        let sigma = estimate_noise(&s);
        eprintln!("scanned box σ=0.03 → {sigma:.4}");
        assert!((0.022..0.04).contains(&sigma), "scanned box σ estimate {sigma}");
    }
}
