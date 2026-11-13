//! Tests of the reconstruction: meshes recovered from synthetic clouds and degenerate inputs.

use super::*;

use crate::{Rng, synth};

fn cloud_of(mesh: &Mesh, n: usize, sigma: f64, rng: &mut Rng) -> PointCloud {
    let pts = mesh.sample_points(n, rng).into_iter().map(|(p, nrm)| p + nrm * (sigma * rng.gauss())).collect();
    PointCloud::from_points(pts)
}

#[test]
fn sphere_cloud_becomes_a_closed_mesh() {
    let mut rng = Rng::new(7);
    let sphere = synth::sphere(Point3::new(1.0, 2.0, 3.0), 10.0, 64, 32);
    let c = cloud_of(&sphere, 6000, 0.02, &mut rng);
    let (m, rep) = reconstruct(&c, &ReconParams::default()).unwrap();
    assert!(m.validate().is_ok());
    assert!(rep.holes_left <= 2, "{rep:?}");
    for p in &m.positions {
        let r = (p - Point3::new(1.0, 2.0, 3.0)).norm();
        assert!((r - 10.0).abs() < 0.1, "{r}");
    }
    // area of a sphere of radius 10 is 1257 mm²
    assert!((m.area() - 1256.6).abs() < 80.0, "{}", m.area());
    // outward orientation: the signed volume is positive
    let mut vol = 0.0;
    for i in 0..m.triangle_count() {
        if let Some([a, b, c]) = m.triangle(i) {
            vol += a.coords.dot(&b.coords.cross(&c.coords)) / 6.0;
        }
    }
    assert!(vol > 0.0, "{vol}");
}

/// Degenerate and extreme clouds are refused or reconstructed, never a panic.
#[test]
fn hostile_clouds_never_panic() {
    let mut rng = Rng::new(11);
    let sphere = synth::sphere(Point3::origin(), 10.0, 32, 16);
    let base = cloud_of(&sphere, 800, 0.0, &mut rng);
    let scaled = |f: f64| PointCloud::from_points(base.positions.iter().map(|p| Point3::from(p.coords * f)).collect());
    let mut clouds = vec![
        PointCloud::from_points(vec![Point3::new(1.0, 2.0, 3.0); 100]),
        PointCloud::from_points((0..200).map(|i| Point3::new(f64::from(i) * 0.1, 0.0, 0.0)).collect()),
        // a perfectly regular grid: every four neighbours are cospherical
        PointCloud::from_points((0..400).map(|i| Point3::new(f64::from(i % 20), f64::from(i / 20), 0.0)).collect()),
        scaled(1e-9),
        scaled(1e8),
        scaled(1e12),
        PointCloud::from_points((0..500).map(|_| Point3::from(rng.unit_vector() * rng.range(0.0, 50.0))).collect()),
        PointCloud::from_points(
            (0..300).map(|i| Point3::new(if i % 2 == 0 { 0.0 } else { 1e6 }, 0.0, 0.0) + rng.unit_vector()).collect(),
        ),
    ];
    // normals: all zero, random, and given on a sphere
    let mut c = base.clone();
    c.normals = Some(vec![Vec3::zeros(); base.len()]);
    clouds.push(c);
    let mut c = base.clone();
    c.normals = Some((0..base.len()).map(|_| rng.unit_vector() * 1e-30).collect());
    clouds.push(c);
    for c in &clouds {
        for params in [
            ReconParams::default(),
            ReconParams { radius_scale: 0.5, passes: 1, fill_holes_max_edges: 0, ..ReconParams::default() },
            ReconParams { radius_scale: 50.0, passes: 6, normals_k: 3, ..ReconParams::default() },
            ReconParams { normals_k: 64, ..ReconParams::default() },
        ] {
            if let Ok((m, _)) = reconstruct(c, &params) {
                assert!(m.validate().is_ok());
            }
        }
    }
}

#[test]
fn given_normals_are_used_and_degenerate_input_is_refused() {
    let mut rng = Rng::new(3);
    let plane = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 20.0, 20.0, 4, 4);
    let s = plane.sample_points(1500, &mut rng);
    let c = PointCloud::new(s.iter().map(|(p, _)| *p).collect(), Some(s.iter().map(|(_, n)| *n).collect())).unwrap();
    let (m, rep) = reconstruct(&c, &ReconParams::default()).unwrap();
    assert!(rep.normals_given);
    assert!((m.area() - 400.0).abs() < 40.0, "{}", m.area());
    assert!(matches!(
        reconstruct(&PointCloud::from_points(vec![Point3::origin(); 50]), &ReconParams::default()),
        Err(ReconError::TooFew(1))
    ));
    let line: Vec<Point3> = (0..100).map(|i| Point3::new(f64::from(i), 0.0, 0.0)).collect();
    assert!(reconstruct(&PointCloud::from_points(line), &ReconParams::default()).is_err());
    let bad = ReconParams { radius_scale: f64::NAN, ..ReconParams::default() };
    assert!(reconstruct(&c, &bad).is_err());
}
