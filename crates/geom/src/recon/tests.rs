//! Tests of the reconstruction: meshes recovered from synthetic clouds and degenerate inputs.

use super::*;
use std::f64::consts::PI;

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

/// The points as a file in another unit holds them: scaled, written as text and read back, then
/// converted to millimetres again (what `mesh.import` with `units` does).
fn through_a_unit_file(pts: &[Point3], per_mm: f64, mm_per_unit: f64) -> Vec<Point3> {
    let f = |v: f64| format!("{}", v * per_mm).parse::<f64>().unwrap_or(f64::NAN) * mm_per_unit;
    pts.iter().map(|p| Point3::new(f(p.x), f(p.y), f(p.z))).collect()
}

/// A regular lattice (the bracket tessellation) is full of exact ties; a copy converted through
/// another unit differs in the last bits of many coordinates and must still give the same mesh.
#[test]
fn units_do_not_change_the_reconstruction_of_a_regular_cloud() {
    // Arrange
    let original = synth::bracket().positions;
    let (want, _) = reconstruct(&PointCloud::from_points(original.clone()), &ReconParams::default()).unwrap();
    for (per_mm, mm_per_unit) in [(0.001, 1000.0), (0.1, 10.0), (1.0 / 25.4, 25.4), (1.0 / 304.8, 304.8), (1000.0, 0.001)]
    {
        // Act
        let converted = through_a_unit_file(&original, per_mm, mm_per_unit);
        let (got, _) = reconstruct(&PointCloud::from_points(converted), &ReconParams::default()).unwrap();
        // Assert
        assert_eq!(got.triangles, want.triangles, "factor {per_mm}");
        assert_eq!(got.positions, want.positions, "factor {per_mm}");
    }
}

#[test]
fn snapping_moves_points_by_a_billionth_of_the_extent_and_is_idempotent() {
    let pts = vec![
        Point3::new(0.1, 0.2, 0.3),
        Point3::new(100.0 / 3.0, 7.7, -2.5),
        Point3::new(5.0, 5.0, 5.0),
        Point3::new(60.0, 40.0, 20.0),
    ];
    let once = snap::snapped(&pts);
    for (a, b) in pts.iter().zip(&once) {
        assert!((a - b).norm() <= 1e-9 * 60.0, "{a:?} moved to {b:?}");
    }
    assert_eq!(snap::snapped(&once), once);
    // exact lattice values stay as they are
    assert_eq!(once.get(2), pts.get(2));
}

#[test]
fn snapping_leaves_degenerate_extents_alone() {
    let one = vec![Point3::new(1.5, -2.0, 3.0)];
    assert_eq!(snap::snapped(&one), one);
    assert!(snap::snapped(&[]).is_empty());
    let tiny = vec![Point3::origin(), Point3::new(1e-310, 0.0, 0.0)];
    assert_eq!(snap::snapped(&tiny), tiny);
}

#[test]
fn pseudo_angle_orders_like_the_angle() {
    let mut last = f64::NEG_INFINITY;
    for step in -199..=200 {
        let a = f64::from(step) * PI / 200.0;
        let p = pivot::pseudo_angle(a.sin(), a.cos());
        assert!(p > last, "not monotone at {a}: {p} after {last}");
        last = p;
    }
    assert_eq!(pivot::pseudo_angle(0.0, 0.0), 0.0);
    assert_eq!(pivot::pseudo_angle(0.0, -1.0), 2.0);
    assert!((pivot::pseudo_angle(1.0, 0.0) - 1.0).abs() < 1e-15);
}

#[test]
fn lattice_step_ignores_noise_at_a_power_of_two_extent_and_caps_outliers() {
    let part = |hi: f64| vec![Point3::new(10.0, 0.0, 0.0), Point3::new(hi, 1.0, 1.0)];
    let hi = 74.0_f64;
    let below = f64::from_bits(hi.to_bits() - 1);
    assert_eq!(snap::lattice_step(&part(hi)), snap::lattice_step(&part(below)));
    // one stray point at 1e7 mm must not coarsen the lattice beyond a nanometre
    let mut with_outlier = synth::bracket().positions;
    with_outlier.push(Point3::new(1e7, 0.0, 0.0));
    let step = snap::lattice_step(&with_outlier).unwrap();
    assert!(step <= 1e-6, "{step}");
    let moved = snap::snapped(&with_outlier).iter().zip(&with_outlier).map(|(a, b)| (a - b).norm()).fold(0.0, f64::max);
    assert!(moved <= 1e-6, "{moved}");
}
