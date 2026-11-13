//! Tests of the primitive fits: recovered primitives from synthetic points and degenerate inputs.

use super::estimate::*;
use super::model::*;
use super::*;

use surfacecraft_geom::{Rng, synth};

pub(crate) fn sample(m: &Mesh) -> (Vec<Point3>, Vec<Vec3>) {
    (m.positions.clone(), m.vertex_normals())
}

fn noisy(mut m: Mesh, sigma: f64) -> Mesh {
    synth::add_noise(&mut m, sigma, &mut Rng::new(11));
    m
}

#[test]
fn exact_fits_recover_parameters() {
    let s = synth::sphere(Point3::new(1.0, 2.0, 3.0), 7.0, 32, 16);
    let (p, n) = sample(&s);
    let f = fit(PrimitiveKind::Sphere, &p, &n).unwrap();
    assert!(f.rms < 1e-9, "{f:?}");

    let axis = Vec3::new(1.0, 1.0, 2.0).normalize();
    let c = synth::cylinder(Point3::new(5.0, 0.0, -3.0), axis, 4.0, 20.0, 48, 10);
    let (p, n) = sample(&c);
    let f = fit(PrimitiveKind::Cylinder, &p, &n).unwrap();
    assert!(f.rms < 1e-8, "{f:?}");
    let Primitive::Cylinder { axis: a, radius, .. } = f.primitive else { panic!() };
    assert!((radius - 4.0).abs() < 1e-8 && v(&a).cross(&axis).norm() < 1e-8);

    let k = synth::cone(Point3::new(0.0, 0.0, 0.0), axis, 10.0, 4.0, 15.0, 48, 10);
    let (p, n) = sample(&k);
    let f = fit(PrimitiveKind::Cone, &p, &n).unwrap();
    assert!(f.rms < 1e-7, "{f:?}");
    let Primitive::Cone { half_angle, .. } = f.primitive else { panic!() };
    assert!((half_angle - (6.0f64 / 15.0).atan()).abs() < 1e-7);

    let t = synth::torus(Point3::new(1.0, -1.0, 2.0), axis, 12.0, 3.0, 48, 24);
    let (p, n) = sample(&t);
    let f = fit(PrimitiveKind::Torus, &p, &n).unwrap();
    assert!(f.rms < 1e-7, "{f:?}");
    let Primitive::Torus { major_radius, minor_radius, .. } = f.primitive else { panic!() };
    assert!((major_radius - 12.0).abs() < 1e-6 && (minor_radius - 3.0).abs() < 1e-6);
}

/// Small patches of a torus with a large tube (a yoke-like body): the tube radius exceeds half
/// the patch extent and the circle of tube centres is only a short arc, so the fit must not
/// drift to a spindle torus or a negative major radius.
#[test]
fn torus_patches_with_large_tubes_are_fitted() {
    let axis = Vec3::new(0.3, -0.2, 1.0).normalize();
    let center = Point3::new(1.0, -2.0, 3.0);
    let x = surfacecraft_geom::any_perpendicular(&axis);
    let y = axis.cross(&x);
    // narrow in the major angle (±8°), wide in the tube angle (±60°) around the outer equator
    for (k, (big_r, r)) in [(34.0, 20.0), (40.0, 21.0), (46.0, 25.0)].into_iter().enumerate() {
        let full = synth::torus(center, axis, big_r, r, 180, 120);
        let normals = full.vertex_normals();
        let mut rng = Rng::new(3 + k as u64);
        let (mut p, mut n) = (Vec::new(), Vec::new());
        for (q, nq) in full.positions.iter().zip(&normals) {
            let d = q - center;
            let (h, th) = (d.dot(&axis), d.dot(&y).atan2(d.dot(&x)));
            let ph = h.atan2(d.dot(&x).hypot(d.dot(&y)) - big_r);
            if th.to_degrees().abs() <= 8.0 && ph.to_degrees().abs() <= 60.0 {
                // noise of the scan (σ = 0.05 mm) and of its vertex normals
                p.push(q + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.05);
                n.push((nq + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.05).normalize());
            }
        }
        let f = fit(PrimitiveKind::Torus, &p, &n).expect("a torus fit");
        let Primitive::Torus { major_radius, minor_radius, .. } = f.primitive else { panic!() };
        assert!(f.rms <= 0.1, "torus {k}: rms {} major {major_radius} minor {minor_radius}", f.rms);
        assert!(minor_radius < major_radius, "torus {k}: spindle torus {major_radius} {minor_radius}");
    }
}

/// A broad partial torus (200° round the axis, the tube from 30° to 80° above the equator)
/// whose scan normals point towards the tube centre, on a large part: the start with the wrong
/// normal side stalls at a spurious, penalty-bound torus whose RMS is still below 0.3 % of the
/// extent, so the fit must try the most promising start (the smallest initial residual) first
/// and must not accept a penalty-bound result as converged.
#[test]
fn broad_concave_torus_patch_is_not_stopped_at_a_spurious_start() {
    let axis = Vec3::new(-0.63, -0.03, -0.77).normalize();
    let center = Point3::new(-18.0, -1.8, 29.0);
    let x = surfacecraft_geom::any_perpendicular(&axis);
    let y = axis.cross(&x);
    let (big_r, r) = (55.5, 15.2);
    let full = synth::torus(center, axis, big_r, r, 360, 120);
    let normals = full.vertex_normals();
    let mut rng = Rng::new(7);
    let (mut p, mut n) = (Vec::new(), Vec::new());
    for (q, nq) in full.positions.iter().zip(&normals) {
        let d = q - center;
        let (h, th) = (d.dot(&axis), d.dot(&y).atan2(d.dot(&x)));
        let ph = h.atan2(d.dot(&x).hypot(d.dot(&y)) - big_r);
        if (-50.0..=150.0).contains(&th.to_degrees()) && (30.0..=80.0).contains(&ph.to_degrees()) {
            p.push(q + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.02);
            // scan normals towards the tube centre
            n.push((-nq + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.01).normalize());
        }
    }
    let f = fit(PrimitiveKind::Torus, &p, &n).expect("a torus fit");
    let Primitive::Torus { major_radius, minor_radius, .. } = f.primitive else { panic!() };
    eprintln!("{} pts: rms {} major {major_radius} minor {minor_radius}", p.len(), f.rms);
    assert!(f.rms <= 0.1, "rms {} major {major_radius} minor {minor_radius}", f.rms);
    assert!((major_radius - big_r).abs() < 1.0 && (minor_radius - r).abs() < 0.5, "{major_radius} {minor_radius}");
}

/// The foot fillet of a boss (a quarter round, radius 2, about a ring of radius 10 on a boss of
/// radius 8): a full revolution of a short, small tube arc, with the scan normals pointing into
/// the void (towards the tube centre) or the other way.
#[test]
fn boss_foot_quarter_round_is_fitted() {
    let axis = Vec3::new(0.0, 0.0, 1.0);
    let center = Point3::new(30.0, 20.0, 10.0);
    let x = Vec3::x();
    let y = Vec3::y();
    for flip in [false, true] {
        let full = synth::torus(center, axis, 10.0, 2.0, 360, 90);
        let normals = full.vertex_normals();
        let mut rng = Rng::new(5);
        let (mut p, mut n) = (Vec::new(), Vec::new());
        for (q, nq) in full.positions.iter().zip(&normals) {
            let d = q - center;
            let rho = d.dot(&x).hypot(d.dot(&y));
            // the quadrant inside the corner: towards the axis and down
            if rho < 9.95 && d.z < -0.05 {
                p.push(q + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.03);
                let m = if flip { -nq } else { *nq };
                n.push((m + Vec3::new(rng.gauss(), rng.gauss(), rng.gauss()) * 0.03).normalize());
            }
        }
        let f = fit(PrimitiveKind::Torus, &p, &n).expect("a torus fit");
        let Primitive::Torus { major_radius, minor_radius, .. } = f.primitive else { panic!() };
        eprintln!("flip {flip}: {} pts rms {} major {major_radius} minor {minor_radius}", p.len(), f.rms);
        assert!(
            (major_radius - 10.0).abs() < 0.3 && (minor_radius - 2.0).abs() < 0.1,
            "flip {flip}: major {major_radius} minor {minor_radius}"
        );
    }
}

/// Gate M2: RMS ≤ 0.1 mm on noisy synthetic data (σ = 0.05 mm) for every kind.
#[test]
fn gate_m2_noisy_fits_within_rms() {
    let axis = Vec3::new(0.3, -0.2, 1.0).normalize();
    let cases: Vec<(PrimitiveKind, Mesh)> = vec![
        (PrimitiveKind::Plane, synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 30.0, 40, 30)),
        (PrimitiveKind::Sphere, synth::sphere(Point3::new(3.0, 4.0, 5.0), 15.0, 64, 32)),
        (PrimitiveKind::Cylinder, synth::cylinder(Point3::origin(), axis, 8.0, 30.0, 64, 20)),
        (PrimitiveKind::Cone, synth::cone(Point3::origin(), axis, 12.0, 5.0, 20.0, 64, 20)),
        (PrimitiveKind::Torus, synth::torus(Point3::origin(), axis, 20.0, 5.0, 96, 32)),
    ];
    for (kind, mesh) in cases {
        let m = noisy(mesh, 0.05);
        let (p, n) = sample(&m);
        let f = fit(kind, &p, &n).unwrap_or_else(|| panic!("{kind:?} failed"));
        eprintln!("gate_m2 {kind:?}: rms {:.4} mm max {:.4} mm", f.rms, f.max);
        assert!(f.rms <= 0.1, "{kind:?} rms {}", f.rms);
    }
}

#[test]
fn patch_mesh_covers_points() {
    let c = synth::cylinder(Point3::origin(), Vec3::z(), 5.0, 10.0, 32, 4);
    let (p, n) = sample(&c);
    let f = fit(PrimitiveKind::Cylinder, &p, &n).unwrap();
    let m = f.primitive.patch_mesh(&p, 16);
    let b = m.bbox();
    assert!((b.size().z - 10.0).abs() < 1e-6 && (b.size().x - 10.0).abs() < 0.1);
    assert!(angular_range([0.1, 0.2, 6.2].into_iter()).1 - angular_range([0.1, 0.2, 6.2].into_iter()).0 < 1.0);
}

#[test]
fn bad_input_gives_none() {
    assert!(fit(PrimitiveKind::Sphere, &[], &[]).is_none());
    let line: Vec<Point3> = (0..20).map(|i| Point3::new(f64::from(i), 0.0, 0.0)).collect();
    assert!(fit(PrimitiveKind::Plane, &line, &[]).is_none());
    assert!(fit(PrimitiveKind::Sphere, &[Point3::new(f64::NAN, 0.0, 0.0); 10], &[]).is_none());
}
