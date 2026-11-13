use super::*;
use crate::{Rng, Vec3, synth};

fn noisy(mut m: Mesh, sigma: f64, seed: u64) -> Mesh {
    m.weld(1e-9);
    synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    m
}

fn volume(m: &Mesh) -> f64 {
    signed_volume(m)
}

#[test]
fn a_noisy_sphere_becomes_a_closed_manifold_of_genus_zero_close_to_the_sphere() {
    let scan = noisy(synth::sphere(Point3::origin(), 15.0, 120, 60), 0.03, 3);
    let (out, rep) = remesh(&scan, &Params::default()).unwrap();
    let ps = pieces::pieces(&out);
    assert_eq!(ps.len(), 1);
    assert!(ps[0].closed());
    assert_eq!(ps[0].genus(), Some(0));
    let worst = out.positions.iter().map(|p| (p.coords.norm() - 15.0).abs()).fold(0.0, f64::max);
    assert!(worst < 0.1, "max deviation from the exact sphere {worst}; {rep:?}");
    let exact = 4.0 / 3.0 * std::f64::consts::PI * 15.0f64.powi(3);
    assert!((volume(&out) - exact).abs() / exact < 0.005, "{} vs {exact}", volume(&out));
}

#[test]
fn a_torus_keeps_its_handle() {
    let scan = noisy(synth::torus(Point3::origin(), Vec3::z(), 12.0, 4.0, 120, 48), 0.02, 4);
    let (out, _) = remesh(&scan, &Params::default()).unwrap();
    let ps = pieces::pieces(&out);
    assert_eq!(ps.len(), 1);
    assert_eq!(ps[0].genus(), Some(1));
    assert!(ps[0].closed());
}

#[test]
fn an_open_sheet_is_refused() {
    let sheet = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 30.0, 30.0, 40, 40);
    let err = remesh(&sheet, &Params::default()).unwrap_err();
    assert!(matches!(err, RemeshError::Open { .. }), "{err}");
}

#[test]
fn a_resolution_beyond_the_limits_is_refused_before_memory_is_spent() {
    let scan = synth::sphere(Point3::origin(), 15.0, 80, 40);
    let tight = Params { limits: Limits { max_columns: 1000, ..Limits::default() }, ..Params::default() };
    assert!(matches!(remesh(&scan, &tight), Err(RemeshError::TooMuchWork { .. })));
    let fine = Params { voxel: Some(0.01), ..Params::default() };
    assert!(matches!(remesh(&scan, &fine), Err(RemeshError::TooMuchWork { .. })));
}
