//! Tests of the deviation analysis: statistics, distributions and samples on synthetic scans.

use super::*;

use surfacecraft_geom::{Rng, synth};

#[test]
fn stats_colors_and_noise() {
    let s = stats(&[0.0, 0.05, -0.05, 0.3, f64::NAN], 0.1);
    assert_eq!(s.count, 4);
    assert!((s.within - 0.75).abs() < 1e-12);
    assert!((s.max_abs - 0.3).abs() < 1e-12);
    assert!((s.min + 0.05).abs() < 1e-12 && (s.max - 0.3).abs() < 1e-12);
    assert_eq!(s.histogram.iter().sum::<usize>(), s.count);
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
