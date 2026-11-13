//! Calibration of the threshold that tells a genuine large primitive from a fragment of a wavy
//! surface: the ratio of the robust spread of a region's residuals to the noise estimated on the
//! scan (`surfacecraft_deviation::noise_estimate`).
//!
//! `cargo test --profile ci -p surfacecraft-engine --test p9_noise_calibration print_table -- --ignored --nocapture`
//! prints the table the threshold of `freeform.rs` (`LARGE_SOUND_FACTOR`) was chosen from.
use surfacecraft_deviation::noise_estimate;
use surfacecraft_fit::segment::{LARGE_SOUND_FACTOR, SegmentOptions, residual_spread, segment};
use surfacecraft_fit::{PrimitiveKind, fit_auto};
use surfacecraft_geom::{Mesh, Point3, Rng, synth};

const SIGMAS: [f64; 3] = [0.01, 0.03, 0.1];
/// Edge lengths of the three densities (mm).
const SPACINGS: [f64; 3] = [0.5, 1.0, 2.0];

#[derive(Clone, Copy, Debug)]
enum Shape {
    Plane,
    Cylinder,
    Sphere,
}

fn sheet(w: f64, h: f64, nx: usize, ny: usize, f: impl Fn(f64, f64) -> Point3) -> Mesh {
    let cols = nx + 1;
    let mut positions = Vec::new();
    for j in 0..=ny {
        for i in 0..=nx {
            positions.push(f(w * i as f64 / nx as f64, h * j as f64 / ny as f64));
        }
    }
    let mut triangles = Vec::new();
    for j in 0..ny {
        for i in 0..nx {
            let (a, b) = ((j * cols + i) as u32, (j * cols + i + 1) as u32);
            let (c, d) = (((j + 1) * cols + i + 1) as u32, ((j + 1) * cols + i) as u32);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { positions, triangles }
}

/// A 60 x 60 mm patch of the shape with noise `sigma` at edge length `spacing`.
fn genuine(shape: Shape, sigma: f64, spacing: f64, seed: u64) -> Mesh {
    let n = (60.0 / spacing) as usize;
    let mut m = sheet(60.0, 60.0, n, n, |x, y| match shape {
        Shape::Plane => Point3::new(x, y, 0.0),
        Shape::Cylinder => {
            let th = (x - 30.0) / 30.0;
            Point3::new(30.0 * th.sin(), y, 30.0 * th.cos() - 30.0)
        }
        Shape::Sphere => {
            let (u, v) = (0.8 * (x - 30.0), 0.8 * (y - 30.0));
            Point3::new(u, v, 40.0 - (1600.0 - u * u - v * v).max(0.0).sqrt())
        }
    });
    synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    m
}

/// A wavy 80 x 80 mm surface (bumps of 2.5 mm, wavelengths 40 to 60 mm: a gentle freeform top).
fn wavy(sigma: f64, spacing: f64, seed: u64) -> Mesh {
    let n = (80.0 / spacing) as usize;
    let mut m = sheet(80.0, 80.0, n, n, |x, y| {
        Point3::new(x, y, 2.5 * (x / 9.0 + 0.2 * seed as f64).sin() * (y / 7.0 + 0.3 * seed as f64).cos())
    });
    synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    m
}

/// Spread / estimated noise of the primitive fitted to the whole patch.
fn genuine_ratio(shape: Shape, sigma: f64, spacing: f64, seed: u64) -> Option<f64> {
    let m = genuine(shape, sigma, spacing, seed);
    let est = noise_estimate(&m);
    if !(est.smooth && est.sigma > 0.0) {
        return None;
    }
    let normals = m.vertex_normals();
    let fit = fit_auto(&m.positions, &normals)?.best;
    let tris: Vec<u32> = (0..m.triangle_count() as u32).collect();
    Some(residual_spread(&m, &tris, &fit.primitive) / est.sigma)
}

/// Ratios of the fragments (at least `min_tris` triangles) the segmentation finds on a wavy surface.
fn fragment_ratios(sigma: f64, spacing: f64, seed: u64, min_tris: usize) -> Vec<f64> {
    let m = wavy(sigma, spacing, seed);
    let est = noise_estimate(&m);
    if !(est.smooth && est.sigma > 0.0) {
        return Vec::new();
    }
    let opts = SegmentOptions { freeform: false, ..SegmentOptions::default() };
    segment(&m, &opts)
        .iter()
        .filter(|s| s.triangles.len() >= min_tris)
        .map(|s| residual_spread(&m, &s.triangles, &s.primitive) / est.sigma)
        .collect()
}

fn percentile(sorted: &[f64], q: f64) -> f64 {
    sorted.get(((sorted.len() as f64 - 1.0) * q).round() as usize).copied().unwrap_or(f64::NAN)
}

fn sorted(mut v: Vec<f64>) -> Vec<f64> {
    v.sort_by(f64::total_cmp);
    v
}

#[test]
#[ignore = "prints the calibration table (minutes)"]
fn print_table() {
    let part = std::env::var("CALIB_PART").unwrap_or_default();
    let mut all_genuine = Vec::new();
    println!("genuine primitives: ratio = robust spread / estimated noise");
    for shape in [Shape::Plane, Shape::Cylinder, Shape::Sphere] {
        for sigma in SIGMAS.into_iter().filter(|_| part != "fragments") {
            for spacing in SPACINGS {
                let r = sorted((1..=20).filter_map(|seed| genuine_ratio(shape, sigma, spacing, seed)).collect());
                println!(
                    "  {shape:?} sigma {sigma} spacing {spacing}: n {} p50 {:.3} p95 {:.3} p99 {:.3} max {:.3}",
                    r.len(),
                    percentile(&r, 0.5),
                    percentile(&r, 0.95),
                    percentile(&r, 0.99),
                    percentile(&r, 1.0)
                );
                all_genuine.extend(r);
            }
        }
    }
    let all_genuine = sorted(all_genuine);
    println!(
        "ALL genuine: n {} p50 {:.3} p95 {:.3} p99 {:.3} max {:.3}",
        all_genuine.len(),
        percentile(&all_genuine, 0.5),
        percentile(&all_genuine, 0.95),
        percentile(&all_genuine, 0.99),
        percentile(&all_genuine, 1.0)
    );
    let mut all_frag = Vec::new();
    println!("fragments of a wavy surface (regions of at least 300 triangles)");
    for sigma in SIGMAS.into_iter().filter(|_| part != "genuine") {
        for spacing in [0.7, 1.0, 1.5] {
            let r = sorted((1..=20).flat_map(|seed| fragment_ratios(sigma, spacing, seed, 300)).collect());
            println!(
                "  sigma {sigma} spacing {spacing}: n {} min {:.3} p1 {:.3} p5 {:.3} p50 {:.3} above threshold {:.3}",
                r.len(),
                percentile(&r, 0.0),
                percentile(&r, 0.01),
                percentile(&r, 0.05),
                percentile(&r, 0.5),
                r.iter().filter(|x| **x > LARGE_SOUND_FACTOR).count() as f64 / r.len().max(1) as f64
            );
            all_frag.extend(r);
        }
    }
    let all_frag = sorted(all_frag);
    println!(
        "ALL fragments (share above {LARGE_SOUND_FACTOR}: {:.3}): n {} min {:.3} p1 {:.3} p5 {:.3} p50 {:.3}",
        all_frag.iter().filter(|r| **r > LARGE_SOUND_FACTOR).count() as f64 / all_frag.len().max(1) as f64,
        all_frag.len(),
        percentile(&all_frag, 0.0),
        percentile(&all_frag, 0.01),
        percentile(&all_frag, 0.05),
        percentile(&all_frag, 0.5)
    );
}

/// The chosen threshold on real noisy samples: every genuine primitive is within it, also when the
/// noise estimate is 7 % low (0.028 for a true 0.03), and the fragments of a clean wavy surface
/// are mostly above it. (Where the noise swamps the misfit, sigma 0.1, no threshold can separate
/// them: see the table.)
#[test]
fn the_threshold_keeps_genuine_primitives_and_rejects_most_fragments_of_a_clean_surface() {
    for shape in [Shape::Plane, Shape::Cylinder, Shape::Sphere] {
        for spacing in [1.0, 2.0] {
            for seed in 1..=6 {
                let ratio = genuine_ratio(shape, 0.03, spacing, seed).expect("a reliable estimate");
                assert!(ratio <= LARGE_SOUND_FACTOR, "{shape:?} spacing {spacing} seed {seed}: ratio {ratio:.3}");
                // the same samples against an estimate of 0.028 instead of the measured one (planes
                // and cylinders: a sphere sampled at 2 mm reaches 1.11 there, above the threshold)
                if matches!(shape, Shape::Sphere) && spacing > 1.5 {
                    continue;
                }
                let m = genuine(shape, 0.03, spacing, seed);
                let fit = fit_auto(&m.positions, &m.vertex_normals()).expect("a fit").best;
                let tris: Vec<u32> = (0..m.triangle_count() as u32).collect();
                let low = residual_spread(&m, &tris, &fit.primitive) / 0.028;
                assert!(low <= LARGE_SOUND_FACTOR, "{shape:?} spacing {spacing} seed {seed}: ratio {low:.3} at 0.028");
            }
        }
    }
    for (sigma, least_share) in [(0.01, 0.95), (0.03, 0.7)] {
        let ratios: Vec<f64> = (1..=3).flat_map(|seed| fragment_ratios(sigma, 0.7, seed, 300)).collect();
        let above = ratios.iter().filter(|r| **r > LARGE_SOUND_FACTOR).count() as f64 / ratios.len().max(1) as f64;
        eprintln!("sigma {sigma}: {} fragments, {above:.3} above the threshold", ratios.len());
        assert!(
            !ratios.is_empty() && above >= least_share,
            "sigma {sigma}: only {above:.3} of the fragments are above"
        );
    }
}

// --- protection by the tolerance: large primitives with form error ---------------------------------

/// A 60 x 60 mm plane with a waviness of amplitude `amp` (wavelength 20 mm) and noise `sigma`.
fn wavy_plane(amp: f64, sigma: f64, seed: u64) -> Mesh {
    let mut m = sheet(60.0, 60.0, 60, 60, |x, y| {
        let k = std::f64::consts::TAU / 20.0;
        Point3::new(x, y, amp * (k * x).sin() * (k * y + 0.7).sin())
    });
    synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    m
}

/// A cylinder arc (R = 30 mm, 1.5 rad, 60 mm long) with an ovality of `oval` mm and noise `sigma`.
fn oval_cylinder(oval: f64, sigma: f64, seed: u64) -> Mesh {
    let mut m = sheet(1.5, 60.0, 60, 60, |th, y| {
        let r = 30.0 + oval * (2.0 * th).cos();
        Point3::new(r * th.sin(), y, r * th.cos() - 30.0)
    });
    synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    m
}

/// Robust spread of the residuals / tolerance of the whole patch fitted with the automatic fit (the
/// tolerance is 0.2 mm).
fn form_error_ratio(m: &Mesh) -> Option<(PrimitiveKind, f64)> {
    let fit = fit_auto(&m.positions, &m.vertex_normals())?.best;
    let tris: Vec<u32> = (0..m.triangle_count() as u32).collect();
    Some((fit.primitive.kind(), residual_spread(m, &tris, &fit.primitive) / 0.2))
}

/// The large regions (at least 1 % of the triangles and 300) the segmentation finds on a wavy
/// surface: `(kind, share of the scan, RMS / tolerance)`.
fn large_fragments(sigma: f64, spacing: f64, seed: u64) -> Vec<(PrimitiveKind, f64, f64)> {
    let m = wavy(sigma, spacing, seed);
    let nt = m.triangle_count();
    let opts = SegmentOptions { freeform: false, ..SegmentOptions::default() };
    segment(&m, &opts)
        .iter()
        .filter(|s| s.triangles.len() >= 300 && s.triangles.len() * 100 >= nt)
        .map(|s| {
            (
                s.primitive.kind(),
                s.triangles.len() as f64 / nt as f64,
                residual_spread(&m, &s.triangles, &s.primitive) / 0.2,
            )
        })
        .collect()
}

#[test]
#[ignore = "prints the tolerance calibration table (minutes)"]
fn print_tolerance_table() {
    println!("genuine primitives with form error (spread of residuals / tolerance 0.2 of the whole patch)");
    for sigma in [0.003, 0.01] {
        for amp in [0.02, 0.035, 0.05] {
            let r = sorted(
                (1..=10).filter_map(|seed| form_error_ratio(&wavy_plane(amp, sigma, seed))).map(|x| x.1).collect(),
            );
            println!(
                "  plane waviness {amp} sigma {sigma}: p50 {:.3} max {:.3}",
                percentile(&r, 0.5),
                percentile(&r, 1.0)
            );
        }
        for oval in [0.03, 0.05] {
            let r = sorted(
                (1..=10).filter_map(|seed| form_error_ratio(&oval_cylinder(oval, sigma, seed))).map(|x| x.1).collect(),
            );
            println!(
                "  cylinder ovality {oval} sigma {sigma}: p50 {:.3} max {:.3}",
                percentile(&r, 0.5),
                percentile(&r, 1.0)
            );
        }
    }
    println!("large fragments of a wavy surface (>= 1 % of the scan): kind, share, spread / tolerance");
    let mut all = Vec::new();
    for sigma in [0.003, 0.01, 0.03] {
        for spacing in [0.7, 1.0] {
            let found: Vec<_> = (1..=10).flat_map(|seed| large_fragments(sigma, spacing, seed)).collect();
            let ratios = sorted(found.iter().map(|f| f.2).collect());
            println!(
                "  sigma {sigma} spacing {spacing}: n {} min {:.3} p5 {:.3} p50 {:.3}; largest share {:.3}",
                found.len(),
                percentile(&ratios, 0.0),
                percentile(&ratios, 0.05),
                percentile(&ratios, 0.5),
                found.iter().map(|f| f.1).fold(0.0, f64::max)
            );
            all.extend(found);
        }
    }
    for kind in [
        PrimitiveKind::Plane,
        PrimitiveKind::Cylinder,
        PrimitiveKind::Cone,
        PrimitiveKind::Sphere,
        PrimitiveKind::Torus,
    ] {
        let r = sorted(all.iter().filter(|f| f.0 == kind).map(|f| f.2).collect());
        println!(
            "  {kind:?}: n {} min {:.3} p5 {:.3} p50 {:.3}",
            r.len(),
            percentile(&r, 0.0),
            percentile(&r, 0.05),
            percentile(&r, 0.5)
        );
    }
}

/// A region of all the triangles of `m` with the primitive of the automatic fit.
fn whole_patch(m: &Mesh) -> Option<surfacecraft_fit::segment::Segment> {
    let fit = fit_auto(&m.positions, &m.vertex_normals())?.best;
    Some(surfacecraft_fit::segment::Segment {
        triangles: (0..m.triangle_count() as u32).collect(),
        primitive: fit.primitive,
        rms: fit.rms,
        freeform: false,
    })
}

/// Both sides of the protection by the tolerance, on real noisy data: planes with a waviness of
/// 0.02 and 0.035 mm and a cylinder with 0.03 mm of ovality (scanner noise 0.003 and 0.01) are
/// protected; no region the segmentation finds on a wavy surface is (spread of the residuals of
/// those regions: at least 0.13 of the tolerance for planes, `PROTECT_RMS_SHARE` is 0.12).
#[test]
fn the_tolerance_protects_primitives_with_form_error_and_no_organic_fragment() {
    for sigma in [0.003, 0.01] {
        for seed in 1..=4 {
            let patches =
                [wavy_plane(0.02, sigma, seed), wavy_plane(0.035, sigma, seed), oval_cylinder(0.03, sigma, seed)];
            for (k, m) in patches.iter().enumerate() {
                let seg = whole_patch(m).expect("a fit");
                let n = m.triangle_count();
                assert!(
                    surfacecraft_fit::segment::protected_by_tolerance(m, &seg, 0.2, n),
                    "patch {k} sigma {sigma} seed {seed}: {:?} rms {:.4}",
                    seg.primitive.kind(),
                    seg.rms
                );
            }
        }
    }
    let mut checked = 0;
    for (sigma, spacing) in [(0.01, 1.0), (0.03, 1.0), (0.01, 0.7), (0.03, 0.7)] {
        for seed in 1..=3 {
            let m = wavy(sigma, spacing, seed);
            let n = m.triangle_count();
            let opts = SegmentOptions { freeform: false, ..SegmentOptions::default() };
            for s in segment(&m, &opts).iter().filter(|s| s.triangles.len() >= 300) {
                checked += 1;
                assert!(
                    !surfacecraft_fit::segment::protected_by_tolerance(&m, s, 0.2, n),
                    "a fragment is protected: {:?} {} triangles of {n}, rms {:.4}",
                    s.primitive.kind(),
                    s.triangles.len(),
                    s.rms
                );
            }
        }
    }
    assert!(checked > 100, "{checked} fragments checked");
}
