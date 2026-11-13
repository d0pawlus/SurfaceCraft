//! The protection of large primitives by the tolerance and the bow (see `protect`).
use super::Segment;
use super::protect::protected_by_tolerance;
use crate::fit_auto;
use surfacecraft_geom::{Mesh, Point3, Rng, synth};

const TOLERANCE: f64 = 0.2;

/// A grid over `[0, w] x [0, h]` with vertices `f(x, y)`.
fn sheet(w: f64, h: f64, n: usize, f: impl Fn(f64, f64) -> Point3) -> Mesh {
    let cols = n + 1;
    let mut positions = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            positions.push(f(w * i as f64 / n as f64, h * j as f64 / n as f64));
        }
    }
    let mut triangles = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let (a, b) = ((j * cols + i) as u32, (j * cols + i + 1) as u32);
            let (c, d) = (((j + 1) * cols + i + 1) as u32, ((j + 1) * cols + i) as u32);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { positions, triangles }
}

/// All the triangles of `m` as one region with the automatic fit's primitive.
fn whole(m: &Mesh) -> Segment {
    let fit = fit_auto(&m.positions, &m.vertex_normals()).expect("a fit").best;
    Segment {
        triangles: (0..m.triangle_count() as u32).collect(),
        primitive: fit.primitive,
        rms: fit.rms,
        freeform: false,
    }
}

/// A `side` mm patch with a smooth waviness of amplitude `amp` (wavelength `wavelength`), noise
/// 0.003 mm: a plane with form error, or, when the patch is small, a doubly curved bump.
fn wavy(side: f64, wavelength: f64, amp: f64, seed: u64) -> Mesh {
    let k = std::f64::consts::TAU / wavelength;
    let mut m =
        sheet(side, side, (side * 1.5) as usize, |x, y| Point3::new(x, y, amp * (k * x).sin() * (k * y + 0.7).sin()));
    synth::add_noise(&mut m, 0.003, &mut Rng::new(seed));
    m
}

/// A plane with a form error of 0.4 x the tolerance (RMS 0.08 mm, waviness of 0.16 mm over 120 mm)
/// is kept, and a doubly curved patch of the same RMS to its best plane (a 20 mm bump of the same
/// height: 0.4 x the tolerance again) is not: the first bows 7e-4 of its size, the second 4e-3.
#[test]
fn a_plane_with_form_error_is_kept_and_a_doubly_curved_patch_of_the_same_residual_is_not() {
    let plane = wavy(120.0, 120.0, 0.16, 1);
    let bump = wavy(20.0, 20.0, 0.16, 2);
    let (p, b) = (whole(&plane), whole(&bump));
    eprintln!("plane rms {:.4}, bump rms {:.4}", p.rms, b.rms);
    assert!((p.rms - b.rms).abs() < 0.03, "the same residual: {} and {}", p.rms, b.rms);
    assert!(p.rms > 0.5 * 0.4 * TOLERANCE, "form error of the order of 0.4 x the tolerance: {}", p.rms);
    assert!(protected_by_tolerance(&plane, &p, TOLERANCE, 0.003), "the plane with form error is lost");
    assert!(!protected_by_tolerance(&bump, &b, TOLERANCE, 0.003), "the doubly curved patch is protected");
}

/// A region below the size gate (500 triangles) is unaffected, whatever its bow.
#[test]
fn a_small_region_is_not_protected() {
    let small = wavy(8.0, 120.0, 0.002, 3);
    let seg = whole(&small);
    assert!(seg.triangles.len() < 500, "{} triangles", seg.triangles.len());
    assert!(!protected_by_tolerance(&small, &seg, TOLERANCE, 0.003));
}

/// The noise is taken out in quadrature: a clean plane read through noise above 0.12 x the
/// tolerance (0.05 mm here) is protected, as it is at low noise.
#[test]
fn noise_above_a_share_of_the_tolerance_does_not_unprotect_a_plane() {
    let mut m = sheet(120.0, 120.0, 100, |x, y| Point3::new(x, y, 0.0));
    synth::add_noise(&mut m, 0.05, &mut Rng::new(4));
    let seg = whole(&m);
    assert!(protected_by_tolerance(&m, &seg, TOLERANCE, 0.05));
}

/// A plane is judged by an extent that never exceeds a fifth of the scan's diagonal: a face of a
/// small part (30 mm) is protected although it is shorter than 50 tolerances at a coarse one.
#[test]
fn the_plane_extent_limit_scales_with_the_scan() {
    let mut m = sheet(30.0, 30.0, 60, |x, y| Point3::new(x, y, 0.0));
    synth::add_noise(&mut m, 0.003, &mut Rng::new(5));
    let seg = whole(&m);
    // 50 x 1.0 mm = 50 mm is more than the face; 0.2 x its diagonal (8.5 mm) is not
    assert!(protected_by_tolerance(&m, &seg, 1.0, 0.003));
}

// --- regions that wrap a cylinder or a sphere, and the tiers of the slope limit ---------------------

use crate::primitive::Primitive;

/// A region of all the triangles of `m` with the given primitive and its measured RMS.
fn with_primitive(m: &Mesh, primitive: Primitive) -> Segment {
    let rms =
        (m.positions.iter().map(|p| primitive.distance(p).powi(2)).sum::<f64>() / m.positions.len() as f64).sqrt();
    Segment { triangles: (0..m.triangle_count() as u32).collect(), primitive, rms, freeform: false }
}

/// 200 degrees of a cylinder (R = 30 mm, 20 mm long, cells of about 1.2 mm) with a smooth radial
/// deviation of amplitude `amp` (two waves around the arc), noise 0.003.
fn wrapped_cylinder(amp: f64, seed: u64) -> Mesh {
    let span = 200f64.to_radians();
    let mut m = sheet(1.0, 1.0, 90, |a, b| {
        let th = (a - 0.5) * span;
        let r = 30.0 + amp * (4.0 * std::f64::consts::PI * a).sin() * (std::f64::consts::PI * b).sin();
        Point3::new(r * th.cos(), r * th.sin(), 20.0 * b)
    });
    synth::add_noise(&mut m, 0.003, &mut Rng::new(seed));
    m
}

fn cylinder() -> Primitive {
    Primitive::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: 30.0 }
}

/// A cap of a sphere (R = 30 mm) out to 110 degrees from its pole (more than a hemisphere), with a
/// smooth radial deviation of amplitude `amp`, noise 0.003.
fn big_cap(amp: f64, seed: u64) -> Mesh {
    let reach = 110f64.to_radians();
    let mut m = sheet(1.0, 1.0, 100, |a, b| {
        let (psi, phi) = (0.03 + a * (reach - 0.03), b * std::f64::consts::TAU * 0.98);
        let r = 30.0 + amp * (3.0 * std::f64::consts::PI * a).sin() * (2.0 * phi).cos();
        Point3::new(r * psi.sin() * phi.cos(), r * psi.sin() * phi.sin(), r * psi.cos())
    });
    synth::add_noise(&mut m, 0.003, &mut Rng::new(seed));
    m
}

fn sphere() -> Primitive {
    Primitive::Sphere { center: [0.0; 3], radius: 30.0 }
}

/// The tolerance of these tests is 1.0 mm: the bow limit of the largest tier is 2e-3 of the extent
/// (100 mm of arc: 0.2 mm), well inside 0.5 x the tolerance; the form-error limit is 0.08 mm.
const COARSE: f64 = 1.0;

/// A region that wraps 200 degrees of a cylinder: a real bow (0.3 mm RMS, above the slope limit) is
/// not protected, form error under the limit (0.085 mm, over the form-error limit so the bow decides)
/// is. The height field is fitted in the cylinder's own chart: over a best-fit plane the arc folds,
/// the quadric fits worse than the primitive and the bow used to read zero.
#[test]
fn a_cylinder_wrapped_200_degrees_is_judged_by_its_bow_in_its_own_chart() {
    let (bowed, tidy) = (wrapped_cylinder(0.42, 1), wrapped_cylinder(0.12, 2));
    assert!(
        !protected_by_tolerance(&bowed, &with_primitive(&bowed, cylinder()), COARSE, 0.003),
        "a bowed wrap is protected"
    );
    assert!(protected_by_tolerance(&tidy, &with_primitive(&tidy, cylinder()), COARSE, 0.003), "a tidy wrap is lost");
}

/// The same for a cap of a sphere beyond a hemisphere.
#[test]
fn a_sphere_cap_beyond_a_hemisphere_is_judged_by_its_bow_in_its_own_chart() {
    let (bowed, tidy) = (big_cap(0.42, 3), big_cap(0.12, 4));
    assert!(
        !protected_by_tolerance(&bowed, &with_primitive(&bowed, sphere()), COARSE, 0.003),
        "a bowed cap is protected"
    );
    assert!(protected_by_tolerance(&tidy, &with_primitive(&tidy, sphere()), COARSE, 0.003), "a tidy cap is lost");
}

/// A region of 2.7 % of a larger scan is judged by the 5e-4 tier: a plane patch that bows 4e-4 of
/// its size is protected, one that bows 6e-4 is not (the plate that makes the scan large is flat).
#[test]
fn a_region_under_five_percent_of_the_scan_is_judged_by_the_tight_slope() {
    let case = |amp: f64| {
        let k = std::f64::consts::TAU / 50.0;
        let mut patch = sheet(50.0, 50.0, 50, |x, y| Point3::new(x, y, amp * (k * x).sin() * (k * y + 0.7).sin()));
        let triangles = patch.triangle_count() as u32;
        patch.append(&sheet(300.0, 300.0, 100, |x, y| Point3::new(x + 100.0, y, 0.0)));
        synth::add_noise(&mut patch, 0.003, &mut Rng::new(6));
        let plane = Primitive::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] };
        let rms = (patch.positions.iter().take(51 * 51).map(|p| plane.distance(p).powi(2)).sum::<f64>()
            / (51.0 * 51.0))
            .sqrt();
        let seg = Segment { triangles: (0..triangles).collect(), primitive: plane, rms, freeform: false };
        protected_by_tolerance(&patch, &seg, TOLERANCE, 0.003)
    };
    assert!(case(0.05), "slope 5e-4 x 0.8: the patch should be protected");
    assert!(!case(0.14), "slope well over 5e-4: the patch should not be protected");
}
