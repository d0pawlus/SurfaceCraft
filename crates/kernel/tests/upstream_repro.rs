//! Minimal reproductions of solidcore problems found while building `kernel::hybrid` (plan P3b Z5).
//!
//! Every test asserts the EXPECTED, correct behaviour and is `#[ignore]`d: it fails on the pinned
//! solidcore (`b9c02da`) and is meant to pass once upstream fixes the problem. Run them with
//! `cargo test -p surfacecraft-kernel --profile ci --test upstream_repro -- --ignored`.
//! The draft issue texts are in `docs/upstream/solidcore-p3.md`.
use solidcore_ops as ops;
use surfacecraft_kernel::record::brep::Surface;
use surfacecraft_kernel::{solid, step};

/// Problem a: a cone that narrows along +z (radius 5 at the base, 1 at the top, height 10)
/// is exported to STEP and read back as ONE body.
///
/// Input: `ops::make_cone([0;3], 5, 1, 10)` as a record, `step::export_solids`, `step::import`.
/// Expected: one body back (the same solid as before).
/// Actual: the STEP import splits the solid into several bodies, one per face (the negative
/// half-angle conical face does not survive `step_brep::import`).
#[test]
#[ignore = "solidcore b9c02da: narrowing cone (negative half angle) splits into several bodies after STEP round trip"]
fn cone_narrowing_along_z_survives_step_round_trip() {
    let cone = ops::make_cone([0.0; 3], 5.0, 1.0, 10.0).expect("make_cone");
    let (rec, _) = cone.to_record().expect("to_record");
    let text = step::export_solids(&[("cone", &rec)]).expect("export_solids");
    let bodies = step::import(text.as_bytes()).expect("import");
    assert_eq!(bodies.len(), 1, "a single solid must come back as one body, got {} bodies", bodies.len());
}

/// Problem b: filleting one straight edge of a box gives a cylindrical face.
///
/// Input: box 20 × 10 × 5, `solid::fillet` on edge 0 with radius 2.
/// Expected: exactly one `Surface::Cylinder` face (the round).
/// Actual: the round comes back as a `Surface::Cone` with half angle 0 (SMLib's own form of a
/// cylinder from `ops::blend`), so every consumer must special-case it.
#[test]
#[ignore = "solidcore b9c02da: ops::blend returns a fillet cylinder as Cone with half angle 0"]
fn fillet_of_box_edge_is_a_cylinder_face() {
    let b = solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).expect("make_box");
    let f = solid::fillet(&b, &[0], 2.0).expect("fillet");
    let kinds: Vec<&str> = f
        .faces
        .iter()
        .map(|face| match &face.surface {
            Surface::Plane { .. } | Surface::PlaneThrough { .. } => "plane",
            Surface::Cylinder { .. } => "cylinder",
            Surface::Cone { .. } => "cone",
            _ => "other",
        })
        .collect();
    let cylinders = f.faces.iter().filter(|face| matches!(face.surface, Surface::Cylinder { .. })).count();
    assert_eq!(cylinders, 1, "one cylindrical round expected, face kinds were {kinds:?}");
}

/// Problem c: two cylinders whose axes are 0.01° apart (same radius, both based at the origin)
/// intersect in a solid.
///
/// Input: cylinder r 5 h 20 at the origin, and the same cylinder turned 0.01° about x.
/// Expected: `ops::boolean(.., Intersect)` returns a solid of volume between 1400 and π·25·20.
/// Actual: the boolean fails with kernel status 1001 (`SM_ERR`), so no lens solid is built.
/// (The same pair turned 0.5° intersects fine, volume ≈ 1551.9 mm³, so the failure needs a
/// near-coincident axis, as the fit noise of scanned revolved surfaces gives.)
#[test]
#[ignore = "solidcore b9c02da: boolean of near-coaxial revolved surfaces (axes 0.01 deg apart) fails with status 1001"]
fn near_coaxial_cylinders_intersect() {
    let a = solid::make_cylinder([0.0; 3], 5.0, 20.0).expect("make_cylinder");
    let c = 0.01_f64.to_radians().cos();
    let s = 0.01_f64.to_radians().sin();
    let turn = [[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]];
    let tilted =
        solid::transform(&solid::make_cylinder([0.0; 3], 5.0, 20.0).expect("make_cylinder"), &turn).expect("transform");
    let (ba, _) = ops::Body::from_record(&a, false).expect("from_record a");
    let (bb, _) = ops::Body::from_record(&tilted, false).expect("from_record b");
    let out = ops::boolean::boolean(ba, bb, ops::BooleanOp::Intersect).expect("intersect must succeed");
    let body = out.expect("the intersection is not empty");
    let v = body.volume(1e-6).expect("volume");
    assert!((1400.0..=std::f64::consts::PI * 25.0 * 20.0).contains(&v), "lens volume {v}");
}
