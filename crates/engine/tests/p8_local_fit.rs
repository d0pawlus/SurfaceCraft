//! P8 local fine fit through `align.icp`: only a marked part of the scan drives the fit, and the
//! resulting motion moves the whole scan. Gate: a bracket scan, moved by 0.5° and 0.3 mm against
//! its reference, has its +X half bulged by 1 mm. Fitting on the undeformed half recovers the
//! motion there to 0.01 mm, while a whole-scan fit is measurably worse on that same half.
use serde_json::json;
use std::collections::BTreeSet;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Transform, Vec3, synth};

/// The motion between the reference and the scan: 0.5° about an oblique axis, 0.3 mm of shift.
fn truth() -> Transform {
    Transform::rotation_axis_angle(&Vec3::new(0.2, 0.9, -0.4), 0.5_f64.to_radians())
        .then_after(&Transform::translation(Vec3::new(0.2, -0.2, 0.1)))
}

/// The bulge on the +X half (x > 30 mm): a 1 mm hump pushed along +Z, zero on the other half.
fn bulge(x: f64) -> f64 {
    if x > 30.0 && x < 60.0 { (std::f64::consts::PI * (x - 30.0) / 30.0).sin() } else { 0.0 }
}

/// The reference bracket and its scan (the same vertices and triangles, bulged and moved), with
/// the triangles of the undeformed half: those whose centroid lies at x < 30 mm.
fn setup() -> (Session, Mesh, Vec<u32>) {
    let reference = synth::bracket();
    let mut bulged = reference.clone();
    for p in &mut bulged.positions {
        let dz = bulge(p.x);
        *p += Vec3::new(0.0, 0.0, dz);
    }
    let scan = bulged.transformed(&truth().inverse());
    let marked: Vec<u32> = (0..reference.triangle_count())
        .filter(|&t| reference.face_centroid(t).is_some_and(|c| c.x < 30.0))
        .map(|t| t as u32)
        .collect();
    let mut s = Session::new();
    s.doc.add_mesh("reference", reference.clone());
    s.doc.add_mesh("scan", scan);
    (s, reference, marked)
}

/// A mesh of the session, by name.
fn mesh_of(s: &Session, name: &str) -> Mesh {
    let (id, _) = s.doc.find_by_name(name).unwrap();
    s.doc.mesh(id).unwrap().mesh.as_ref().clone()
}

/// The vertices of the given triangles.
fn vertices_of(mesh: &Mesh, tris: &[u32]) -> BTreeSet<u32> {
    tris.iter().filter_map(|&t| mesh.triangles.get(t as usize)).flatten().copied().collect()
}

/// The largest distance (mm) between the vertices `verts` in `scan` and in `reference`.
fn worst(scan: &Mesh, reference: &Mesh, verts: &BTreeSet<u32>) -> f64 {
    verts
        .iter()
        .filter_map(|&v| {
            let (a, b) = (scan.positions.get(v as usize)?, reference.positions.get(v as usize)?);
            Some((a - b).norm())
        })
        .fold(0.0, f64::max)
}

/// Gate P8: the fit on the undeformed half recovers the motion there to 0.01 mm.
#[test]
fn gate_p8_local_fit_recovers_motion_on_the_marked_half() {
    let (mut s, reference, marked) = setup();
    let out = s.execute("align.icp", json!({"object": "scan", "target": "reference", "triangles": marked})).unwrap();
    assert_eq!(out["converged"], true, "{out}");
    let verts = vertices_of(&reference, &marked);
    let err = worst(&mesh_of(&s, "scan"), &reference, &verts);
    assert!(err <= 0.01, "the marked half is {err} mm from the reference after the local fit: {out}");
    assert!(out["rms_after_marked"].as_f64().unwrap() <= 0.01, "{out}");
    assert!(out["rms_before_marked"].as_f64().unwrap() > out["rms_after_marked"].as_f64().unwrap(), "{out}");
    assert_eq!(out["points_used"].as_u64(), Some(verts.len() as u64), "{out}");
}

/// Gate P8: a whole-scan fit is measurably worse on the same undeformed half.
#[test]
fn gate_p8_whole_scan_fit_is_worse_on_the_marked_half() {
    let (mut s, reference, marked) = setup();
    s.execute("align.icp", json!({"object": "scan", "target": "reference"})).unwrap();
    let err = worst(&mesh_of(&s, "scan"), &reference, &vertices_of(&reference, &marked));
    assert!(err > 0.1, "a whole-scan fit is only {err} mm off on the undeformed half");
}

/// The same local fit, with the marked half given as a region of the scan.
#[test]
fn local_fit_accepts_a_region() {
    let (mut s, reference, marked) = setup();
    let region = s.execute("region.create", json!({"mesh": "scan", "triangles": marked, "name": "left"})).unwrap();
    let out = s
        .execute("align.icp", json!({"object": "scan", "target": "reference", "region": region["id"].clone()}))
        .unwrap();
    assert_eq!(out["converged"], true, "{out}");
    let err = worst(&mesh_of(&s, "scan"), &reference, &vertices_of(&reference, &marked));
    assert!(err <= 0.01, "the region is {err} mm from the reference after the local fit");
}

/// A marked part too small to fit is refused, and the error points at align.n_point.
#[test]
fn local_fit_refuses_a_part_too_small_to_fit() {
    let (mut s, _, _) = setup();
    let before = mesh_of(&s, "scan");
    let err = s.execute("align.icp", json!({"object": "scan", "target": "reference", "triangles": [0]})).unwrap_err();
    assert!(err.to_string().contains("align.n_point"), "{err}");
    assert_eq!(mesh_of(&s, "scan"), before, "a refused fit must not move the scan");
}

/// Hostile selections are parameter errors, never panics or silent whole-object fits.
#[test]
fn local_fit_rejects_hostile_selections() {
    let (mut s, _, _) = setup();
    let base = json!({"object": "scan", "target": "reference"});
    let extras = [
        json!({"triangles": [u32::MAX]}),
        json!({"triangles": []}),
        json!({"region": 999}),
        json!({"triangles": [0], "region": 999}),
    ];
    for extra in extras {
        let mut params = base.clone();
        if let (Some(p), Some(e)) = (params.as_object_mut(), extra.as_object()) {
            p.extend(e.clone());
        }
        assert!(s.execute("align.icp", params).is_err(), "{extra}");
    }
}
