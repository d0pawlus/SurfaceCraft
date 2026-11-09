//! P8 (D1): `mesh.decimate` with a verified tolerance. The command reports what it measured; each
//! test measures the result again on its own (original vertices to the reduced surface, reduced
//! vertices to the original) so the report is not taken on trust.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3, synth};

fn mesh_of(s: &Session, name: &str) -> Mesh {
    let id = s.doc.find_by_name(name).unwrap().0;
    (*s.doc.mesh(id).unwrap().mesh).clone()
}

/// (share within `tol`, max) of the distances from the vertices of `a` to the surface of `b`.
fn one_way(a: &Mesh, b: &Mesh, tol: f64) -> (f64, f64) {
    let bvh = Bvh::build(b);
    let d: Vec<f64> = a.positions.iter().map(|p| bvh.closest(p, f64::INFINITY).unwrap().distance).collect();
    let within = d.iter().filter(|&&x| x <= tol).count() as f64 / d.len() as f64;
    (within, d.iter().copied().fold(0.0, f64::max))
}

fn direction(r: &Value, key: &str) -> (f64, f64) {
    (r["tolerance_report"][key]["share_within"].as_f64().unwrap(), r["tolerance_report"][key]["max"].as_f64().unwrap())
}

/// The reported numbers, both ways, against the gate; returns the triangle counts.
fn check_report(r: &Value, tol: f64) -> (u64, u64) {
    let rep = &r["tolerance_report"];
    assert_eq!(rep["met"], true, "{r}");
    for key in ["original_to_reduced", "reduced_to_original"] {
        let (share, max) = direction(r, key);
        assert!(share >= 0.999, "{key}: share {share} in {r}");
        assert!(max <= 2.0 * tol, "{key}: max {max} in {r}");
        assert!(rep[key]["p99"].as_f64().unwrap() <= max);
        assert!(rep[key]["samples"].as_u64().unwrap() > 0);
    }
    let (before, after) = (rep["triangles_before"].as_u64().unwrap(), rep["triangles_after"].as_u64().unwrap());
    assert_eq!(r["triangles"].as_u64().unwrap(), after);
    assert_eq!(r["triangles_before"].as_u64().unwrap(), before);
    assert!(rep["iterations"].as_u64().unwrap() >= 1);
    (before, after)
}

fn check_independently(orig: &Mesh, red: &Mesh, tol: f64) {
    let (share, max) = one_way(orig, red, tol);
    assert!(share >= 0.999 && max <= 2.0 * tol, "original to reduced: share {share}, max {max}");
    let (share, max) = one_way(red, orig, tol);
    assert!(share >= 0.999 && max <= 2.0 * tol, "reduced to original: share {share}, max {max}");
}

/// A sphere of radius 20 with more than 200k triangles reduces at least five-fold within 0.05 mm.
#[test]
fn p8_sphere_tolerance() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "size": [20.0], "resolution": 460, "name": "ball"})).unwrap();
    let orig = mesh_of(&s, "ball");
    assert!(orig.triangle_count() >= 200_000, "{}", orig.triangle_count());
    let tol = 0.05;
    let r = s.execute("mesh.decimate", json!({"object": "ball", "tolerance": tol})).unwrap();
    let (before, after) = check_report(&r, tol);
    assert!(after * 5 <= before, "{before} -> {after}");
    assert_eq!(r["removed_triangles"].as_u64().unwrap(), before - after);
    check_independently(&orig, &mesh_of(&s, "ball"), tol);
    // undo restores the original
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(mesh_of(&s, "ball").triangle_count(), orig.triangle_count());
}

/// A box with all twelve edges rounded (r = 2): the fillets are not cut by the reduction.
#[test]
fn p8_filleted_box_tolerance() {
    let (size, r, tol) = (Vec3::new(30.0, 24.0, 18.0), 2.0, 0.05);
    let cad = synth::rounded_cuboid(Point3::origin(), size, r, 80);
    let mut s = Session::new();
    s.doc.add_mesh("block", cad.clone());
    let report = s.execute("mesh.decimate", json!({"object": "block", "tolerance": tol})).unwrap();
    let (before, after) = check_report(&report, tol);
    assert!(after * 5 <= before, "{before} -> {after}");
    let red = mesh_of(&s, "block");
    check_independently(&cad, &red, tol);
    // every reduced vertex lies on the analytic rounded box: a cut fillet would put vertices on its
    // chord, up to r (1 - cos 45 deg) = 0.59 mm inside the surface
    let (centre, inner) = (Point3::origin() + size / 2.0, size / 2.0 - Vec3::new(r, r, r));
    for p in &red.positions {
        let q = p - centre;
        let c = Vec3::new(q.x.clamp(-inner.x, inner.x), q.y.clamp(-inner.y, inner.y), q.z.clamp(-inner.z, inner.z));
        let off = ((q - c).norm() - r).abs();
        assert!(off <= 1.5 * tol, "vertex {p} is {off} mm off the rounded surface");
    }
    // and the fillets still have vertices (twelve edges, several along each arc)
    let on_fillet = red
        .positions
        .iter()
        .filter(|p| {
            let q = *p - centre;
            [q.x.abs() > inner.x + 1e-6, q.y.abs() > inner.y + 1e-6, q.z.abs() > inner.z + 1e-6]
                .iter()
                .filter(|&&b| b)
                .count()
                >= 2
        })
        .count();
    assert!(on_fillet >= 36, "{on_fillet} vertices on the fillets");
}

/// A noisy plane reduces at least fifty-fold.
#[test]
fn p8_noisy_plane_tolerance() {
    let mut s = Session::new();
    s.execute(
        "mesh.synth",
        json!({"shape": "plane", "size": [100.0, 100.0], "resolution": 200, "noise": 0.01, "name": "sheet"}),
    )
    .unwrap();
    let orig = mesh_of(&s, "sheet");
    let tol = 0.05;
    let r = s.execute("mesh.decimate", json!({"object": "sheet", "tolerance": tol})).unwrap();
    let (before, after) = check_report(&r, tol);
    assert!(after * 50 <= before, "{before} -> {after}");
    check_independently(&orig, &mesh_of(&s, "sheet"), tol);
}

/// Without `tolerance` nothing changes; `tolerance` with `ratio` takes the ratio as a floor; the
/// parameters are validated.
#[test]
fn p8_parameters() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "size": [20.0], "resolution": 64, "name": "ball"})).unwrap();
    let plain = s.execute("mesh.decimate", json!({"object": "ball", "ratio": 0.25})).unwrap();
    assert!(plain.get("tolerance_report").is_none(), "{plain}");
    s.execute("edit.undo", Value::Null).unwrap();
    // a floor of 80 % of the triangles holds although the tolerance would allow far less
    let floor = s.execute("mesh.decimate", json!({"object": "ball", "tolerance": 0.5, "ratio": 0.8})).unwrap();
    let before = floor["triangles_before"].as_u64().unwrap();
    assert!(floor["triangles"].as_u64().unwrap() >= before * 8 / 10 - 2, "{floor}");
    assert_eq!(floor["tolerance_report"]["met"], true);
    s.execute("edit.undo", Value::Null).unwrap();
    // the same call twice gives the same mesh
    let a = s.execute("mesh.decimate", json!({"object": "ball", "tolerance": 0.1, "share": 0.99})).unwrap();
    let first = mesh_of(&s, "ball");
    s.execute("edit.undo", Value::Null).unwrap();
    let b = s.execute("mesh.decimate", json!({"object": "ball", "tolerance": 0.1, "share": 0.99})).unwrap();
    assert_eq!(a, b);
    assert_eq!(first, mesh_of(&s, "ball"));
    s.execute("edit.undo", Value::Null).unwrap();
    for bad in [
        json!({"object": "ball", "tolerance": 0.0}),
        json!({"object": "ball", "tolerance": -1.0}),
        json!({"object": "ball", "tolerance": 0.1, "share": 0.0}),
        json!({"object": "ball", "tolerance": 0.1, "share": 1.5}),
        json!({"object": "ball", "share": 0.9}),
    ] {
        assert!(s.execute("mesh.decimate", bad.clone()).is_err(), "{bad}");
    }
    assert_eq!(mesh_of(&s, "ball").triangle_count() as u64, before);
}
