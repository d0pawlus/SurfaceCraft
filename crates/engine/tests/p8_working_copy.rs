//! P8 (D2): working copies. The original stays as it was; edits act on the copy while it exists;
//! `mesh.working_distance` measures the copy against the original in both directions. Each test
//! gives the expected numbers from known geometry, not from the command under test.
use serde_json::{Value, json};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, WorkingCopy};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3, synth};

/// Tolerance of the exact comparisons (mm).
const EPS: f64 = 1e-9;
/// Offset of the lifted vertices of the known-offset test (mm).
const LIFT: f64 = 0.3;

fn id_of(s: &Session, name: &str) -> ObjectId {
    s.doc.find_by_name(name).unwrap().0
}

fn original(s: &Session, name: &str) -> Mesh {
    (*s.doc.mesh(id_of(s, name)).unwrap().mesh).clone()
}

fn working(s: &Session, name: &str) -> Option<Mesh> {
    s.doc.mesh(id_of(s, name)).unwrap().working.as_ref().map(|w| (*w.mesh).clone())
}

fn near(v: &Value, expected: f64, eps: f64) {
    let x = v.as_f64().unwrap_or(f64::NAN);
    assert!((x - expected).abs() <= eps, "{x} is not {expected} (±{eps}) in {v}");
}

fn distance(s: &mut Session, name: &str) -> Value {
    s.execute("mesh.working_distance", json!({"object": name})).unwrap()
}

/// A flat sheet `name`, 50 × 50 mm, sixteen cells a side, with its working copy started.
fn sheet_with_copy(s: &mut Session, name: &str) {
    s.execute("mesh.synth", json!({"shape": "plane", "size": [50.0, 50.0], "resolution": 16, "name": name})).unwrap();
    s.execute("mesh.working_begin", json!({"object": name})).unwrap();
}

/// Ground truth: every copy vertex with x > 0 lifted by `LIFT` along the sheet normal. Each copy
/// vertex is then exactly 0 or `LIFT` from the original, and the reverse direction stays within it.
#[test]
fn p8_working_distance_known_offset() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let orig = original(&s, "sheet");
    let lifted: Vec<Point3> =
        orig.positions.iter().map(|p| if p.x > 0.0 { Point3::new(p.x, p.y, p.z + LIFT) } else { *p }).collect();
    let lifted_count = lifted.iter().zip(&orig.positions).filter(|(a, b)| a.z > b.z).count();
    let copy = Mesh::new(lifted, orig.triangles.clone()).unwrap();
    s.doc.mesh_mut(id_of(&s, "sheet")).unwrap().working = Some(WorkingCopy { mesh: Arc::new(copy) });
    let csv = std::env::temp_dir().join(format!("sc-d2-offset-{}.csv", std::process::id()));
    let csv_path = csv.to_string_lossy().to_string();
    let r = s
        .execute("mesh.working_distance", json!({"object": "sheet", "tolerance": LIFT / 2.0, "path": csv_path}))
        .unwrap();
    let n = orig.vertex_count() as f64;
    let fwd = &r["working_to_original"];
    assert_eq!(fwd["count"], orig.vertex_count(), "{r}");
    near(&fwd["max"], LIFT, EPS);
    near(&fwd["min"], 0.0, EPS);
    near(&fwd["rms"], LIFT * (lifted_count as f64 / n).sqrt(), EPS);
    near(&fwd["bands"][0]["share"], (n - lifted_count as f64) / n, EPS);
    assert!(r["original_to_working"]["max"].as_f64().unwrap() <= LIFT + EPS, "{r}");
    // the per-vertex map: signed, positive where the copy lies above the original
    let text = std::fs::read_to_string(&csv).unwrap();
    let rows: Vec<Vec<f64>> =
        text.lines().skip(1).map(|l| l.split(',').map(|c| c.parse().unwrap()).collect()).collect();
    assert_eq!(rows.len(), orig.vertex_count());
    for row in &rows {
        let expect = if row[1] > 0.0 { LIFT } else { 0.0 };
        assert!((row[4] - expect).abs() <= EPS && (row[5] - expect).abs() <= EPS, "{row:?}");
    }
    let _ = std::fs::remove_file(csv);
}

/// A flat sheet decimated on its copy: every vertex of either surface still lies on the other, so
/// both directions are exactly zero, the copy has fewer triangles and the original is untouched.
#[test]
fn p8_decimated_flat_sheet_stays_on_the_original() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let before = original(&s, "sheet");
    let r = s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    assert_eq!(r["working"], true, "{r}");
    assert!(r["triangles"].as_u64().unwrap() < r["triangles_before"].as_u64().unwrap(), "{r}");
    assert_eq!(original(&s, "sheet"), before);
    let d = distance(&mut s, "sheet");
    near(&d["working_to_original"]["max"], 0.0, EPS);
    near(&d["original_to_working"]["max"], 0.0, EPS);
}

/// Two plates, the small one removed on the copy: the copy lies on the original (forward zero), and
/// the removed plate is far from what is left (reverse), by exactly the brute-force distance.
#[test]
fn p8_removed_piece_shows_in_the_reverse_direction() {
    let big = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 50.0, 50.0, 16, 16);
    let small = synth::plane(Point3::new(100.0, 0.0, 0.0), Vec3::x(), Vec3::y(), 10.0, 10.0, 4, 4);
    let mut both = big.clone();
    both.append(&small);
    let mut s = Session::new();
    let id = s.doc.add_mesh("two", both.clone());
    s.execute("mesh.working_begin", json!({"object": "two"})).unwrap();
    let removed = s.execute("mesh.remove_small", json!({"object": "two", "min_triangles": 100})).unwrap();
    assert_eq!(removed["removed_triangles"], small.triangle_count(), "{removed}");
    assert_eq!(s.doc.mesh(id).unwrap().mesh.triangle_count(), both.triangle_count());
    let d = distance(&mut s, "two");
    near(&d["working_to_original"]["max"], 0.0, EPS);
    let bvh = Bvh::build(&big);
    let expected =
        small.positions.iter().map(|p| bvh.closest(p, f64::INFINITY).map_or(0.0, |h| h.distance)).fold(0.0, f64::max);
    // the far edge of the small plate (x = 110) is 110 - 50 = 60 mm from the big plate's edge (x = 50)
    assert!((expected - 60.0).abs() < EPS, "brute force gives {expected}");
    near(&d["original_to_working"]["max"], expected, 1e-6);
}

/// A noisy sheet decimated on its copy with a tolerance: the vertex map is bounded by the bound the
/// reduction measured against its input (the original, since the copy started identical to it).
#[test]
fn p8_tolerance_decimate_on_the_copy_within_the_measured_bound() {
    let mut s = Session::new();
    s.execute(
        "mesh.synth",
        json!({"shape": "plane", "size": [100.0, 100.0], "resolution": 80, "noise": 0.01, "name": "sheet"}),
    )
    .unwrap();
    let id = id_of(&s, "sheet");
    s.doc.add_region("patch", id, vec![0, 1, 2]);
    s.execute("mesh.working_begin", json!({"object": "sheet"})).unwrap();
    let before = original(&s, "sheet");
    let r = s.execute("mesh.decimate", json!({"object": "sheet", "tolerance": 0.05})).unwrap();
    assert_eq!(r["tolerance_report"]["met"], true, "{r}");
    assert_eq!(original(&s, "sheet"), before);
    assert_eq!(s.doc.regions.len(), 1, "regions of the original survive edits to the copy");
    let d = distance(&mut s, "sheet");
    let bound = r["tolerance_report"]["reduced_to_original"]["max"].as_f64().unwrap();
    let max = d["working_to_original"]["max"].as_f64().unwrap();
    assert!(max > 0.0 && max <= bound + EPS, "map max {max} against the bound {bound}");
}

/// Undo and redo walk back and forth through start, edit and the copy's removal.
#[test]
fn p8_undo_redo_with_a_working_copy() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let full = working(&s, "sheet").unwrap().triangle_count();
    s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    let reduced = working(&s, "sheet").unwrap().triangle_count();
    assert!(reduced < full);
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(working(&s, "sheet").unwrap().triangle_count(), full);
    s.execute("edit.undo", Value::Null).unwrap();
    assert!(working(&s, "sheet").is_none());
    s.execute("edit.redo", Value::Null).unwrap();
    assert_eq!(working(&s, "sheet").unwrap().triangle_count(), full);
    s.execute("edit.redo", Value::Null).unwrap();
    assert_eq!(working(&s, "sheet").unwrap().triangle_count(), reduced);
}

/// Promote replaces the original with the copy, drops the regions of the old topology, and undo
/// brings back both the original and the copy.
#[test]
fn p8_promote_replaces_the_original_and_undo_restores_it() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let id = id_of(&s, "sheet");
    s.doc.add_region("patch", id, vec![0, 1, 2]);
    let original_triangles = original(&s, "sheet").triangle_count();
    s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    let copy_triangles = working(&s, "sheet").unwrap().triangle_count();
    let p = s.execute("mesh.working_promote", json!({"object": "sheet"})).unwrap();
    assert_eq!(p["triangles"], copy_triangles, "{p}");
    assert_eq!(p["triangles_before"], original_triangles, "{p}");
    assert_eq!(p["regions_dropped"], 1, "{p}");
    assert_eq!(original(&s, "sheet").triangle_count(), copy_triangles);
    assert!(working(&s, "sheet").is_none());
    assert!(s.doc.regions.is_empty());
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(original(&s, "sheet").triangle_count(), original_triangles);
    assert_eq!(working(&s, "sheet").unwrap().triangle_count(), copy_triangles);
    assert_eq!(s.doc.regions.len(), 1);
}

/// Discard drops the copy and leaves the original alone; undo brings the copy back.
#[test]
fn p8_discard_keeps_the_original_and_undo_restores_the_copy() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let original_triangles = original(&s, "sheet").triangle_count();
    s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    let copy_triangles = working(&s, "sheet").unwrap().triangle_count();
    let d = s.execute("mesh.working_discard", json!({"object": "sheet"})).unwrap();
    assert_eq!(d["triangles"], copy_triangles, "{d}");
    assert!(working(&s, "sheet").is_none());
    assert_eq!(original(&s, "sheet").triangle_count(), original_triangles);
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(working(&s, "sheet").unwrap().triangle_count(), copy_triangles);
}

/// A rigid transform moves the original and its copy together, so the distance stays zero.
#[test]
fn p8_transform_moves_the_copy_with_the_original() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    s.execute("mesh.transform", json!({"object": "sheet", "translate": [0.0, 0.0, 5.0]})).unwrap();
    let moved = original(&s, "sheet");
    assert!((moved.positions[0].z - 5.0).abs() < EPS);
    assert!((working(&s, "sheet").unwrap().positions[0].z - 5.0).abs() < EPS);
    let d = distance(&mut s, "sheet");
    near(&d["working_to_original"]["max"], 0.0, EPS);
}

/// Misuse gives errors, and a failed step leaves the document as it was.
#[test]
fn p8_working_copy_errors() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    s.execute("mesh.synth", json!({"shape": "box", "name": "cube"})).unwrap();
    assert!(s.execute("mesh.working_begin", json!({"object": "sheet"})).is_err());
    assert!(s.execute("mesh.working_distance", json!({"object": "sheet", "tolerance": -1.0})).is_err());
    for id in ["mesh.working_distance", "mesh.working_promote", "mesh.working_discard"] {
        assert!(s.execute(id, json!({"object": "cube"})).is_err(), "{id} without a copy");
    }
    assert!(working(&s, "sheet").is_some());
    assert!(s.execute("mesh.working_discard", json!({"object": "nope"})).is_err());
}

/// A project keeps the copy and its distance numbers; a project without a copy has no `working` key.
#[test]
fn p8_save_and_open_keep_the_copy() {
    let dir = std::env::temp_dir().join(format!("sc-d2-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("copy.scproj").to_string_lossy().to_string();
    let plain = dir.join("plain.scproj").to_string_lossy().to_string();
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    let before = distance(&mut s, "sheet");
    s.execute("doc.save", json!({"path": path})).unwrap();
    s.execute("mesh.working_discard", json!({"object": "sheet"})).unwrap();
    s.execute("doc.save", json!({"path": plain})).unwrap();
    assert!(!std::fs::read_to_string(&plain).unwrap().contains("\"working\""));
    let mut t = Session::new();
    t.execute("doc.open", json!({"path": path})).unwrap();
    let after = distance(&mut t, "sheet");
    assert_eq!(before["working_to_original"], after["working_to_original"]);
    assert_eq!(before["original_to_working"], after["original_to_working"]);
    let _ = std::fs::remove_dir_all(dir);
}
