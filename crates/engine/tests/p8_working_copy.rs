//! P8 (D2): working copies. The original stays as it was; edits act on the copy while it exists;
//! `mesh.working_distance` measures the copy against the original in both directions. Each test
//! gives the expected numbers from known geometry, not from the command under test.
use serde_json::{Value, json};
use std::sync::Arc;
use surfacecraft_doc::{Document, ObjectId, QuadObject, WorkingCopy};
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

/// A project keeps the copy: reopened, the copy and the original have the same geometry and the same
/// distance numbers. A project without a copy has no `working` key, and it still opens, with no copy.
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
    let (orig, copy) = (original(&s, "sheet"), working(&s, "sheet").unwrap());
    s.execute("doc.save", json!({"path": path})).unwrap();
    let mut t = Session::new();
    t.execute("doc.open", json!({"path": path})).unwrap();
    assert_eq!(original(&t, "sheet"), orig);
    assert_eq!(working(&t, "sheet"), Some(copy));
    let after = distance(&mut t, "sheet");
    assert_eq!(before["working_to_original"], after["working_to_original"]);
    assert_eq!(before["original_to_working"], after["original_to_working"]);
    // a project written without a copy: no `working` key, it opens, and there is no copy
    s.execute("mesh.working_discard", json!({"object": "sheet"})).unwrap();
    s.execute("doc.save", json!({"path": plain})).unwrap();
    assert!(!std::fs::read_to_string(&plain).unwrap().contains("\"working\""));
    let mut u = Session::new();
    u.execute("doc.open", json!({"path": plain})).unwrap();
    assert_eq!(original(&u, "sheet"), orig);
    assert!(working(&u, "sheet").is_none());
    let _ = std::fs::remove_dir_all(dir);
}

/// Gate: `align.center` moves the original by the translation that puts its bounding-box centre at
/// the origin, and moves the working copy by the same translation. A flat copy stays on the original.
#[test]
fn p8_align_moves_the_copy_with_the_original() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    let (orig_before, copy_before) = (original(&s, "sheet"), working(&s, "sheet").unwrap());
    // the sheet spans [0, 50] × [0, 50] × {0}: its centre is (25, 25, 0)
    let shift = -orig_before.bbox().center().coords;
    assert!((shift - Vec3::new(-25.0, -25.0, 0.0)).norm() < EPS, "shift {shift}");
    let r = s.execute("align.center", json!({"object": "sheet"})).unwrap();
    assert_eq!(r["applied"], true, "{r}");
    let (orig_after, copy_after) = (original(&s, "sheet"), working(&s, "sheet").unwrap());
    assert_moved(&orig_before, &orig_after, shift);
    assert_moved(&copy_before, &copy_after, shift);
    assert!(orig_after.bbox().center().coords.norm() < EPS, "centre is not at the origin");
    let d = distance(&mut s, "sheet");
    near(&d["working_to_original"]["max"], 0.0, EPS);
    near(&d["original_to_working"]["max"], 0.0, EPS);
}

/// Every vertex of `after` is the vertex of `before` moved by `shift`.
fn assert_moved(before: &Mesh, after: &Mesh, shift: Vec3) {
    assert_eq!(before.positions.len(), after.positions.len());
    for (b, a) in before.positions.iter().zip(&after.positions) {
        assert!((a - b - shift).norm() < EPS, "moved by {}, expected {shift}", a - b);
    }
}

/// A quad layout over the whole sheet: a placeholder layout, not a fit.
fn quad_over(mesh: ObjectId) -> QuadObject {
    QuadObject {
        id: ObjectId(0),
        name: "layout".into(),
        mesh,
        region: None,
        frame: [[0.0; 3]; 4],
        domain: [[0.0, 1.0], [0.0, 1.0]],
        nu: 1,
        nv: 1,
        vertices: vec![[0.0; 3]; 4],
        crease_u: Vec::new(),
        crease_v: Vec::new(),
        faces: Vec::new(),
        normals: Vec::new(),
        visible: true,
    }
}

/// Gate: `mesh.info` shows the working copy's counts and bounds next to the original's, and
/// `mesh.export` with `working` writes the copy (STL keeps the triangle count). The sixteen-cell
/// sheet has 2 × 16 × 16 = 512 triangles; the copy has the count `mesh.decimate` reports.
#[test]
fn p8_info_and_export_show_the_working_copy() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let info = s.execute("mesh.info", json!({"object": "sheet"})).unwrap();
    assert!(info.get("working").is_some(), "a fresh copy is shown: {info}");
    assert_eq!(info["triangles"], 512, "{info}");
    let d = s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    let copy_tris = d["triangles"].as_u64().unwrap();
    let info = s.execute("mesh.info", json!({"object": "sheet"})).unwrap();
    assert_eq!(info["triangles"], 512, "the original is not decimated: {info}");
    assert_eq!(info["working"]["triangles"], copy_tris, "{info}");
    assert_eq!(info["working"]["vertices"], working(&s, "sheet").unwrap().vertex_count());
    for k in 0..3 {
        near(&info["working"]["bbox_min"][k], info["bbox_min"][k].as_f64().unwrap(), EPS);
        near(&info["working"]["bbox_max"][k], info["bbox_max"][k].as_f64().unwrap(), EPS);
    }
    let out = s.execute("mesh.export", json!({"object": "sheet", "working": true})).unwrap();
    let back = s.execute("mesh.import", json!({"data": out["data"].clone(), "format": "stl", "name": "copy"})).unwrap();
    assert_eq!(back["triangles"], copy_tris, "the copy was exported");
    let out = s.execute("mesh.export", json!({"object": "sheet"})).unwrap();
    let back =
        s.execute("mesh.import", json!({"data": out["data"].clone(), "format": "stl", "name": "original"})).unwrap();
    assert_eq!(back["triangles"], 512, "the original was exported");
    s.execute("mesh.working_discard", json!({"object": "sheet"})).unwrap();
    assert!(s.execute("mesh.info", json!({"object": "sheet"})).unwrap().get("working").is_none());
    assert!(s.execute("mesh.export", json!({"object": "sheet", "working": true})).is_err());
}

/// Gate: promoting a copy whose triangles changed reports the quad layouts of the original that it
/// dropped; promoting one with the same triangles keeps them. Undo brings the layout back.
#[test]
fn p8_promote_reports_the_quad_layouts_dropped() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let id = id_of(&s, "sheet");
    s.doc.add_quad(quad_over(id));
    s.execute("mesh.smooth", json!({"object": "sheet", "iterations": 2})).unwrap();
    let kept = s.execute("mesh.working_promote", json!({"object": "sheet"})).unwrap();
    assert_eq!(kept["quad_layouts_dropped"], 0, "{kept}");
    assert_eq!(s.doc.quads.len(), 1);
    s.execute("mesh.working_begin", json!({"object": "sheet"})).unwrap();
    s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    let dropped = s.execute("mesh.working_promote", json!({"object": "sheet"})).unwrap();
    assert_eq!(dropped["quad_layouts_dropped"], 1, "{dropped}");
    assert!(s.doc.quads.is_empty());
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(s.doc.quads.len(), 1, "undo brings the layout back");
}

/// Gate: a copy or an original without triangles has nothing to measure, and `mesh.working_distance`
/// says so instead of giving numbers.
#[test]
fn p8_distance_refuses_meshes_without_triangles() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let id = id_of(&s, "sheet");
    let empty = || Mesh { positions: Vec::new(), triangles: Vec::new() };
    s.doc.mesh_mut(id).unwrap().working = Some(WorkingCopy { mesh: Arc::new(empty()) });
    assert!(s.execute("mesh.working_distance", json!({"object": "sheet"})).is_err(), "an empty copy");
    s.execute("mesh.working_discard", json!({"object": "sheet"})).unwrap();
    s.execute("mesh.working_begin", json!({"object": "sheet"})).unwrap();
    s.doc.mesh_mut(id).unwrap().mesh = Arc::new(empty());
    assert!(s.execute("mesh.working_distance", json!({"object": "sheet"})).is_err(), "an empty original");
}

/// Gate: a repeated id in `also` (the object again, or one id twice) moves each mesh once: the original
/// and its copy move by the centring shift, not by a multiple of it.
#[test]
fn p8_align_applies_the_transform_once_with_repeated_ids() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    s.execute("mesh.decimate", json!({"object": "sheet", "ratio": 0.5})).unwrap();
    let (orig_before, copy_before) = (original(&s, "sheet"), working(&s, "sheet").unwrap());
    let shift = -orig_before.bbox().center().coords;
    s.execute("align.center", json!({"object": "sheet", "also": ["sheet", "sheet"]})).unwrap();
    assert_moved(&orig_before, &original(&s, "sheet"), shift);
    assert_moved(&copy_before, &working(&s, "sheet").unwrap(), shift);
}

/// A project whose working copy has an out-of-range index, or a NaN coordinate, does not open.
#[test]
fn p8_hostile_working_copy_does_not_open() {
    let dir = std::env::temp_dir().join(format!("sc-d2-hostile-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let base = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 1.0, 1.0, 1, 1);
    let bad = [
        Mesh { positions: base.positions.clone(), triangles: vec![[0, 1, 99]] },
        Mesh {
            positions: vec![Point3::new(f64::NAN, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            triangles: vec![[0, 1, 2]],
        },
    ];
    for (k, copy) in bad.into_iter().enumerate() {
        let mut d = Document::new();
        let id = d.add_mesh("sheet", base.clone());
        d.mesh_mut(id).unwrap().working = Some(WorkingCopy { mesh: Arc::new(copy) });
        let path = dir.join(format!("bad-{k}.scproj")).to_string_lossy().to_string();
        std::fs::write(&path, surfacecraft_doc::format::to_json(&d).unwrap()).unwrap();
        let mut s = Session::new();
        assert!(s.execute("doc.open", json!({"path": path})).is_err(), "copy {k} opened");
    }
    let _ = std::fs::remove_dir_all(dir);
}

/// Gate: a session without file access refuses `path` for the distance map and writes nothing; the
/// summary alone still works.
#[test]
fn p8_distance_path_needs_file_access() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    s.allow_files = false;
    let path = std::env::temp_dir().join(format!("sc-d2-nofiles-{}.csv", std::process::id()));
    let path = path.to_string_lossy().to_string();
    assert!(s.execute("mesh.working_distance", json!({"object": "sheet", "path": path})).is_err());
    assert!(!std::path::Path::new(&path).exists(), "nothing was written");
    assert!(s.execute("mesh.working_distance", json!({"object": "sheet"})).is_ok());
}

/// Gate: promoting a copy with the same triangles (smoothing only moves vertices) keeps the regions of
/// the original, and nothing is reported dropped.
#[test]
fn p8_promote_with_unchanged_triangles_keeps_regions() {
    let mut s = Session::new();
    sheet_with_copy(&mut s, "sheet");
    let id = id_of(&s, "sheet");
    s.doc.add_region("patch", id, vec![0, 1, 2]);
    s.execute("mesh.smooth", json!({"object": "sheet", "iterations": 3})).unwrap();
    let p = s.execute("mesh.working_promote", json!({"object": "sheet"})).unwrap();
    assert_eq!(p["regions_dropped"], 0, "{p}");
    assert_eq!(p["quad_layouts_dropped"], 0, "{p}");
    assert_eq!(p["triangles"], p["triangles_before"], "{p}");
    assert_eq!(s.doc.regions.len(), 1);
}
