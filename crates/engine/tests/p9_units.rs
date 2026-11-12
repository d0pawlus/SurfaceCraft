//! P9-U1: units of scans. STL, OBJ, PLY and XYZ carry no length unit; `mesh.import` and
//! `mesh.import_stream` take `units` and convert to millimetres, `mesh.set_units` converts a mesh
//! already imported (with everything that depends on it), and a size that looks like a unit
//! mistake is reported as `suspicious_scale` by `mesh.import`, `mesh.info`, `scan.qa` and
//! `scan.to_solid`.
use serde_json::{Value, json};
use std::path::PathBuf;
use surfacecraft_doc::{LengthUnit, format};
use surfacecraft_engine::Session;
use surfacecraft_fit::primitive::Primitive;
use surfacecraft_geom::Mesh;

/// Removes its directory when dropped.
struct TempDir(PathBuf);

impl TempDir {
    fn new(tag: &str) -> Self {
        let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos());
        let dir = std::env::temp_dir().join(format!("surfacecraft-p9-units-{tag}-{}-{nanos}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        Self(dir)
    }

    fn file(&self, name: &str) -> String {
        self.0.join(name).to_string_lossy().to_string()
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// The mesh object `name` of `s`, copied out.
fn mesh_of(s: &Session, name: &str) -> Mesh {
    let (id, _) = s.doc.find_by_name(name).unwrap();
    s.doc.mesh(id).unwrap().mesh.as_ref().clone()
}

/// Largest distance between corresponding vertices, relative to the diagonal of `reference`.
fn worst_gap(a: &Mesh, reference: &Mesh) -> f64 {
    assert_eq!(a.positions.len(), reference.positions.len(), "vertex counts differ");
    let diag = reference.bbox().diagonal();
    a.positions.iter().zip(&reference.positions).map(|(p, q)| (p - q).norm() / diag).fold(0.0, f64::max)
}

/// Largest distance between the corners of corresponding triangles, relative to the largest
/// coordinate of `reference`. STL re-indexes the vertices by first use, so the triangles are
/// compared, not the vertex lists. Single precision is judged against the size of the coordinates.
fn worst_corner_gap(a: &Mesh, reference: &Mesh) -> f64 {
    assert_eq!(a.triangles.len(), reference.triangles.len(), "triangle counts differ");
    let magnitude = reference.positions.iter().map(|p| p.coords.amax()).fold(0.0, f64::max);
    let corners = |m: &Mesh, t: &[u32; 3]| t.map(|v| m.positions[v as usize]);
    a.triangles
        .iter()
        .zip(&reference.triangles)
        .flat_map(|(ta, tb)| corners(a, ta).into_iter().zip(corners(reference, tb)))
        .map(|(p, q)| (p - q).norm() / magnitude)
        .fold(0.0, f64::max)
}

/// A bracket as the test reference (millimetres), in a session of its own.
fn bracket_session() -> Session {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "name": "part"})).unwrap();
    s
}

/// The bracket written to `dir` with every length multiplied by `factor` (the file in another unit).
fn scaled_file(dir: &TempDir, name: &str, factor: f64) -> String {
    let mut s = bracket_session();
    s.execute(
        "mesh.scale_from_length",
        json!({"object": "part", "from": [0, 0, 0], "to": [1, 0, 0], "length": factor, "origin": [0, 0, 0]}),
    )
    .unwrap();
    let path = dir.file(name);
    s.execute("mesh.export", json!({"object": "part", "path": path})).unwrap();
    path
}

/// Coordinates per millimetre of each unit: a file in `unit` holds this many units per mm.
fn units_per_mm(unit: &str) -> f64 {
    match unit {
        "m" => 0.001,
        "cm" => 0.1,
        "in" => 1.0 / 25.4,
        "ft" => 1.0 / 304.8,
        "um" => 1000.0,
        _ => 1.0,
    }
}

fn import_as(s: &mut Session, path: &str, units: Option<&str>, name: &str) -> Value {
    let mut params = json!({"path": path, "name": name});
    if let Some(u) = units {
        params["units"] = json!(u);
    }
    s.execute("mesh.import", params).unwrap()
}

/// A box with one far outlier vertex, as an OBJ file (eight corners plus the outlier).
fn box_with_outlier(dir: &TempDir, name: &str, size: f64) -> String {
    let h = size / 2.0;
    let corners = [[-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h], [-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h]];
    let mut text: String = corners.iter().map(|c| format!("v {} {} {}\n", c[0], c[1], c[2])).collect();
    text.push_str("v 1000000 0 0\n");
    text.push_str("f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n");
    let path = dir.file(name);
    std::fs::write(&path, text).unwrap();
    path
}

// --- units of the import ---------------------------------------------------------------------

#[test]
fn metres_file_imports_as_the_millimetre_mesh_when_units_is_m() {
    // Arrange
    let dir = TempDir::new("metres");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let reference = bracket_session();
    // Act
    let mut s = Session::new();
    let out = import_as(&mut s, &path, Some("m"), "scan");
    // Assert
    let got = mesh_of(&s, "scan");
    let want = mesh_of(&reference, "part");
    assert_eq!(got.triangle_count(), want.triangle_count());
    assert!(worst_gap(&got, &want) <= 1e-9, "vertices differ by more than 1e-9 of the diagonal");
    assert_eq!(out["source_units"], "m");
    assert!(out["bbox_max"][0].as_f64().unwrap() > 1.0, "the import is in millimetres");
}

#[test]
fn every_unit_gives_the_same_mesh_as_its_millimetre_file() {
    let want = mesh_of(&bracket_session(), "part");
    for unit in ["cm", "in", "ft", "um"] {
        let dir = TempDir::new(unit);
        let path = scaled_file(&dir, &format!("part-{unit}.obj"), units_per_mm(unit));
        let mut s = Session::new();
        import_as(&mut s, &path, Some(unit), "scan");
        let got = mesh_of(&s, "scan");
        assert_eq!(got.triangle_count(), want.triangle_count(), "{unit}");
        assert!(worst_gap(&got, &want) <= 1e-9, "{unit}: vertices differ");
    }
}

#[test]
fn stl_in_metres_agrees_within_single_precision() {
    // STL stores each coordinate as f32, so the metre file cannot be exact: about 6e-8 relative.
    let dir = TempDir::new("stl");
    let path = scaled_file(&dir, "metres.stl", units_per_mm("m"));
    let mut s = Session::new();
    import_as(&mut s, &path, Some("m"), "scan");
    let got = mesh_of(&s, "scan");
    let want = mesh_of(&bracket_session(), "part");
    assert_eq!(got.triangle_count(), want.triangle_count());
    assert!(worst_corner_gap(&got, &want) <= 1e-6, "STL corners differ by more than single precision");
}

#[test]
fn unknown_unit_is_an_error() {
    let dir = TempDir::new("unknown");
    let path = scaled_file(&dir, "part.obj", 1.0);
    let mut s = Session::new();
    let r = s.execute("mesh.import", json!({"path": path, "units": "yd"}));
    assert!(r.is_err(), "a yard is not a unit SurfaceCraft knows");
    assert!(s.doc.meshes.is_empty(), "nothing was imported");
}

#[test]
fn omitted_units_is_the_same_mesh_as_units_mm() {
    let dir = TempDir::new("omitted");
    let path = scaled_file(&dir, "part.obj", 1.0);
    let mut plain = Session::new();
    import_as(&mut plain, &path, None, "scan");
    let mut mm = Session::new();
    let out = import_as(&mut mm, &path, Some("mm"), "scan");
    let (a, b) = (mesh_of(&plain, "scan"), mesh_of(&mm, "scan"));
    assert!(a.positions.iter().zip(&b.positions).all(|(p, q)| p.coords == q.coords), "bit-identical positions");
    assert_eq!(a.triangles, b.triangles);
    assert_eq!(out["source_units"], "mm");
}

#[test]
fn mesh_info_shows_the_source_unit() {
    let dir = TempDir::new("info");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut s = Session::new();
    import_as(&mut s, &path, Some("m"), "scan");
    let info = s.execute("mesh.info", json!({"object": "scan"})).unwrap();
    assert_eq!(info["source_units"], "m");
}

// --- the size check --------------------------------------------------------------------------

fn warning_code(v: &Value) -> Option<&str> {
    v["scale_warning"]["code"].as_str()
}

#[test]
fn metre_file_read_as_millimetres_gives_suspicious_scale_with_a_fix() {
    let dir = TempDir::new("warn-metres");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut s = Session::new();
    let out = import_as(&mut s, &path, None, "scan");
    assert_eq!(warning_code(&out), Some("suspicious_scale"));
    let suggestion = out["scale_warning"]["suggestion"].as_str().unwrap();
    assert!(suggestion.contains("\"units\": \"m\""), "the exact call: {suggestion}");
    assert!(suggestion.contains("mesh.set_units"), "the other way to fix it: {suggestion}");
    assert!(out["scale_warning"]["extent"].as_f64().unwrap() < 1.0);
}

#[test]
fn normal_sixty_millimetre_part_has_no_warning() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [60.0, 40.0, 20.0], "name": "block"})).unwrap();
    let info = s.execute("mesh.info", json!({"object": "block"})).unwrap();
    assert!(info.get("scale_warning").is_none(), "no warning for 60 mm: {info}");
}

#[test]
fn half_millimetre_part_is_below_one_millimetre_and_warns_as_metres() {
    let mut s = Session::new();
    let out = s.execute("mesh.synth", json!({"shape": "box", "size": [0.5, 0.5, 0.5], "name": "tiny"})).unwrap();
    assert_eq!(warning_code(&out), Some("suspicious_scale"));
    assert!(out["scale_warning"]["message"].as_str().unwrap().contains("metres"));
}

#[test]
fn thirty_metre_part_read_as_millimetres_warns_as_micrometres() {
    let mut s = Session::new();
    let out = s.execute("mesh.synth", json!({"shape": "box", "size": [30000.0, 20000.0, 9000.0], "name": "huge"}));
    let out = out.unwrap();
    assert_eq!(warning_code(&out), Some("suspicious_scale"));
    let suggestion = out["scale_warning"]["suggestion"].as_str().unwrap();
    assert!(suggestion.contains("\"um\""), "{suggestion}");
}

#[test]
fn one_far_outlier_does_not_change_the_decision() {
    let dir = TempDir::new("outlier");
    let mut s = Session::new();
    let sixty = box_with_outlier(&dir, "sixty.obj", 60.0);
    let out = import_as(&mut s, &sixty, None, "sixty");
    assert!(out.get("scale_warning").is_none(), "a 60 mm part with one outlier is still fine: {out}");
    let half = box_with_outlier(&dir, "half.obj", 0.5);
    let out = import_as(&mut s, &half, None, "half");
    assert_eq!(warning_code(&out), Some("suspicious_scale"), "a 0.5 mm part is still too small");
}

#[test]
fn scan_qa_reports_suspicious_scale_on_the_metre_scan_only() {
    let dir = TempDir::new("qa");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), None, "metres");
    import_as(&mut s, &scaled_file(&dir, "mm.obj", 1.0), None, "millimetres");
    let has = |qa: &Value| qa["findings"].as_array().unwrap().iter().any(|f| f["code"] == "suspicious_scale");
    assert!(has(&s.execute("scan.qa", json!({"object": "metres"})).unwrap()));
    assert!(!has(&s.execute("scan.qa", json!({"object": "millimetres"})).unwrap()));
}

#[test]
fn scan_to_solid_reports_suspicious_scale_on_the_metre_scan_only() {
    let dir = TempDir::new("solid");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), None, "metres");
    import_as(&mut s, &scaled_file(&dir, "mm.obj", 1.0), None, "millimetres");
    let codes = |out: &Value| -> Vec<String> {
        out["issues"].as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap().to_string()).collect()
    };
    let bad = s.execute("scan.to_solid", json!({"scan": "metres"})).unwrap();
    assert!(codes(&bad).iter().any(|c| c == "suspicious_scale"), "{:?}", codes(&bad));
    let good = s.execute("scan.to_solid", json!({"scan": "millimetres"})).unwrap();
    assert!(!codes(&good).iter().any(|c| c == "suspicious_scale"));
}

// --- mesh.set_units --------------------------------------------------------------------------

#[test]
fn set_units_turns_a_wrong_import_into_the_correct_mesh() {
    let dir = TempDir::new("set-units");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut wrong = Session::new();
    import_as(&mut wrong, &path, None, "scan");
    let out = wrong.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).unwrap();
    assert!((out["factor"].as_f64().unwrap() - 1000.0).abs() < 1e-9);
    let mut right = Session::new();
    import_as(&mut right, &path, Some("m"), "scan");
    let (a, b) = (mesh_of(&wrong, "scan"), mesh_of(&right, "scan"));
    assert!(worst_gap(&a, &b) <= 1e-9);
    assert_eq!(wrong.doc.mesh(wrong.doc.find_by_name("scan").unwrap().0).unwrap().source_units, LengthUnit::M);
}

#[test]
fn set_units_rescales_regions_and_a_fitted_cylinder_made_before_the_fix() {
    let dir = TempDir::new("cylinder");
    let mut cyl = Session::new();
    cyl.execute("mesh.synth", json!({"shape": "cylinder", "size": [10.0, 30.0], "name": "part"})).unwrap();
    let path = dir.file("cylinder.obj");
    cyl.execute("mesh.export", json!({"object": "part", "path": path})).unwrap();
    let mut s = Session::new();
    import_as(&mut s, &scaled_file_of(&dir, "m.obj", &path, 0.001), None, "scan");
    let n = s.doc.meshes[0].mesh.triangle_count() as u32;
    s.execute("region.create", json!({"mesh": "scan", "triangles": (0..n).collect::<Vec<u32>>(), "name": "all"}))
        .unwrap();
    s.execute("primitive.fit", json!({"region": "all"})).unwrap();
    let rms_before = s.doc.regions[0].rms.unwrap();
    s.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).unwrap();
    let rms_after = s.doc.regions[0].rms.unwrap();
    assert!((rms_after - rms_before * 1000.0).abs() <= 1e-9 * rms_before * 1000.0, "region RMS {rms_after}");
    let radius = match &s.doc.primitives[0].primitive {
        Primitive::Cylinder { radius, .. } => *radius,
        other => panic!("expected a cylinder, got {other:?}"),
    };
    assert!((radius - 10.0).abs() <= 1e-6, "radius {radius}");
}

/// The file at `src` with every coordinate multiplied by `factor`, written to `name` in `dir`.
fn scaled_file_of(dir: &TempDir, name: &str, src: &str, factor: f64) -> String {
    let mut s = Session::new();
    s.execute("mesh.import", json!({"path": src, "name": "src"})).unwrap();
    s.execute(
        "mesh.scale_from_length",
        json!({"object": "src", "from": [0, 0, 0], "to": [1, 0, 0], "length": factor, "origin": [0, 0, 0]}),
    )
    .unwrap();
    let path = dir.file(name);
    s.execute("mesh.export", json!({"object": "src", "path": path})).unwrap();
    path
}

#[test]
fn one_undo_restores_the_mesh_after_set_units() {
    let dir = TempDir::new("undo");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), None, "scan");
    let before = mesh_of(&s, "scan");
    s.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).unwrap();
    s.execute("edit.undo", Value::Null).unwrap();
    let after = mesh_of(&s, "scan");
    assert!(worst_gap(&after, &before) <= 1e-12);
    let (id, _) = s.doc.find_by_name("scan").unwrap();
    assert_eq!(s.doc.mesh(id).unwrap().source_units, LengthUnit::Mm);
}

#[test]
fn set_units_refuses_millimetres_and_a_second_conversion() {
    let dir = TempDir::new("refuse");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), None, "scan");
    assert!(s.execute("mesh.set_units", json!({"object": "scan", "from": "mm"})).is_err());
    s.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).unwrap();
    assert!(s.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).is_err(), "already converted");
}

#[test]
fn set_units_refuses_with_a_fitted_surface_and_changes_nothing() {
    let dir = TempDir::new("surface");
    let mut s = Session::new();
    let mut wave = Session::new();
    wave.execute("mesh.synth", json!({"shape": "wave", "resolution": 40, "name": "wave"})).unwrap();
    let mm_path = dir.file("wave-mm.obj");
    wave.execute("mesh.export", json!({"object": "wave", "path": mm_path})).unwrap();
    let path = scaled_file_of(&dir, "wave-m.obj", &mm_path, units_per_mm("m"));
    import_as(&mut s, &path, None, "scan");
    let n = s.doc.meshes[0].mesh.triangle_count() as u32;
    s.execute("region.create", json!({"mesh": "scan", "triangles": (0..n).collect::<Vec<u32>>(), "name": "free"}))
        .unwrap();
    s.execute("surface.fit", json!({"region": "free"})).unwrap();
    let before = mesh_of(&s, "scan");
    let depth = s.history.undo_labels().len();
    let err = s.execute("mesh.set_units", json!({"object": "scan", "from": "m"}));
    assert!(err.is_err(), "a fitted surface cannot be rescaled");
    assert_eq!(s.history.undo_labels().len(), depth, "no undo step");
    assert_eq!(worst_gap(&mesh_of(&s, "scan"), &before), 0.0);
}

// --- saving, loading, points, streams ---------------------------------------------------------

#[test]
fn source_units_survive_save_and_an_old_project_loads_as_millimetres() {
    let dir = TempDir::new("project");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "cm.obj", units_per_mm("cm")), Some("cm"), "scan");
    let text = format::to_json(&s.doc).unwrap();
    let loaded = format::from_json(&text).unwrap();
    assert_eq!(loaded.meshes[0].source_units, LengthUnit::Cm);
    // an old project: the field is simply not there
    let mut json: Value = serde_json::from_str(&text).unwrap();
    let doc_meshes = json["document"]["meshes"].as_array_mut().unwrap();
    for m in doc_meshes.iter_mut() {
        m.as_object_mut().unwrap().remove("source_units");
    }
    let old = format::from_json(&json.to_string()).unwrap();
    assert_eq!(old.meshes[0].source_units, LengthUnit::Mm);
}

#[test]
fn point_cloud_import_respects_units() {
    let dir = TempDir::new("points");
    let want = mesh_of(&bracket_session(), "part");
    let mut text = String::new();
    for p in &want.positions {
        text.push_str(&format!("{} {} {}\n", p.x, p.y, p.z));
    }
    let mm_path = dir.file("points-mm.xyz");
    std::fs::write(&mm_path, &text).unwrap();
    let m_text: String =
        want.positions.iter().map(|p| format!("{} {} {}\n", p.x * 0.001, p.y * 0.001, p.z * 0.001)).collect();
    let m_path = dir.file("points-m.xyz");
    std::fs::write(&m_path, m_text).unwrap();
    let mut a = Session::new();
    import_as(&mut a, &mm_path, None, "cloud");
    let mut b = Session::new();
    let out = import_as(&mut b, &m_path, Some("m"), "cloud");
    assert!(out["point_cloud"].is_object());
    let (ma, mb) = (mesh_of(&a, "cloud"), mesh_of(&b, "cloud"));
    // Ball pivoting breaks ties on the last bits of the coordinates, so the two runs may differ by a
    // few triangles; the size and the place of the reconstruction must agree.
    let (ta, tb) = (ma.triangle_count() as f64, mb.triangle_count() as f64);
    assert!((ta - tb).abs() <= 0.02 * ta, "triangles {ta} and {tb}");
    let (da, db) = (ma.bbox(), mb.bbox());
    assert!((da.min - db.min).norm() <= 1e-6 * da.diagonal(), "placement differs");
    assert!((da.max - db.max).norm() <= 1e-6 * da.diagonal(), "size differs");
}

#[test]
#[cfg(not(target_arch = "wasm32"))]
fn streamed_store_in_metres_matches_the_millimetre_store() {
    let dir = TempDir::new("stream");
    let m_path = dir.file("stream-m.stl");
    let mm_path = dir.file("stream-mm.stl");
    let mut s = bracket_session();
    s.execute("mesh.export", json!({"object": "part", "path": mm_path})).unwrap();
    s.execute(
        "mesh.scale_from_length",
        json!({"object": "part", "from": [0, 0, 0], "to": [1, 0, 0], "length": 0.001, "origin": [0, 0, 0]}),
    )
    .unwrap();
    s.execute("mesh.export", json!({"object": "part", "path": m_path})).unwrap();
    let mm_store = dir.file("store-mm");
    let m_store = dir.file("store-m");
    let mm = s.execute("mesh.import_stream", json!({"path": mm_path, "store_dir": mm_store})).unwrap();
    let m = s.execute("mesh.import_stream", json!({"path": m_path, "store_dir": m_store, "units": "m"})).unwrap();
    assert_eq!(m["triangles"], mm["triangles"]);
    let (a, b) = (m["bbox_max"][0].as_f64().unwrap(), mm["bbox_max"][0].as_f64().unwrap());
    assert!((a - b).abs() <= 1e-6 * b.abs().max(1.0));
    let (ea, eb) = (m["levels"][1]["error_bound"].as_f64().unwrap(), mm["levels"][1]["error_bound"].as_f64().unwrap());
    assert!((ea - eb).abs() <= 1e-6 * eb.abs().max(1e-12), "error bounds must scale with the store");
}
