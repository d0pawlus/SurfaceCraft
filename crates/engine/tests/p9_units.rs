//! P9-U1: units of scans. STL, OBJ, PLY and XYZ carry no length unit; `mesh.import` and
//! `mesh.import_stream` take `units` and convert to millimetres, `mesh.set_units` converts a mesh
//! already imported (with everything that depends on it), and a size that looks like a unit
//! mistake is reported as `suspicious_scale` by `mesh.import`, `mesh.info`, `scan.qa` and
//! `scan.to_solid`.
#[path = "p9_units/support.rs"]
mod support;
use serde_json::{Value, json};
use support::*;
use surfacecraft_doc::{LengthUnit, format};
use surfacecraft_engine::Session;
use surfacecraft_fit::primitive::Primitive;
use surfacecraft_geom::{Mesh, Point3};

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
fn one_far_outlier_does_not_change_the_decision_on_a_mesh_of_a_hundred_vertices_or_more() {
    let dir = TempDir::new("outlier");
    let mut s = Session::new();
    let sixty = plane_with_outlier(&dir, "sixty.obj", 60.0, 40.0);
    let out = import_as(&mut s, &sixty, None, "sixty");
    assert!(out.get("scale_warning").is_none(), "a 60 mm part with one outlier is still fine: {out}");
    let half = plane_with_outlier(&dir, "half.obj", 0.5, 0.5);
    let out = import_as(&mut s, &half, None, "half");
    assert_eq!(warning_code(&out), Some("suspicious_scale"), "a 0.5 mm part is still too small");
    let huge = plane_with_outlier(&dir, "huge.obj", 30000.0, 20000.0);
    let out = import_as(&mut s, &huge, None, "huge");
    assert_eq!(out["scale_warning"]["problem"], "micrometres", "a genuinely large part is still reported");
}

#[test]
fn acknowledging_millimetres_is_not_refused_by_a_dependent_body() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [0.5, 0.5, 0.5], "name": "tiny"})).unwrap();
    s.execute("body.box", json!({"size": [40.0, 30.0, 15.0], "name": "block"})).unwrap();
    let out = s.execute("mesh.set_units", json!({"object": "tiny", "from": "mm"}));
    let out = out.expect("an acknowledgement changes nothing, so bodies do not refuse it");
    assert_eq!(out["factor"], 1.0);
    assert!(info_of(&mut s, "tiny").get("scale_warning").is_none(), "the warning goes away");
    assert!(
        s.execute("mesh.set_units", json!({"object": "tiny", "from": "m"})).is_err(),
        "a real rescale is still refused"
    );
}

#[test]
fn set_units_with_force_has_no_units_note() {
    let dir = TempDir::new("force-note");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), None, "scan");
    s.execute("body.box", json!({"size": [40.0, 30.0, 15.0], "name": "block"})).unwrap();
    let out = s.execute("mesh.set_units", json!({"object": "scan", "from": "m", "force": true})).unwrap();
    assert!(out.get("units_note").is_none(), "no note after force: {out}");
}

#[test]
fn a_rejected_overwrite_keeps_the_units_record() {
    let dir = TempDir::new("keep-record");
    let m_path = scaled_file(&dir, "stream-m.stl", units_per_mm("m"));
    let store = dir.file("store");
    let mut s = Session::new();
    s.execute("mesh.import_stream", json!({"path": m_path, "store_dir": store, "units": "m"})).unwrap();
    let rejected = s
        .execute("mesh.import_stream", json!({"path": dir.file("missing.stl"), "store_dir": store, "overwrite": true}));
    assert!(rejected.is_err());
    let info = s.execute("mesh.store_info", json!({"store_dir": store})).unwrap();
    assert_eq!(info["units"], "m", "the record survives a rejected call: {info}");
}

#[test]
fn a_hostile_units_record_falls_back_to_nothing_remembered() {
    let dir = TempDir::new("hostile-record");
    let m_path = scaled_file(&dir, "stream-m.stl", units_per_mm("m"));
    let store = dir.file("store");
    let mut s = Session::new();
    s.execute("mesh.import_stream", json!({"path": m_path, "store_dir": store, "units": "m"})).unwrap();
    let record = std::path::Path::new(&store).join("units.json");
    let hostile: Vec<Vec<u8>> = vec![
        b"not json".to_vec(),
        br#"{"units": 5, "stated": true}"#.to_vec(),
        br#"{"units": "m", "units": "yd"}"#.to_vec(),
        br#"{"units": "m", "stated": "yes"}"#.to_vec(),
        vec![0xff, 0xfe, 0x00, 0x81],
        vec![b' '; 5 * 1024 * 1024],
    ];
    for bytes in hostile {
        std::fs::write(&record, &bytes).unwrap();
        let info = s
            .execute("mesh.store_info", json!({"store_dir": store}))
            .unwrap_or_else(|e| panic!("store_info must not fail on a hostile record: {e:?}"));
        assert!(info.get("units").is_none(), "nothing remembered: {info}");
    }
}

#[test]
fn a_small_pin_keeps_its_apex_and_is_not_read_as_metres() {
    let dir = TempDir::new("pin-apex");
    let path = dir.file("pin.obj");
    std::fs::write(&path, "v -0.6 0 0\nv 0.6 0 0\nv 0 0 1.2\nv 0 0.6 0\nf 1 2 3\nf 1 2 4\n").unwrap();
    let mut s = Session::new();
    let out = import_as(&mut s, &path, None, "pin");
    assert!(out.get("scale_warning").is_none(), "1.2 mm pin: {out}");
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
fn set_units_in_the_unit_already_recorded_only_acknowledges_it() {
    let dir = TempDir::new("ack-same");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), Some("m"), "scan");
    let before = mesh_of(&s, "scan");
    let out = s.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).unwrap();
    assert_eq!(out["factor"], 1.0);
    assert_eq!(worst_gap(&mesh_of(&s, "scan"), &before), 0.0, "nothing is rescaled");
    assert!(out["units_note"].as_str().unwrap().contains("stays"));
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
    // The points of the two files differ in the last bits (0.001 is not a power of two), and the
    // bracket is a regular lattice full of exact ties; reconstruction rounds the cloud to a lattice
    // far below scanner accuracy first, so both runs triangulate the very same points.
    assert_eq!(ma.triangles, mb.triangles, "the stated unit changed the triangulation");
    assert_eq!(ma.positions, mb.positions, "the stated unit changed the points");
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

// --- review round 1: corrections, acknowledgement, overflow, what is left alone ----------------

#[test]
fn set_units_corrects_a_cm_choice_to_m_and_equals_a_direct_m_import() {
    let dir = TempDir::new("correct-cm-m");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut wrong = Session::new();
    import_as(&mut wrong, &path, Some("cm"), "scan");
    let out = wrong.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).unwrap();
    assert!((out["factor"].as_f64().unwrap() - 100.0).abs() < 1e-9, "factor from cm to m is 100");
    let mut right = Session::new();
    import_as(&mut right, &path, Some("m"), "scan");
    assert!(worst_gap(&mesh_of(&wrong, "scan"), &mesh_of(&right, "scan")) <= 1e-9);
    assert_eq!(info_of(&mut wrong, "scan")["source_units"], "m");
}

#[test]
fn set_units_corrects_m_to_mm() {
    let dir = TempDir::new("correct-m-mm");
    let path = scaled_file(&dir, "mm.obj", 1.0);
    let mut wrong = Session::new();
    import_as(&mut wrong, &path, Some("m"), "scan");
    wrong.execute("mesh.set_units", json!({"object": "scan", "from": "mm"})).unwrap();
    let mut right = Session::new();
    import_as(&mut right, &path, Some("mm"), "scan");
    assert!(worst_gap(&mesh_of(&wrong, "scan"), &mesh_of(&right, "scan")) <= 1e-9);
}

#[test]
fn set_units_corrects_in_to_ft() {
    let dir = TempDir::new("correct-in-ft");
    let path = scaled_file(&dir, "feet.obj", units_per_mm("ft"));
    let mut wrong = Session::new();
    import_as(&mut wrong, &path, Some("in"), "scan");
    let out = wrong.execute("mesh.set_units", json!({"object": "scan", "from": "ft"})).unwrap();
    assert!((out["factor"].as_f64().unwrap() - 12.0).abs() < 1e-9);
    let mut right = Session::new();
    import_as(&mut right, &path, Some("ft"), "scan");
    assert!(worst_gap(&mesh_of(&wrong, "scan"), &mesh_of(&right, "scan")) <= 1e-9);
}

#[test]
fn set_units_mm_acknowledges_a_defaulted_unit_without_rescaling() {
    let dir = TempDir::new("ack");
    let path = plain_box(&dir, "pin.obj", 0.8);
    let mut s = Session::new();
    let before = import_as(&mut s, &path, None, "pin");
    assert_eq!(warning_code(&before), Some("suspicious_scale"), "an unstated 0.8 mm part warns");
    let geometry = mesh_of(&s, "pin");
    let out = s.execute("mesh.set_units", json!({"object": "pin", "from": "mm"})).unwrap();
    assert!(out["scale_warning"].is_null(), "acknowledged: {out}");
    assert_eq!(worst_gap(&mesh_of(&s, "pin"), &geometry), 0.0, "no rescale");
    assert!(info_of(&mut s, "pin").get("scale_warning").is_none());
}

#[test]
fn explicit_mm_silences_the_warning_on_a_real_small_part() {
    let dir = TempDir::new("pin-mm");
    let path = plain_box(&dir, "pin.obj", 0.8);
    let mut s = Session::new();
    let out = import_as(&mut s, &path, Some("mm"), "pin");
    assert!(out.get("scale_warning").is_none(), "a stated unit is not second-guessed: {out}");
    assert!(info_of(&mut s, "pin").get("scale_warning").is_none());
}

#[test]
fn a_stated_unit_that_does_not_fit_gives_no_warning() {
    let dir = TempDir::new("stated-cm");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut s = Session::new();
    let out = import_as(&mut s, &path, Some("cm"), "scan");
    assert!(out.get("scale_warning").is_none(), "stated cm: {out}");
}

#[test]
fn a_25_metre_hull_section_stated_in_mm_gives_no_warning() {
    let dir = TempDir::new("hull");
    let path = plain_box(&dir, "hull.obj", 25000.0);
    let mut s = Session::new();
    let out = import_as(&mut s, &path, Some("mm"), "hull");
    assert!(out.get("scale_warning").is_none(), "{out}");
}

#[test]
fn set_units_refuses_after_scale_from_length() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [60.0, 40.0, 20.0], "name": "block"})).unwrap();
    s.execute("mesh.scale_from_length", json!({"object": "block", "from": [0, 0, 0], "to": [1, 0, 0], "length": 2.0}))
        .unwrap();
    assert!(s.execute("mesh.set_units", json!({"object": "block", "from": "m"})).is_err());
}

#[test]
fn set_units_refuses_while_bodies_exist_unless_forced_and_reports_what_it_left() {
    let dir = TempDir::new("left");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), None, "scan");
    s.execute("body.box", json!({"size": [40.0, 30.0, 15.0], "name": "block"})).unwrap();
    let before = mesh_of(&s, "scan");
    assert!(s.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).is_err());
    assert_eq!(worst_gap(&mesh_of(&s, "scan"), &before), 0.0, "refusal changes nothing");
    let out = s.execute("mesh.set_units", json!({"object": "scan", "from": "m", "force": true})).unwrap();
    assert_eq!(out["left_alone"]["bodies"], 1);
}

#[test]
fn scale_from_length_reports_what_it_left_alone() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [60.0, 40.0, 20.0], "name": "block"})).unwrap();
    s.execute("body.box", json!({"size": [40.0, 30.0, 15.0], "name": "other"})).unwrap();
    let out = s
        .execute(
            "mesh.scale_from_length",
            json!({"object": "block", "from": [0, 0, 0], "to": [1, 0, 0], "length": 2.0}),
        )
        .unwrap();
    assert_eq!(out["left_alone"]["bodies"], 1);
}

#[test]
fn set_units_scales_a_working_copy_and_one_undo_restores_it() {
    let dir = TempDir::new("working");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), None, "scan");
    s.execute("mesh.working_begin", json!({"object": "scan"})).unwrap();
    let (id, _) = s.doc.find_by_name("scan").unwrap();
    let before = s.doc.mesh(id).unwrap().working.as_ref().unwrap().mesh.bbox().diagonal();
    s.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).unwrap();
    let after = s.doc.mesh(id).unwrap().working.as_ref().unwrap().mesh.bbox().diagonal();
    assert!((after - before * 1000.0).abs() <= 1e-9 * after);
    s.execute("edit.undo", Value::Null).unwrap();
    let restored = s.doc.mesh(id).unwrap().working.as_ref().unwrap().mesh.bbox().diagonal();
    assert!((restored - before).abs() <= 1e-12 * before);
}

#[test]
fn set_units_scales_a_mesh_moved_by_mesh_transform() {
    let dir = TempDir::new("moved");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), None, "scan");
    s.execute("mesh.transform", json!({"object": "scan", "translate": [5.0, 0.0, 0.0]})).unwrap();
    let moved = mesh_of(&s, "scan");
    s.execute("mesh.set_units", json!({"object": "scan", "from": "m"})).unwrap();
    let scaled = mesh_of(&s, "scan");
    let expect = Mesh {
        positions: moved.positions.iter().map(|p| Point3::from(p.coords * 1000.0)).collect(),
        triangles: moved.triangles.clone(),
    };
    assert!(worst_gap(&scaled, &expect) <= 1e-9);
}

#[test]
fn metres_import_exports_and_reimports_as_the_same_millimetres() {
    let dir = TempDir::new("roundtrip");
    let mut s = Session::new();
    import_as(&mut s, &scaled_file(&dir, "metres.obj", units_per_mm("m")), Some("m"), "scan");
    let out = dir.file("roundtrip.obj");
    s.execute("mesh.export", json!({"object": "scan", "path": out})).unwrap();
    let mut again = Session::new();
    import_as(&mut again, &out, None, "scan");
    assert!(worst_gap(&mesh_of(&again, "scan"), &mesh_of(&s, "scan")) <= 1e-9);
}

#[test]
fn scale_and_units_multiply() {
    let dir = TempDir::new("scale-units");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut both = Session::new();
    both.execute("mesh.import", json!({"path": path, "units": "m", "scale": 2.0, "name": "scan"})).unwrap();
    let mut once = Session::new();
    once.execute("mesh.import", json!({"path": path, "scale": 2000.0, "name": "scan"})).unwrap();
    assert!(worst_gap(&mesh_of(&both, "scan"), &mesh_of(&once, "scan")) <= 1e-12);
}

#[test]
fn a_coordinate_that_overflows_after_scaling_is_refused() {
    let dir = TempDir::new("overflow");
    let path = dir.file("huge.obj");
    std::fs::write(&path, "v 1e306 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n").unwrap();
    let mut s = Session::new();
    let r = s.execute("mesh.import", json!({"path": path, "units": "ft", "name": "huge"}));
    assert!(r.is_err(), "an infinite coordinate must not enter the document");
    assert!(s.doc.meshes.is_empty());
}

#[test]
fn extreme_coordinates_never_enter_the_document_finite_or_refused() {
    let dir = TempDir::new("extreme");
    for (i, value) in ["1e306", "-1e308", "1.7e308"].iter().enumerate() {
        for unit in ["ft", "m", "um", "in"] {
            let path = dir.file(&format!("extreme-{i}-{unit}.obj"));
            std::fs::write(&path, format!("v {value} 0 0\nv 0 1 0\nv 0 0 1\nf 1 2 3\n")).unwrap();
            let mut s = Session::new();
            if s.execute("mesh.import", json!({"path": path, "units": unit, "name": "x"})).is_ok() {
                assert!(mesh_of(&s, "x").positions.iter().all(|p| p.coords.iter().all(|c| c.is_finite())));
            }
        }
    }
}

#[test]
fn reconstruct_takes_units_like_import() {
    let dir = TempDir::new("reconstruct-units");
    let want = mesh_of(&bracket_session(), "part");
    let mm_text: String = want.positions.iter().map(|p| format!("{} {} {}\n", p.x, p.y, p.z)).collect();
    let m_text: String =
        want.positions.iter().map(|p| format!("{} {} {}\n", p.x * 0.001, p.y * 0.001, p.z * 0.001)).collect();
    let (mm_path, m_path) = (dir.file("points-mm.xyz"), dir.file("points-m.xyz"));
    std::fs::write(&mm_path, mm_text).unwrap();
    std::fs::write(&m_path, m_text).unwrap();
    let mut a = Session::new();
    a.execute("mesh.reconstruct", json!({"path": mm_path, "name": "cloud"})).unwrap();
    let mut b = Session::new();
    b.execute("mesh.reconstruct", json!({"path": m_path, "units": "m", "name": "cloud"})).unwrap();
    let (ma, mb) = (mesh_of(&a, "cloud"), mesh_of(&b, "cloud"));
    assert_eq!(ma.triangles, mb.triangles, "the stated unit changed the triangulation");
    assert_eq!(ma.positions, mb.positions, "the stated unit changed the points");
}

#[test]
fn import_stream_records_its_unit_and_store_info_shows_it_with_the_size_check() {
    let dir = TempDir::new("store-units");
    let m_path = dir.file("stream-m.stl");
    let mut s = bracket_session();
    s.execute(
        "mesh.scale_from_length",
        json!({"object": "part", "from": [0, 0, 0], "to": [1, 0, 0], "length": 0.001, "origin": [0, 0, 0]}),
    )
    .unwrap();
    s.execute("mesh.export", json!({"object": "part", "path": m_path})).unwrap();
    let stated = dir.file("store-stated");
    s.execute("mesh.import_stream", json!({"path": m_path, "store_dir": stated, "units": "m"})).unwrap();
    let info = s.execute("mesh.store_info", json!({"store_dir": stated})).unwrap();
    assert_eq!(info["units"], "m");
    assert!(info.get("scale_warning").is_none(), "stated: {info}");
    let unstated = dir.file("store-unstated");
    s.execute("mesh.import_stream", json!({"path": m_path, "store_dir": unstated})).unwrap();
    let info = s.execute("mesh.store_info", json!({"store_dir": unstated})).unwrap();
    assert_eq!(warning_code(&info), Some("suspicious_scale"), "{info}");
}

#[test]
fn to_solid_verdict_follows_the_acknowledgement_and_the_finding_is_listed_once() {
    let dir = TempDir::new("verdict");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut acknowledged = Session::new();
    import_as(&mut acknowledged, &path, Some("m"), "scan");
    let good = acknowledged.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();
    let mut unstated = Session::new();
    import_as(&mut unstated, &path, None, "scan");
    let bad = unstated.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();
    assert_ne!(bad["verdict"], "pass", "an unstated scale is never a pass");
    let count = |out: &Value| {
        let in_issues = out["issues"].as_array().unwrap().iter().filter(|i| i["code"] == "suspicious_scale").count();
        let in_findings = out["scan_report"]["findings"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|f| f["code"] == "suspicious_scale")
            .count();
        in_issues + in_findings
    };
    assert_eq!(count(&bad), 1, "listed once across findings and issues");
    assert_eq!(count(&good), 0);
    let mut fresh = Session::new();
    import_as(&mut fresh, &path, Some("m"), "scan");
    let same = fresh.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();
    assert_eq!(good["verdict"], same["verdict"], "the acknowledged metres verdict is the millimetre one");
}
