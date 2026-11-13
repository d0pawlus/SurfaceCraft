//! P9-U1, review rounds: the size of a stated unit, the millimetre confirmation, degenerate meshes,
//! the store's robust extent and a failed overwrite.
#[path = "p9_units/support.rs"]
mod support;
use serde_json::json;
use support::*;
use surfacecraft_engine::Session;

// --- review round 4 ------------------------------------------------------------------------------

#[test]
fn a_millimetre_file_stated_as_metres_may_already_be_millimetres() {
    let dir = TempDir::new("stated-mm-file");
    let path = plain_box(&dir, "block45.obj", 45.0);
    let mut s = Session::new();
    let out = import_as(&mut s, &path, Some("m"), "block");
    assert_eq!(out["scale_warning"]["problem"], "stated_unit_implausible", "{out}");
    assert!(out["scale_warning"]["suggestion"].as_str().unwrap().contains("without \"units\""));
}

#[test]
fn a_real_metre_file_stated_as_metres_is_silent() {
    let dir = TempDir::new("stated-metres");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut s = Session::new();
    let out = import_as(&mut s, &path, Some("m"), "scan");
    assert!(out.get("scale_warning").is_none(), "{out}");
}

#[test]
fn the_suggestion_always_names_the_millimetre_confirmation() {
    let dir = TempDir::new("confirm-mm");
    let path = scaled_file(&dir, "metres.obj", units_per_mm("m"));
    let mut s = Session::new();
    let out = import_as(&mut s, &path, None, "scan");
    let suggestion = out["scale_warning"]["suggestion"].as_str().unwrap();
    assert!(suggestion.contains("confirm millimetres with mesh.set_units from mm"), "{suggestion}");
}

#[test]
fn a_degenerate_mesh_gives_no_warning() {
    let dir = TempDir::new("degenerate");
    let path = dir.file("point.obj");
    std::fs::write(&path, "v 2 2 2\nv 2 2 2\nv 2 2 2\nf 1 2 3\n").unwrap();
    let mut s = Session::new();
    let out = import_as(&mut s, &path, None, "point");
    assert!(out.get("scale_warning").is_none(), "no size to judge: {out}");
}

#[test]
fn a_stored_part_with_one_far_outlier_is_judged_by_its_robust_extent() {
    let dir = TempDir::new("store-outlier");
    let path = grid_stl(&dir, "grid.stl", 60.0, 40.0, 60, true);
    let store = dir.file("store");
    let mut s = Session::new();
    s.execute("mesh.import_stream", json!({"path": path, "store_dir": store, "chunk_triangles": 64})).unwrap();
    let info = s.execute("mesh.store_info", json!({"store_dir": store})).unwrap();
    assert!(info.get("scale_warning").is_none(), "a 60 mm part with one stray triangle is fine: {info}");
    assert!(info["chunks"].as_u64().unwrap() >= 100, "enough chunks for the trimming to apply");
}

#[test]
fn a_failed_overwrite_restores_the_record_only_over_an_intact_store() {
    let dir = TempDir::new("damaged-overwrite");
    let m_path = scaled_file(&dir, "stream-m.stl", units_per_mm("m"));
    let store = dir.file("store");
    let mut s = Session::new();
    s.execute("mesh.import_stream", json!({"path": m_path, "store_dir": store, "units": "m"})).unwrap();
    // damage a block of the store: it is no longer intact
    let block = std::fs::read_dir(&store)
        .unwrap()
        .map(|e| e.unwrap().path())
        .filter(|p| p.file_name().is_some_and(|n| n != "units.json" && n != "index.scx"))
        .max_by_key(|p| std::fs::metadata(p).map(|m| m.len()).unwrap_or(0))
        .expect("a data file");
    std::fs::write(&block, b"").unwrap();
    let rejected = s
        .execute("mesh.import_stream", json!({"path": dir.file("missing.stl"), "store_dir": store, "overwrite": true}));
    assert!(rejected.is_err());
    // the store no longer opens (a data file is short), so its record must not be put back
    assert!(!std::path::Path::new(&store).join("units.json").exists(), "no record next to a damaged store");
}
