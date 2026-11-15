//! A large scan picked for import is not imported at once: the app offers to reduce it while it is
//! read, and the answer decides what `mesh.import` is run with.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::import_reduce::{DEFAULT_REDUCE_TO, ImportChoice, OFFER_ABOVE, grouped};

/// Triangles the header of [`large_header`] declares.
const DECLARED: u32 = 12_000_000;

/// The header of a binary STL that declares [`DECLARED`] triangles (no triangle follows: what the
/// app does before it reads the file depends on the header alone).
fn large_header() -> Vec<u8> {
    let mut bytes = vec![0u8; 80];
    bytes.extend_from_slice(&DECLARED.to_le_bytes());
    bytes
}

/// A small sphere as an STL file.
fn small_file() -> Vec<u8> {
    let mut s = surfacecraft_engine::Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "resolution": 24, "name": "ball"})).unwrap();
    let out = s.execute("mesh.export", json!({"object": "ball", "format": "stl"})).unwrap();
    surfacecraft_doc::base64::decode(out["data"].as_str().unwrap()).unwrap()
}

#[test]
fn a_small_file_is_imported_without_a_question() {
    let mut app = SurfaceCraftApp::headless();
    app.import_picked_bytes("ball.stl", small_file());
    assert_eq!(app.import_offer(), None);
    assert_eq!(app.session.doc.meshes.len(), 1);
    let entry = app.session.log.last().unwrap();
    assert_eq!(entry.params["name"], "ball");
    assert!(entry.params.get("reduce_to").is_none());
}

#[test]
fn a_large_file_waits_for_the_answer_and_cancel_drops_it() {
    assert!(u64::from(DECLARED) > OFFER_ABOVE);
    let mut app = SurfaceCraftApp::headless();
    app.import_picked_bytes("scan.stl", large_header());
    let offer = app.import_offer().expect("the app asks first");
    assert_eq!(offer.triangles, u64::from(DECLARED));
    assert_eq!(offer.reduce_to, DEFAULT_REDUCE_TO);
    assert!(app.session.log.is_empty(), "nothing ran yet");
    app.answer_import_offer(ImportChoice::Cancel);
    assert_eq!(app.import_offer(), None);
    assert!(app.session.log.is_empty(), "a cancelled offer runs nothing");
}

#[test]
fn a_file_picked_while_another_waits_does_not_replace_it() {
    let mut app = SurfaceCraftApp::headless();
    app.import_picked_bytes("scan.stl", large_header());
    app.import_picked_bytes("ball.stl", small_file());
    assert_eq!(app.import_offer().map(|o| o.triangles), Some(u64::from(DECLARED)));
    assert!(app.session.log.is_empty(), "the second file was not imported behind the dialog");
    app.answer_import_offer(ImportChoice::Cancel);
    app.import_picked_bytes("ball.stl", small_file());
    assert_eq!(app.session.doc.meshes.len(), 1);
}

#[test]
fn the_answer_decides_how_the_import_runs() {
    for (choice, reduce_to) in [(ImportChoice::Reduce(750_000), Some(750_000)), (ImportChoice::Full, None)] {
        let mut app = SurfaceCraftApp::headless();
        app.import_picked_bytes("scan.stl", large_header());
        app.answer_import_offer(choice);
        assert_eq!(app.import_offer(), None);
        let entry = app.session.log.last().expect("the import ran");
        assert_eq!(entry.command, "mesh.import");
        assert_eq!(entry.params.get("reduce_to").and_then(serde_json::Value::as_u64), reduce_to, "{choice:?}");
        assert_eq!(entry.params["name"], "scan");
        // the header promised triangles the file does not hold: an error, and no mesh
        assert!(!entry.ok);
        assert!(app.session.doc.meshes.is_empty());
    }
}

#[test]
fn a_reduction_out_of_range_is_brought_into_it() {
    let mut app = SurfaceCraftApp::headless();
    app.import_picked_bytes("scan.stl", large_header());
    app.answer_import_offer(ImportChoice::Reduce(3));
    let entry = app.session.log.last().unwrap();
    assert_eq!(entry.params["reduce_to"], 1000);
}

#[test]
fn the_dialog_says_how_large_the_scan_is_and_offers_the_reduction() {
    let mut app = SurfaceCraftApp::headless();
    app.import_picked_bytes("scan.stl", large_header());
    let mut h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.run();
    h.get_by_label("Large scan");
    h.get_by_label_contains(&grouped(u64::from(DECLARED)));
    h.get_by_label("Reduce and import");
    h.get_by_label("Import in full");
    h.get_by_label("Cancel").click();
    h.run();
    assert_eq!(h.state().import_offer(), None);
    assert!(h.query_by_label("Large scan").is_none());
}

#[test]
fn a_reduced_import_says_what_it_did_on_the_status_line() {
    let note = surfacecraft_ui_egui::import_reduce::result_note(&json!({
        "triangles": 1_999_000, "reduced": {"source_triangles": 12_000_000, "max_error": 0.0421}
    }))
    .unwrap();
    assert!(
        note.contains("12\u{a0}000\u{a0}000") && note.contains("1\u{a0}999\u{a0}000") && note.contains("0.042"),
        "{note}"
    );
    assert_eq!(surfacecraft_ui_egui::import_reduce::result_note(&json!({"triangles": 5})), None);
}
