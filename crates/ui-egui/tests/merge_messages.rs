//! The refusals of `mesh.merge` at its input bounds reach the interface in Polish: the messages the
//! merge writes match their templates in `errors.pl.json`.
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_geom::merge::{MAX_SCANS, Params, merge};
use surfacecraft_geom::{Cancel, Point3, synth};
use surfacecraft_ui_egui::i18n::{Lang, engine_message, set_lang};

fn polish(message: &str) -> String {
    set_lang(Lang::Pl);
    let shown = engine_message(message);
    set_lang(Lang::En);
    shown
}

#[test]
fn a_scan_far_from_the_origin_is_refused_in_polish() {
    let mut s = Session::new();
    s.doc.add_mesh("near", synth::sphere(Point3::origin(), 10.0, 16, 8));
    s.doc.add_mesh("far", synth::sphere(Point3::new(1e10, 0.0, 0.0), 10.0, 16, 8));
    let e = s.execute("mesh.merge", json!({"objects": ["near", "far"], "tolerance": 0.1})).unwrap_err().to_string();
    let shown = polish(&e);
    assert!(shown.contains("przesuń go bliżej początku układu"), "{e} / {shown}");
}

#[test]
fn too_many_scans_are_refused_in_polish() {
    let one = synth::sphere(Point3::origin(), 1.0, 8, 4);
    let many = vec![one; MAX_SCANS + 1];
    let e = merge(&many, &Params { tolerance: 0.1, ..Params::default() }, &Cancel::new(), &mut |_| {})
        .unwrap_err()
        .to_string();
    let shown = polish(&e);
    assert!(shown.contains("naraz można scalić najwyżej"), "{e} / {shown}");
}
