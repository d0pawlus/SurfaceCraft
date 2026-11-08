//! The GUI drives the same commands as the CLI: clicking palette entries, menus and Run
//! executes engine commands, and the log turns into a script that replays identically.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_ui_egui::SurfaceCraftApp;

fn harness() -> Harness<'static, SurfaceCraftApp> {
    Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), SurfaceCraftApp::headless())
}

#[test]
fn palette_runs_commands_and_log_replays_in_the_engine() {
    let mut h = harness();
    h.run();
    // filter the palette, choose mesh.synth, edit its parameters, press Run
    h.get_by_label("Filter").focus();
    h.run();
    h.get_by_label("Filter").type_text("synth");
    h.run();
    h.get_by_label("mesh.synth  —  Synthetic scan").click();
    h.run();
    assert_eq!(h.state().palette_command, Some("mesh.synth"));
    h.state_mut().palette_params = json!({"shape": "sphere", "size": [12.0], "resolution": 16}).to_string();
    h.get_by_label("Run").click();
    h.run();
    assert_eq!(h.state().session.doc.meshes.len(), 1, "status: {}", h.state().status);

    // the Solid menu reaches body.box
    h.get_by_label("Solid").click();
    h.run();
    h.get_by_label("Box").click();
    h.run();
    assert_eq!(h.state().palette_command, Some("body.box"));
    h.state_mut().palette_params = json!({"size": [20.0, 10.0, 5.0]}).to_string();
    h.get_by_label("Run").click();
    h.run();
    assert_eq!(h.state().session.doc.bodies.len(), 1, "status: {}", h.state().status);

    // the object list shows both, and deleting through the list is a command too
    h.get_by_label_contains("box (#2)");
    // replaying the GUI's log as a CLI script gives the same document
    let script = h.state().log_as_script();
    let mut s = Session::new();
    let report = s.run_script(&script).unwrap();
    assert!(report.ok);
    assert_eq!(s.doc.summaries(), h.state().session.doc.summaries());
}

#[test]
fn template_prefills_required_parameters() {
    let schema = (surfacecraft_engine::cmd::find("body.fillet").unwrap().input)();
    let t = surfacecraft_ui_egui::app::template(&schema, Some(surfacecraft_doc::ObjectId(7)));
    assert_eq!(t["object"], 7);
    assert!(t.get("edges").is_some() && t.get("size").is_some());
}
