//! G2: command forms generated from the input schemas, live preview, and the feature history
//! panel, driven by clicks the way a user would.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::{Value, json};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::command_panel::PreviewStatus;

fn harness() -> Harness<'static, SurfaceCraftApp> {
    Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), SurfaceCraftApp::headless())
}

fn params(h: &Harness<'static, SurfaceCraftApp>) -> Value {
    serde_json::from_str(&h.state().palette_params).unwrap()
}

fn settle(h: &mut Harness<'static, SurfaceCraftApp>) {
    for _ in 0..3 {
        h.run();
    }
}

/// A fully constrained 40 × 20 plate sketch and its extrusion.
fn plate(h: &mut Harness<'static, SurfaceCraftApp>) {
    let app = h.state_mut();
    app.run_command("sketch.create", json!({"name": "base"})).unwrap();
    app.run_command(
        "sketch.add",
        json!({"sketch": "base", "entities": [{"polygon": {"points": [[0.0, 0.0], [40.0, 0.0], [40.0, 20.0], [0.0, 20.0]]}}]}),
    )
    .unwrap();
    app.run_command("feature.extrude", json!({"sketch": "base", "distance": 10.0, "name": "plate"})).unwrap();
}

/// The 3-2-1 form is the longest everyday form: in a 1440 x 900 window all of it is in view at
/// once, so nobody has to scroll to find the third datum or the optional settings.
#[test]
fn the_3_2_1_form_fits_the_tool_panel_without_scrolling() {
    let mut h = Harness::builder()
        .with_size((1440.0, 900.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), SurfaceCraftApp::headless());
    h.state_mut().run_command("mesh.synth", json!({"shape": "bracket"})).unwrap();
    h.state_mut().choose_command("align.datum");
    settle(&mut h);
    let ok_top = h.get_by_label("OK").rect().top();
    let picks: Vec<egui::Rect> = h.get_all_by_label("Pick in viewport").map(|n| n.rect()).collect();
    assert_eq!(picks.len(), 3, "one pick button per datum");
    for r in &picks {
        assert!(r.bottom() <= ok_top, "a pick button at {r:?} is below the form's visible part ({ok_top})");
    }
    // nothing of a datum sticks out of the panel on the right
    let panel_right = h.get_by_label("OK").rect().right();
    for r in h.get_all_by_label("Edit as text").map(|n| n.rect()) {
        assert!(r.right() <= panel_right + 1.0, "\"Edit as text\" at {r:?} sticks out of the panel ({panel_right})");
    }
    let last = h.get_by_label("Move the scan").rect();
    assert!(last.bottom() <= ok_top, "the last optional field at {last:?} is out of view ({ok_top})");
}

/// After a tool fails, what went wrong is next to OK and Cancel, in view however long the form
/// is, not under the last field where it has to be scrolled to.
#[test]
fn the_failure_of_a_tool_is_shown_next_to_its_buttons() {
    let mut h = Harness::builder()
        .with_size((1440.0, 900.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), SurfaceCraftApp::headless());
    h.state_mut().run_command("mesh.synth", json!({"shape": "bracket"})).unwrap();
    h.state_mut().choose_command("align.datum");
    settle(&mut h);
    // a datum that is no datum: the engine refuses the parameters
    h.state_mut().palette_params = json!({"object": "sample", "primary": 1, "secondary": 2, "tertiary": 3}).to_string();
    assert!(!h.state_mut().run_palette());
    settle(&mut h);
    // in words, not as the JSON the log keeps
    assert!(h.query_by_label_contains("\"kind\"").is_none(), "the failure is shown as raw JSON");
    h.get_by_label_contains("Could not finish: invalid parameters");
    let failure = h.get_by_label_contains("Could not finish").rect();
    let ok = h.get_by_label("OK").rect();
    assert!(failure.bottom() <= ok.top(), "the failure at {failure:?} is not above the buttons at {ok:?}");
    assert!(ok.top() - failure.bottom() < 40.0, "the failure at {failure:?} is far from the buttons at {ok:?}");
}

#[test]
fn a_form_is_generated_for_each_command() {
    let mut h = harness();
    // a plain command: enum, optional numbers and strings
    h.state_mut().choose_command("mesh.synth");
    settle(&mut h);
    h.get_by_label("Shape *");
    h.get_by_label("Resolution");
    h.get_by_label("Noise");
    // a command with a nested object of optional settings, and an object reference
    h.state_mut().choose_command("region.auto_segment");
    settle(&mut h);
    h.get_by_label("Scan *");
    h.get_by_label("Options");
    h.get_by_label("Replace");
    // a feature: object picker for the sketch, a number, the mode enum
    h.state_mut().choose_command("feature.extrude");
    settle(&mut h);
    h.get_by_label("Sketch *");
    h.get_by_label("Distance *");
    h.get_by_label("Mode");
    // the JSON toggle is still there for power users
    h.get_by_label("JSON").click();
    settle(&mut h);
    assert!(h.state().cmd_panel.json_mode);
    h.get_by_label("Parameters (JSON)");
}

#[test]
fn editing_a_field_changes_the_produced_json() {
    let mut h = harness();
    h.state_mut().choose_command("mesh.synth");
    settle(&mut h);
    assert!(params(&h).get("noise").is_none());
    // an optional parameter is set with its check box ...
    h.get_by_role_and_label(egui::accesskit::Role::CheckBox, "Noise").click();
    settle(&mut h);
    assert!(params(&h).get("noise").is_some(), "{}", h.state().palette_params);
    // ... and unset again
    h.get_by_role_and_label(egui::accesskit::Role::CheckBox, "Noise").click();
    settle(&mut h);
    assert!(params(&h).get("noise").is_none());
    // a number field: click, type, Enter
    h.get_by_label("Resolution").click();
    settle(&mut h);
    let field = h.get_by_role_and_label(egui::accesskit::Role::SpinButton, "Resolution");
    field.click();
    settle(&mut h);
    h.get_by_role_and_label(egui::accesskit::Role::SpinButton, "Resolution").type_text("48");
    h.key_press(egui::Key::Enter);
    settle(&mut h);
    assert_eq!(params(&h)["resolution"], 48, "{}", h.state().palette_params);
    // the JSON text and the form are one state: editing the text shows in the form
    h.state_mut().palette_params = json!({"shape": "sphere", "resolution": 16}).to_string();
    settle(&mut h);
    assert_eq!(params(&h)["resolution"], 16);
}

#[test]
fn an_invalid_value_shows_a_validation_message() {
    let mut h = harness();
    h.state_mut().choose_command("mesh.synth");
    settle(&mut h);
    assert!(h.query_by_label_contains("Must be at least").is_none());
    h.state_mut().palette_params = json!({"shape": "sphere", "resolution": -3}).to_string();
    settle(&mut h);
    h.get_by_label_contains("Must be at least");
    h.get_by_label_contains("need attention");
    h.state_mut().palette_params = json!({"shape": "blob", "resolution": 8}).to_string();
    settle(&mut h);
    h.get_by_label_contains("allowed values");
    // an object that does not exist
    h.state_mut().choose_command("region.auto_segment");
    h.state_mut().palette_params = json!({"mesh": 99}).to_string();
    settle(&mut h);
    h.get_by_label_contains("No object 99");
}

#[test]
fn preview_then_cancel_leaves_the_document_unchanged() {
    let mut h = harness();
    h.state_mut().choose_command("body.box");
    h.state_mut().palette_params = json!({"size": [20.0, 10.0, 5.0]}).to_string();
    settle(&mut h);
    let before =
        (h.state().session.doc.summaries(), h.state().session.log.len(), h.state().session.history.undo_labels().len());
    h.get_by_label("Preview").click();
    settle(&mut h);
    let app = h.state();
    assert!(matches!(app.cmd_panel.status, PreviewStatus::Ready(1)), "{:?}", app.cmd_panel.status);
    assert_eq!(app.cmd_panel.items.len(), 1, "the box is drawn as a preview item");
    assert_eq!(app.session.doc.summaries(), before.0, "the document does not have it yet");
    assert_eq!(app.session.log.len(), before.1, "the preview is not a logged command");
    h.get_by_label("Cancel").click();
    settle(&mut h);
    let app = h.state();
    assert!(app.cmd_panel.items.is_empty() && !app.cmd_panel.preview_on);
    assert_eq!(app.palette_command, None, "Cancel closes the tool");
    assert_eq!((app.session.doc.summaries(), app.session.log.len(), app.session.history.undo_labels().len()), before);
    // open the tool again, preview, then OK commits and closes the tool
    h.state_mut().choose_command("body.box");
    h.state_mut().palette_params = json!({"size": [20.0, 10.0, 5.0]}).to_string();
    settle(&mut h);
    h.get_by_label("Preview").click();
    settle(&mut h);
    h.get_by_label("OK").click();
    settle(&mut h);
    let app = h.state();
    assert_eq!(app.session.doc.bodies.len(), 1, "{}", app.status);
    assert!(app.cmd_panel.items.is_empty());
    assert_eq!(app.palette_command, None, "a tool that changed the document closes");
}

#[test]
fn a_heavy_command_has_no_live_preview() {
    let mut h = harness();
    h.state_mut().choose_command("region.auto_segment");
    settle(&mut h);
    assert!(surfacecraft_ui_egui::command_panel::no_preview("region.auto_segment").is_some());
    assert!(h.get_all_by_label("Preview").next().is_some(), "the toggle is shown, disabled");
    assert!(!h.state().cmd_panel.preview_on);
}

#[test]
fn the_history_panel_suppresses_edits_and_shows_failures() {
    let mut h = harness();
    plate(&mut h);
    settle(&mut h);
    assert_eq!(h.state().session.doc.features.len(), 1);
    h.get_by_label_contains("extrude (extrude)").click();
    settle(&mut h);
    assert!(h.state().session.doc.features[0].suppressed, "{}", h.state().status);
    h.get_by_label_contains("extrude (extrude)").click();
    settle(&mut h);
    assert!(!h.state().session.doc.features[0].suppressed);
    // the same form edits the feature's parameters
    h.get_all_by_label("Edit").last().unwrap().click();
    settle(&mut h);
    h.get_by_label_contains("Editing extrude");
    h.get_by_role_and_label(egui::accesskit::Role::SpinButton, "Distance *").click();
    settle(&mut h);
    h.get_by_role_and_label(egui::accesskit::Role::SpinButton, "Distance *").type_text("15");
    h.key_press(egui::Key::Enter);
    settle(&mut h);
    h.get_by_label("Apply").click();
    settle(&mut h);
    let v = h.state().session.doc.features[0].op.clone();
    let v = serde_json::to_value(v).unwrap();
    assert_eq!(v["distance"], 15.0, "{}", h.state().status);
    // a failed feature shows its error
    let id = h.state().session.doc.features[0].id;
    if let Some(f) = h.state_mut().session.doc.feature_mut(id) {
        f.error = Some("boom".into());
    }
    settle(&mut h);
    h.get_by_label_contains("boom");
}
