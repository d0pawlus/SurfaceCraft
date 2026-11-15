//! Auto-align in the window (no GPU): the tool is the first of the Align stage, its first press
//! only proposes a pose (the scan is drawn in that pose, the document is untouched), the second
//! press applies it as one undo step, and Cancel or an edited form drops the proposal.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::{Value, json};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::workflow::{Entry, Stage};

const SIZE: [u32; 2] = [800, 600];
/// The synthetic scan lies tilted by this much before it is aligned, degrees.
const TILT_DEGREES: f64 = 25.0;

/// An app with a box-shaped scan turned out of the axes and moved away from the origin, and the
/// auto-align tool open on it.
fn tilted_box() -> SurfaceCraftApp {
    let mut app = SurfaceCraftApp::headless();
    app.run_command(
        "mesh.synth",
        json!({"shape": "box", "size": [60.0, 30.0, 10.0], "resolution": 24, "name": "scan"}),
    )
    .unwrap();
    let id = app.session.doc.meshes[0].id;
    app.run_command(
        "mesh.transform",
        json!({"object": id.0, "rotate": {"axis": [1.0, 2.0, 0.5], "degrees": TILT_DEGREES}, "translate": [40.0, -15.0, 22.0]}),
    )
    .unwrap();
    app.selected_object = Some(id);
    app.choose_command("align.auto");
    app
}

fn harness(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    Harness::builder().with_size((1600.0, 1000.0)).build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app)
}

fn positions(app: &SurfaceCraftApp) -> Vec<[u64; 3]> {
    app.session.doc.meshes[0].mesh.positions.iter().map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect()
}

fn undo_steps(app: &SurfaceCraftApp) -> usize {
    app.session.history.undo_labels().len()
}

#[test]
fn align_stage_offers_auto_align_first_with_its_own_icon() {
    let first = Stage::Align.entries().first();

    assert!(matches!(first, Some(Entry::Cmd("align.auto"))), "{first:?}");
    let icon = surfacecraft_ui_egui::workflow::icon_of("align.auto");
    assert_eq!(icon, "align-auto");
    assert!(surfacecraft_ui_egui::icons::NAMES.contains(&icon));
}

#[test]
fn first_press_proposes_and_draws_the_scan_in_the_proposed_pose() {
    let mut h = harness(tilted_box());
    h.run();
    let before = positions(h.state());
    let steps = undo_steps(h.state());
    assert!(h.state_mut().viewport_frame(SIZE).overlay.models.is_empty(), "nothing is proposed yet");

    h.get_by_label("Propose").click();
    h.run();

    let app = h.state_mut();
    let last = app.session.log.last().unwrap().clone();
    assert_eq!(last.command, "align.auto");
    assert!(last.ok, "{}", last.outcome);
    assert_eq!(last.outcome["applied"], false);
    assert_eq!(positions(app), before, "a proposal does not move the scan");
    assert_eq!(undo_steps(app), steps, "a proposal is not an undo step");
    assert_eq!(app.palette_command, Some("align.auto"), "the tool stays open on its proposal");
    let frame = app.viewport_frame(SIZE);
    let key = frame.items[0].key;
    let matrix: [[f64; 4]; 4] = serde_json::from_value(last.outcome["matrix"].clone()).unwrap();
    assert_eq!(frame.overlay.models, vec![(key, matrix)], "the scan is drawn in the proposed pose");
    h.get_by_label("Apply");
}

#[test]
fn second_press_applies_the_proposal_as_one_undo_step() {
    let mut h = harness(tilted_box());
    h.run();
    let before = positions(h.state());
    let steps = undo_steps(h.state());
    h.get_by_label("Propose").click();
    h.run();

    h.get_by_label("Apply").click();
    h.run();

    let app = h.state_mut();
    let last = app.session.log.last().unwrap().clone();
    assert_eq!(last.outcome["applied"], true, "{}", last.outcome);
    assert_eq!(undo_steps(app), steps + 1, "one undo step");
    assert_eq!(app.palette_command, None, "the tool closes once the scan moved");
    assert!(app.viewport_frame(SIZE).overlay.models.is_empty(), "the scan is where it is drawn");
    let bbox = app.session.doc.meshes[0].mesh.bbox();
    assert!(bbox.min.z.abs() < 1e-6, "the part lies on z = 0: {}", bbox.min.z);
    let (dx, dy, dz) = (bbox.max.x - bbox.min.x, bbox.max.y - bbox.min.y, bbox.max.z - bbox.min.z);
    assert!(dx > dy && dy > dz, "the long side is along X: {dx} {dy} {dz}");
    app.run_command("edit.undo", Value::Null).unwrap();
    assert_eq!(positions(app), before, "undo brings the scan back");
}

#[test]
fn cancel_drops_the_proposal_and_leaves_the_scan() {
    let mut h = harness(tilted_box());
    h.run();
    let before = positions(h.state());
    h.get_by_label("Propose").click();
    h.run();

    h.get_by_label("Cancel").click();
    h.run();

    let app = h.state_mut();
    assert_eq!(app.palette_command, None);
    assert!(app.viewport_frame(SIZE).overlay.models.is_empty(), "no pose is previewed");
    assert_eq!(positions(app), before);
}

#[test]
fn editing_the_form_after_a_proposal_asks_for_a_new_one() {
    let mut h = harness(tilted_box());
    h.run();
    h.get_by_label("Propose").click();
    h.run();
    let id = h.state().session.doc.meshes[0].id.0;

    h.state_mut().palette_params = json!({"object": id, "mode": "all_faces"}).to_string();
    h.run();

    assert!(h.state_mut().viewport_frame(SIZE).overlay.models.is_empty(), "the proposal was for other settings");
    h.get_by_label("Propose");
}

#[test]
fn a_guess_on_an_organic_scan_is_marked_as_one() {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("mesh.synth", json!({"shape": "sphere", "size": [20.0], "resolution": 32, "name": "scan"}))
        .unwrap();
    app.selected_object = Some(app.session.doc.meshes[0].id);
    app.choose_command("align.auto");
    let mut h = harness(app);
    h.run();

    h.get_by_label("Propose").click();
    h.run();

    let last = h.state().session.log.last().unwrap().clone();
    assert_eq!(last.outcome["confident"], false, "{}", last.outcome);
    h.get_by_label_contains("Low confidence");
}

#[test]
fn a_proposal_brings_the_proposed_pose_into_view() {
    let mut h = harness(tilted_box());
    h.run();
    let far = h.state().camera.target;

    h.get_by_label("Propose").click();
    h.run();

    let target = h.state().camera.target;
    assert!((target - far).norm() > 10.0, "the camera followed the proposal: {far:?} -> {target:?}");
    assert!(target.z > 0.0 && target.z < 10.0, "it looks at the part lying on z = 0: {target:?}");
}

#[test]
fn reopening_the_tool_after_cancel_starts_without_the_old_proposal() {
    let mut h = harness(tilted_box());
    h.run();
    h.get_by_label("Propose").click();
    h.run();
    h.get_by_label("Cancel").click();
    h.run();

    h.state_mut().choose_command("align.auto");
    h.run();

    assert!(h.state_mut().viewport_frame(SIZE).overlay.models.is_empty(), "the cancelled proposal is gone");
    h.get_by_label("Propose");
}

#[test]
fn a_mesh_named_in_also_is_previewed_with_the_scan() {
    let mut app = tilted_box();
    app.run_command("mesh.synth", json!({"shape": "sphere", "size": [5.0], "resolution": 12, "name": "fixture"}))
        .unwrap();
    let scan = app.session.doc.meshes[0].id.0;
    app.choose_command("align.auto");
    app.palette_params = json!({"object": scan, "also": ["fixture"]}).to_string();
    let mut h = harness(app);
    h.run();

    h.get_by_label("Propose").click();
    h.run();

    let moved = h.state_mut().viewport_frame(SIZE).overlay.models.len();
    assert_eq!(moved, 2, "the scan and the fixture are both drawn in the proposed pose");
}

#[test]
fn a_failed_apply_leaves_the_form_proposing() {
    let mut h = harness(tilted_box());
    h.run();
    let scan = h.state().session.doc.meshes[0].id.0;
    let before = positions(h.state());
    h.state_mut().palette_params = json!({"object": scan, "also": ["no such mesh"]}).to_string();
    h.run();
    h.get_by_label("Propose").click();
    h.run();

    h.get_by_label("Apply").click();
    h.run();

    let app = h.state();
    assert!(!app.session.log.last().unwrap().ok, "applying to a missing mesh fails");
    assert_eq!(positions(app), before, "nothing moved");
    assert_eq!(app.palette_command, Some("align.auto"), "the tool stays open on the failure");
    h.get_by_label("Propose");
}
