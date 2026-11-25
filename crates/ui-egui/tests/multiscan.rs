//! The "Many scans" wizard through the app (no GPU): the commands it calls are the engine's own,
//! on synthetic scans only (copies of the synthetic bracket, each moved to a pose of its own).
//! The wizard's state machine is tested with canned outcomes in `src/multiscan/tests.rs`; here the
//! real commands answer.
//!
//! Bounds are fixed before the first run and lie well away from the measured values (the
//! measurements are printed, run with `--nocapture`).
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use surfacecraft_doc::ObjectId;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::multiscan::{Action, GlobalState, MergeState, PairState, Step};

/// Where the copies start from: a rotation (axis, degrees) and a shift (mm) each.
const POSES: [([f64; 3], f64, [f64; 3]); 3] = [
    ([0.0, 0.0, 1.0], 0.0, [0.0, 0.0, 0.0]),
    ([1.0, 2.0, 0.5], 31.0, [55.0, -20.0, 12.0]),
    ([0.3, -1.0, 1.0], -47.0, [-40.0, 35.0, -18.0]),
];

/// An app with `n` copies of the bracket, `scan0`, `scan1`, ..., each in its own pose.
fn app_with_scans(n: usize) -> (SurfaceCraftApp, Vec<ObjectId>) {
    let mut app = SurfaceCraftApp::headless();
    let mut ids = Vec::new();
    for (k, (axis, degrees, shift)) in POSES.iter().take(n).enumerate() {
        let name = format!("scan{k}");
        app.run_command("mesh.synth", json!({"shape": "bracket", "noise": 0.02, "seed": 7 + k, "name": name})).unwrap();
        let id = app.session.doc.meshes.last().map(|m| m.id).unwrap();
        let motion = json!({"object": id.0, "rotate": {"axis": axis, "degrees": degrees}, "translate": shift});
        app.run_command("mesh.transform", motion).unwrap();
        ids.push(id);
    }
    (app, ids)
}

fn undo_depth(app: &SurfaceCraftApp) -> usize {
    app.session.history.undo_labels().len()
}

/// Every mesh's vertex positions, bit for bit: the document as far as the wizard can touch it.
fn snapshot(app: &SurfaceCraftApp) -> Vec<(u64, Vec<[u64; 3]>)> {
    let bits = |m: &surfacecraft_doc::MeshObject| {
        m.mesh.positions.iter().map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect()
    };
    app.session.doc.meshes.iter().map(|m| (m.id.0, bits(m))).collect()
}

fn wizard_step(app: &SurfaceCraftApp) -> Step {
    app.wizard().map(|w| w.step()).unwrap()
}

/// Accepts every pair the command proposes a pose for, in order.
fn accept_the_pairs(app: &mut SurfaceCraftApp) {
    while let Some((_, pair)) = app.wizard().unwrap().current_pair() {
        let acceptable = matches!(&pair.state, PairState::Proposed(p) if p.acceptable());
        assert!(acceptable, "a pose is proposed for every pair: {:?}", pair.state);
        app.wizard_act(Action::AcceptPair);
    }
}

/// The largest distance between the corners of the two scans' boxes, mm.
fn corner_gap(app: &SurfaceCraftApp, a: ObjectId, b: ObjectId) -> f64 {
    let (a, b) = (app.session.doc.mesh(a).unwrap().mesh.bbox(), app.session.doc.mesh(b).unwrap().mesh.bbox());
    [(a.min - b.min).norm(), (a.max - b.max).norm()].into_iter().fold(0.0, f64::max)
}

/// The wizard up to the merge step: pairs accepted, global alignment applied (the weakly held
/// scans, if any, accepted).
fn to_the_merge_step(app: &mut SurfaceCraftApp) {
    app.open_multiscan();
    app.wizard_act(Action::Next);
    accept_the_pairs(app);
    app.wizard_act(Action::Next);
    let weak: Vec<usize> = match app.wizard().unwrap().global() {
        GlobalState::Proposed(r) => r.scans.iter().filter(|s| s.weakly_held()).map(|s| s.index).collect(),
        other => panic!("the global poses are proposed: {other:?}"),
    };
    for index in weak {
        app.wizard_act(Action::ToggleWeak(index));
    }
    app.wizard_act(Action::ApplyGlobal);
    assert!(matches!(app.wizard().unwrap().global(), GlobalState::Applied(_)), "{:?}", app.wizard().unwrap().problem());
    app.wizard_act(Action::Next);
    assert_eq!(wizard_step(app), Step::Merge);
}

#[test]
fn three_scans_are_aligned_and_merged_into_one_mesh() {
    let (mut app, ids) = app_with_scans(3);
    let depth = undo_depth(&app);

    to_the_merge_step(&mut app);
    app.wizard_act(Action::Merge);

    let MergeState::Done(report) = app.wizard().unwrap().merge().clone() else {
        panic!("{:?} {:?}", app.wizard().unwrap().merge(), app.wizard().unwrap().problem())
    };
    assert!(!report.refused);
    let merged = ObjectId(report.id.unwrap());
    let gaps: Vec<f64> = ids.iter().map(|id| corner_gap(&app, ids[0], *id)).collect();
    let merged_gap = corner_gap(&app, ids[0], merged);
    eprintln!("scan gaps {gaps:?} mm, merged gap {merged_gap:.4} mm, {} triangles", report.triangles);
    assert!(gaps.iter().all(|g| *g <= 1.0), "the scans lie in one frame: {gaps:?}");
    assert!(merged_gap <= 1.0, "the merged mesh is the part: {merged_gap}");
    assert!(report.triangles > 100);
    assert_eq!(app.session.doc.meshes.len(), 4, "the scans stay and the merged mesh is new");
    assert_eq!(undo_depth(&app), depth + 2 + 1 + 1, "two pairs, the global poses, the merge");

    app.wizard_act(Action::Finish);

    assert!(app.wizard().is_none(), "the wizard closes");
    assert_eq!(app.selected_object, Some(merged));
    assert_eq!(undo_depth(&app), depth + 4, "finishing adds nothing");
}

#[test]
fn cancel_at_every_step_leaves_the_document_as_it_was() {
    let (mut app, _) = app_with_scans(3);
    let (depth, before) = (undo_depth(&app), snapshot(&app));

    app.open_multiscan();
    app.wizard_act(Action::Cancel);
    assert!(app.wizard().is_none());
    assert_eq!((undo_depth(&app), snapshot(&app)), (depth, before.clone()), "at the first step");

    app.open_multiscan();
    app.wizard_act(Action::Next);
    app.wizard_act(Action::AcceptPair);
    assert_eq!(undo_depth(&app), depth + 1, "an accepted pair is one undo step");
    app.wizard_act(Action::Cancel);
    assert_eq!((undo_depth(&app), snapshot(&app)), (depth, before.clone()), "in the pairs");

    to_the_merge_step(&mut app);
    app.wizard_act(Action::Merge);
    assert_eq!(app.session.doc.meshes.len(), 4);
    app.wizard_act(Action::Cancel);
    assert!(app.wizard().is_none());
    assert_eq!(app.session.doc.meshes.len(), 3, "the merged mesh is gone");
    assert_eq!((undo_depth(&app), snapshot(&app)), (depth, before), "after the merge");
}

#[test]
fn going_back_undoes_the_step_left_and_nothing_else() {
    let (mut app, _) = app_with_scans(3);
    let depth = undo_depth(&app);
    to_the_merge_step(&mut app);
    let before_merge = snapshot(&app);
    app.wizard_act(Action::Merge);
    assert_eq!(app.session.doc.meshes.len(), 4);

    app.wizard_act(Action::Back);
    assert_eq!((wizard_step(&app), app.session.doc.meshes.len()), (Step::Global, 3), "the merge is undone");
    assert_eq!(snapshot(&app), before_merge, "the scans stay as the global alignment left them");
    assert_eq!(undo_depth(&app), depth + 3);

    app.wizard_act(Action::Back);
    assert_eq!(wizard_step(&app), Step::Pairs);
    assert_eq!(undo_depth(&app), depth + 2, "the global poses are undone");
    assert!(matches!(app.wizard().unwrap().global(), GlobalState::Idle));

    app.wizard_act(Action::Back);
    assert_eq!(wizard_step(&app), Step::Pick);
    assert_eq!(undo_depth(&app), depth, "both pairs are undone");
}

#[test]
fn a_change_made_behind_the_wizard_blocks_its_undo() {
    let (mut app, ids) = app_with_scans(2);
    app.open_multiscan();
    app.wizard_act(Action::Next);
    app.wizard_act(Action::AcceptPair);
    app.run_command("mesh.transform", json!({"object": ids[0].0, "translate": [1.0, 0.0, 0.0]})).unwrap();
    let (depth, before) = (undo_depth(&app), snapshot(&app));

    app.wizard_act(Action::Cancel);

    assert!(app.wizard().is_some(), "the wizard stays open");
    assert_eq!((undo_depth(&app), snapshot(&app)), (depth, before), "nothing was undone");
    app.wizard_act(Action::Leave);
    assert!(app.wizard().is_none());
}

#[test]
fn scans_that_overlap_but_disagree_are_refused_and_can_be_merged_anyway() {
    let (mut app, ids) = app_with_scans(2);
    // the second copy back on the first, then shifted by a few millimetres: they overlap, but
    // disagree where they do
    app.open_multiscan();
    app.wizard_act(Action::Next);
    app.wizard_act(Action::AcceptPair);
    let shift = json!({"object": ids[1].0, "translate": [2.5, 1.5, 0.0]});
    app.wizard_act(Action::Leave);
    app.run_command("mesh.transform", shift).unwrap();
    app.open_multiscan();
    app.wizard_act(Action::Next);
    app.wizard_act(Action::RejectPair);
    app.wizard_act(Action::Next);
    app.wizard_act(Action::Next);
    assert_eq!(wizard_step(&app), Step::Merge);
    let depth = undo_depth(&app);

    app.wizard_act(Action::Merge);

    let MergeState::Refused(report) = app.wizard().unwrap().merge().clone() else {
        panic!("{:?}", app.wizard().unwrap().merge())
    };
    assert!(report.refused && report.id.is_none());
    assert_eq!(undo_depth(&app), depth, "a refusal is no undo step");
    assert_eq!(app.session.doc.meshes.len(), 2, "nothing was made");
    eprintln!("refusals: {:?}", report.refusals().map(|i| i.code.clone()).collect::<Vec<_>>());
    assert!(report.warn_would_help(), "{:?}", report.issues);

    app.wizard_act(Action::SetWarn(true));
    app.wizard_act(Action::Merge);

    assert!(matches!(app.wizard().unwrap().merge(), MergeState::Done(_)), "{:?}", app.wizard().unwrap().merge());
    assert_eq!(app.session.doc.meshes.len(), 3);
}

#[test]
fn a_failed_command_keeps_the_wizard_open_and_shows_its_error() {
    let (mut app, ids) = app_with_scans(1);
    let empty = app.session.doc.add_mesh("empty", Default::default());
    app.open_multiscan();
    assert_eq!(app.wizard().unwrap().scans(), [ids[0], empty], "a document of two meshes starts with both chosen");

    app.wizard_act(Action::Next);

    assert_eq!(wizard_step(&app), Step::Pairs);
    let state = app.wizard().unwrap().pairs()[0].state.clone();
    let PairState::Failed(message) = state else { panic!("{state:?}") };
    assert!(!message.is_empty());
    let mut h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.run();
    assert!(h.query_all_by_label_contains(&message).next().is_some(), "the error is on screen: {message}");
    h.get_by_label("Try again");
    h.get_by_label("Reject");
}

#[test]
fn the_panel_walks_the_pairs_with_clicks() {
    let (app, _) = app_with_scans(2);
    let mut h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);
    h.state_mut().open_multiscan();
    h.run();
    h.get_by_label("Choose the scans");
    h.get_by_label_contains("1. scan0");

    h.get_by_label("Next").click();
    h.run();

    h.get_by_label("Accept");
    assert_eq!(h.state().wizard().unwrap().step(), Step::Pairs);
    assert!(!h.state_mut().viewport_frame([800, 600]).overlay.models.is_empty(), "the proposed pose is drawn");
    h.get_by_label("Accept").click();
    h.run();
    h.get_by_label_contains("Every pair is decided");
    assert!(h.state_mut().viewport_frame([800, 600]).overlay.models.is_empty(), "nothing is previewed once applied");
}

#[test]
fn the_wizard_is_closed_when_a_scan_it_works_on_is_deleted() {
    let (mut app, ids) = app_with_scans(2);
    app.open_multiscan();
    app.run_command("doc.delete", json!({"object": ids[0].0})).unwrap();
    let mut h = Harness::builder()
        .with_size((1600.0, 1000.0))
        .build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app);

    h.run();

    assert!(h.state().wizard().is_none(), "the wizard has nothing to work on");
}

/// Lets a background command finish: the window polls the job every frame.
fn wait_for_the_job(app: &mut SurfaceCraftApp) {
    let started = std::time::Instant::now();
    while app.advance_job(started.elapsed().as_secs_f64()) {
        assert!(started.elapsed().as_secs() < 120, "the command did not finish");
        std::thread::sleep(std::time::Duration::from_millis(10));
    }
}

#[test]
fn a_background_proposal_comes_back_to_the_wizard_when_the_job_finishes() {
    let (mut app, _) = app_with_scans(2);
    app.background_jobs = true;
    app.open_multiscan();

    app.wizard_act(Action::Next);

    assert!(app.is_busy(), "the pair is searched for in the background");
    assert!(app.wizard().unwrap().running().is_some());
    wait_for_the_job(&mut app);
    assert!(app.wizard().unwrap().running().is_none());
    assert!(matches!(app.wizard().unwrap().pairs()[0].state, PairState::Proposed(_)));
}

#[test]
fn cancelling_the_busy_dialog_leaves_the_wizard_on_its_step_with_the_document_untouched() {
    let (mut app, _) = app_with_scans(2);
    app.background_jobs = true;
    app.open_multiscan();
    let (depth, before) = (undo_depth(&app), snapshot(&app));
    app.wizard_act(Action::Next);
    assert!(app.is_busy());

    app.cancel_job();

    assert!(!app.is_busy());
    assert_eq!(wizard_step(&app), Step::Pairs, "the wizard stays open");
    assert!(matches!(app.wizard().unwrap().pairs()[0].state, PairState::Failed(_)), "it can be tried again");
    assert!(app.wizard().unwrap().running().is_none());
    assert_eq!((undo_depth(&app), snapshot(&app)), (depth, before));
}
