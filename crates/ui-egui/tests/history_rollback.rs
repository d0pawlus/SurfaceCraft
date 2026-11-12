//! P8-H7: the roll-back marker in the history panel. The marker is clicked or dragged through the
//! real panel; the rolled-back features are greyed out and cannot be edited while parked.
use egui_kittest::Harness;
use egui_kittest::kittest::{NodeT, Queryable};
use serde_json::json;
use surfacecraft_doc::ObjectId;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::history_panel::is_rolled_back;

/// Extrude, cut and fillet a block; returns the fillet's feature id.
fn block_with_fillet(app: &mut SurfaceCraftApp) -> ObjectId {
    let run = |app: &mut SurfaceCraftApp, id: &str, p: serde_json::Value| {
        app.run_command(id, p).unwrap_or_else(|e| panic!("{id}: {e}"))
    };
    run(
        app,
        "sketch.create",
        json!({"name": "base", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    );
    run(
        app,
        "sketch.add",
        json!({"sketch": "base", "entities": [{"polygon": {"points": [[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]]}}]}),
    );
    run(app, "feature.extrude", json!({"sketch": "base", "distance": 10.0, "name": "block"}));
    run(
        app,
        "sketch.create",
        json!({"name": "hole", "plane": {"origin": [0.0, 0.0, 10.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    );
    run(app, "sketch.add", json!({"sketch": "hole", "entities": [{"circle": {"center": [0.0, 0.0], "radius": 3.0}}]}));
    run(app, "feature.extrude", json!({"sketch": "hole", "distance": -10.0, "mode": "cut", "target": "block"}));
    let f = run(app, "feature.fillet", json!({"body": "block", "edges": [0], "size": 0.1}));
    ObjectId(f["id"].as_u64().expect("fillet id"))
}

fn harness(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    Harness::builder().with_size((1600.0, 1000.0)).build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app)
}

#[test]
fn clicking_the_marker_issues_a_rollback_and_the_parked_rows_are_greyed() {
    let mut app = SurfaceCraftApp::headless();
    let fillet = block_with_fillet(&mut app);
    app.run_command("feature.rollback", json!({"to": {"feature": fillet.0}})).expect("roll back");
    let mut h = harness(app);
    h.run();
    assert!(is_rolled_back(h.state(), fillet), "the fillet is parked");
    assert!(h.get_by_label_contains("(fillet)").accesskit_node().is_disabled(), "the parked row is greyed out");
    let log0 = h.state().session.log.len();
    h.get_by_label_contains("Roll-back marker").click();
    h.run();
    let app = h.state();
    assert_eq!(app.session.log.last().map(|e| e.command.as_str()), Some("feature.rollback"));
    assert_eq!(app.session.log.len(), log0 + 1, "the click is one rollback");
    assert!(!is_rolled_back(app, fillet), "the marker went to the end: everything is active");
    assert_eq!(app.session.doc.features.len(), 3);
}

#[test]
fn the_roll_back_here_button_rolls_back_before_that_feature() {
    let mut app = SurfaceCraftApp::headless();
    let fillet = block_with_fillet(&mut app);
    let mut h = harness(app);
    h.run();
    h.get_all_by_label("Roll back here").last().expect("a button per active row").click();
    h.run();
    assert!(is_rolled_back(h.state(), fillet), "the fillet is parked");
    assert_eq!(h.state().session.doc.features.len(), 2);
}
