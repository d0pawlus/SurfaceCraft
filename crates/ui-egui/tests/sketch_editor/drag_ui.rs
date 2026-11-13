//! Tessellation of sketch drawings, and drags through the UI with a constrained rectangle.
use super::common::*;
use egui_kittest::Harness;
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_sketch::{Constraint, Entity, Sketch};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::sketch_editor::Tool;

#[test]
fn sketch_drawings_are_tessellated_once_and_again_only_when_the_sketch_changes() {
    let mut app = SurfaceCraftApp::headless();
    let a = new_sketch_on_xy(&mut app);
    app.run_command(
        "sketch.add",
        json!({"sketch": a.0, "entities": [{"line": {"from": [0.0, 0.0], "to": [10.0, 0.0]}}]}),
    )
    .unwrap();
    let other = new_sketch_on_xy(&mut app);
    app.run_command(
        "sketch.add",
        json!({"sketch": other.0, "entities": [{"line": {"from": [0.0, 0.0], "to": [4.0, 4.0]}}]}),
    )
    .unwrap();
    app.begin_sketch_edit(a);
    let mut h = harness(app);
    h.run();
    let first = h.state().sketch_cache.tessellations;
    assert_eq!(first, 2, "the edited sketch and the reference sketch, once each");
    for _ in 0..5 {
        h.run();
    }
    assert_eq!(h.state().sketch_cache.tessellations, first, "no document change, no tessellation");
    h.state_mut()
        .run_command(
            "sketch.add",
            json!({"sketch": other.0, "entities": [{"line": {"from": [1.0, 1.0], "to": [2.0, 3.0]}}]}),
        )
        .unwrap();
    h.run();
    assert_eq!(h.state().sketch_cache.tessellations, first + 1, "the changed sketch once");
    h.run();
    assert_eq!(h.state().sketch_cache.tessellations, first + 1);
    h.state_mut()
        .run_command(
            "sketch.add",
            json!({"sketch": a.0, "entities": [{"line": {"from": [0.0, 5.0], "to": [3.0, 5.0]}}]}),
        )
        .unwrap();
    h.run();
    assert_eq!(h.state().sketch_cache.tessellations, first + 2, "only the edited sketch this time");
}

/// A 10 x 5 rectangle on a sketch of the XY plane, edited and rendered.
fn rect_harness() -> (Harness<'static, SurfaceCraftApp>, ObjectId) {
    let mut app = SurfaceCraftApp::headless();
    let id = new_sketch_on_xy(&mut app);
    app.run_command("sketch.add", json!({"sketch": id.0, "entities": [{"polygon": {"points": [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0]]}}]})).unwrap();
    let mut h = harness(app);
    h.run();
    no_snaps(&mut h);
    (h, id)
}

fn point_of(h: &Harness<'static, SurfaceCraftApp>, id: ObjectId, i: usize) -> [f64; 2] {
    h.state().session.doc.sketch(id).expect("sketch").sketch.points[i]
}

#[test]
fn a_drag_whose_first_movement_is_far_still_moves_the_pressed_point() {
    let (mut h, id) = rect_harness();
    let from = screen_of(h.state(), id, [10.0, 5.0]);
    // the first reported pointer position is about 40 px away, far beyond the pick radius
    let far = from + egui::vec2(40.0, 25.0);
    h.hover_at(from);
    h.run();
    h.drag_at(from);
    h.run();
    h.hover_at(far);
    h.run();
    h.drop_at(far);
    h.run();
    let moved = point_of(&h, id, 2);
    assert!((moved[0] - 10.0).abs() > 0.5 || (moved[1] - 5.0).abs() > 0.5, "the pressed corner moved: {moved:?}");
    assert_eq!(h.state().session.log.last().map(|e| e.command.as_str()), Some("sketch.move_point"));
    assert_eq!(point_of(&h, id, 1), [10.0, 0.0], "no other point was picked");
}

#[test]
fn a_drag_passing_over_another_point_keeps_the_pressed_point() {
    let (mut h, id) = rect_harness();
    let from = screen_of(h.state(), id, [10.0, 5.0]);
    let over_other = screen_of(h.state(), id, [10.0, 0.0]);
    let end = screen_of(h.state(), id, [14.0, -3.0]);
    h.hover_at(from);
    h.run();
    h.drag_at(from);
    h.run();
    // the first movement lands exactly on a neighbouring corner
    h.hover_at(over_other);
    h.run();
    h.hover_at(end);
    h.run();
    h.drop_at(end);
    h.run();
    let p2 = point_of(&h, id, 2);
    assert!((p2[0] - 14.0).abs() < 0.1 && (p2[1] + 3.0).abs() < 0.1, "the pressed corner follows: {p2:?}");
    assert_eq!(point_of(&h, id, 1), [10.0, 0.0], "the corner passed over stays");
}

#[test]
fn undo_during_a_drag_cancels_it_and_commits_nothing() {
    let mut app = SurfaceCraftApp::headless();
    let id = new_sketch_on_xy(&mut app);
    app.run_command(
        "sketch.add",
        json!({"sketch": id.0, "entities": [{"line": {"from": [0.0, 0.0], "to": [10.0, 0.0]}}]}),
    )
    .unwrap();
    app.run_command(
        "sketch.add",
        json!({"sketch": id.0, "entities": [{"line": {"from": [20.0, 0.0], "to": [25.0, 0.0]}}]}),
    )
    .unwrap();
    let mut h = harness(app);
    h.run();
    no_snaps(&mut h);
    let from = screen_of(h.state(), id, [25.0, 0.0]);
    let mid = screen_of(h.state(), id, [27.0, 3.0]);
    let end = screen_of(h.state(), id, [30.0, 6.0]);
    h.hover_at(from);
    h.run();
    h.drag_at(from);
    h.run();
    h.hover_at(mid);
    h.run();
    assert!(h.state().sketch_editor.drag.is_some(), "the drag is under way");
    // undo while the button is held: the dragged point disappears from the sketch
    h.state_mut().run_command("edit.undo", Value::Null).unwrap();
    h.hover_at(end);
    h.run();
    assert!(h.state().sketch_editor.drag.is_none(), "the drag was cancelled");
    let after_undo = surfacecraft_doc::format::to_json(&h.state().session.doc).expect("json");
    let log = h.state().session.log.len();
    h.drop_at(end);
    h.run();
    assert_eq!(h.state().session.log.len(), log, "no command on release");
    assert_eq!(surfacecraft_doc::format::to_json(&h.state().session.doc).expect("json"), after_undo);
}

#[test]
fn a_drag_is_cancelled_when_the_sketch_differs_from_its_baseline() {
    let sk = polyline();
    let mut e = editor(Tool::Select);
    assert!(e.drag_begin(&sk, 2, [10.0, 5.0]));
    e.drag_update([12.0, 7.0]);
    e.drag_check(ObjectId(1), &sk);
    assert!(e.drag.is_some(), "the same sketch keeps the drag");
    let mut other = sk.clone();
    other.constraints.push(Constraint::Horizontal { line: 0 });
    e.drag_check(ObjectId(1), &other);
    assert!(e.drag.is_none(), "a changed sketch cancels it");
    assert!(e.drag_end().is_empty());
    assert!(e.drag_begin(&sk, 2, [10.0, 5.0]));
    e.drag_check(ObjectId(2), &sk);
    assert!(e.drag.is_none(), "another sketch cancels it");
}

#[test]
fn a_drag_onto_a_collapsed_dimensioned_line_keeps_the_last_valid_preview() {
    let mut sk = Sketch::default();
    sk.add_point([0.0, 0.0]);
    sk.add_point([10.0, 0.0]);
    sk.entities.push(Entity::Line { a: 0, b: 1 });
    sk.constraints.push(Constraint::Length { line: 0, value: 10.0 });
    let mut e = editor(Tool::Select);
    assert!(e.drag_begin(&sk, 1, [10.0, 0.0]));
    e.drag_update([4.0, 3.0]);
    let valid = e.drag.as_ref().expect("drag").preview.clone();
    e.drag_update([0.0, 0.0]);
    let preview = e.drag.as_ref().expect("drag").preview.clone();
    let (a, b) = (preview.points[0], preview.points[1]);
    assert!(((a[0] - b[0]).hypot(a[1] - b[1]) - 10.0).abs() < 1e-9, "the preview keeps the length");
    assert_ne!(preview, valid, "the preview moved on from the earlier valid state toward (0, 0)");
    let cmds = e.drag_end();
    assert_eq!(cmds.len(), 1, "the release commits the preview");
    // executing the release command on the same sketch reproduces the preview exactly
    let mut session = Session::new();
    let id = session.doc.add_sketch("s", sk);
    assert_eq!(id, ObjectId(1));
    session.execute(cmds[0].id, cmds[0].params.clone()).expect("the committed move");
    assert_eq!(session.doc.sketch(id).expect("sketch").sketch.points, preview.points);
}
#[test]
fn drag_press_picks_the_point_under_the_press_not_under_the_pointer() {
    let sk = polyline();
    let mut e = editor(Tool::Select);
    // pressed on (10, 5); the pointer is already on (10, 0) when the drag is reported
    assert!(e.drag_press(&sk, [10.0, 5.0], [10.0, 0.0], 0.5));
    assert_eq!(e.drag.as_ref().map(|d| d.point), Some(2));
    e.escape();
    assert!(!e.drag_press(&sk, [50.0, 50.0], [10.0, 0.0], 0.5), "a press on nothing starts nothing");
}
