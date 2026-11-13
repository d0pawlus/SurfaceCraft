//! Dragging a sketch point: the state machine and the mouse drags through the UI.
use super::common::*;
use egui_kittest::Harness;
use serde_json::json;
use surfacecraft_sketch::{Constraint, Entity, Sketch};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::sketch_editor::Tool;

#[test]
fn dragging_a_free_corner_puts_the_point_at_the_cursor() {
    let sk = polyline();
    let mut e = editor(Tool::Select);
    assert!(e.drag_begin(&sk, 2, [10.0, 5.0]));
    e.drag_update([12.25, 7.5]);
    let cmds = e.drag_end();
    assert_eq!(cmds.len(), 1, "one drag, one command");
    assert_eq!(cmds[0].id, "sketch.move_point");
    assert_eq!(cmds[0].params, json!({"sketch": 1, "point": 2, "to": [12.25, 7.5]}));
    let moved = surfacecraft_ui_egui::sketch_editor::drag::moved(&sk, 2, [12.25, 7.5]).expect("solves");
    let p = moved.points[2];
    assert!((p[0] - 12.25).abs() < 1e-6 && (p[1] - 7.5).abs() < 1e-6, "{p:?}");
    assert!(e.drag.is_none(), "the drag is over");
}

#[test]
fn dragging_a_corner_of_a_constrained_rectangle_keeps_it_a_rectangle() {
    let mut sk = Sketch::default();
    for p in [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0]] {
        sk.add_point(p);
    }
    for (a, b) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
        sk.entities.push(Entity::Line { a, b });
    }
    sk.constraints.extend([
        Constraint::Horizontal { line: 0 },
        Constraint::Horizontal { line: 2 },
        Constraint::Vertical { line: 1 },
        Constraint::Vertical { line: 3 },
        Constraint::Fix { point: 0, at: [0.0, 0.0] },
    ]);
    let mut e = editor(Tool::Select);
    assert!(e.drag_begin(&sk, 2, [10.0, 5.0]));
    e.drag_update([14.0, 9.0]);
    let cmds = e.drag_end();
    assert_eq!(cmds.len(), 1);
    let after = surfacecraft_ui_egui::sketch_editor::drag::moved(&sk, 2, [14.0, 9.0]).expect("solves");
    let (a, b, c, d) = (after.points[0], after.points[1], after.points[2], after.points[3]);
    let tol = 1e-9;
    assert!((a[1] - b[1]).abs() < tol && (c[1] - d[1]).abs() < tol, "horizontal sides");
    assert!((b[0] - c[0]).abs() < tol && (d[0] - a[0]).abs() < tol, "vertical sides");
    assert!(a[0].abs() < tol && a[1].abs() < tol, "the fixed corner stays");
}

#[test]
fn dragging_a_fully_dimensioned_point_moves_nothing_and_issues_no_command() {
    let mut sk = polyline();
    sk.constraints.push(Constraint::Fix { point: 2, at: [10.0, 5.0] });
    let mut e = editor(Tool::Select);
    assert!(e.drag_begin(&sk, 2, [3.0, 3.0]));
    e.drag_update([4.0, 1.0]);
    assert!(e.drag_end().is_empty(), "the solver puts the fixed point back: no command");
}

#[test]
fn drags_only_start_in_the_select_tool_on_a_real_point() {
    let sk = polyline();
    let mut e = editor(Tool::Line);
    assert!(!e.drag_begin(&sk, 2, [10.0, 5.0]), "no drag while drawing");
    let mut e = editor(Tool::Select);
    assert!(!e.drag_begin(&sk, 99, [0.0, 0.0]), "no such point");
    assert!(e.drag_end().is_empty());
}

#[test]
fn escape_during_a_drag_cancels_it_without_a_command() {
    let sk = polyline();
    let mut e = editor(Tool::Select);
    assert!(e.drag_begin(&sk, 2, [10.0, 5.0]));
    e.drag_update([12.0, 7.0]);
    e.escape();
    assert!(e.drag.is_none());
    assert!(e.drag_end().is_empty());
}

/// Presses on `from`, moves to `to` in steps and releases there (the egui drag threshold is passed
/// by the intermediate moves).
fn drag_by_mouse(h: &mut Harness<'static, SurfaceCraftApp>, from: egui::Pos2, to: egui::Pos2) {
    h.hover_at(from);
    h.run();
    h.drag_at(from);
    h.run();
    for t in [0.25_f32, 0.5, 0.75, 1.0] {
        h.hover_at(from + (to - from) * t);
        h.run();
    }
    h.drop_at(to);
    h.run();
}

#[test]
fn a_mouse_drag_of_a_point_is_one_command_and_one_undo_step() {
    let mut app = SurfaceCraftApp::headless();
    let id = new_sketch_on_xy(&mut app);
    app.run_command("sketch.add", json!({"sketch": id.0, "entities": [{"polygon": {"points": [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0]]}}]})).unwrap();
    let mut h = harness(app);
    h.run();
    no_snaps(&mut h);
    let (log0, undo0) = (h.state().session.log.len(), h.state().session.history.undo_labels().len());
    let from = screen_of(h.state(), id, [10.0, 5.0]);
    let to = screen_of(h.state(), id, [14.5, 3.25]);
    drag_by_mouse(&mut h, from, to);
    let app = h.state();
    assert_eq!(app.session.log.len(), log0 + 1, "one command for the drag");
    assert_eq!(app.session.log.last().map(|e| e.command.as_str()), Some("sketch.move_point"));
    assert_eq!(app.session.history.undo_labels().len(), undo0 + 1, "one undo step");
    let p = app.session.doc.sketch(id).expect("sketch").sketch.points[2];
    assert!((p[0] - 14.5).abs() < 0.05 && (p[1] - 3.25).abs() < 0.05, "the corner follows the pointer: {p:?}");
}

#[test]
fn esc_during_a_mouse_drag_leaves_the_document_unchanged() {
    let mut app = SurfaceCraftApp::headless();
    let id = new_sketch_on_xy(&mut app);
    app.run_command("sketch.add", json!({"sketch": id.0, "entities": [{"line": {"from": [0.0, 0.0], "to": [10.0, 0.0]}}, {"line": {"from": [10.0, 0.0], "to": [10.0, 5.0]}}]})).unwrap();
    let mut h = harness(app);
    h.run();
    no_snaps(&mut h);
    let before = surfacecraft_doc::format::to_json(&h.state().session.doc).expect("json");
    let log0 = h.state().session.log.len();
    let from = screen_of(h.state(), id, [10.0, 0.0]);
    let to = screen_of(h.state(), id, [13.0, -4.0]);
    h.hover_at(from);
    h.run();
    h.drag_at(from);
    h.run();
    h.hover_at(to);
    h.run();
    h.key_press(egui::Key::Escape);
    h.run();
    h.drop_at(to);
    h.run();
    let app = h.state();
    assert_eq!(surfacecraft_doc::format::to_json(&app.session.doc).expect("json"), before, "document hash unchanged");
    assert_eq!(app.session.log.len(), log0);
}
