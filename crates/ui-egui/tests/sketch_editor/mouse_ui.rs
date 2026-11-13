//! The sketch editor through the real UI with simulated pointer events.
use super::common::*;
use egui_kittest::kittest::Queryable;
use serde_json::json;
use surfacecraft_doc::ObjectId;
use surfacecraft_sketch::Constraint;
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::sketch_editor::Tool;
use surfacecraft_ui_egui::sketch_editor::view::PlaneView;

#[test]
fn mouse_drawing_through_the_ui_is_one_command_per_gesture_and_esc_cancels() {
    let mut app = SurfaceCraftApp::headless();
    app.new_sketch_on([0.0, 0.0, 1.0]);
    let id = app.sketch_editor.target.expect("editing");
    let mut h = harness(app);
    h.run();
    no_snaps(&mut h);
    h.get_by_label("Line").click();
    h.run();
    assert_eq!(h.state().sketch_editor.tool, Tool::Line);

    let (a, b) = ([-20.0, -10.0], [15.0, -25.0]);
    let (log0, undo0) = (h.state().session.log.len(), h.state().session.history.undo_labels().len());
    let pa = screen_of(h.state(), id, a);
    click_at(&mut h, pa);
    assert_eq!(h.state().session.log.len(), log0, "the first click is no command");
    // Esc mid-gesture: nothing happens to the document
    h.key_press(egui::Key::Escape);
    h.run();
    assert!(h.state().sketch_editor.gesture.is_empty());
    assert_eq!(h.state().session.log.len(), log0);
    // a full gesture: one sketch.add
    click_at(&mut h, pa);
    let pb = screen_of(h.state(), id, b);
    click_at(&mut h, pb);
    let app = h.state();
    assert_eq!(app.session.log.len(), log0 + 1, "one command for the gesture");
    assert_eq!(app.session.log.last().map(|e| e.command.as_str()), Some("sketch.add"));
    assert_eq!(app.session.history.undo_labels().len(), undo0 + 1, "one undo step");
    let o = app.session.doc.sketch(id).expect("sketch");
    assert_eq!(o.sketch.points.len(), 2);
    for (got, want) in o.sketch.points.iter().zip([a, b]) {
        assert!((got[0] - want[0]).hypot(got[1] - want[1]) < 0.05, "{got:?} vs {want:?}");
    }
    // the solver status is on screen
    assert!(h.query_by_label_contains("Under-defined").is_some());
}

#[test]
fn constraint_by_mouse_and_the_status_and_conflict_show_on_screen() {
    let mut app = SurfaceCraftApp::headless();
    app.new_sketch_on([0.0, 0.0, 1.0]);
    let id = app.sketch_editor.target.expect("editing");
    app.run_command(
        "sketch.add",
        json!({"sketch": id.0, "entities": [{"line": {"from": [-20.0, -10.0], "to": [20.0, -15.0]}}]}),
    )
    .expect("add");
    let mut h = harness(app);
    h.run();
    no_snaps(&mut h);
    assert!(h.query_by_label_contains("Under-defined: 4").is_some());
    // pick the horizontal tool and click the line
    h.get_by_label("Horizontal").click();
    h.run();
    let mid = screen_of(h.state(), id, [0.0, -12.5]);
    click_at(&mut h, mid);
    let o = h.state().session.doc.sketch(id).expect("sketch");
    assert_eq!(o.sketch.constraints, vec![Constraint::Horizontal { line: 0 }]);
    assert!(h.query_by_label_contains("Under-defined: 3").is_some(), "one degree of freedom less");
    // two contradicting lengths: over-defined, shown with the conflicting constraint
    h.state_mut()
        .run_command(
            "sketch.constrain",
            json!({"sketch": id.0, "constraints": [{"type": "length", "line": 0, "value": 5.0}, {"type": "length", "line": 0, "value": 7.0}]}),
        )
        .expect("constrain");
    h.run();
    // the status is on the strip's button; the conflicting constraints are in its details
    assert!(h.query_by_label_contains("Over-defined").is_some());
    assert!(h.query_by_label_contains("Conflicting constraints: #").is_none(), "folded");
    h.get_by_label_contains("Over-defined").click();
    h.run();
    assert!(h.query_by_label_contains("Conflicting constraints: #").is_some());
}

#[test]
fn double_clicking_a_dimension_label_opens_it_and_enter_edits_it() {
    let mut app = SurfaceCraftApp::headless();
    app.new_sketch_on([0.0, 0.0, 1.0]);
    let id = app.sketch_editor.target.expect("editing");
    app.run_command(
        "sketch.add",
        json!({"sketch": id.0, "entities": [{"line": {"from": [-20.0, -10.0], "to": [20.0, -10.0]}}]}),
    )
    .expect("add");
    app.run_command(
        "sketch.constrain",
        json!({"sketch": id.0, "constraints": [{"type": "length", "line": 0, "value": 40.0}]}),
    )
    .expect("constrain");
    let mut h = harness(app);
    h.run();
    h.run();
    let (idx, rect) = h.state().sketch_editor.labels.first().copied().expect("the dimension label was drawn");
    let origin = h.state().viewport_rect.expect("viewport").min;
    let pos = origin + egui::vec2((rect[0] + rect[2]) / 2.0, (rect[1] + rect[3]) / 2.0);
    h.hover_at(pos);
    h.run();
    // the harness advances time between frames: allow a slow double click
    h.ctx.options_mut(|o| o.input_options.max_double_click_delay = 10.0);
    for pressed in [true, false, true, false] {
        h.event(egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        });
    }
    h.run();
    let pending = h.state().sketch_editor.pending.clone().expect("the value field opened");
    assert_eq!((pending.index, pending.text.as_str()), (Some(idx), "40"));
    if let Some(p) = h.state_mut().sketch_editor.pending.as_mut() {
        p.text = "55".into();
    }
    let n = h.state().session.log.len();
    if let Some(p) = h.state_mut().sketch_editor.pending.as_mut() {
        p.text = "55".into();
    }
    let cmds = h.state_mut().sketch_editor.commit_pending();
    for c in cmds {
        h.state_mut().run_command(c.id, c.params).expect("edit");
    }
    h.run();
    let app = h.state();
    assert_eq!(app.session.log.len(), n + 1);
    assert_eq!(app.session.log.last().map(|e| e.command.as_str()), Some("sketch.edit_constraint"));
    let o = app.session.doc.sketch(id).expect("sketch");
    assert!(matches!(o.sketch.constraints[idx], Constraint::Length { value, .. } if (value - 55.0).abs() < 1e-12));
}

#[test]
fn a_section_sketch_is_reference_geometry_and_the_camera_looks_onto_the_plane() {
    let mut app = SurfaceCraftApp::headless();
    app.run_command("mesh.synth", json!({"shape": "sphere", "size": [12.0], "resolution": 24})).expect("synth");
    let section = app
        .run_command("sketch.from_section", json!({"mesh": 1, "origin": [0.0, 0.0, 0.0], "normal": [0.0, 0.0, 1.0]}))
        .expect("section");
    let section_id = ObjectId(section["id"].as_u64().expect("id"));
    app.new_sketch_on([0.0, 0.0, 1.0]);
    let mine = app.sketch_editor.target.expect("editing");
    assert_ne!(mine, section_id);
    // the camera looks straight down onto the XY plane, x to the right
    let cam = app.camera.clone();
    assert!(cam.forward().z < -0.99, "{:?}", cam.forward());
    let plane = app.session.doc.sketch(mine).expect("sketch").sketch.plane.clone();
    let view = PlaneView::new(&cam, &plane, 800.0, 600.0);
    let (o, x) = (view.to_screen([0.0, 0.0]).expect("o"), view.to_screen([10.0, 0.0]).expect("x"));
    assert!(x[0] > o[0] + 1.0 && (x[1] - o[1]).abs() < 0.5 * (x[0] - o[0]), "x runs to the right: {o:?} {x:?}");
    // and the pixel under the origin maps back to it
    let back = view.to_plane(o[0], o[1]).expect("back");
    assert!(back[0].abs() < 1e-3 && back[1].abs() < 1e-3);
    // the section is drawn as reference: the editor sees it as another sketch of the document
    let mut h = harness(app);
    h.run();
    assert!(h.state().session.doc.sketch(section_id).is_some());
    assert!(h.state().sketch_editing());
}
