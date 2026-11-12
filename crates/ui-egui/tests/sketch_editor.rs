//! The sketch editor (G5) without a GPU: the tool state machines, the snapping maths, constraint
//! picking, dimension editing and the solver status, then the same through the real UI with
//! simulated pointer events. Every action must reach the engine as one `sketch.*` command.
use egui_kittest::Harness;
use egui_kittest::kittest::Queryable;
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_sketch::{Constraint, Entity, Sketch, SolveStatus};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::sketch_editor::snap::{self, Hit, Snap, SnapConfig, SnapKind};
use surfacecraft_ui_egui::sketch_editor::view::PlaneView;
use surfacecraft_ui_egui::sketch_editor::{Cmd, SketchEditor, Tool, status_view};
use surfacecraft_ui_egui::theme::Token;

/// A 10 x 5 rectangle (lines 0..=3) and a circle of radius 3 at (20, 0) (entity 4, point 4).
fn rect_sketch() -> Sketch {
    let mut s = Sketch::default();
    for p in [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0], [20.0, 0.0]] {
        s.add_point(p);
    }
    for (a, b) in [(0, 1), (1, 2), (2, 3), (3, 0)] {
        s.entities.push(Entity::Line { a, b });
    }
    s.entities.push(Entity::Circle { center: 4, radius: 3.0 });
    s
}

fn editor(tool: Tool) -> SketchEditor {
    let mut e = SketchEditor::default();
    e.begin(ObjectId(1));
    e.set_tool(tool);
    e
}

fn click(e: &mut SketchEditor, sk: &Sketch, p: [f64; 2]) -> Vec<Cmd> {
    e.click(sk, Snap::free(p), 0.5)
}

fn constrain(c: Value) -> Vec<Cmd> {
    vec![Cmd { id: "sketch.constrain", params: json!({"sketch": 1, "constraints": [c]}) }]
}

#[test]
fn line_tool_click_click_is_one_sketch_add_in_plane_coordinates() {
    let sk = Sketch::default();
    let mut e = editor(Tool::Line);
    assert!(click(&mut e, &sk, [1.0, 2.0]).is_empty(), "the first click only places the start");
    let cmds = click(&mut e, &sk, [4.0, 6.0]);
    assert_eq!(
        cmds,
        vec![Cmd {
            id: "sketch.add",
            params: json!({"sketch": 1, "entities": [{"line": {"from": [1.0, 2.0], "to": [4.0, 6.0]}}]})
        }]
    );
    assert!(e.gesture.is_empty(), "the gesture is over");
    // a second line is a new gesture
    assert!(click(&mut e, &sk, [0.0, 0.0]).is_empty());
}

#[test]
fn escape_mid_gesture_issues_no_command_and_drops_the_tool_next() {
    let sk = Sketch::default();
    let mut e = editor(Tool::Polyline);
    click(&mut e, &sk, [0.0, 0.0]);
    click(&mut e, &sk, [5.0, 0.0]);
    assert_eq!(e.gesture.len(), 2);
    e.escape();
    assert!(e.gesture.is_empty() && e.finish().is_empty(), "Esc cancelled the gesture, nothing to finish");
    assert_eq!(e.tool, Tool::Polyline, "the first Esc only cancels the gesture");
    e.escape();
    assert_eq!(e.tool, Tool::Select, "the next Esc cancels the tool");
    // a half-typed dimension is cancelled too
    let sk = rect_sketch();
    let mut e = editor(Tool::Dimension);
    click(&mut e, &sk, [5.0, 0.0]);
    assert!(e.pending.is_some());
    e.escape();
    assert!(e.pending.is_none());
}

#[test]
fn polyline_is_one_command_and_closes_on_the_first_point() {
    let sk = Sketch::default();
    let mut e = editor(Tool::Polyline);
    for p in [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0]] {
        assert!(click(&mut e, &sk, p).is_empty());
    }
    let cmds = e.finish();
    assert_eq!(cmds.len(), 1, "a whole chain is one command (one undo step)");
    assert_eq!(cmds[0].id, "sketch.add");
    assert_eq!(cmds[0].params["entities"].as_array().map(Vec::len), Some(2));
    // a click on the first point closes it as a polygon
    for p in [[0.0, 0.0], [4.0, 0.0], [4.0, 3.0]] {
        click(&mut e, &sk, p);
    }
    let cmds = click(&mut e, &sk, [0.0, 0.0]);
    assert_eq!(cmds.len(), 1);
    assert_eq!(cmds[0].params["entities"][0]["polygon"]["points"].as_array().map(Vec::len), Some(3));
    // a click on the last point (a double click) ends the chain
    click(&mut e, &sk, [0.0, 0.0]);
    click(&mut e, &sk, [2.0, 0.0]);
    assert_eq!(click(&mut e, &sk, [2.0, 0.0]).len(), 1);
}

#[test]
fn circle_rectangle_and_arcs_make_one_entity_each() {
    let sk = Sketch::default();
    let mut e = editor(Tool::Circle);
    click(&mut e, &sk, [1.0, 1.0]);
    let c = click(&mut e, &sk, [4.0, 5.0]);
    assert_eq!(c[0].params["entities"][0]["circle"]["radius"], json!(5.0));
    assert_eq!(c[0].params["entities"][0]["circle"]["center"], json!([1.0, 1.0]));

    let mut e = editor(Tool::Rect);
    click(&mut e, &sk, [0.0, 0.0]);
    let c = click(&mut e, &sk, [6.0, 4.0]);
    assert_eq!(
        c[0].params["entities"][0]["polygon"]["points"],
        json!([[0.0, 0.0], [6.0, 0.0], [6.0, 4.0], [0.0, 4.0]])
    );
    // a degenerate rectangle is nothing
    click(&mut e, &sk, [0.0, 0.0]);
    assert!(click(&mut e, &sk, [3.0, 0.0]).is_empty());

    // 3-point arc: start, end, a point on it (the upper half of the unit circle)
    let mut e = editor(Tool::Arc3);
    click(&mut e, &sk, [1.0, 0.0]);
    click(&mut e, &sk, [-1.0, 0.0]);
    let c = click(&mut e, &sk, [0.0, 1.0]);
    let arc = &c[0].params["entities"][0]["arc"];
    assert_eq!(arc["start"], json!([1.0, 0.0]));
    assert_eq!(arc["end"], json!([-1.0, 0.0]));
    // through the lower half it runs counter-clockwise from the other end
    click(&mut e, &sk, [1.0, 0.0]);
    click(&mut e, &sk, [-1.0, 0.0]);
    let c = click(&mut e, &sk, [0.0, -1.0]);
    let arc = &c[0].params["entities"][0]["arc"];
    assert_eq!(arc["start"], json!([-1.0, 0.0]));
    assert_eq!(arc["end"], json!([1.0, 0.0]));
    // collinear points are not an arc: no command, a notice
    click(&mut e, &sk, [0.0, 0.0]);
    click(&mut e, &sk, [2.0, 0.0]);
    assert!(click(&mut e, &sk, [1.0, 0.0]).is_empty());
    assert!(e.notice.is_some());

    // centre arc: centre, start, end direction (put on the start's radius)
    let mut e = editor(Tool::ArcCentre);
    click(&mut e, &sk, [0.0, 0.0]);
    click(&mut e, &sk, [2.0, 0.0]);
    let c = click(&mut e, &sk, [0.0, 7.0]);
    let arc = &c[0].params["entities"][0]["arc"];
    assert_eq!(arc["center"], json!([0.0, 0.0]));
    assert_eq!(arc["end"], json!([0.0, 2.0]));
}

#[test]
fn typed_length_places_the_end_along_the_cursor() {
    let sk = Sketch::default();
    let mut e = editor(Tool::Line);
    click(&mut e, &sk, [1.0, 1.0]);
    let c = e.type_value(&sk, 5.0, [1.0, 9.0]);
    assert_eq!(c[0].params["entities"][0]["line"]["to"], json!([1.0, 6.0]));
    // a circle takes a typed radius
    let mut e = editor(Tool::Circle);
    click(&mut e, &sk, [0.0, 0.0]);
    let c = e.type_value(&sk, 2.5, [9.0, 0.0]);
    assert_eq!(c[0].params["entities"][0]["circle"]["radius"], json!(2.5));
    // nothing sensible: no command
    let mut e = editor(Tool::Line);
    click(&mut e, &sk, [0.0, 0.0]);
    assert!(e.type_value(&sk, -1.0, [1.0, 0.0]).is_empty());
}

#[test]
fn snapping_endpoints_midpoints_centres_grid_and_inference() {
    let sk = rect_sketch();
    let all = SnapConfig::default();
    let s = snap::snap_point(&sk, [10.2, 0.1], None, &[], 0.5, 1.0, &all);
    assert_eq!((s.pos, s.kind), ([10.0, 0.0], SnapKind::Endpoint));
    let s = snap::snap_point(&sk, [20.1, -0.2], None, &[], 0.5, 1.0, &all);
    assert_eq!((s.pos, s.kind), ([20.0, 0.0], SnapKind::Centre));
    let s = snap::snap_point(&sk, [5.2, 0.2], None, &[], 0.5, 1.0, &all);
    assert_eq!((s.pos, s.kind), ([5.0, 0.0], SnapKind::Midpoint));
    // the grid, when nothing else is near
    let grid_only = SnapConfig { points: false, midpoints: false, inference: false, grid: true };
    let s = snap::snap_point(&sk, [3.1, 7.4], None, &[], 0.5, 0.5, &grid_only);
    assert_eq!((s.pos, s.kind), ([3.0, 7.5], SnapKind::Grid));
    // horizontal and vertical inference from the last placed point
    let infer = SnapConfig { points: true, midpoints: false, inference: true, grid: false };
    let s = snap::snap_point(&sk, [30.0, 2.3], Some([2.0, 2.0]), &[], 0.5, 1.0, &infer);
    assert_eq!((s.pos, s.kind), ([30.0, 2.0], SnapKind::Horizontal));
    let s = snap::snap_point(&sk, [2.2, 40.0], Some([2.0, 2.0]), &[], 0.5, 1.0, &infer);
    assert_eq!((s.pos, s.kind), ([2.0, 40.0], SnapKind::Vertical));
    // an existing point beats the inference; the gesture's own points snap too (closing)
    let s = snap::snap_point(&sk, [10.1, 0.3], Some([2.0, 0.0]), &[], 0.5, 1.0, &infer);
    assert_eq!(s.kind, SnapKind::Endpoint);
    let s = snap::snap_point(&sk, [-30.1, -30.1], None, &[[-30.0, -30.0]], 0.5, 1.0, &all);
    assert_eq!((s.pos, s.kind), ([-30.0, -30.0], SnapKind::Endpoint));
    // out of reach and everything off: the cursor as it is
    let none = SnapConfig { points: false, midpoints: false, inference: false, grid: false };
    assert_eq!(snap::snap_point(&sk, [1.234, 5.678], None, &[], 0.5, 1.0, &none), Snap::free([1.234, 5.678]));
    // the grid step follows the zoom in 1-2-5 steps
    assert!((snap::grid_step(0.1, 14.0) - 2.0).abs() < 1e-12);
    assert!((snap::grid_step(1.0, 14.0) - 20.0).abs() < 1e-12);
    assert!((snap::grid_step(0.02, 14.0) - 0.5).abs() < 1e-12);
}

#[test]
fn picking_finds_lines_arcs_circles_and_points() {
    let sk = rect_sketch();
    assert_eq!(snap::pick(&sk, [5.0, 0.2], 0.5, snap::Want::Line), Some(Hit::Entity(0)));
    assert_eq!(snap::pick(&sk, [5.0, 2.5], 0.5, snap::Want::Line), None);
    assert_eq!(snap::pick(&sk, [23.1, 0.0], 0.5, snap::Want::Circular), Some(Hit::Entity(4)));
    assert_eq!(snap::pick(&sk, [10.1, 0.1], 0.5, snap::Want::Point), Some(Hit::Point(1)));
    // points win over the lines through them
    assert_eq!(snap::pick(&sk, [10.1, 0.1], 0.5, snap::Want::PointOrEntity), Some(Hit::Point(1)));
    // an arc is hit only along its sweep
    let mut a = Sketch::default();
    let (c, s, e) = (a.add_point([0.0, 0.0]), a.add_point([1.0, 0.0]), a.add_point([-1.0, 0.0]));
    a.entities.push(Entity::Arc { center: c, a: s, b: e });
    assert!(snap::entity_distance(&a, 0, [0.0, 1.0]) < 1e-9, "upper half is the arc");
    assert!(snap::entity_distance(&a, 0, [0.0, -1.0]) > 0.9, "lower half is not");
}

#[test]
fn constraint_tools_issue_the_expected_sketch_constrain() {
    let sk = rect_sketch();
    let mut e = editor(Tool::Horizontal);
    assert_eq!(click(&mut e, &sk, [5.0, 0.1]), constrain(json!({"type": "horizontal", "line": 0})));
    e.set_tool(Tool::Vertical);
    assert_eq!(click(&mut e, &sk, [10.0, 2.5]), constrain(json!({"type": "vertical", "line": 1})));
    assert!(click(&mut e, &sk, [5.0, 2.5]).is_empty(), "a miss does nothing");

    e.set_tool(Tool::Parallel);
    assert!(click(&mut e, &sk, [5.0, 0.0]).is_empty());
    assert_eq!(e.picks, vec![Hit::Entity(0)]);
    assert!(click(&mut e, &sk, [5.0, 0.1]).is_empty(), "the same line twice is not a pair");
    assert_eq!(click(&mut e, &sk, [5.0, 5.0]), constrain(json!({"type": "parallel", "a": 0, "b": 2})));
    assert!(e.picks.is_empty());

    e.set_tool(Tool::Perpendicular);
    click(&mut e, &sk, [5.0, 0.0]);
    assert_eq!(click(&mut e, &sk, [10.0, 2.5]), constrain(json!({"type": "perpendicular", "a": 0, "b": 1})));
    e.set_tool(Tool::Equal);
    click(&mut e, &sk, [5.0, 0.0]);
    assert_eq!(click(&mut e, &sk, [5.0, 5.0]), constrain(json!({"type": "equal", "a": 0, "b": 2})));

    // tangent: line and circle in either order
    e.set_tool(Tool::Tangent);
    click(&mut e, &sk, [5.0, 0.0]);
    assert_eq!(click(&mut e, &sk, [23.0, 0.0]), constrain(json!({"type": "tangent", "line": 0, "entity": 4})));
    click(&mut e, &sk, [23.0, 0.0]);
    assert_eq!(click(&mut e, &sk, [5.0, 5.0]), constrain(json!({"type": "tangent", "line": 2, "entity": 4})));

    e.set_tool(Tool::Coincident);
    click(&mut e, &sk, [10.0, 0.0]);
    assert_eq!(click(&mut e, &sk, [10.0, 5.0]), constrain(json!({"type": "coincident", "a": 1, "b": 2})));

    e.set_tool(Tool::Fix);
    assert_eq!(click(&mut e, &sk, [0.1, 5.1]), constrain(json!({"type": "fix", "point": 3, "at": [0.0, 5.0]})));
}

#[test]
fn dimension_tool_clicks_an_entity_then_a_typed_value_makes_the_constraint() {
    let sk = rect_sketch();
    let mut e = editor(Tool::Dimension);
    assert!(click(&mut e, &sk, [5.0, 0.0]).is_empty(), "the click only opens the value field");
    let p = e.pending.clone().expect("a pending length");
    assert_eq!(p.text, "10", "prefilled with the current length");
    set_text(&mut e, "12,5");
    assert_eq!(e.commit_pending(), constrain(json!({"type": "length", "line": 0, "value": 12.5})));
    assert!(e.pending.is_none());

    click(&mut e, &sk, [23.0, 0.0]);
    set_text(&mut e, "4");
    assert_eq!(e.commit_pending(), constrain(json!({"type": "radius", "entity": 4, "value": 4.0})));

    click(&mut e, &sk, [0.0, 0.0]);
    assert!(click(&mut e, &sk, [10.0, 5.0]).is_empty());
    assert_eq!(e.pending.as_ref().map(|p| p.text.as_str()), Some("11.1803"));
    set_text(&mut e, "12");
    assert_eq!(e.commit_pending(), constrain(json!({"type": "distance", "a": 0, "b": 2, "value": 12.0})));

    e.set_tool(Tool::Angle);
    click(&mut e, &sk, [5.0, 0.0]);
    click(&mut e, &sk, [10.0, 2.5]);
    assert_eq!(e.pending.as_ref().map(|p| p.text.as_str()), Some("90"));
    set_text(&mut e, "60");
    assert_eq!(e.commit_pending(), constrain(json!({"type": "angle", "a": 0, "b": 1, "degrees": 60.0})));

    // a bad value keeps the field open and says why
    e.set_tool(Tool::Dimension);
    click(&mut e, &sk, [5.0, 0.0]);
    for bad in ["abc", "-3", "0", ""] {
        set_text(&mut e, bad);
        assert!(e.commit_pending().is_empty(), "{bad:?}");
        assert!(e.pending.is_some() && e.notice.is_some());
    }
}

/// Runs editor commands through a session, as the app does.
fn run(s: &mut Session, cmds: &[Cmd]) {
    for c in cmds {
        s.execute(c.id, c.params.clone()).unwrap_or_else(|err| panic!("{}: {err}", c.id));
    }
}

fn new_sketch(s: &mut Session, params: Value) -> ObjectId {
    let v = s.execute("sketch.create", params).expect("sketch.create");
    ObjectId(v["id"].as_u64().expect("id"))
}

#[test]
fn editing_a_dimension_issues_sketch_edit_constraint_and_the_solver_follows() {
    let mut s = Session::new();
    let id = new_sketch(&mut s, json!({}));
    let mut e = SketchEditor::default();
    e.begin(id);
    e.set_tool(Tool::Line);
    let sk = s.doc.sketch(id).expect("sketch").sketch.clone();
    click(&mut e, &sk, [0.0, 0.0]);
    run(&mut s, &click(&mut e, &sk, [10.0, 0.0]));
    e.set_tool(Tool::Fix);
    let sk = s.doc.sketch(id).expect("sketch").sketch.clone();
    run(&mut s, &click(&mut e, &sk, [0.0, 0.0]));
    e.set_tool(Tool::Horizontal);
    run(&mut s, &click(&mut e, &sk, [5.0, 0.0]));
    e.set_tool(Tool::Dimension);
    click(&mut e, &sk, [5.0, 0.0]);
    run(&mut s, &e.commit_pending());
    let o = s.doc.sketch(id).expect("sketch");
    assert_eq!(o.report.as_ref().map(|r| r.status), Some(SolveStatus::FullyDefined), "{:?}", o.report);
    let dim = o.sketch.constraints.iter().position(|c| matches!(c, Constraint::Length { .. })).expect("a length");
    let sk = o.sketch.clone();

    // double click on the dimension: the value field opens with the current value
    let mut e2 = SketchEditor::default();
    e2.begin(id);
    assert!(e2.begin_edit(&sk, dim));
    assert_eq!(e2.pending.as_ref().map(|p| p.text.as_str()), Some("10"));
    assert!(!e2.begin_edit(&sk, 0) || matches!(sk.constraints[0], Constraint::Length { .. }), "only dimensions open");
    set_text(&mut e2, "15");
    let cmds = e2.commit_pending();
    assert_eq!(cmds.len(), 1);
    assert_eq!(cmds[0].id, "sketch.edit_constraint");
    assert_eq!(
        cmds[0].params,
        json!({"sketch": id.0, "index": dim, "constraint": {"type": "length", "line": 0, "value": 15.0}})
    );
    let undo_before = s.history.undo_labels().len();
    run(&mut s, &cmds);
    assert_eq!(s.history.undo_labels().len(), undo_before + 1, "one undo step");
    let o = s.doc.sketch(id).expect("sketch");
    assert!(matches!(o.sketch.constraints[dim], Constraint::Length { value, .. } if (value - 15.0).abs() < 1e-12));
    assert!((o.sketch.points[1][0] - 15.0).abs() < 1e-6, "the solver stretched the line: {:?}", o.sketch.points);

    // deleting a constraint is an edit without a replacement
    let del = e2.delete_constraint(dim);
    assert_eq!(del[0].params, json!({"sketch": id.0, "index": dim}));
    run(&mut s, &del);
    assert!(
        s.doc.sketch(id).expect("sketch").sketch.constraints.iter().all(|c| !matches!(c, Constraint::Length { .. }))
    );
}

#[test]
fn drawing_works_on_a_tilted_plane_in_plane_space() {
    let mut s = Session::new();
    let id = new_sketch(&mut s, json!({"plane": {"origin": [5.0, 6.0, 7.0], "normal": [1.0, 2.0, 2.0]}}));
    let mut e = SketchEditor::default();
    e.begin(id);
    e.set_tool(Tool::Line);
    let sk = s.doc.sketch(id).expect("sketch").sketch.clone();
    click(&mut e, &sk, [1.0, 2.0]);
    run(&mut s, &click(&mut e, &sk, [4.0, 6.0]));
    let o = s.doc.sketch(id).expect("sketch");
    assert_eq!(o.sketch.points, vec![[1.0, 2.0], [4.0, 6.0]]);
}

#[test]
fn solver_status_maps_to_a_line_and_a_token_and_names_the_conflict() {
    assert_eq!(status_view(None).key, "sketch.status.unsolved");
    let mut s = Session::new();
    let id = new_sketch(&mut s, json!({}));
    s.execute("sketch.add", json!({"sketch": id.0, "entities": [{"line": {"from": [0.0, 0.0], "to": [10.0, 0.0]}}]}))
        .expect("add");
    let under = status_view(s.doc.sketch(id).expect("sketch").report.as_ref());
    assert_eq!((under.key, under.token), ("sketch.status.under", Token::StatusWarning));
    assert_eq!(under.dof, 4, "a free line has four degrees of freedom");
    // two lengths that disagree: over-defined, the second one is the conflict
    s.execute(
        "sketch.constrain",
        json!({"sketch": id.0, "constraints": [{"type": "length", "line": 0, "value": 5.0}, {"type": "length", "line": 0, "value": 7.0}]}),
    )
    .expect("constrain");
    let over = status_view(s.doc.sketch(id).expect("sketch").report.as_ref());
    assert_eq!((over.key, over.token), ("sketch.status.over", Token::StatusError));
    assert!(!over.conflicting.is_empty(), "{over:?}");
    // fully defined
    let mut s = Session::new();
    let id = new_sketch(&mut s, json!({}));
    s.execute("sketch.add", json!({"sketch": id.0, "entities": [{"circle": {"center": [0.0, 0.0], "radius": 2.0}}]}))
        .expect("add");
    s.execute(
        "sketch.constrain",
        json!({"sketch": id.0, "constraints": [{"type": "fix", "point": 0, "at": [0.0, 0.0]}, {"type": "radius", "entity": 0, "value": 2.0}]}),
    )
    .expect("constrain");
    let full = status_view(s.doc.sketch(id).expect("sketch").report.as_ref());
    assert_eq!((full.key, full.token, full.dof), ("sketch.status.fully", Token::StatusOk, 0));
}

fn harness(app: SurfaceCraftApp) -> Harness<'static, SurfaceCraftApp> {
    Harness::builder().with_size((1600.0, 1000.0)).build_ui_state(|ui, app: &mut SurfaceCraftApp| app.draw(ui), app)
}

fn screen_of(app: &SurfaceCraftApp, id: ObjectId, p: [f64; 2]) -> egui::Pos2 {
    let rect = app.viewport_rect.expect("viewport");
    let plane = app.session.doc.sketch(id).expect("sketch").sketch.plane.clone();
    let q = PlaneView::new(&app.camera, &plane, rect.width(), rect.height()).to_screen(p).expect("on screen");
    rect.min + egui::vec2(q[0], q[1])
}

fn click_at(h: &mut Harness<'static, SurfaceCraftApp>, pos: egui::Pos2) {
    h.hover_at(pos);
    h.run();
    h.drag_at(pos);
    h.drop_at(pos);
    h.run();
}

fn no_snaps(h: &mut Harness<'static, SurfaceCraftApp>) {
    h.state_mut().sketch_editor.snap = SnapConfig { points: false, midpoints: false, inference: false, grid: false };
}

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

fn set_text(e: &mut SketchEditor, text: &str) {
    if let Some(p) = e.pending.as_mut() {
        p.text = text.into();
    }
}

/// The polyline of the drag tests: points (0, 0), (10, 0), (10, 5); lines 0-1 and 1-2.
fn polyline() -> Sketch {
    let mut s = Sketch::default();
    for p in [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0]] {
        s.add_point(p);
    }
    s.entities.push(Entity::Line { a: 0, b: 1 });
    s.entities.push(Entity::Line { a: 1, b: 2 });
    s
}

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

/// A sketch on the XY plane, edited (the camera looks onto it), as the tests above start.
fn new_sketch_on_xy(app: &mut SurfaceCraftApp) -> ObjectId {
    app.new_sketch_on([0.0, 0.0, 1.0]);
    app.sketch_editor.target.expect("editing")
}

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
    let d = e.drag.as_ref().expect("drag");
    let (a, b) = (d.preview.points[0], d.preview.points[1]);
    assert!(((a[0] - b[0]).hypot(a[1] - b[1]) - 10.0).abs() < 1e-9, "the preview keeps the length");
    let cmds = e.drag_end();
    assert!(cmds.len() <= 1);
    assert!(valid.points.len() == 2);
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
