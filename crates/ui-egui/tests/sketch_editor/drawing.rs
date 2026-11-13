//! Drawing tools as state machines: one command per gesture, typed lengths, Esc and the arcs.
use super::common::*;
use serde_json::json;
use surfacecraft_sketch::Sketch;
use surfacecraft_ui_egui::sketch_editor::{Cmd, Tool};

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
