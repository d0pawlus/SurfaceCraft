//! Drawing on a tilted plane, and the solver status as a line, a token and a conflict.
use super::common::*;
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_ui_egui::sketch_editor::{SketchEditor, Tool, status_view};
use surfacecraft_ui_egui::theme::Token;

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
