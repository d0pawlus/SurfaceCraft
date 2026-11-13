//! Editing a dimension issues `sketch.edit_constraint` and the solver follows.
use super::common::*;
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_sketch::{Constraint, SolveStatus};
use surfacecraft_ui_egui::sketch_editor::{SketchEditor, Tool};

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
