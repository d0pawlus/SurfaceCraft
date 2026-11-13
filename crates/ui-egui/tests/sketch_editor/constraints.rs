//! Constraint tools issue the expected `sketch.constrain` commands.
use super::common::*;
use serde_json::json;
use surfacecraft_ui_egui::sketch_editor::Tool;
use surfacecraft_ui_egui::sketch_editor::snap::Hit;

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
