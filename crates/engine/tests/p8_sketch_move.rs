//! P8-H7: `sketch.move_point` (the command behind a mouse drag of a sketch point). The point goes
//! to the target, the solver moves the rest of the sketch, a point the constraints fix stays put,
//! and one move is one undo step.
use serde_json::{Value, json};
use surfacecraft_engine::Session;

/// A polyline of three free points: (0, 0), (10, 0), (10, 5).
fn polyline(s: &mut Session) {
    s.execute(
        "sketch.create",
        json!({"name": "pl", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "pl", "entities": [{"line": {"from": [0.0, 0.0], "to": [10.0, 0.0]}}, {"line": {"from": [10.0, 0.0], "to": [10.0, 5.0]}}]}),
    )
    .unwrap();
}

fn point(s: &Session, i: usize) -> [f64; 2] {
    let sk = &s.doc.sketches.iter().find(|o| o.name == "pl").expect("sketch").sketch;
    sk.points[i]
}

#[test]
fn a_free_point_goes_exactly_to_the_target() {
    let mut s = Session::new();
    polyline(&mut s);
    s.execute("sketch.move_point", json!({"sketch": "pl", "point": 2, "to": [12.25, 7.5]})).unwrap();
    let p = point(&s, 2);
    assert!((p[0] - 12.25).abs() < 1e-6 && (p[1] - 7.5).abs() < 1e-6, "{p:?}");
}

#[test]
fn a_rectangle_stays_a_rectangle_when_a_corner_is_dragged() {
    let mut s = Session::new();
    s.execute(
        "sketch.create",
        json!({"name": "rect", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "rect", "entities": [{"polygon": {"points": [[0.0, 0.0], [10.0, 0.0], [10.0, 5.0], [0.0, 5.0]]}}]}),
    )
    .unwrap();
    // entities 0..=3 are the sides (0-1 bottom, 1-2 right, 2-3 top, 3-0 left)
    let r = s
        .execute(
            "sketch.constrain",
            json!({"sketch": "rect", "constraints": [
                {"type": "horizontal", "line": 0}, {"type": "horizontal", "line": 2},
                {"type": "vertical", "line": 1}, {"type": "vertical", "line": 3},
                {"type": "fix", "point": 0, "at": [0.0, 0.0]}
            ]}),
        )
        .unwrap();
    assert_eq!(r["report"]["status"], "under_defined", "the width and height are free: {r}");
    s.execute("sketch.move_point", json!({"sketch": "rect", "point": 2, "to": [13.0, 8.0]})).unwrap();
    let sk = &s.doc.sketches[0].sketch;
    let (a, b, c, d) = (sk.points[0], sk.points[1], sk.points[2], sk.points[3]);
    let tol = 1e-9;
    assert!((a[1] - b[1]).abs() < tol && (c[1] - d[1]).abs() < tol, "horizontal sides");
    assert!((b[0] - c[0]).abs() < tol && (d[0] - a[0]).abs() < tol, "vertical sides");
    assert!(a[0].abs() < tol && a[1].abs() < tol, "the fixed corner stays");
}

#[test]
fn a_fixed_point_does_not_move() {
    let mut s = Session::new();
    polyline(&mut s);
    s.execute(
        "sketch.constrain",
        json!({"sketch": "pl", "constraints": [{"type": "fix", "point": 0, "at": [0.0, 0.0]}]}),
    )
    .unwrap();
    s.execute("sketch.move_point", json!({"sketch": "pl", "point": 0, "to": [3.0, 3.0]})).unwrap();
    assert!(point(&s, 0)[0].abs() < 1e-9 && point(&s, 0)[1].abs() < 1e-9);
}

#[test]
fn one_move_is_one_undo_step() {
    let mut s = Session::new();
    polyline(&mut s);
    let before = point(&s, 2);
    s.execute("sketch.move_point", json!({"sketch": "pl", "point": 2, "to": [1.0, 1.0]})).unwrap();
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(point(&s, 2), before);
}

#[test]
fn hostile_parameters_are_errors() {
    let mut s = Session::new();
    polyline(&mut s);
    let hostile = [
        json!({"sketch": "pl", "point": 99, "to": [0.0, 0.0]}),
        json!({"sketch": "pl", "point": -1, "to": [0.0, 0.0]}),
        json!({"sketch": "pl", "point": 0, "to": [0.0]}),
        json!({"sketch": "nope", "point": 0, "to": [0.0, 0.0]}),
        json!({"sketch": "pl", "point": 0}),
    ];
    for v in hostile {
        assert!(s.execute("sketch.move_point", v.clone()).is_err(), "{v} must be refused");
    }
}
