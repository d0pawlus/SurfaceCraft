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

/// One line from (0, 0) to (10, 0) with its length fixed to 10.
fn length_line(s: &mut Session) {
    polyline(s);
    s.execute("sketch.constrain", json!({"sketch": "pl", "constraints": [{"type": "length", "line": 0, "value": 10.0}]}))
        .unwrap();
}

fn line_length(s: &Session) -> f64 {
    let (a, b) = (point(s, 0), point(s, 1));
    (a[0] - b[0]).hypot(a[1] - b[1])
}

#[test]
fn a_move_that_collapses_a_dimensioned_line_never_commits_a_broken_sketch() {
    let mut s = Session::new();
    length_line(&mut s);
    let before = s.doc.sketches[0].sketch.clone();
    let r = s.execute("sketch.move_point", json!({"sketch": "pl", "point": 1, "to": [0.0, 0.0]}));
    match r {
        Err(_) => assert_eq!(s.doc.sketches[0].sketch, before, "a refused move changes nothing"),
        Ok(v) => {
            assert!(v["report"]["residual"].as_f64().unwrap() <= 1e-6, "{v}");
            assert!((line_length(&s) - 10.0).abs() < 1e-9, "the length holds: {}", line_length(&s));
        }
    }
}

#[test]
fn a_move_onto_the_degenerate_position_gives_the_nearest_valid_sketch() {
    let mut s = Session::new();
    length_line(&mut s);
    s.execute("sketch.move_point", json!({"sketch": "pl", "point": 1, "to": [0.0, 0.0]})).unwrap();
    assert!((line_length(&s) - 10.0).abs() < 1e-9, "the length holds: {}", line_length(&s));
    let p = point(&s, 1);
    assert!(p[0].hypot(p[1]) < 3.0 || (point(&s, 0)[0] - p[0]).hypot(point(&s, 0)[1] - p[1]) > 9.0, "{p:?}");
}

#[test]
fn contradicting_dimensions_refuse_the_move_and_leave_the_document_alone() {
    let mut s = Session::new();
    length_line(&mut s);
    // a second length on the same line that contradicts the first cannot be met
    let added = s.execute(
        "sketch.constrain",
        json!({"sketch": "pl", "constraints": [{"type": "length", "line": 0, "value": 12.0}]}),
    );
    assert!(added.is_ok(), "{added:?}");
    let before = s.doc.sketches[0].sketch.clone();
    let r = s.execute("sketch.move_point", json!({"sketch": "pl", "point": 2, "to": [11.0, 6.0]}));
    assert!(r.is_err(), "an unsatisfied sketch is never committed: {r:?}");
    assert_eq!(s.doc.sketches[0].sketch, before);
}

#[test]
fn redundant_but_satisfied_constraints_still_allow_a_move() {
    let mut s = Session::new();
    length_line(&mut s);
    s.execute(
        "sketch.constrain",
        json!({"sketch": "pl", "constraints": [{"type": "length", "line": 0, "value": 10.0}]}),
    )
    .unwrap();
    s.execute("sketch.move_point", json!({"sketch": "pl", "point": 2, "to": [12.0, 7.0]})).unwrap();
    assert!((line_length(&s) - 10.0).abs() < 1e-9);
}
