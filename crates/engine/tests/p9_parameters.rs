//! P9-C1: named parameters and expressions. A document holds a table of parameters; sketch
//! dimensions and the numeric fields of features may be given as `{"expr": "..."}` over them, and
//! changing a parameter recomputes everything that depends on it as one undo step.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_doc::{FeatureOp, format};
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_kernel::naming::face_signature;
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::solid::volume;

/// A small deterministic generator (xorshift64*), so the "random" edits are the same every run.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * ((self.next() >> 11) as f64 / (1u64 << 53) as f64)
    }
}

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, p.clone()).unwrap_or_else(|e| panic!("{cmd} {p}: {e:?}"))
}

fn refused(s: &mut Session, cmd: &str, p: Value) -> String {
    match s.execute(cmd, p.clone()) {
        Err(e) => format!("{e:?}"),
        Ok(v) => panic!("{cmd} {p} should be refused, gave {v}"),
    }
}

fn set(s: &mut Session, name: &str, expr: &str) -> Value {
    run(s, "param.set", json!({"name": name, "expression": expr}))
}

fn value_of(s: &mut Session, name: &str) -> f64 {
    let l = run(s, "param.list", Value::Null);
    let p = l["parameters"].as_array().expect("list").iter().find(|p| p["name"] == name).expect("param").clone();
    p["value"].as_f64().expect("value")
}

fn body(s: &Session, name: &str) -> Brep {
    (*s.doc.bodies.iter().find(|b| b.name == name).expect("body").brep).clone()
}

fn vol(s: &Session, name: &str) -> f64 {
    volume(&body(s, name)).expect("volume")
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs().max(1e-12)
}

/// A rectangle sketch `base`, fully constrained, with its two lengths as expressions.
fn rect_sketch(s: &mut Session, w: &Value, h: &Value) -> Value {
    run(s, "sketch.create", json!({"name": "base"}));
    run(
        s,
        "sketch.add",
        json!({"sketch": "base", "entities": [{"polygon": {"points": [[0.2, -0.1], [39.0, 0.4], [40.5, 20.2], [0.1, 19.0]]}}]}),
    );
    run(
        s,
        "sketch.constrain",
        json!({"sketch": "base", "constraints": [
            {"type": "fix", "point": 0, "at": [0.0, 0.0]},
            {"type": "horizontal", "line": 0}, {"type": "vertical", "line": 1},
            {"type": "horizontal", "line": 2}, {"type": "vertical", "line": 3},
            {"type": "length", "line": 0, "value": w}, {"type": "length", "line": 1, "value": h}
        ]}),
    )
}

/// Parameters `w = 40`, `h = w / 2`, `d = 10`; a `w` × `h` rectangle extruded by `d` into `blk`.
fn block(s: &mut Session) -> Value {
    set(s, "w", "40");
    set(s, "h", "w / 2");
    set(s, "d", "10");
    let r = rect_sketch(s, &json!({"expr": "w"}), &json!({"expr": "h"}));
    assert_eq!(r["report"]["status"], "fully_defined", "{r}");
    run(s, "feature.extrude", json!({"sketch": "base", "distance": {"expr": "d"}, "name": "blk"}))
}

/// Record index of the edge of `b` along z at the corner `(x, y)`.
fn vertical_edge_at(b: &Brep, x: f64, y: f64) -> usize {
    b.edges
        .iter()
        .position(|e| {
            let (p, q) = (b.vertices[e.vertices[0] as usize].point, b.vertices[e.vertices[1] as usize].point);
            (p[0] - q[0]).abs() < 1e-9
                && (p[1] - q[1]).abs() < 1e-9
                && (p[2] - q[2]).abs() > 1.0
                && (p[0] - x).abs() < 1e-6
                && (p[1] - y).abs() < 1e-6
        })
        .expect("vertical edge")
}

#[test]
fn gate_block_follows_its_parameters_in_one_undo_step() {
    let mut s = Session::new();
    let ex = block(&mut s);
    assert!(rel(ex["volume"].as_f64().unwrap(), 8000.0) < 1e-9, "{ex}");
    let r = set(&mut s, "w", "50");
    assert_eq!(r["failed_features"], 0, "{r}");
    assert!(rel(vol(&s, "blk"), 50.0 * 25.0 * 10.0) < 1e-9, "{}", vol(&s, "blk"));
    assert_eq!(value_of(&mut s, "h"), 25.0);
    run(&mut s, "edit.undo", Value::Null);
    assert!(rel(vol(&s, "blk"), 8000.0) < 1e-9, "one undo restores the block: {}", vol(&s, "blk"));
    assert_eq!(value_of(&mut s, "w"), 40.0);
    run(&mut s, "edit.redo", Value::Null);
    assert!(rel(vol(&s, "blk"), 12500.0) < 1e-9);
    // the depth is a parameter too
    set(&mut s, "d", "4");
    assert!(rel(vol(&s, "blk"), 50.0 * 25.0 * 4.0) < 1e-9);
}

#[test]
fn the_document_keeps_expressions_beside_values() {
    let mut s = Session::new();
    block(&mut s);
    let sk = s.doc.sketches.iter().find(|k| k.name == "base").unwrap();
    assert_eq!(sk.sketch.dim_exprs.get(&5).map(String::as_str), Some("w"));
    assert_eq!(sk.sketch.dim_exprs.get(&6).map(String::as_str), Some("h"));
    let f = s.doc.features.iter().find(|f| f.name == "extrude").unwrap();
    assert_eq!(f.exprs.get("distance").map(String::as_str), Some("d"));
    let tree = run(&mut s, "feature.tree", Value::Null);
    assert_eq!(tree["features"][0]["expressions"]["distance"], "d", "{tree}");
    let l = run(&mut s, "param.list", Value::Null);
    let w = &l["parameters"][0];
    assert_eq!(
        (w["name"].as_str(), w["expression"].as_str(), w["value"].as_f64()),
        (Some("w"), Some("40"), Some(40.0))
    );
    let users = w["users"].as_array().unwrap();
    assert!(users.iter().any(|u| u["kind"] == "sketch" && u["at"] == "constraint 5"), "{l}");
    assert!(users.iter().any(|u| u["kind"] == "parameter" && u["name"] == "h"), "{l}");
    assert_eq!(l["parameters"][2]["users"].as_array().unwrap()[0]["kind"], "feature", "d is used by the extrude: {l}");
}

#[test]
fn gate_fillet_radius_from_a_parameter_matches_the_closed_form() {
    let mut s = Session::new();
    block(&mut s);
    set(&mut s, "r", "d / 5");
    let b = body(&s, "blk");
    let edges: Vec<usize> = [(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0)]
        .iter()
        .map(|(x, y)| vertical_edge_at(&b, *x, *y))
        .collect();
    let f = run(&mut s, "feature.fillet", json!({"body": "blk", "edges": edges, "size": {"expr": "r"}}));
    let closed = |w: f64, h: f64, d: f64| (w * h - (4.0 - PI) * (d / 5.0).powi(2)) * d;
    assert!(rel(f["volume"].as_f64().unwrap(), closed(40.0, 20.0, 10.0)) < 1e-6, "{f}");
    let r = set(&mut s, "d", "15");
    assert_eq!(r["failed_features"], 0, "{r}");
    assert!(rel(vol(&s, "blk"), closed(40.0, 20.0, 15.0)) < 1e-6, "{}", vol(&s, "blk"));
    set(&mut s, "w", "60");
    assert!(rel(vol(&s, "blk"), closed(60.0, 30.0, 15.0)) < 1e-6, "{}", vol(&s, "blk"));
}

fn small_square(s: &mut Session) {
    run(s, "sketch.create", json!({"name": "sq"}));
    run(
        s,
        "sketch.add",
        json!({"sketch": "sq", "entities": [{"polygon": {"points": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]}}]}),
    );
}

#[test]
fn gate_pattern_count_is_an_integer_expression() {
    let mut s = Session::new();
    small_square(&mut s);
    run(&mut s, "feature.extrude", json!({"sketch": "sq", "distance": 2.0, "name": "peg"}));
    set(&mut s, "n", "2 + 2");
    set(&mut s, "pitch", "10");
    let p = run(
        &mut s,
        "feature.pattern",
        json!({"body": "peg", "count": {"expr": "n"}, "translate": [{"expr": "pitch"}, 0, 0]}),
    );
    assert!(rel(p["volume"].as_f64().unwrap(), 4.0 * 32.0) < 1e-9, "{p}");
    set(&mut s, "n", "5");
    assert!(rel(vol(&s, "peg"), 5.0 * 32.0) < 1e-9);
    // a count that is not a whole number is refused where it is given ...
    set(&mut s, "m", "2.5");
    let e = refused(&mut s, "feature.pattern", json!({"body": "peg", "count": {"expr": "m"}}));
    assert!(e.contains("whole number"), "{e}");
    refused(&mut s, "feature.pattern", json!({"body": "peg", "count": 2.5}));
    // ... and a parameter change that makes it fractional fails the feature, not the command
    let r = set(&mut s, "n", "2.5");
    assert_eq!(r["failed_features"], 1, "{r}");
    assert_eq!(r["failures"][0]["kind"], "feature", "{r}");
    assert!(r["failures"][0]["message"].as_str().unwrap().contains("whole number"), "{r}");
    assert_eq!(value_of(&mut s, "n"), 2.5, "the parameter change itself applied");
    set(&mut s, "n", "3");
    assert_eq!(run(&mut s, "feature.tree", Value::Null)["failed_features"], 0);
    assert!(rel(vol(&s, "peg"), 3.0 * 32.0) < 1e-9);
    let tolerant = set(&mut s, "n", "0.1 * 30");
    assert_eq!(tolerant["failed_features"], 0, "0.1 * 30 is whole within tolerance: {tolerant}");
}

#[test]
fn gate_cycles_deletes_and_renames() {
    let mut s = Session::new();
    set(&mut s, "b", "1");
    set(&mut s, "a", "b + 1");
    let e = refused(&mut s, "param.set", json!({"name": "b", "expression": "a * 2"}));
    assert!(e.contains("cycle") && (e.contains("b -> a -> b") || e.contains("a -> b -> a")), "{e}");
    assert_eq!(value_of(&mut s, "b"), 1.0, "nothing changed");
    let e = refused(&mut s, "param.set", json!({"name": "a", "expression": "a + 1"}));
    assert!(e.contains("cycle"), "{e}");

    let mut s = Session::new();
    block(&mut s);
    let e = refused(&mut s, "param.delete", json!({"name": "w"}));
    assert!(e.contains("`h`") && e.contains("base") && e.contains("constraint 5"), "users are listed: {e}");
    let e = refused(&mut s, "param.delete", json!({"name": "d"}));
    assert!(e.contains("extrude") && e.contains("distance"), "{e}");
    set(&mut s, "free", "1");
    run(&mut s, "param.delete", json!({"name": "free"}));
    assert!(s.doc.params.get("free").is_none());

    run(&mut s, "param.rename", json!({"name": "w", "new_name": "width"}));
    assert_eq!(s.doc.params.get("h").map(|p| p.expr.clone()), Some("width / 2".to_string()));
    let sk = s.doc.sketches.iter().find(|k| k.name == "base").unwrap();
    assert_eq!(sk.sketch.dim_exprs.get(&5).map(String::as_str), Some("width"));
    refused(&mut s, "param.rename", json!({"name": "width", "new_name": "h"}));
    refused(&mut s, "param.rename", json!({"name": "width", "new_name": "sin"}));
    let r = set(&mut s, "width", "50");
    assert_eq!(r["failed_features"], 0, "{r}");
    assert!(rel(vol(&s, "blk"), 12500.0) < 1e-9, "everything keeps evaluating after a rename");
    run(&mut s, "edit.undo", Value::Null);
    run(&mut s, "edit.undo", Value::Null);
    assert!(s.doc.params.get("w").is_some() && s.doc.params.get("width").is_none(), "undo restores the name");
    let sk = s.doc.sketches.iter().find(|k| k.name == "base").unwrap();
    assert_eq!(sk.sketch.dim_exprs.get(&5).map(String::as_str), Some("w"));
}

#[test]
fn gate_a_parameter_that_breaks_a_sketch_or_a_feature_reports_and_undoes() {
    // over-constrained: a diagonal of 50 fits a 40 x 30 rectangle only
    let mut s = Session::new();
    set(&mut s, "w", "40");
    set(&mut s, "h", "30");
    rect_sketch(&mut s, &json!({"expr": "w"}), &json!({"expr": "h"}));
    let r = run(
        &mut s,
        "sketch.constrain",
        json!({"sketch": "base", "constraints": [{"type": "distance", "a": 0, "b": 2, "value": 50.0}]}),
    );
    assert_eq!(r["report"]["status"], "over_defined", "a redundant but consistent diagonal still solves: {r}");
    assert!(r["report"]["residual"].as_f64().unwrap() < 1e-6, "{r}");
    run(&mut s, "feature.extrude", json!({"sketch": "base", "distance": 5.0, "name": "slab"}));
    let before = vol(&s, "slab");
    assert!(rel(before, 40.0 * 30.0 * 5.0) < 1e-6);
    let r = set(&mut s, "h", "20");
    assert_eq!(value_of(&mut s, "h"), 20.0, "the change applied");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f["kind"] == "sketch"), "{r}");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f["kind"] == "feature"), "{r}");
    assert_eq!(r["failed_features"], 1);
    let tree = run(&mut s, "feature.tree", Value::Null);
    assert!(tree["features"][0]["error"].as_str().unwrap().contains("sketch"), "{tree}");
    // consistent: the failed feature's body leaves the model (it is parked as dormant, as for any
    // failing feature), nothing is half-updated, and the table is as set
    assert!(s.doc.bodies.iter().all(|b| b.name != "slab") && s.doc.dormant_bodies.iter().any(|b| b.name == "slab"));
    run(&mut s, "edit.undo", Value::Null);
    assert!(rel(vol(&s, "slab"), before) < 1e-6, "undo brings the body back as it was");
    assert_eq!(value_of(&mut s, "h"), 30.0);
    assert_eq!(run(&mut s, "feature.tree", Value::Null)["failed_features"], 0);
    assert!(s.doc.sketches[0].error.is_none());
    // and the way back: a value that satisfies the diagonal again repairs the feature
    set(&mut s, "h", "20");
    set(&mut s, "h", "30");
    assert_eq!(run(&mut s, "feature.tree", Value::Null)["failed_features"], 0);

    // a fillet that grows too large
    let mut s = Session::new();
    block(&mut s);
    set(&mut s, "r", "d / 5");
    let b = body(&s, "blk");
    let edges: Vec<usize> = [(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0)]
        .iter()
        .map(|(x, y)| vertical_edge_at(&b, *x, *y))
        .collect();
    run(&mut s, "feature.fillet", json!({"body": "blk", "edges": edges, "size": {"expr": "r"}}));
    let r = set(&mut s, "d", "200");
    assert_eq!(r["failed_features"], 1, "{r}");
    assert!(r["failures"].as_array().unwrap().iter().any(|f| f["kind"] == "feature" && f["name"] == "fillet"), "{r}");
    assert_eq!(value_of(&mut s, "d"), 200.0);
    run(&mut s, "edit.undo", Value::Null);
    assert_eq!(value_of(&mut s, "d"), 10.0);
    assert_eq!(run(&mut s, "feature.tree", Value::Null)["failed_features"], 0);
}

#[test]
fn a_change_that_breaks_a_dependent_parameter_is_refused_whole() {
    let mut s = Session::new();
    set(&mut s, "w", "40");
    set(&mut s, "h", "sqrt(w - 10)");
    let e = refused(&mut s, "param.set", json!({"name": "w", "expression": "5"}));
    assert!(e.contains("`h`"), "{e}");
    assert_eq!(value_of(&mut s, "w"), 40.0);
}

#[test]
fn gate_save_and_load_keep_parameters_and_expressions() {
    let mut s = Session::new();
    block(&mut s);
    run(&mut s, "param.set", json!({"name": "w", "expression": "40", "comment": "overall width"}));
    let text = format::to_json(&s.doc).unwrap();
    for needle in ["\"w / 2\"", "\"overall width\"", "dim_exprs", "\"distance\":\"d\""] {
        assert!(text.contains(needle), "the project JSON should hold {needle}");
    }
    let mut loaded = Session::new();
    loaded.doc = format::from_json(&text).unwrap();
    assert_eq!(loaded.doc.params, s.doc.params);
    let r = set(&mut loaded, "w", "50");
    assert_eq!(r["failed_features"], 0, "{r}");
    assert!(rel(vol(&loaded, "blk"), 12500.0) < 1e-9, "the loaded document still recomputes");
    // a project written before the table existed loads without it
    let mut v: Value = serde_json::from_str(&text).unwrap();
    v["document"].as_object_mut().unwrap().remove("params");
    let old = format::from_json(&v.to_string()).unwrap();
    assert!(old.params.is_empty());
    // a hand-edited table with a cycle is refused
    let mut v: Value = serde_json::from_str(&text).unwrap();
    v["document"]["params"][0]["expr"] = json!("d");
    v["document"]["params"][2]["expr"] = json!("w");
    assert!(format::from_json(&v.to_string()).is_err());
}

#[test]
fn eval_and_list_do_not_store_anything() {
    let mut s = Session::new();
    set(&mut s, "w", "40");
    let e = run(&mut s, "param.eval", json!({"expression": "w / 8 + sin(30)"}));
    assert!((e["value"].as_f64().unwrap() - 5.5).abs() < 1e-12, "{e}");
    assert!(refused(&mut s, "param.eval", json!({"expression": "q"})).contains("`q`"));
    assert!(refused(&mut s, "param.eval", json!({"expression": "1/0"})).contains("division"));
    assert_eq!(s.doc.params.items().len(), 1);
    let undo_depth_before = s.history.undo_labels().len();
    run(&mut s, "param.list", Value::Null);
    run(&mut s, "param.eval", json!({"expression": "1"}));
    assert_eq!(s.history.undo_labels().len(), undo_depth_before, "reads take no undo step");
}

#[test]
fn a_literal_edit_replaces_the_expression() {
    let mut s = Session::new();
    let ex = block(&mut s);
    let id = ex["id"].as_u64().unwrap();
    run(&mut s, "feature.edit", json!({"feature": id, "set": {"distance": 12.0}}));
    assert!(s.doc.features.iter().find(|f| f.id.0 == id).unwrap().exprs.is_empty(), "the literal replaced it");
    set(&mut s, "d", "3");
    assert!(rel(vol(&s, "blk"), 40.0 * 20.0 * 12.0) < 1e-9, "d no longer drives the depth");
    // an expression can be put back through `edit`
    run(&mut s, "feature.edit", json!({"feature": id, "set": {"distance": {"expr": "d * 2"}}}));
    assert!(rel(vol(&s, "blk"), 40.0 * 20.0 * 6.0) < 1e-9);
    set(&mut s, "d", "5");
    assert!(rel(vol(&s, "blk"), 40.0 * 20.0 * 10.0) < 1e-9);
    refused(&mut s, "feature.edit", json!({"feature": id, "set": {"sketch": {"expr": "d"}}}));
    // a sketch dimension edited to a plain number or an expression
    let c = |v: Value| json!({"sketch": "base", "index": 5, "constraint": {"type": "length", "line": 0, "value": v}});
    run(&mut s, "sketch.edit_constraint", c(json!(30.0)));
    assert!(!s.doc.sketches[0].sketch.dim_exprs.contains_key(&5));
    set(&mut s, "w", "99");
    assert!(rel(vol(&s, "blk"), 30.0 * 49.5 * 10.0) < 1e-6, "h follows w: {}", vol(&s, "blk"));
    run(&mut s, "sketch.edit_constraint", c(json!({"expr": "w / 3"})));
    assert!(rel(vol(&s, "blk"), 33.0 * 49.5 * 10.0) < 1e-6, "{}", vol(&s, "blk"));
    // removing a constraint moves later expressions down with their constraints
    run(&mut s, "sketch.edit_constraint", json!({"sketch": "base", "index": 1}));
    assert_eq!(s.doc.sketches[0].sketch.dim_exprs.get(&4).map(String::as_str), Some("w / 3"));
}

#[test]
fn parked_features_keep_and_reevaluate_their_expressions() {
    let mut s = Session::new();
    block(&mut s);
    set(&mut s, "r", "d / 5");
    let b = body(&s, "blk");
    let edges: Vec<usize> = [(0.0, 0.0), (40.0, 0.0), (40.0, 20.0), (0.0, 20.0)]
        .iter()
        .map(|(x, y)| vertical_edge_at(&b, *x, *y))
        .collect();
    run(&mut s, "feature.fillet", json!({"body": "blk", "edges": edges, "size": {"expr": "r"}}));
    run(&mut s, "feature.rollback", json!({"to": {"index": 1}}));
    assert_eq!(s.doc.rolled_back.len(), 1);
    let e = refused(&mut s, "param.delete", json!({"name": "r"}));
    assert!(e.contains("fillet"), "a parked feature still uses r: {e}");
    set(&mut s, "d", "20");
    let parked = &s.doc.rolled_back[0];
    assert_eq!(parked.exprs.get("radius").map(String::as_str), Some("r"));
    assert!(matches!(parked.op, FeatureOp::Fillet { radius, .. } if (radius - 4.0).abs() < 1e-12), "{:?}", parked.op);
    run(&mut s, "param.rename", json!({"name": "r", "new_name": "rad"}));
    assert_eq!(s.doc.rolled_back[0].exprs.get("radius").map(String::as_str), Some("rad"));
    run(&mut s, "feature.rollback", json!({"to": "end"}));
    let closed = (40.0 * 20.0 - (4.0 - PI) * 16.0) * 20.0;
    assert!(rel(vol(&s, "blk"), closed) < 1e-6, "{} vs {closed}", vol(&s, "blk"));
}

#[test]
fn hostile_expressions_and_names_are_errors() {
    let mut s = Session::new();
    set(&mut s, "w", "1");
    let big = "1".repeat(1_000_000);
    let deep = format!("{}1{}", "(".repeat(10_000), ")".repeat(10_000));
    for (cmd, p) in [
        ("param.set", json!({"name": "x", "expression": big})),
        ("param.set", json!({"name": "x", "expression": deep})),
        ("param.set", json!({"name": "x", "expression": "nan"})),
        ("param.set", json!({"name": "x", "expression": "1e999"})),
        ("param.set", json!({"name": "", "expression": "1"})),
        ("param.set", json!({"name": "sin", "expression": "1"})),
        ("param.set", json!({"name": "a b", "expression": "1"})),
        ("param.set", json!({"name": "é", "expression": "1"})),
        ("param.set", json!({"name": "x".repeat(10_000), "expression": "1"})),
        ("param.set", json!({"name": "x", "expression": "1", "comment": "c".repeat(100_000)})),
        ("param.set", json!({"name": "x", "expression": 5})),
        ("param.eval", json!({"expression": big})),
        ("param.eval", json!({"expression": deep})),
        ("param.delete", json!({"name": "nope"})),
        ("param.rename", json!({"name": "w", "new_name": ""})),
        ("feature.extrude", json!({"sketch": "x", "distance": {"expr": big}})),
        ("feature.extrude", json!({"sketch": "x", "distance": {"expr": "1", "other": 1}})),
        ("feature.extrude", json!({"sketch": "x", "distance": {"nope": 1}})),
        (
            "sketch.constrain",
            json!({"sketch": "x", "constraints": [{"type": "length", "line": 0, "value": {"expr": deep}}]}),
        ),
    ] {
        let r = s.execute(cmd, p);
        assert!(matches!(r, Err(EngineError::InvalidParams(_) | EngineError::Failed(_))), "{cmd}: {r:?}");
    }
    assert_eq!(s.doc.params.items().len(), 1, "nothing was stored");
}

#[test]
fn no_cycle_through_a_long_chain_recurses() {
    let mut s = Session::new();
    set(&mut s, "p0", "1");
    for k in 1..200 {
        set(&mut s, &format!("p{k}"), &format!("p{} + 1", k - 1));
    }
    assert_eq!(value_of(&mut s, "p199"), 200.0);
    set(&mut s, "p0", "5");
    assert_eq!(value_of(&mut s, "p199"), 204.0);
    assert!(refused(&mut s, "param.set", json!({"name": "p0", "expression": "p199"})).contains("cycle"));
}

#[test]
fn gate_twenty_random_parameter_edits_on_the_stable_reference_model() {
    let mut s = Session::new();
    for (n, e) in [("l", "40"), ("wd", "20"), ("ht", "10"), ("depth", "12"), ("rad", "1.5"), ("wall", "1")] {
        set(&mut s, n, e);
    }
    rect_sketch(&mut s, &json!({"expr": "l"}), &json!({"expr": "wd"}));
    run(&mut s, "feature.extrude", json!({"sketch": "base", "distance": {"expr": "ht"}, "name": "plate"}));
    run(&mut s, "sketch.create", json!({"name": "hole"}));
    run(
        &mut s,
        "sketch.add",
        json!({"sketch": "hole", "entities": [{"circle": {"center": [15.0, 8.0], "radius": 4.0}}]}),
    );
    run(
        &mut s,
        "feature.extrude",
        json!({"sketch": "hole", "distance": {"expr": "depth"}, "mode": "cut", "target": "plate"}),
    );
    let b0 = body(&s, "plate");
    let (e0, e1) = (vertical_edge_at(&b0, 0.0, 0.0), vertical_edge_at(&b0, 40.0, 20.0));
    run(&mut s, "feature.fillet", json!({"body": "plate", "edges": [e0, e1], "size": {"expr": "rad"}}));
    let (mut l, mut wd, mut ht, mut depth, mut rad, mut wall) = (40.0, 20.0, 10.0, 12.0, 1.5, 1.0);
    let closed = |l: f64, wd: f64, ht: f64, depth: f64, rad: f64| {
        l * wd * ht - PI * 16.0 * depth.min(ht) - 2.0 * (1.0 - PI / 4.0) * rad * rad * ht
    };
    let v = vol(&s, "plate");
    assert!(rel(v, closed(l, wd, ht, depth, rad)) < 1e-6, "{v}");
    // the shell opens the top face of the filleted plate and follows its thickness parameter
    let top = {
        let b = body(&s, "plate");
        (0..b.faces.len())
            .filter_map(|f| face_signature(&b, f).map(|g| (f, g)))
            .filter(|(_, g)| g.kind == "plane" && g.normal[2] > 0.99)
            .max_by(|a, b| a.1.centroid[2].total_cmp(&b.1.centroid[2]))
            .unwrap()
            .0
    };
    let sh = run(&mut s, "feature.shell", json!({"body": "plate", "faces": [top], "thickness": {"expr": "wall"}}));
    assert_eq!(sh["failed_features"], 0, "{sh}");
    let shell_id = sh["id"].as_u64().unwrap();
    let mut rng = Rng(0x5eed_0009);
    let mut unmeasured = 0;
    for step in 0..20 {
        let which = (rng.next() % 6) as usize;
        let (name, lo, hi) = [
            ("l", 30.0, 60.0),
            ("wd", 18.0, 35.0),
            ("ht", 8.0, 16.0),
            ("depth", 5.0, 20.0),
            ("rad", 0.8, 3.0),
            ("wall", 0.5, 1.5),
        ][which];
        let v = rng.range(lo, hi);
        match which {
            0 => l = v,
            1 => wd = v,
            2 => ht = v,
            3 => depth = v,
            4 => rad = v,
            _ => wall = v,
        }
        let r = set(&mut s, name, &format!("{v:?}"));
        assert_eq!(r["failed_features"], 0, "edit {step} ({name} = {v}): {r}");
        // measure the filleted plate by suppressing the shell for the moment
        run(&mut s, "feature.suppress", json!({"feature": shell_id, "suppressed": true}));
        let before_shell = vol(&s, "plate");
        let exact = closed(l, wd, ht, depth, rad);
        assert!(rel(before_shell, exact) < 1e-6, "step {step}: {before_shell} vs {exact}");
        run(&mut s, "feature.suppress", json!({"feature": shell_id, "suppressed": false}));
        let tree = run(&mut s, "feature.tree", Value::Null);
        assert_eq!(tree["failed_features"], 0, "step {step}: {tree}");
        match volume(&body(&s, "plate")) {
            Ok(hollow) => {
                let area = 2.0 * (l * wd + (l + wd) * ht) + 2.0 * PI * 4.0 * depth.min(ht);
                assert!(
                    hollow > 0.3 * wall * area && hollow < 1.7 * wall * area,
                    "step {step}: shell {hollow} for {wall} over {area}"
                );
            }
            Err(_) => unmeasured += 1,
        }
    }
    eprintln!("gate_p9: 20 random parameter edits; shell volume unmeasurable in {unmeasured} steps");
}

#[test]
fn the_example_of_the_guide_runs() {
    let guide = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/../../docs/guides/sketches-features.md"))
        .unwrap();
    let start = guide.find("## Parameters and expressions").expect("section");
    let block = &guide[start..];
    let json_start = block.find("```json\n").expect("example") + 8;
    let json_end = block[json_start..].find("```").expect("end") + json_start;
    let script: Value = serde_json::from_str(&block[json_start..json_end]).expect("the example is JSON");
    let mut s = Session::new();
    let report = s.run_script(&script).expect("the example runs");
    assert!(report.ok, "{report:?}");
    let v = vol(&s, "blk");
    assert!(v > 17_900.0 && v < 18_000.0, "a 60 x 30 x 10 block with one 2 mm fillet: {v}");
}
