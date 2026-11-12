//! P9-C1 review round 2: edits that keep intent, what `doc.open` reports, loading that is stable.
use serde_json::{Value, json};
use surfacecraft_doc::format;
use surfacecraft_engine::Session;
use surfacecraft_kernel::solid::volume;

fn run(s: &mut Session, cmd: &str, p: Value) -> Value {
    s.execute(cmd, p.clone()).unwrap_or_else(|e| panic!("{cmd} {p}: {e:?}"))
}

fn set(s: &mut Session, name: &str, expr: &str) -> Value {
    run(s, "param.set", json!({"name": name, "expression": expr}))
}

fn vol(s: &Session, name: &str) -> f64 {
    volume(&s.doc.bodies.iter().find(|b| b.name == name).expect("body").brep).expect("volume")
}

fn square(s: &mut Session) {
    run(s, "sketch.create", json!({"name": "sq"}));
    run(
        s,
        "sketch.add",
        json!({"sketch": "sq", "entities": [{"polygon": {"points": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]}}]}),
    );
}

fn open_text(s: &mut Session, text: &str) -> Value {
    let path = std::env::temp_dir().join(format!("p9r_{}_{}.scproj", std::process::id(), text.len()));
    std::fs::write(&path, text).unwrap();
    let r = run(s, "doc.open", json!({"path": path.to_str().unwrap()}));
    let _ = std::fs::remove_file(&path);
    r
}

/// A peg with `translate = [a, b, 0]` as expressions, patterned.
fn patterned(s: &mut Session) -> u64 {
    set(s, "a", "10");
    set(s, "b", "20");
    square(s);
    run(s, "feature.extrude", json!({"sketch": "sq", "distance": 2.0, "name": "peg"}));
    let p =
        run(s, "feature.pattern", json!({"body": "peg", "count": 2, "translate": [{"expr": "a"}, {"expr": "b"}, 0]}));
    p["id"].as_u64().unwrap()
}

#[test]
fn editing_one_component_keeps_the_expressions_of_the_others() {
    let mut s = Session::new();
    let id = patterned(&mut s);
    let r = run(&mut s, "feature.edit", json!({"feature": id, "set": {"translate": [10.0, 20.0, 0.0]}}));
    assert!(r.get("expressions_cleared").is_none(), "nothing changed, nothing lost: {r}");
    let r = run(&mut s, "feature.edit", json!({"feature": id, "set": {"translate": [10.0, 20.0, 5.0]}}));
    let f = s.doc.features.iter().find(|f| f.id.0 == id).unwrap();
    assert_eq!(f.exprs.get("translate.0").map(String::as_str), Some("a"));
    assert_eq!(f.exprs.get("translate.1").map(String::as_str), Some("b"));
    assert!(r.get("expressions_cleared").is_none(), "{r}");
    // a component that really changed loses its expression, and the result says so
    let r = run(&mut s, "feature.edit", json!({"feature": id, "set": {"translate": [11.0, 20.0, 5.0]}}));
    assert_eq!(r["expressions_cleared"], json!([{"field": "translate.0", "expression": "a"}]), "{r}");
    let f = s.doc.features.iter().find(|f| f.id.0 == id).unwrap();
    assert!(!f.exprs.contains_key("translate.0") && f.exprs.contains_key("translate.1"));
    // a scalar literal replaces its expression and is reported
    let ex = s.doc.features.iter().find(|f| f.name == "extrude").unwrap().id.0;
    set(&mut s, "d", "3");
    run(&mut s, "feature.edit", json!({"feature": ex, "set": {"distance": {"expr": "d"}}}));
    let r = run(&mut s, "feature.edit", json!({"feature": ex, "set": {"distance": 7.0}}));
    assert_eq!(r["expressions_cleared"], json!([{"field": "distance", "expression": "d"}]), "{r}");
}

#[test]
fn doc_open_reports_a_recompute_and_what_fails_after_it() {
    let mut s = Session::new();
    patterned(&mut s);
    let saved = format::to_json(&s.doc).unwrap();
    let mut t = Session::new();
    let r = open_text(&mut t, &saved);
    assert_eq!(r["recomputed"], false, "{r}");
    assert_eq!(r["failures"], json!([]));
    // stale and fine
    let mut v: Value = serde_json::from_str(&saved).unwrap();
    v["document"]["params"][0]["expr"] = json!("12");
    let r = open_text(&mut Session::new(), &v.to_string());
    assert_eq!(r["recomputed"], true, "{r}");
    assert_eq!(r["failures"], json!([]));
    // stale and failing: a fractional count
    set(&mut s, "n", "2");
    let mut s = Session::new();
    set(&mut s, "n", "2");
    square(&mut s);
    run(&mut s, "feature.extrude", json!({"sketch": "sq", "distance": 2.0, "name": "peg"}));
    run(&mut s, "feature.pattern", json!({"body": "peg", "count": {"expr": "n"}, "translate": [10, 0, 0]}));
    let mut v: Value = serde_json::from_str(&format::to_json(&s.doc).unwrap()).unwrap();
    v["document"]["params"][0]["expr"] = json!("2.5");
    let r = open_text(&mut Session::new(), &v.to_string());
    assert_eq!(r["recomputed"], true);
    assert_eq!(r["failures"][0]["kind"], "feature", "{r}");
    assert!(r["failures"][0]["message"].as_str().unwrap().contains("whole number"), "{r}");
    assert_eq!(r["failed_features"], 1);
}

#[test]
fn non_trivial_floats_round_trip_without_a_recompute() {
    let mut s = Session::new();
    set(&mut s, "w", "100 / 3");
    set(&mut s, "k", "sin(30) + sqrt(2) * 10");
    set(&mut s, "q", "w * k / 7");
    square(&mut s);
    run(&mut s, "feature.extrude", json!({"sketch": "sq", "distance": {"expr": "q / 30"}, "name": "peg"}));
    let text = format::to_json(&s.doc).unwrap();
    let doc = format::from_json(&text).unwrap();
    assert!(!doc.needs_recompute);
    assert_eq!(format::to_json(&doc).unwrap(), text);
    let mut t = Session::new();
    let r = open_text(&mut t, &text);
    assert_eq!(r["recomputed"], false);
    assert_eq!(format::to_json(&t.doc).unwrap(), text, "opening changes nothing");
}

/// A failed feature, a parked feature with expressions and a dormant body survive save and open.
#[test]
fn a_project_with_failures_parked_features_and_dormant_bodies_reopens_identically() {
    let mut s = Session::new();
    set(&mut s, "n", "2");
    set(&mut s, "nx", "1");
    set(&mut s, "k", "2");
    square(&mut s);
    run(&mut s, "feature.extrude", json!({"sketch": "sq", "distance": 2.0, "name": "peg"}));
    run(&mut s, "feature.pattern", json!({"body": "peg", "count": {"expr": "n"}, "translate": [10, 0, 0]}));
    run(
        &mut s,
        "feature.mirror",
        json!({"body": "peg", "plane_origin": [0, 0, 0], "plane_normal": [{"expr": "nx"}, 0, 0]}),
    );
    run(&mut s, "feature.rollback", json!({"to": {"index": 2}}));
    assert_eq!(s.doc.rolled_back.len(), 1);
    // a body whose only feature fails is dormant; the pattern fails too
    run(&mut s, "sketch.create", json!({"name": "sq2"}));
    run(
        &mut s,
        "sketch.add",
        json!({"sketch": "sq2", "entities": [{"polygon": {"points": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]}}]}),
    );
    run(&mut s, "feature.extrude", json!({"sketch": "sq2", "distance": {"expr": "1 / (k - 1)"}, "name": "slab"}));
    set(&mut s, "k", "1");
    set(&mut s, "n", "2.5");
    assert_eq!(s.doc.features.iter().filter(|f| f.error.is_some()).count(), 2);
    assert_eq!(s.doc.dormant_bodies.len(), 1, "slab");
    let text = format::to_json(&s.doc).unwrap();
    let before = (s.doc.features.len(), s.doc.rolled_back.len(), s.doc.dormant_bodies.len());
    let mut first = Session::new();
    let r1 = open_text(&mut first, &text);
    let mut second = Session::new();
    let r2 = open_text(&mut second, &text);
    assert_eq!(r1, r2, "deterministic");
    for t in [&first, &second] {
        assert_eq!((t.doc.features.len(), t.doc.rolled_back.len(), t.doc.dormant_bodies.len()), before);
        assert_eq!(t.doc.features.iter().filter(|f| f.error.is_some()).count(), 2, "the same failures");
        assert_eq!(t.doc.rolled_back[0].exprs.get("plane_normal.0").map(String::as_str), Some("nx"));
    }
    assert_eq!(format::to_json(&first.doc).unwrap(), format::to_json(&second.doc).unwrap());
    assert!(vol(&first, "peg") > 0.0);
}

#[test]
fn a_document_with_a_broken_expression_can_be_repaired() {
    let mut s = Session::new();
    set(&mut s, "d", "10");
    set(&mut s, "free", "1");
    square(&mut s);
    let ex = run(&mut s, "feature.extrude", json!({"sketch": "sq", "distance": {"expr": "d"}, "name": "peg"}));
    let id = ex["id"].as_u64().unwrap();
    s.doc.features[0].exprs.insert("distance".into(), "d +".into());
    for (cmd, p) in
        [("param.delete", json!({"name": "free"})), ("param.rename", json!({"name": "free", "new_name": "f2"}))]
    {
        assert!(s.execute(cmd, p).is_err(), "{cmd} is blocked while the text is broken");
    }
    // the way out: replace the broken expression through feature.edit
    run(&mut s, "feature.edit", json!({"feature": id, "set": {"distance": {"expr": "d * 2"}}}));
    run(&mut s, "param.rename", json!({"name": "free", "new_name": "f2"}));
    run(&mut s, "param.delete", json!({"name": "f2"}));
    // a literal repairs it too
    s.doc.features[0].exprs.insert("distance".into(), "(".into());
    assert!(s.execute("param.delete", json!({"name": "d"})).is_err());
    run(&mut s, "feature.edit", json!({"feature": id, "set": {"distance": 4.0}}));
    run(&mut s, "param.delete", json!({"name": "d"}));
    // and a broken sketch dimension is repaired by sketch.edit_constraint
    set(&mut s, "w", "5");
    run(
        &mut s,
        "sketch.constrain",
        json!({"sketch": "sq", "constraints": [{"type": "length", "line": 0, "value": {"expr": "w"}}]}),
    );
    s.doc.sketches[0].sketch.dim_exprs.insert(0, "w *".into());
    assert!(s.execute("param.delete", json!({"name": "w"})).is_err());
    run(
        &mut s,
        "sketch.edit_constraint",
        json!({"sketch": "sq", "index": 0, "constraint": {"type": "length", "line": 0, "value": 4.0}}),
    );
    run(&mut s, "param.delete", json!({"name": "w"}));
}
