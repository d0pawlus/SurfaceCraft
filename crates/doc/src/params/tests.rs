//! Tests of the parameter table.
use super::*;

fn table(defs: &[(&str, &str)]) -> ParamTable {
    defs.iter().fold(ParamTable::default(), |t, (n, e)| t.with_param(n, e, None).expect("defines"))
}

fn value(t: &ParamTable, n: &str) -> f64 {
    t.get(n).map_or(f64::NAN, |p| p.value)
}

#[test]
fn values_follow_dependencies() {
    let t = table(&[("w", "40"), ("h", "w / 2"), ("d", "10")]);
    assert_eq!((value(&t, "w"), value(&t, "h"), value(&t, "d")), (40.0, 20.0, 10.0));
    let t2 = t.with_param("w", "50", None).expect("redefines");
    assert_eq!(value(&t2, "h"), 25.0, "the dependant follows");
    assert_eq!(t2.items().iter().map(|p| p.name.as_str()).collect::<Vec<_>>(), ["w", "h", "d"], "order kept");
}

#[test]
fn a_parameter_may_depend_on_one_defined_after_a_redefinition() {
    let t = table(&[("a", "1"), ("b", "a + 1"), ("c", "b + 1")]);
    let t = t.with_param("a", "10", None).expect("redefines");
    assert_eq!(value(&t, "c"), 12.0);
    let t = table(&[("x", "1"), ("y", "2"), ("z", "x + y")]).with_param("x", "y * 3", None).expect("redefines");
    assert_eq!(value(&t, "z"), 8.0, "x now depends on y, defined later");
}

#[test]
fn cycles_are_refused_and_named() {
    let t = table(&[("b", "1"), ("a", "b + 1")]);
    let e = t.with_param("b", "a * 2", None).expect_err("cycle");
    assert!(e.0.contains("cycle") && e.0.contains('a') && e.0.contains('b'), "{e}");
    assert!(e.0.contains("b -> a -> b") || e.0.contains("a -> b -> a"), "{e}");
    let e = t.with_param("a", "a + 1", None).expect_err("self cycle");
    assert!(e.0.contains("a -> a"), "{e}");
    let long = table(&[("p0", "1"), ("p1", "p0"), ("p2", "p1"), ("p3", "p2")]);
    let e = long.with_param("p0", "p3", None).expect_err("long cycle");
    assert!(e.0.contains("p0") && e.0.contains("p3"), "{e}");
}

#[test]
fn definitions_that_cannot_evaluate_are_refused() {
    let t = table(&[("w", "40")]);
    assert!(t.with_param("h", "q + 1", None).expect_err("unknown").0.contains("`q`"));
    assert!(t.with_param("h", "1 / 0", None).is_err());
    assert!(t.with_param("h", "(", None).is_err());
    assert!(t.with_param("sin", "1", None).is_err());
    assert!(t.with_param("1h", "1", None).is_err());
    let t = table(&[("w", "40"), ("h", "sqrt(w - 10)")]);
    let e = t.with_param("w", "5", None).expect_err("a dependant stops evaluating");
    assert!(e.0.contains("`h`"), "{e}");
    assert_eq!(value(&t, "w"), 40.0, "the table is unchanged");
}

#[test]
fn rename_rewrites_whole_names_in_dependants() {
    let t = table(&[("w", "40"), ("width", "7"), ("h", "w / 2 + width")]);
    let r = t.renamed("w", "len").expect("renames");
    assert_eq!(r.get("h").map(|p| p.expr.as_str()), Some("len / 2 + width"));
    assert_eq!(value(&r, "h"), 27.0);
    assert!(t.renamed("w", "width").is_err(), "taken");
    assert!(t.renamed("w", "pi").is_err(), "reserved");
    assert!(t.renamed("nope", "x").is_err());
    assert_eq!(t.renamed("w", "w").map(|r| r == t), Ok(true), "renaming to itself changes nothing");
}

#[test]
fn users_are_listed_and_delete_works_when_free() {
    let t = table(&[("w", "40"), ("h", "w / 2")]);
    assert_eq!(t.users_of("w").len(), 1);
    assert!(t.users_of("h").is_empty());
    let t = t.without("h").expect("free");
    assert!(t.get("h").is_none());
    assert!(t.without("zzz").is_err());
}

#[test]
fn a_loaded_table_is_revalidated() {
    let bad: ParamTable =
        serde_json::from_str(r#"[{"name":"a","expr":"b","value":1},{"name":"b","expr":"a","value":1}]"#).expect("json");
    assert!(bad.validated().is_err());
    let lying: ParamTable = serde_json::from_str(r#"[{"name":"a","expr":"2*3","value":99}]"#).expect("json");
    assert_eq!(lying.validated().map(|t| value(&t, "a")), Ok(6.0), "the value is recomputed");
    let dup: ParamTable =
        serde_json::from_str(r#"[{"name":"a","expr":"1","value":1},{"name":"a","expr":"2","value":1}]"#).expect("json");
    assert!(dup.validated().is_err());
}

#[test]
fn a_long_chain_does_not_recurse() {
    let mut t = table(&[("a0", "1")]);
    for k in 1..MAX_PARAMS {
        t = t.with_param(&format!("a{k}"), &format!("a{} + 1", k - 1), None).expect("chain");
    }
    assert_eq!(value(&t, &format!("a{}", MAX_PARAMS - 1)), MAX_PARAMS as f64);
    assert!(t.with_param("extra", "1", None).is_err(), "the table is full");
}

#[test]
fn comments_are_kept_unless_replaced() {
    let t = ParamTable::default().with_param("w", "1", Some("width")).expect("defines");
    let t = t.with_param("w", "2", None).expect("redefines");
    assert_eq!(t.get("w").map(|p| p.comment.as_str()), Some("width"));
    let t = t.with_param("w", "2", Some("")).expect("clears");
    assert_eq!(t.get("w").map(|p| p.comment.as_str()), Some(""));
}

#[test]
fn changes_list_new_and_moved_values() {
    let a = table(&[("w", "40"), ("h", "w / 2")]);
    let b = a.with_param("w", "50", None).expect("redefines").with_param("k", "1", None).expect("adds");
    let ch = b.changes_since(&a);
    assert_eq!(ch.len(), 3);
    assert!(ch.contains(&("k".to_string(), None, 1.0)));
}
