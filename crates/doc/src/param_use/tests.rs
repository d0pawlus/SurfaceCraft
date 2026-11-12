//! Tests of expression application.
use super::*;
use crate::Mode;
use crate::ObjectId;
use crate::feature::FeatureOp;

fn params() -> ParamTable {
    [("d", "10"), ("n", "2 + 2"), ("f", "0.1 * 30"), ("half", "n / 2.5")]
        .iter()
        .fold(ParamTable::default(), |t, (k, e)| t.with_param(k, e, None).expect("defines"))
}

fn extrude() -> FeatureOp {
    FeatureOp::Extrude { sketch: ObjectId(1), distance: 1.0, draft_deg: 0.0, mode: Mode::New, target: None }
}

fn pattern() -> FeatureOp {
    FeatureOp::Pattern {
        target: ObjectId(1),
        count: 2,
        translate: [1.0, 2.0, 3.0],
        rotate_axis: None,
        rotate_origin: [0.0; 3],
        rotate_deg: 0.0,
    }
}

fn exprs(list: &[(&str, &str)]) -> BTreeMap<String, String> {
    list.iter().map(|(k, v)| ((*k).to_string(), (*v).to_string())).collect()
}

#[test]
fn a_scalar_field_takes_its_expression() {
    let op = apply_feature_exprs(&extrude(), &exprs(&[("distance", "d * 2")]), &params()).expect("applies");
    assert!(matches!(op, FeatureOp::Extrude { distance, .. } if distance == 20.0));
}

#[test]
fn a_vector_component_takes_its_expression() {
    let op = apply_feature_exprs(&pattern(), &exprs(&[("translate.1", "d + 1")]), &params()).expect("applies");
    assert!(matches!(op, FeatureOp::Pattern { translate, .. } if translate == [1.0, 11.0, 3.0]));
}

#[test]
fn counts_need_a_whole_number() {
    let ok = apply_feature_exprs(&pattern(), &exprs(&[("count", "n")]), &params()).expect("4");
    assert!(matches!(ok, FeatureOp::Pattern { count: 4, .. }));
    let tol = apply_feature_exprs(&pattern(), &exprs(&[("count", "f")]), &params()).expect("0.1 * 30 is whole");
    assert!(matches!(tol, FeatureOp::Pattern { count: 3, .. }));
    for bad in ["half", "-1", "1e9", "n / 3"] {
        assert!(apply_feature_exprs(&pattern(), &exprs(&[("count", bad)]), &params()).is_err(), "{bad}");
    }
}

#[test]
fn unusable_targets_are_errors() {
    let p = params();
    assert!(apply_feature_exprs(&extrude(), &exprs(&[("nope", "1")]), &p).is_err());
    assert!(apply_feature_exprs(&extrude(), &exprs(&[("translate", "1")]), &p).is_err(), "no such field");
    assert!(apply_feature_exprs(&pattern(), &exprs(&[("rotate_axis.0", "1")]), &p).is_err(), "unset axis");
    assert!(apply_feature_exprs(&pattern(), &exprs(&[("translate.3", "1")]), &p).is_err());
    assert!(apply_feature_exprs(&pattern(), &exprs(&[("translate", "1")]), &p).is_err(), "a vector as a whole");
    assert!(apply_feature_exprs(&extrude(), &exprs(&[("distance", "zz")]), &p).is_err(), "unknown name");
}

#[test]
fn dimension_expressions_reach_the_constraints() {
    let mut sk = Sketch::default();
    sk.add_line([0.0, 0.0], [1.0, 0.0]);
    sk.constraints.push(surfacecraft_sketch::Constraint::Length { line: 0, value: 1.0 });
    sk.dim_exprs.insert(0, "d * 3".into());
    let out = apply_dim_exprs(&sk, &params()).expect("applies");
    assert_eq!(out.constraints.first().and_then(|c| c.dimension()), Some(30.0));
    sk.dim_exprs.insert(0, "zz".into());
    assert!(apply_dim_exprs(&sk, &params()).is_err());
}

#[test]
fn users_and_renames_cover_parked_features() {
    let mut doc = Document { params: params(), ..Document::default() };
    let body = doc.fresh_id();
    doc.add_feature("e", extrude(), body);
    doc.add_feature("e2", extrude(), body);
    doc.features[0].exprs = exprs(&[("distance", "d")]);
    doc.features[1].exprs = exprs(&[("distance", "d + 1")]);
    let parked = doc.features.remove(1);
    doc.rolled_back.push(parked);
    let users = users_of(&doc, "d");
    assert_eq!(users.iter().filter(|u| u.kind == "feature").count(), 2, "{users:?}");
    assert!(users.iter().all(|u| u.kind != "parameter"), "no parameter uses d: {users:?}");
    let (sk, f, r) =
        renamed_references(&doc, &doc.params.renamed("d", "depth").expect("table"), "d", "depth").expect("renames");
    assert!(sk.is_empty());
    assert_eq!(f[0].exprs.get("distance").map(String::as_str), Some("depth"));
    assert_eq!(r[0].exprs.get("distance").map(String::as_str), Some("depth + 1"));
}

#[test]
fn a_document_from_outside_is_checked() {
    let mut doc = Document { params: params(), ..Document::default() };
    assert!(validate_document(&doc).is_ok());
    let body = doc.fresh_id();
    doc.add_feature("e", extrude(), body);
    doc.features[0].exprs = exprs(&[("sketch", "1")]);
    assert!(validate_document(&doc).is_err(), "a non-numeric field");
}

#[test]
fn features_without_expressions_are_never_serialised_on_load() {
    // a base body is a whole B-rep: the checks of a load return before touching the operation
    let brep = surfacecraft_kernel::solid::make_box([0.0; 3], [1.0, 2.0, 3.0]).expect("box");
    let mut doc = Document::default();
    let body = doc.fresh_id();
    doc.add_feature("base", FeatureOp::Base { brep: std::sync::Arc::new(brep) }, body);
    let f = &doc.features[0];
    assert!(f.exprs.is_empty());
    assert_eq!(check_feature(f, &doc.params), Ok(()));
    assert!(feature_current(f, &doc.params));
    // and a document of many such bodies validates at once
    for k in 0..200 {
        let b = doc.fresh_id();
        let brep = surfacecraft_kernel::solid::make_box([0.0; 3], [1.0, 2.0, 3.0 + f64::from(k)]).expect("box");
        doc.add_feature("base", FeatureOp::Base { brep: std::sync::Arc::new(brep) }, b);
    }
    let started = std::time::Instant::now();
    let (_, stale) = validate_document(&doc).expect("valid");
    assert!(!stale);
    assert!(started.elapsed() < std::time::Duration::from_millis(500), "{:?}", started.elapsed());
}
