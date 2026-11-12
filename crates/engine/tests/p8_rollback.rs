//! P8-H7: the roll-back marker of the feature history. Features after the marker are parked
//! (not replayed), so the body is what it was at that step; new features land at the marker;
//! rolling to the end restores everything. Every volume is compared with a closed form or with
//! a replay of the same operations in a fresh session.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_engine::Session;
use surfacecraft_kernel::solid::volume;

/// Volume of the block after extrude (20 x 20 x 10) and a through hole of radius 3 (90 pi).
const BLOCK_CUT: f64 = 4000.0 - 90.0 * PI;

fn id_of(v: &Value) -> u64 {
    v["id"].as_u64().unwrap_or_else(|| panic!("no feature id in {v}"))
}

fn block_volume(s: &Session) -> f64 {
    let body = s.doc.bodies.iter().find(|b| b.name == "block").expect("block body");
    volume(&body.brep).expect("volume")
}

/// Extrude 20 x 20 x 10, then a radius-3 hole cut through it. Returns the cut's feature id.
fn extrude_and_cut(s: &mut Session) -> u64 {
    s.execute(
        "sketch.create",
        json!({"name": "base", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "base", "entities": [{"polygon": {"points": [[-10.0, -10.0], [10.0, -10.0], [10.0, 10.0], [-10.0, 10.0]]}}]}),
    )
    .unwrap();
    s.execute("feature.extrude", json!({"sketch": "base", "distance": 10.0, "name": "block"})).unwrap();
    s.execute(
        "sketch.create",
        json!({"name": "hole", "plane": {"origin": [0.0, 0.0, 10.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute("sketch.add", json!({"sketch": "hole", "entities": [{"circle": {"center": [0.0, 0.0], "radius": 3.0}}]}))
        .unwrap();
    let cut = s
        .execute("feature.extrude", json!({"sketch": "hole", "distance": -10.0, "mode": "cut", "target": "block"}))
        .unwrap();
    id_of(&cut)
}

/// Extrude, cut, fillet (edge 0) with the fillet's feature id.
fn with_fillet(s: &mut Session) -> u64 {
    extrude_and_cut(s);
    let f = s.execute("feature.fillet", json!({"body": "block", "edges": [0], "size": 0.1})).unwrap();
    id_of(&f)
}

fn active_ops(s: &Session) -> Vec<String> {
    s.doc.features.iter().map(|f| f.op.label().to_string()).collect()
}

fn node_rolled_back(tree: &Value, id: u64) -> bool {
    tree["features"]
        .as_array()
        .and_then(|a| a.iter().find(|n| n["id"] == json!(id)))
        .and_then(|n| n["rolled_back"].as_bool())
        .unwrap_or_else(|| panic!("no node {id} in {tree}"))
}

#[test]
fn rolling_back_before_the_fillet_gives_the_pre_fillet_volume() {
    let mut s = Session::new();
    let fillet = with_fillet(&mut s);
    assert!((block_volume(&s) - BLOCK_CUT).abs() > 1e-3, "the fillet changed the body");
    let tree = s.execute("feature.rollback", json!({"to": {"feature": fillet}})).unwrap();
    assert_eq!(tree["marker"], 2, "two features are active: {tree}");
    assert!(node_rolled_back(&tree, fillet));
    let rel = (block_volume(&s) - BLOCK_CUT).abs() / BLOCK_CUT;
    assert!(rel < 1e-6, "pre-fillet volume off by {rel}");
}

#[test]
fn a_chamfer_added_while_rolled_back_is_inserted_before_the_fillet() {
    let mut s = Session::new();
    let fillet = with_fillet(&mut s);
    s.execute("feature.rollback", json!({"to": {"feature": fillet}})).unwrap();
    s.execute("feature.chamfer", json!({"body": "block", "edges": [1], "size": 0.2})).unwrap();
    // the cut is an extrude with mode cut, so both base features read "extrude"
    assert_eq!(active_ops(&s), ["extrude", "extrude", "chamfer"], "the chamfer is at the marker");
    let tree = s.execute("feature.tree", Value::Null).unwrap();
    assert!(node_rolled_back(&tree, fillet), "the fillet stays rolled back: {tree}");
}

#[test]
fn rolling_to_the_end_matches_a_replay_of_all_features() {
    let mut s = Session::new();
    let fillet = with_fillet(&mut s);
    s.execute("feature.rollback", json!({"to": {"feature": fillet}})).unwrap();
    s.execute("feature.chamfer", json!({"body": "block", "edges": [1], "size": 0.2})).unwrap();
    let tree = s.execute("feature.rollback", json!({"to": "end"})).unwrap();
    assert_eq!(tree["marker"], 4, "every feature is active again: {tree}");
    assert!(!node_rolled_back(&tree, fillet));
    // the chamfer takes another edge than the fillet, so the fillet still finds its edge (a fillet
    // whose edge the chamfer consumed fails, as it should); a full replay gives the same body
    let got = block_volume(&s);
    let replayed = s.execute("feature.replay", Value::Null).unwrap();
    assert_eq!(replayed["failed_features"], 0, "{replayed}");
    let want = block_volume(&s);
    assert!((got - want).abs() / want < 1e-9, "{got} vs replay {want}");
    assert!(got < BLOCK_CUT, "the chamfer and the fillet both removed material");
}

#[test]
fn undo_and_redo_move_the_marker_back_and_forth() {
    let mut s = Session::new();
    let fillet = with_fillet(&mut s);
    s.execute("feature.rollback", json!({"to": {"feature": fillet}})).unwrap();
    assert_eq!(s.doc.rolled_back.len(), 1);
    s.execute("edit.undo", Value::Null).unwrap();
    assert!(s.doc.rolled_back.is_empty(), "undo restores the fillet");
    assert!((block_volume(&s) - BLOCK_CUT).abs() > 1e-3);
    s.execute("edit.redo", Value::Null).unwrap();
    assert_eq!(s.doc.rolled_back.len(), 1, "redo rolls back again");
    assert!((block_volume(&s) - BLOCK_CUT).abs() < 1e-6 * BLOCK_CUT);
}

#[test]
fn the_marker_survives_a_save_and_load() {
    let mut s = Session::new();
    let fillet = with_fillet(&mut s);
    s.execute("feature.rollback", json!({"to": {"feature": fillet}})).unwrap();
    let text = surfacecraft_doc::format::to_json(&s.doc).expect("serialise");
    let loaded = surfacecraft_doc::format::from_json(&text).expect("load");
    assert_eq!(loaded.rolled_back.len(), 1, "the parked fillet is saved");
    assert_eq!(loaded.features.len(), 2);
}

#[test]
fn hostile_roll_back_values_are_errors_not_panics() {
    let mut s = Session::new();
    with_fillet(&mut s);
    let hostile = [
        json!({"to": {"index": 99}}),
        json!({"to": {"index": -1}}),
        json!({"to": {"index": 1.5}}),
        json!({"to": {"feature": 424242}}),
        json!({"to": {"feature": "no such feature"}}),
        json!({"to": "nowhere"}),
        json!({"to": null}),
        json!({}),
        json!({"to": {"index": 1}, "extra": true}),
    ];
    for v in hostile {
        assert!(s.execute("feature.rollback", v.clone()).is_err(), "{v} must be refused");
    }
    assert_eq!(s.doc.features.len(), 3, "a refused roll-back changes nothing");
    assert!(s.doc.rolled_back.is_empty());
}

#[test]
fn rolling_back_to_an_index_counts_from_the_first_feature() {
    let mut s = Session::new();
    with_fillet(&mut s);
    let tree = s.execute("feature.rollback", json!({"to": {"index": 1}})).unwrap();
    assert_eq!(tree["marker"], 1);
    let rel = (block_volume(&s) - (4000.0)).abs() / 4000.0;
    assert!(rel < 1e-6, "only the extrude is replayed: {}", block_volume(&s));
}

fn body_named(s: &Session, name: &str) -> Option<surfacecraft_doc::ObjectId> {
    s.doc.bodies.iter().find(|b| b.name == name).map(|b| b.id)
}

fn feature_names(s: &Session) -> Vec<String> {
    s.doc.features.iter().chain(s.doc.rolled_back.iter()).map(|f| f.name.clone()).collect()
}

/// A second sketch with a 4 x 4 square and an extrusion of it into a new body called `name`.
fn extrude_square(s: &mut Session, name: &str) {
    let sk = format!("sq {name}");
    s.execute(
        "sketch.create",
        json!({"name": sk, "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": sk, "entities": [{"polygon": {"points": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]}}]}),
    )
    .unwrap();
    s.execute("feature.extrude", json!({"sketch": sk, "distance": 2.0, "name": name})).unwrap();
}

#[test]
fn rolling_forward_restores_the_body_with_its_name_and_id() {
    let mut s = Session::new();
    extrude_and_cut(&mut s);
    let id = body_named(&s, "block").expect("block");
    s.doc.set_visible(id, false);
    s.execute("feature.rollback", json!({"to": {"index": 0}})).unwrap();
    assert!(body_named(&s, "block").is_none(), "the body does not exist before its first feature");
    s.execute("feature.rollback", json!({"to": "end"})).unwrap();
    assert_eq!(body_named(&s, "block"), Some(id), "same id, same name");
    assert!((block_volume(&s) - BLOCK_CUT).abs() / BLOCK_CUT < 1e-6);
    let body = s.doc.body(id).expect("body");
    assert!(!body.visible && !body.shown(), "the user's choice to hide the body survives the round trip");
}

#[test]
fn a_hidden_body_stays_hidden_after_editing_an_unrelated_feature() {
    let mut s = Session::new();
    extrude_and_cut(&mut s);
    extrude_square(&mut s, "other");
    let other = body_named(&s, "other").expect("other");
    s.doc.set_visible(other, false);
    s.execute("feature.edit", json!({"feature": "extrude", "set": {"distance": 12.0}})).unwrap();
    assert!(!s.doc.body(other).expect("other").shown(), "a replay does not unhide it");
    assert!(s.doc.body(body_named(&s, "block").expect("block")).expect("block").shown());
}

#[test]
fn a_failed_roll_forward_keeps_the_body_identity_for_the_repair() {
    let mut s = Session::new();
    s.execute(
        "sketch.create",
        json!({"name": "base", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "base", "entities": [{"polygon": {"points": [[0.0, 0.0], [4.0, 0.0], [4.0, 4.0], [0.0, 4.0]]}}]}),
    )
    .unwrap();
    let ext = s.execute("feature.extrude", json!({"sketch": "base", "distance": 2.0, "name": "block"})).unwrap();
    let (feature, id) = (id_of(&ext), body_named(&s, "block").expect("block"));
    s.doc.set_visible(id, false);
    s.execute("feature.rollback", json!({"to": {"index": 0}})).unwrap();
    s.execute("doc.delete", json!({"object": "base"})).unwrap();
    let tree = s.execute("feature.rollback", json!({"to": "end"})).unwrap();
    assert_eq!(tree["failed_features"], 1, "the extrusion lost its sketch: {tree}");
    assert!(body_named(&s, "block").is_none(), "no geometry, so not in the model");
    assert_ne!(s.doc.unique_name("block"), "block", "the name stays reserved");
    // repair: a new sketch for the same feature
    s.execute(
        "sketch.create",
        json!({"name": "fixed", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "fixed", "entities": [{"polygon": {"points": [[0.0, 0.0], [3.0, 0.0], [3.0, 3.0], [0.0, 3.0]]}}]}),
    )
    .unwrap();
    let sk = s.doc.sketches.iter().find(|o| o.name == "fixed").expect("sketch").id.0;
    s.execute("feature.edit", json!({"feature": feature, "set": {"sketch": sk}})).unwrap();
    assert_eq!(body_named(&s, "block"), Some(id), "same id and name after the repair");
    assert!(!s.doc.body(id).expect("body").visible, "and the user's visibility");
}

#[test]
fn a_parked_body_name_stays_reserved_while_rolled_back() {
    let mut s = Session::new();
    extrude_and_cut(&mut s);
    let id = body_named(&s, "block").expect("block");
    s.execute("feature.rollback", json!({"to": {"index": 0}})).unwrap();
    extrude_square(&mut s, "block");
    assert!(body_named(&s, "block").is_none(), "the new body does not take the parked name");
    assert!(body_named(&s, "block 2").is_some(), "the existing clash convention: a numbered name");
    s.execute("feature.rollback", json!({"to": "end"})).unwrap();
    assert_eq!(body_named(&s, "block"), Some(id), "later commands that say \"block\" mean the original body");
    assert!(body_named(&s, "block 2").is_some());
}

#[test]
fn parked_feature_names_count_as_taken() {
    let mut s = Session::new();
    extrude_and_cut(&mut s);
    s.execute("feature.rollback", json!({"to": {"index": 0}})).unwrap();
    extrude_square(&mut s, "other");
    let names = feature_names(&s);
    let mut unique = names.clone();
    unique.sort();
    unique.dedup();
    assert_eq!(names.len(), unique.len(), "every feature name is unique: {names:?}");
}

#[test]
fn a_parked_fillet_whose_edge_was_consumed_fails_on_roll_forward_and_is_reported() {
    let mut s = Session::new();
    let fillet = with_fillet(&mut s);
    s.execute("feature.rollback", json!({"to": {"feature": fillet}})).unwrap();
    // the chamfer takes the edge the parked fillet rounds
    s.execute("feature.chamfer", json!({"body": "block", "edges": [0], "size": 0.2})).unwrap();
    let tree = s.execute("feature.rollback", json!({"to": "end"})).unwrap();
    assert_eq!(tree["marker"], 4, "the fillet is back in the history, not lost: {tree}");
    assert!(s.doc.rolled_back.is_empty());
    assert_eq!(tree["failed_features"], 1, "{tree}");
    let failures = tree["failures"].as_array().expect("failures listed");
    assert_eq!(failures.len(), 1, "{tree}");
    assert_eq!(failures[0]["id"], json!(fillet));
    assert!(!failures[0]["error"].as_str().unwrap_or_default().is_empty());
    assert!(s.doc.feature(surfacecraft_doc::ObjectId(fillet)).expect("fillet").error.is_some(), "marked failed");
    assert!(block_volume(&s) > 0.0, "the document is consistent: the body is the chamfered block");
}
