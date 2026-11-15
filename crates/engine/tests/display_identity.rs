//! Display keys name the geometry they were made from: they stay while it stays (so nothing is
//! uploaded, cut or tessellated again) and are never given to other geometry (so no cache of the
//! viewport shows an old picture), and the preview reuses what the session already derived.
use serde_json::json;
use std::collections::HashSet;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Point3, synth};

fn with_square_sketch(s: &mut Session) {
    s.execute(
        "sketch.create",
        json!({"name": "sq", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "sq", "entities": [{"polygon": {"points": [[-5.0, -5.0], [5.0, -5.0], [5.0, 5.0], [-5.0, 5.0]]}}]}),
    )
    .unwrap();
}

#[test]
fn display_keys_stay_while_nothing_changes() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", synth::sphere(Point3::origin(), 10.0, 24, 12));
    s.execute("body.box", json!({"size": [20.0, 10.0, 5.0]})).unwrap();
    with_square_sketch(&mut s);
    // both lists stay alive, so a key equal by chance of a reused address cannot pass the test
    let a = s.display_items();
    let b = s.display_items();
    assert_eq!(a.len(), 3, "mesh, body and sketch are drawn");
    let keys = |v: &[surfacecraft_engine::DisplayItem]| v.iter().map(|d| (d.id, d.key)).collect::<Vec<_>>();
    assert_eq!(keys(&a), keys(&b));
}

#[test]
fn changed_geometry_never_gets_a_key_seen_before() {
    let mut s = Session::new();
    let mut seen: HashSet<usize> = HashSet::new();
    for k in 0..40 {
        // the same mesh made and removed again: an allocator would hand out the same address
        let id = s.doc.add_mesh("scan", synth::sphere(Point3::origin(), 10.0, 16, 8));
        let item = s.display_items().into_iter().find(|d| d.id == id).unwrap();
        assert!(seen.insert(item.key), "round {k}: key {} was already used by other geometry", item.key);
        assert!(s.doc.remove(id));
        assert!(s.display_items().is_empty());
    }
}

#[test]
fn an_edited_sketch_gets_a_new_key_and_its_new_shape() {
    let mut s = Session::new();
    with_square_sketch(&mut s);
    let before = s.display_items().remove(0);
    s.execute(
        "sketch.add",
        json!({"sketch": "sq", "entities": [{"polygon": {"points": [[20.0, 0.0], [30.0, 0.0], [30.0, 10.0]]}}]}),
    )
    .unwrap();
    let after = s.display_items().remove(0);
    assert_ne!(before.key, after.key);
    assert!(after.mesh.triangle_count() > before.mesh.triangle_count());
}

#[test]
fn the_preview_reuses_the_display_data_of_unchanged_objects() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [20.0, 10.0, 5.0], "name": "first"})).unwrap();
    let base = s.display_items();
    let mut copy = s.preview("body.box", json!({"size": [4.0, 4.0, 4.0], "name": "second"})).unwrap();
    let shown = copy.display_items();
    assert_eq!(shown.len(), 2);
    let first = base.first().unwrap();
    let same = shown.iter().find(|d| d.id == first.id).unwrap();
    assert_eq!(same.key, first.key, "the unchanged box is not tessellated again for the preview");
    assert!(std::sync::Arc::ptr_eq(&same.mesh, &first.mesh));
    let new = shown.iter().find(|d| d.id != first.id).unwrap();
    assert!(base.iter().all(|d| d.key != new.key), "the preview's own box has a key of its own");
}
