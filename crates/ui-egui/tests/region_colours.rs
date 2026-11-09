//! Regression: region and deviation colours reach the frame's draw items, and the colour buffer
//! is rewritten the first time colours appear (the first colour stamp used to collide with the
//! "no colours" state). No GPU needed.
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_ui_egui::shot::draw_items;
use surfacecraft_ui_egui::viewport::DrawItem;

fn mesh_item(items: &[DrawItem]) -> &DrawItem {
    items.first().expect("a draw item")
}

#[test]
fn auto_segment_colours_the_drawn_mesh() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [40, 30, 20], "resolution": 16})).unwrap();
    let before = draw_items(&mut s);
    let base = mesh_item(&before);
    assert!(base.colors.is_none() && base.colors_key == 0);

    s.execute("region.auto_segment", json!({"mesh": 1})).unwrap();
    let after = draw_items(&mut s);
    let item = mesh_item(&after);
    let colors = item.colors.as_ref().expect("per-vertex region colours");
    assert_eq!(colors.len(), item.mesh.positions.len());
    assert!(colors.iter().any(|c| *c != item.color), "regions differ from the base colour");
    assert_eq!(item.key, base.key, "same geometry key: only the colour stream changes");
    assert_ne!(item.colors_key, base.colors_key);
}

#[test]
fn deviation_map_colours_the_drawn_mesh() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [40, 30, 20], "resolution": 8, "name": "a"})).unwrap();
    s.execute("mesh.synth", json!({"shape": "sphere", "size": [30, 30, 30], "resolution": 12, "name": "b"})).unwrap();
    s.execute("deviation.compute", json!({"object": "a", "against": "b"})).unwrap();
    let items = draw_items(&mut s);
    let a = items.iter().find(|d| d.colors.is_some()).expect("deviation colours on the scan");
    assert!(a.colors_key != 0);
}
