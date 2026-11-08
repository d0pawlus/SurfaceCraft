//! P2: large scans are drawn through a coarse level of detail; colours and selections still refer
//! to the original mesh.
use std::sync::Arc;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Point3, synth};

fn session_with_sphere(budget: usize) -> (Session, surfacecraft_doc::ObjectId, usize) {
    let mut s = Session::new();
    s.tess.budget = budget;
    let m = synth::sphere(Point3::origin(), 20.0, 120, 60);
    let n = m.triangle_count();
    let id = s.doc.add_mesh("scan", m);
    (s, id, n)
}

#[test]
fn meshes_above_the_budget_are_drawn_coarse_and_cached() {
    let (mut s, id, n) = session_with_sphere(2_000);
    let a = s.display_items();
    let item = a.iter().find(|d| d.id == id).unwrap();
    assert!(item.mesh.triangle_count() <= 2_000 && item.mesh.triangle_count() > 100, "{}", item.mesh.triangle_count());
    assert_eq!(item.pick_mesh.triangle_count(), n, "picking keeps the full mesh");
    assert!(item.lod.is_some());
    // the second call reuses the cached level of detail (same drawn mesh, same key)
    let b = s.display_items();
    let item2 = b.iter().find(|d| d.id == id).unwrap();
    assert_eq!(item.key, item2.key);
    assert!(Arc::ptr_eq(&item.mesh, &item2.mesh));
    // a selection of original triangles lands on drawn triangles
    let drawn = item.drawn_triangles(&[0, 1, 2, (n / 2) as u32, (n - 1) as u32]);
    assert!(!drawn.is_empty() && drawn.iter().all(|&t| (t as usize) < item.mesh.triangle_count()));
}

#[test]
fn small_meshes_are_drawn_as_they_are() {
    let (mut s, id, n) = session_with_sphere(0);
    let items = s.display_items();
    let item = items.iter().find(|d| d.id == id).unwrap();
    assert!(item.lod.is_none());
    assert_eq!(item.mesh.triangle_count(), n);
    assert_eq!(item.drawn_triangles(&[3, 4]), vec![3, 4]);
}

#[test]
fn overlay_colours_are_mapped_and_a_new_overlay_changes_the_colour_key() {
    let (mut s, id, _) = session_with_sphere(2_000);
    let nv = s.doc.mesh(id).unwrap().mesh.vertex_count();
    s.overlays.insert(id, Arc::new(vec![[1.0, 0.0, 0.0, 1.0]; nv]));
    let a = s.display_items();
    let a = a.iter().find(|d| d.id == id).unwrap();
    let colors = a.colors.as_ref().unwrap();
    assert_eq!(colors.len(), a.mesh.vertex_count(), "colours follow the drawn vertices");
    assert!(colors.iter().all(|c| (c[0] - 1.0).abs() < 1e-6));
    // unchanged overlay: same identity, so the GPU colour buffer is not touched
    let again = s.display_items();
    let again = again.iter().find(|d| d.id == id).unwrap();
    assert_eq!(a.colors_key, again.colors_key);
    assert_ne!(a.colors_key, 0);
    // a new overlay on the same geometry: same geometry key, different colour key
    s.overlays.insert(id, Arc::new(vec![[0.0, 1.0, 0.0, 1.0]; nv]));
    let b = s.display_items();
    let b = b.iter().find(|d| d.id == id).unwrap();
    assert_eq!(a.key, b.key, "geometry is not re-uploaded");
    assert_ne!(a.colors_key, b.colors_key, "but the colours are");
    assert!(b.colors.as_ref().unwrap().iter().all(|c| (c[1] - 1.0).abs() < 1e-6));
}
