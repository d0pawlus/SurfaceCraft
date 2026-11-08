//! Picking on a scan drawn through the display level of detail still returns original triangle ids.
use surfacecraft_geom::{Point3, synth};
use surfacecraft_ui_egui::SurfaceCraftApp;

#[test]
fn picking_a_coarse_drawn_scan_returns_a_full_resolution_triangle() {
    let mut app = SurfaceCraftApp::headless();
    app.session.tess.budget = 500;
    let mesh = synth::sphere(Point3::origin(), 20.0, 120, 60);
    let full = mesh.clone();
    app.session.doc.add_mesh("scan", mesh);
    app.camera.fit(&app.session.doc.bbox());
    assert!(app.pick_at(400.0, 300.0, 800.0, 600.0, false), "the sphere is under the view centre");
    let p = &app.picks[0];
    assert!((p.triangle as usize) < full.triangle_count());
    assert!(full.triangle_count() > 500, "the test mesh is above the budget");
    // the picked point lies on the sphere, and the triangle id belongs to the full mesh near it
    assert!((p.point.coords.norm() - 20.0).abs() < 0.3, "{:?}", p.point);
    let c = full.face_centroid(p.triangle as usize).unwrap();
    assert!((c - p.point).norm() < 3.0, "picked triangle {} is far from the hit", p.triangle);
}
