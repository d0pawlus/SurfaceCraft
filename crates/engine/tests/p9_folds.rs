//! P9 folds: `mesh.clean` removes the small folds of a scan (groups of triangles that face against the
//! surface around them and pass through the sheet) and `scan.to_solid` with a `thickness` then builds
//! its wall; what is not a fold stays. Synthetic: a noisy panel of 30 x 30 mm.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

const CELLS: usize = 30;
const WALL_MM: f64 = 1.0;
/// A vertex pushed over its neighbours (mm): the scan crosses itself there.
const KNOT: (f64, f64, f64) = (2.2, 0.3, 0.5);

fn panel() -> Mesh {
    let n = CELLS as f64;
    let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), n, n, CELLS, CELLS);
    for p in &mut m.positions {
        p.z = 1.5 * (p.x / 9.0).sin() * (p.y / 7.0).cos();
    }
    synth::add_noise(&mut m, 0.01, &mut Rng::new(3));
    m
}

fn knotted_panel() -> Mesh {
    knotted_panel_standing(KNOT.2)
}

/// The knot, with the pushed vertex `up` mm above the panel.
fn knotted_panel_standing(up: f64) -> Mesh {
    let mut m = panel();
    let centre = |i: &usize| (m.positions[*i].x - 15.0).hypot(m.positions[*i].y - 15.0);
    let v = (0..m.positions.len()).min_by(|a, b| centre(a).total_cmp(&centre(b))).unwrap();
    m.positions[v] += Vec3::new(KNOT.0, KNOT.1, up);
    m
}

fn session_with(mesh: Mesh) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    s
}

fn mesh_of(s: &Session) -> Mesh {
    let id = s.doc.find_by_name("scan").unwrap().0;
    (*s.doc.mesh(id).unwrap().mesh).clone()
}

fn codes(out: &Value) -> Vec<String> {
    out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str().map(str::to_string)).collect()
}

#[test]
fn mesh_clean_removes_the_fold_and_reports_what_it_did() {
    // Arrange
    let mut s = session_with(knotted_panel());

    // Act
    let out = s.execute("mesh.clean", json!({"object": "scan", "remove_folds": true})).unwrap();

    // Assert
    let folds = &out["folds"];
    assert!(folds["removed"].as_u64().unwrap() >= 1, "{out}");
    assert_eq!(folds["left"], 0, "{out}");
    assert!(
        folds["removed_triangles"].as_u64().unwrap() >= 1 && folds["removed_area"].as_f64().unwrap() > 0.0,
        "{out}"
    );
    assert!(out["added_triangles"].as_u64().unwrap() >= 1, "the hole is filled: {out}");
    assert_eq!(surfacecraft_geom::edit::boundary_loops(&mesh_of(&s)).len(), 1, "no hole is left");
}

#[test]
fn without_the_fold_removal_the_wall_is_refused_with_it_the_solid_is_built() {
    // Arrange
    let mut kept = session_with(knotted_panel());
    kept.execute("mesh.clean", json!({"object": "scan"})).unwrap();
    let mut repaired = session_with(knotted_panel());

    // Act
    let refused = kept.execute("scan.to_solid", json!({"scan": "scan", "thickness": WALL_MM, "clean": false})).unwrap();
    let out = repaired.execute("scan.to_solid", json!({"scan": "scan", "thickness": WALL_MM})).unwrap();

    // Assert
    assert_eq!(refused["verdict"], "fail", "{refused}");
    assert!(codes(&refused).contains(&"unresolved_crossings".to_string()), "{refused}");
    assert_ne!(out["verdict"], "fail", "{out}");
    assert_eq!(out["thickened"]["unresolved_vertices"], 0, "{out}");
    assert!(!codes(&out).contains(&"self_intersection".to_string()), "{out}");
    assert!(out["thickened"]["folds"]["removed"].as_u64().unwrap() >= 1, "{out}");
}

#[test]
fn a_scan_without_a_fold_is_cleaned_exactly_as_before() {
    for scan in [panel(), synth::sphere(Point3::origin(), 20.0, 40, 20)] {
        // Arrange
        let (mut with, mut without) = (session_with(scan.clone()), session_with(scan));

        // Act
        let out = with.execute("mesh.clean", json!({"object": "scan", "remove_folds": true})).unwrap();
        without.execute("mesh.clean", json!({"object": "scan", "remove_folds": false})).unwrap();

        // Assert
        assert_eq!(out["folds"]["removed"], 0, "{out}");
        let (a, b) = (mesh_of(&with), mesh_of(&without));
        assert!(a.positions == b.positions && a.triangles == b.triangles);
    }
}

fn thickened(extra: Value) -> (Session, Value) {
    let mut s = session_with(knotted_panel());
    let mut p = json!({"scan": "scan", "thickness": WALL_MM});
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    let out = s.execute("scan.to_solid", p).unwrap();
    (s, out)
}

#[test]
fn a_fold_taller_than_the_tolerance_makes_the_verdict_a_review_that_names_it() {
    // Arrange and act: the fold stands 0.3 mm and more out of the panel, the tolerance is 0.2 mm
    let (_, out) = thickened(json!({}));

    // Assert: the solid is built, the removal is reported with its numbers and the verdict is not a pass
    let folds = &out["thickened"]["folds"];
    assert!(folds["removed"].as_u64().unwrap() >= 1, "{out}");
    assert!(folds["largest_removed_height"].as_f64().unwrap() > 0.2, "{out}");
    assert!(
        folds["largest_fill_deviation"].as_f64().unwrap() >= folds["largest_removed_height"].as_f64().unwrap(),
        "{out}"
    );
    assert!(folds["removed_area_share"].as_f64().unwrap() > 0.0, "{out}");
    assert!(out["body"].is_object(), "{out}");
    assert_eq!(out["verdict"], "review", "{out}");
    assert!(codes(&out).contains(&"scan_folds_removed".to_string()), "{out}");
}

#[test]
fn the_deviation_is_measured_against_the_scan_as_it_was() {
    // Arrange and act
    let (_, out) = thickened(json!({}));

    // Assert: the removed vertex stands at least as far from the body as the fold stood out of the surface
    let height = out["thickened"]["folds"]["largest_removed_height"].as_f64().unwrap();
    let max = out["deviation"]["max_abs"].as_f64().unwrap();
    assert!(max >= 0.7 * height && max > 0.2, "max |deviation| {max} against a fold of {height} mm: {out}");
}

#[test]
fn the_symmetric_layer_judges_the_removed_points_where_the_wall_has_its_middle() {
    // Arrange and act
    let (_, out) = thickened(json!({"symmetric": true}));

    // Assert
    let height = out["thickened"]["folds"]["largest_removed_height"].as_f64().unwrap();
    let max = out["deviation"]["max_abs"].as_f64().unwrap();
    assert!(max >= 0.7 * height && max > 0.2, "max |deviation| {max} against a fold of {height} mm: {out}");
    assert!(codes(&out).contains(&"scan_folds_removed".to_string()), "{out}");
}

#[test]
fn the_fold_removal_is_part_of_the_one_undo_step_and_undo_restores_the_fold() {
    // Arrange
    let scan = knotted_panel();
    let (triangles, positions) = (scan.triangles.clone(), scan.positions.clone());
    let mut s = session_with(scan);
    let steps = s.history.undo_labels().len();

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": WALL_MM})).unwrap();
    let repaired = mesh_of(&s);
    s.execute("edit.undo", Value::Null).unwrap();

    // Assert
    assert!(out["thickened"]["folds"]["removed"].as_u64().unwrap() >= 1, "{out}");
    assert_ne!(repaired.triangles, triangles, "the scan object is repaired in place, like the hole filling");
    assert_eq!(s.history.undo_labels().len(), steps, "one step");
    let restored = mesh_of(&s);
    assert!(restored.triangles == triangles && restored.positions == positions, "undo restores the fold");
}

#[test]
fn a_fold_deeper_in_the_wall_than_its_middle_is_not_flattered_by_the_back_face() {
    // Arrange: a fold 0.5 mm into a wall of 0.8 mm: its tip is 0.3 mm from the inner face (the whole body would
    // measure 0.3 mm) and 0.5 mm from the scan layer
    let mut s = session_with(knotted_panel_standing(-0.5));

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": 0.8})).unwrap();

    // Assert: the tip is measured against the scan layer only, so it is not measured (it faces the other face), is
    // counted as such, and the true depth is reported and raises the issue
    let folds = &out["thickened"]["folds"];
    assert!(folds["removed"].as_u64().unwrap() >= 1, "{out}");
    assert!(folds["removed_points_not_facing"].as_u64().unwrap() >= 1, "{out}");
    assert!(folds["largest_removed_height"].as_f64().unwrap() > 0.4, "{out}");
    assert!(
        folds["removed_points_max_deviation"].as_f64().unwrap() < 0.1,
        "nothing flatters the removed margin: {out}"
    );
    assert_eq!(out["verdict"], "review", "{out}");
    assert!(codes(&out).contains(&"scan_folds_removed".to_string()), "{out}");
}

#[test]
fn removed_points_on_the_surface_land_on_the_outer_face_of_a_symmetric_wall() {
    // Arrange and act
    let (_, out) = thickened(json!({"symmetric": true}));

    // Assert: the margin of the fold lay on the scan, so with the half-wall shift it lies on the scan layer (the
    // outer face); without the shift (or against the other face) it would be 0.5 mm off or not face the body
    let folds = &out["thickened"]["folds"];
    let (count, vertices) = (folds["removed_points"].as_u64().unwrap(), 16 * 3);
    assert!(count * 2 >= vertices, "the points face the scan layer: {out}");
    assert!(folds["removed_points_within"].as_f64().unwrap() > 0.5, "{out}");
    let (_, one_sided) = thickened(json!({}));
    assert!(one_sided["thickened"]["folds"]["removed_points_within"].as_f64().unwrap() > 0.5, "{one_sided}");
}
