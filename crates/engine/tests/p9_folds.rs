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
    let mut m = panel();
    let centre = |i: &usize| (m.positions[*i].x - 15.0).hypot(m.positions[*i].y - 15.0);
    let v = (0..m.positions.len()).min_by(|a, b| centre(a).total_cmp(&centre(b))).unwrap();
    m.positions[v] += Vec3::new(KNOT.0, KNOT.1, KNOT.2);
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
