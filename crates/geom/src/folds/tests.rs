use super::*;
use crate::shell::{ThickenError, ThickenParams, thicken};
use crate::{Point3, Rng, Vec3, synth};

const EDGE: f64 = 1.0;
const CELLS: usize = 30;
const WALL: ThickenParams = ThickenParams { thickness: 1.0, symmetric: false };
/// A knot pushes a vertex over its neighbours: this far in edges (along x), sideways and up (mm).
const KNOT: (f64, f64, f64) = (2.2, 0.3, 0.5);

fn panel() -> Mesh {
    let n = CELLS as f64 * EDGE;
    let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), n, n, CELLS, CELLS);
    for p in &mut m.positions {
        p.z = 1.5 * (p.x / 9.0).sin() * (p.y / 7.0).cos();
    }
    synth::add_noise(&mut m, 0.01, &mut Rng::new(3));
    m
}

fn vertex_at(m: &Mesh, x: f64, y: f64) -> usize {
    let d = |i: usize| (m.positions[i].x - x).hypot(m.positions[i].y - y);
    (0..m.positions.len()).min_by(|&a, &b| d(a).total_cmp(&d(b))).unwrap()
}

fn knotted(x: f64, y: f64) -> Mesh {
    pushed(x, y, Vec3::new(KNOT.0 * EDGE, KNOT.1, KNOT.2))
}

fn pushed(x: f64, y: f64, by: Vec3) -> Mesh {
    let mut m = panel();
    let v = vertex_at(&m, x, y);
    m.positions[v] += by;
    m
}

fn loop_lengths(m: &Mesh) -> Vec<usize> {
    let mut l: Vec<usize> = edit::boundary_loops(m).iter().map(Vec::len).collect();
    l.sort_unstable();
    l
}

fn unresolved(m: &Mesh) -> Result<usize, ThickenError> {
    thicken(m, &WALL).map(|t| t.report.unresolved_vertices)
}

fn same_mesh(a: &Mesh, b: &Mesh) -> bool {
    a.positions == b.positions && a.triangles == b.triangles
}

#[test]
fn a_knot_in_a_panel_is_removed_and_the_hole_is_filled() {
    // Arrange: a knot the shell cannot clear
    let mut m = knotted(15.0, 15.0);
    let (area_before, loops_before) = (m.area(), loop_lengths(&m));
    assert!(matches!(unresolved(&m), Err(ThickenError::UnresolvedCrossings { .. })), "the fixture must be refused");
    assert!(find_folds(&m).iter().any(|f| f.small));
    let original = m.positions.clone();
    // Act
    let report = remove_folds(&mut m);
    // Assert: gone, no hole, rim and the rest of the surface untouched, the shell is built
    assert!(report.removed >= 1 && report.unfillable == 0, "{report:?}");
    assert!(find_folds(&m).iter().all(|f| !f.small), "{:?}", find_folds(&m));
    assert_eq!(loop_lengths(&m), loops_before);
    assert!((m.area() - area_before).abs() < 0.05 * area_before);
    let kept: std::collections::HashSet<[u64; 3]> =
        m.positions.iter().map(|p| p.coords.map(f64::to_bits).into()).collect();
    for p in original.iter().filter(|p| (p.x - 15.0).hypot(p.y - 15.0) > 8.0) {
        assert!(kept.contains(&<[u64; 3]>::from(p.coords.map(f64::to_bits))), "a vertex far from the knot moved");
    }
    assert_eq!(unresolved(&m).unwrap(), 0);
}

#[test]
fn a_fold_on_the_rim_only_trims_the_rim() {
    let mut m = pushed(0.0, 15.0, Vec3::new(0.0, 2.4 * EDGE, 0.5));
    let crossing = find_folds(&m).iter().filter(|f| f.crossing).count();
    let report = remove_folds(&mut m);
    assert!(crossing >= 1, "the fixture must have a fold");
    assert!(report.removed >= 1, "{report:?}");
    assert_eq!(edit::boundary_loops(&m).len(), 1);
    assert!(find_folds(&m).iter().all(|f| !f.crossing));
}

/// Two parallel vertical sheets 0.3 mm apart standing on the panel, facing away from each other, leaning over.
fn panel_with_fin(lean: f64) -> Mesh {
    let mut m = panel();
    for (k, gap) in [0.0, 0.3].into_iter().enumerate() {
        let mut fin = synth::plane(
            Point3::new(10.0 + gap, 10.0, 0.0),
            Vec3::new(lean.sin(), 0.0, lean.cos()),
            Vec3::y(),
            3.0,
            10.0,
            3,
            10,
        );
        if k == 1 {
            fin.triangles.iter_mut().for_each(|t| t.swap(1, 2));
        }
        m.append(&fin);
    }
    m
}

#[test]
fn a_thin_fin_seen_from_both_sides_is_not_touched() {
    for lean in [0.0_f64, 0.5, 0.9] {
        let mut m = panel_with_fin(lean);
        let before = m.clone();
        let report = remove_folds(&mut m);
        assert_eq!(report.removed, 0, "lean {lean}: {report:?}");
        assert!(same_mesh(&m, &before));
    }
}

#[test]
fn a_real_bump_and_a_shallow_step_are_not_touched() {
    let mut m = panel();
    for p in &mut m.positions {
        let r2 = (p.x - 12.0).powi(2) + (p.y - 12.0).powi(2);
        p.z += 2.0 * (-r2 / 8.0).exp();
        if p.x > 20.0 {
            p.z += 0.15;
        }
    }
    let before = m.clone();
    assert_eq!(remove_folds(&mut m).removed, 0);
    assert!(same_mesh(&m, &before));
}

#[test]
fn meshes_without_a_fold_stay_bit_identical() {
    let closed = synth::sphere(Point3::origin(), 20.0, 40, 20);
    let open = panel();
    for m in [closed, open] {
        let mut out = m.clone();
        let report = remove_folds(&mut out);
        assert!(report.removed == 0 && same_mesh(&out, &m), "{report:?}");
    }
}

#[test]
fn hostile_meshes_never_panic_and_are_left_alone() {
    let empty = Mesh { positions: vec![], triangles: vec![] };
    let mut nan = panel();
    nan.positions[5].x = f64::NAN;
    let mut twice = panel();
    let copy = twice.clone();
    twice.append(&copy);
    let mut three = panel();
    three.triangles.push(three.triangles[0]);
    three.triangles.push([three.triangles[0][0], three.triangles[0][1], 77]);
    let flat = Mesh { positions: vec![Point3::origin(); 4], triangles: vec![[0, 1, 2], [1, 2, 3]] };
    let out_of_range = Mesh { positions: vec![Point3::origin(); 3], triangles: vec![[0, 1, 9]] };
    for original in [empty, nan, twice, three, flat, out_of_range] {
        let mut m = original.clone();
        let report = remove_folds(&mut m);
        assert_eq!(report.removed, 0, "{report:?}");
        assert_eq!(m.positions.len(), original.positions.len().max(m.positions.len().min(original.positions.len())));
    }
}

#[test]
fn the_result_does_not_depend_on_the_run() {
    let (mut a, mut b) = (knotted(15.0, 15.0), knotted(15.0, 15.0));
    remove_folds(&mut a);
    remove_folds(&mut b);
    assert!(same_mesh(&a, &b));
}

/// Measured (release): a million triangles. `cargo test --release -p surfacecraft-geom million -- --ignored --nocapture`
#[test]
#[ignore = "timing"]
fn a_million_triangles_take_seconds_not_minutes() {
    let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 500.0, 500.0, 700, 700);
    for p in &mut m.positions {
        p.z = (p.x / 40.0).sin() + (p.y / 30.0).cos();
    }
    let v = vertex_at(&m, 250.0, 250.0);
    m.positions[v] += Vec3::new(1.6, 0.2, 0.3);
    let t = std::time::Instant::now();
    let report = remove_folds(&mut m);
    eprintln!("{} triangles: {:.2} s, {report:?}", m.triangle_count(), t.elapsed().as_secs_f64());
}
