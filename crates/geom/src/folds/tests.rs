use super::detect::{Incidence, Site};
use super::remove::with_margin;
use super::*;
use crate::shell::{ThickenError, ThickenParams, thicken};
use crate::{Point3, Rng, Vec3, synth};

const EDGE: f64 = 1.0;
const CELLS: usize = 30;
const WALL: ThickenParams = ThickenParams { thickness: 1.0, symmetric: false };
/// The tolerance the fill is judged by (mm).
const TOL: f64 = 0.2;
/// A knot pushes a vertex over its neighbours: this far in edges (along x), sideways and up (mm).
const KNOT: (f64, f64, f64) = (2.2, 0.3, 0.5);

fn run(m: &mut Mesh) -> FoldReport {
    let n = m.triangle_count();
    remove_folds(m, TOL, &Cancel::new(), n)
}

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
    let report = run(&mut m);
    // Assert: gone, no hole, rim and the rest of the surface untouched, the shell is built
    assert!(report.removed >= 1 && report.refused == Refused::default(), "{report:?}");
    assert!(report.largest_removed_height > 0.1, "the fold stood out of the surface: {report:?}");
    assert!(report.largest_fill_deviation >= report.largest_removed_height, "it covers the fold too: {report:?}");
    assert!(report.removed_area_share > 0.0 && report.removed_area_share < 0.05, "{report:?}");
    assert!(report.removed_mesh.triangle_count() >= report.removed, "the scan as it was is kept: {report:?}");
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
    let report = run(&mut m);
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
    for lean in [0.0_f64, 0.5, 0.9, 1.3, 1.5] {
        let mut m = panel_with_fin(lean);
        let before = m.clone();
        let report = run(&mut m);
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
    assert_eq!(run(&mut m).removed, 0);
    assert!(same_mesh(&m, &before));
}

#[test]
fn meshes_without_a_fold_stay_bit_identical() {
    let closed = synth::sphere(Point3::origin(), 20.0, 40, 20);
    let open = panel();
    for m in [closed, open] {
        let mut out = m.clone();
        let report = run(&mut out);
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
    let far = {
        // a tiny median edge and one finite vertex far away: the cell index would overflow
        let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 0.01, 0.01, 10, 10);
        let n = m.positions.len() as u32;
        m.positions.extend([
            Point3::new(1e300, 0.0, 0.0),
            Point3::new(1e300, 1e-3, 0.0),
            Point3::new(1e300, 0.0, 1e-3),
        ]);
        m.triangles.push([n, n + 1, n + 2]);
        m
    };
    let out_of_range = Mesh { positions: vec![Point3::origin(); 3], triangles: vec![[0, 1, 9]] };
    for original in [empty, nan, twice, three, flat, far, out_of_range] {
        let mut m = original.clone();
        let report = run(&mut m);
        assert_eq!(report.removed, 0, "{report:?}");
        assert_eq!(m.positions.len(), original.positions.len().max(m.positions.len().min(original.positions.len())));
    }
}

#[test]
fn the_result_does_not_depend_on_the_run() {
    let (mut a, mut b) = (knotted(15.0, 15.0), knotted(15.0, 15.0));
    run(&mut a);
    run(&mut b);
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
    let report = run(&mut m);
    eprintln!("{} triangles: {:.2} s, {report:?}", m.triangle_count(), t.elapsed().as_secs_f64());
}

/// A ridge (a crease of 90 degrees along x = 15), the height of the ideal surface there.
fn ridge_z(x: f64) -> f64 {
    4.0 - (x - 15.0).abs()
}

fn ridge_panel() -> Mesh {
    let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 30.0, 30.0, CELLS, CELLS);
    m.positions.iter_mut().for_each(|p| p.z = ridge_z(p.x));
    m
}

/// How far the ideal surface points `ideal` lie from `m` (mm).
fn reach(m: &Mesh, ideal: &[Point3]) -> f64 {
    let bvh = crate::Bvh::build(m);
    ideal.iter().map(|p| bvh.closest(p, f64::MAX).map_or(f64::INFINITY, |h| h.distance)).fold(0.0, f64::max)
}

#[test]
fn a_sharp_crease_with_an_inverted_sliver_is_put_back_not_rounded_off() {
    // Arrange: a vertex of the crease is pushed over its neighbour; the fold is small and crosses the sheet
    let mut m = ridge_panel();
    let v = vertex_at(&m, 15.0, 15.0);
    m.positions[v] += Vec3::new(0.3, 2.2 * EDGE, 0.5);
    let before = m.clone();
    assert!(find_folds(&m).iter().any(|f| f.small), "the fixture must hold a fold that could be removed");

    // Act
    let report = run(&mut m);

    // Assert: a smooth patch would round the crease off by more than the feature test allows, so nothing is taken out
    assert_eq!((report.removed, report.refused.beside_feature), (0, 1), "{report:?}");
    assert!(same_mesh(&m, &before));
}

#[test]
fn a_fold_away_from_a_crease_is_removed_and_the_crease_is_not_touched() {
    // Arrange: the same kind of fold, seven cells from the crease of the ridge
    let mut m = ridge_panel();
    let v = vertex_at(&m, 8.0, 15.0);
    m.positions[v] += Vec3::new(0.2, 1.8 * EDGE, 0.2);
    let crease: Vec<Point3> = (0..61).map(|k| Point3::new(15.0, 0.5 * k as f64, ridge_z(15.0))).collect();

    // Act
    let report = run(&mut m);

    // Assert
    assert!(report.removed >= 1, "{report:?}");
    assert!(reach(&m, &crease) <= 1e-9, "the crease moved");
    assert_eq!(report.refused.fill_refused, 0, "{report:?}");
}

#[test]
fn a_small_boss_with_one_flipped_triangle_at_its_foot_stays() {
    // Arrange
    let mut m = panel();
    for p in &mut m.positions {
        p.z += 2.0 * (-((p.x - 10.0).powi(2) + (p.y - 10.0).powi(2)) / 2.5).exp();
    }
    let foot = vertex_at(&m, 13.0, 10.0);
    m.positions[foot] += Vec3::new(KNOT.0 * EDGE, KNOT.1, KNOT.2);
    let apex_vertex = vertex_at(&m, 10.0, 10.0);
    let apex = m.positions[apex_vertex];

    // Act
    let report = run(&mut m);

    // Assert
    assert!(reach(&m, &[apex]) <= 0.5 * TOL, "the boss lost its top: {report:?}");
    assert_eq!(report.refused.fill_refused, 0, "{report:?}");
}

#[test]
fn a_hook_whose_underside_faces_away_but_crosses_nothing_is_untouched() {
    // Arrange: a strip rolled up by 220 degrees above the panel, not attached to it
    let mut m = panel();
    let (rows, cols, radius) = (24usize, 4usize, 3.0);
    let base = m.positions.len() as u32;
    for i in 0..=rows {
        let angle = 220.0_f64.to_radians() * i as f64 / rows as f64;
        for j in 0..=cols {
            m.positions.push(Point3::new(
                20.0 + radius * angle.sin(),
                5.0 + j as f64 * 0.5,
                6.0 + radius - radius * angle.cos(),
            ));
        }
    }
    for i in 0..rows as u32 {
        for j in 0..cols as u32 {
            let a = base + i * (cols as u32 + 1) + j;
            let b = a + cols as u32 + 1;
            m.triangles.extend([[a, a + 1, b], [a + 1, b + 1, b]]);
        }
    }
    let before = m.clone();

    // Act
    let report = run(&mut m);

    // Assert
    assert_eq!(report.removed, 0, "{report:?}");
    assert!(same_mesh(&m, &before));
}

#[test]
fn a_hub_with_thousands_of_triangles_is_left_and_not_pulled_in() {
    // Arrange: a fan of 3000 triangles around vertex 0, one of them a "fold"
    let n = 3000u32;
    let mut m = Mesh { positions: vec![Point3::origin()], triangles: Vec::new() };
    for i in 0..n {
        let a = f64::from(i) / f64::from(n) * std::f64::consts::TAU;
        m.positions.push(Point3::new(a.cos(), a.sin(), 0.0));
    }
    (0..n).for_each(|i| m.triangles.push([0, 1 + i, 1 + (i + 1) % n]));
    let site = Site { triangles: vec![0], area: 0.0, extent: 0.01, height: 0.0, on_rim: false, crossing: true };
    let rim_tris = vec![false; m.triangles.len()];

    // Act
    let margin = with_margin(&m, &Incidence::new(&m), &rim_tris, &site);

    // Assert
    assert!(margin.is_none());
}

#[test]
fn a_cancelled_call_and_a_useless_tolerance_leave_the_mesh_alone() {
    let cancel = Cancel::new();
    cancel.cancel();
    let mut m = knotted(15.0, 15.0);
    let before = m.clone();
    let n = m.triangle_count();
    let report = remove_folds(&mut m, TOL, &cancel, n);
    assert!(report.cancelled && same_mesh(&m, &before));
    for tolerance in [0.0, -1.0, f64::NAN, f64::INFINITY] {
        assert_eq!(remove_folds(&mut m, tolerance, &Cancel::new(), n).removed, 0);
        assert!(same_mesh(&m, &before));
    }
}

#[test]
fn a_fold_in_a_patch_is_removed_but_is_no_point_of_the_scan() {
    // Arrange: the same knot, but every triangle is a patch of hole filling (none was recorded by the scanner)
    let mut m = knotted(15.0, 15.0);

    // Act
    let report = remove_folds(&mut m, TOL, &Cancel::new(), 0);

    // Assert: repaired, and nothing of the scan is said to have been removed or moved
    assert!(report.removed >= 1 && report.removed_triangles >= 1, "{report:?}");
    assert_eq!((report.largest_removed_height, report.largest_fill_deviation, report.removed_area), (0.0, 0.0, 0.0));
    assert!(report.removed_mesh.triangles.is_empty());
    assert_eq!(unresolved(&m).unwrap(), 0);
}

#[test]
fn a_fold_on_a_noisy_smooth_panel_is_removed_although_noise_makes_sharp_pairs() {
    // Arrange: small triangles (0.5 mm) with noise (0.06 mm): about 5 % of the neighbouring pairs meet at 30
    // degrees or more, but no crease runs anywhere
    let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 20.0, 20.0, 40, 40);
    m.positions.iter_mut().for_each(|p| p.z = (p.x / 6.0).sin() * (p.y / 5.0).cos());
    synth::add_noise(&mut m, 0.06, &mut Rng::new(5));
    let v = vertex_at(&m, 10.0, 10.0);
    m.positions[v] += Vec3::new(1.5, 0.2, 0.3);
    let n = m.triangle_count();
    let neighbours = m.triangle_neighbours();
    let sharp = (0..n)
        .flat_map(|t| {
            neighbours[t].iter().filter(move |&&o| o != u32::MAX && o as usize > t).map(move |&o| (t, o as usize))
        })
        .filter(|&(a, b)| m.face_normal(a).zip(m.face_normal(b)).is_some_and(|(x, y)| x.dot(&y) < 0.866))
        .count();

    // Act
    let report = remove_folds(&mut m, 0.5, &Cancel::new(), n);

    // Assert
    assert!(sharp > 100, "the noise must make sharp pairs: {sharp}");
    assert!(report.removed >= 1, "{report:?}");
    assert_eq!(report.refused.beside_feature, 0, "{report:?}");
}

#[test]
fn one_far_vertex_blinds_only_its_own_triangle() {
    // Arrange: a knot, and a triangle with a vertex at 1e300 whose cell index is out of range
    let mut m = knotted(15.0, 15.0);
    let n = m.positions.len() as u32;
    m.positions.extend([Point3::new(1e300, 0.0, 0.0), Point3::new(1e300, 1.0, 0.0), Point3::new(1e300, 0.0, 1.0)]);
    m.triangles.push([n, n + 1, n + 2]);

    // Act
    let report = run(&mut m);

    // Assert
    assert_eq!(report.skipped_triangles, 1, "{report:?}");
    assert!(report.removed >= 1, "the knot is still found: {report:?}");
}

#[test]
fn hole_filling_appends_its_triangles_and_does_not_reorder_the_scan() {
    // Arrange: a panel with a hole (the triangles around one vertex taken out)
    let full = panel();
    let hub = vertex_at(&full, 15.0, 15.0) as u32;
    let mut m = full.clone();
    m.triangles.retain(|t| !t.contains(&hub));
    let before = m.triangles.clone();

    // Act
    let filled = edit::fill_holes(&mut m, 20, true);
    edit::compact(&mut m);

    // Assert: the scan's triangles come first, in their order (the tag `t < scan_triangles` depends on it)
    assert!(filled.filled == 1 && m.triangles.len() > before.len());
    let remap: Vec<[Point3; 3]> = before.iter().map(|t| t.map(|v| full.positions[v as usize])).collect();
    let after: Vec<[Point3; 3]> =
        m.triangles.iter().take(before.len()).map(|t| t.map(|v| m.positions[v as usize])).collect();
    assert_eq!(remap, after);
}

#[test]
fn a_fill_is_judged_by_the_ring_for_a_tiny_group_allowing_a_third_of_the_fold() {
    use super::measure::Taken;
    // a group with no clean surface (one ring): what is left of the ring against a third of the fold's height
    let strays = |ring: f64, height: f64| Taken { ring_deviation: ring, height, ..Taken::default() }.strays(0.15);
    assert!(strays(0.30, 0.10));
    assert!(!strays(0.30, 0.90));
    // with clean surface around, that alone decides
    let clean = |dev: f64| Taken { clean_deviation: dev, clean_samples: 5, ring_deviation: 9.0, ..Taken::default() };
    assert!(clean(0.2).strays(0.15) && !clean(0.1).strays(0.15));
}

#[test]
fn the_final_surface_decides_how_far_the_removed_material_stood() {
    use super::measure::against_final;
    // Arrange: what is left is the plane z = 0; the removed soup has a triangle at 0.6 mm and a fold vertex at 0.9
    let left = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 10.0, 10.0, 4, 4);
    let removed = Mesh {
        positions: vec![Point3::new(5.0, 5.0, 0.6), Point3::new(5.5, 5.0, 0.6), Point3::new(5.0, 5.5, 0.6)],
        triangles: vec![[0, 1, 2]],
    };

    // Act
    let (height, deviation) = against_final(&left, &removed, &[Point3::new(2.0, 2.0, 0.9)]);

    // Assert
    assert!((height - 0.9).abs() < 1e-9 && (deviation - 0.9).abs() < 1e-9);
}
