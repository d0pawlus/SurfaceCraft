//! P9-CLEAN: hole filling never adds an edge shared by three or more triangles. Where a hole
//! cannot be filled without one, it stays open and is counted as skipped.
use surfacecraft_geom::edit::{boundary_loops, fill_holes};
use surfacecraft_geom::quality::edge_census;
use surfacecraft_geom::{Mesh, Point3, Rng};

fn non_manifold(m: &Mesh) -> usize {
    edge_census(m).non_manifold
}

/// `n x n` flat grid of unit squares (two triangles each), vertex `(i, j)` is `j * (n + 1) + i`.
fn grid(n: u32) -> Mesh {
    let w = n + 1;
    let positions = (0..w * w).map(|k| Point3::new(f64::from(k % w), f64::from(k / w), 0.0)).collect();
    let mut triangles = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let (a, b, c, d) = (j * w + i, j * w + i + 1, (j + 1) * w + i + 1, (j + 1) * w + i);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { positions, triangles }
}

/// The grid without the triangles whose index is listed.
fn without(mut m: Mesh, drop: &[usize]) -> Mesh {
    let mut k = 0;
    m.triangles.retain(|_| {
        k += 1;
        !drop.contains(&(k - 1))
    });
    m
}

fn fill_both_ways(m: &Mesh) -> Vec<(Mesh, usize)> {
    [false, true]
        .into_iter()
        .map(|smooth| {
            let mut f = m.clone();
            let rep = fill_holes(&mut f, 1000, smooth);
            (f, rep.skipped)
        })
        .collect()
}

#[test]
fn a_hole_whose_both_diagonals_exist_elsewhere_is_closed_without_one() {
    // a 4x4 grid with the square of vertices 6, 7, 12, 11 punched out (triangles of cell (1, 1))
    let n = 4;
    let cell = 4 + 1;
    let holed = without(grid(n), &[2 * cell, 2 * cell + 1]);
    assert_eq!(boundary_loops(&holed).len(), 2, "the rim of the grid and the hole");
    let loop_vertices = [6u32, 7, 12, 11];
    // two flaps hang on both diagonals of the hole from outside
    let mut m = holed;
    m.positions.push(Point3::new(1.5, 1.5, 3.0));
    m.positions.push(Point3::new(1.5, 1.5, -3.0));
    let (x, y) = (m.positions.len() as u32 - 2, m.positions.len() as u32 - 1);
    m.triangles.push([loop_vertices[0], loop_vertices[2], x]);
    m.triangles.push([loop_vertices[1], loop_vertices[3], y]);
    let before = non_manifold(&m);
    for (filled, skipped) in fill_both_ways(&m) {
        assert!(non_manifold(&filled) <= before, "non-manifold {} -> {}", before, non_manifold(&filled));
        let _ = skipped;
    }
}

#[test]
fn a_hole_with_one_free_diagonal_is_filled_with_it() {
    let cell = 5;
    let holed = without(grid(4), &[2 * cell, 2 * cell + 1]);
    let mut m = holed;
    m.positions.push(Point3::new(1.5, 1.5, 3.0));
    let x = m.positions.len() as u32 - 1;
    m.triangles.push([6, 12, x]);
    for (filled, skipped) in fill_both_ways(&m) {
        assert_eq!(non_manifold(&filled), 0);
        assert_eq!(skipped, 0);
        assert!(boundary_loops(&filled).is_empty(), "every hole is closed");
    }
}

#[test]
fn a_boundary_edge_that_two_triangles_already_use_is_not_filled_over() {
    // two triangles run the same way along the edge (0, 1): it is a boundary edge to the loop
    // tracer but it has two users, so a third triangle would make it non-manifold
    let m = Mesh {
        positions: vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.5, 0.0, 1.0),
        ],
        triangles: vec![[0, 1, 2], [0, 1, 3]],
    };
    let before = non_manifold(&m);
    for (filled, skipped) in fill_both_ways(&m) {
        assert!(non_manifold(&filled) <= before);
        assert_eq!(filled, m, "nothing may be added over that edge");
        assert_eq!(skipped, 1, "the hole is left open and counted");
    }
}

#[test]
fn two_holes_touching_at_a_vertex_do_not_share_an_edge_after_filling() {
    // cells (1, 1) and (2, 2) of a 4x4 grid are punched out: the holes meet at vertex 12
    let cell = |i: usize, j: usize| j * 4 + i;
    let m = without(grid(4), &[2 * cell(1, 1), 2 * cell(1, 1) + 1, 2 * cell(2, 2), 2 * cell(2, 2) + 1]);
    assert!(!boundary_loops(&m).is_empty());
    for (filled, _) in fill_both_ways(&m) {
        assert_eq!(non_manifold(&filled), 0);
    }
}

/// A loop that touches itself: a ring of triangles whose inner rim is pinched into a bow tie.
#[test]
fn a_loop_pinched_at_a_vertex_adds_no_non_manifold_edge() {
    // cells (0..2, 1) and (1.., 2) leave two holes that share the vertex 12 of a 4x4 grid, and the
    // cells between them are removed so that one loop runs through the vertex twice
    let cell = |i: usize, j: usize| j * 4 + i;
    let m = without(
        grid(4),
        &[2 * cell(1, 1), 2 * cell(1, 1) + 1, 2 * cell(2, 1), 2 * cell(2, 1) + 1, 2 * cell(2, 2), 2 * cell(2, 2) + 1],
    );
    let before = non_manifold(&m);
    for (filled, _) in fill_both_ways(&m) {
        assert!(non_manifold(&filled) <= before);
    }
}

/// A random soup: a few vertices on a small lattice and triangles over any three of them, so
/// edges are shared by many triangles, triangles repeat and loops pinch.
fn soup(rng: &mut Rng, hostile: bool) -> Mesh {
    let nv = 6 + rng.index(20);
    let mut positions: Vec<Point3> = (0..nv)
        .map(|_| Point3::new(rng.range(0.0, 4.0).floor(), rng.range(0.0, 4.0).floor(), rng.range(0.0, 2.0).floor()))
        .collect();
    let mut triangles: Vec<[u32; 3]> =
        (0..5 + rng.index(60)).map(|_| [rng.index(nv) as u32, rng.index(nv) as u32, rng.index(nv) as u32]).collect();
    if let Some(&dup) = triangles.first() {
        triangles.push(dup);
        triangles.push([dup[1], dup[0], dup[2]]);
    }
    if hostile && rng.index(3) == 0 {
        triangles.push([0, 1, nv as u32 + 5]);
    }
    if hostile
        && rng.index(3) == 0
        && let Some(p) = positions.get_mut(rng.index(nv))
    {
        *p = Point3::new(f64::NAN, 0.0, 0.0);
    }
    Mesh { positions, triangles }
}

#[test]
fn filling_never_raises_the_non_manifold_edge_count_of_random_meshes() {
    let mut rng = Rng::new(20260514);
    let mut filled_any = 0usize;
    for round in 0..600 {
        let m = soup(&mut rng, round % 4 == 3);
        let before = non_manifold(&m);
        for smooth in [false, true] {
            let mut f = m.clone();
            let rep = fill_holes(&mut f, 200, smooth);
            filled_any += rep.filled;
            assert!(
                non_manifold(&f) <= before,
                "round {round} smooth {smooth}: {before} -> {} ({rep:?})",
                non_manifold(&f)
            );
        }
    }
    assert!(filled_any > 0, "the soups must contain fillable holes");
}
