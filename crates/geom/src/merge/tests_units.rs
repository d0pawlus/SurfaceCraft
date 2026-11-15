//! Unit tests of the merge's parts: ownership, eating back, orientation, border distance, welding,
//! restoring, the corner rule, the guard, the hole fill and its repairs.
use super::guard::{Corners, Guard, Pts};
use super::union::{Origin, Union};
use super::*;
use crate::{Point3, Vec3, synth};

/// A flat grid of `nx` × `ny` cells of size `h` in the plane z = 0 from `(x0, y0)`, facing +z.
fn sheet(nx: usize, ny: usize, (x0, y0): (f64, f64), h: f64) -> Mesh {
    let mut m = Mesh::default();
    for j in 0..=ny {
        for i in 0..=nx {
            m.positions.push(Point3::new(x0 + h * i as f64, y0 + h * j as f64, 0.0));
        }
    }
    let at = |i: usize, j: usize| (j * (nx + 1) + i) as u32;
    for j in 0..ny {
        for i in 0..nx {
            m.triangles.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            m.triangles.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }
    m
}

fn merged(scans: &[Mesh], tol: f64) -> (Mesh, Report) {
    let (m, rep) = merge(scans, &Params { tolerance: tol }, &Cancel::new(), &mut |_| {}).unwrap();
    (m.unwrap(), rep)
}

fn prepared(m: &Mesh) -> prep::Scan {
    prep::prepare(m, &Cancel::new()).unwrap()
}

/// x of the centres of the result's triangles that came from the triangles of `scan` (by matching
/// the result's vertices against the scan's vertex positions), in the middle band of y.
fn kept_x(result: &Mesh, scan: &Mesh) -> Vec<f64> {
    let own: std::collections::HashSet<[u64; 3]> =
        scan.positions.iter().map(|p| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]).collect();
    let is_own = |v: u32| {
        let p = result.positions[v as usize];
        own.contains(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
    };
    (0..result.triangle_count())
        .filter(|&t| result.triangles[t].iter().all(|&v| is_own(v)))
        .filter_map(|t| result.face_centroid(t))
        .filter(|c| (30.0..=70.0).contains(&c.y))
        .map(|c| c.x)
        .collect()
}

#[test]
fn the_scan_farther_from_its_own_border_keeps_the_overlap() {
    // A over x 0..60, B over x 40..100 (another tessellation), C over x 80..140; y 0..100. In the
    // middle band of y the border distance of A is 60 - x, of B x - 40 (and 100 - x): the seams lie
    // midway, at x = 50 and x = 90. The scan listed first keeping everything would put A's edge at 60.
    let a = sheet(30, 50, (0.0, 0.0), 2.0);
    let b = sheet(24, 40, (40.0, 0.0), 2.5);
    let c = sheet(30, 50, (80.0, 0.0), 2.0);
    let (m, rep) = merged(&[a.clone(), b.clone(), c.clone()], 0.1);
    let (xa, xb, xc) = (kept_x(&m, &a), kept_x(&m, &b), kept_x(&m, &c));
    let max = |v: &[f64]| v.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let min = |v: &[f64]| v.iter().copied().fold(f64::INFINITY, f64::min);
    // the seam within two cells of the midway lines (measured 48.7 / 51.3 and 88.7 / 91.3)
    assert!((46.0..=54.0).contains(&max(&xa)), "A ends at {}", max(&xa));
    assert!((46.0..=54.0).contains(&min(&xb)) && (86.0..=94.0).contains(&max(&xb)), "{} {}", min(&xb), max(&xb));
    assert!((86.0..=94.0).contains(&min(&xc)), "C starts at {}", min(&xc));
    assert_eq!(rep.after.pieces, 1);
}

#[test]
fn copies_within_the_tie_band_go_to_the_scan_listed_first() {
    let a = sheet(20, 20, (0.0, 0.0), 1.0);
    // a copy moved by a twentieth of an edge: border distances differ by less than one median edge
    let b = sheet(20, 20, (0.05, 0.0), 1.0);
    let (m, rep) = merged(&[a.clone(), b.clone(), a.clone()], 0.1);
    assert_eq!(rep.scans[1].kept + rep.scans[2].kept, 0, "{:?}", rep.scans);
    assert_eq!(m.triangle_count(), a.triangle_count());
}

#[test]
fn redundant_triangles_on_the_edge_of_a_kept_region_are_eaten_back() {
    let g = sheet(12, 12, (0.0, 0.0), 1.0);
    let scans = vec![prepared(&g), prepared(&g)];
    let n = g.triangle_count();
    let mut removed = vec![vec![false; n], vec![false; n]];
    removed[1][n / 2] = true;
    let eaten = own::eat_back(&scans, &mut removed, 0.1, &Cancel::new()).unwrap();
    assert!(eaten > 0);
    assert!(removed[0].iter().all(|r| !r), "the scan listed first eats nothing of what the other keeps");
    assert!(removed[1].iter().filter(|r| **r).count() > 1);
}

#[test]
fn exact_duplicate_triangles_of_a_scan_are_left_out() {
    let mut g = sheet(6, 6, (0.0, 0.0), 1.0);
    let t = g.triangles[10];
    g.triangles.push([t[1], t[2], t[0]]);
    let s = prepared(&g);
    assert!(s.non_manifold >= 1);
    assert_eq!(s.topology.non_manifold_edges, 0);
    assert_eq!(s.topology.unoriented_edges, 0);
}

#[test]
fn inner_skins_and_debris_are_not_refused_but_reported() {
    let mut hollow = synth::sphere(Point3::origin(), 20.0, 32, 16);
    let inner = synth::sphere(Point3::origin(), 15.0, 24, 12);
    hollow.append(&Mesh {
        positions: inner.positions,
        triangles: inner.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect(),
    });
    let speck = synth::sphere(Point3::new(60.0, 0.0, 0.0), 0.5, 8, 4);
    hollow.append(&Mesh {
        positions: speck.positions,
        triangles: speck.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect(),
    });
    assert_eq!(nest::negative_pieces(&hollow, &Cancel::new()).unwrap(), (0, 1, 1));
    let other = synth::sphere(Point3::new(200.0, 0.0, 0.0), 5.0, 16, 8);
    let (_, rep) = merged(&[hollow, other], 0.2);
    assert!(!rep.refused);
    assert!(rep.issues.contains(&Issue::InnerSkins { scan: 0, skins: 1, debris: 1 }), "{:?}", rep.issues);
}

#[test]
fn border_distance_is_infinite_on_a_closed_scan_and_reaches_the_middle_of_a_tube() {
    let closed = prepared(&synth::sphere(Point3::origin(), 10.0, 16, 8));
    assert!(closed.border_distance.iter().all(|d| d.is_infinite()));
    // an open tube of length 40 with a border at each end: the middle ring lies 20 from both
    let tube = prepared(&synth::cylinder(Point3::origin(), Vec3::z(), 5.0, 40.0, 24, 40));
    let far = tube.border_distance.iter().copied().fold(0.0, f64::max);
    assert!((19.0..=21.0).contains(&far), "{far}");
    let ends = tube.mesh.positions.iter().zip(&tube.border_distance).filter(|(p, _)| p.z < 1e-9 || p.z > 40.0 - 1e-9);
    assert!(ends.into_iter().all(|(_, d)| *d == 0.0));
}

/// A union of the triangles `tris` over `pos`, vertex `v` of scan `scan_of[v]`, triangle `i` kept from
/// scan `of[i]` (triangle index `i`).
fn union_of(pos: Vec<Point3>, scan_of: Vec<u32>, tris: Vec<[u32; 3]>) -> Union {
    let origin = tris.iter().enumerate().map(|(i, _)| Origin::Kept(scan_of[tris[i][0] as usize], i as u32)).collect();
    Union {
        normals: vec![Vec3::z(); pos.len()],
        positions: pos,
        scan_of,
        triangles: tris,
        origin,
        vertex_of: Vec::new(),
        welded: 0,
        max_move: 0.0,
        welds_undone: 0,
        collapsed: Vec::new(),
        initial_front: Vec::new(),
    }
}

#[test]
fn a_weld_goes_to_an_earlier_scan_and_never_chains() {
    let r = 1.0;
    // vertex 0 of scan 0, vertex 1 of scan 1 at 0.6 r, vertex 2 of scan 2 at 1.2 r from vertex 0
    let pos = vec![Point3::origin(), Point3::new(0.6 * r, 0.0, 0.0), Point3::new(1.2 * r, 0.0, 0.0)];
    let u = union_of(pos, vec![0, 1, 2], Vec::new());
    let to = u.weld_targets(&[0, 1, 2], r);
    assert_eq!(to, vec![0, 0, 2], "1 goes to 0; 2 is within r of 1 only, which is welded itself: no chain");
}

#[test]
fn a_weld_that_would_double_or_flip_a_triangle_is_found() {
    // triangle (0, 1, 2) of scan 0 and (3, 5, 4) of scan 1 lying on it with the other winding
    let pos = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(0.001, 0.0, 0.0),
        Point3::new(1.0, 0.001, 0.0),
        Point3::new(0.0, 1.001, 0.0),
    ];
    let u = union_of(pos.clone(), vec![0, 0, 0, 1, 1, 1], vec![[0, 1, 2], [3, 5, 4]]);
    let doubled = u.clashing(&[0, 1, 2, 0, 1, 2]);
    assert_eq!(doubled, vec![5], "the same three corners in two triangles: the culprit is the last welded corner");
    // a sliver whose apex welds across its base: flipped
    let pos = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(2.0, 0.0, 0.0),
        Point3::new(1.0, 0.05, 0.0),
        Point3::new(1.0, -0.05, 0.0),
    ];
    let u = union_of(pos, vec![0, 0, 1, 0], vec![[0, 1, 2]]);
    assert_eq!(u.clashing(&[0, 1, 3, 3]), vec![2], "the apex welded to the other side turns the triangle over");
}

#[test]
fn a_removed_triangle_is_not_restored_over_a_triangle_with_the_same_corners() {
    let g = sheet(4, 4, (0.0, 0.0), 1.0);
    let back =
        Mesh { positions: g.positions.clone(), triangles: g.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect() };
    let scans = vec![prepared(&g), prepared(&back)];
    let removed = vec![vec![false; g.triangle_count()], vec![true; g.triangle_count()]];
    let lost = vec![vec![false; g.positions.len()], vec![true; g.positions.len()]];
    let mut u = union::assemble(&scans, &removed);
    let restored = union::restore(&mut u, &scans, (&removed, &lost), &mut |_, _| true, &Cancel::new()).unwrap();
    assert_eq!(restored, 0, "every triangle of the second scan has the corners of one of the first, reversed");
}

#[test]
fn the_corner_rule_never_bridges_two_rims() {
    // two sheets side by side, 0.5 apart, both untouched: every vertex is a rim vertex of its piece
    let mut m = sheet(2, 2, (0.0, 0.0), 1.0);
    m.append(&sheet(2, 2, (2.5, 0.0), 1.0));
    let scans = vec![prepared(&m)];
    let u = union::assemble(&scans, &[vec![false; m.triangle_count()]]);
    let rim_a = (2, 5); // (2, 0) and (2, 1): the right rim of the first sheet
    let rim_b = (9, 12); // (2.5, 0) and (2.5, 1): the left rim of the second
    let none = Corners::new(&u, &scans, &[]);
    assert!(!none.allows(Pts::of(&u), [rim_a.0, rim_b.0, rim_a.1]), "three rim vertices, two pieces");
    // with vertices 1 and 2 cut: a cut vertex with a rim vertex of each piece is refused, a cut vertex
    // with a rim edge of one piece (closing that edge) is allowed
    let cut = Corners::new(&u, &scans, &[[2, 1, 4]]);
    assert!(!cut.allows(Pts::of(&u), [rim_a.1, 2, rim_b.0]), "rims of two pieces in one triangle");
    assert!(cut.allows(Pts::of(&u), [2, rim_b.0, rim_b.1]), "a cut vertex and one rim edge");
}

#[test]
fn a_long_triangle_beyond_the_grid_is_still_tested() {
    let scans = vec![prepared(&sheet(2, 2, (0.0, 0.0), 1.0))];
    let mut u = union::assemble(&scans, &[vec![true; 8]]);
    // a long thin triangle (100 cells of 1 along x) recorded, then a small one crossing it at x = 90
    u.positions.extend([
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(100.0, 0.0, 0.0),
        Point3::new(50.0, 2.0, 0.0),
        Point3::new(90.0, 0.2, -1.0),
        Point3::new(90.0, 0.2, 1.0),
        Point3::new(90.5, 2.0, 0.0),
    ]);
    u.normals.extend([Vec3::z(); 6]);
    u.scan_of.extend([0; 6]);
    let mut guard = Guard::new(&scans, &u, 0.01, 1.0);
    let n = u.positions.len() as u32 - 6;
    assert!(guard.restorable(Pts::of(&u), [n, n + 1, n + 2]));
    assert!(!guard.restorable(Pts::of(&u), [n + 3, n + 4, n + 5]), "it pierces the long one at x = 90");
}

/// A disc of `k` spokes: centre 0, inner ring 1..=k, outer ring k+1..=2k, the fan around the centre
/// and a strip to the outer ring, in the plane z = 0 facing +z.
fn disc(k: usize) -> Mesh {
    let mut m = Mesh { positions: vec![Point3::origin()], triangles: Vec::new() };
    for (r, _) in [(5.0, 0), (10.0, 1)] {
        for i in 0..k {
            let a = std::f64::consts::TAU * i as f64 / k as f64;
            m.positions.push(Point3::new(r * a.cos(), r * a.sin(), 0.0));
        }
    }
    let (inner, outer) = (|i: usize| 1 + (i % k) as u32, |i: usize| (1 + k + i % k) as u32);
    for i in 0..k {
        m.triangles.push([0, inner(i), inner(i + 1)]);
        m.triangles.push([inner(i), outer(i), outer(i + 1)]);
        m.triangles.push([inner(i), outer(i + 1), inner(i + 1)]);
    }
    m
}

/// Removes the fan of a disc of `k` spokes (and with `rim` also one strip triangle reaching the outer
/// border) and fills the small holes: (loops filled, open seam edges left).
fn fill_disc(k: usize, rim: bool) -> (usize, usize) {
    let d = disc(k);
    let scans = vec![prepared(&d)];
    let mut removed: Vec<bool> = (0..d.triangle_count()).map(|t| t % 3 == 0).collect();
    if rim {
        // the two strip triangles of the first spoke: the hole reaches the outer border
        removed[1] = true;
        removed[2] = true;
    }
    let removed = vec![removed];
    let mut u = union::assemble(&scans, &removed);
    let mut guard = Guard::new(&scans, &u, 0.1, 4.0);
    let (filled, _, _) = fill::fill_small(&mut u, &scans, &mut guard, &Cancel::new()).unwrap();
    (filled, u.front(&scans).len())
}

#[test]
fn small_seam_holes_up_to_the_limit_are_filled_and_openings_are_not() {
    assert_eq!(fill_disc(3, false), (1, 0));
    assert_eq!(fill_disc(fill::SMALL_LOOP_EDGES, false), (1, 0));
    assert_eq!(fill_disc(fill::SMALL_LOOP_EDGES + 1, false).0, 0, "a loop longer than the limit stays open");
    // a hole that reaches the scan's own border is an opening: none of its loop is a closed seam loop
    assert_eq!(fill_disc(12, true).0, 0);
}

#[test]
fn seam_triangles_that_pinch_a_vertex_or_border_an_unfillable_hole_are_taken_back() {
    let scans = vec![prepared(&sheet(2, 2, (0.0, 0.0), 1.0))];
    // two seam triangles meeting at vertex 0 only (a bow tie), and one more elsewhere
    let pos = vec![
        Point3::origin(),
        Point3::new(1.0, 0.0, 0.0),
        Point3::new(0.0, 1.0, 0.0),
        Point3::new(-1.0, 0.0, 0.0),
        Point3::new(0.0, -1.0, 0.0),
        Point3::new(5.0, 5.0, 0.0),
        Point3::new(6.0, 5.0, 0.0),
        Point3::new(5.0, 6.0, 0.0),
    ];
    let mut u = union_of(pos, vec![0; 8], vec![[0, 1, 2], [0, 3, 4], [5, 6, 7]]);
    u.origin = vec![Origin::Seam; 3];
    let mut guard = Guard::new(&scans, &u, 0.1, 4.0);
    assert_eq!(fill::unpinch(&mut u, &mut guard), 2, "both fans at the pinch go, the far triangle stays");
    assert_eq!(u.triangles, vec![[5, 6, 7]]);
    // a hole of a few edges that could not be filled: the seam triangles at its vertices go
    assert_eq!(fill::widen(&mut u, &mut guard, &[vec![5, 6, 9]]), 1);
    assert!(u.triangles.is_empty());
    // a pinch the scans already had is left alone: the seam triangle (0, 2, 5) joins one of its fans
    // and makes no new one
    let mut v = union_of(
        vec![
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ],
        vec![0; 6],
        vec![[0, 1, 2], [0, 3, 4], [0, 2, 5]],
    );
    v.origin[2] = Origin::Seam;
    assert_eq!(fill::unpinch(&mut v, &mut guard), 0);
}

#[test]
fn coverage_of_an_empty_result_is_finite() {
    let empty = Mesh::default();
    let bvh = crate::Bvh::build(&empty);
    let r = check::Built { mesh: &empty, bvh: &bvh, origin: &[], scan_of: &[] };
    let t = [Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
    let cov = check::coverage(&r, (1, &|_| Some(t)), 0.1, &Cancel::new()).unwrap();
    assert!(cov.worst.is_finite() && cov.share() == 0.0 && cov.samples > 0, "{cov:?}");
}

/// A sheet over the columns at `xs` and `ny` unit rows, facing +z.
fn sheet_x(xs: &[f64], ny: usize) -> Mesh {
    let mut m = Mesh::default();
    for j in 0..=ny {
        for &x in xs {
            m.positions.push(Point3::new(x, j as f64, 0.0));
        }
    }
    let w = xs.len();
    let at = |i: usize, j: usize| (j * w + i) as u32;
    for j in 0..ny {
        for i in 0..w - 1 {
            m.triangles.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            m.triangles.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }
    m
}

#[test]
fn surface_beyond_the_edge_of_a_kept_region_is_not_eaten_even_within_the_tolerance() {
    // B is A with one more column 0.05 wide beyond A's border (half the tolerance): that column is
    // not on A's surface, though its far corners lie within the tolerance of A's border edge
    let cols: Vec<f64> = (0..=10).map(f64::from).collect();
    let mut wide = cols.clone();
    wide.push(10.05);
    let (a, b) = (sheet_x(&cols, 6), sheet_x(&wide, 6));
    let scans = vec![prepared(&a), prepared(&b)];
    let mut removed = vec![vec![false; a.triangle_count()], vec![false; b.triangle_count()]];
    // B already lost everything left of x = 9
    for (t, r) in removed[1].iter_mut().enumerate() {
        *r = b.face_centroid(t).unwrap().x < 9.0;
    }
    own::eat_back(&scans, &mut removed, 0.1, &Cancel::new()).unwrap();
    let thin_kept = (0..b.triangle_count()).filter(|&t| b.face_centroid(t).unwrap().x > 10.0 && !removed[1][t]).count();
    assert_eq!(thin_kept, 12, "the thin column stays");
    let on_a = (0..b.triangle_count())
        .filter(|&t| (9.0..10.0).contains(&b.face_centroid(t).unwrap().x) && !removed[1][t])
        .count();
    assert_eq!(on_a, 0, "the column on A is eaten");
}
