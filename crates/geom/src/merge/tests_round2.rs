//! Tests of review round 2: displaced flat scans, the corner rule after recovery, the guard's search
//! box and cost, the disjoint notice, welds end to end, bounded rays, coverage budget, rows.
use super::guard::{Corners, Guard, Pts};
use super::union::{Origin, Union};
use super::*;
use crate::{Point3, Transform, Vec3, synth};

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

fn moved(m: &Mesh, d: Vec3) -> Mesh {
    m.transformed(&Transform::translation(d))
}

fn flipped(m: &Mesh) -> Mesh {
    Mesh { positions: m.positions.clone(), triangles: m.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect() }
}

fn report(scans: &[Mesh], tol: f64) -> Report {
    merge(scans, &Params { tolerance: tol }, &Cancel::new(), &mut |_| {}).unwrap().1
}

fn codes(rep: &Report) -> Vec<String> {
    rep.issues.iter().map(|i| format!("{i:?}").split([' ', '{']).next().unwrap_or("").to_string()).collect()
}

#[test]
fn flat_sheets_displaced_along_their_normal_are_refused_and_sheets_apart_are_a_union() {
    let tol = 0.1;
    let a = sheet(40, 40, (0.0, 0.0), 1.0);
    let b = sheet(32, 32, (10.0, 5.0), 1.25);
    for times in [10.0, 20.0, 100.0] {
        let rep = report(&[a.clone(), moved(&b, Vec3::new(0.0, 0.0, times * tol))], tol);
        assert!(rep.refused, "{times} tolerances: {:?}", rep.issues);
        assert!(codes(&rep).contains(&"Misaligned".to_string()), "{:?}", rep.issues);
    }
    // beside each other, sharing nothing: a union of two pieces with the notice
    let rep = report(&[a.clone(), moved(&b, Vec3::new(60.0, 0.0, 0.0))], tol);
    assert!(!rep.refused && codes(&rep).contains(&"Disjoint".to_string()), "{:?}", rep.issues);
}

#[test]
fn opposite_sides_of_a_wall_and_a_hollow_part_are_not_refused() {
    let tol = 0.1;
    // the front of a wall seen by one scan, its back (2 mm behind, facing away) by the other
    let front = sheet(20, 20, (0.0, 0.0), 1.0);
    let back = flipped(&moved(&sheet(20, 20, (2.0, 2.0), 1.0), Vec3::new(0.0, 0.0, -2.0)));
    let rep = report(&[front, back], tol);
    assert!(!rep.refused, "{:?}", rep.issues);
    // a hollow sphere (outer skin and inner skin facing in) with a view of its outer skin
    let mut hollow = synth::sphere(Point3::origin(), 20.0, 40, 20);
    hollow.append(&flipped(&synth::sphere(Point3::origin(), 16.0, 32, 16)));
    let view = synth::sphere(Point3::origin(), 20.0, 44, 22);
    let cap = Mesh {
        positions: view.positions.clone(),
        triangles: (0..view.triangle_count())
            .filter(|&t| view.face_centroid(t).unwrap().x > 5.0)
            .map(|t| view.triangles[t])
            .collect(),
    };
    let rep = report(&[hollow, cap], 0.3);
    assert!(!rep.refused, "{:?}", rep.issues);
}

#[test]
fn a_shelf_over_a_floor_seen_by_different_scans_that_share_a_wall_is_not_refused() {
    // a wall in the plane x = 40 (facing -x), both scans see it; one also sees the floor (z = 0), the
    // other a shelf over most of it (z = 10, facing up)
    let wall = |h: f64| {
        let s = sheet(20, 10, (0.0, 0.0), h * 2.0);
        Mesh {
            positions: s.positions.iter().map(|p| Point3::new(40.0, p.x, p.y)).collect(),
            triangles: s.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect(),
        }
    };
    let mut floor = sheet(40, 40, (0.0, 0.0), 1.0);
    floor.append(&wall(1.0));
    let mut shelf = moved(&sheet(24, 32, (0.0, 0.0), 1.25), Vec3::new(0.0, 0.0, 10.0));
    shelf.append(&wall(0.9));
    let rep = report(&[floor, shelf], 0.1);
    assert!(!rep.refused, "{:?}", rep.issues);
}

/// A union of `pos` (scan of every vertex `scan_of`) with the triangles `tris`, kept from scan 0.
fn union_of(pos: Vec<Point3>, scan_of: Vec<u32>, tris: Vec<[u32; 3]>) -> Union {
    let origin = (0..tris.len()).map(|i| Origin::Kept(0, i as u32)).collect();
    Union {
        normals: vec![Vec3::z(); pos.len()],
        positions: pos,
        scan_of,
        triangles: tris,
        origin,
        vertex_of: vec![Vec::new()],
        welded: 0,
        max_move: 0.0,
        welds_undone: 0,
        collapsed: Vec::new(),
        initial_front: Vec::new(),
    }
}

#[test]
fn vertices_added_after_the_corner_rule_was_made_count_as_cut() {
    let scans = vec![prep::prepare(&sheet(2, 2, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let mut u = union::assemble(&scans, &[vec![false; 8]]);
    let rule = Corners::new(&u, &scans, &[]);
    // a hole between three vertices the recovery added (all past the rule's vertices)
    let n = u.positions.len() as u32;
    u.positions.extend([Point3::new(5.0, 0.0, 0.0), Point3::new(6.0, 0.0, 0.0), Point3::new(5.0, 1.0, 0.0)]);
    u.normals.extend([Vec3::z(); 3]);
    u.scan_of.extend([0; 3]);
    assert!(rule.allows(Pts::of(&u), [n, n + 1, n + 2]));
    // the rule's own rim vertices of one piece without a shared rim edge are still refused
    assert!(!rule.allows(Pts::of(&u), [0, 4, 8]));
}

fn guard_over<'a>(scans: &'a [prep::Scan], u: &Union, tol: f64) -> Guard<'a> {
    Guard::new(scans, u, tol, 1.0)
}

#[test]
fn a_parallel_layer_within_the_tolerance_is_refused_although_the_boxes_do_not_meet() {
    let scans = vec![prep::prepare(&sheet(1, 1, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let tol = 0.1;
    let u = union_of(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
            Point3::new(0.5, 0.5, 0.5 * tol),
            Point3::new(3.0, 0.5, 0.5 * tol),
            Point3::new(0.5, 3.0, 0.5 * tol),
        ],
        vec![0, 0, 0, 1, 1, 1],
        Vec::new(),
    );
    let mut g = guard_over(&scans, &u, tol);
    assert!(g.restorable(Pts::of(&u), [0, 1, 2]));
    assert!(!g.restorable(Pts::of(&u), [3, 4, 5]), "a layer of another scan 0.5 tol over a recorded triangle");
}

#[test]
fn a_large_candidate_is_tested_against_the_new_triangles_within_a_bounded_cost() {
    let scans = vec![prep::prepare(&sheet(1, 1, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let mut pos = Vec::new();
    for i in 0..500 {
        let x = 2.0 * f64::from(i);
        pos.extend([Point3::new(x, 0.0, 1.0), Point3::new(x + 1.0, 0.0, 1.0), Point3::new(x, 1.0, 1.0)]);
    }
    // a large thin triangle passing through the small one at x = 600
    pos.extend([Point3::new(-1.0, 0.3, 0.0), Point3::new(1000.0, 0.3, 0.0), Point3::new(600.2, 0.3, 3.0)]);
    let u = union_of(pos, vec![0; 1503], Vec::new());
    let mut g = guard_over(&scans, &u, 0.01);
    for i in 0..500u32 {
        assert!(g.restorable(Pts::of(&u), [3 * i, 3 * i + 1, 3 * i + 2]));
    }
    assert!(!g.restorable(Pts::of(&u), [1500, 1501, 1502]), "it pierces the small triangle at x = 600");
    // the large candidate is compared with each new triangle once (500), not once per cell
    assert!(g.compared() <= 600, "{}", g.compared());
}

#[test]
fn a_stray_ray_hit_does_not_join_scans_that_share_nothing() {
    let r = |a: usize, b: usize, over: usize| PairResidual { a, b, samples: 20_000, over, ..PairResidual::default() };
    let issues = pair_issues(2, &[r(0, 1, 201), r(1, 0, 0)]);
    assert!(issues.iter().any(|i| matches!(i, Issue::Disjoint { .. })), "{issues:?}");
    let displaced = pair_issues(2, &[r(0, 1, 5000), r(1, 0, 5000)]);
    assert!(
        displaced.iter().any(|i| matches!(i, Issue::Misaligned { .. }))
            && !displaced.iter().any(|i| matches!(i, Issue::Disjoint { .. }))
    );
}

#[test]
fn welds_end_to_end_collapse_count_and_leave_no_doubled_edge() {
    // scan 0: a triangle (0, 1, 2); scan 1: a triangle (3, 4, 5) whose corners 3 and 4 lie within the
    // radius of corner 0 of scan 0: both weld to 0 and the triangle collapses
    let mut u = union_of(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.01, 0.0, 0.0),
            Point3::new(0.0, 0.01, 0.0),
            Point3::new(-1.0, -1.0, 0.0),
        ],
        vec![0, 0, 0, 1, 1, 1],
        vec![[0, 1, 2], [3, 5, 4]],
    );
    u.origin = vec![Origin::Kept(0, 0), Origin::Kept(1, 0)];
    let to = u.weld_targets(&[0, 1, 2, 3, 4, 5], 0.05);
    assert_eq!(to, vec![0, 1, 2, 0, 0, 5]);
    u.remap(&to);
    assert_eq!(u.collapsed, vec![(1, 0)], "two corners of one triangle welded into one: collapsed");
    assert_eq!(u.triangles, vec![[0, 1, 2]]);
    // the fold rule alone: a triangle squeezed to a sliver by a weld is found
    let v = union_of(
        vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 0.05, 0.0),
            Point3::new(1.0, 0.004, 0.0),
        ],
        vec![0, 0, 1, 0],
        vec![[0, 1, 2]],
    );
    assert_eq!(v.clashing(&[0, 1, 3, 3]), vec![2], "below a tenth of its area");
}

#[test]
fn a_weld_into_doubled_corners_is_undone_and_reported() {
    let g = sheet(4, 4, (0.0, 0.0), 1.0);
    let scans = vec![
        prep::prepare(&g, &Cancel::new()).unwrap(),
        prep::prepare(&moved(&g, Vec3::new(0.0, 0.0, 0.001)), &Cancel::new()).unwrap(),
    ];
    // the first row of both removed: both cut along y = 1, where the welds would give each kept
    // triangle of the second's next row an edge of the first's, run the same way
    let mut cut = vec![false; 32];
    cut.iter_mut().take(8).for_each(|c| *c = true);
    let mut u = union::assemble(&scans, &[cut.clone(), cut]);
    let ends: Vec<u32> = (0..u.positions.len() as u32).collect();
    assert!(!u.clashing(&u.weld_targets(&ends, 0.01)).is_empty());
    assert!(u.weld_front(&scans, 0.01, &Cancel::new()).is_some());
    assert!(u.welds_undone > 0, "the clashing welds are reported as undone");
    let mut edges = std::collections::HashMap::new();
    for t in &u.triangles {
        for k in 0..3 {
            *edges.entry((t[k], t[(k + 1) % 3])).or_insert(0) += 1;
        }
    }
    assert!(edges.values().all(|&n| n == 1), "no directed edge twice after the last undo");
}

#[test]
fn a_pile_of_triangles_at_one_place_does_not_hang_the_comparison() {
    // 200 000 triangles of distinct vertices at the same three places: no duplicate by index
    let mut pile = Mesh::default();
    for k in 0..200_000u32 {
        pile.positions.extend([Point3::new(0.0, 0.0, 5.0), Point3::new(1.0, 0.0, 5.0), Point3::new(0.0, 1.0, 5.0)]);
        pile.triangles.push([3 * k, 3 * k + 1, 3 * k + 2]);
    }
    let r = merge(&[sheet(4, 4, (0.0, 0.0), 1.0), pile], &Params { tolerance: 0.1 }, &Cancel::new(), &mut |_| {});
    assert!(r.is_ok());
}

#[test]
fn every_removed_triangle_gets_a_sample_when_the_budget_is_spread() {
    // 1000 unit triangles at a tolerance so small that the subdivision would need 4096 points each
    let m = sheet(25, 20, (0.0, 0.0), 1.0);
    let empty = Mesh::default();
    let bvh = crate::Bvh::build(&empty);
    let r = check::Built { mesh: &empty, bvh: &bvh, origin: &[], scan_of: &[] };
    let n = m.triangle_count();
    let cov = check::coverage(&r, (n, &|i| m.triangle(i)), 1e-4, &Cancel::new()).unwrap();
    assert!(cov.samples <= check::MAX_REMOVED_SAMPLES && cov.samples >= n, "{}", cov.samples);
    assert!((cov.area - m.area()).abs() < 1e-6 * m.area());
}

#[test]
fn two_layers_of_two_scans_fire_the_stacked_share() {
    let tol = 0.1;
    let s = sheet(4, 4, (0.0, 0.0), 1.0);
    let mut pos = s.positions.clone();
    pos.extend(moved(&s, Vec3::new(0.3, 0.2, 0.5 * tol)).positions);
    let mut tris = s.triangles.clone();
    tris.extend(s.triangles.iter().map(|t| t.map(|v| v + 25)));
    let mut scan_of = vec![0u32; 25];
    scan_of.extend([1u32; 25]);
    let mut u = union_of(pos, scan_of, tris);
    u.origin = (0..64).map(|i| Origin::Kept(u32::from(i >= 32), i as u32 % 32)).collect();
    let mesh = Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() };
    let bvh = crate::Bvh::build(&mesh);
    let r = check::Built { mesh: &mesh, bvh: &bvh, origin: &u.origin, scan_of: &u.scan_of };
    let (share, triangles) = check::stacked_share(&r, tol, &Cancel::new()).unwrap();
    assert!(share > STACKED_SHARE && triangles > 0, "{share} {triangles}");
}

#[test]
fn too_many_triangles_are_refused_before_anything_is_built() {
    let three = vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
    let big = Mesh { positions: three.clone(), triangles: vec![[0, 1, 2]; MAX_TRIANGLES] };
    let small = Mesh { positions: three, triangles: vec![[0, 1, 2]] };
    let r = merge(&[big, small], &Params { tolerance: 0.1 }, &Cancel::new(), &mut |_| {});
    assert!(matches!(r, Err(MergeError::Params(_))), "{r:?}");
}

#[test]
fn a_recovered_speck_is_taken_back_and_forgotten_by_the_guard() {
    let scans = vec![prep::prepare(&sheet(1, 1, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let big = sheet(20, 20, (0.0, 0.0), 1.0);
    let mut pos = big.positions.clone();
    pos.extend([Point3::new(40.0, 0.0, 0.0), Point3::new(40.5, 0.0, 0.0), Point3::new(40.0, 0.5, 0.0)]);
    // a triangle standing through the speck
    pos.extend([Point3::new(40.1, 0.1, -1.0), Point3::new(40.3, 0.1, -1.0), Point3::new(40.2, 0.1, 1.0)]);
    let n = big.positions.len() as u32;
    let pierce = [n + 3, n + 4, n + 5];
    let count = pos.len();
    let mut u = union_of(pos, vec![0; count], big.triangles.clone());
    // recovered as the recovery does it: through the guard, then added to the union
    let mut g = guard_over(&scans, &u, 0.01);
    assert!(g.restorable(Pts::of(&u), [n, n + 1, n + 2]));
    u.triangles.push([n, n + 1, n + 2]);
    u.origin.push(Origin::Recovered(0, 0));
    assert!(!g.restorable(Pts::of(&u), pierce), "recorded: a triangle through it is refused");
    assert_eq!(fill::drop_specks(&mut u, &mut g), 1);
    assert_eq!(u.triangles.len(), big.triangles.len());
    assert!(g.restorable(Pts::of(&u), pierce), "taken back: the guard forgot it");
}

#[test]
fn two_pieces_over_each_other_tied_by_a_third_are_the_part_not_a_misalignment() {
    // 0 and 2 share nothing and lie over each other displaced (a step); 1 agrees with both
    let r = |a: usize, b: usize, near: usize, over: usize| PairResidual {
        a,
        b,
        samples: 20_000,
        near,
        within: near,
        over,
        ..PairResidual::default()
    };
    let pairs =
        [r(0, 1, 6000, 0), r(1, 0, 4700, 0), r(1, 2, 4400, 0), r(2, 1, 10_000, 0), r(0, 2, 0, 900), r(2, 0, 0, 2200)];
    let issues = pair_issues(3, &pairs);
    assert!(issues.is_empty(), "{issues:?}");
    // the same two alone are refused
    let alone = pair_issues(2, &[r(0, 1, 0, 900), r(1, 0, 0, 2200)]);
    assert!(alone.iter().any(|i| matches!(i, Issue::Misaligned { .. })), "{alone:?}");
}

#[test]
fn a_scan_s_own_second_layer_comes_back_but_another_scan_s_does_not() {
    let scans = vec![prep::prepare(&sheet(1, 1, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let tol = 0.1;
    let mut pos = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(4.0, 0.0, 0.0), Point3::new(0.0, 4.0, 0.0)];
    let inner = [Point3::new(0.5, 0.5, 0.5 * tol), Point3::new(3.0, 0.5, 0.5 * tol), Point3::new(0.5, 3.0, 0.5 * tol)];
    pos.extend(inner);
    pos.extend(inner);
    // 0..3 kept of scan 0; 3..6 scan 0's own second layer; 6..9 the same layer from scan 1
    let u = union_of(pos, vec![0, 0, 0, 0, 0, 0, 1, 1, 1], vec![[0, 1, 2]]);
    let mut g = guard_over(&scans, &u, tol);
    assert!(g.restorable(Pts::of(&u), [3, 4, 5]), "the scan's own layer, as scanned");
    let v = union_of(u.positions.clone(), u.scan_of.clone(), vec![[0, 1, 2]]);
    let mut h = guard_over(&scans, &v, tol);
    assert!(!h.restorable(Pts::of(&v), [6, 7, 8]), "another scan's layer over it is a stack");
    assert!(!h.accept(Pts::of(&v), [3, 4, 5]), "a seam triangle is never let cover");
}
