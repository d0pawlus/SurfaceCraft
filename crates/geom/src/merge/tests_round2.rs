//! Tests of review round 2: displaced flat scans, the corner rule after recovery, the guard's search
//! box and cost, the disjoint notice, welds end to end, bounded rays, coverage budget, rows.
use super::guard::{Corners, Guard, Pts};
use super::union::{Origin, Union};
use super::*;
use crate::{Point3, Transform, Vec3, synth};

pub(super) fn sheet(nx: usize, ny: usize, (x0, y0): (f64, f64), h: f64) -> Mesh {
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

pub(super) fn moved(m: &Mesh, d: Vec3) -> Mesh {
    m.transformed(&Transform::translation(d))
}

pub(super) fn flipped(m: &Mesh) -> Mesh {
    Mesh { positions: m.positions.clone(), triangles: m.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect() }
}

pub(super) fn report(scans: &[Mesh], tol: f64) -> Report {
    let rep = merge(scans, &Params { tolerance: tol, ..Params::default() }, &Cancel::new(), &mut |_| {}).unwrap().1;
    let shares: Vec<String> = rep
        .pairs
        .iter()
        .map(|p| format!("{}-{} over_share {:.4} agree_share {:.4}", p.a, p.b, p.over_share(), p.agree_share()))
        .collect();
    eprintln!("SHARES {shares:?}");
    rep
}

pub(super) fn warns(rep: &Report) -> bool {
    rep.issues.iter().any(|i| matches!(i, Issue::LayersApart { .. }))
}

pub(super) fn codes(rep: &Report) -> Vec<String> {
    rep.issues.iter().map(|i| format!("{i:?}").split([' ', '{']).next().unwrap_or("").to_string()).collect()
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
        vec![[0, 1, 2]],
    );
    // the kept triangle's box (z = 0) and the layer's (z = 0.05) do not meet: only the box grown by
    // the tolerance finds it
    let mut g = guard_over(&scans, &u, tol);
    assert!(!g.restorable(Pts::of(&u), [3, 4, 5]), "a layer of another scan 0.5 tol over a kept triangle");
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
    assert_eq!(g.compared(), 500);
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
    // the rows count the collapsed triangle as such, not as removed
    let one = |p: [Point3; 3]| Mesh { positions: p.to_vec(), triangles: vec![[0, 1, 2]] };
    let scans = vec![
        prep::prepare(&one([u.positions[0], u.positions[1], u.positions[2]]), &Cancel::new()).unwrap(),
        prep::prepare(
            &one([Point3::new(0.01, 0.0, 0.0), Point3::new(-1.0, -1.0, 0.0), Point3::new(0.0, 0.01, 0.0)]),
            &Cancel::new(),
        )
        .unwrap(),
    ];
    let mut rows = vec![ScanRow { triangles: 1, ..ScanRow::default() }; 2];
    let gone = gone_ids(&scans, &[vec![false], vec![false]], &u);
    fill_rows(&scans, &gone, &u, &mut rows);
    assert_eq!((rows[1].kept, rows[1].collapsed, rows[1].removed), (0, 1, 0), "{:?}", rows[1]);
    assert_eq!((rows[0].kept, rows[0].removed), (1, 0));
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
    let r = merge(&[big, small], &Params { tolerance: 0.1, ..Params::default() }, &Cancel::new(), &mut |_| {});
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
    // a seam triangle on scanned surface (the 4 x 4 sheet supports it), refused for covering alone
    let support = vec![prep::prepare(&sheet(4, 4, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let mut k = guard_over(&support, &v, tol);
    k.corners = None;
    assert!(!k.accept(Pts::of(&v), [3, 4, 5]), "a seam triangle is never let cover");
    assert_eq!((k.crossing, k.unsupported, k.bad_corners), (1, 0, 0));
}

fn inside_out(m: Mesh) -> Mesh {
    flipped(&m)
}

#[test]
fn small_cavities_are_inner_skins_and_many_small_loose_inverted_pieces_are_inverted() {
    // a solid with two small cavities of about 0.6 % of its area each (together over 1 %)
    let mut solid = synth::sphere(Point3::origin(), 20.0, 32, 16);
    for x in [-8.0, 8.0] {
        solid.append(&inside_out(synth::sphere(Point3::new(x, 0.0, 0.0), 1.55, 12, 6)));
    }
    assert_eq!(nest::negative_pieces(&solid, &Cancel::new()).unwrap().counts(), (0, 2, 0));
    // the same two inverted pieces loose outside: debris together over 1 %, so inverted
    let mut loose = synth::sphere(Point3::origin(), 20.0, 32, 16);
    for x in [-40.0, 40.0] {
        loose.append(&inside_out(synth::sphere(Point3::new(x, 0.0, 0.0), 1.55, 12, 6)));
    }
    assert_eq!(nest::negative_pieces(&loose, &Cancel::new()).unwrap().counts(), (2, 0, 0));
}

#[test]
fn the_spread_gives_each_triangle_points_by_its_area_and_each_point_an_equal_weight() {
    // three triangles of area 1, 3 and 6 and ten points: 1, 3 and 6 points of weight 1 each
    let tri = |a: f64, z: f64| [Point3::new(0.0, 0.0, z), Point3::new(2.0 * a, 0.0, z), Point3::new(0.0, 1.0, z)];
    let tris = [tri(1.0, 0.0), tri(3.0, 1.0), tri(6.0, 2.0)];
    let gone = |i: usize| tris.get(i).copied();
    let mut got: Vec<(Point3, f64)> = Vec::new();
    let mut emit = |(p, _, w): (Point3, Vec3, f64)| {
        got.push((p, w));
        Some(())
    };
    assert!(check::spread((3, &gone), 10.0, 10, &mut emit).is_some());
    assert_eq!(got.len(), 10);
    assert!(got.iter().all(|(_, w)| (w - 1.0).abs() < 1e-12));
    let per: Vec<usize> = (0..3).map(|z| got.iter().filter(|(p, _)| p.z == f64::from(z)).count()).collect();
    assert_eq!(per, vec![1, 3, 6]);
}

#[test]
fn a_removed_triangle_of_infinite_area_does_not_overflow_the_coverage() {
    let big = 1e300;
    let huge = [Point3::origin(), Point3::new(big, 0.0, 0.0), Point3::new(0.0, big, 0.0)];
    let small = [Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
    let empty = Mesh::default();
    let bvh = crate::Bvh::build(&empty);
    let r = check::Built { mesh: &empty, bvh: &bvh, origin: &[], scan_of: &[] };
    let gone = |i: usize| if i == 0 { Some(huge) } else { Some(small) };
    // more triangles than the budget allows to subdivide, fewer than the budget: the path by area
    let n = check::MAX_REMOVED_SAMPLES / 4096 + 1;
    assert!(check::coverage(&r, (n, &gone), 1e-4, &Cancel::new()).is_some());
}

#[test]
fn a_test_over_the_budget_is_counted_apart_and_forgotten_triangles_cost_nothing() {
    let scans = vec![prep::prepare(&sheet(1, 1, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let mut pos = Vec::new();
    for i in 0..20 {
        let x = 0.01 * f64::from(i);
        pos.extend([Point3::new(x, 0.0, 0.5), Point3::new(x + 0.005, 0.0, 0.5), Point3::new(x, 0.005, 0.5)]);
    }
    // a small candidate in their grid cell, below them, and a large one under everything
    pos.extend([Point3::new(0.0, 0.0, 0.2), Point3::new(0.1, 0.0, 0.2), Point3::new(0.0, 0.1, 0.2)]);
    pos.extend([Point3::new(-500.0, -500.0, -2.0), Point3::new(500.0, -500.0, -2.0), Point3::new(0.0, 500.0, -2.0)]);
    let count = pos.len();
    let u = union_of(pos, vec![0; count], Vec::new());
    let mut g = guard_over(&scans, &u, 0.01);
    let small: Vec<[u32; 3]> = (0..20u32).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect();
    for t in &small {
        assert!(g.restorable(Pts::of(&u), *t));
    }
    g.max_near = 10;
    assert!(!g.restorable(Pts::of(&u), [60, 61, 62]));
    assert_eq!((g.over_budget, g.crossing), (1, 0), "refused for the budget, not as crossing");
    for t in small.iter().skip(5) {
        g.forget(Pts::of(&u), *t);
    }
    assert!(g.restorable(Pts::of(&u), [63, 64, 65]), "the large candidate compares the 5 live ones only");
    assert_eq!(g.compared(), 5);
}

/// A disc of `k` spokes: centre 0, inner ring 1..=k, outer ring k+1..=2k (fan, then strip).
fn disc_of(k: usize) -> Mesh {
    let mut m = Mesh { positions: vec![Point3::origin()], triangles: Vec::new() };
    for r in [5.0, 10.0] {
        for i in 0..k {
            let a = std::f64::consts::TAU * i as f64 / k as f64;
            m.positions.push(Point3::new(r * a.cos(), r * a.sin(), 0.0));
        }
    }
    let (inner, outer) = (|i: usize| 1 + (i % k) as u32, |i: usize| (1 + k + i % k) as u32);
    for i in 0..k {
        m.triangles.push([0, inner(i), inner(i + 1)]);
    }
    for i in 0..k {
        m.triangles.push([inner(i), outer(i), outer(i + 1)]);
        m.triangles.push([inner(i), outer(i + 1), inner(i + 1)]);
    }
    m
}

#[test]
fn a_hole_inside_a_recovered_patch_is_filled_under_the_corner_rule() {
    let k = 6;
    let d = disc_of(k);
    let scans = vec![prep::prepare(&d, &Cancel::new()).unwrap()];
    // everything removed: the corner rule is made over an empty union
    let mut u = union::assemble(&scans, &[vec![true; d.triangle_count()]]);
    let mut guard = guard_over(&scans, &u, 0.1);
    guard.corners = Some(Corners::new(&u, &scans, &u.front(&scans)));
    // the recovery puts the strip back on the scan's own vertices, added after the rule
    let at: Vec<u32> =
        (0..d.positions.len() as u32).map(|v| union::add_vertex(&mut u, &scans, 0, v).unwrap()).collect();
    for (i, t) in d.triangles.iter().enumerate().skip(k) {
        u.triangles.push(t.map(|v| at[v as usize]));
        u.origin.push(Origin::Recovered(0, i as u32));
    }
    let (filled, added, _) = fill::fill_small(&mut u, &scans, &mut guard, &Cancel::new()).unwrap();
    assert_eq!((filled, guard.bad_corners), (1, 0), "the hole of the fan is filled, nothing refused by the rule");
    assert_eq!(added, k - 2);
}

#[test]
fn the_recovery_puts_a_scan_s_own_triangle_back_where_the_corner_rule_would_refuse_a_seam_triangle() {
    let g = sheet(6, 6, (0.0, 0.0), 1.0);
    let scans = vec![prep::prepare(&g, &Cancel::new()).unwrap()];
    // one interior triangle removed; the rule made with no cut vertex refuses any seam triangle
    let mut removed = vec![false; g.triangle_count()];
    removed[30] = true;
    let removed = vec![removed];
    let mut u = union::assemble(&scans, &removed);
    let mut guard = guard_over(&scans, &u, 0.1);
    guard.corners = Some(Corners::new(&u, &scans, &[]));
    let t = g.triangles[30].map(|v| u.vertex_of[0][v as usize]);
    assert!(!guard.corners.as_ref().unwrap().allows(Pts::of(&u), t), "the rule refuses it as a seam triangle");
    let rep = recover::recover(&mut u, &scans, &removed, &mut guard, (0.1, &Cancel::new())).unwrap();
    assert_eq!((rep.uncovered, rep.triangles, guard.bad_corners), (1, 1, 0), "{rep:?}");
    assert!(u.origin.contains(&Origin::Recovered(0, 30)));
}

#[test]
fn a_triangle_sharing_one_vertex_whose_opposite_edge_pierces_the_other_is_refused() {
    let scans = vec![prep::prepare(&sheet(1, 1, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let pos = vec![
        Point3::new(0.0, 0.0, 0.0),
        Point3::new(4.0, 0.0, 0.0),
        Point3::new(0.0, 4.0, 0.0),
        // an edge through the kept triangle at (1, 1, 0)
        Point3::new(1.0, 1.0, -1.0),
        Point3::new(1.0, 1.0, 1.0),
        // a triangle above it, touching it at the shared vertex only
        Point3::new(1.0, 1.0, 1.0),
        Point3::new(2.0, 0.0, 1.0),
    ];
    let u = union_of(pos, vec![0, 0, 0, 1, 1, 1, 1], vec![[0, 1, 2]]);
    let mut g = guard_over(&scans, &u, 0.01);
    assert!(!g.restorable(Pts::of(&u), [0, 3, 4]), "its edge opposite the shared vertex pierces the kept one");
    assert_eq!(g.crossing, 1);
    assert!(g.restorable(Pts::of(&u), [0, 5, 6]), "above it, meeting it at the shared vertex: no crossing");
}

#[test]
fn the_rows_agree_with_a_count_of_the_result_on_a_scan_with_a_duplicate_and_a_flipped_triangle() {
    let tol = 0.1;
    let mut a = sheet(30, 30, (0.0, 0.0), 1.0);
    let first = a.triangles[0];
    a.triangles.push(first);
    a.triangles[100] = [a.triangles[100][0], a.triangles[100][2], a.triangles[100][1]];
    let b = sheet(24, 24, (14.0, 3.0), 1.25);
    let (mesh, rep) =
        merge(&[a.clone(), b], &Params { tolerance: tol, ..Params::default() }, &Cancel::new(), &mut |_| {}).unwrap();
    let mesh = mesh.unwrap();
    let row = &rep.scans[0];
    assert_eq!(row.triangles, a.triangle_count());
    assert_eq!(row.dropped, 2, "the second copy and the flipped one: {row:?}");
    assert_eq!(row.triangles, row.kept + row.restored + row.collapsed + row.removed, "{row:?}");
    // independently: the result triangles that are triangles of scan 0 at exactly their places (scan
    // 0 is listed first, so none of its vertices moves)
    let key = |p: [Point3; 3]| {
        let mut k = p.map(|q| [q.x.to_bits(), q.y.to_bits(), q.z.to_bits()]);
        k.sort_unstable();
        k
    };
    let own: std::collections::HashSet<_> = (0..a.triangle_count()).filter_map(|t| a.triangle(t)).map(key).collect();
    let found = (0..mesh.triangle_count()).filter_map(|t| mesh.triangle(t)).filter(|t| own.contains(&key(*t))).count();
    assert_eq!(found, row.kept + row.restored, "{row:?}");
}

#[test]
fn thousands_of_small_inverted_pieces_cost_one_bvh_and_a_bounded_number_of_casts() {
    // a large sphere and 3000 tiny inverted spheres in the corners of its box (inside its box,
    // outside the sphere): every one is tested against the large sphere, none against the others
    let mut m = synth::sphere(Point3::origin(), 50.0, 64, 32);
    let count = 3000;
    for k in 0..count {
        let s = f64::from(k as u32);
        let c = Point3::new(45.0 - 0.01 * s, 45.0 - 0.003 * (s % 100.0), 45.0);
        m.append(&inside_out(synth::sphere(c, 0.002, 6, 3)));
    }
    let (n, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    assert_eq!(n.counts(), (0, 0, count));
    assert_eq!(n.unjudged, 0);
    // two BVHs (triangles and boxes) built once; the holding boxes found through the box BVH and
    // three short casts a piece: a few hundred units of work a piece, not one box per piece (3000 x
    // 3001), and within the stage's budget
    assert_eq!(cost.builds, 1);
    assert!(cost.work <= count * 1000 && cost.work <= cost.budget, "{cost:?}");
    assert!(nest::negative_pieces(&m, &Cancel::after_looks(2)).is_none(), "a cancellation stops the stage");
}

#[test]
fn the_subdivision_estimate_saturates_instead_of_wrapping() {
    // on a 32-bit target a million removed triangles at 4096 points each pass usize::MAX
    let near_max = usize::MAX - 10;
    assert_eq!(check::with_split(near_max, 64), usize::MAX);
    assert_eq!(check::with_split(0, usize::MAX), usize::MAX);
    assert_eq!(check::with_split(5, 3), 14);
}

#[test]
fn a_loop_left_open_for_the_budget_is_counted_once() {
    let k = 6;
    let d = disc_of(k);
    let scans = vec![prep::prepare(&d, &Cancel::new()).unwrap()];
    let removed: Vec<bool> = (0..d.triangle_count()).map(|t| t < k).collect();
    let mut u = union::assemble(&scans, &[removed]);
    let mut guard = guard_over(&scans, &u, 0.1);
    guard.max_near = 0;
    let (filled, _, left) = fill::fill_small(&mut u, &scans, &mut guard, &Cancel::new()).unwrap();
    assert_eq!((filled, left.len()), (0, 1));
    assert_eq!((guard.over_budget, guard.crossing), (1, 0), "one loop, every try over the budget");
}

#[test]
fn a_large_candidate_over_the_budget_is_refused_before_any_collection() {
    let scans = vec![prep::prepare(&sheet(1, 1, (0.0, 0.0), 1.0), &Cancel::new()).unwrap()];
    let mut pos = Vec::new();
    for i in 0..5 {
        let x = 0.01 * f64::from(i);
        pos.extend([Point3::new(x, 0.0, 0.5), Point3::new(x + 0.005, 0.0, 0.5), Point3::new(x, 0.005, 0.5)]);
    }
    pos.extend([Point3::new(-500.0, -500.0, -2.0), Point3::new(500.0, -500.0, -2.0), Point3::new(0.0, 500.0, -2.0)]);
    let count = pos.len();
    let u = union_of(pos, vec![0; count], Vec::new());
    let mut g = guard_over(&scans, &u, 0.01);
    for i in 0..5u32 {
        assert!(g.restorable(Pts::of(&u), [3 * i, 3 * i + 1, 3 * i + 2]));
    }
    g.max_near = 2;
    let before = g.collections();
    assert!(!g.restorable(Pts::of(&u), [15, 16, 17]));
    assert_eq!((g.over_budget, g.collections()), (1, before), "refused with nothing collected");
}

#[test]
fn the_uncovered_pass_polls_by_the_removed_triangles_it_looks_at() {
    let g = sheet(48, 48, (0.0, 0.0), 1.0);
    let scans = vec![prep::prepare(&g, &Cancel::new()).unwrap()];
    // removed: odd indices only (none divisible by 1024), 2304 of them
    let removed = vec![(0..g.triangle_count()).map(|t| t % 2 == 1).collect::<Vec<bool>>()];
    let u = union::assemble(&scans, &removed);
    let counter = Cancel::after_looks(usize::MAX);
    assert!(recover::uncovered(&u, &scans, &removed, 0.1, &counter).is_some());
    let looks = usize::MAX - counter.looks_left();
    assert_eq!(looks, 3, "one look per 1024 removed triangles: 0, 1024, 2048");
    assert!(recover::uncovered(&u, &scans, &removed, 0.1, &Cancel::after_looks(2)).is_none());
}
