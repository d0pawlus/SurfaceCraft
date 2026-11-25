//! Tests of the nesting stage's budget: piles and nested shells stay within work proportional to
//! the input, the pieces that decide the verdict are judged before a pile spends the budget,
//! unjudged pieces refuse the scan when they are a material share of it, and a cancellation is seen
//! within bounded work.
use super::*;
use crate::{Point3, Vec3, synth};

/// An inverted tetrahedron at `o` with its own four vertices.
fn tetra_inverted(o: Point3, s: f64) -> Mesh {
    let p = [o, o + Vec3::new(s, 0.0, 0.0), o + Vec3::new(0.0, s, 0.0), o + Vec3::new(0.0, 0.0, s)];
    // outward: [0,2,1], [0,1,3], [0,3,2], [1,2,3]; inverted: each reversed
    Mesh { positions: p.to_vec(), triangles: vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]] }
}

/// An inverted octahedron of radius `r` about the origin (8 triangles).
fn inverted_shell(r: f64) -> Mesh {
    let positions = vec![
        Point3::new(r, 0.0, 0.0),
        Point3::new(-r, 0.0, 0.0),
        Point3::new(0.0, r, 0.0),
        Point3::new(0.0, -r, 0.0),
        Point3::new(0.0, 0.0, r),
        Point3::new(0.0, 0.0, -r),
    ];
    // outward faces [x, y, z] in each octant, reversed
    let out = [[0, 2, 4], [2, 1, 4], [1, 3, 4], [3, 0, 4], [2, 0, 5], [1, 2, 5], [3, 1, 5], [0, 3, 5]];
    Mesh { positions, triangles: out.iter().map(|t: &[u32; 3]| [t[0], t[2], t[1]]).collect() }
}

fn inverted_sphere(c: Point3, r: f64) -> Mesh {
    let m = synth::sphere(c, r, 8, 4);
    Mesh { positions: m.positions.clone(), triangles: m.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect() }
}

/// 5000 coincident inverted spheres at the origin, each with its own vertices and as many triangles
/// as [`inverted_sphere`] gives any sphere.
fn pile() -> Mesh {
    let mut m = Mesh::default();
    for _ in 0..5000 {
        m.append(&inverted_sphere(Point3::origin(), 1.0));
    }
    m
}

fn within_budget(cost: &nest::Cost, m: &Mesh) {
    // at most one capped cast or listing past the budget
    assert!(cost.work <= cost.budget + (1 << 14), "{cost:?}");
    assert!(cost.budget <= nest::WORK_BASE + nest::WORK_PER_TRIANGLE * m.triangle_count());
}

#[test]
fn an_inverted_sphere_after_a_pile_is_still_refused() {
    // the pile first in the file, a large inverted sphere (judged first: it is the largest piece,
    // over 40 % of the area) last, away from the pile
    let mut m = pile();
    m.append(&inverted_sphere(Point3::new(200.0, 0.0, 0.0), 60.0));
    let (n, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    within_budget(&cost, &m);
    assert!(n.inverted >= 1, "{n:?}");
    // the pile itself is unjudged and far more than 1 % of the area: refused too
    assert!(n.unjudged > 0 && n.unjudged_refuses, "{n:?}");
    let issues = scan_issues(&[prep::prepare(&m, &Cancel::new()).unwrap()]);
    assert!(issues.iter().any(|i| matches!(i, Issue::Inverted { .. })), "{issues:?}");
    assert!(issues.iter().any(|i| matches!(i, Issue::InvertedUnjudged { .. })), "{issues:?}");
}

#[test]
fn an_inverted_sphere_beside_a_pile_in_another_scan_is_refused() {
    let lone = inverted_sphere(Point3::new(50.0, 0.0, 0.0), 2.0);
    let heap = pile();
    let scans = [prep::prepare(&lone, &Cancel::new()).unwrap(), prep::prepare(&heap, &Cancel::new()).unwrap()];
    let issues = scan_issues(&scans);
    assert!(issues.iter().any(|i| matches!(i, Issue::Inverted { scan: 0, .. })), "{issues:?}");
    assert!(issues.iter().any(|i| matches!(i, Issue::InvertedUnjudged { scan: 1, .. })), "{issues:?}");
    // not lifted by alignment "warn"
    assert!(issues.iter().any(|i| matches!(i, Issue::InvertedUnjudged { .. }) && i.refuses_under(Alignment::Warn)));
    let (_, cost) = nest::negative_pieces_costed(&heap, &Cancel::new()).unwrap();
    within_budget(&cost, &heap);
}

#[test]
fn a_hollow_part_beside_a_pile_has_its_skin_judged_and_the_pile_decides() {
    // a hollow sphere (outer shell, inner skin facing in) at x = 50 and the pile at the origin: the
    // inner skin is larger than any pile piece, so it is judged first and cheaply
    let mut m = synth::sphere(Point3::new(50.0, 0.0, 0.0), 10.0, 32, 16);
    let inner = synth::sphere(Point3::new(50.0, 0.0, 0.0), 8.0, 24, 12);
    m.append(&Mesh {
        positions: inner.positions.clone(),
        triangles: inner.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect(),
    });
    m.append(&pile());
    let (n, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    within_budget(&cost, &m);
    // the skin is judged an inner skin (first and cheaply); the pile refuses the scan
    assert!(n.skins >= 1 && n.unjudged_refuses, "{n:?}");
}

#[test]
fn a_pile_of_coincident_tetrahedra_stays_within_the_budget_and_refuses() {
    // 5000 inverted tetrahedra at one place beside a sphere: every probe lies in every box
    let mut m = synth::sphere(Point3::new(100.0, 0.0, 0.0), 10.0, 32, 16);
    for _ in 0..5000 {
        m.append(&tetra_inverted(Point3::origin(), 1.0));
    }
    let (n, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    within_budget(&cost, &m);
    // the pile is most of the area: refused, as turned inside out (pieces judged alone, debris
    // together over 1 %) or as unjudged; never accepted
    assert!(n.inverted > 0 || n.unjudged_refuses, "{n:?}");
}

#[test]
fn many_concentric_inverted_shells_stay_within_the_budget_and_refuse() {
    // 1500 concentric inverted shells of 8 triangles: every other one lies inside an even number of
    // the others and reads as turned inside out (small, but together far over 1 % of the area);
    // those the budget does not reach are unjudged and refuse too
    let mut m = Mesh::default();
    for k in 0..1500 {
        m.append(&inverted_shell(1.0 + 0.01 * f64::from(k)));
    }
    let (n, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    within_budget(&cost, &m);
    assert!(n.inverted >= 1 || n.unjudged_refuses, "{n:?}");
}

#[test]
fn a_cancellation_is_seen_within_a_bounded_amount_of_work() {
    // 20 000 coincident tetrahedra: one listing of their boxes costs about 1.26 units a box (measured),
    // about 25 000, more than a look's worth plus one cast, were it not capped at 16 384
    let mut m = synth::sphere(Point3::new(100.0, 0.0, 0.0), 10.0, 32, 16);
    for _ in 0..20_000 {
        m.append(&tetra_inverted(Point3::origin(), 1.0));
    }
    let (_, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    // a look at least every CANCEL_WORK units of work plus one capped cast or listing (which may
    // overshoot its cap by a node or a leaf, a few units)
    assert!(cost.looks * (nest::CANCEL_WORK + (1 << 14) + 64) >= cost.work, "{cost:?}");
    assert!(nest::negative_pieces(&m, &Cancel::after_looks(3)).is_none());
}

/// A cube of side `side` with `n` small spherical voids (inner skins, facing in) on a grid inside;
/// with `mixed`, every tenth void is five times larger.
fn cube_with_voids(n: usize, mixed: bool) -> Mesh {
    let per = (n as f64).cbrt().ceil() as usize;
    let gap = 4.0;
    let side = gap * (per as f64 + 1.0) + 40.0;
    let mut m = synth::cuboid(Point3::new(-20.0, -20.0, -20.0), Vec3::new(side, side, side), 8);
    let mut k = 0;
    'grid: for i in 0..per {
        for j in 0..per {
            for l in 0..per {
                if k == n {
                    break 'grid;
                }
                let c = Point3::new(gap * (i as f64 + 1.0), gap * (j as f64 + 1.0), gap * (l as f64 + 1.0));
                let r = if mixed && k % 10 == 0 { 1.5 } else { 0.3 };
                m.append(&inverted_sphere(c, r));
                k += 1;
            }
        }
    }
    m
}

/// The measurement of review round 8 (run by hand): the nesting cost of a correct part with many
/// small voids against the budget.
#[test]
#[ignore = "measurement, run by hand"]
fn measure_the_nesting_cost_of_many_voids() {
    for (n, mixed) in [(1000, false), (10_000, false), (50_000, false), (10_000, true)] {
        let m = cube_with_voids(n, mixed);
        let t0 = std::time::Instant::now();
        let (neg, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
        eprintln!(
            "MEASURE voids {n} mixed {mixed}: triangles {} work {} budget {} ratio {:.2} skins {} unjudged {} inverted {} refuses {} in {:.1} s",
            m.triangle_count(),
            cost.work,
            cost.budget,
            cost.budget as f64 / cost.work.max(1) as f64,
            neg.skins,
            neg.unjudged,
            neg.inverted,
            neg.unjudged_refuses,
            t0.elapsed().as_secs_f64()
        );
    }
}

#[test]
fn a_part_with_thousands_of_voids_is_accepted_with_every_void_judged() {
    // 3000 small spherical voids on a grid inside a cube: each is cast against the cube only (the
    // one piece whose box holds it), so the work per void does not grow with the others
    let m = cube_with_voids(3000, false);
    let (n, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    assert_eq!((n.skins, n.unjudged, n.inverted, n.unjudged_refuses), (3000, 0, 0, false), "{n:?}");
    // measured about 20 times below the budget; bound: 1.5 times
    assert!(cost.work as f64 * 1.5 <= cost.budget as f64, "{cost:?}");
    assert_eq!(cost.builds, 1, "the cube's BVH, once");
}

#[test]
fn scans_far_from_the_origin_are_refused_and_a_sphere_at_a_million_merges() {
    let near = synth::sphere(Point3::origin(), 10.0, 32, 16);
    let go =
        |scans: &[Mesh]| merge(scans, &Params { tolerance: 0.1, ..Params::default() }, &Cancel::new(), &mut |_| {});
    // an inverted sphere at 1e120: its points collapse in floating point and its volume would read
    // as flat; refused at the boundary, not accepted
    let far = inverted_sphere(Point3::new(1e120, 0.0, 0.0), 10.0);
    assert!(matches!(go(&[near.clone(), far]), Err(MergeError::Params(_))));
    let million = synth::sphere(Point3::new(1e6, 0.0, 0.0), 10.0, 32, 16);
    let (mesh, rep) = go(&[near.clone(), million]).unwrap();
    assert!(mesh.is_some() && !rep.refused, "{:?}", rep.issues);
}

#[test]
fn more_scans_than_the_cap_are_refused() {
    let one = synth::sphere(Point3::origin(), 1.0, 8, 4);
    let many = vec![one; MAX_SCANS + 1];
    let r = merge(&many, &Params { tolerance: 0.1, ..Params::default() }, &Cancel::new(), &mut |_| {});
    assert!(matches!(r, Err(MergeError::Params(_))), "{r:?}");
}
