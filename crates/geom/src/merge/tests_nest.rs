//! Tests of the nesting stage's budget: piles and nested shells stay within work proportional to
//! the input, unjudged pieces become inner skins, and a cancellation is seen within bounded work.
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

#[test]
fn a_pile_of_coincident_inverted_pieces_stays_within_the_budget() {
    // 5000 inverted tetrahedra at one place, each with its own vertices: every probe lies in every
    // box; without the budget this is 5000 x 5000 box listings and casts through the whole pile
    let mut m = synth::sphere(Point3::new(100.0, 0.0, 0.0), 10.0, 32, 16);
    let count = 5000;
    for _ in 0..count {
        m.append(&tetra_inverted(Point3::origin(), 1.0));
    }
    let (n, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    assert!(cost.work <= cost.budget + (1 << 14), "{cost:?}");
    assert!(cost.budget <= nest::WORK_BASE + nest::WORK_PER_TRIANGLE * m.triangle_count());
    // the pieces the budget could not judge are inner skins, counted and reported (the few judged
    // before it ran out are whatever their rays say)
    assert!(n.unjudged > 0 && n.skins >= n.unjudged, "{n:?}");
    let issues = scan_issues(&[prep::prepare(&m, &Cancel::new()).unwrap()]);
    assert!(issues.iter().any(|i| matches!(i, Issue::InnerSkins { unjudged, .. } if *unjudged > 0)), "{issues:?}");
}

#[test]
fn many_concentric_inverted_shells_stay_within_the_budget() {
    // 1500 concentric inverted shells of 8 triangles: a ray from the innermost crosses all of them
    let mut m = Mesh::default();
    for k in 0..1500 {
        m.append(&inverted_shell(1.0 + 0.01 * f64::from(k)));
    }
    let (_, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    assert!(cost.work <= cost.budget + (1 << 14), "{cost:?}");
    assert!(cost.budget <= nest::WORK_BASE + nest::WORK_PER_TRIANGLE * m.triangle_count());
}

#[test]
fn a_cancellation_is_seen_within_a_bounded_amount_of_work() {
    let mut m = synth::sphere(Point3::new(100.0, 0.0, 0.0), 10.0, 32, 16);
    for _ in 0..3000 {
        m.append(&tetra_inverted(Point3::origin(), 1.0));
    }
    let (_, cost) = nest::negative_pieces_costed(&m, &Cancel::new()).unwrap();
    // a look at least every CANCEL_WORK units of work plus one cast
    assert!(cost.looks * (nest::CANCEL_WORK + (1 << 14)) >= cost.work, "{cost:?}");
    assert!(nest::negative_pieces(&m, &Cancel::after_looks(3)).is_none());
}
