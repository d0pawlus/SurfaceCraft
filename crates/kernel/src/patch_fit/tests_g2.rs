//! G2 on a net that cannot grow, and G2 with dense scan data, on the five-sided hole in the
//! unrolled cylinder.
use super::tests::{Rng, cylinder_hole, on_cylinder, regular};
use super::*;

/// Is `p` inside the convex counter-clockwise polygon?
fn inside(poly: &[[f64; 2]], p: [f64; 2]) -> bool {
    (0..poly.len()).all(|k| {
        let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
        (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= 0.0
    })
}

#[test]
fn g2_holds_when_the_net_cannot_grow() {
    // the start net is the largest one: the curvature rows, linearised about the previous round, only
    // exist from the second round on, which must run at the same net
    let c = 9;
    let mut req = cylinder_hole(Continuity::G2);
    req.control = Some([c, c]);
    req.max_control = c;
    let (patch, rounds) = fit_patch_rounds(&req).unwrap();
    let r = &patch.report;
    assert_eq!(r.control, [c, c], "{r:?}");
    assert!(r.rounds >= 2, "{r:?} {rounds:?}");
    for s in &r.sides {
        assert!(s.curvature_error_max.unwrap() <= 1.5 * G2_TARGET_REL, "{r:?} {rounds:?}");
        assert!(s.angle_max_deg.unwrap() <= G1_TARGET_DEG, "{r:?} {rounds:?}");
    }
}

#[test]
fn g2_with_dense_data_on_the_cylinder() {
    let mut req = cylinder_hole(Continuity::G2);
    let hole = regular(5, [0.0, 0.0], 6.0);
    let mut rng = Rng(5);
    for _ in 0..8 {
        rng.next();
    }
    // 1500 points of the cylinder inside the hole, noise 0.01 mm
    while req.data.len() < 1500 {
        let (s, z) = (12.0 * rng.next() - 6.0, 12.0 * rng.next() - 6.0);
        if inside(&hole, [s, z]) {
            let p = on_cylinder(s, z);
            req.data.push([p[0] + 0.01 * rng.normal(), p[1] + 0.01 * rng.normal(), p[2] + 0.01 * rng.normal()]);
        }
    }
    let patch = fit_patch(&req).unwrap();
    let r = &patch.report;
    assert!(r.within_tolerance, "{r:?}");
    for s in &r.sides {
        assert!(s.curvature_error_max.unwrap() <= 1.5 * G2_TARGET_REL, "{r:?}");
        assert!(s.angle_max_deg.unwrap() <= G1_TARGET_DEG, "{r:?}");
    }
}

#[test]
fn one_g2_side_between_noisy_soft_sides_meets_every_target() {
    // the case of a patch joined to a neighbour on a real scan: one side continues the reference
    // with G2, the other four are traced on the noisy scan and soft, the data are noisy
    let mut req = cylinder_hole(Continuity::G2);
    let mut rng = Rng(17);
    for _ in 0..8 {
        rng.next();
    }
    for side in req.sides.iter_mut().skip(1) {
        side.continuity = Continuity::G0;
        side.reference = None;
        side.soft = true;
        let last = side.points.len() - 1;
        for p in &mut side.points[1..last] {
            for c in p.iter_mut() {
                *c += 0.01 * rng.normal();
            }
        }
    }
    let hole = regular(5, [0.0, 0.0], 6.0);
    while req.data.len() < 1500 {
        let (s, z) = (12.0 * rng.next() - 6.0, 12.0 * rng.next() - 6.0);
        if inside(&hole, [s, z]) {
            let p = on_cylinder(s, z);
            req.data.push([p[0] + 0.01 * rng.normal(), p[1] + 0.01 * rng.normal(), p[2] + 0.01 * rng.normal()]);
        }
    }
    let (patch, rounds) = fit_patch_rounds(&req).unwrap();
    let r = &patch.report;
    let side = &r.sides[0];
    eprintln!(
        "MEASURED one G2 side: net {:?} rounds {} gap {:.5} angle {:.4} curvature {:.4} corners {}",
        r.control,
        r.rounds,
        side.gap_max,
        side.angle_max_deg.unwrap(),
        side.curvature_error_max.unwrap(),
        r.corners.len()
    );
    // the noisy ends of the soft sides are no conflict with the reference: nothing is relaxed
    assert!(r.corners.is_empty(), "{r:?}");
    assert!(side.gap_max <= G0_TARGET, "{r:?} {rounds:?}");
    assert!(side.angle_max_deg.unwrap() <= G1_TARGET_DEG, "{r:?} {rounds:?}");
    assert!(side.curvature_error_max.unwrap() <= G2_TARGET_REL, "{r:?} {rounds:?}");
    assert!(r.within_targets, "{r:?}");
}

#[test]
fn a_round_that_halves_the_side_miss_within_the_tolerance_is_kept() {
    // a curvature round that meets the sides far better is kept although its data deviation tripled
    let kept = Summary { data_ok: true, data_max: 0.05, side_miss: 100.0 };
    let new = Summary { data_ok: true, data_max: 0.15, side_miss: 8.0 };
    assert!(prefer(&new, &kept));
    assert!(!prefer(&kept, &new));
    // outside the tolerance a round whose data deviation grew that much is still refused
    let kept = Summary { data_ok: false, data_max: 0.3, side_miss: 100.0 };
    let new = Summary { data_ok: false, data_max: 0.9, side_miss: 8.0 };
    assert!(!prefer(&new, &kept));
}
