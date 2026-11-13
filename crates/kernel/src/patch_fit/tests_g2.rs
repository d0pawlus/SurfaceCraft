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
        assert!(s.curvature_error_max.unwrap() <= G2_TARGET_REL, "{r:?} {rounds:?}");
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
        assert!(s.curvature_error_max.unwrap() <= G2_TARGET_REL, "{r:?}");
        assert!(s.angle_max_deg.unwrap() <= G1_TARGET_DEG, "{r:?}");
    }
}
