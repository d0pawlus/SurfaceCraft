//! What the report says: a patch that turns over inside the outline is folded, contradictory hard
//! conditions are reported, a side that misses its target says so, and messages name the caller's
//! side.
use super::tests::{Rng, cylinder_hole, g0, polygon, regular, request};
use super::*;

const R: f64 = 20.0;
const PI: f64 = std::f64::consts::PI;

/// A cap of the sphere of radius 20 down to the polar angle `alpha`, with a flattening
/// (azimuthal equidistant: `R·θ` along the azimuth) given for the sides and the data.
fn cap(alpha: f64) -> Request {
    let at = |theta: f64, phi: f64| [R * theta.sin() * phi.cos(), R * theta.sin() * phi.sin(), R * theta.cos()];
    let flat = |theta: f64, phi: f64| [R * theta * phi.cos(), R * theta * phi.sin()];
    let sides = (0..5)
        .map(|k| {
            let phi = |i: usize| 2.0 * PI * (k as f64 + i as f64 / 40.0) / 5.0;
            let points = (0..=40).map(|i| at(alpha, phi(i))).collect();
            let uv = Some((0..=40).map(|i| flat(alpha, phi(i))).collect());
            Side { uv, ..g0(points) }
        })
        .collect();
    let mut req = request(sides);
    let (mut rng, mut data, mut data_uv) = (Rng(5), Vec::new(), Vec::new());
    while data.len() < 1500 {
        let theta = (1.0 - rng.next() * (1.0 - alpha.cos())).acos();
        let phi = 2.0 * PI * rng.next();
        let p = at(theta, phi);
        data.push([p[0] + 0.02 * rng.normal(), p[1] + 0.02 * rng.normal(), p[2] + 0.02 * rng.normal()]);
        data_uv.push(flat(theta, phi));
    }
    req.data = data;
    req.data_uv = Some(data_uv);
    req
}

#[test]
fn a_dome_past_a_hemisphere_is_not_folded() {
    // down to a polar angle of 110° the cap overhangs its rim: it is no graph over the plane of the
    // outline, and it is a correct surface
    let patch = fit_patch(&cap(110.0_f64.to_radians())).unwrap();
    let r = &patch.report;
    eprintln!(
        "MEASURED dome 110°: gaps {:?} max {:.4} within_targets {}",
        r.sides.iter().map(|s| s.gap_max).collect::<Vec<_>>(),
        r.max,
        r.within_targets
    );
    assert!(!r.folded, "{r:?}");
    assert!(r.warnings.iter().any(|w| w.contains("not a graph over the plane")), "{:?}", r.warnings);
    assert!(r.warnings.iter().all(|w| !w.contains("folds inside")), "{:?}", r.warnings);
    assert!(r.within_targets, "{r:?}");
}

/// A band of `2·half` radians of a cylinder of radius 20 and height 10 (more than half a turn for
/// `half` > 90°), flattened by unrolling (`R·θ`, `z`).
fn wrap(half: f64) -> Request {
    let at = |theta: f64, z: f64| [R * theta.cos(), R * theta.sin(), z];
    let flat = |theta: f64, z: f64| [R * theta, z];
    let line = |a: [f64; 2], b: [f64; 2], n: usize| -> Side {
        let t = |i: usize| i as f64 / n as f64;
        let p = |i: usize| [a[0] + t(i) * (b[0] - a[0]), a[1] + t(i) * (b[1] - a[1])];
        Side {
            uv: Some((0..=n).map(|i| flat(p(i)[0], p(i)[1])).collect()),
            ..g0((0..=n).map(|i| at(p(i)[0], p(i)[1])).collect())
        }
    };
    let mut req = request(vec![
        line([-half, -5.0], [half, -5.0], 60),
        line([half, -5.0], [half, 5.0], 12),
        line([half, 5.0], [-half, 5.0], 60),
        line([-half, 5.0], [-half, -5.0], 12),
    ]);
    let (mut rng, mut data, mut data_uv) = (Rng(9), Vec::new(), Vec::new());
    while data.len() < 2000 {
        let (theta, z) = (half * (2.0 * rng.next() - 1.0), 5.0 * (2.0 * rng.next() - 1.0));
        let p = at(theta, z);
        data.push([p[0] + 0.02 * rng.normal(), p[1] + 0.02 * rng.normal(), p[2] + 0.02 * rng.normal()]);
        data_uv.push(flat(theta, z));
    }
    req.data = data;
    req.data_uv = Some(data_uv);
    req
}

#[test]
fn more_than_half_a_turn_of_a_cylinder_is_not_folded() {
    let r = fit_patch(&wrap(100.0_f64.to_radians())).unwrap().report;
    eprintln!(
        "MEASURED wrap 200°: gaps {:?} max {:.4} within_targets {}",
        r.sides.iter().map(|s| s.gap_max).collect::<Vec<_>>(),
        r.max,
        r.within_targets
    );
    assert!(!r.folded, "{r:?}");
    assert!(r.within_targets, "{r:?}");
}

fn pentagon_with_points(points: Vec<[f64; 3]>) -> Request {
    let mut req = request(polygon(&regular(5, [0.0, 0.0], 15.0), 30, &|x, y| [x, y, 0.0]));
    req.hard_points = points;
    req
}

#[test]
fn contradictory_hard_points_are_reported_and_repeated_ones_are_not() {
    // one place above and below the flat patch: both project onto the same parameters in every round
    let bad = fit_patch(&pentagon_with_points(vec![[1.0, 1.0, 0.4], [1.0, 1.0, -0.4]])).unwrap().report;
    assert!(bad.hard_residual >= 0.05, "{}", bad.hard_residual);
    assert!(!bad.within_targets, "{bad:?}");
    assert!(bad.warnings.iter().any(|w| w.contains("contradict")), "{:?}", bad.warnings);
    // the same point twice is redundant, not contradictory
    let same = fit_patch(&pentagon_with_points(vec![[1.0, 1.0, 0.0], [1.0, 1.0, 0.0]])).unwrap().report;
    assert!(same.hard_residual <= 1e-8, "{}", same.hard_residual);
    assert!(same.within_targets, "{same:?}");
    assert!(same.warnings.iter().all(|w| !w.contains("contradict")), "{:?}", same.warnings);
}

#[test]
fn a_side_that_misses_its_target_says_so_and_one_that_meets_it_does_not() {
    let met = fit_patch(&cylinder_hole(Continuity::G1)).unwrap().report;
    assert!(met.sides.iter().all(|s| s.met), "{met:?}");
    assert!(met.warnings.iter().all(|w| !w.contains("not met")), "{:?}", met.warnings);
    // a net of 4 × 4 cannot follow a G1 side of a cylinder to 0.05°
    let mut req = cylinder_hole(Continuity::G1);
    req.control = Some([4, 4]);
    req.max_control = 4;
    let missed = fit_patch(&req).unwrap().report;
    assert!(missed.sides.iter().any(|s| !s.met), "{missed:?}");
    assert!(!missed.within_targets);
    let named = missed.warnings.iter().filter(|w| w.contains("G1 not met")).count();
    assert_eq!(named, missed.sides.iter().filter(|s| !s.met).count(), "{:?}", missed.warnings);
    assert!(missed.warnings.iter().any(|w| w.contains("gap") && w.contains("target")), "{:?}", missed.warnings);
}

#[test]
fn messages_name_the_callers_side() {
    let mut req = cylinder_hole(Continuity::G1);
    req.side_ids = vec![7, 8, 9, 10, 11];
    req.control = Some([4, 4]);
    req.max_control = 4;
    let report = fit_patch(&req).unwrap().report;
    let id_of =
        |w: &String| w.strip_prefix("side ").and_then(|r| r.split(':').next()).and_then(|n| n.parse::<usize>().ok());
    let ids: Vec<usize> = report.warnings.iter().filter_map(id_of).collect();
    assert!(!ids.is_empty() && ids.iter().all(|i| (7..=11).contains(i)), "{:?}", report.warnings);
    // a side 3 mm off its reference face: the error names the side and hints at adjacency
    let mut off = cylinder_hole(Continuity::G1);
    off.side_ids = vec![7, 8, 9, 10, 11];
    let last = off.sides[2].points.len() - 1;
    for p in &mut off.sides[2].points[1..last] {
        p[0] *= 1.15;
        p[1] *= 1.15;
    }
    let err = fit_patch(&off).unwrap_err().to_string();
    assert!(err.contains("side 9") && err.contains("reference face"), "{err}");
}

/// The measured misses of a fit in units of the product targets (smaller is better; 1 is the target).
fn ratios(r: &Report) -> [f64; 3] {
    let m = |f: &dyn Fn(&SideReport) -> Option<f64>, t: f64| r.sides.iter().filter_map(f).fold(0.0, f64::max) / t;
    [
        m(&|s| Some(s.gap_max), G0_TARGET),
        m(&|s| s.angle_max_deg, G1_TARGET_DEG),
        m(&|s| s.curvature_error_max, G2_TARGET_REL),
    ]
}

/// The product targets are gated on a hole of radius 4 mm in the cylinder, where the method reaches
/// them in the first round with at least 2x to 4x to spare (so one more or one less round changes
/// nothing): gap 0.46, angle 0.11, curvature 0.24 of their targets measured.
#[test]
fn every_side_meets_its_target_on_a_small_curved_hole() {
    use super::tests::cylinder_hole_of;
    for (c, max_ratio) in [(Continuity::G0, 0.67), (Continuity::G1, 0.67), (Continuity::G2, 0.67)] {
        let r = fit_patch(&cylinder_hole_of(4.0, c)).unwrap().report;
        let q = ratios(&r);
        eprintln!(
            "MEASURED {c:?} r 4: gap {:.2}, angle {:.2}, curvature {:.2} of their targets, net {:?}",
            q[0], q[1], q[2], r.control
        );
        assert!(r.sides.iter().all(|s| s.met), "{r:?}");
        assert!(r.within_targets, "{r:?}");
        assert!(q.iter().all(|x| *x <= max_ratio), "margin to the targets is under 1.5x: {q:?}");
    }
}

/// The pentagon of `contradictory_hard_points...` scaled by `k` (the points with it).
fn scaled_pentagon_with(k: f64, z: &[f64]) -> Request {
    let corners = regular(5, [0.0, 0.0], 15.0 * k);
    let mut req = request(polygon(&corners, 30, &|x, y| [x, y, 0.0]));
    req.hard_points = z.iter().map(|z| [k, k, z * k]).collect();
    req
}

#[test]
fn the_contradiction_verdict_does_not_depend_on_the_scale_of_the_part() {
    for k in [0.001, 1.0, 1000.0] {
        let bad = fit_patch(&scaled_pentagon_with(k, &[0.4, -0.4])).unwrap().report;
        assert!(bad.warnings.iter().any(|w| w.contains("contradict")), "k {k}: {:?}", bad.warnings);
        assert!(bad.hard_residual >= 0.05 * k, "k {k}: {}", bad.hard_residual);
        let ok = fit_patch(&scaled_pentagon_with(k, &[0.0, 0.0])).unwrap().report;
        assert!(ok.warnings.iter().all(|w| !w.contains("contradict")), "k {k}: {:?}", ok.warnings);
    }
}
