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
fn a_cap_turned_over_past_its_rim_plane_is_folded() {
    // down to a polar angle of 110° the cap overhangs its rim: its normal turns past the plane of the outline
    let patch = fit_patch(&cap(110.0_f64.to_radians())).unwrap();
    let r = &patch.report;
    assert!(r.folded, "{r:?}");
    assert!(!r.within_targets, "{r:?}");
    assert!(r.warnings.iter().any(|w| w.contains("folds over inside the outline")), "{:?}", r.warnings);
}

#[test]
fn a_cap_that_stays_a_graph_over_its_rim_plane_is_not_folded() {
    let patch = fit_patch(&cap(70.0_f64.to_radians())).unwrap();
    let r = &patch.report;
    assert!(!r.folded, "{r:?}");
    assert!(r.warnings.iter().all(|w| !w.contains("folds")), "{:?}", r.warnings);
    assert!(r.sides.iter().all(|s| s.met), "{r:?}");
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
    let id_of = |w: &String| w.strip_prefix("side ").and_then(|r| r.split(':').next()).and_then(|n| n.parse::<usize>().ok());
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


