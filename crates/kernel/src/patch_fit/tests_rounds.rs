//! The rounds of `fit_patch` and the smooth densification of the sides, on the wave of the
//! diagnostic: four hard G0 sides lying on the surface, given as polylines of few points, and noisy
//! data. A chord lies off the surface by its sag; the fit must not suffer from it.
use super::tests::{Rng, polygon, regular, request};
use super::*;

/// Noise of the scan, mm.
const SIGMA: f64 = 0.0308;
/// Scan points.
const DATA: usize = 2150;
const CORNERS: [[f64; 2]; 4] = [[10.0, 8.0], [50.0, 8.0], [50.0, 32.0], [10.0, 32.0]];

fn wave(x: f64, y: f64) -> f64 {
    3.0 * (x / 12.0).sin() * (y / 9.0).cos()
}

fn on_wave(x: f64, y: f64) -> [f64; 3] {
    [x, y, wave(x, y)]
}

/// The region x in [10, 50], y in [8, 32] of the wave: four sides of `n` points each, noisy data.
fn wave_request(n: usize) -> Request {
    let mut req = request(polygon(&CORNERS, n - 1, &on_wave));
    let mut rng = Rng(2150);
    for _ in 0..8 {
        rng.next();
    }
    while req.data.len() < DATA {
        let (x, y) = (10.0 + 40.0 * rng.next(), 8.0 + 24.0 * rng.next());
        req.data.push([x, y, wave(x, y) + SIGMA * rng.normal()]);
    }
    req
}

/// Fits the wave with sides of `n` points and checks what every case must give.
fn check(n: usize) -> Patch {
    let (patch, rounds) = fit_patch_rounds(&wave_request(n)).unwrap();
    let r = &patch.report;
    assert!((0.024..=0.040).contains(&r.rms), "N = {n}: {r:?} {rounds:?}");
    assert!(r.max <= 0.2, "N = {n}: {r:?} {rounds:?}");
    assert!(r.within_tolerance, "N = {n}: {r:?} {rounds:?}");
    patch
}

#[test]
fn wave_with_dense_sides() {
    let patch = check(401);
    for s in &patch.report.sides {
        assert!(s.gap_max <= G0_TARGET, "{:?}", patch.report);
    }
}

#[test]
fn wave_with_sparse_sides() {
    check(41);
}

#[test]
fn wave_with_very_sparse_sides() {
    check(21);
}

#[test]
fn flat_sides_are_smoothed_with_their_parameters() {
    let mut req = wave_request(21);
    for s in req.sides.iter_mut() {
        s.uv = Some(s.points.iter().map(|p| [p[0], p[1]]).collect());
    }
    let patch = fit_patch(&req).unwrap();
    let r = &patch.report;
    assert!(!r.warnings.iter().any(|w| w.contains("could not be smoothed")), "{r:?}");
    assert!((0.024..=0.040).contains(&r.rms), "{r:?}");
    assert!(r.within_tolerance, "{r:?}");
}

#[test]
fn two_point_sides_stay_straight() {
    let sides = polygon(&regular(5, [0.0, 0.0], 15.0), 1, &|x, y| [x, y, 0.0]);
    assert!(sides.iter().all(|s| s.points.len() == 2));
    let patch = fit_patch(&request(sides)).unwrap();
    for s in &patch.report.sides {
        assert!(s.gap_max <= 1e-9, "{:?}", patch.report);
    }
    assert!(!patch.report.warnings.iter().any(|w| w.contains("could not be smoothed")), "{:?}", patch.report);
}

/// A side of 21 points on the wave, along y = 8.
fn sparse_side() -> Vec<[f64; 3]> {
    (0..21).map(|i| on_wave(10.0 + 2.0 * i as f64, 8.0)).collect()
}

#[test]
fn smoothing_follows_the_curve_between_the_points() {
    let given = sparse_side();
    let mut pts = vec![given.clone()];
    let mut uvs = vec![Vec::new()];
    let notes = curve::smooth_sides(&mut pts, &mut uvs, false, &[false], 47.0);
    assert!(notes.is_empty(), "{notes:?}");
    let out = &pts[0];
    assert!(out.len() > 3 * given.len(), "{} points", out.len());
    assert_eq!(out.first(), given.first());
    assert_eq!(out.last(), given.last());
    for p in &given {
        assert!(out.contains(p), "{p:?} is not kept");
    }
    let off = out.iter().map(|p| (p[2] - wave(p[0], p[1])).abs()).fold(0.0_f64, f64::max);
    assert!(off <= 5e-4, "off the wave by {off}");
    // the chords between the given points are off it by their sag, which is what the fit used to follow
    let sag = given.windows(2).map(|w| (0.5 * (w[0][2] + w[1][2]) - wave(0.5 * (w[0][0] + w[1][0]), 8.0)).abs());
    assert!(sag.fold(0.0_f64, f64::max) > 1e-3);
}

#[test]
fn smoothing_keeps_corners_and_soft_sides() {
    let line = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [2.0, 1.0, 0.0], [2.0, 2.0, 0.0]];
    let mut pts = vec![line.to_vec(), sparse_side()];
    let before = pts.clone();
    let mut uvs = vec![Vec::new(), Vec::new()];
    let notes = curve::smooth_sides(&mut pts, &mut uvs, false, &[false, true], 3.0);
    assert!(notes.is_empty(), "{notes:?}");
    // straight stretches around a corner come back as they were, a soft side is not touched
    assert_eq!(pts, before);
}

#[test]
fn smoothing_gives_up_on_a_side_it_cannot_interpolate() {
    // not samples of a curve: the side keeps its points and the warning names it
    let mut broken = sparse_side();
    broken[7][2] = f64::NAN;
    let mut pts = vec![sparse_side(), broken];
    let mut uvs = vec![Vec::new(), Vec::new()];
    let notes = curve::smooth_sides(&mut pts, &mut uvs, false, &[false, false], 47.0);
    assert_eq!(notes, vec!["side 1: could not be smoothed, its points are used as a polyline".to_string()]);
    assert!(pts[0].len() > 21);
    assert_eq!(pts[1].len(), 21);
    assert!(pts[1][7][2].is_nan());
}

fn summary(data_ok: bool, data_max: f64, side_miss: f64) -> Summary {
    Summary { data_ok, data_max, side_miss }
}

#[test]
fn kept_round_prefers_data_within_tolerance() {
    // a round within the data tolerance beats one outside it, whatever the sides
    let outside_met = summary(false, 0.30, 1.0);
    let inside_missed = summary(true, 0.15, 2.3);
    assert!(prefer(&inside_missed, &outside_met));
    assert!(!prefer(&outside_met, &inside_missed));
    // inside the tolerance the smaller side miss wins ...
    let kept = summary(true, 0.100, 2.3);
    assert!(prefer(&summary(true, 0.100, 2.0), &kept));
    assert!(!prefer(&summary(true, 0.100, 2.5), &kept));
    // ... unless its data deviation is more than 10 % larger
    assert!(prefer(&summary(true, 0.109, 2.0), &kept));
    assert!(!prefer(&summary(true, 0.120, 2.0), &kept));
    // ... except when it brings the sides from missed to met
    assert!(prefer(&summary(true, 0.120, 1.0), &kept));
    // rounds that meet the sides tie, and the smaller data deviation wins
    let met = summary(true, 0.100, 0.4);
    assert!(prefer(&summary(true, 0.090, 0.9), &met));
    assert!(!prefer(&summary(true, 0.110, 0.2), &met));
    // outside the tolerance the same rules hold
    let kept = summary(false, 0.40, 3.0);
    assert!(!prefer(&summary(false, 0.50, 2.0), &kept));
    assert!(prefer(&summary(false, 0.50, 1.0), &kept));
    assert!(prefer(&summary(false, 0.38, 2.0), &kept));
    // no data: nothing to compare but the sides
    assert!(prefer(&summary(true, 0.0, 1.5), &summary(true, 0.0, 2.0)));
    assert!(!prefer(&summary(true, 0.0, 1.0), &summary(true, 0.0, 1.0)));
}

#[test]
#[ignore = "prints the rounds of the wave cases"]
fn print_rounds_of_the_wave_cases() {
    for n in [401, 41, 21] {
        let (patch, rounds) = fit_patch_rounds(&wave_request(n)).unwrap();
        for (i, r) in rounds.iter().enumerate() {
            eprintln!(
                "ROUND N={n} round={} net={:?} rms={:.4} max={:.4} gap_max={:.5}",
                i + 1,
                r.net,
                r.rms,
                r.max,
                r.gap_max
            );
        }
        let r = &patch.report;
        eprintln!(
            "KEPT N={n} net={:?} rounds={} rms={:.4} p99={:.4} max={:.4} within_targets={} warnings={:?}",
            r.control, r.rounds, r.rms, r.p99, r.max, r.within_targets, r.warnings
        );
    }
}
