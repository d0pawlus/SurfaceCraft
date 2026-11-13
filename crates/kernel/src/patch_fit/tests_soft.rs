//! Soft sides, data outside the outline and the snapping of referenced sides onto their faces.
use super::tests::{Rng, cylinder_face, cylinder_hole, on_cylinder, regular, request};
use super::*;

/// Noise of the scan, mm.
const SIGMA: f64 = 0.03;
/// Points of a side polyline.
const SIDE_POINTS: usize = 60;
const RADIUS: f64 = 20.0;

fn rng(seed: u64) -> Rng {
    let mut r = Rng(seed);
    for _ in 0..8 {
        r.next();
    }
    r
}

fn pentagon() -> Vec<[f64; 2]> {
    regular(5, [0.0, 0.0], RADIUS)
}

/// Distance of `p` inside the convex counter-clockwise polygon (negative outside).
fn inner_distance(p: [f64; 2], poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    (0..n)
        .map(|k| {
            let (a, b) = (poly[k], poly[(k + 1) % n]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            (dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / dx.hypot(dy)
        })
        .fold(f64::INFINITY, f64::min)
}

/// Distance of `p` to the polygon's boundary.
fn boundary_distance(p: [f64; 2], poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    (0..n)
        .map(|k| {
            let (a, b) = (poly[k], poly[(k + 1) % n]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy)).clamp(0.0, 1.0);
            (p[0] - a[0] - t * dx).hypot(p[1] - a[1] - t * dy)
        })
        .fold(f64::INFINITY, f64::min)
}

/// The five straight sides of the pentagon in the plane z = 0 with the heights `height(is_corner)`.
fn pentagon_sides(soft: bool, mut height: impl FnMut(bool) -> f64) -> Vec<Side> {
    let poly = pentagon();
    (0..poly.len())
        .map(|k| {
            let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
            let points = (0..SIDE_POINTS)
                .map(|i| {
                    let t = i as f64 / (SIDE_POINTS - 1) as f64;
                    let end = i == 0 || i + 1 == SIDE_POINTS;
                    [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), height(end)]
                })
                .collect();
            Side { points, continuity: Continuity::G0, reference: None, uv: None, soft }
        })
        .collect()
}

/// `n` noisy points inside the pentagon, at least `margin` from its boundary, at the height
/// `dip(distance to the boundary)`.
fn data_inside(rng: &mut Rng, n: usize, margin: f64, dip: impl Fn(f64) -> f64) -> Vec<[f64; 3]> {
    let poly = pentagon();
    let mut out = Vec::with_capacity(n);
    while out.len() < n {
        let p = [2.0 * RADIUS * rng.next() - RADIUS, 2.0 * RADIUS * rng.next() - RADIUS];
        let d = inner_distance(p, &poly);
        if d >= margin {
            out.push([p[0], p[1], dip(d) + SIGMA * rng.normal()]);
        }
    }
    out
}

/// The diagnostic's noisy-sides case: sides that carry the data's noise (corners exact), 1500 points.
fn noisy_request(soft: bool) -> Request {
    let mut r = rng(11);
    let sides = pentagon_sides(soft, |end| if end { 0.0 } else { SIGMA * r.normal() });
    let mut req = request(sides);
    req.data = data_inside(&mut r, 1500, 0.5, |_| 0.0);
    req
}

#[test]
fn noisy_sides_soft() {
    let patch = fit_patch(&noisy_request(true)).unwrap();
    let r = &patch.report;
    assert!((0.024..=0.036).contains(&r.rms), "{r:?}");
    assert!(r.control[0] <= 12 && r.control[1] <= 12, "{r:?}");
    assert!(r.within_tolerance, "{r:?}");
    assert!(r.sides.iter().all(|s| s.soft), "{r:?}");
}

#[test]
fn noisy_sides_hard_are_reported() {
    let patch = fit_patch(&noisy_request(false)).unwrap();
    assert!(!patch.report.within_targets, "{:?}", patch.report);
}

#[test]
fn rim_soft_sides() {
    // the scan rounds off within 1 mm of the boundary and the traced sides sit at the rounded height
    let mut r = rng(21);
    let sides = pentagon_sides(true, |_| -0.2);
    let mut req = request(sides);
    req.data =
        data_inside(&mut r, 1500, 0.1, |d| if d < 1.0 { -0.1 * (1.0 + (std::f64::consts::PI * d).cos()) } else { 0.0 });
    let patch = fit_patch(&req).unwrap();
    assert!(patch.report.max <= 0.2, "{:?}", patch.report);
}

#[test]
fn data_outside_outline_is_ignored() {
    let mut r = rng(31);
    let mut req = request(pentagon_sides(false, |_| 0.0));
    req.data = data_inside(&mut r, 1200, 0.5, |_| 0.0);
    let poly = pentagon();
    let mut far = 0;
    while far < 150 {
        let p = [2.2 * RADIUS * r.next() - 1.1 * RADIUS, 2.2 * RADIUS * r.next() - 1.1 * RADIUS];
        if inner_distance(p, &poly) < 0.0 && (1.0..=3.0).contains(&boundary_distance(p, &poly)) {
            req.data.push([p[0], p[1], 5.0]);
            far += 1;
        }
    }
    let patch = fit_patch(&req).unwrap();
    let rep = &patch.report;
    assert!(rep.outside >= 150, "{rep:?}");
    assert!(rep.max <= 0.2, "{rep:?}");
    assert!(rep.warnings.iter().any(|w| w.contains("lie outside the outline and were ignored")), "{rep:?}");
}

/// A five-sided hole in the unrolled cylinder whose sides are chords of 8 segments (densified in a
/// straight line), so that the side points lie off the cylinder by the sag of the chords.
fn chordal_cylinder_hole() -> Request {
    let corners = regular(5, [0.0, 0.0], 6.0);
    let sides = (0..corners.len())
        .map(|k| {
            let (a, b) = (corners[k], corners[(k + 1) % corners.len()]);
            let vertex = |i: usize| {
                let t = i as f64 / 8.0;
                on_cylinder(a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]))
            };
            let mut points = Vec::new();
            for i in 0..8 {
                let (p, q) = (vertex(i), vertex(i + 1));
                for j in 0..5 {
                    let t = j as f64 / 5.0;
                    points.push([p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1]), p[2] + t * (q[2] - p[2])]);
                }
            }
            points.push(vertex(8));
            Side { points, continuity: Continuity::G1, reference: Some(cylinder_face()), uv: None, soft: false }
        })
        .collect();
    request(sides)
}

#[test]
fn snap_moves_sides_onto_the_reference() {
    let patch = fit_patch(&chordal_cylinder_hole()).unwrap();
    for s in &patch.report.sides {
        assert!(s.snap_max.is_some_and(|m| m > 0.0 && m <= 0.1), "{:?}", patch.report);
        assert!(s.angle_max_deg.unwrap() <= G1_TARGET_DEG, "{:?}", patch.report);
    }
}

#[test]
fn side_far_from_reference_is_refused() {
    let mut req = cylinder_hole(Continuity::G1);
    // side 2 lifted 0.5 mm off the cylinder, its neighbours follow at the corners so the loop closes
    let lift = |p: &mut [f64; 3]| {
        p[0] *= 1.0 + 0.5 / RADIUS;
        p[1] *= 1.0 + 0.5 / RADIUS;
    };
    req.sides[2].points.iter_mut().for_each(lift);
    let (first, last) = (req.sides[2].points[0], req.sides[2].points[req.sides[2].points.len() - 1]);
    let prev = req.sides[1].points.len() - 1;
    req.sides[1].points[prev] = first;
    req.sides[3].points[0] = last;
    let err = fit_patch(&req).unwrap_err();
    assert!(err.to_string().contains("off its reference face"), "{err}");
}
