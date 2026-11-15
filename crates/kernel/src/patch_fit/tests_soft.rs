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

#[test]
fn soft_sides_off_the_data_still_give_a_sheet() {
    // the traced sides lie 0.3 mm above the noisy data: the closest points of the side samples on the
    // patch would crowd at the border of what the patch reaches, the samples' own parameters do not
    let mut r = rng(41);
    let mut req = request(pentagon_sides(true, |_| 0.3));
    req.data = data_inside(&mut r, 1500, 0.5, |_| 0.0);
    let patch = fit_patch(&req).unwrap();
    assert_eq!(patch.sheet.faces.len(), 1);
    assert!(patch.report.sides.iter().all(|s| s.soft), "{:?}", patch.report);
}

/// Height of a cylinder of radius `BEND` with its axis along y, touching z = 0 from below.
const BEND: f64 = 40.0;
fn bent(x: f64) -> f64 {
    (BEND * BEND - x * x).sqrt() - BEND
}

/// Surface texture of a real scan (a cast or machined skin): ripples far below the tolerance whose
/// curvature, `TEXTURE * (TAU / RIPPLE)^2 = 0.033 1/mm`, is larger than the shape's own.
const TEXTURE: f64 = 0.03;
const RIPPLE: f64 = 6.0;
fn textured(x: f64, y: f64) -> f64 {
    let w = std::f64::consts::TAU / RIPPLE;
    bent(x) + TEXTURE * (w * x).sin() * (w * y).sin()
}

/// Largest error of the patch's normal curvature across the bend against the cylinder's `1 / BEND`,
/// on a grid well inside the outline.
fn bend_curvature_error(patch: &Patch) -> f64 {
    let probe = crate::nurbs::SurfaceProbe::new(&patch.surface).unwrap();
    let mut worst = 0.0_f64;
    for i in -4..=4 {
        for j in -4..=4 {
            let (x, y) = (2.0 * f64::from(i), 2.0 * f64::from(j));
            let (_, _, uv) = probe.closest_uv([x, y, bent(x)]).unwrap();
            let d = crate::vpatch::derivatives(&patch.surface, uv).unwrap();
            let (su, sv) = (d[1], d[2]);
            // the parameter direction whose image is closest to the x axis
            let (e, f, g) = (geom::dot(su, su), geom::dot(su, sv), geom::dot(sv, sv));
            let (bu, bv) = (su[0], sv[0]);
            let det = e * g - f * f;
            let (du, dv) = ((g * bu - f * bv) / det, (e * bv - f * bu) / det);
            let sd = geom::add(geom::scale(su, du), geom::scale(sv, dv));
            let sdd = geom::add(
                geom::add(geom::scale(d[3], du * du), geom::scale(d[4], 2.0 * du * dv)),
                geom::scale(d[5], dv * dv),
            );
            let n = geom::unit(geom::cross(su, sv)).unwrap();
            let kappa = geom::dot(sdd, n) / geom::dot(sd, sd);
            worst = worst.max((kappa.abs() - 1.0 / BEND).abs());
        }
    }
    worst
}

/// The textured cylinder as a request: soft sides on the scan, 6000 points, nets up to 24.
fn textured_request(tolerance: f64) -> Request {
    let mut r = rng(31);
    let poly = pentagon();
    let sides: Vec<Side> = (0..poly.len())
        .map(|k| {
            let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
            let points = (0..SIDE_POINTS)
                .map(|i| {
                    let t = i as f64 / (SIDE_POINTS - 1) as f64;
                    let (x, y) = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]));
                    let end = i == 0 || i + 1 == SIDE_POINTS;
                    [x, y, if end { bent(x) } else { textured(x, y) }]
                })
                .collect();
            Side { points, continuity: Continuity::G0, reference: None, uv: None, soft: true }
        })
        .collect();
    let mut req = request(sides);
    req.max_control = 24;
    req.tolerance = tolerance;
    let flat = data_inside(&mut r, 6000, 0.5, |_| 0.0);
    req.data = flat.into_iter().map(|p| [p[0], p[1], p[2] / 3.0 + textured(p[0], p[1])]).collect();
    req
}

#[test]
fn soft_fit_of_a_dense_scan_keeps_the_curvature_of_the_shape() {
    // a net sized from the number of points (22 by 22 here) follows the texture: its curvature is
    // off by 0.032 1/mm, more than the cylinder's own 0.025, and useless as a G2 reference. Within
    // the tolerance the coarse start is kept: measured 0.0009 1/mm (macOS aarch64)
    let patch = fit_patch(&textured_request(0.2)).unwrap();
    let rep = &patch.report;
    assert_eq!(rep.control, [6, 6], "{rep:?}");
    assert!(rep.within_tolerance, "{rep:?}");
    assert!(bend_curvature_error(&patch) <= 0.005, "{:.5} {rep:?}", bend_curvature_error(&patch));
}

#[test]
fn soft_fit_grows_the_net_when_the_tolerance_asks_for_the_texture() {
    // the texture is 0.03 mm high: a tolerance below it cannot be met by the start net
    let coarse = fit_patch(&textured_request(0.2)).unwrap().report;
    let fine = fit_patch(&textured_request(0.02)).unwrap().report;
    eprintln!(
        "MEASURED textured: start max {:.4}; grown net {:?} rounds {} max {:.4}",
        coarse.max, fine.control, fine.rounds, fine.max
    );
    assert!(fine.control[0] > 6 && fine.control[1] > 6, "{fine:?}");
    assert!(fine.rounds > 1, "{fine:?}");
    // measured: max 0.067 at 6 by 6, 0.039 at the grown net; bound at 1.5 times less than the start
    assert!(fine.max <= coarse.max / 1.5, "{} against {}: {fine:?}", fine.max, coarse.max);
}
