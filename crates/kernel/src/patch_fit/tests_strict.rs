//! `within_targets` is strict (no allowance for frozen sides) and a fold is detected. The fold is
//! exercised on a hand-made patch measured by `measure`: `fit_patch` cannot be made to fold, because
//! every row of the solve places the interior points by their own positions, which keeps the patch a
//! graph over the plane of the outline.
use super::measure::{Goal, Measure};
use super::rows::Layout;
use super::tests::{g0, request};
use super::*;
use crate::nurbs::SurfaceProbe;

fn side_report(gap: f64) -> SideReport {
    SideReport {
        continuity: Continuity::G0,
        soft: false,
        gap_max: gap,
        angle_max_deg: None,
        curvature_error_max: None,
        samples: 20,
        folded: false,
        snap_max: None,
        met: gap <= G0_TARGET,
    }
}

fn measure_of(sides: Vec<SideReport>) -> Measure {
    let n = sides.len();
    Measure {
        sides,
        feet: vec![Vec::new(); n],
        starts: vec![Vec::new(); n],
        rms: 0.0,
        p99: 0.0,
        max: 0.0,
        points: 0,
        outside: 0,
        hard_max: 0.0,
        hard_ok: true,
        soft_max: None,
        rest_ok: vec![true; n],
        side_terms: vec![[0.0, 0.0]; n],
        hard_term: 0.0,
        data_term: 0.0,
        miss: [0.0, 0.0],
        interior_folded: false,
        overhangs_plane: false,
    }
}

#[test]
fn a_frozen_side_is_not_excused_from_the_targets() {
    let m = measure_of(vec![side_report(2.0 * G0_TARGET)]);
    // the rounds may stop once the gap no longer improves ...
    assert!(m.sides_ok(&[true]));
    assert!(!m.sides_ok(&[false]));
    // ... but the targets are not met, frozen or not
    assert!(!m.targets_met());
    assert!(measure_of(vec![side_report(G0_TARGET)]).targets_met());
}

fn plane_face() -> Face {
    Face {
        surface: Surface::Plane { origin: [0.0; 3], u_axis: [1.0, 0.0, 0.0], v_axis: [0.0, 1.0, 0.0] },
        nurbs: None,
        domain: [[-50.0, 50.0], [-50.0, 50.0]],
        loops: Vec::new(),
    }
}

/// The square [0, 20]² in the plane z = 0, parameters (x, y); the side along y = 0 continues the
/// plane z = 0 (G1), the others are G0.
fn square_request() -> Request {
    let corners = [[0.0, 0.0], [20.0, 0.0], [20.0, 20.0], [0.0, 20.0]];
    let sides = (0..4)
        .map(|k| {
            let (a, b) = (corners[k], corners[(k + 1) % 4]);
            let points: Vec<[f64; 3]> = (0..=10)
                .map(|i| {
                    let t = f64::from(i) / 10.0;
                    [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), 0.0]
                })
                .collect();
            let uv = Some(points.iter().map(|p| [p[0], p[1]]).collect());
            let mut side = g0(points);
            side.uv = uv;
            if k == 0 {
                side.continuity = Continuity::G1;
                side.reference = Some(plane_face());
            }
            side
        })
        .collect();
    request(sides)
}

/// A 6 × 6 bicubic over `domain` lying in the plane z = 0 with `y` running with `v` (or against it
/// when `flip`).
fn flat_spline(domain: [[f64; 2]; 2], flip: bool) -> Surface {
    let knots = |[a, b]: [f64; 2]| {
        let h = (b - a) / 3.0;
        vec![a, a, a, a, a + h, a + 2.0 * h, b, b, b, b]
    };
    let points = (0..6)
        .map(|i| {
            (0..6)
                .map(|j| {
                    let x = domain[0][0] + (domain[0][1] - domain[0][0]) * f64::from(i) / 5.0;
                    let t = if flip { 1.0 - f64::from(j) / 5.0 } else { f64::from(j) / 5.0 };
                    [x, domain[1][0] + (domain[1][1] - domain[1][0]) * t, 0.0]
                })
                .collect()
        })
        .collect();
    Surface::Spline { u_knots: knots(domain[0]), v_knots: knots(domain[1]), points }
}

/// A 6 × 6 bicubic in the plane z = 0 whose control rows in u are pushed across each other: the
/// patch runs forward, back and forward again (a pleat), so its normal vanishes and flips.
fn pleated_spline(domain: [[f64; 2]; 2]) -> Surface {
    let Surface::Spline { u_knots, v_knots, points } = flat_spline(domain, false) else { panic!("a spline") };
    let x = [0.0, 0.2, 0.9, 0.1, 0.8, 1.0];
    let points = points
        .iter()
        .enumerate()
        .map(|(i, row)| row.iter().map(|p| [domain[0][0] + (domain[0][1] - domain[0][0]) * x[i], p[1], 0.0]).collect())
        .collect();
    Surface::Spline { u_knots, v_knots, points }
}

/// The measurement of `flat_spline` against the square's sides.
fn measure_square(flip: bool) -> Measure {
    measure_surface(flip, false)
}

fn measure_surface(flip: bool, pleat: bool) -> Measure {
    let mut prep = start::prepare(&square_request(), &Cancel::new()).unwrap();
    let net = [6, 6];
    let margin = |d: usize| (prep.lp.bbox[d][1] - prep.lp.bbox[d][0]) / (net[d] as f64 - 1.0);
    let domain = [
        [prep.lp.bbox[0][0] - margin(0), prep.lp.bbox[0][1] + margin(0)],
        [prep.lp.bbox[1][0] - margin(1), prep.lp.bbox[1][1] + margin(1)],
    ];
    let layout = Layout::new(domain, net);
    let samples = rows::sample_sides(&prep.lp, &layout, &Cancel::new()).unwrap();
    let surface = if pleat { pleated_spline(domain) } else { flat_spline(domain, flip) };
    let probe = SurfaceProbe::new(&surface).unwrap();
    let cancel = Cancel::new();
    let goal = Goal { tolerance: 0.2, frozen: &[false; 4], cancel: &cancel };
    measure::measure(&surface, &probe, &prep.lp, &layout, &samples, &mut prep.interior, &goal).unwrap()
}

#[test]
fn a_patch_that_runs_back_over_the_reference_face_is_folded() {
    // v runs against y: leaving the side at y = 0 into the patch, the surface moves over the face
    let m = measure_square(true);
    assert!(m.sides[0].folded, "{:?}", m.sides[0]);
    assert!(!m.rest_ok[0]);
    assert!(!m.targets_met());
    // the same surface with v running with y leaves the face behind: no fold, every target met
    let ok = measure_square(false);
    assert!(!ok.sides[0].folded, "{:?}", ok.sides[0]);
    assert!(ok.targets_met(), "{:?}", ok.sides);
}

#[test]
fn a_pleated_surface_folds_and_a_plain_one_does_not() {
    let pleated = measure_surface(false, true);
    assert!(pleated.interior_folded, "the normal vanishes and flips across the pleat");
    assert!(!pleated.targets_met());
    assert!(pleated.side_miss() > 1.0);
    let plain = measure_surface(false, false);
    assert!(!plain.interior_folded && !plain.overhangs_plane);
}
