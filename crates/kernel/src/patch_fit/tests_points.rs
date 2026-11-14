//! Soft interior points: soft position rows with their weight, not scan data.
use super::tests::{polygon, regular, request};
use super::*;
use crate::nurbs::SurfaceProbe;
use crate::vpatch::derivatives;

/// A flat pentagon of radius 15 in the plane z = 0 with the given soft points and a small
/// smoothing (the points then decide the shape of the interior).
fn flat_pentagon(soft_points: Vec<([f64; 3], f64)>) -> Request {
    let mut req = request(polygon(&regular(5, [0.0, 0.0], 15.0), 30, &|x, y| [x, y, 0.0]));
    req.soft_points = soft_points;
    req.smoothing = Some(1e-5);
    req
}

/// Height of the patch under the centre of the pentagon.
fn height_under_centre(patch: &Patch) -> f64 {
    let probe = SurfaceProbe::new(&patch.surface).unwrap();
    let (_, _, uv) = probe.closest_uv([0.0, 0.0, 1.0]).unwrap();
    derivatives(&patch.surface, uv).unwrap()[0][2]
}

#[test]
fn a_soft_point_pulls_the_patch_by_its_weight() {
    let mut last = f64::NEG_INFINITY;
    for w in [0.01, 1.0, 100.0] {
        let patch = fit_patch(&flat_pentagon(vec![([0.0, 0.0, 1.0], w)])).unwrap();
        let h = height_under_centre(&patch);
        assert!(h > last, "weight {w}: height {h} after {last}");
        last = h;
        // one point is no scan data
        assert_eq!(patch.report.points, 0, "{:?}", patch.report);
        let miss = patch.report.soft_points_max.unwrap();
        assert!((miss - (1.0 - h)).abs() < 0.02, "weight {w}: distance {miss}, height {h}");
    }
    assert!(last > 0.9, "height {last} at weight 100");
}

#[test]
fn soft_points_are_not_data() {
    // four soft points are more than the minimum number of data points, and still no data
    let pts = vec![([2.0, 0.0, 0.5], 1.0), ([-2.0, 0.0, 0.5], 1.0), ([0.0, 2.0, 0.5], 1.0), ([0.0, -2.0, 0.5], 1.0)];
    let patch = fit_patch(&flat_pentagon(pts)).unwrap();
    let r = &patch.report;
    assert_eq!(r.points, 0, "{r:?}");
    assert!(r.warnings.iter().any(|w| w == WARN_NO_DATA), "{r:?}");
    assert!(r.soft_points_max.is_some_and(|m| m.is_finite()), "{r:?}");
    // without soft points nothing is reported
    assert!(fit_patch(&flat_pentagon(Vec::new())).unwrap().report.soft_points_max.is_none());
}

#[test]
fn a_soft_point_outside_the_outline_is_refused_by_name() {
    let pts = vec![([0.0, 0.0, 0.5], 1.0), ([40.0, 0.0, 0.0], 1.0)];
    let e = fit_patch(&flat_pentagon(pts)).unwrap_err().to_string();
    assert!(e.contains("soft point 1") && e.contains("outside the outline"), "{e}");
}
