use super::*;
use crate::patch::{chord_curve, interpolated_net};

const R: f64 = 10.0;
/// Points along a curve or across a patch at which the true surface is checked.
const CHECK: usize = 60;

/// The closest point on the cylinder of radius [`R`] about the z axis.
fn on_cylinder(p: Vec3) -> Vec3 {
    let r = p.x.hypot(p.y);
    Vec3::new(p.x * R / r, p.y * R / r, p.z)
}

/// The point on the sphere of radius [`R`] about the origin under `p`, moved along the normal by a
/// deterministic noise of amplitude `sigma` (it varies over a fraction of a millimetre).
fn on_noisy_sphere(p: Vec3, sigma: f64) -> Vec3 {
    let n = p.normalize();
    let noise = (37.1 * p.x + 11.3 * p.y + 5.7 * p.z).sin() * (23.9 * p.y - 13.1 * p.z).cos();
    n * (R + sigma * noise * std::f64::consts::SQRT_2)
}

fn curve_gap(c: &[Vec3; 4]) -> f64 {
    (0..=CHECK).map(|k| (curve_point(c, k as f64 / CHECK as f64).xy().norm() - R).abs()).fold(0.0, f64::max)
}

fn net_rms(net: &Net) -> f64 {
    let mut sum = 0.0;
    for i in 0..=CHECK {
        for j in 0..=CHECK {
            let (p, _, _) = eval(net, i as f64 / CHECK as f64, j as f64 / CHECK as f64);
            sum += (p.norm() - R).powi(2);
        }
    }
    (sum / ((CHECK + 1) * (CHECK + 1)) as f64).sqrt()
}

fn at_angle(deg: f64, z: f64) -> Vec3 {
    let a = deg.to_radians();
    Vec3::new(R * a.cos(), R * a.sin(), z)
}

fn sphere_corner(lat: f64, lon: f64) -> Vec3 {
    let (la, lo) = (lat.to_radians(), lon.to_radians());
    Vec3::new(R * la.cos() * lo.cos(), R * la.cos() * lo.sin(), R * la.sin())
}

/// A 100 degree arc of a cylinder of radius 10, rising 3 mm: its ends and its chord interpolant.
fn arc() -> (Vec3, Vec3, [Vec3; 4]) {
    let (a, b) = (at_angle(0.0, 0.0), at_angle(100.0, 3.0));
    (a, b, chord_curve(a, b, &on_cylinder))
}

/// The boundary curves (on the sphere) of a 40 x 40 degree patch, and its interpolated net under a
/// scan with noise `sigma`.
fn sphere_patch(sigma: f64) -> Net {
    let c = [
        sphere_corner(-20.0, -20.0),
        sphere_corner(-20.0, 20.0),
        sphere_corner(20.0, 20.0),
        sphere_corner(20.0, -20.0),
    ];
    let exact = |p: Vec3| on_noisy_sphere(p, 0.0);
    let curve = |x: Vec3, y: Vec3| refit_curve(chord_curve(x, y, &exact), &|p| Some(exact(p)));
    let (bot, right, top, left) = (curve(c[0], c[1]), curve(c[1], c[2]), curve(c[3], c[2]), curve(c[0], c[3]));
    interpolated_net(&bot, &right, &top, &left, &|p| on_noisy_sphere(p, sigma))
}

fn boundary_kept(a: &Net, b: &Net) -> bool {
    (0..4).all(|k| a[k][0] == b[k][0] && a[3][k] == b[3][k] && a[k][3] == b[k][3] && a[0][k] == b[0][k])
}

#[test]
fn a_curve_over_a_strongly_curved_arc_is_refitted_much_closer_to_it() {
    // Arrange
    let (a, b, chord) = arc();

    // Act
    let refit = refit_curve(chord, &|p| Some(on_cylinder(p)));

    // Assert
    let (before, after) = (curve_gap(&chord), curve_gap(&refit));
    eprintln!("largest gap from the arc: chord interpolant {before:.4} mm, refitted {after:.4} mm");
    let thirds = [a + (b - a) / 3.0, a + (b - a) * (2.0 / 3.0)];
    let shift = (refit[1] - thirds[0]).norm().max((refit[2] - thirds[1]).norm()) / (b - a).norm();
    eprintln!("inner control points lie at most {shift:.3} of the chord off the chord's thirds");
    assert!(shift < MAX_SHIFT_SHARE, "{shift}");
    assert_ne!(refit, chord, "the refit changes the curve");
    assert!(after < 0.25 * before, "{before} -> {after}");
    assert_eq!((refit[0], refit[3]), (a, b), "the ends stay where they are");
}

#[test]
fn a_curve_with_half_its_samples_out_of_reach_is_refitted_on_the_others() {
    // Arrange: the scan ends at 50 degrees, the far half of the arc has no scan point within reach
    let (_, _, chord) = arc();
    let project = |p: Vec3| (p.y.atan2(p.x).to_degrees() < 50.0).then(|| on_cylinder(p));
    let before = curve_samples(&chord, &project);
    assert_eq!(valid(&before), 3, "{before:?}");

    // Act
    let refit = refit_curve(chord, &project);

    // Assert: kept, and closer over the samples valid both times
    assert_ne!(refit, chord);
    assert!(closer(&before, &curve_samples(&refit, &project), MIN_CURVE_SAMPLES));
}

#[test]
fn a_curve_with_too_few_samples_on_the_scan_stays_as_it_was() {
    // Arrange: the scan ends at 20 degrees
    let (_, _, chord) = arc();
    let project = |p: Vec3| (p.y.atan2(p.x).to_degrees() < 20.0).then(|| on_cylinder(p));
    assert!(valid(&curve_samples(&chord, &project)) < MIN_CURVE_SAMPLES);

    // Act / Assert
    assert_eq!(refit_curve(chord, &project), chord);
}

#[test]
fn a_patch_over_a_noisy_sphere_is_refitted_closer_to_the_sphere() {
    // Arrange: a patch whose boundary lies on the sphere, its inside from a scan with noise 0.05 mm
    let interpolated = sphere_patch(0.05);

    // Act
    let refit = refit_net(interpolated, &|p| Some(on_noisy_sphere(p, 0.05)));

    // Assert
    let (before, after) = (net_rms(&interpolated), net_rms(&refit));
    eprintln!("RMS distance from the sphere: interpolated {before:.4} mm, refitted {after:.4} mm");
    assert_ne!(refit, interpolated, "the refit changes the patch");
    assert!(after < 0.6 * before, "{before} -> {after}");
    assert!(boundary_kept(&interpolated, &refit), "the boundary is shared with the neighbours and stays");
}

#[test]
fn a_patch_with_part_of_its_samples_out_of_reach_is_refitted_on_the_others() {
    // Arrange: a third of the patch (longitude above 7 degrees) has no scan point within reach
    let interpolated = sphere_patch(0.05);
    let project = |p: Vec3| (p.y.atan2(p.x).to_degrees() < 7.0).then(|| on_noisy_sphere(p, 0.05));
    let before = net_samples(&interpolated, &project);
    let n = valid(&before);
    assert!((MIN_NET_SAMPLES..before.len()).contains(&n), "{n} valid samples");

    // Act
    let refit = refit_net(interpolated, &project);

    // Assert
    assert_ne!(refit, interpolated);
    assert!(closer(&before, &net_samples(&refit, &project), MIN_NET_SAMPLES));
    assert!(boundary_kept(&interpolated, &refit));
}

#[test]
fn a_patch_with_too_few_samples_on_the_scan_stays_as_it_was() {
    // Arrange: only the strip below -12 degrees of longitude reaches the scan
    let interpolated = sphere_patch(0.05);
    let project = |p: Vec3| (p.y.atan2(p.x).to_degrees() < -12.0).then(|| on_noisy_sphere(p, 0.05));
    assert!(valid(&net_samples(&interpolated, &project)) < MIN_NET_SAMPLES);

    // Act / Assert
    assert_eq!(refit_net(interpolated, &project), interpolated);
}

#[test]
fn a_refit_that_would_fold_or_turn_the_patch_is_refused() {
    // Arrange: a flat patch, and a "scan" 1000 mm under it: fitting it pulls the inside down into
    // walls that face sideways
    let flat = sphere_patch(0.0).map(|row| row.map(|p| [p[0], p[1], 0.0]));
    let project = |p: Vec3| Some(Vec3::new(p.x, p.y, -1000.0));
    let unguarded = net_through(&flat, &net_samples(&flat, &project)).unwrap();

    // Act
    let refit = refit_net(flat, &project);

    // Assert: the unguarded fit is closer at the samples but turns the patch, so it is not taken
    assert!(closer(&net_samples(&flat, &project), &net_samples(&unguarded, &project), MIN_NET_SAMPLES));
    assert!(!keeps_facing(&flat, &unguarded));
    assert_eq!(refit, flat);
}

#[test]
fn a_projection_that_gives_nothing_usable_leaves_the_curve_and_the_patch_as_they_were() {
    // Arrange
    let (a, b) = (at_angle(0.0, 0.0), at_angle(60.0, 1.0));
    let c = chord_curve(a, b, &on_cylinder);
    let net = interpolated_net(&c, &c, &c, &c, &on_cylinder);
    let nan = |_: Vec3| Some(Vec3::repeat(f64::NAN));
    let none = |_: Vec3| -> Option<Vec3> { None };
    let identity = |p: Vec3| Some(p);
    let projects: [Project; 3] = [&nan, &none, &identity];

    for project in projects {
        // Act / Assert
        assert_eq!(refit_curve(c, project), c);
        assert_eq!(refit_net(net, project), net);
    }
}

#[test]
fn a_refit_never_puts_its_samples_farther_from_the_scan_and_changes_noisy_curves() {
    // a coarse edge over more than about 120 degrees of a surface does not occur at the wrap's budgets
    for (deg, sigma) in [(20.0, 0.0), (90.0, 0.02), (120.0, 0.1)] {
        // Arrange
        let project = |p: Vec3| Some(on_noisy_sphere(p, sigma));
        let c = chord_curve(sphere_corner(0.0, 0.0), sphere_corner(10.0, deg), &|p| on_noisy_sphere(p, sigma));

        // Act
        let refit = refit_curve(c, &project);

        // Assert
        let (before, after) = (curve_samples(&c, &project), curve_samples(&refit, &project));
        assert!(refit == c || closer(&before, &after, MIN_CURVE_SAMPLES), "{deg} {sigma}");
        if sigma > 0.0 {
            assert_ne!(refit, c, "{deg} {sigma}: a noisy curve is refitted");
        }
    }
}

#[test]
fn a_refit_that_loses_a_sample_is_not_taken_however_well_the_rest_fit() {
    // Arrange: before, all six samples reach the scan; after, five fit far better and one is lost
    let before: Vec<Sample> = (0..6).map(|_| Some((Vec3::zeros(), 0.04))).collect();
    let mut after: Vec<Sample> = (0..6).map(|_| Some((Vec3::zeros(), 0.0001))).collect();
    let keeps_all = after.clone();
    after[5] = None;

    // Act / Assert
    assert!(!closer(&before, &after, MIN_CURVE_SAMPLES), "a lost sample refuses the refit");
    assert!(closer(&before, &keeps_all, MIN_CURVE_SAMPLES), "one that keeps them all is taken as before");
}

#[test]
fn a_refit_that_would_slide_a_curve_off_the_scan_is_refused() {
    // Arrange: the scan ends at 60 degrees of the arc; points beyond it have no scan point within
    // reach. A refit that kept fitting the near samples but slid the far ones off the scan would lose
    // them, so whatever the refit returns keeps every sample the chord interpolant had.
    let (_, _, chord) = arc();
    let project = |p: Vec3| (p.y.atan2(p.x).to_degrees() < 60.0).then(|| on_cylinder(p));
    let before = curve_samples(&chord, &project);

    // Act
    let refit = refit_curve(chord, &project);

    // Assert
    let after = curve_samples(&refit, &project);
    let lost = before.iter().zip(&after).filter(|(a, b)| a.is_some() && b.is_none()).count();
    assert_eq!(lost, 0, "{before:?} -> {after:?}");
}

#[test]
fn a_fit_on_samples_at_one_end_that_swings_the_far_control_point_is_refused() {
    // Arrange: a straight curve 10 mm long; only its first 4.5 mm reach a "scan" that rises steeply
    // (12 mm over the curve's length), so the least-squares cubic through those samples swings its far
    // inner control point far away to come back to the fixed end
    let (a, b) = (Vec3::zeros(), Vec3::new(10.0, 0.0, 0.0));
    let c = [a, a + (b - a) / 3.0, a + (b - a) * (2.0 / 3.0), b];
    let project = |p: Vec3| (p.x < 4.5).then(|| Vec3::new(p.x, p.y, 1.2 * p.x));
    let s = curve_samples(&c, &project);
    let unguarded = curve_through(&c, &s).unwrap();
    assert!(!within_shift(&[c[1], c[2]], &[unguarded[1], unguarded[2]], 10.0), "{unguarded:?}");
    let far = (unguarded[2] - c[2]).norm() / 10.0;
    eprintln!("the unguarded fit moves the far control point by {far:.2} of the chord (bound {MAX_SHIFT_SHARE})");

    // Act / Assert
    assert_eq!(refit_curve(c, &project), c);
}
