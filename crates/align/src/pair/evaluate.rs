//! How well a given pose brings two scans together, measured the way [`super::pair_coarse`]
//! measures its own poses (same tolerance rule, same border-aware conflicts, both directions).
use super::score::fit_of;
use super::{AlignError, Scene};
use surfacecraft_geom::{Cancel, Mesh, Transform};

/// The evidence of one pose.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseFit {
    /// Tolerance (mm) used: the caller's, or the default of [`super::pair_coarse`].
    pub tolerance: f64,
    /// Noise (mm) of the pair, as [`super::PairResult::noise`].
    pub noise: f64,
    /// Share (0..1) of the moving scan's area within the tolerance of the fixed scan.
    pub overlap_moving: f64,
    /// Share (0..1) of the fixed scan's area within the tolerance of the moving scan.
    pub overlap_fixed: f64,
    /// Share (0..1) of the moving scan's area where the fixed scan is, but off it.
    pub conflict_moving: f64,
    /// Share (0..1) of the fixed scan's area where the moving scan is, but off it.
    pub conflict_fixed: f64,
    /// RMS distance (mm) of the matched part of the moving scan (infinite when nothing matched).
    pub rms: f64,
}

/// Measures `pose` (moving -> fixed) without searching or refining it. `tolerance` as in
/// [`super::PairOptions::tolerance`]. For tests and diagnostics.
pub fn pose_fit(
    moving: &Mesh,
    fixed: &Mesh,
    pose: &Transform,
    tolerance: Option<f64>,
    cancel: &Cancel,
) -> Result<PoseFit, AlignError> {
    let scene = Scene::new(moving, fixed, cancel)?;
    scene.check_tolerance(tolerance)?;
    let tau = scene.tolerance_at(pose, tolerance);
    let on_fixed = fit_of(&scene.fixed, &scene.mov.pts, pose, tau, true);
    let on_moving = scene.fits_fixed(std::slice::from_ref(pose), tau);
    let other = on_moving.first();
    Ok(PoseFit {
        tolerance: tau,
        noise: scene.noise,
        overlap_moving: on_fixed.share,
        overlap_fixed: other.map_or(0.0, |f| f.share),
        conflict_moving: on_fixed.conflict,
        conflict_fixed: other.map_or(0.0, |f| f.conflict),
        rms: on_fixed.rms,
    })
}

#[cfg(test)]
mod tests {
    use super::super::verdict::MAX_MUTUAL_CONFLICT;
    use super::*;
    use surfacecraft_geom::{Point3, Rng, Vec3, synth};

    fn bracket(block: usize, seed: u64) -> Mesh {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), block);
        m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
        m.append(&synth::sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 32, 16));
        synth::add_noise(&mut m, 0.05, &mut Rng::new(seed));
        m
    }

    /// The bracket turned half a turn about its vertical axis through the centre of its box lands the
    /// box on itself and the boss on the sphere and the sphere on the boss: a pose that fits the
    /// faces and that both scans contradict. The right pose is contradicted by neither.
    #[test]
    fn a_pose_on_the_wrong_features_is_contradicted_by_both_scans() {
        let (fixed, moving) = (bracket(12, 1), bracket(15, 2));
        let half_turn = Transform::translation(Vec3::new(60.0, 40.0, 0.0))
            .then_after(&Transform::rotation_axis_angle(&Vec3::z(), std::f64::consts::PI));
        let right = pose_fit(&moving, &fixed, &Transform::identity(), None, &Cancel::new()).unwrap();
        let wrong = pose_fit(&moving, &fixed, &half_turn, None, &Cancel::new()).unwrap();
        eprintln!("right {right:?}\nwrong {wrong:?}");
        assert!(right.overlap_moving > 0.9 && right.overlap_fixed > 0.9);
        assert!(right.conflict_moving.min(right.conflict_fixed) <= MAX_MUTUAL_CONFLICT / 2.0);
        assert!(wrong.overlap_moving > 0.5, "the faces fit: {}", wrong.overlap_moving);
        assert!(wrong.conflict_moving.min(wrong.conflict_fixed) >= 2.0 * MAX_MUTUAL_CONFLICT);
    }

    #[test]
    fn a_tolerance_the_caller_gives_is_used_as_it_is() {
        let (fixed, moving) = (bracket(12, 1), bracket(15, 2));
        let f = pose_fit(&moving, &fixed, &Transform::identity(), Some(1.25), &Cancel::new()).unwrap();
        assert_eq!(f.tolerance, 1.25);
        let bad = pose_fit(&moving, &fixed, &Transform::identity(), Some(-1.0), &Cancel::new());
        assert!(bad.is_err());
    }
}
