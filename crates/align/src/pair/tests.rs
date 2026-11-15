//! Unit tests of `pair_coarse` (candidates, tolerance, verdict inputs).
use super::*;
use surfacecraft_geom::{Rng, Vec3, synth};

fn bracket(block: usize, seed: u64) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), block);
    m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
    m.append(&synth::sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 32, 16));
    synth::add_noise(&mut m, 0.05, &mut Rng::new(seed));
    m
}

fn part(m: &Mesh, keep: impl Fn(&Point3) -> bool) -> Mesh {
    let ids: Vec<u32> =
        (0..m.triangle_count()).filter(|&i| m.face_centroid(i).is_some_and(|c| keep(&c))).map(|i| i as u32).collect();
    m.submesh(&ids)
}

/// Two partial scans share half the part: no principal frame lines them up (their centroids
/// and axes differ), so a candidate near the truth can only come from the descriptors.
#[test]
fn only_the_descriptors_find_the_pose_of_partial_scans() {
    let fixed = part(&bracket(12, 1), |c| c.z > 1.0 && c.x <= 54.0);
    let moving = part(&bracket(15, 2), |c| c.z > 1.0 && c.x >= 6.0);
    let pose = Transform::rotation_axis_angle(&Vec3::new(0.2, 0.7, -0.4), 2.0)
        .then_after(&Transform::translation(Vec3::new(60.0, 10.0, -30.0)));
    let moved = moving.transformed(&pose);
    let scene = Scene::new(&moved, &fixed, None, &Cancel::new()).unwrap();
    let hyps = scene.hypotheses(&Cancel::new()).unwrap();
    let truth = pose.inverse();
    let miss = |method: CoarseMethod| {
        hyps.iter()
            .filter(|c| c.method == method)
            .map(|c| c.t.max_displacement(&truth, &moved.positions))
            .fold(f64::INFINITY, f64::min)
    };
    let (descriptors, principal) = (miss(CoarseMethod::Descriptors), miss(CoarseMethod::PrincipalAxes));
    assert!(descriptors < 3.0, "descriptor candidates miss the truth by {descriptors} mm");
    assert!(principal > 8.0, "the principal axes should not find it ({principal} mm): the test would prove nothing");
}

#[test]
fn a_whole_part_is_found_from_its_principal_axes_alone() {
    let fixed = bracket(12, 1);
    let moving = bracket(15, 2);
    let pose = Transform::rotation_axis_angle(&Vec3::new(1.0, 0.1, 0.3), 2.8)
        .then_after(&Transform::translation(Vec3::new(-50.0, 20.0, 90.0)));
    let moved = moving.transformed(&pose);
    let scene = Scene::new(&moved, &fixed, None, &Cancel::new()).unwrap();
    let hyps = scene.hypotheses(&Cancel::new()).unwrap();
    let near = hyps
        .iter()
        .filter(|c| c.method == CoarseMethod::PrincipalAxes)
        .map(|c| c.t.max_displacement(&pose.inverse(), &moved.positions))
        .fold(f64::INFINITY, f64::min);
    assert!(near < 3.0, "{near}");
}

/// A pose whose own residual is large (it lies on other regions of the surface) does not loosen
/// the tolerance: it never exceeds `TAU_SIGMAS` times the noise of the pair, which is the noise
/// of the scans (0.05 mm each), not of the pose.
#[test]
fn the_tolerance_is_capped_by_the_noise_of_the_pair() {
    let (fixed, moving) = (bracket(12, 1), bracket(15, 2));
    let scene = Scene::new(&moving, &fixed, None, &Cancel::new()).unwrap();
    let noise = scene.noise.unwrap();
    let cap = TAU_SIGMAS * noise;
    eprintln!("noise of the pair {noise:.4}, cap {cap:.3}, range {:?}", TAU_RANGE);
    assert!((0.04..0.14).contains(&noise), "two scans of sigma 0.05 give 0.07: {noise}");
    let off = Transform::translation(Vec3::new(4.0, 2.0, 0.0));
    let (right, wrong) = (scene.tolerance_at(&Transform::identity(), None), scene.tolerance_at(&off, None));
    eprintln!("tolerance at the right pose {right:.3}, at a pose 4.5 mm off {wrong:.3}");
    assert!(right <= cap && wrong <= cap);
    assert!(wrong >= TAU_RANGE.0 * scene.diag);
    assert_eq!(scene.tolerance_at(&off, Some(2.0)), 2.0, "the caller's tolerance is not capped");
}

/// With no trustworthy noise estimate the tolerance is capped by the edge length instead: a pose
/// that is far off has a large residual and still cannot loosen the tolerance beyond it.
#[test]
fn without_a_noise_estimate_the_tolerance_is_capped_by_the_edge_length() {
    let (fixed, moving) = (bracket(12, 1), bracket(15, 2));
    let coarse = ScanNoise { sigma: 0.2, smooth: false };
    let scene = Scene::new(&moving, &fixed, Some((coarse, coarse)), &Cancel::new()).unwrap();
    assert_eq!(scene.noise, None);
    let cap = TAU_EDGE * scene.edge;
    let off = Transform::translation(Vec3::new(4.0, 2.0, 0.0));
    let tau = scene.tolerance_at(&off, None);
    eprintln!("edge {:.2} mm, cap {cap:.3}, tolerance {tau:.3}", scene.edge);
    assert!(tau <= cap.max(TAU_RANGE.0 * scene.diag) + 1e-12);
    assert!(scene.edge > 1.0 && scene.edge < 20.0, "{}", scene.edge);
}

/// A cancelled run stops before it estimates any noise.
#[test]
fn a_cancelled_pair_stops_early() {
    let (fixed, moving) = (bracket(12, 1), bracket(15, 2));
    let cancel = Cancel::new();
    cancel.cancel();
    assert!(matches!(pair_coarse(&moving, &fixed, &PairOptions::default(), &cancel), Err(AlignError::Cancelled)));
}

/// The noise estimates handed in (as `align.global` does, once per scan) give the very answer
/// the pair gives when it estimates them itself.
#[test]
fn noise_estimates_handed_in_change_nothing() {
    let (fixed, moving) = (bracket(12, 1), bracket(15, 2));
    let moved = moving.transformed(&Transform::translation(Vec3::new(30.0, -20.0, 10.0)));
    let own = pair_coarse(&moved, &fixed, &PairOptions::default(), &Cancel::new()).unwrap();
    let noises = Some((ScanNoise::of(&moved), ScanNoise::of(&fixed)));
    let given = pair_coarse(&moved, &fixed, &PairOptions { noises, ..PairOptions::default() }, &Cancel::new()).unwrap();
    assert_eq!(own.transform, given.transform);
    assert_eq!((own.tolerance, own.noise, own.verdict), (given.tolerance, given.noise, given.verdict));
    assert!(own.noise > 0.0);
}

/// Finding 6: the count of refined candidates is produced by `solve`; with a single candidate
/// there is nothing to compare, so the right pose is still not certain, and `refined` says 1.
#[test]
fn one_candidate_gives_refined_one_and_never_unique() {
    let fixed = bracket(12, 1);
    let pose = Transform::rotation_axis_angle(&Vec3::new(1.0, 0.1, 0.3), 2.8)
        .then_after(&Transform::translation(Vec3::new(-50.0, 20.0, 90.0)));
    let moved = bracket(15, 2).transformed(&pose);
    let scene = Scene::new(&moved, &fixed, None, &Cancel::new()).unwrap();
    let hyps = scene.hypotheses(&Cancel::new()).unwrap();
    let truth = pose.inverse();
    let best = hyps
        .iter()
        .min_by(|a, b| {
            a.t.max_displacement(&truth, &moved.positions).total_cmp(&b.t.max_displacement(&truth, &moved.positions))
        })
        .copied()
        .unwrap();
    let alone = solve(&scene, vec![best], &PairOptions::default(), &Cancel::new()).unwrap();
    assert_eq!(alone.refined, 1);
    assert_eq!(alone.verdict, PairVerdict::Ambiguous);
    assert_eq!(alone.confidence, 0.0);
    let err = alone.transform.max_displacement(&truth, &moved.positions);
    assert!(err < 0.2, "the pose itself is right: {err}");
    let all = solve(&scene, hyps, &PairOptions::default(), &Cancel::new()).unwrap();
    assert!(all.refined >= 2, "{}", all.refined);
    assert_eq!(all.verdict, PairVerdict::Unique);
}
