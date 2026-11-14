//! Gate 2 of P9-M3, the decisive one: loop closure is what the relaxation is for. A ring of eight
//! scans where every pair pose carries a known error (a part common to all of them, like a drift of
//! the scanner, and a small random part): chaining the poses along the ring piles the error up at
//! the far scan; the relaxation spreads it around the loop. The test fails if `relax` returns the
//! chain it was given.
use surfacecraft_align::global::{Edge, misfit, relax};
use surfacecraft_geom::{Cancel, Mesh, Point3, Rng, Transform, Vec3, synth};

const SCANS: usize = 8;
/// Error of every pair pose: a drift of 0.3 degrees about the ring axis and 0.05 mm along it (the
/// same in every scan's frame, so it adds up around the ring), plus a random part of at most 0.05 degrees and 0.02 mm.
const DRIFT_DEGREES: f64 = 0.3;
const DRIFT_MM: f64 = 0.05;
const RANDOM_DEGREES: f64 = 0.05;
const RANDOM_MM: f64 = 0.02;
/// The relaxation must beat the chain at the far scan by at least this factor (measured: far more).
const REQUIRED_FACTOR: f64 = 3.0;

fn truth(k: usize) -> Transform {
    Transform::rotation_axis_angle(&Vec3::z(), (45.0 * k as f64).to_radians())
}

fn ring_edges(seed: u64) -> (Vec<Edge>, Mesh) {
    let part = synth::sphere(Point3::origin(), 30.0, 40, 20);
    let mut rng = Rng::new(seed);
    let mut edges = Vec::new();
    for k in 0..SCANS {
        let (i, j) = (k, (k + 1) % SCANS);
        let error = Transform::rotation_axis_angle(&Vec3::z(), DRIFT_DEGREES.to_radians())
            .then_after(&Transform::translation(Vec3::new(0.0, 0.0, DRIFT_MM)))
            .then_after(&Transform::rotation_axis_angle(
                &rng.unit_vector(),
                rng.range(0.0, RANDOM_DEGREES.to_radians()),
            ))
            .then_after(&Transform::translation(rng.unit_vector() * rng.range(0.0, RANDOM_MM)));
        // the pose of scan j in the frame of scan i, with the error in the frame of scan j
        let measured = truth(i).inverse().then_after(&truth(j)).then_after(&error);
        // virtual points: points of scan j, in its own frame
        let points = part.sample_points(150, &mut rng).into_iter().map(|(p, _)| truth(j).inverse().apply(&p)).collect();
        edges.push(Edge { i, j, measured, weight: 1.0, tolerance: 0.2, points, strong: true });
    }
    (edges, part)
}

/// The poses chained along 0, 1, ..., 7 (the spanning tree of a ring whose edges are equally strong).
fn chained(edges: &[Edge]) -> Vec<Transform> {
    let mut poses = vec![Transform::identity()];
    for e in edges.iter().take(SCANS - 1) {
        let last = poses.last().copied().unwrap_or_default();
        poses.push(last.then_after(&e.measured));
    }
    poses
}

fn far_error(poses: &[Transform], part: &Mesh, k: usize) -> f64 {
    poses[k].max_displacement(&truth(k), &part.positions)
}

#[test]
fn relaxation_spreads_the_loop_error_the_chain_piles_up() {
    let mut worst_factor = f64::INFINITY;
    for seed in 0..5 {
        let (edges, part) = ring_edges(seed);
        let chain = chained(&edges);
        let relaxed = relax(SCANS, 0, &edges, &chain, &Cancel::new()).unwrap();
        let (chain_err, relaxed_err) =
            (far_error(&chain, &part, SCANS - 1), far_error(&relaxed.poses, &part, SCANS - 1));
        let worst = |poses: &[Transform]| edges.iter().map(|e| misfit(e, poses).rms_mm).fold(0.0, f64::max);
        let (worst_c, worst_r) = (worst(&chain), worst(&relaxed.poses));
        let factor = chain_err / relaxed_err.max(1e-12);
        eprintln!(
            "seed {seed}: far scan chained {chain_err:.4} mm, relaxed {relaxed_err:.4} mm, factor {factor:.1}; worst edge misfit {worst_c:.4} -> {worst_r:.4} mm; {} iterations, converged {}",
            relaxed.iterations, relaxed.converged
        );
        assert!(relaxed.converged);
        assert!(relaxed_err * REQUIRED_FACTOR <= chain_err, "seed {seed}: chained {chain_err}, relaxed {relaxed_err}");
        assert!(worst_r < worst_c, "seed {seed}: the worst edge misfit did not shrink");
        worst_factor = worst_factor.min(factor);
    }
    eprintln!("smallest factor over five seeds: {worst_factor:.1} (required {REQUIRED_FACTOR})");
}

#[test]
fn the_fixed_scan_and_the_scans_without_an_edge_stay_where_they_are() {
    let (edges, _) = ring_edges(7);
    let kept: Vec<Edge> = edges.into_iter().filter(|e| e.i < 3 && e.j < 3).collect();
    let init: Vec<Transform> = (0..5).map(|k| Transform::translation(Vec3::new(k as f64, 0.0, 0.0))).collect();
    let r = relax(5, 0, &kept, &init, &Cancel::new()).unwrap();
    assert_eq!(r.poses[0], init[0]);
    assert_eq!(r.poses[3], init[3]);
    assert_eq!(r.poses[4], init[4]);
}

#[test]
fn a_cancelled_relaxation_is_an_error() {
    let (edges, _) = ring_edges(1);
    let cancel = Cancel::new();
    cancel.cancel();
    assert!(relax(SCANS, 0, &edges, &chained(&edges), &cancel).is_err());
}
