//! `pair_coarse` gives the same answer on one thread and on many: every sample, trial and sum is
//! seeded or ordered, nothing depends on how the work is spread.
use surfacecraft_align::{PairOptions, PairResult, pair_coarse};
use surfacecraft_geom::{Cancel, Mesh, Point3, Rng, Transform, Vec3, synth};

fn scan(block: usize, seed: u64) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), block);
    m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
    m.append(&synth::sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 32, 16));
    synth::add_noise(&mut m, 0.05, &mut Rng::new(seed));
    m
}

fn run_on(threads: usize, moving: &Mesh, fixed: &Mesh) -> PairResult {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
    pool.install(|| pair_coarse(moving, fixed, &PairOptions::default(), &Cancel::new()).unwrap())
}

fn bits(r: &PairResult) -> Vec<u64> {
    let mut v: Vec<u64> = r.transform.to_rows().iter().flatten().map(|x| x.to_bits()).collect();
    v.extend([r.confidence, r.tolerance, r.overlap_moving, r.overlap_fixed, r.rms].map(f64::to_bits));
    v.push(r.candidates as u64);
    v.extend([r.stability, r.probe_noise, r.probe_threshold, r.probe_slide_clearance].map(f64::to_bits));
    v.extend([r.probed, r.probes_failed, r.slides, r.refined].map(|n| n as u64));
    v.push(r.icp_iterations as u64);
    v
}

#[test]
fn the_pair_alignment_is_identical_on_one_thread_and_many() {
    let fixed = scan(12, 1);
    let pose = Transform::rotation_axis_angle(&Vec3::new(0.3, -0.8, 0.5), 2.4)
        .then_after(&Transform::translation(Vec3::new(90.0, -40.0, 30.0)));
    let moving = scan(15, 2).transformed(&pose);
    let one = run_on(1, &moving, &fixed);
    let many = run_on(6, &moving, &fixed);
    assert_eq!(bits(&one), bits(&many), "{one:?} vs {many:?}");
    assert_eq!(one.verdict, many.verdict);
    // and it is the right answer, not a constant
    let err = one.transform.max_displacement(&pose.inverse(), &moving.positions);
    assert!(err < 0.2, "{err}");
}
