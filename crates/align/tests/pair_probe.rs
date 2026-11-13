//! The probes of `pair_coarse` through its public API: a pose whose displaced starts cannot be
//! refined is never `unique`, and says how many failed.
use surfacecraft_align::{PairOptions, PairVerdict, pair_coarse};
use surfacecraft_geom::{Cancel, Mesh, Point3, Rng, Transform, Vec3, synth};

fn scan(block: usize, seed: u64) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), block);
    m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
    m.append(&synth::sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 32, 16));
    synth::add_noise(&mut m, 0.05, &mut Rng::new(seed));
    m
}

#[test]
fn a_pose_with_failed_probes_is_not_unique() {
    let fixed = scan(12, 1);
    let moving = scan(15, 2).transformed(&Transform::rotation_axis_angle(&Vec3::new(0.2, 0.9, 0.1), 2.0));
    let run = |probe_step| {
        let opts = PairOptions { probe_step, ..PairOptions::default() };
        pair_coarse(&moving, &fixed, &opts, &Cancel::new()).unwrap()
    };
    let normal = run(None);
    assert_eq!(normal.verdict, PairVerdict::Unique, "{normal:?}");
    assert_eq!((normal.probes_failed, normal.probed), (0, 6));
    // starts displaced fifty times the extent of the scan have no pairs: nothing was learnt
    let far = run(Some(50.0));
    assert_eq!(far.probed, 6);
    assert_eq!(far.probes_failed, 6, "{far:?}");
    assert_eq!(far.verdict, PairVerdict::Ambiguous);
    assert!(far.confidence <= 0.25);
}
