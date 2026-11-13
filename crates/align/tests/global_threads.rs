//! `align_global` gives the same answer on one thread and on many: every pair, sample and sum is
//! seeded or ordered, and nothing depends on how the work is spread.
use surfacecraft_align::{GlobalOptions, GlobalResult, align_global};
use surfacecraft_geom::{Cancel, Mesh, Point3, Rng, Transform, Vec3, synth};

fn scan(block: usize, seed: u64) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), block);
    m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
    m.append(&synth::sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 32, 16));
    synth::add_noise(&mut m, 0.05, &mut Rng::new(seed));
    m
}

fn run_on(threads: usize, scans: &[&Mesh]) -> GlobalResult {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();
    pool.install(|| align_global(scans, &GlobalOptions::default(), &Cancel::new()).unwrap())
}

fn bits(r: &GlobalResult) -> Vec<u64> {
    let mut v: Vec<u64> = Vec::new();
    for s in &r.scans {
        v.extend(s.pose.to_rows().iter().flatten().map(|x| x.to_bits()));
        v.extend(s.chained.to_rows().iter().flatten().map(|x| x.to_bits()));
    }
    for e in &r.edges {
        v.extend(e.transform.to_rows().iter().flatten().map(|x| x.to_bits()));
        v.extend([e.rms, e.tolerance, e.confidence, e.overlap_moving, e.overlap_fixed].map(f64::to_bits));
        v.extend(e.misfit_after.iter().flat_map(|m| [m.rms_mm.to_bits(), m.angle_degrees.to_bits()]));
    }
    v.push(r.iterations as u64);
    v.extend([r.loops.before.worst_mm, r.loops.after.worst_mm, r.loops.worst_edge_ratio].map(f64::to_bits));
    v
}

#[test]
fn the_global_alignment_is_identical_on_one_thread_and_many() {
    let poses = [
        Transform::identity(),
        Transform::rotation_axis_angle(&Vec3::new(0.3, -0.8, 0.5), 2.4)
            .then_after(&Transform::translation(Vec3::new(90.0, -40.0, 30.0))),
        Transform::rotation_axis_angle(&Vec3::new(-0.6, 0.1, 0.9), 1.1)
            .then_after(&Transform::translation(Vec3::new(-70.0, 20.0, 60.0))),
    ];
    let meshes: Vec<Mesh> = (0..3).map(|k| scan(12 + 3 * k, 1 + k as u64).transformed(&poses[k])).collect();
    let refs: Vec<&Mesh> = meshes.iter().collect();
    let one = run_on(1, &refs);
    let many = run_on(6, &refs);
    assert_eq!(bits(&one), bits(&many));
    assert_eq!(one.verdict, many.verdict);
    // and it is the right answer, not a constant
    let truth = poses[0].then_after(&poses[2].inverse());
    let err = one.scans[2].pose.max_displacement(&truth, &meshes[2].positions);
    assert!(err < 0.3, "{err}");
}
