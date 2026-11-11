#![cfg(not(target_arch = "wasm32"))]
//! Segmentation of organic scans: the result does not depend on the thread count, and (ignored,
//! `perf_segment_organic`) how long it takes at scan scale.
//!
//! `SURFACECRAFT_PERF_SEG_KTRIS=500 cargo test --release -p surfacecraft-fit --test segment_parallel -- --ignored --nocapture`
//! picks the size in thousands of triangles (default 500); `SURFACECRAFT_PERF_SEG_THREADS=1,8`
//! lists the thread counts to compare (default: 1 and every core).
use std::time::Instant;
use surfacecraft_fit::segment::{Segment, SegmentOptions, segment};
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

/// A bumpy closed sphere-like shape: radius and bump amplitude (mm), and a scale on the wave
/// numbers of the three sine factors over the unit sphere.
#[derive(Clone, Copy)]
struct Shape {
    radius: f64,
    bump: f64,
    wave_scale: f64,
}

/// The shape of the scale measurement (R = 100 mm): at 0.5 M triangles it ends in thousands of
/// regions.
const LARGE: Shape = Shape { radius: 100.0, bump: 2.5, wave_scale: 1.0 };
/// A medium shape (the equality test and the reference digests): 25 k triangles end in a couple of hundred regions.
const MEDIUM: Shape = Shape { radius: 20.0, bump: 0.5, wave_scale: 0.7 };
/// Wave numbers of the three bump factors.
const WAVES: [f64; 3] = [37.0, 29.0, 23.0];
/// Scanner noise along the normal (mm).
const NOISE: f64 = 0.02;

/// A closed bumpy shape of about `triangles` triangles with scanner noise: the surface is never
/// one primitive for long, so segmentation ends in many small regions.
fn organic_scan(triangles: usize, shape: Shape) -> Mesh {
    // a UV sphere has 2 · nu · nv triangles with nu = 2 nv
    let nv = ((triangles as f64 / 4.0).sqrt().round() as usize).max(4);
    let mut m = synth::sphere(Point3::origin(), shape.radius, 2 * nv, nv);
    let [wx, wy, wz] = WAVES.map(|w| w * shape.wave_scale);
    for p in &mut m.positions {
        let d = p.coords / shape.radius;
        let bump = shape.bump * (wx * d.x + 0.3).sin() * (wy * d.y + 1.1).sin() * (wz * d.z + 2.0).sin();
        *p = Point3::from(d * (shape.radius + bump));
    }
    synth::add_noise(&mut m, NOISE, &mut Rng::new(11));
    m
}

/// Runs `segment` on a pool of `threads` threads.
fn segment_on(threads: usize, mesh: &Mesh, opts: &SegmentOptions) -> Vec<Segment> {
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("thread pool");
    pool.install(|| segment(mesh, opts))
}

/// One digest of everything a segmentation returns (triangles, primitive parameters, RMS bits).
fn digest(segs: &[Segment]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    let mut eat = |x: u64| h = (h ^ x).wrapping_mul(0x0000_0100_0000_01b3);
    for s in segs {
        eat(s.triangles.len() as u64);
        s.triangles.iter().for_each(|&t| eat(u64::from(t)));
        eat(s.rms.to_bits());
        let json = serde_json::to_string(&s.primitive).unwrap_or_default();
        json.bytes().for_each(|b| eat(u64::from(b)));
    }
    h
}

#[test]
fn organic_segmentation_is_identical_on_one_thread_and_many() {
    let mesh = organic_scan(25_000, MEDIUM);
    let opts = SegmentOptions::default();
    let many = std::thread::available_parallelism().map_or(4, usize::from).max(4);
    let one = segment_on(1, &mesh, &opts);
    let par = segment_on(many, &mesh, &opts);
    assert!(one.len() >= 100, "too few regions to mean anything: {}", one.len());
    assert_eq!(one.len(), par.len());
    assert_eq!(digest(&one), digest(&par), "{} regions", one.len());
    assert_eq!(one, par);
}

#[test]
fn thread_count_does_not_change_a_closed_bracket_either() {
    let mut m = synth::bracket();
    synth::add_noise(&mut m, 0.05, &mut Rng::new(4));
    let opts = SegmentOptions::default();
    assert_eq!(segment_on(1, &m, &opts), segment_on(6, &m, &opts));
}

/// Digests of the reference scans, to compare two builds of the segmentation (bit-identical
/// output): `cargo test -p surfacecraft-fit --test segment_parallel print_reference_digests --
/// --ignored --nocapture`. Not asserted: float functions may differ between platforms.
#[test]
#[ignore = "prints digests for comparing builds"]
fn print_reference_digests() {
    let opts = SegmentOptions::default();
    let mut bracket = synth::bracket();
    synth::add_noise(&mut bracket, 0.05, &mut Rng::new(4));
    let full = synth::plane(Point3::new(-12.0, -12.0, 0.0), Vec3::x(), Vec3::y(), 24.0, 24.0, 160, 160);
    let keep: Vec<u32> = (0..full.triangle_count())
        .filter(|&i| full.face_centroid(i).is_some_and(|c| (4.0..=12.0).contains(&c.x.hypot(c.y))))
        .map(|i| i as u32)
        .collect();
    let mut annulus = full.submesh(&keep);
    synth::add_noise(&mut annulus, 0.01, &mut Rng::new(5));
    let mut block = synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 12.0), 60);
    synth::add_noise(&mut block, 0.03, &mut Rng::new(6));
    let scans =
        [("bracket", bracket), ("annulus", annulus), ("block", block), ("organic 25k", organic_scan(25_000, MEDIUM))];
    for (name, mesh) in scans {
        let segs = segment_on(2, &mesh, &opts);
        eprintln!("digest {name}: {:016x} ({} regions)", digest(&segs), segs.len());
    }
}

#[test]
#[ignore = "minutes of CPU at 1 M triangles; run explicitly in release"]
fn perf_segment_organic() {
    let ktris: usize = std::env::var("SURFACECRAFT_PERF_SEG_KTRIS").ok().and_then(|v| v.parse().ok()).unwrap_or(500);
    let cores = std::thread::available_parallelism().map_or(1, usize::from);
    let counts: Vec<usize> = std::env::var("SURFACECRAFT_PERF_SEG_THREADS")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![1, cores]);
    let mesh = organic_scan(ktris * 1000, LARGE);
    let mut digests = Vec::new();
    for threads in counts {
        surfacecraft_fit::stats::reset();
        let t = Instant::now();
        let segs = segment_on(threads, &mesh, &SegmentOptions::default());
        let secs = t.elapsed().as_secs_f64();
        let d = digest(&segs);
        let phases: Vec<String> =
            surfacecraft_fit::stats::phase_seconds().iter().map(|(n, s)| format!("{n} {s:.1}")).collect();
        eprintln!(
            "perf segment: {} tris, {threads} threads, {secs:.2} s, {} regions, digest {d:016x}; phases (s, summed stopwatch stretches): {}",
            mesh.triangle_count(),
            segs.len(),
            phases.join(", ")
        );
        let w = surfacecraft_fit::stats::snapshot();
        eprintln!(
            "perf segment work: {} seed fits, {} grow refits, {} final fits, {} speculative jobs ({} used)",
            w.seed_fits, w.grow_refits, w.final_fits, w.speculative_jobs, w.speculative_used
        );
        digests.push(d);
    }
    assert!(digests.windows(2).all(|w| w[0] == w[1]), "thread counts disagree: {digests:x?}");
}
