#![cfg(not(target_arch = "wasm32"))]
//! Segmentation of organic scans into a few large freeform regions (P9-O7): the fragments a
//! bumpy surface falls into become freeform regions, primitives next to them are still found, and
//! the result does not depend on the number of threads.
use std::time::Instant;
use surfacecraft_fit::PrimitiveKind;
use surfacecraft_fit::segment::{Segment, SegmentOptions, kinds, segment};
use surfacecraft_geom::{Mesh, Point3, Rng, synth};

/// A bumpy closed sphere of about `triangles` triangles with scanner noise `noise` (mm): radius,
/// bump amplitude (mm) and a scale on the wave numbers of the three sine factors.
fn organic_blob(triangles: usize, radius: f64, bump: f64, wave_scale: f64, noise: f64) -> Mesh {
    let nv = ((triangles as f64 / 4.0).sqrt().round() as usize).max(4);
    let mut m = synth::sphere(Point3::origin(), radius, 2 * nv, nv);
    let [wx, wy, wz] = [37.0, 29.0, 23.0].map(|w| w * wave_scale);
    for p in &mut m.positions {
        let d = p.coords / radius;
        *p = Point3::from(
            d * (radius + bump * (wx * d.x + 0.3).sin() * (wy * d.y + 1.1).sin() * (wz * d.z + 2.0).sin()),
        );
    }
    synth::add_noise(&mut m, noise, &mut Rng::new(11));
    m
}

/// The blob of the gate: R = 100 mm, bumps of 2.5 mm, σ = 0.03 mm, about 150 k triangles.
fn gate_blob() -> Mesh {
    organic_blob(150_000, 100.0, 2.5, 1.0, 0.03)
}

fn assigned(segs: &[Segment]) -> usize {
    segs.iter().map(|s| s.triangles.len()).sum()
}

fn freeform_triangles(segs: &[Segment]) -> usize {
    segs.iter().filter(|s| s.freeform).map(|s| s.triangles.len()).sum()
}

/// Gate 1: a closed organic blob (it shattered into hundreds of slivers with 30 % of the
/// triangles unassigned) comes out as a few regions, almost every triangle assigned, nearly all
/// of them in regions flagged freeform.
#[test]
fn organic_blob_is_a_few_freeform_regions() {
    let mesh = gate_blob();
    let t = Instant::now();
    let segs = segment(&mesh, &SegmentOptions::default());
    let share = assigned(&segs) as f64 / mesh.triangle_count() as f64;
    eprintln!(
        "blob {} tris: {} regions ({} freeform), {:.1} % assigned, {:.2} s",
        mesh.triangle_count(),
        segs.len(),
        segs.iter().filter(|s| s.freeform).count(),
        100.0 * share,
        t.elapsed().as_secs_f64()
    );
    assert!(segs.len() <= 50, "{} regions", segs.len());
    assert!(share >= 0.95, "{:.1} % assigned", 100.0 * share);
    assert!(freeform_triangles(&segs) as f64 >= 0.9 * assigned(&segs) as f64, "freeform regions hold too little");
    assert!(segs.iter().all(|s| s.triangles.windows(2).all(|w| w[0] < w[1])), "triangles are sorted and unique");
}

/// Without the consolidation the same kind of surface still ends in fragments (what the option
/// is for), with it in a few regions.
#[test]
fn freeform_false_keeps_the_fragments() {
    let mesh = organic_blob(25_000, 20.0, 0.5, 0.7, 0.02);
    let plain = segment(&mesh, &SegmentOptions { freeform: false, ..SegmentOptions::default() });
    let merged = segment(&mesh, &SegmentOptions::default());
    eprintln!("medium blob: {} fragments, {} regions with freeform", plain.len(), merged.len());
    assert!(plain.len() >= 100 && plain.iter().all(|s| !s.freeform), "{} fragments", plain.len());
    assert!(merged.len() <= 50, "{} regions", merged.len());
    assert!(assigned(&merged) as f64 >= 0.95 * mesh.triangle_count() as f64);
}

/// The bracket (planes, a cylinder, a sphere, all sharp-edged) is not freeform: the same regions
/// as without the consolidation.
#[test]
fn prismatic_part_keeps_its_primitives() {
    let mut m = synth::bracket();
    synth::add_noise(&mut m, 0.05, &mut Rng::new(4));
    let on = segment(&m, &SegmentOptions::default());
    let off = segment(&m, &SegmentOptions { freeform: false, ..SegmentOptions::default() });
    assert_eq!(on, off);
    assert_eq!(kinds(&on), vec![(PrimitiveKind::Plane, 6), (PrimitiveKind::Sphere, 1), (PrimitiveKind::Cylinder, 1)]);
}

/// The result is the same on one thread and on many (regions, flags, bits of the stand-ins).
#[test]
fn freeform_segmentation_is_identical_on_one_thread_and_many() {
    let mesh = organic_blob(25_000, 20.0, 0.5, 0.7, 0.02);
    let opts = SegmentOptions::default();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("thread pool");
        pool.install(|| segment(&mesh, &opts))
    };
    let one = run(1);
    assert!(one.iter().any(|s| s.freeform));
    assert_eq!(one, run(6));
}

/// Options that make no sense fall back to the defaults instead of failing.
#[test]
fn hostile_freeform_options_do_not_panic() {
    let mesh = organic_blob(4_000, 20.0, 0.5, 0.7, 0.02);
    for (share, crease) in [(f64::NAN, f64::NAN), (-1.0, -5.0), (1e30, 1e30), (f64::INFINITY, f64::NEG_INFINITY)] {
        let o = SegmentOptions { freeform_min_share: share, freeform_crease_deg: crease, ..SegmentOptions::default() };
        let _ = segment(&mesh, &o);
    }
}

/// Before and after on the same scan, back to back: `SURFACECRAFT_PERF_SEG_KTRIS=500 cargo test
/// --profile ci -p surfacecraft-fit --test segment_freeform perf_ -- --ignored --nocapture`
/// (`freeform: false` is the segmentation as it was before the consolidation).
#[test]
#[ignore = "minutes at 500 k triangles; run explicitly"]
fn perf_freeform_against_fragments() {
    let ktris: usize = std::env::var("SURFACECRAFT_PERF_SEG_KTRIS").ok().and_then(|v| v.parse().ok()).unwrap_or(150);
    let mesh = organic_blob(ktris * 1000, 100.0, 2.5, 1.0, 0.03);
    for (name, freeform) in [("before (freeform: false)", false), ("after", true)] {
        surfacecraft_fit::stats::reset();
        let t = Instant::now();
        let segs = segment(&mesh, &SegmentOptions { freeform, ..SegmentOptions::default() });
        let w = surfacecraft_fit::stats::snapshot();
        eprintln!(
            "perf {name}: {} tris, {} regions, {:.1} % assigned, {:.1} s, {} seed fits, {} LM runs",
            mesh.triangle_count(),
            segs.len(),
            100.0 * assigned(&segs) as f64 / mesh.triangle_count() as f64,
            t.elapsed().as_secs_f64(),
            w.seed_fits,
            w.lm_runs
        );
    }
}
