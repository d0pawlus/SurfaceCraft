#![cfg(not(target_arch = "wasm32"))]
//! Segmentation of organic scans into a few large freeform regions (P9-O7): the fragments a
//! bumpy surface falls into become freeform regions, primitives next to them are still found, and
//! the result does not depend on the number of threads.
use std::time::Instant;
use surfacecraft_fit::PrimitiveKind;
use surfacecraft_fit::segment::{Segment, SegmentOptions, kinds, segment, segment_with_stats};
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
    let mesh = organic_blob(25_000, 20.0, 1.2, 0.7, 0.02);
    let plain = segment(&mesh, &SegmentOptions { freeform: false, ..SegmentOptions::default() });
    let merged = segment(&mesh, &SegmentOptions::default());
    eprintln!("medium blob: {} fragments, {} regions with freeform", plain.len(), merged.len());
    assert!(plain.len() >= 50 && plain.iter().all(|s| !s.freeform), "{} fragments", plain.len());
    assert!(merged.len() <= 50, "{} regions", merged.len());
    assert!(merged.iter().any(|s| s.freeform), "no freeform region");
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

/// The result is the same on one thread and on many (regions, flags, bits of the stand-ins), with
/// speculative jobs run, used and discarded alike (the seeds the halo skips included).
#[test]
fn freeform_segmentation_is_identical_on_one_thread_and_many() {
    let mesh = organic_blob(25_000, 20.0, 1.2, 0.7, 0.02);
    let opts = SegmentOptions::default();
    let run = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("thread pool");
        pool.install(|| segment_with_stats(&mesh, &opts))
    };
    let (one, one_stats) = run(1);
    let (many, many_stats) = run(6);
    eprintln!("speculation on 6 threads: {many_stats:?}");
    assert_eq!(one_stats.speculative_jobs, 0);
    assert!(many_stats.speculative_used > 0, "{many_stats:?}");
    assert!(many_stats.speculative_used < many_stats.speculative_jobs, "{many_stats:?}");
    assert!(one.iter().any(|s| s.freeform));
    assert_eq!(one, many);
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

// --- primitives that must survive next to, or among, freeform areas (review of PR 82) -------------

/// A grid mesh of `nx` x `ny` cells over `[0, w] x [0, h]`, the vertex of `(x, y)` being `f(x, y)`.
fn sheet(w: f64, h: f64, nx: usize, ny: usize, f: impl Fn(f64, f64) -> Point3) -> Mesh {
    let cols = nx + 1;
    let mut positions = Vec::with_capacity(cols * (ny + 1));
    for j in 0..=ny {
        for i in 0..=nx {
            positions.push(f(w * i as f64 / nx as f64, h * j as f64 / ny as f64));
        }
    }
    let mut triangles = Vec::with_capacity(2 * nx * ny);
    for j in 0..ny {
        for i in 0..nx {
            let (a, b) = ((j * cols + i) as u32, (j * cols + i + 1) as u32);
            let (c, d) = (((j + 1) * cols + i + 1) as u32, ((j + 1) * cols + i) as u32);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { positions, triangles }
}

/// Smooth bumps of a few mm (wavelengths 18 to 21 mm).
fn bump(x: f64, y: f64) -> f64 {
    3.0 * (0.35 * x + 0.3).sin() * (0.3 * y + 1.1).sin()
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

fn centroid(m: &Mesh, t: u32) -> Point3 {
    m.face_centroid(t as usize).unwrap_or_else(Point3::origin)
}

/// A large shallow cylindrical panel (R = 1000 mm, 400 x 600 mm, sigma 0.03 mm) that continues
/// into a small organic patch (100 mm of bumps, tangent to it) stays ONE cylinder region.
#[test]
fn shallow_cylindrical_panel_next_to_an_organic_patch_stays_one_cylinder() {
    const R: f64 = 1000.0;
    // rows every 2.5 mm over the panel, every 1 mm over the patch
    let mut mesh = sheet(400.0, 1.0, 160, 340, |x, v| {
        let y = if v < 240.0 / 340.0 { v * 340.0 * 2.5 } else { 600.0 + (v * 340.0 - 240.0) };
        let theta = (x - 200.0) / R;
        let r = R + 1.5 * (0.14 * x + 0.3).sin() * (0.12 * y + 1.1).sin() * smoothstep((y - 600.0) / 15.0);
        Point3::new(r * theta.sin(), y, r * theta.cos() - R)
    });
    synth::add_noise(&mut mesh, 0.03, &mut Rng::new(3));
    let panel: Vec<u32> = (0..mesh.triangle_count() as u32).filter(|&t| centroid(&mesh, t).y < 585.0).collect();
    let segs = segment(&mesh, &SegmentOptions::default());
    let report: Vec<_> = segs.iter().map(|s| (s.primitive.kind(), s.freeform, s.triangles.len())).collect();
    eprintln!("panel {} of {} triangles: {report:?}", panel.len(), mesh.triangle_count());
    let panel_cylinders: Vec<&Segment> = segs
        .iter()
        .filter(|s| !s.freeform && s.primitive.kind() == PrimitiveKind::Cylinder)
        .filter(|s| s.triangles.iter().filter(|t| panel.binary_search(t).is_ok()).count() * 2 > panel.len())
        .collect();
    assert_eq!(panel_cylinders.len(), 1, "the panel is one cylinder region: {report:?}");
    let held = panel_cylinders[0].triangles.iter().filter(|t| panel.binary_search(t).is_ok()).count();
    assert!(held as f64 >= 0.95 * panel.len() as f64, "{held} of {} panel triangles", panel.len());
    let lost: usize = segs
        .iter()
        .filter(|s| s.freeform)
        .map(|s| s.triangles.iter().filter(|t| panel.binary_search(t).is_ok()).count())
        .sum();
    assert!(lost * 50 <= panel.len(), "{lost} panel triangles went freeform");
}

/// A planar mounting face (r = 25 mm) that a sculpted surface runs into tangentially stays a
/// plane: it borders the sculpted area at every edge without a crease, but its own fit is at the
/// noise level and large. The sculpted surround is freeform.
#[test]
fn planar_mounting_face_in_a_tangent_sculpted_surface_stays_a_plane() {
    let mut mesh = sheet(120.0, 120.0, 150, 150, |x, y| {
        let r = (x - 60.0).hypot(y - 60.0);
        Point3::new(x, y, bump(x, y) * smoothstep((r - 25.0) / 10.0))
    });
    synth::add_noise(&mut mesh, 0.03, &mut Rng::new(5));
    let near = |t: u32, lo: f64, hi: f64| {
        let c = centroid(&mesh, t);
        (lo..hi).contains(&(c.x - 60.0).hypot(c.y - 60.0))
    };
    let face: Vec<u32> = (0..mesh.triangle_count() as u32).filter(|&t| near(t, 0.0, 22.0)).collect();
    let surround: Vec<u32> = (0..mesh.triangle_count() as u32).filter(|&t| near(t, 40.0, 55.0)).collect();
    let segs = segment(&mesh, &SegmentOptions::default());
    let held = |s: &Segment, of: &[u32]| s.triangles.iter().filter(|t| of.binary_search(t).is_ok()).count();
    let report: Vec<_> = segs.iter().map(|s| (s.primitive.kind(), s.freeform, s.triangles.len())).collect();
    eprintln!("mounting face {} triangles, surround {}: {report:?}", face.len(), surround.len());
    let plane = segs
        .iter()
        .find(|s| !s.freeform && s.primitive.kind() == PrimitiveKind::Plane && held(s, &face) * 2 > face.len());
    assert!(
        plane.is_some_and(|p| held(p, &face) as f64 >= 0.95 * face.len() as f64),
        "the face is a plane: {report:?}"
    );
    let free: usize = segs.iter().filter(|s| s.freeform).map(|s| held(s, &surround)).sum();
    assert!(free as f64 >= 0.8 * surround.len() as f64, "{free} of {} surround triangles freeform", surround.len());
}

/// A corrugated plate: six cylinders (R = 6 mm, 8 mm wide, 20 mm long, ~500 triangles each: small)
/// that alternate hill and valley and meet tangentially. Every one is a genuine primitive
/// that fits at the noise level; none of them is a fragment of something else.
fn corrugated_plate() -> Mesh {
    const R: f64 = 6.0;
    const HALF_ANGLE: f64 = 0.7;
    const STRIPS: usize = 6;
    let arc = 2.0 * HALF_ANGLE * R;
    let (nx, ny) = ((STRIPS as f64 * arc / 0.8) as usize, 25);
    sheet(STRIPS as f64 * arc, 20.0, nx, ny, |s, y| {
        let strip = ((s / arc) as usize).min(STRIPS - 1);
        let u = s - strip as f64 * arc;
        // direction angle along the profile: turns by 1/R per mm, to the left on a hill, to the right in a valley
        let sign = if strip.is_multiple_of(2) { -1.0 } else { 1.0 };
        let phi0 = -sign * HALF_ANGLE;
        // end of the previous strips: x advances by 2 R sin(half angle) per strip, z returns to 0 per pair
        let chord = 2.0 * R * HALF_ANGLE.sin();
        let (xa, za) = (strip as f64 * chord, 0.0);
        let phi = phi0 + sign * u / R;
        let x = xa + R * sign * (phi.sin() - phi0.sin());
        let z = za - R * sign * (phi.cos() - phi0.cos());
        // strips alternate, so a valley starts where the hill ended: both at height 0 at the junctions
        Point3::new(x, y, z)
    })
}

#[test]
fn corrugated_plate_keeps_its_six_cylinders() {
    let mut mesh = corrugated_plate();
    synth::add_noise(&mut mesh, 0.03, &mut Rng::new(7));
    let plain = segment(&mesh, &SegmentOptions { freeform: false, ..SegmentOptions::default() });
    let segs = segment(&mesh, &SegmentOptions::default());
    let report: Vec<_> = segs.iter().map(|s| (s.primitive.kind(), s.freeform, s.triangles.len())).collect();
    let plain_report: Vec<_> =
        plain.iter().map(|s| (s.primitive.kind(), s.freeform, s.triangles.len(), (s.rms * 1e3).round())).collect();
    eprintln!("corrugated plate {} triangles: {report:?}\nplain {plain_report:?}", mesh.triangle_count());
    let cylinders = |segs: &[Segment]| {
        segs.iter().filter(|s| s.primitive.kind() == PrimitiveKind::Cylinder && s.triangles.len() >= 400).count()
    };
    assert_eq!(cylinders(&plain), 6, "the six strips are cylinders: {plain_report:?}");
    assert_eq!(cylinders(&segs), 6, "{report:?}");
    assert!(segs.iter().all(|s| !s.freeform), "{report:?}");
    assert_eq!(segs, plain, "nothing was consolidated");
}

/// With a noise above the tolerance no fit can be judged against it: nothing is consolidated, the
/// result is the one of `freeform: false`.
#[test]
fn noise_above_the_tolerance_consolidates_nothing() {
    let mesh = organic_blob(25_000, 20.0, 1.2, 0.7, 0.02);
    let noisy = segment(&mesh, &SegmentOptions { noise: Some(0.5), ..SegmentOptions::default() });
    let plain = segment(&mesh, &SegmentOptions { freeform: false, ..SegmentOptions::default() });
    assert_eq!(noisy, plain);
    assert!(noisy.iter().all(|s| !s.freeform));
}
