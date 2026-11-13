//! Unit tests of the freeform consolidation on hand-made regions.
use super::Segment;
use super::freeform::{Settings, consolidate, refit_piece};
use crate::primitive::{Primitive, PrimitiveKind};
use surfacecraft_geom::{Mesh, Point3, Vec3, synth};

const NOISE: f64 = 0.03;

fn settings(noise: f64) -> Settings {
    Settings {
        strong_tris: 1000,
        dominant_tris: 1_000_000,
        scene: super::protect::Scene { area: 0.0, diagonal: 0.0 },
        protect_noise: 0.0,
        min_cluster: 5,
        min_triangles: 10,
        noise,
        noise_known: true,
        tolerance: 0.2,
        cos_crease: 40f64.to_radians().cos(),
    }
}

fn plane_region(tris: Vec<u32>, rms: f64) -> Segment {
    Segment {
        triangles: tris,
        primitive: Primitive::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] },
        rms,
        freeform: false,
    }
}

/// A region the fit calls a sphere of radius 300 (it spans a small part of it).
fn patch_region(tris: Vec<u32>, rms: f64) -> Segment {
    Segment {
        triangles: tris,
        primitive: Primitive::Sphere { center: [0.0, 0.0, 300.0], radius: 300.0 },
        rms,
        freeform: false,
    }
}

/// A grid over `[0, w] x [0, h]` with vertices `f(x, y)`.
fn sheet(w: f64, h: f64, nx: usize, ny: usize, f: impl Fn(f64, f64) -> Point3) -> Mesh {
    let cols = nx + 1;
    let mut positions = Vec::new();
    for j in 0..=ny {
        for i in 0..=nx {
            positions.push(f(w * i as f64 / nx as f64, h * j as f64 / ny as f64));
        }
    }
    let mut triangles = Vec::new();
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

fn centroid(m: &Mesh, t: u32) -> Point3 {
    m.face_centroid(t as usize).unwrap_or_else(Point3::origin)
}

fn run(m: &Mesh, segs: Vec<Segment>, set: &Settings) -> Vec<Segment> {
    consolidate(m, &m.vertex_normals(), &m.triangle_neighbours(), segs, set)
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Flat for 12 < x < 28, smooth bumps on both sides (tangent where they meet).
fn flat_then_wavy() -> Mesh {
    sheet(40.0, 40.0, 40, 40, |x, y| {
        let z = 2.5 * (0.35 * x).sin() * (0.3 * y + 1.0).sin() * smoothstep(((x - 20.0).abs() - 8.0) / 6.0);
        Point3::new(x, y, z)
    })
}

/// (a) Three large genuine primitives fit at the noise level, ten tiny curved fragments fit far
/// tighter: the scanner noise is a property of the scan, not of these fits, so the genuine plane
/// stays a primitive and is not absorbed as an island of the fragments' area.
#[test]
fn a_genuine_plane_next_to_tightly_fitting_fragments_stays() {
    let m = flat_then_wavy();
    let (flat, rest): (Vec<u32>, Vec<u32>) =
        (0..m.triangle_count() as u32).partition(|&t| (12.0..28.0).contains(&centroid(&m, t).x));
    let mut segs = vec![plane_region(flat.clone(), NOISE)];
    segs.extend(rest.chunks(rest.len().div_ceil(10)).map(|c| patch_region(c.to_vec(), 0.003)));
    let out = run(&m, segs, &settings(NOISE));
    let plane = out.iter().find(|s| s.primitive.kind() == PrimitiveKind::Plane && !s.freeform);
    assert!(plane.is_some_and(|p| p.triangles == flat), "the genuine plane was consumed: {} regions", out.len());
    assert!(out.iter().any(|s| s.freeform), "the fragments were not consolidated");
}

/// (b) An organic scan whose local misfits all fit at 0.02 mm: that is model error, not noise
/// (the scan's noise is 0.005), so the small planar fragments are fragments and the large planar
/// patch among them is an island of their area.
#[test]
fn fragments_whose_misfit_is_small_but_above_the_noise_are_consolidated() {
    let m = sheet(40.0, 40.0, 40, 40, |x, y| Point3::new(x, y, 2.5 * (0.35 * x).sin() * (0.3 * y + 1.0).sin()));
    let nt = m.triangle_count() as u32;
    let large: Vec<u32> = (0..nt).filter(|&t| centroid(&m, t).x < 12.0).collect();
    let rest: Vec<u32> = (0..nt).filter(|&t| centroid(&m, t).x >= 12.0 && centroid(&m, t).y < 30.0).collect();
    let mut segs = vec![plane_region(large, 0.02)];
    segs.extend(rest.chunks(rest.len().div_ceil(8)).map(|c| plane_region(c.to_vec(), 0.02)));
    let out = run(&m, segs, &settings(0.005));
    let freeform: usize = out.iter().filter(|s| s.freeform).map(|s| s.triangles.len()).sum();
    assert!(
        freeform >= 9 * nt as usize / 10 * 8 / 10,
        "only {freeform} of {nt} triangles were consolidated: {} regions",
        out.len()
    );
    assert!(out.iter().all(|s| s.freeform || s.triangles.len() < 100), "a plane region survived");
}

/// A floor (x 0 to 10, z = 0) and a wall (x = 10, z 0 to 10) that meet at a 90 degree crease.
fn folded_sheet() -> Mesh {
    let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 10.0, 10.0, 10, 10);
    m.append(&synth::plane(Point3::new(10.0, 0.0, 0.0), Vec3::z(), Vec3::y(), 10.0, 10.0, 10, 10));
    m.weld(1e-9);
    m
}

fn on_wall(m: &Mesh, t: u32) -> bool {
    centroid(m, t).x > 10.0 - 1e-6
}

fn tris_where(m: &Mesh, keep: impl Fn(Point3) -> bool) -> Vec<u32> {
    (0..m.triangle_count() as u32).filter(|&t| keep(centroid(m, t))).collect()
}

/// Is `tris` one connected piece (across shared edges)?
fn is_connected(m: &Mesh, tris: &[u32]) -> bool {
    let nb = m.triangle_neighbours();
    let set: std::collections::HashSet<u32> = tris.iter().copied().collect();
    let mut seen = std::collections::HashSet::new();
    let mut stack: Vec<u32> = tris.first().copied().into_iter().collect();
    while let Some(t) = stack.pop() {
        if seen.insert(t) {
            stack.extend(nb[t as usize].iter().copied().filter(|o| set.contains(o) && !seen.contains(o)));
        }
    }
    seen.len() == tris.len()
}

/// A weak region that straddles a sharp crease is split there: the part on the floor, where five
/// more fragments make an organic area, joins it; the part on the wall (a piece with only this
/// fragment) stays a region, with a fit of its own (a plane through x = 10), not the floor's.
#[test]
fn a_weak_region_across_a_crease_is_split_and_the_rest_is_refitted() {
    let m = folded_sheet();
    let floor = tris_where(&m, |c| c.z < 1e-6);
    let strip = tris_where(&m, |c| (c.z < 1e-6 && c.x > 7.0) || (c.x > 10.0 - 1e-6 && c.z < 3.0));
    let rest: Vec<u32> = floor.iter().copied().filter(|t| !strip.contains(t)).collect();
    let mut segs = vec![plane_region(strip.clone(), 0.1)];
    segs.extend(rest.chunks(rest.len().div_ceil(5)).map(|c| plane_region(c.to_vec(), 0.1)));
    let out = run(&m, segs, &settings(NOISE));
    let mut seen = std::collections::HashSet::new();
    assert!(out.iter().all(|s| !s.triangles.is_empty()));
    assert!(out.iter().flat_map(|s| s.triangles.iter()).all(|t| seen.insert(*t)), "a triangle is in two regions");
    for s in &out {
        let wall = s.triangles.iter().filter(|&&t| on_wall(&m, t)).count();
        assert!(wall == 0 || wall == s.triangles.len(), "a region across the crease");
    }
    let wall_part: Vec<u32> = strip.iter().copied().filter(|&t| on_wall(&m, t)).collect();
    let wall = out.iter().find(|s| s.triangles == wall_part).expect("the wall part of the strip stays one region");
    let n = match &wall.primitive {
        Primitive::Plane { normal, .. } => Vec3::new(normal[0], normal[1], normal[2]),
        other => panic!("not a plane: {other:?}"),
    };
    assert!(n.x.abs() > 0.99, "the wall part still carries the floor's plane: normal {n:?}");
    assert!(wall.rms <= NOISE, "stale RMS {}", wall.rms);
    assert!(out.iter().any(|s| floor.iter().all(|t| s.triangles.contains(t))), "the floor is one area");
}

/// A tray: floor z = 0 (x < 15), steps up to an organic middle (16 < x < 44), steps down to a
/// floor z = 2 (x > 45); 1 mm cells.
fn tray() -> Mesh {
    sheet(60.0, 20.0, 60, 20, |x, y| {
        let z = if x < 15.0 {
            0.0
        } else if x < 16.0 {
            5.0 * (x - 15.0)
        } else if x <= 44.0 {
            5.0 + 1.5 * (0.5 * x).sin() * (0.45 * y + 1.0).sin()
        } else if x < 45.0 {
            5.0 - 3.0 * (x - 44.0)
        } else {
            2.0
        };
        Point3::new(x, y, z)
    })
}

/// One fragment through the whole tray along its first rows, cut in two by the organic middle
/// that five more fragments make; the two floors it leaves are two regions with their own fits.
fn tray_regions(m: &Mesh, first_row: f64) -> Vec<Segment> {
    let band =
        tris_where(m, |c| (first_row..4.0).contains(&c.y) && (c.x < 14.5 || (16.5..43.5).contains(&c.x) || c.x > 45.5));
    let middle = tris_where(m, |c| c.y >= 4.0 && (16.5..43.5).contains(&c.x));
    let mut segs = vec![plane_region(band, 0.1)];
    segs.extend(middle.chunks(middle.len().div_ceil(5)).map(|c| plane_region(c.to_vec(), 0.1)));
    segs
}

fn z_of_plane(p: &Primitive, at: Point3) -> f64 {
    // the height of the plane above `at`'s x, y
    match p {
        Primitive::Plane { point, normal } if normal[2].abs() > 1e-9 => {
            point[2] - (normal[0] * (at.x - point[0]) + normal[1] * (at.y - point[1])) / normal[2]
        }
        _ => f64::NAN,
    }
}

#[test]
fn a_region_cut_in_two_by_an_organic_middle_yields_two_regions_with_their_own_fits() {
    let m = tray();
    let out = run(&m, tray_regions(&m, 0.0), &settings(NOISE));
    let floor_of = |keep: fn(Point3) -> bool| tris_where(&m, keep);
    let left = floor_of(|c| c.y < 4.0 && c.x < 14.5);
    let right = floor_of(|c| c.y < 4.0 && c.x > 45.5);
    let find = |tris: &[u32]| {
        out.iter().find(|s| s.triangles == tris).map(|s| z_of_plane(&s.primitive, Point3::new(5.0, 2.0, 0.0)))
    };
    let (zl, zr) = (find(&left), find(&right));
    assert_eq!(zl.map(|z| z.round()), Some(0.0), "the left floor is a region of its own, at z = 0: {zl:?}");
    assert_eq!(zr.map(|z| z.round()), Some(2.0), "the right floor is a region of its own, at z = 2: {zr:?}");
}

/// No region is disconnected or smaller than `min_triangles`; a remnant that small is released.
#[test]
fn no_undersized_or_disconnected_region_is_returned() {
    let m = tray();
    let mut segs = tray_regions(&m, 1.0);
    // a second fragment whose left remnant is 4 triangles (two cells of the first row)
    let tiny = tris_where(&m, |c| c.y < 1.0 && (c.x < 2.0 || (30.0..40.0).contains(&c.x)));
    let tiny_left: Vec<u32> = tiny.iter().copied().filter(|&t| centroid(&m, t).x < 2.0).collect();
    segs.push(plane_region(tiny, 0.1));
    let set = settings(NOISE);
    let out = run(&m, segs, &set);
    for s in &out {
        assert!(s.triangles.len() >= set.min_triangles, "an undersized region of {} triangles", s.triangles.len());
        assert!(is_connected(&m, &s.triangles), "a disconnected region of {} triangles", s.triangles.len());
    }
    let held: std::collections::HashSet<u32> = out.iter().flat_map(|s| s.triangles.iter().copied()).collect();
    assert!(tiny_left.iter().all(|t| !held.contains(t)), "the undersized remnant is released");
}

#[test]
fn consolidation_does_not_depend_on_the_thread_count() {
    let m = tray();
    let run_on = |threads: usize| {
        let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().expect("thread pool");
        pool.install(|| run(&m, tray_regions(&m, 0.0), &settings(NOISE)))
    };
    assert_eq!(run_on(1), run_on(6));
}

/// Five fragments that each cross the crease have their first triangles on the floor, but they
/// are fragments on the wall as well: both sides qualify as organic areas (the rule: a fragment
/// counts once in every smooth piece it occupies). Both areas are one plane, so each is
/// returned as that plane's region, and nothing is left over.
#[test]
fn a_fragment_counts_in_every_piece_it_occupies() {
    let m = folded_sheet();
    let segs: Vec<Segment> = (0..5)
        .map(|k| {
            let (lo, hi) = (2.0 * k as f64, 2.0 * k as f64 + 2.0);
            let band = tris_where(&m, |c| {
                (lo..hi).contains(&c.y) && ((c.z < 1e-6 && c.x > 3.0) || (c.x > 10.0 - 1e-6 && c.z < 7.0))
            });
            plane_region(band, 0.1)
        })
        .collect();
    let out = run(&m, segs, &settings(NOISE));
    let floor = tris_where(&m, |c| c.z < 1e-6);
    let wall = tris_where(&m, |c| c.x > 10.0 - 1e-6);
    assert_eq!(out.len(), 2, "one region per side: {:?}", out.iter().map(|s| s.triangles.len()).collect::<Vec<_>>());
    assert!(out.iter().any(|s| s.triangles == floor) && out.iter().any(|s| s.triangles == wall));
}

/// Without a known noise (the fallback) no large region is demoted, however it fits: here a
/// large plane that fits badly, with fragments all around, stays.
#[test]
fn without_a_known_noise_a_large_region_is_never_demoted() {
    let m = flat_then_wavy();
    let (flat, rest): (Vec<u32>, Vec<u32>) =
        (0..m.triangle_count() as u32).partition(|&t| (12.0..28.0).contains(&centroid(&m, t).x));
    let mut segs = vec![plane_region(flat.clone(), 0.15)];
    segs.extend(rest.chunks(rest.len().div_ceil(10)).map(|c| patch_region(c.to_vec(), 0.3)));
    let set = Settings { noise: 0.05, noise_known: false, ..settings(NOISE) };
    let out = run(&m, segs, &set);
    assert!(out.iter().any(|s| s.triangles == flat && !s.freeform), "the large region was demoted");
}

/// A cylindrical strip (R = 42 mm, 10 mm wide, 60 mm long) with noise 0.185 mm: against the
/// tolerance 0.2 the plane fits at about 0.205 and the cylinder at about 0.185.
fn strip_between_plane_and_cylinder() -> Mesh {
    const R: f64 = 42.0;
    let mut m = sheet(10.0, 60.0, 20, 120, |x, y| {
        let x = x - 5.0;
        Point3::new(x, y, R - (R * R - x * x).sqrt())
    });
    synth::add_noise(&mut m, 0.185, &mut surfacecraft_geom::Rng::new(9));
    m
}

/// `fit_auto` prefers the simpler plane when it is within 20 % of the best RMS; the plane at
/// 0.205 misses the tolerance 0.2 while the cylinder at 0.185 meets it. A cut piece must not be
/// released then: a primitive fits it.
#[test]
fn a_piece_is_kept_when_only_the_model_auto_does_not_prefer_fits_the_tolerance() {
    let m = strip_between_plane_and_cylinder();
    let tris: Vec<u32> = (0..m.triangle_count() as u32).collect();
    let normals = m.vertex_normals();
    let set = Settings { tolerance: 0.2, ..settings(NOISE) };
    let auto = super::fits::fit_tris(&m, &normals, &tris).expect("a fit");
    eprintln!("auto: {:?} rms {:.4}", auto.primitive.kind(), auto.rms);
    assert!(
        auto.rms > 0.2 && auto.primitive.kind() == PrimitiveKind::Plane,
        "the scenario: auto picks a plane above the tolerance"
    );
    let piece = refit_piece(&m, &normals, &tris, PrimitiveKind::Plane, &set)
        .expect("the piece is released although the cylinder fits");
    assert_eq!(piece.primitive.kind(), PrimitiveKind::Cylinder);
    assert!(piece.rms <= 0.2);
}
