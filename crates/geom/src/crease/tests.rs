//! Tests of the sharpened hole patches: sharp edges recovered from synthetic meshes with holes cut in them.

use crate::edit::{boundary_loops, fill_holes, fill_holes_with};
use crate::{Mesh, Point3, Rng, Vec3, synth};

const SIDE: f64 = 20.0;

/// A cone side read with points of its neighbour (RMS 0.101 as a plane, 0.064 as a quadric)
/// is a quadric; a flat side whose quadric only follows the noise stays a plane.
#[test]
fn a_side_bends_when_the_quadric_gains_beyond_its_extra_terms() {
    assert!(super::bends(79, 0.1007, 0.0640));
    assert!(!super::bends(92, 0.0276, 0.0270));
    assert!(!super::bends(20, 0.0300, 0.0220));
}

/// Distance from `p` to the surface of the cube [0, SIDE]³.
fn to_cube(p: &Point3) -> f64 {
    let q = [
        (p.x - SIDE / 2.0).abs() - SIDE / 2.0,
        (p.y - SIDE / 2.0).abs() - SIDE / 2.0,
        (p.z - SIDE / 2.0).abs() - SIDE / 2.0,
    ];
    let outside = q.iter().map(|c| c.max(0.0).powi(2)).sum::<f64>().sqrt();
    let inside = q.iter().copied().fold(f64::NEG_INFINITY, f64::max).min(0.0);
    (outside + inside).abs()
}

/// The cube scan (1.25 mm triangles, noise `sigma`) with a ball of radius `r` around `at`
/// cut out and filled again; returns the mesh and the worst distance of a vertex and of a
/// triangle centroid from the cube.
fn cut_and_fill(at: Point3, r: f64, sigma: f64) -> (Mesh, f64, f64) {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(SIDE, SIDE, SIDE), 16);
    m.weld(1e-9);
    if sigma > 0.0 {
        synth::add_noise(&mut m, sigma, &mut Rng::new(5));
    }
    let keep: Vec<u32> = (0..m.triangle_count() as u32)
        .filter(|&t| m.face_centroid(t as usize).is_some_and(|c| (c - at).norm() > r))
        .collect();
    let mut cut = m.submesh(&keep);
    assert_eq!(boundary_loops(&cut).len(), 1);
    let rep = fill_holes(&mut cut, 1000, true);
    assert_eq!(rep.filled, 1);
    assert!(boundary_loops(&cut).is_empty());
    let thinnest = (0..cut.triangle_count()).map(|t| cut.face_area(t)).fold(f64::INFINITY, f64::min);
    assert!(thinnest > 1e-4, "a triangle of area {thinnest} mm² is left");
    let vertices = cut.positions.iter().map(to_cube).fold(0.0, f64::max);
    let centroids =
        (0..cut.triangle_count()).filter_map(|t| cut.face_centroid(t)).map(|c| to_cube(&c)).fold(0.0, f64::max);
    (cut, vertices, centroids)
}

#[test]
fn a_hole_over_a_sharp_edge_is_filled_with_the_edge() {
    let (_, vertices, centroids) = cut_and_fill(Point3::new(10.0, 0.0, SIDE), 3.0, 0.0);
    assert!(vertices < 1e-6, "a patch vertex is {vertices} mm off the cube");
    assert!(centroids < 0.05, "a patch triangle is {centroids} mm off the cube");
}

#[test]
fn a_hole_over_a_corner_is_filled_with_the_corner() {
    let (_, vertices, centroids) = cut_and_fill(Point3::new(0.0, 0.0, SIDE), 3.0, 0.0);
    assert!(vertices < 1e-6, "a patch vertex is {vertices} mm off the cube");
    assert!(centroids < 0.1, "a patch triangle is {centroids} mm off the cube");
}

#[test]
fn a_noisy_hole_over_a_sharp_edge_stays_within_the_noise() {
    let (_, vertices, _) = cut_and_fill(Point3::new(10.0, 0.0, SIDE), 3.0, 0.03);
    assert!(vertices < 0.12, "a vertex is {vertices} mm off the cube at σ = 0.03");
}

#[test]
fn the_fill_is_deterministic() {
    let (a, _, _) = cut_and_fill(Point3::new(10.0, 0.0, SIDE), 3.0, 0.03);
    let (b, _, _) = cut_and_fill(Point3::new(10.0, 0.0, SIDE), 3.0, 0.03);
    assert_eq!(a, b);
}

/// A hole in a face, near a corner but not over an edge, is none of the corner's business:
/// the patch stays in the face and no vertex is dragged to the corner.
#[test]
fn a_hole_near_a_corner_stays_in_its_face() {
    let (m, vertices, centroids) = cut_and_fill(Point3::new(3.0, 3.0, SIDE), 2.0, 0.0);
    assert!(vertices < 1e-9 && centroids < 1e-9, "{vertices} {centroids}");
    let top: f64 = (0..m.triangle_count())
        .filter(|&t| m.face_centroid(t).is_some_and(|c| c.z > SIDE - 1e-6))
        .map(|t| m.face_area(t))
        .sum();
    assert!((top - SIDE * SIDE).abs() < 1e-6, "the top has area {top}: the patch overlaps it");
}

/// A Z-shaped step (top, wall, floor), 1 mm triangles, as an open sheet over 20 mm in y.
fn step(height: f64, sigma: f64, seed: u64) -> Mesh {
    let profile = |s: f64| -> (f64, f64) {
        if s <= 10.0 {
            (s, height)
        } else if s <= 10.0 + height {
            (10.0, height - (s - 10.0))
        } else {
            (s - height, 0.0)
        }
    };
    let (ns, ny) = ((20.0 + height).round() as usize, 20usize);
    let mut m = Mesh::default();
    for i in 0..=ns {
        let (x, z) = profile(i as f64);
        for j in 0..=ny {
            m.positions.push(Point3::new(x, j as f64, z));
        }
    }
    let id = |i: usize, j: usize| (i * (ny + 1) + j) as u32;
    for i in 0..ns {
        for j in 0..ny {
            m.triangles.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
            m.triangles.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
        }
    }
    if sigma > 0.0 {
        synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    }
    m
}

/// Distance from `p` to the step's profile (top, wall and floor as segments in x, z).
fn to_step(p: &Point3, height: f64) -> f64 {
    let seg = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dz) = (b.0 - a.0, b.1 - a.1);
        let t = (((p.x - a.0) * dx + (p.z - a.1) * dz) / (dx * dx + dz * dz)).clamp(0.0, 1.0);
        (a.0 + t * dx - p.x).hypot(a.1 + t * dz - p.z)
    };
    seg((0.0, height), (10.0, height)).min(seg((10.0, height), (10.0, 0.0))).min(seg((10.0, 0.0), (20.0, 0.0)))
}

/// The top and the floor of a step have one normal but are two faces: a hole over the wall
/// must not be filled worse, or with slivers, because they were read as one tilted side.
#[test]
fn a_hole_over_a_step_is_not_made_worse() {
    for (height, r) in [(2.0, 3.0), (3.0, 5.0), (4.0, 4.0)] {
        for seed in 1..=10 {
            let m = step(height, 0.03, seed);
            let at = Point3::new(10.0, 10.0, height / 2.0);
            let keep: Vec<u32> = (0..m.triangle_count() as u32)
                .filter(|&t| m.face_centroid(t as usize).is_some_and(|c| (c - at).norm() > r))
                .collect();
            let cut = m.submesh(&keep);
            let worst = |mm: &Mesh| mm.positions.iter().map(|p| to_step(p, height)).fold(0.0, f64::max);
            let (mut faired, mut sharp) = (cut.clone(), cut.clone());
            // the sheet's own border is a longer loop than the hole and stays open
            fill_holes_with(&mut faired, 60, true, false);
            let rep = fill_holes_with(&mut sharp, 60, true, true);
            assert_eq!(rep.filled, 1);
            assert!(
                worst(&sharp) <= worst(&faired) + 1e-9,
                "step {height} hole {r} seed {seed}: {} mm, faired {} mm",
                worst(&sharp),
                worst(&faired)
            );
            let thinnest = |mm: &Mesh| (0..mm.triangle_count()).map(|t| mm.face_area(t)).fold(f64::INFINITY, f64::min);
            assert!(thinnest(&sharp) >= 0.5 * thinnest(&faired).min(0.02), "a sliver of {} mm²", thinnest(&sharp));
        }
    }
}

/// A hundred holes of 0.8 to 2.5 mm over the edges of an irregular scan of a cube (0.7 to
/// 1.4 mm triangles, σ = 0.03 mm): every patch stays within 0.12 mm of the cube, none is
/// worse than the faired one, and nine in ten are sharpened.
#[test]
fn holes_over_the_edges_of_a_noisy_scan_keep_the_edges() {
    let mut cad = synth::cuboid(Point3::origin(), Vec3::new(SIDE, SIDE, SIDE), 1);
    cad.weld(1e-9);
    let d = synth::ScanDefects { spacing: 0.7, noise: 0.03, holes: 0, spikes: 0.0, debris: 0, ..Default::default() };
    let (mut bad, mut worse, mut n) = (0, 0, 0);
    let mut reasons = std::collections::BTreeMap::new();
    for seed in 1..=20u64 {
        let scan = synth::scanify(&cad, &d, &mut Rng::new(seed));
        let mut rng = Rng::new(seed + 1000);
        for _ in 0..5 {
            // a point on one of the edges of the top face, away from the corners
            let t = rng.range(4.0, SIDE - 4.0);
            let at = match rng.index(4) {
                0 => Point3::new(t, 0.0, SIDE),
                1 => Point3::new(t, SIDE, SIDE),
                2 => Point3::new(0.0, t, SIDE),
                _ => Point3::new(SIDE, t, SIDE),
            };
            let at = at + Vec3::new(rng.range(-0.8, 0.8), rng.range(-0.8, 0.8), 0.0);
            let r = rng.range(0.8, 2.5);
            let keep: Vec<u32> = (0..scan.triangle_count() as u32)
                .filter(|&t| scan.face_centroid(t as usize).is_some_and(|c| (c - at).norm() > r))
                .collect();
            let cut = scan.submesh(&keep);
            if boundary_loops(&cut).len() != 1 {
                continue;
            }
            let first = cut.positions.len();
            let worst = |mm: &Mesh| mm.positions.iter().skip(first).map(to_cube).fold(0.0, f64::max);
            let (mut faired, mut sharp) = (cut.clone(), cut.clone());
            fill_holes_with(&mut faired, 1000, true, false);
            fill_holes_with(&mut sharp, 1000, true, true);
            n += 1;
            let why = super::LAST_REASON.with(std::cell::Cell::get).unwrap_or("sharpened");
            *reasons.entry(why).or_insert(0usize) += 1;
            if worst(&sharp) > 0.12 {
                bad += 1;
            }
            if worst(&sharp) > worst(&faired) + 1e-9 {
                worse += 1;
            }
        }
    }
    let sharpened = reasons.get("sharpened").copied().unwrap_or(0);
    assert!(n >= 90, "only {n} holes were cut");
    assert_eq!((bad, worse), (0, 0), "over 0.12 mm, worse than faired, of {n} holes: {reasons:?}");
    assert!(sharpened * 10 >= n * 9, "{sharpened} of {n} sharpened: {reasons:?}");
}
