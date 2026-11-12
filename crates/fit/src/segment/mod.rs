//! Segmentation of a mesh into regions that each follow one primitive ("auto segment").
//!
//! Fit-and-grow, in the spirit of Efficient RANSAC (Schnabel et al. 2007) but deterministic and
//! driven by mesh connectivity: seeds are taken from the flattest unassigned triangles; a seed
//! patch (its smooth neighbourhood) gets an automatic primitive fit; the region then grows
//! across neighbouring triangles whose corners lie within the tolerance of the primitive and
//! whose normal agrees with it, refitting as it doubles. Regions below `min_triangles` are
//! released. Everything is mesh-side (design doc §5).
mod fits;
mod grow;
mod merge;
mod relax;
mod seeds;

use crate::primitive::{Primitive, PrimitiveKind};
use fits::fit_tris;
use grow::Ctx;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use surfacecraft_geom::{Cancel, Mesh, Point3, Vec3};

/// Segmentation settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default)]
pub struct SegmentOptions {
    /// Distance tolerance of a triangle corner to its region's primitive (mm).
    pub tolerance: f64,
    /// Largest angle between a triangle normal and the primitive normal (degrees).
    pub max_angle_deg: f64,
    /// Smallest region kept (triangles).
    pub min_triangles: usize,
    /// Size of the seed patch (triangles).
    pub seed_size: usize,
}

impl Default for SegmentOptions {
    fn default() -> Self {
        Self { tolerance: 0.2, max_angle_deg: 20.0, min_triangles: 30, seed_size: 60 }
    }
}

/// One region found.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Segment {
    /// Triangle indices, sorted.
    pub triangles: Vec<u32>,
    /// Primitive of the region.
    pub primitive: Primitive,
    /// RMS of the region's vertices to it (mm).
    pub rms: f64,
}

/// Vertices and their normals of a set of triangles.
#[must_use]
pub fn region_points(mesh: &Mesh, normals: &[Vec3], tris: &[u32]) -> (Vec<Point3>, Vec<Vec3>) {
    // sort + dedup the corner ids: cost proportional to the region, not to the whole mesh (seeds
    // call this thousands of times on scans of millions of vertices)
    let mut ids: Vec<u32> =
        tris.iter().filter_map(|&t| mesh.triangles.get(t as usize)).flat_map(|t| t.iter().copied()).collect();
    surfacecraft_geom::par::sort_unstable(&mut ids);
    ids.dedup();
    let mut p = Vec::with_capacity(ids.len());
    let mut n = Vec::with_capacity(ids.len());
    for vi in ids {
        if let (Some(pp), Some(nn)) = (mesh.positions.get(vi as usize), normals.get(vi as usize)) {
            p.push(*pp);
            n.push(*nn);
        }
    }
    (p, n)
}

/// Grows a set of triangles from `seeds` across edges whose dihedral angle is below
/// `max_angle_deg` ("select smooth region").
#[must_use]
pub fn grow_smooth(mesh: &Mesh, seeds: &[u32], max_angle_deg: f64) -> Vec<u32> {
    let nb = mesh.triangle_neighbours();
    let cos = max_angle_deg.to_radians().cos();
    let mut inside = vec![false; mesh.triangle_count()];
    let mut q: VecDeque<u32> = VecDeque::new();
    for &s in seeds {
        if let Some(f) = inside.get_mut(s as usize)
            && !*f
        {
            *f = true;
            q.push_back(s);
        }
    }
    while let Some(t) = q.pop_front() {
        let Some(nt) = mesh.face_normal(t as usize) else { continue };
        for &o in nb.get(t as usize).into_iter().flatten() {
            if o == u32::MAX || inside.get(o as usize).copied().unwrap_or(true) {
                continue;
            }
            if mesh.face_normal(o as usize).is_some_and(|no| no.dot(&nt) >= cos) {
                if let Some(f) = inside.get_mut(o as usize) {
                    *f = true;
                }
                q.push_back(o);
            }
        }
    }
    inside.iter().enumerate().filter(|(_, f)| **f).map(|(i, _)| i as u32).collect()
}
/// Vertex normals averaged over their one ring twice, only across neighbours within 35° (so
/// scanner noise on dense meshes stops breaking regions up, while sharp edges stay sharp), and
/// per-triangle normals from them.
fn smoothed_normals(mesh: &Mesh) -> (Vec<Vec3>, Vec<Option<Vec3>>) {
    let mut n = mesh.vertex_normals();
    let cos = 35f64.to_radians().cos();
    for _ in 0..2 {
        let mut acc = n.clone();
        for t in &mesh.triangles {
            for k in 0..3 {
                let (a, b) = (t[k] as usize, t[(k + 1) % 3] as usize);
                let (Some(na), Some(nb)) = (n.get(a).copied(), n.get(b).copied()) else { continue };
                if na.dot(&nb) >= cos {
                    if let Some(x) = acc.get_mut(a) {
                        *x += nb;
                    }
                    if let Some(x) = acc.get_mut(b) {
                        *x += na;
                    }
                }
            }
        }
        n = acc.into_iter().map(|v| surfacecraft_geom::normalize(v).unwrap_or_else(Vec3::zeros)).collect();
    }
    let f = surfacecraft_geom::par::map(mesh.triangle_count(), |t| {
        let tri = mesh.triangles.get(t)?;
        let raw = mesh.face_normal(t)?;
        let s: Vec3 = tri.iter().filter_map(|&v| n.get(v as usize)).sum();
        // keep the orientation of the triangle itself
        surfacecraft_geom::normalize(s).map(|v| if v.dot(&raw) < 0.0 { raw } else { v })
    });
    (n, f)
}
/// Tolerance used when the options carry none that makes sense (mm).
const FALLBACK_TOLERANCE: f64 = 0.2;
/// Fewest triangles of a seed patch: also the smallest seed size.
const MIN_SEED_SIZE: usize = 8;

/// Seeds in the order they are tried: `(flatness key, triangle)`, flattest neighbourhood first
/// (those are the best seeds).
fn seed_order(nb: &[[u32; 3]], fnorm: &[Option<Vec3>]) -> Vec<(u64, u32)> {
    let mut order: Vec<(u64, u32)> = surfacecraft_geom::par::map(nb.len(), |t| {
        let n = fnorm[t].unwrap_or_else(Vec3::z);
        let worst = nb[t]
            .iter()
            .filter(|&&o| o != u32::MAX)
            .filter_map(|&o| fnorm.get(o as usize).copied().flatten())
            .map(|m| 1.0 - m.dot(&n))
            .fold(0.0, f64::max);
        // non-negative floats order like their bits
        (worst.max(0.0).to_bits(), t as u32)
    });
    surfacecraft_geom::par::sort_unstable(&mut order);
    order
}

/// Area of a seed patch: `seed_tris` triangles of the median area.
fn seed_patch_area(areas: &[f64], seed_tris: usize) -> f64 {
    let mut a: Vec<f64> = areas.iter().copied().filter(|x| *x > 0.0).collect();
    let k = a.len() / 2;
    let med = if a.is_empty() { 0.0 } else { *a.select_nth_unstable_by(k, f64::total_cmp).1 };
    med * seed_tris as f64
}

/// Segments `mesh` into primitive regions.
///
/// Robust to real scans (P1): seed patches are grown to a fixed *area* (the median triangle area
/// times `seed_size`), so dense and sparse parts of a scan get seeds of the same physical size;
/// fits use at most [`fits::FIT_POINTS`] points; adjacent regions whose union is still one
/// primitive of the same kind within the tolerance are merged (a face cut by a hole or a seam
/// comes out whole).
///
/// The result does not depend on the number of threads: regions are found as if one seed after the
/// other (see `seeds`), and every later step is independent per region or per triangle.
#[must_use]
pub fn segment(mesh: &Mesh, opts: &SegmentOptions) -> Vec<Segment> {
    segment_with_stats(mesh, opts).0
}

/// [`segment`] that gives up when `cancel` is set: `None` then, checked between the seeds of the
/// region growth and between the later phases, so a cancelled run ends within a moment.
#[must_use]
pub fn segment_cancellable(mesh: &Mesh, opts: &SegmentOptions, cancel: &Cancel) -> Option<Vec<Segment>> {
    run(mesh, opts, cancel).map(|(segs, _)| segs)
}

/// Work of one [`segment`] run that does not show in its result.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SegmentStats {
    /// Region jobs run ahead of their turn.
    pub speculative_jobs: usize,
    /// Speculative jobs whose result was used (the rest was discarded or recomputed).
    pub speculative_used: usize,
}

/// [`segment`] and what the run did besides (per run, unlike the global counters of `stats`).
#[must_use]
pub fn segment_with_stats(mesh: &Mesh, opts: &SegmentOptions) -> (Vec<Segment>, SegmentStats) {
    run(mesh, opts, &Cancel::new()).unwrap_or_default()
}

/// The segmentation; `None` when `cancel` was set before it finished.
fn run(mesh: &Mesh, opts: &SegmentOptions, cancel: &Cancel) -> Option<(Vec<Segment>, SegmentStats)> {
    let nt = mesh.triangle_count();
    if nt == 0 || mesh.validate().is_err() {
        return Some((Vec::new(), SegmentStats::default()));
    }
    let tol = if opts.tolerance.is_finite() && opts.tolerance > 0.0 { opts.tolerance } else { FALLBACK_TOLERANCE };
    let cos = opts.max_angle_deg.clamp(1.0, 89.0).to_radians().cos();
    let sw = crate::stats::Stopwatch::start();
    let nb = mesh.triangle_neighbours();
    let (normals, fnorm) = smoothed_normals(mesh);
    let areas: Vec<f64> = surfacecraft_geom::par::map(nt, |t| mesh.face_area(t));
    let seed_tris = opts.seed_size.max(MIN_SEED_SIZE);
    let order = seed_order(&nb, &fnorm);
    sw.stop(crate::stats::Phase::Prep);
    let ctx = Ctx {
        mesh,
        normals: &normals,
        nb: &nb,
        fnorm: &fnorm,
        areas: &areas,
        seed_tris,
        seed_area: seed_patch_area(&areas, seed_tris),
        tol,
        cos,
    };
    let sw = crate::stats::Stopwatch::start();
    let (found, stats) = seeds::grow_regions(&ctx, &order, opts.min_triangles, cancel);
    sw.stop(crate::stats::Phase::Regions);
    if cancel.is_cancelled() {
        return None;
    }
    let out = final_fits(mesh, &normals, found, tol);
    let sw = crate::stats::Stopwatch::start();
    let out = relax::relax_boundaries(mesh, &normals, &nb, out, tol, cancel);
    sw.stop(crate::stats::Phase::Relax);
    if cancel.is_cancelled() {
        return None;
    }
    let sw = crate::stats::Stopwatch::start();
    let merged = merge::merge_adjacent(mesh, &normals, &nb, out, tol);
    sw.stop(crate::stats::Phase::Merge);
    (!cancel.is_cancelled()).then_some((merged, stats))
}

/// The automatic fit of every region on its final triangles (the growth fit saw fewer); one that
/// is within the tolerance replaces the growth fit. Independent per region, so it runs in
/// parallel; it feeds nothing the growth reads, so it can wait until growth is done.
fn final_fits(mesh: &Mesh, normals: &[Vec3], mut segs: Vec<Segment>, tol: f64) -> Vec<Segment> {
    let sw = crate::stats::Stopwatch::start();
    let fits = surfacecraft_geom::par::map_slice(&segs, |s| {
        crate::stats::final_fit();
        fit_tris(mesh, normals, &s.triangles)
    });
    for (s, f) in segs.iter_mut().zip(fits) {
        if let Some(f) = f
            && f.rms <= tol
        {
            s.primitive = f.primitive;
            s.rms = f.rms;
        }
    }
    sw.stop(crate::stats::Phase::Final);
    segs
}

// How many segments of each kind.
#[must_use]
pub fn kinds(segs: &[Segment]) -> Vec<(PrimitiveKind, usize)> {
    let mut v: Vec<(PrimitiveKind, usize)> = Vec::new();
    for s in segs {
        let k = s.primitive.kind();
        match v.iter_mut().find(|(kk, _)| *kk == k) {
            Some(e) => e.1 += 1,
            None => v.push((k, 1)),
        }
    }
    v.sort();
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    /// Gate M2 (Region): the bracket (6 planar faces, a cylindrical boss, a spherical dome)
    /// with σ = 0.05 mm noise segments into the right kinds, each region within RMS 0.1 mm.
    #[test]
    fn gate_m2_auto_segment_bracket() {
        let mut m = synth::bracket();
        synth::add_noise(&mut m, 0.05, &mut Rng::new(4));
        let segs = segment(&m, &SegmentOptions::default());
        let k = kinds(&segs);
        eprintln!(
            "gate_m2 segment: {k:?}, rms {:?}",
            segs.iter().map(|s| (s.rms * 1e4).round() / 1e4).collect::<Vec<_>>()
        );
        assert_eq!(k, vec![(PrimitiveKind::Plane, 6), (PrimitiveKind::Sphere, 1), (PrimitiveKind::Cylinder, 1)]);
        assert!(segs.iter().all(|s| s.rms <= 0.1));
        // nearly every triangle is assigned
        let assigned: usize = segs.iter().map(|s| s.triangles.len()).sum();
        assert!(assigned as f64 >= 0.9 * m.triangle_count() as f64, "{assigned} of {}", m.triangle_count());
    }

    /// An annular plane (a disc with a hole) is one plane region, not a chain of fragments
    /// (merging runs to a fixed point).
    #[test]
    fn annular_plane_is_one_region() {
        let full = synth::plane(Point3::new(-12.0, -12.0, 0.0), Vec3::x(), Vec3::y(), 24.0, 24.0, 160, 160);
        let keep: Vec<u32> = (0..full.triangle_count())
            .filter(|&i| {
                full.face_centroid(i).is_some_and(|c| {
                    let r = c.x.hypot(c.y);
                    (4.0..=12.0).contains(&r)
                })
            })
            .map(|i| i as u32)
            .collect();
        let mut m = full.submesh(&keep);
        synth::add_noise(&mut m, 0.01, &mut Rng::new(5));
        let segs = segment(&m, &SegmentOptions::default());
        let planes: Vec<usize> =
            segs.iter().filter(|s| s.primitive.kind() == PrimitiveKind::Plane).map(|s| s.triangles.len()).collect();
        let total = m.triangle_count();
        assert!(
            planes.iter().copied().max().unwrap_or(0) as f64 >= 0.95 * total as f64,
            "annulus fragmented: {planes:?} of {total}"
        );
    }

    #[test]
    fn smooth_growth_stops_at_sharp_edges() {
        let m = synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 10.0), 4);
        let face = grow_smooth(&m, &[0], 10.0);
        assert_eq!(face.len(), 4 * 4 * 2);
        assert!(segment(&Mesh::default(), &SegmentOptions::default()).is_empty());
    }

    /// A smooth selection on a noisy scan stops at a sharp edge: the thin triangles a scan has,
    /// whose normals noise turns by tens of degrees, must not carry it from the top of a box
    /// down its walls.
    #[test]
    fn grow_smooth_does_not_leak_over_a_sharp_edge_of_a_noisy_scan() {
        // 80 divisions a side: the wall triangles are 0.5 mm long and 0.15 mm high
        let mut cad = synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 12.0), 80);
        cad.weld(1e-9);
        let mut leaks = Vec::new();
        for seed in 1..=24 {
            let mut scan = cad.clone();
            synth::add_noise(&mut scan, 0.03, &mut Rng::new(seed));
            let top = (0..scan.triangle_count())
                .find(|&t| {
                    scan.face_centroid(t)
                        .is_some_and(|c| c.z > 11.9 && (c.x - 20.0).abs() < 2.0 && (c.y - 15.0).abs() < 2.0)
                })
                .unwrap();
            let region = grow_smooth(&scan, &[top as u32], 25.0);
            let below = region.iter().filter(|&&t| scan.face_centroid(t as usize).is_some_and(|c| c.z < 11.0)).count();
            let on_top =
                (0..scan.triangle_count()).filter(|&t| scan.face_centroid(t).is_some_and(|c| c.z > 11.9)).count();
            if below > 0 || region.len() * 10 < on_top * 9 {
                leaks.push((seed, region.len(), below, on_top));
            }
        }
        assert!(leaks.is_empty(), "(seed, region, below the top, top triangles): {leaks:?}");
    }
}
