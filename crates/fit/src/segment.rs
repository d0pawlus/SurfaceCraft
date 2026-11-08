//! Segmentation of a mesh into regions that each follow one primitive ("auto segment").
//!
//! Fit-and-grow, in the spirit of Efficient RANSAC (Schnabel et al. 2007) but deterministic and
//! driven by mesh connectivity: seeds are taken from the flattest unassigned triangles; a seed
//! patch (its smooth neighbourhood) gets an automatic primitive fit; the region then grows
//! across neighbouring triangles whose corners lie within the tolerance of the primitive and
//! whose normal agrees with it, refitting as it doubles. Regions below `min_triangles` are
//! released. Everything is mesh-side (design doc §5).
use crate::auto::fit_auto;
use crate::primitive::{Primitive, PrimitiveKind};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use surfacecraft_geom::{Mesh, Point3, Vec3};

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
    let mut seen = vec![false; mesh.vertex_count()];
    let mut p = Vec::new();
    let mut n = Vec::new();
    for &t in tris {
        let Some(tri) = mesh.triangles.get(t as usize) else { continue };
        for &vi in tri {
            if let Some(s) = seen.get_mut(vi as usize)
                && !*s
            {
                *s = true;
                if let (Some(pp), Some(nn)) = (mesh.positions.get(vi as usize), normals.get(vi as usize)) {
                    p.push(*pp);
                    n.push(*nn);
                }
            }
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

fn accepts(mesh: &Mesh, prim: &Primitive, t: usize, tol: f64, cos: f64) -> bool {
    let Some(corners) = mesh.triangle(t) else { return false };
    if corners.iter().any(|c| prim.distance(c).abs() > tol) {
        return false;
    }
    let (Some(n), Some(c)) = (mesh.face_normal(t), mesh.face_centroid(t)) else { return false };
    prim.normal(&c).dot(&n).abs() >= cos
}

/// Segments `mesh` into primitive regions.
#[must_use]
pub fn segment(mesh: &Mesh, opts: &SegmentOptions) -> Vec<Segment> {
    let nt = mesh.triangle_count();
    if nt == 0 {
        return Vec::new();
    }
    let tol = if opts.tolerance.is_finite() && opts.tolerance > 0.0 { opts.tolerance } else { 0.2 };
    let cos = opts.max_angle_deg.clamp(1.0, 89.0).to_radians().cos();
    let nb = mesh.triangle_neighbours();
    let normals = mesh.vertex_normals();
    // flatness of each triangle: smallest = flattest neighbourhood (best seeds)
    let mut order: Vec<(f64, u32)> = (0..nt)
        .map(|t| {
            let n = mesh.face_normal(t).unwrap_or_else(Vec3::z);
            let worst = nb
                .get(t)
                .into_iter()
                .flatten()
                .filter(|&&o| o != u32::MAX)
                .filter_map(|&o| mesh.face_normal(o as usize))
                .map(|m| 1.0 - m.dot(&n))
                .fold(0.0, f64::max);
            (worst, t as u32)
        })
        .collect();
    order.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut owner = vec![u32::MAX; nt];
    let mut out: Vec<Segment> = Vec::new();
    let mut tried = vec![false; nt];
    let max_regions = 10_000;
    for &(_, seed) in &order {
        if out.len() >= max_regions {
            break;
        }
        let s = seed as usize;
        if owner.get(s) != Some(&u32::MAX) || tried.get(s).copied().unwrap_or(true) {
            continue;
        }
        // seed patch: smooth unassigned neighbourhood, breadth first
        let mut patch = vec![seed];
        let mut mark = vec![false; nt];
        mark[s] = true;
        let mut q = VecDeque::from([seed]);
        while let Some(t) = q.pop_front() {
            if patch.len() >= opts.seed_size.max(8) {
                break;
            }
            let Some(n0) = mesh.face_normal(t as usize) else { continue };
            for &o in nb.get(t as usize).into_iter().flatten() {
                let oi = o as usize;
                if o == u32::MAX || mark.get(oi).copied().unwrap_or(true) || owner.get(oi) != Some(&u32::MAX) {
                    continue;
                }
                if mesh.face_normal(oi).is_some_and(|m| m.dot(&n0) > 0.94) {
                    mark[oi] = true;
                    patch.push(o);
                    q.push_back(o);
                }
            }
        }
        for &t in &patch {
            if let Some(x) = tried.get_mut(t as usize) {
                *x = true;
            }
        }
        if patch.len() < 8 {
            continue;
        }
        let (p, n) = region_points(mesh, &normals, &patch);
        let Some(mut fit) = fit_auto(&p, &n).map(|a| a.best) else { continue };
        if fit.rms > tol {
            continue;
        }
        // grow, refitting each time the region doubles
        let id = out.len() as u32;
        let mut region: Vec<u32> = Vec::new();
        let mut inr = vec![false; nt];
        let mut q: VecDeque<u32> = VecDeque::new();
        for &t in &patch {
            if accepts(mesh, &fit.primitive, t as usize, tol, cos) {
                inr[t as usize] = true;
                region.push(t);
                q.push_back(t);
            }
        }
        let mut next_refit = region.len().max(8) * 2;
        while let Some(t) = q.pop_front() {
            for &o in nb.get(t as usize).into_iter().flatten() {
                let oi = o as usize;
                if o == u32::MAX || inr.get(oi).copied().unwrap_or(true) || owner.get(oi) != Some(&u32::MAX) {
                    continue;
                }
                if accepts(mesh, &fit.primitive, oi, tol, cos) {
                    inr[oi] = true;
                    region.push(o);
                    q.push_back(o);
                }
            }
            if region.len() >= next_refit {
                next_refit *= 2;
                let (p, n) = region_points(mesh, &normals, &region);
                if let Some(a) = fit_auto(&p, &n)
                    && a.best.rms <= tol
                {
                    fit = a.best;
                }
            }
        }
        if region.len() < opts.min_triangles {
            continue;
        }
        let (p, n) = region_points(mesh, &normals, &region);
        if let Some(a) = fit_auto(&p, &n)
            && a.best.rms <= tol
        {
            fit = a.best;
        }
        for &t in &region {
            if let Some(o) = owner.get_mut(t as usize) {
                *o = id;
            }
        }
        region.sort_unstable();
        out.push(Segment { triangles: region, primitive: fit.primitive, rms: fit.rms });
    }
    out
}

/// How many segments of each kind.
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

    #[test]
    fn smooth_growth_stops_at_sharp_edges() {
        let m = synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 10.0), 4);
        let face = grow_smooth(&m, &[0], 10.0);
        assert_eq!(face.len(), 4 * 4 * 2);
        assert!(segment(&Mesh::default(), &SegmentOptions::default()).is_empty());
    }
}
