//! The honesty checks of the merge, measured on the result: is every removed triangle covered by
//! the result (points sampled over the removed triangles by area, not only their corners), and does
//! any triangle the merge added (restored or built in the seam) cross another triangle?
use super::union::{Origin, Union};
use crate::{Aabb, Bvh, Cancel, Mesh, Point3, Vec3, nesting, par};

/// Points sampled over the removed triangles at most (beyond the one per triangle every triangle gets).
pub const MAX_REMOVED_SAMPLES: usize = 2_000_000;
/// Subdivisions of a removed triangle's sides at most.
const MAX_SPLIT: usize = 64;
/// Samples measured between two looks at the cancellation flag.
const CHUNK: usize = 1 << 15;
/// Triangles near an added one tested for a crossing at most (the rest counts as a crossing).
const MAX_NEAR: usize = 1 << 16;

/// How well the result covers what was removed.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Coverage {
    /// Points sampled over the removed triangles.
    pub samples: usize,
    /// Area removed (mm²).
    pub area: f64,
    /// Of it, the area whose samples lie within the tolerance of facing surface of the result (mm²).
    pub area_within: f64,
    /// Largest distance of a sample from the result (mm).
    pub worst: f64,
}

impl Coverage {
    /// Share of the removed area within the tolerance (1 when nothing was removed).
    #[must_use]
    pub fn share(&self) -> f64 {
        if self.area > 0.0 { self.area_within / self.area } else { 1.0 }
    }
}

/// Points over triangle `t`: the centres of the `k × k` subdivision of its sides, each standing for
/// an equal share of its area.
fn subdivide(t: &[Point3; 3], k: usize, out: &mut Vec<(Point3, f64)>) {
    let [a, b, c] = *t;
    let n = k as f64;
    let each = crate::tri::area(&a, &b, &c) / (n * n);
    let at = |u: f64, v: f64| a + (b - a) * (u / n) + (c - a) * (v / n);
    for i in 0..k {
        for j in 0..k - i {
            let (fi, fj) = (i as f64, j as f64);
            out.push((at(fi + 1.0 / 3.0, fj + 1.0 / 3.0), each));
            if i + j + 1 < k {
                out.push((at(fi + 2.0 / 3.0, fj + 2.0 / 3.0), each));
            }
        }
    }
}

/// Points sampled over the removed triangles by area (spacing at most the tolerance where the cap
/// allows), with the normal of their triangle.
fn samples(removed: &[[Point3; 3]], tol: f64) -> Vec<(Point3, Vec3, f64)> {
    let total: f64 = removed.iter().map(|[a, b, c]| crate::tri::area(a, b, c)).sum();
    let each = (tol * tol).max(total / MAX_REMOVED_SAMPLES as f64);
    let mut out = Vec::new();
    let mut buf = Vec::new();
    for t in removed {
        let Some(n) = crate::tri::normal(&t[0], &t[1], &t[2]) else { continue };
        let area = crate::tri::area(&t[0], &t[1], &t[2]);
        let k = if each > 0.0 { ((area / each).sqrt().ceil() as usize).clamp(1, MAX_SPLIT) } else { 1 };
        buf.clear();
        subdivide(t, k, &mut buf);
        out.extend(buf.iter().map(|&(p, w)| (p, n, w)));
    }
    out
}

/// Measures the points sampled over `removed` against `result`; `None` when cancelled.
pub(super) fn coverage(result: &Mesh, removed: &[[Point3; 3]], tol: f64, cancel: &Cancel) -> Option<Coverage> {
    let pts = samples(removed, tol);
    let bvh = Bvh::build(result);
    let mut cov = Coverage { samples: pts.len(), ..Coverage::default() };
    for chunk in pts.chunks(CHUNK) {
        if cancel.is_cancelled() {
            return None;
        }
        let d = par::map_slice(chunk, |(p, n, _)| {
            bvh.closest_facing(p, f64::INFINITY, &[*n], 0.0).map_or(f64::INFINITY, |h| h.distance)
        });
        for ((_, _, w), x) in chunk.iter().zip(d) {
            cov.area += w;
            if x <= tol {
                cov.area_within += w;
            }
            cov.worst = cov.worst.max(x);
        }
    }
    Some(cov)
}

/// Triangles the merge added (restored or built) that cross a triangle of the result they share no
/// vertex with.
pub(super) fn added_crossings(u: &Union) -> usize {
    let mesh = Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() };
    let bvh = Bvh::build(&mesh);
    let added: Vec<usize> =
        (0..u.triangles.len()).filter(|&i| !matches!(u.origin.get(i), Some(Origin::Kept(..)))).collect();
    let flags = par::map_slice(&added, |&i| {
        let (Some(c), Some(t)) = (mesh.triangle(i), mesh.triangles.get(i)) else { return false };
        let (near, more) = bvh.overlapping_up_to(&Aabb::from_points(c.iter()), MAX_NEAR, &|_| true);
        more || near.iter().any(|&o| {
            let (Some(y), Some(ot)) = (mesh.triangle(o as usize), mesh.triangles.get(o as usize)) else { return false };
            o as usize != i && !ot.iter().any(|v| t.contains(v)) && nesting::crosses(&c, &y)
        })
    });
    flags.iter().filter(|f| **f).count()
}

/// Vertices of the result closer than this share of the median edge to a vertex of another scan
/// without an edge between them are left unwelded along the seam.
pub const DUPLICATE_SHARE: f64 = 0.01;

/// Vertices of the result (those triangles use) with a vertex of another scan closer than `radius`
/// that shares no edge with them: two copies of one point left unwelded.
pub(super) fn unwelded(u: &Union, radius: f64) -> usize {
    let mut used = vec![false; u.positions.len()];
    let mut edges: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    for t in &u.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges.insert((a.min(b), a.max(b)));
            if let Some(f) = used.get_mut(a as usize) {
                *f = true;
            }
        }
    }
    let ids: Vec<u32> =
        (0..u.positions.len() as u32).filter(|&v| used.get(v as usize).copied().unwrap_or(false)).collect();
    let pts: Vec<Point3> = ids.iter().filter_map(|&v| u.positions.get(v as usize).copied()).collect();
    let tree = crate::KdTree::build(&pts);
    let flags = par::map_slice(&ids, |&v| {
        let (Some(p), Some(sv)) = (u.positions.get(v as usize), u.scan_of.get(v as usize)) else { return false };
        tree.within(p, radius)
            .into_iter()
            .filter_map(|j| ids.get(j).copied())
            .any(|w| w != v && u.scan_of.get(w as usize) != Some(sv) && !edges.contains(&(v.min(w), v.max(w))))
    });
    flags.iter().filter(|f| **f).count()
}

/// The scan a union triangle comes from (`None` for a seam triangle).
fn scan_of(o: Option<&Origin>) -> Option<u32> {
    match o {
        Some(Origin::Kept(s, _) | Origin::Restored(s, _) | Origin::Recovered(s, _)) => Some(*s),
        _ => None,
    }
}

/// Share of the area of the scan triangles in the result whose centre lies within `tol` of facing
/// surface of a scan triangle of another scan that shares no vertex with it: two layers left where
/// the scans overlapped (seam triangles are left out on both sides).
pub(super) fn stacked_share(u: &Union, tol: f64) -> f64 {
    let mesh = Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() };
    let bvh = Bvh::build(&mesh);
    let ids: Vec<usize> = (0..mesh.triangles.len()).filter(|&i| scan_of(u.origin.get(i)).is_some()).collect();
    let flags = par::map_slice(&ids, |&i| {
        let (Some(c), Some(n), Some(t), Some(s)) =
            (mesh.face_centroid(i), mesh.face_normal(i), mesh.triangles.get(i), scan_of(u.origin.get(i)))
        else {
            return (0.0, false);
        };
        let other = |o: usize| {
            scan_of(u.origin.get(o)).is_some_and(|so| so != s)
                && mesh.triangles.get(o).is_some_and(|ot| !ot.iter().any(|v| t.contains(v)))
                && mesh.face_normal(o).is_some_and(|m| m.dot(&n) >= super::overlap::COS_FACING)
        };
        (mesh.face_area(i), bvh.closest_filtered(&c, tol, &other).is_some())
    });
    let total = flags.iter().fold(0.0, |a, f| a + f.0);
    let stacked = flags.iter().filter(|f| f.1).fold(0.0, |a, f| a + f.0);
    if total > 0.0 { stacked / total } else { 0.0 }
}
