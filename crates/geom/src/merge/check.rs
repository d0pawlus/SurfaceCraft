//! The honesty checks of the merge, measured on the result: is every removed triangle covered by
//! the result (points sampled over the removed triangles by area, not only their corners), and does
//! any triangle the merge added (restored or built in the seam) cross another triangle?
use super::union::Origin;
use crate::{Aabb, Bvh, Cancel, Mesh, Point3, Vec3, nesting, par};

/// Points sampled over the removed triangles at most (a cap on the whole check, not per triangle).
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

/// The result as the checks read it: its mesh (union vertex indices), a BVH over it, the origin of
/// every triangle and the scan of every vertex.
pub(super) struct Built<'m> {
    pub(super) mesh: &'m Mesh,
    pub(super) bvh: &'m Bvh,
    pub(super) origin: &'m [Origin],
    pub(super) scan_of: &'m [u32],
}

/// Points over triangle `t`: the centres of the `k × k` subdivision of its sides, each standing for
/// an equal share of its area.
fn subdivide(t: &[Point3; 3], k: usize, out: &mut Vec<(Point3, Vec3, f64)>) {
    let [a, b, c] = *t;
    let Some(nrm) = crate::tri::normal(&a, &b, &c) else { return };
    let n = k as f64;
    let each = crate::tri::area(&a, &b, &c) / (n * n);
    let at = |u: f64, v: f64| a + (b - a) * (u / n) + (c - a) * (v / n);
    for i in 0..k {
        for j in 0..k - i {
            let (fi, fj) = (i as f64, j as f64);
            out.push((at(fi + 1.0 / 3.0, fj + 1.0 / 3.0), nrm, each));
            if i + j + 1 < k {
                out.push((at(fi + 2.0 / 3.0, fj + 2.0 / 3.0), nrm, each));
            }
        }
    }
}

/// Measures `pts` against the result into `cov`.
fn measure(r: &Built<'_>, pts: &[(Point3, Vec3, f64)], tol: f64, cov: &mut Coverage) {
    let d = par::map_slice(pts, |(p, n, _)| {
        r.bvh.closest_facing(p, f64::INFINITY, &[*n], 0.0).map_or(f64::INFINITY, |h| h.distance)
    });
    for ((_, _, w), x) in pts.iter().zip(d) {
        cov.samples += 1;
        cov.area += w;
        if x <= tol {
            cov.area_within += w;
        }
        // a sample with no facing result surface at all (an empty result) counts as uncovered, with
        // the worst distance left finite
        if x.is_finite() {
            cov.worst = cov.worst.max(x);
        }
    }
}

/// Measures points sampled over the `n` removed triangles `gone(i)` against the result, made and
/// measured in blocks (never all at once), at most [`MAX_REMOVED_SAMPLES`] in all:
/// - the centres of a subdivision with spacing at most the tolerance, when that stays within the
///   budget (every triangle at least once);
/// - else, when there are fewer triangles than the budget, one point in every triangle and the rest
///   of the budget by area;
/// - else exactly the budget spread by area (a triangle smaller than the share of one point may get
///   none).
///
/// `None` when cancelled.
pub(super) fn coverage(
    r: &Built<'_>,
    (n, gone): (usize, &dyn Fn(usize) -> Option<[Point3; 3]>),
    tol: f64,
    cancel: &Cancel,
) -> Option<Coverage> {
    let area = |t: &[Point3; 3]| crate::tri::area(&t[0], &t[1], &t[2]);
    let total: f64 = (0..n).filter_map(gone).map(|t| area(&t)).sum();
    let each = tol * tol;
    let split =
        |t: &[Point3; 3]| if each > 0.0 { ((area(t) / each).sqrt().ceil() as usize).clamp(1, MAX_SPLIT) } else { 1 };
    // what the subdivision emits: k² points a triangle
    let estimate: usize = (0..n).filter_map(gone).map(|t| split(&t).pow(2)).sum();
    let mut cov = Coverage::default();
    let mut buf: Vec<(Point3, Vec3, f64)> = Vec::with_capacity(CHUNK);
    let mut emit = |p: (Point3, Vec3, f64)| -> Option<()> {
        buf.push(p);
        if buf.len() >= CHUNK {
            if cancel.is_cancelled() {
                return None;
            }
            measure(r, &buf, tol, &mut cov);
            buf.clear();
        }
        Some(())
    };
    if estimate <= MAX_REMOVED_SAMPLES {
        let mut one = Vec::new();
        for t in (0..n).filter_map(gone) {
            one.clear();
            subdivide(&t, split(&t), &mut one);
            one.iter().try_for_each(|&p| emit(p))?;
        }
    } else if n < MAX_REMOVED_SAMPLES {
        let rest = (MAX_REMOVED_SAMPLES - n) as f64;
        for t in (0..n).filter_map(gone) {
            let m = 1 + (area(&t) / total.max(f64::MIN_POSITIVE) * rest) as usize;
            let nrm = crate::tri::normal(&t[0], &t[1], &t[2]).unwrap_or_else(Vec3::z);
            (0..m).try_for_each(|k| emit((super::overlap::point_in(&t, k), nrm, area(&t) / m as f64)))?;
        }
    } else {
        spread(n, gone, total, &mut emit)?;
    }
    if cancel.is_cancelled() {
        return None;
    }
    measure(r, &buf, tol, &mut cov);
    Some(cov)
}

/// Exactly [`MAX_REMOVED_SAMPLES`] points spread over the removed triangles by area (the k-th at the
/// area fraction `(k + ½) / count`), handed to `emit` one by one.
fn spread(
    n: usize,
    gone: &dyn Fn(usize) -> Option<[Point3; 3]>,
    total: f64,
    emit: &mut dyn FnMut((Point3, Vec3, f64)) -> Option<()>,
) -> Option<()> {
    let count = MAX_REMOVED_SAMPLES;
    let step = total / count as f64;
    let (mut i, mut upto, mut k) = (0usize, 0.0, 0usize);
    while k < count && i < n {
        let Some(t) = gone(i) else {
            i += 1;
            continue;
        };
        let a = crate::tri::area(&t[0], &t[1], &t[2]);
        let nrm = crate::tri::normal(&t[0], &t[1], &t[2]).unwrap_or_else(Vec3::z);
        while k < count && (k as f64 + 0.5) * step < upto + a {
            emit((super::overlap::point_in(&t, k), nrm, step))?;
            k += 1;
        }
        upto += a;
        i += 1;
    }
    Some(())
}

/// Triangles the merge added (restored or built) that cross a triangle of the result they share no
/// vertex with. `None` when cancelled.
pub(super) fn added_crossings(r: &Built<'_>, cancel: &Cancel) -> Option<usize> {
    let added: Vec<usize> =
        (0..r.mesh.triangles.len()).filter(|&i| !matches!(r.origin.get(i), Some(Origin::Kept(..)))).collect();
    let mut count = 0;
    for chunk in added.chunks(CHUNK) {
        if cancel.is_cancelled() {
            return None;
        }
        let flags = par::map_slice(chunk, |&i| {
            let (Some(c), Some(t)) = (r.mesh.triangle(i), r.mesh.triangles.get(i)) else { return false };
            let (near, more) = r.bvh.overlapping_up_to(&Aabb::from_points(c.iter()), MAX_NEAR, &|_| true);
            more || near.iter().any(|&o| {
                let (Some(y), Some(ot)) = (r.mesh.triangle(o as usize), r.mesh.triangles.get(o as usize)) else {
                    return false;
                };
                o as usize != i && !ot.iter().any(|v| t.contains(v)) && nesting::crosses(&c, &y)
            })
        });
        count += flags.iter().filter(|f| **f).count();
    }
    Some(count)
}

/// Vertices of the result closer than this share of the median edge to a vertex of another scan
/// without an edge between them are left unwelded along the seam.
pub const DUPLICATE_SHARE: f64 = 0.01;

/// Vertices of the result (those triangles use) with a vertex of another scan closer than `radius`
/// that shares no edge with them: two copies of one point left unwelded.
pub(super) fn unwelded(r: &Built<'_>, radius: f64, cancel: &Cancel) -> Option<usize> {
    let positions = &r.mesh.positions;
    let mut used = vec![false; positions.len()];
    let mut edges: std::collections::HashSet<(u32, u32)> = std::collections::HashSet::new();
    for t in &r.mesh.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges.insert((a.min(b), a.max(b)));
            if let Some(f) = used.get_mut(a as usize) {
                *f = true;
            }
        }
    }
    let ids: Vec<u32> =
        (0..positions.len() as u32).filter(|&v| used.get(v as usize).copied().unwrap_or(false)).collect();
    let pts: Vec<Point3> = ids.iter().filter_map(|&v| positions.get(v as usize).copied()).collect();
    let tree = crate::KdTree::build(&pts);
    let mut count = 0;
    for chunk in ids.chunks(CHUNK) {
        if cancel.is_cancelled() {
            return None;
        }
        let flags = par::map_slice(chunk, |&v| {
            let (Some(p), Some(sv)) = (positions.get(v as usize), r.scan_of.get(v as usize)) else { return false };
            tree.within(p, radius)
                .into_iter()
                .filter_map(|j| ids.get(j).copied())
                .any(|w| w != v && r.scan_of.get(w as usize) != Some(sv) && !edges.contains(&(v.min(w), v.max(w))))
        });
        count += flags.iter().filter(|f| **f).count();
    }
    Some(count)
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
/// Returns (share, triangles counted).
pub(super) fn stacked_share(r: &Built<'_>, tol: f64, cancel: &Cancel) -> Option<(f64, usize)> {
    let ids: Vec<usize> = (0..r.mesh.triangles.len()).filter(|&i| scan_of(r.origin.get(i)).is_some()).collect();
    let mut flags: Vec<(f64, bool)> = Vec::with_capacity(ids.len());
    for chunk in ids.chunks(CHUNK) {
        if cancel.is_cancelled() {
            return None;
        }
        flags.extend(stacked_flags(r, chunk, tol));
    }
    let total = flags.iter().fold(0.0, |a, f| a + f.0);
    let stacked = flags.iter().filter(|f| f.1).fold(0.0, |a, f| a + f.0);
    let count = flags.iter().filter(|f| f.1).count();
    Some((if total > 0.0 { stacked / total } else { 0.0 }, count))
}

/// (area, lies on another scan) for the triangles `ids`.
fn stacked_flags(r: &Built<'_>, ids: &[usize], tol: f64) -> Vec<(f64, bool)> {
    let (mesh, bvh, origin) = (r.mesh, r.bvh, r.origin);
    par::map_slice(ids, |&i| {
        let (Some(c), Some(n), Some(t), Some(s)) =
            (mesh.face_centroid(i), mesh.face_normal(i), mesh.triangles.get(i), scan_of(origin.get(i)))
        else {
            return (0.0, false);
        };
        let other = |o: usize| {
            scan_of(origin.get(o)).is_some_and(|so| so != s)
                && mesh.triangles.get(o).is_some_and(|ot| !ot.iter().any(|v| t.contains(v)))
                && mesh.face_normal(o).is_some_and(|m| m.dot(&n) >= super::overlap::COS_FACING)
        };
        (mesh.face_area(i), bvh.closest_filtered(&c, tol, &other).is_some())
    })
}
