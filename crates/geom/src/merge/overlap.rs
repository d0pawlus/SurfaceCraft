//! Where two scans cover the same surface and how well they agree there, from points sampled over
//! each scan by area: the residual of every pair (rms, a high quantile, the worst, the share beyond
//! the tolerance), the share of each scan that another one covers, and the evidence that one of two
//! scans is turned inside out (their normals oppose where they meet).
use super::prep::Scan;
use crate::{Cancel, Mesh, Point3, Vec3, par};

/// Points sampled over each scan for the pair statistics.
pub const PAIR_SAMPLES: usize = 20_000;
/// Samples handled between two looks at the cancellation flag.
const CHUNK: usize = 4096;
/// Two surfaces face the same way when the cosine of the angle of their normals is at least this.
pub(super) const COS_FACING: f64 = 0.5;
/// Two surfaces oppose when that cosine is at most minus this.
const COS_OPPOSED: f64 = 0.5;

/// A point sampled over a scan, with the normal of its triangle.
#[derive(Clone, Copy, Debug)]
pub(super) struct Sample {
    pub(super) point: Point3,
    pub(super) normal: Vec3,
}

/// Fractional part (for the low-discrepancy sequence; exactly rounded operations only).
fn frac(x: f64) -> f64 {
    x - x.floor()
}

/// `count` points over the triangles of `mesh`, spread by area: the k-th point lies at the area
/// fraction `(k + ½) / count` along the triangles, at a place in its triangle from a low-discrepancy
/// sequence (the plastic number's), so the same mesh gives the same points everywhere.
pub(super) fn area_samples(mesh: &Mesh, count: usize) -> Vec<Sample> {
    let mut prefix = Vec::with_capacity(mesh.triangles.len());
    let mut total = 0.0;
    for t in 0..mesh.triangles.len() {
        total += mesh.face_area(t);
        prefix.push(total);
    }
    if !(total > 0.0) || count == 0 {
        return Vec::new();
    }
    const A1: f64 = 0.754_877_666_246_692_7; // 1 / plastic number
    const A2: f64 = 0.569_840_290_998_053_3; // its square
    let each = total / count as f64;
    (0..count)
        .filter_map(|k| {
            let at = (k as f64 + 0.5) * each;
            let t = prefix.partition_point(|&c| c < at).min(prefix.len() - 1);
            let [a, b, c] = mesh.triangle(t)?;
            let (mut u, mut v) = (frac(0.5 + A1 * k as f64), frac(0.5 + A2 * k as f64));
            if u + v > 1.0 {
                (u, v) = (1.0 - u, 1.0 - v);
            }
            let normal = mesh.face_normal(t)?;
            Some(Sample { point: a + (b - a) * u + (c - a) * v, normal })
        })
        .collect()
}

/// What one sample of a scan found on another.
#[derive(Clone, Copy, Debug)]
enum Seen {
    /// Facing surface of the other scan within the reach: the distance to it.
    Facing(f64),
    /// Opposed surface of the other scan within the tolerance (and no facing one within it).
    Opposed,
}

/// The agreement of scan `a`'s samples with scan `b`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PairResidual {
    /// The scan sampled.
    pub a: usize,
    /// The scan measured against.
    pub b: usize,
    /// Samples of `a`.
    pub samples: usize,
    /// Of them, on facing surface of `b` (inside it, not past its border) within the reach.
    pub near: usize,
    /// Of those, within the tolerance.
    pub within: usize,
    /// Samples on opposed surface of `b` within the tolerance.
    pub opposed: usize,
    /// Root mean square of the distances of the near samples (mm).
    pub rms: f64,
    /// 95th percentile of those distances (mm).
    pub p95: f64,
    /// Largest of those distances (mm).
    pub worst: f64,
}

impl PairResidual {
    /// Share of the near samples farther than the tolerance.
    #[must_use]
    pub fn beyond_share(&self) -> f64 {
        if self.near == 0 { 0.0 } else { (self.near - self.within) as f64 / self.near as f64 }
    }
}

/// What sample `s` of one scan finds on `other` (reach `reach`, tolerance `tol`).
fn look(s: &Sample, other: &Scan, reach: f64, tol: f64) -> Option<Seen> {
    let grown = other.bbox.dist2(&s.point);
    if !(grown <= reach * reach) {
        return None;
    }
    let facing = other.bvh.closest_facing(&s.point, reach, &[s.normal], COS_FACING);
    if let Some(h) = facing.filter(|h| other.interior(h.triangle, &h.point)) {
        if h.distance <= tol {
            return Some(Seen::Facing(h.distance));
        }
        // a nearer opposed surface within the tolerance says more than a facing one farther away
        let opposed = other.bvh.closest(&s.point, tol).filter(|o| opposes(other, s, o));
        return Some(opposed.map_or(Seen::Facing(h.distance), |_| Seen::Opposed));
    }
    other.bvh.closest(&s.point, tol).filter(|o| opposes(other, s, o)).map(|_| Seen::Opposed)
}

fn opposes(other: &Scan, s: &Sample, hit: &crate::SurfaceHit) -> bool {
    other.face_normal(hit.triangle).dot(&s.normal) <= -COS_OPPOSED && other.interior(hit.triangle, &hit.point)
}

/// Residuals of the samples of scan `a` on every other scan; `None` when cancelled.
fn residuals_of(
    scans: &[Scan],
    a: usize,
    samples: &[Sample],
    (reach, tol): (f64, f64),
    cancel: &Cancel,
) -> Option<Vec<Vec<Option<Seen>>>> {
    let mut per_other: Vec<Vec<Option<Seen>>> = vec![Vec::with_capacity(samples.len()); scans.len()];
    for chunk in samples.chunks(CHUNK) {
        if cancel.is_cancelled() {
            return None;
        }
        let seen: Vec<Vec<Option<Seen>>> = par::map_slice(chunk, |s| {
            scans.iter().enumerate().map(|(b, o)| if b == a { None } else { look(s, o, reach, tol) }).collect()
        });
        for row in seen {
            for (b, v) in row.into_iter().enumerate() {
                if let Some(list) = per_other.get_mut(b) {
                    list.push(v);
                }
            }
        }
    }
    Some(per_other)
}

/// Summary of what the samples of `a` saw on `b`.
fn summary(a: usize, b: usize, samples: usize, seen: &[Option<Seen>], tol: f64) -> PairResidual {
    let mut d: Vec<f64> =
        seen.iter().filter_map(|s| if let Some(Seen::Facing(x)) = s { Some(*x) } else { None }).collect();
    let opposed = seen.iter().filter(|s| matches!(s, Some(Seen::Opposed))).count();
    let near = d.len();
    let within = d.iter().filter(|&&x| x <= tol).count();
    let rms = if near == 0 { 0.0 } else { (d.iter().map(|x| x * x).sum::<f64>() / near as f64).sqrt() };
    d.sort_by(f64::total_cmp);
    let p95 = d.get((near * 95 / 100).min(near.saturating_sub(1))).copied().unwrap_or(0.0);
    let worst = d.last().copied().unwrap_or(0.0);
    PairResidual { a, b, samples, near, within, opposed, rms, p95, worst }
}

/// The pair residuals (every ordered pair with something near) and, per scan, the share of its
/// samples within the tolerance of facing surface of another scan. `None` when cancelled.
pub(super) fn residuals(
    scans: &[Scan],
    reach: f64,
    tol: f64,
    cancel: &Cancel,
) -> Option<(Vec<PairResidual>, Vec<f64>)> {
    let mut pairs = Vec::new();
    let mut covered = Vec::with_capacity(scans.len());
    for (a, scan) in scans.iter().enumerate() {
        let samples = area_samples(&scan.mesh, PAIR_SAMPLES);
        let per_other = residuals_of(scans, a, &samples, (reach, tol), cancel)?;
        let hit = (0..samples.len())
            .filter(|&i| {
                per_other.iter().any(|l| l.get(i).is_some_and(|s| matches!(s, Some(Seen::Facing(x)) if *x <= tol)))
            })
            .count();
        covered.push(if samples.is_empty() { 0.0 } else { hit as f64 / samples.len() as f64 });
        for (b, seen) in per_other.iter().enumerate().filter(|(b, _)| *b != a) {
            let r = summary(a, b, samples.len(), seen, tol);
            if r.near > 0 || r.opposed > 0 {
                pairs.push(r);
            }
        }
    }
    Some((pairs, covered))
}
