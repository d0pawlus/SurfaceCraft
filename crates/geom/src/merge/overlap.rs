//! Where two scans cover the same surface and how well they agree there, from points sampled over
//! each scan by area: the residual of every pair (rms, a high quantile, the worst, the share beyond
//! the tolerance), the share of each scan that another one covers, and the evidence that one of two
//! scans is turned inside out (their normals oppose where they meet).
use super::prep::Scan;
use crate::bvh::RayCast;
use crate::{Cancel, Mesh, Point3, Vec3, par};

/// Points sampled over each scan for the pair statistics.
pub const PAIR_SAMPLES: usize = 20_000;
/// Samples handled between two looks at the cancellation flag.
const CHUNK: usize = 4096;
/// Two surfaces face the same way when the cosine of the angle of their normals is at least this.
pub(super) const COS_FACING: f64 = 0.5;
/// Triangle tests of one ray cast at most (a cast that runs out counts as no hit).
const MAX_RAY_TESTS: usize = 1 << 16;
/// A pair is displaced only when at least this share of the samples lies over the other scan.
pub const OVER_SHARE: f64 = 0.05;
/// The high quantile of the residual reported (percent).
const PERCENTILE: usize = 95;
/// Two surfaces oppose when that cosine is at most minus this.
const COS_OPPOSED: f64 = 0.5;

/// A point sampled over a scan, with the normal of its triangle.
#[derive(Clone, Copy, Debug)]
pub(super) struct Sample {
    pub(super) point: Point3,
    pub(super) normal: Vec3,
}

/// The `k`-th point of a low-discrepancy sequence in triangle `t` (the plastic number's: every
/// operation exactly rounded, so the same points everywhere).
pub(super) fn point_in(t: &[Point3; 3], k: usize) -> Point3 {
    const A1: f64 = 0.754_877_666_246_692_7; // 1 / plastic number
    const A2: f64 = 0.569_840_290_998_053_3; // its square
    let (mut u, mut v) = (frac(0.5 + A1 * k as f64), frac(0.5 + A2 * k as f64));
    if u + v > 1.0 {
        (u, v) = (1.0 - u, 1.0 - v);
    }
    t[0] + (t[1] - t[0]) * u + (t[2] - t[0]) * v
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
    let each = total / count as f64;
    (0..count)
        .filter_map(|k| {
            let at = (k as f64 + 0.5) * each;
            let t = prefix.partition_point(|&c| c < at).min(prefix.len() - 1);
            let corners = mesh.triangle(t)?;
            let normal = mesh.face_normal(t)?;
            Some(Sample { point: point_in(&corners, k), normal })
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
    /// No surface within the reach, but along the sample's normal (either way) the first surface of
    /// the other scan, within its size, faces the same way: the two lie over each other, apart (the
    /// distance along the normal).
    Over(f64),
    /// No surface within the reach, and a cast along the normal ran out of its budget: unknown.
    Unknown,
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
    /// Samples with no surface of `b` within the reach that lie over facing surface of `b` farther away
    /// (along their normal, within the size of `b`): `b` is there, but displaced.
    pub over: usize,
    /// Median distance along the normal of the samples over `b` (mm; 0 when none).
    pub over_offset: f64,
    /// Spread of those distances: 75th minus 25th percentile (mm).
    pub over_spread: f64,
    /// Samples whose cast along the normal ran out of its budget (a pile of triangles): not judged.
    pub unknown: usize,
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

    /// Do at least [`OVER_SHARE`] of the samples (and `min`) lie over `b` displaced beyond the reach,
    /// while (almost) none agrees with it within the tolerance? Scans that share surface somewhere are
    /// aligned there: a shelf seen by one over a floor seen by the other is geometry, not a
    /// displacement. (Rays along the normals of a part with many faces
    /// meet a facing surface of a piece that lies elsewhere now and then: a percent of the samples
    /// of pieces that share nothing was measured on a real scan.)
    /// (Scans offset by more than the reach: they share no surface within it but lie over each other.)
    #[must_use]
    pub fn displaced(&self, min: usize) -> bool {
        self.lies_over(min) && self.within < min
    }

    /// Do at least [`OVER_SHARE`] of the samples (and `min`) lie over `b` beyond the reach?
    #[must_use]
    pub fn lies_over(&self, min: usize) -> bool {
        self.over >= min && self.over as f64 >= OVER_SHARE * self.samples as f64
    }

    /// Are the distances of the samples over `b` all about one value: their spread within the larger of
    /// `tol` and [`OFFSET_SPREAD_SHARE`] of their median? (A copy displaced as a whole lies at one
    /// offset; the steps and shelves of a part lie at many.)
    #[must_use]
    pub fn one_offset(&self, tol: f64) -> bool {
        self.over_spread <= tol.max(OFFSET_SPREAD_SHARE * self.over_offset)
    }
}

/// Over samples whose distances spread by at most this share of their median lie at one offset.
pub const OFFSET_SPREAD_SHARE: f64 = 0.1;

/// What sample `s` of one scan finds on `other` (reach `reach`, tolerance `tol`).
fn look(s: &Sample, other: &Scan, reach: f64, tol: f64) -> Option<Seen> {
    // beyond the reach of the other scan's box no surface of it is near; it may still lie over the
    // sample farther away (a flat sheet displaced along its normal has a box of no thickness)
    if !(other.bbox.dist2(&s.point) <= reach * reach) {
        return over(s, other, reach);
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
    let opposed = other.bvh.closest(&s.point, tol).filter(|o| opposes(other, s, o)).map(|_| Seen::Opposed);
    opposed.or_else(|| over(s, other, reach))
}

/// Is the first surface of `other` along the normal of `s` (either way, beyond `reach` and within
/// the size of `other`) facing the same way? Two copies of one surface displaced along it are; scans
/// side by side are not (the first surface met is the other's back, facing the other way).
fn over(s: &Sample, other: &Scan, reach: f64) -> Option<Seen> {
    let wide = other.bbox.diagonal();
    if !(other.bbox.dist2(&s.point) <= wide * wide) {
        return None;
    }
    // a bounded cast: a pile of triangles at one place costs at most MAX_RAY_TESTS, and a cast that
    // runs out says nothing
    let casts = [s.normal, -s.normal].map(|d| other.bvh.ray_cast_bounded(&s.point, &d, wide, &|_| true, MAX_RAY_TESTS));
    // a direction that ran out may hide a nearer surface: the sample says nothing
    if casts.iter().any(|c| matches!(c, RayCast::Exhausted)) {
        return Some(Seen::Unknown);
    }
    let hit = casts
        .iter()
        .filter_map(|c| if let RayCast::Hit(t, at) = c { Some((*t, *at)) } else { None })
        .min_by(|a, b| a.1.total_cmp(&b.1))?;
    (hit.1 > reach && other.face_normal(hit.0).dot(&s.normal) >= COS_FACING).then_some(Seen::Over(hit.1))
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
    let mut offsets: Vec<f64> =
        seen.iter().filter_map(|s| if let Some(Seen::Over(x)) = s { Some(*x) } else { None }).collect();
    offsets.sort_by(f64::total_cmp);
    let over = offsets.len();
    let quantile = |q: usize| offsets.get((over * q / 100).min(over.saturating_sub(1))).copied().unwrap_or(0.0);
    let (over_offset, over_spread) = (quantile(50), quantile(75) - quantile(25));
    let unknown = seen.iter().filter(|s| matches!(s, Some(Seen::Unknown))).count();
    let near = d.len();
    let within = d.iter().filter(|&&x| x <= tol).count();
    let rms = if near == 0 { 0.0 } else { (d.iter().map(|x| x * x).sum::<f64>() / near as f64).sqrt() };
    d.sort_by(f64::total_cmp);
    let p95 = d.get((near * PERCENTILE / 100).min(near.saturating_sub(1))).copied().unwrap_or(0.0);
    let worst = d.last().copied().unwrap_or(0.0);
    PairResidual { a, b, samples, near, within, opposed, over, over_offset, over_spread, unknown, rms, p95, worst }
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
            if r.near > 0 || r.opposed > 0 || r.over > 0 || r.unknown > 0 {
                pairs.push(r);
            }
        }
    }
    Some((pairs, covered))
}
