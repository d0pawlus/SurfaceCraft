//! Poses from descriptor matches: mutual nearest neighbours in descriptor space, then RANSAC over
//! triples of them (Fischler and Bolles 1981), written from the papers.
//!
//! A triple of matches fixes a rigid motion. Triples whose side lengths disagree between the two
//! scans are dropped before any fitting; the others are scored by how many of all matches the
//! motion explains (position and normal). The best distinct motions are refined on their own
//! inliers. Every trial draws from a generator seeded by its index, so the result does not depend
//! on how the trials are spread over threads.
use super::fpfh::{Desc, describe};
use super::sample::Samples;
use super::score::{corners, pose_gap};
use crate::{AlignError, kabsch};
use surfacecraft_geom::{Cancel, Point3, Rng, Transform, par};

/// RANSAC trials.
const TRIALS: usize = 60_000;
/// Trials between two looks at the cancel flag.
const BATCH: usize = 6_000;
/// Matches kept, best descriptor distance first.
const MAX_MATCHES: usize = 2_000;
/// Fewer mutual matches than this are topped up with the best one-way matches.
const MIN_MATCHES: usize = 60;
/// Distinct poses returned.
const KEEP: usize = 16;
/// Descriptor radius, in sample spacings.
const DESCRIPTOR_RADIUS: f64 = 6.5;
/// A match is an inlier within this many spacings.
const INLIER_SPACINGS: f64 = 2.5;
/// Sides of a trial triple are at least this many spacings long (short sides fix no rotation).
const MIN_EDGE_SPACINGS: f64 = 8.0;
/// Sides may differ by this fraction between the scans (on top of the inlier distance).
const EDGE_TOLERANCE: f64 = 0.03;
/// Rotated normals of an inlier match agree to at least this cosine.
const NORMAL_COS: f64 = 0.8;
/// Poses closer than this many spacings (largest probe displacement) are one pose.
const DISTINCT_SPACINGS: f64 = 4.0;

struct Match {
    from: Point3,
    to: Point3,
    n_from: surfacecraft_geom::Vec3,
    n_to: surfacecraft_geom::Vec3,
}

fn dist2(a: &Desc, b: &Desc) -> f32 {
    a.iter().zip(b).map(|(x, y)| (x - y) * (x - y)).sum()
}

fn nearest(q: &Desc, set: &[Option<Desc>]) -> Option<(usize, f32)> {
    let mut best: Option<(usize, f32)> = None;
    for (j, d) in set.iter().enumerate() {
        let Some(d) = d else { continue };
        let e = dist2(q, d);
        if best.is_none_or(|(_, b)| e < b) {
            best = Some((j, e));
        }
    }
    best
}

/// Index pairs (moving, fixed): mutual nearest descriptors first, then the best one-way ones when
/// there are few, by descriptor distance.
fn index_pairs(dm: &[Option<Desc>], df: &[Option<Desc>]) -> Vec<(usize, usize)> {
    let fwd = par::map(dm.len(), |i| dm.get(i).and_then(Option::as_ref).and_then(|d| nearest(d, df)));
    let bwd = par::map(df.len(), |j| df.get(j).and_then(Option::as_ref).and_then(|d| nearest(d, dm)));
    let (mut mutual, mut one_way) = (Vec::new(), Vec::new());
    for (i, f) in fwd.iter().enumerate() {
        let Some((j, e)) = *f else { continue };
        let back = bwd.get(j).copied().flatten().map(|(k, _)| k);
        if back == Some(i) { mutual.push((e, i, j)) } else { one_way.push((e, i, j)) }
    }
    let by_distance = |a: &(f32, usize, usize), b: &(f32, usize, usize)| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1));
    mutual.sort_by(by_distance);
    one_way.sort_by(by_distance);
    let missing = MIN_MATCHES.saturating_sub(mutual.len());
    mutual.extend(one_way.into_iter().take(missing));
    mutual.truncate(MAX_MATCHES);
    mutual.into_iter().map(|(_, i, j)| (i, j)).collect()
}

fn matches_of(mov: &Samples, fix: &Samples, pairs: &[(usize, usize)]) -> Vec<Match> {
    pairs
        .iter()
        .filter_map(|&(i, j)| {
            Some(Match {
                from: *mov.pts.get(i)?,
                to: *fix.pts.get(j)?,
                n_from: *mov.nrm.get(i)?,
                n_to: *fix.nrm.get(j)?,
            })
        })
        .collect()
}

fn is_inlier(m: &Match, t: &Transform, tau: f64) -> bool {
    (t.apply(&m.from) - m.to).norm() < tau && t.apply_vec(&m.n_from).dot(&m.n_to) > NORMAL_COS
}

fn inliers(ms: &[Match], t: &Transform, tau: f64) -> usize {
    ms.iter().filter(|m| is_inlier(m, t, tau)).count()
}

/// One trial: the motion of three random matches when their sides agree, and its inlier count.
fn trial(ms: &[Match], seed: u64, tau: f64, min_edge: f64) -> Option<(usize, Transform)> {
    let mut rng = Rng::new(seed);
    let picks = [rng.index(ms.len()), rng.index(ms.len()), rng.index(ms.len())];
    if picks[0] == picks[1] || picks[0] == picks[2] || picks[1] == picks[2] {
        return None;
    }
    let tri: Vec<&Match> = picks.iter().filter_map(|&k| ms.get(k)).collect();
    for (a, b) in [(0usize, 1usize), (0, 2), (1, 2)] {
        let (ma, mb) = (tri.get(a)?, tri.get(b)?);
        let (dm, df) = ((ma.from - mb.from).norm(), (ma.to - mb.to).norm());
        if dm < min_edge || (dm - df).abs() > tau + EDGE_TOLERANCE * dm {
            return None;
        }
    }
    let from: Vec<Point3> = tri.iter().map(|m| m.from).collect();
    let to: Vec<Point3> = tri.iter().map(|m| m.to).collect();
    let t = kabsch(&from, &to, None).ok()?;
    Some((inliers(ms, &t, tau), t))
}

/// Least-squares motion on the inliers of `t`, repeated; `t` itself when that fails.
fn polish(ms: &[Match], t: &Transform, tau: f64) -> Transform {
    let mut cur = *t;
    for _ in 0..3 {
        let set: Vec<&Match> = ms.iter().filter(|m| is_inlier(m, &cur, tau)).collect();
        let from: Vec<Point3> = set.iter().map(|m| m.from).collect();
        let to: Vec<Point3> = set.iter().map(|m| m.to).collect();
        match kabsch(&from, &to, None) {
            Ok(next) => cur = next,
            Err(_) => break,
        }
    }
    cur
}

fn run_trials(
    ms: &[Match],
    tau: f64,
    min_edge: f64,
    cancel: &Cancel,
) -> Result<Vec<(usize, usize, Transform)>, AlignError> {
    let mut hits = Vec::new();
    for batch in 0..TRIALS.div_ceil(BATCH) {
        if cancel.is_cancelled() {
            return Err(AlignError::Cancelled);
        }
        let start = batch * BATCH;
        let found = par::map(BATCH.min(TRIALS - start), |k| {
            let id = start + k;
            trial(ms, (id as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15) ^ 0xA5A5, tau, min_edge).map(|(n, t)| (n, id, t))
        });
        hits.extend(found.into_iter().flatten());
    }
    Ok(hits)
}

/// Candidate motions (moving → fixed) from local descriptors, best first; empty when the scans
/// have too few matching descriptors to fix a pose.
pub(super) fn descriptor_poses(
    mov: &Samples,
    fix: &Samples,
    spacing: f64,
    cancel: &Cancel,
) -> Result<Vec<Transform>, AlignError> {
    let radius = DESCRIPTOR_RADIUS * spacing;
    let (dm, df) = par::join(|| describe(mov, radius), || describe(fix, radius));
    let ms = matches_of(mov, fix, &index_pairs(&dm, &df));
    if ms.len() < 3 {
        return Ok(Vec::new());
    }
    let (tau, min_edge) = (INLIER_SPACINGS * spacing, MIN_EDGE_SPACINGS * spacing);
    let mut hits = run_trials(&ms, tau, min_edge, cancel)?;
    hits.sort_by(|a, b| b.0.cmp(&a.0).then(a.1.cmp(&b.1)));
    let probes = corners(&mov.pts);
    let mut kept: Vec<Transform> = Vec::new();
    for (_, _, t) in hits {
        if kept.len() >= KEEP {
            break;
        }
        if kept.iter().all(|k| pose_gap(k, &t, &probes) > DISTINCT_SPACINGS * spacing) {
            kept.push(t);
        }
    }
    Ok(kept.iter().map(|t| polish(&ms, t, tau)).collect())
}
