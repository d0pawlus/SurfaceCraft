//! The normal jumps across the edges of a patch network: how tangent-continuous it is, and what angle
//! its creases keep.
use crate::patch::{PatchSet, normal};
use crate::wrap_normals::{EdgeKey, key};
use std::collections::{BTreeMap, BTreeSet};

/// Count, mean, 99th percentile and largest of angles (degrees).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct JumpStats {
    /// Edges measured.
    pub count: usize,
    /// Mean (degrees).
    pub mean_deg: f64,
    /// 99th percentile (degrees).
    pub p99_deg: f64,
    /// Largest (degrees).
    pub max_deg: f64,
}

fn stats_of(mut v: Vec<f64>) -> JumpStats {
    if v.is_empty() {
        return JumpStats::default();
    }
    v.sort_by(f64::total_cmp);
    let count = v.len();
    JumpStats {
        count,
        mean_deg: v.iter().sum::<f64>() / count as f64,
        p99_deg: v.get(((count - 1) as f64 * 0.99) as usize).copied().unwrap_or(0.0),
        max_deg: v.last().copied().unwrap_or(0.0),
    }
}

/// Parameters `(u, v)` at fraction `t` of the loop side `s` (0 bottom, 1 right, 2 top, 3 left).
fn side_uv(s: usize, t: f64) -> (f64, f64) {
    match s {
        0 => (t, 0.0),
        1 => (1.0, t),
        2 => (1.0 - t, 1.0),
        _ => (0.0, 1.0 - t),
    }
}

/// Angles (degrees) between the normals of the two patches on both sides of every interior edge, at 7
/// points along it, with the edge's key.
fn edge_angles(set: &PatchSet) -> Vec<(EdgeKey, Vec<f64>)> {
    let mut by_edge: BTreeMap<EdgeKey, Vec<(usize, usize, bool)>> = BTreeMap::new();
    for (fi, c) in set.corners.iter().enumerate() {
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            by_edge.entry(key(a, b)).or_default().push((fi, s, a < b));
        }
    }
    let mut out = Vec::new();
    for (k, uses) in by_edge {
        let [(f0, s0, fw0), (f1, s1, fw1)] = uses.as_slice() else { continue };
        let (Some(n0), Some(n1)) = (set.nets.get(*f0), set.nets.get(*f1)) else { continue };
        let angles: Vec<f64> = (1..8)
            .filter_map(|i| {
                let t = f64::from(i) / 8.0;
                let (u0, v0) = side_uv(*s0, if *fw0 { t } else { 1.0 - t });
                let (u1, v1) = side_uv(*s1, if *fw1 { t } else { 1.0 - t });
                let (a, b) = (normal(n0, u0, v0)?, normal(n1, u1, v1)?);
                Some(a.dot(&b).clamp(-1.0, 1.0).acos().to_degrees())
            })
            .collect();
        out.push((k, angles));
    }
    out
}

/// The normal jumps across the patch edges: over the smooth edges the worst along each edge, over the
/// `creases` the mean along each edge (the angle the creases keep).
#[must_use]
pub fn jump_stats(set: &PatchSet, creases: &BTreeSet<(u32, u32)>) -> (JumpStats, JumpStats) {
    let (mut smooth, mut sharp) = (Vec::new(), Vec::new());
    for (k, angles) in edge_angles(set) {
        if angles.is_empty() {
            continue;
        }
        if creases.contains(&k) {
            sharp.push(angles.iter().sum::<f64>() / angles.len() as f64);
        } else {
            smooth.push(angles.iter().copied().fold(0.0, f64::max));
        }
    }
    (stats_of(smooth), stats_of(sharp))
}
