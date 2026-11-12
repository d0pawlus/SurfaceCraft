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
    /// Samples along the edges whose patch normal is undefined (a collapsed or singular patch there);
    /// they are not in the other figures, so a network that has any is not shown to be tangent.
    pub invalid_samples: usize,
    /// Edges with at least one such sample (an edge with only such samples is not in `count`).
    pub invalid_edges: usize,
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
        ..JumpStats::default()
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
/// points along it (`None` where a normal is undefined), with the edge's key.
fn edge_angles(set: &PatchSet) -> Vec<(EdgeKey, Vec<Option<f64>>)> {
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
        let angles: Vec<Option<f64>> = (1..8)
            .map(|i| {
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
/// `creases` the mean along each edge (the angle the creases keep). Samples with an undefined normal are
/// counted as invalid in both results, never dropped silently.
#[must_use]
pub fn jump_stats(set: &PatchSet, creases: &BTreeSet<(u32, u32)>) -> (JumpStats, JumpStats) {
    let (mut smooth, mut sharp) = (Vec::new(), Vec::new());
    let (mut bad_smooth, mut bad_sharp) = ((0, 0), (0, 0));
    for (k, samples) in edge_angles(set) {
        let angles: Vec<f64> = samples.iter().flatten().copied().collect();
        let invalid = samples.len() - angles.len();
        let (list, bad) =
            if creases.contains(&k) { (&mut sharp, &mut bad_sharp) } else { (&mut smooth, &mut bad_smooth) };
        if invalid > 0 {
            *bad = (bad.0 + invalid, bad.1 + 1);
        }
        if angles.is_empty() {
            continue;
        }
        let value = if creases.contains(&k) {
            angles.iter().sum::<f64>() / angles.len() as f64
        } else {
            angles.iter().copied().fold(0.0, f64::max)
        };
        list.push(value);
    }
    let with = |s: JumpStats, bad: (usize, usize)| JumpStats { invalid_samples: bad.0, invalid_edges: bad.1, ..s };
    (with(stats_of(smooth), bad_smooth), with(stats_of(sharp), bad_sharp))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn flat(origin: [f64; 2], collapse_bottom: bool) -> crate::patch::Net {
        let mut net = [[[0.0; 3]; 4]; 4];
        for (i, row) in net.iter_mut().enumerate() {
            for (j, p) in row.iter_mut().enumerate() {
                *p = [origin[0] + i as f64, origin[1] + j as f64, 0.0];
            }
        }
        if collapse_bottom {
            for row in net.iter_mut() {
                row[0] = [origin[0], origin[1], 0.0];
            }
        }
        net
    }

    #[test]
    fn an_edge_with_undefined_normals_is_reported_not_dropped() {
        // Arrange: two patches share the edge 1-2; the second is collapsed along it
        let set = PatchSet {
            nets: vec![flat([0.0, 0.0], false), flat([3.0, 0.0], true)],
            corners: vec![[0, 1, 2, 3], [2, 1, 6, 7]],
            vertices: Vec::new(),
        };

        // Act
        let (smooth, _) = jump_stats(&set, &BTreeSet::new());

        // Assert
        assert_eq!(smooth.invalid_edges, 1, "{smooth:?}");
        assert_eq!(smooth.invalid_samples, 7, "{smooth:?}");
        assert_eq!(smooth.count, 0, "an edge with no valid sample has no jump to report");
    }

    #[test]
    fn a_clean_network_reports_no_invalid_samples() {
        let set = PatchSet {
            nets: vec![flat([0.0, 0.0], false), flat([3.0, 0.0], false)],
            corners: vec![[0, 1, 2, 3], [1, 4, 5, 2]],
            vertices: Vec::new(),
        };
        let (smooth, _) = jump_stats(&set, &BTreeSet::new());
        assert_eq!((smooth.invalid_samples, smooth.invalid_edges, smooth.count), (0, 0, 1), "{smooth:?}");
    }
}
