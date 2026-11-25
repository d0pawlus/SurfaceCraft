//! Welding the seam: a front vertex of a later scan that lies within the weld radius of a front
//! vertex of an earlier scan (from its own position: no chains) becomes that vertex. A vertex moves
//! by at most the radius. A weld that would flip a triangle against its normal, fold it to less than
//! [`AREA_KEEP`] of its area, make two triangles run one edge the same way, put three on an edge or
//! give two triangles the same three corners is undone.
use super::prep::{NONE, Scan};
use super::union::{Origin, Union, directed};
use crate::{KdTree, Point3, tri};
use std::collections::HashMap;

/// Seam vertices of different scans closer than this share of the tolerance (and [`WELD_EDGE_SHARE`]
/// of the median edge) are one vertex: two copies of one point a scan pair left on either side of the
/// seam.
pub const WELD_SHARE: f64 = 0.25;
/// The weld radius is at most this share of the median edge (a weld never spans a triangle).
pub const WELD_EDGE_SHARE: f64 = 0.1;
/// A weld that leaves a triangle with less than this share of its area is undone.
pub const AREA_KEEP: f64 = 0.1;

/// The weld radius for tolerance `tol` and median edge `edge` (mm).
pub(super) fn weld_radius(tol: f64, edge: f64) -> f64 {
    (WELD_SHARE * tol).min(WELD_EDGE_SHARE * edge)
}

impl Union {
    /// Welds the front vertices of different scans that coincide within `eps` (see the module).
    /// `None` when cancelled (polled once per undo pass).
    pub(super) fn weld_front(&mut self, scans: &[Scan], eps: f64, cancel: &crate::Cancel) -> Option<()> {
        let mut ends: Vec<u32> = self.front(scans).iter().flat_map(|f| [f[0], f[1]]).collect();
        ends.sort_unstable();
        ends.dedup();
        if ends.is_empty() || !(eps > 0.0) {
            return Some(());
        }
        let mut to = self.weld_targets(&ends, eps);
        // every pass undoes at least one weld, so the passes end by the number of welds; were that ever
        // exceeded, no weld is made at all
        let welds = to.iter().enumerate().filter(|(v, w)| *v as u32 != **w).count();
        let mut undone = 0;
        let mut settled = false;
        for _ in 0..=welds {
            if cancel.is_cancelled() {
                return None;
            }
            let bad = self.clashing(&to);
            if bad.is_empty() {
                settled = true;
                break;
            }
            undone += bad.len();
            for v in bad {
                if let Some(slot) = to.get_mut(v as usize) {
                    *slot = v;
                }
            }
        }
        if !settled {
            undone = welds;
            to = (0..self.positions.len() as u32).collect();
        }
        self.welds_undone += undone;
        for (v, &w) in to.iter().enumerate().filter(|(v, w)| *v as u32 != **w) {
            if let (Some(p), Some(q)) = (self.positions.get(v), self.positions.get(w as usize)) {
                self.max_move = self.max_move.max((q - p).norm());
            }
            self.welded += 1;
        }
        self.remap(&to);
        Some(())
    }

    /// For every front vertex in `ends`, the nearest front vertex of a scan listed earlier within
    /// `eps` of its own position that is not itself welded (itself when none).
    pub(super) fn weld_targets(&self, ends: &[u32], eps: f64) -> Vec<u32> {
        let pts: Vec<Point3> = ends.iter().filter_map(|&v| self.positions.get(v as usize).copied()).collect();
        let tree = KdTree::build(&pts);
        let mut to: Vec<u32> = (0..self.positions.len() as u32).collect();
        for (i, &v) in ends.iter().enumerate() {
            let (Some(p), Some(&sv)) = (pts.get(i), self.scan_of.get(v as usize)) else { continue };
            let best = tree
                .within(p, eps)
                .into_iter()
                .filter_map(|j| Some((ends.get(j).copied()?, pts.get(j)?)))
                .filter(|&(w, _)| {
                    self.scan_of.get(w as usize).is_some_and(|&sw| sw < sv) && to.get(w as usize) == Some(&w)
                })
                .min_by(|a, b| (a.1 - p).norm_squared().total_cmp(&(b.1 - p).norm_squared()).then(a.0.cmp(&b.0)));
            if let (Some((w, _)), Some(slot)) = (best, to.get_mut(v as usize)) {
                *slot = w;
            }
        }
        to
    }

    /// Does the triangle `t`, mapped to `m`, survive the weld badly: turned against its own normal or
    /// folded below [`AREA_KEEP`] of its area? (Two corners welded into one is a collapse: fine.)
    fn folds(&self, t: &[u32; 3], m: &[u32; 3]) -> bool {
        let at = |x: &[u32; 3]| -> Option<[Point3; 3]> {
            Some([
                *self.positions.get(x[0] as usize)?,
                *self.positions.get(x[1] as usize)?,
                *self.positions.get(x[2] as usize)?,
            ])
        };
        let (Some(a), Some(b)) = (at(t), at(m)) else { return false };
        let (na, nb) = (tri::cross(&a[0], &a[1], &a[2]), tri::cross(&b[0], &b[1], &b[2]));
        na.dot(&nb) <= 0.0 || nb.norm() < AREA_KEEP * na.norm()
    }

    /// Flags of the vertices `to` welds and of those welded into.
    fn welded_or_into(&self, to: &[u32]) -> Vec<bool> {
        let mut moved = vec![false; self.positions.len()];
        for (v, &w) in to.iter().enumerate().filter(|(v, w)| *v as u32 != **w) {
            for x in [v, w as usize] {
                if let Some(f) = moved.get_mut(x) {
                    *f = true;
                }
            }
        }
        moved
    }

    /// The welded vertices of the triangles a weld would leave flipped, folded, doubled or on an edge
    /// that runs the same way twice or holds three triangles.
    pub(super) fn clashing(&self, to: &[u32]) -> Vec<u32> {
        let map = |v: u32| to.get(v as usize).copied().unwrap_or(v);
        // only triangles at a welded vertex or a vertex welded into can change
        let moved = self.welded_or_into(to);
        let touched: Vec<&[u32; 3]> = self
            .triangles
            .iter()
            .filter(|t| t.iter().any(|&v| moved.get(v as usize).copied().unwrap_or(false)))
            .collect();
        let live = |m: &[u32; 3]| m[0] != m[1] && m[1] != m[2] && m[0] != m[2];
        let (mut count, mut undirected, mut sets) = (HashMap::new(), HashMap::new(), HashMap::new());
        for m in touched.iter().map(|t| t.map(map)).filter(live) {
            let mut key = m;
            key.sort_unstable();
            *sets.entry(key).or_insert(0u32) += 1;
            for k in 0..3 {
                let (a, b) = (m[k], m[(k + 1) % 3]);
                *count.entry(directed(a, b)).or_insert(0u32) += 1;
                *undirected.entry(directed(a.min(b), a.max(b))).or_insert(0u32) += 1;
            }
        }
        let mut bad: Vec<u32> = Vec::new();
        for t in &touched {
            let m = t.map(map);
            if !live(&m) {
                continue;
            }
            let mut key = m;
            key.sort_unstable();
            let edge_bad = (0..3).any(|k| {
                let (a, b) = (m[k], m[(k + 1) % 3]);
                count.get(&directed(a, b)).copied().unwrap_or(0) > 1
                    || undirected.get(&directed(a.min(b), a.max(b))).copied().unwrap_or(0) > 2
            });
            // the culprit: the welded corner of the latest scan (the highest index); the others are
            // undone in a later pass if the triangle is still bad
            if (edge_bad || sets.get(&key).copied().unwrap_or(0) > 1 || self.folds(t, &m))
                && let Some(v) = t.iter().copied().filter(|&v| map(v) != v).max()
            {
                bad.push(v);
            }
        }
        bad.sort_unstable();
        bad.dedup();
        bad
    }

    /// Points every triangle at the vertices `to` gives and drops those that collapse (recording the
    /// kept ones of a scan in [`Union::collapsed`]).
    pub(super) fn remap(&mut self, to: &[u32]) {
        let mut tris = Vec::with_capacity(self.triangles.len());
        let mut origin = Vec::with_capacity(self.triangles.len());
        for (t, o) in self.triangles.iter().zip(&self.origin) {
            let m = t.map(|v| to.get(v as usize).copied().unwrap_or(v));
            if m[0] != m[1] && m[1] != m[2] && m[0] != m[2] {
                tris.push(m);
                origin.push(*o);
            } else if let Origin::Kept(s, i) = o {
                self.collapsed.push((*s, *i));
            }
        }
        for map in &mut self.vertex_of {
            for v in map.iter_mut().filter(|v| **v != NONE) {
                *v = to.get(*v as usize).copied().unwrap_or(*v);
            }
        }
        self.triangles = tris;
        self.origin = origin;
    }
}
