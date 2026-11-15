//! Peeling the border of a group that cannot be merged whole (P9-O6e).
//!
//! A group whose border cannot be put on its primitive (moving it would fold, turn over or pull off a
//! neighbouring patch) and that is too far from the primitive to be merged as it is would be lost whole,
//! though only a few patches along its border are the trouble. Those patches are peeled off: every patch of
//! the group that has a corner at a vertex of a neighbour that failed the check of the moved patches, or that uses
//! a boundary edge which leaves the primitive by more than the gap, leaves the group and stays a patch.
//! The rest is snapped and merged as any group. Peeling by vertex (not by edge) takes the failing neighbour
//! off the new border altogether: none of its vertices is on it, so the snap does not touch it again.
//!
//! A peel must leave one connected group without a pinch, no more boundary loops than it had (a peeled patch
//! never becomes a hole of the face), at least [`MIN_PEELED_PATCHES`] patches and an area of
//! [`MIN_AREA_IN_GAPS_SQUARED`] squared gaps; otherwise the group is not peeled further and is judged as it is.
//! Each peel removes at least one patch, and a group is peeled [`MAX_PEELS`] times at most, so peeling ends.
//!
//! Topology only (which patches touch which vertices): no geometry is made here.
use super::geometry::{Group, group_of};
use super::place::Gauge;
use super::samples::{face_area, pinch_free_groups, touches};
use crate::record::brep::{Brep, Loop};
use std::collections::BTreeSet;

/// The most times one group is peeled in a round of the merge.
pub(super) const MAX_PEELS: usize = 4;
/// A peeled group of fewer patches than this is not merged (one patch is no gain).
const MIN_PEELED_PATCHES: usize = 2;
/// A peeled group of less area than this many squared gaps is not merged: the floor the caller puts under a
/// group (collapsed and sliver patches lie on any surface and are no analytic face).
const MIN_AREA_IN_GAPS_SQUARED: f64 = 50.0;
/// Cells a side of the grid the area of a patch is taken from.
const AREA_GRID: usize = 4;

/// A group after a peel: its patches and what the merge reads of them.
pub(super) struct Peeled {
    pub faces: Vec<usize>,
    pub group: Group,
}

/// The vertices at the corners of `faces` of `body`.
pub(super) fn corners_of(body: &Brep, faces: impl IntoIterator<Item = usize>) -> BTreeSet<u32> {
    let mut out = BTreeSet::new();
    for f in faces {
        let Some(face) = body.faces.get(f) else { continue };
        for l in &face.loops {
            match l {
                Loop::Edges { edges } => {
                    for oe in edges {
                        out.extend(body.edges.get(oe.edge as usize).map(|e| e.vertices).unwrap_or_default());
                    }
                }
                Loop::Vertex { vertex } => {
                    out.insert(*vertex);
                }
            }
        }
    }
    out
}

/// The patches that use a boundary edge of `group` with a point farther than `limit` from the primitive of `gauge`
/// (a point the primitive has no closest point to counts as far).
pub(super) fn far_patches(body: &Brep, group: &Group, gauge: &Gauge, limit: f64) -> BTreeSet<usize> {
    let mut out = BTreeSet::new();
    for (le, points) in group.boundary.loops.iter().flatten().zip(&group.edges) {
        let far = points.iter().any(|p| gauge.distance(*p).is_none_or(|d| d > limit));
        if let (true, Some(e)) = (far, body.edges.get(le.edge as usize)) {
            out.extend(e.uses.iter().map(|u| u.face as usize));
        }
    }
    out
}

/// What a peel takes off a group: the patches with a corner at one of `corners`, and the patches `patches`.
pub(super) struct Off {
    pub corners: BTreeSet<u32>,
    pub patches: BTreeSet<usize>,
}

/// `faces` of `body` without the patches `off` names, when what is left is a group the merge can take (see the
/// header); `loops` is the number of boundary loops the group had. The reason otherwise.
pub(super) fn peel(body: &Brep, faces: &[usize], off: &Off, max_gap: f64, loops: usize) -> Result<Peeled, String> {
    let keep: Vec<usize> =
        faces.iter().copied().filter(|&f| !off.patches.contains(&f) && !touches(body, f, &off.corners)).collect();
    if keep.len() == faces.len() {
        return Err("no patch of its border is at the vertices that failed".to_string());
    }
    let pieces = pinch_free_groups(body, &keep);
    let [piece] = pieces.as_slice() else {
        return Err(format!("peeling its border would leave {} pieces", pieces.len()));
    };
    if piece.len() < MIN_PEELED_PATCHES {
        return Err(format!("peeling its border would leave {} patch(es)", piece.len()));
    }
    let area: f64 = piece.iter().filter_map(|&f| face_area(body, f, AREA_GRID)).sum();
    let least = MIN_AREA_IN_GAPS_SQUARED * max_gap * max_gap;
    if !(area >= least) {
        return Err(format!("peeling its border would leave {area:.2} mm2 (least {least:.2})"));
    }
    let group = group_of(body, piece).map_err(|e| format!("peeling its border would leave no group: {e}"))?;
    if group.boundary.loops.len() > loops {
        return Err("peeling its border would make a hole in the group".to_string());
    }
    Ok(Peeled { faces: piece.clone(), group })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A flat sheet of `n x n` bilinear patches 4 mm wide in z = 0, numbered `i * n + j`.
    fn flat_sheet(n: usize) -> Brep {
        let id = |i: usize, j: usize| (i * (n + 1) + j) as u32;
        let pt = |i: usize, j: usize| [i as f64 * 4.0, j as f64 * 4.0, 0.0];
        let (mut nets, mut corners) = (Vec::new(), Vec::new());
        for i in 0..n {
            for j in 0..n {
                nets.push(vec![vec![pt(i, j), pt(i, j + 1)], vec![pt(i + 1, j), pt(i + 1, j + 1)]]);
                corners.push([id(i, j), id(i + 1, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        crate::nurbs::sheet_from_bezier_network(&nets, &corners).unwrap()
    }

    fn off(corners: &[u32], patches: &[usize]) -> Off {
        Off { corners: corners.iter().copied().collect(), patches: patches.iter().copied().collect() }
    }

    /// The patches at a corner of the 4 x 4 sheet come off; the 15 left are one group with one loop.
    #[test]
    fn the_patches_at_a_vertex_come_off_and_the_rest_is_one_group() {
        let body = flat_sheet(4);
        let all: Vec<usize> = (0..16).collect();
        let p = peel(&body, &all, &off(&[0], &[]), 0.1, 1).unwrap();
        assert_eq!(p.faces.len(), 15);
        assert!(!p.faces.contains(&0));
        assert_eq!(p.group.boundary.loops.len(), 1);
        // the middle vertex of the sheet (id 12) is a corner of four patches: they would leave a hole
        let why = |o: Off| peel(&body, &all, &o, 0.1, 1).err().unwrap_or_default();
        assert!(why(off(&[12], &[])).contains("hole"));
        assert!(why(off(&[], &[])).contains("no patch"), "a peel must shrink");
    }

    /// Hostile input never panics: faces and vertices out of range, an empty group, a gap that is not a number.
    #[test]
    fn hostile_input_is_refused_without_a_panic() {
        let body = flat_sheet(3);
        let all: Vec<usize> = (0..9).collect();
        assert!(peel(&body, &[usize::MAX, 0, 1], &off(&[0], &[]), 0.1, 1).is_err());
        assert!(peel(&body, &[], &off(&[0], &[]), 0.1, 1).is_err());
        assert!(peel(&body, &all, &off(&[u32::MAX], &[usize::MAX]), 0.1, 1).is_err());
        assert!(peel(&body, &all, &off(&[0], &[]), f64::NAN, 1).is_err());
        assert!(peel(&body, &all, &off(&[0], &[]), f64::INFINITY, 1).is_err());
        assert!(corners_of(&body, [usize::MAX]).is_empty());
        assert_eq!(corners_of(&body, [0]).len(), 4);
    }
}
