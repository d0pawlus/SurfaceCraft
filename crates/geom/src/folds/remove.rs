//! Taking the folds out: the margin around a group, the hole fill, and the two distances that decide
//! whether a group stays out. A group is taken out with the rings of triangles around it (its vertices are
//! tangled with the sheet it crosses; the hole must be bounded by clean surface) and the hole is filled
//! with the smooth hole fill. A fill is accepted only when what it replaced stays within
//! [`super::FILL_ACCEPT_SHARE`] of the tolerance (see `measure`): it is what separates a fold from a crease
//! or the foot of a boss that was rounded off.
use super::Refusal;
use super::detect::{Incidence, MAX_ROW, Site};
use super::measure::{Taken, measure};
use crate::{Mesh, edit, holes};
use std::collections::BTreeSet;

/// Rings of triangles taken out around a group of more than [`TINY_GROUP`] triangles.
const MARGIN_RINGS: usize = 2;
/// A group of at most this many triangles (a single inverted sliver) is taken out with one ring only.
const TINY_GROUP: usize = 4;
/// The group and its margin together have at most this many triangles; beyond it the group is left.
const MAX_MARGIN_TRIANGLES: usize = 4 * super::MAX_TRIANGLES;
/// Longest hole filled after removing a group (boundary edges).
const FILL_MAX_EDGES: usize = 2 * super::MAX_TRIANGLES;

/// The triangles of `site` and the rings of triangles around them, so that the hole's edge lies on clean
/// surface. The margin stops short of the rim (a hole that reaches it would notch the rim): a group within
/// one or two rings of the rim gets fewer rings than it would elsewhere, and the patch of its hole is then
/// bounded by triangles that touch the fold; the measured gate sees that (`Taken::strays`). A group on the
/// rim has no margin at all, and `None` says the group cannot be taken out: it touches a hub (a vertex of more than
/// [`MAX_ROW`] triangles) or its margin is more than [`MAX_MARGIN_TRIANGLES`] triangles.
pub(super) fn with_margin(mesh: &Mesh, inc: &Incidence, rim_tris: &[bool], site: &Site) -> Option<Vec<u32>> {
    let mut set: BTreeSet<u32> = site.triangles.iter().copied().collect();
    if site.on_rim {
        return Some(set.into_iter().collect());
    }
    let rings = if site.triangles.len() <= TINY_GROUP { 1 } else { MARGIN_RINGS };
    for _ in 0..rings {
        let mut ring = BTreeSet::new();
        for &t in &set {
            for &v in mesh.triangles.get(t as usize)? {
                let row = inc.row(v);
                if row.len() > MAX_ROW {
                    return None;
                }
                ring.extend(row.iter().copied());
            }
            if set.len() + ring.len() > MAX_MARGIN_TRIANGLES {
                return None;
            }
        }
        if ring.iter().any(|&t| rim_tris.get(t as usize).copied().unwrap_or(true)) {
            break;
        }
        set.extend(ring);
    }
    Some(set.into_iter().collect())
}

/// What one attempt to take groups out made.
pub(super) struct Attempt {
    pub mesh: Mesh,
    /// Per triangle of `mesh`: it is a triangle of the scan as it was (not a patch).
    pub origin: Vec<bool>,
    /// Groups (indices into the list given) whose hole stayed open or whose patch strayed, and why.
    pub failed: Vec<(usize, Refusal)>,
    pub added: usize,
    pub taken: Vec<Taken>,
    /// Triangles of the mesh before, of the scan as it was, that were taken out.
    pub removed_original: Vec<u32>,
}

fn rim_vertices(mesh: &Mesh) -> Vec<bool> {
    let mut r = vec![false; mesh.positions.len()];
    for &v in edit::boundary_loops(mesh).iter().flatten() {
        if let Some(x) = r.get_mut(v as usize) {
            *x = true;
        }
    }
    r
}

/// The triangles to take out, the group that owns each of their vertices (the first one to claim it) and
/// the count and area per group.
fn mark(mesh: &Mesh, origin: &[bool], plans: &[Vec<u32>], skip: &[bool]) -> (Vec<bool>, Vec<u32>, Vec<Taken>) {
    let mut owner = vec![u32::MAX; mesh.positions.len()];
    let mut dead = vec![false; mesh.triangles.len()];
    let mut taken = vec![Taken::default(); plans.len()];
    for (g, tris) in plans.iter().enumerate().filter(|(g, _)| !skip.get(*g).copied().unwrap_or(false)) {
        for &t in tris {
            let (Some(d), Some(c)) = (dead.get_mut(t as usize), mesh.triangles.get(t as usize)) else { continue };
            if !*d && let Some(x) = taken.get_mut(g) {
                x.triangles += 1;
                x.area +=
                    if origin.get(t as usize).copied().unwrap_or(false) { mesh.face_area(t as usize) } else { 0.0 };
            }
            *d = true;
            for &v in c {
                if let Some(o) = owner.get_mut(v as usize).filter(|o| **o == u32::MAX) {
                    *o = g as u32;
                }
            }
        }
    }
    (dead, owner, taken)
}

/// The groups that failed in `out` (triangles from index `from` on are the patches): a short loop of the
/// new boundary that does not run along the old rim is a hole that stayed open; a patch with a triangle
/// without a normal would leave a vertex that no wall can be built on.
fn unfilled_groups(out: &Mesh, from: usize, rim: &[bool], owner: &[u32]) -> Vec<usize> {
    let on_rim = |v: &u32| rim.get(*v as usize).copied().unwrap_or(false);
    let owned = |v: &u32| owner.get(*v as usize).copied().filter(|&g| g != u32::MAX).map(|g| g as usize);
    let mut failed: Vec<usize> = edit::boundary_loops(out)
        .iter()
        .filter(|lp| lp.len() <= FILL_MAX_EDGES && !lp.iter().any(on_rim))
        .flat_map(|lp| lp.iter().filter_map(owned).collect::<Vec<_>>())
        .collect();
    failed.extend(
        (from..out.triangles.len())
            .filter(|&i| out.face_normal(i).is_none())
            .flat_map(|i| out.triangles.get(i).into_iter().flatten().filter_map(owned).collect::<Vec<_>>()),
    );
    failed
}

/// The mesh with the triangles of `plans` (the groups and their margins) taken out and the holes filled;
/// a group fails when its hole stays open, a patch has a flat triangle, or the fill strays by more than
/// `accept` (mm). Hole filling appends its triangles after the existing ones and does not reorder them, and
/// neither does the removal of the dead ones: `origin` (which triangles are the scan's own) follows.
pub(super) fn try_remove(
    mesh: &Mesh,
    origin: &[bool],
    folds: &[&Site],
    plans: &[Vec<u32>],
    skip: &[bool],
    accept: f64,
) -> Attempt {
    let (dead, owner, mut taken) = mark(mesh, origin, plans, skip);
    let rim = rim_vertices(mesh);
    let mut out = mesh.clone();
    let mut k = dead.iter();
    out.triangles.retain(|_| !k.next().copied().unwrap_or(false));
    // only the hole a group left (never the rim of the scan, nor a window that was already there)
    let touches = |_: &Mesh, lp: &[u32]| {
        !lp.iter().any(|v| rim.get(*v as usize).copied().unwrap_or(false))
            && lp.iter().any(|&v| owner.get(v as usize).is_some_and(|&g| g != u32::MAX))
    };
    let before = out.triangles.len();
    holes::fill_holes_filtered(&mut out, FILL_MAX_EDGES, true, true, &touches);
    let added = out.triangles.len() - before;
    let live = |g: &usize| !skip.get(*g).copied().unwrap_or(false);
    let measured: Vec<Vec<u32>> =
        plans.iter().enumerate().map(|(g, p)| if live(&g) { p.clone() } else { Vec::new() }).collect();
    measure((mesh, &out), origin, &owner, (folds, &measured), &mut taken);
    let mut failed: Vec<(usize, Refusal)> =
        unfilled_groups(&out, before, &rim, &owner).into_iter().map(|g| (g, Refusal::Unfillable)).collect();
    failed
        .extend((0..taken.len()).filter(live).filter(|&g| taken[g].strays(accept)).map(|g| (g, Refusal::FillRefused)));
    failed.sort_unstable_by_key(|f| f.0);
    failed.dedup_by_key(|f| f.0);
    let mut kept: Vec<bool> = origin.iter().zip(&dead).filter(|(_, d)| !**d).map(|(o, _)| *o).collect();
    kept.extend(std::iter::repeat_n(false, added));
    let removed_original = (0..dead.len() as u32)
        .filter(|&t| dead.get(t as usize).copied().unwrap_or(false) && origin.get(t as usize).copied().unwrap_or(false))
        .collect();
    Attempt { mesh: out, origin: kept, failed, added, taken, removed_original }
}
