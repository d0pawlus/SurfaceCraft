//! Taking the folds out: the margin around a group, the hole fill, and the two distances that decide
//! whether a group stays out. A group is taken out with the rings of triangles around it (its vertices are
//! tangled with the sheet it crosses; the hole must be bounded by clean surface) and the hole is filled
//! with the smooth hole fill. Two distances are measured from the mesh as it comes out:
//! - the height of the fold: its own vertices that are gone, to the new surface;
//! - the fill deviation: the vertices of clean surface that went with the margin, to the patch that
//!   replaced them. A fill is accepted only when this stays within [`super::FILL_ACCEPT_SHARE`] of the
//!   tolerance: it is what separates a fold from a crease or the foot of a boss that was rounded off.
use super::detect::{Incidence, MAX_ROW, Site};
use crate::{Bvh, Mesh, edit, holes};
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
/// surface. The margin stops short of the rim (a hole that reaches it would notch the rim), a group on the
/// rim has none, and `None` says the group cannot be taken out: it touches a hub (a vertex of more than
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

/// The surface around a group (the two rings of triangles outside what is taken out) has an edge sharper than
/// this many degrees between neighbouring triangles: a crease, the foot of a boss. A smooth patch would round it
/// off, so the group is not touched.
pub(super) const FEATURE_DEGREES: f64 = 30.0;

/// Is there a crease or a corner in the two rings of triangles around the plan `plan` (a group and its margin)?
pub(super) fn near_a_feature(mesh: &Mesh, inc: &Incidence, neighbours: &[[u32; 3]], plan: &[u32]) -> bool {
    let inside: BTreeSet<u32> = plan.iter().copied().collect();
    let around = |from: &BTreeSet<u32>| -> BTreeSet<u32> {
        from.iter()
            .filter_map(|&t| mesh.triangles.get(t as usize))
            .flatten()
            .flat_map(|&v| inc.row(v).iter().copied().take(MAX_ROW))
            .filter(|t| !inside.contains(t))
            .collect()
    };
    let first = around(&inside);
    let both: BTreeSet<u32> = first.iter().copied().chain(around(&first)).collect();
    let cos = FEATURE_DEGREES.to_radians().cos();
    both.iter().any(|&t| {
        let Some(n) = mesh.face_normal(t as usize) else { return false };
        neighbours
            .get(t as usize)
            .into_iter()
            .flatten()
            .any(|&o| both.contains(&o) && mesh.face_normal(o as usize).is_some_and(|m| n.dot(&m) < cos))
    })
}

/// What was taken out for one group.
#[derive(Clone, Copy, Debug, Default)]
pub(super) struct Taken {
    pub triangles: usize,
    /// Area of the scan's own triangles taken out (patches of earlier hole filling do not count).
    pub area: f64,
    pub height: f64,
    /// Every removed vertex and triangle centre of the scan, to the new surface.
    pub fill_deviation: f64,
    /// The same for the clean surface only (not what touches the fold): what accepts or refuses a fill.
    pub clean_deviation: f64,
}

/// What one attempt to take groups out made.
pub(super) struct Attempt {
    pub mesh: Mesh,
    /// Per triangle of `mesh`: it is a triangle of the scan as it was (not a patch).
    pub origin: Vec<bool>,
    /// Groups (indices into the list given) whose hole stayed open or whose patch strayed.
    pub failed: Vec<usize>,
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

/// Height of each fold and fill deviation of each patch, measured to the mesh that came out: the vertices of
/// a fold that went out of the mesh give the height; the vertices of clean surface that went with a margin
/// and the centres of its triangles (a patch can cut across a crease between vertices that stay) give the
/// deviation. What touches a vertex of the fold is tangled with it and counts for neither.
fn distances(
    (before, after): (&Mesh, &Mesh),
    origin: &[bool],
    owner: &[u32],
    (folds, plans): (&[&Site], &[Vec<u32>]),
    taken: &mut [Taken],
) {
    // only what the scan recorded counts: a vertex that belongs to a fill patch only is not a point of the scan
    let mut scan_vertex = vec![false; before.positions.len()];
    for c in before.triangles.iter().zip(origin).filter(|(_, o)| **o).map(|(c, _)| c) {
        for &v in c {
            if let Some(x) = scan_vertex.get_mut(v as usize) {
                *x = true;
            }
        }
    }
    let mut used = vec![false; before.positions.len()];
    for &v in after.triangles.iter().flatten() {
        if let Some(u) = used.get_mut(v as usize) {
            *u = true;
        }
    }
    let bvh = Bvh::build(after);
    let distance = |p: &crate::Point3| bvh.closest(p, f64::MAX).map(|h| h.distance);
    let fold_triangles: Vec<BTreeSet<u32>> = folds.iter().map(|s| s.triangles.iter().copied().collect()).collect();
    let fold_vertices: Vec<BTreeSet<u32>> = fold_triangles
        .iter()
        .map(|ts| ts.iter().filter_map(|&t| before.triangles.get(t as usize)).flatten().copied().collect())
        .collect();
    // the vertices of the triangles that touch the fold are tangled with it: they are no clean surface
    let tangled: Vec<BTreeSet<u32>> = plans
        .iter()
        .zip(&fold_vertices)
        .map(|(tris, fold)| {
            let touching = tris
                .iter()
                .filter_map(|&t| before.triangles.get(t as usize))
                .filter(|c| c.iter().any(|v| fold.contains(v)));
            touching.flatten().copied().collect()
        })
        .collect();
    for (v, p) in before.positions.iter().enumerate() {
        let (Some(&g), Some(false), Some(true)) = (owner.get(v), used.get(v).copied(), scan_vertex.get(v).copied())
        else {
            continue;
        };
        let (Some(slot), Some(d)) = (taken.get_mut(g as usize), distance(p)) else { continue };
        let id = v as u32;
        slot.fill_deviation = slot.fill_deviation.max(d);
        if fold_vertices.get(g as usize).is_some_and(|f| f.contains(&id)) {
            slot.height = slot.height.max(d);
        } else if !tangled.get(g as usize).is_some_and(|t| t.contains(&id)) {
            slot.clean_deviation = slot.clean_deviation.max(d);
        }
    }
    for (g, tris) in plans.iter().enumerate() {
        let Some(own) = fold_triangles.get(g) else { continue };
        // the triangles that touch a vertex of the fold are bent by it themselves: they count for the report
        // of the deviation but not for the acceptance of the fill
        let touches_fold =
            |t: &u32| before.triangles.get(*t as usize).is_some_and(|c| c.iter().any(|v| fold_vertices[g].contains(v)));
        let scan_triangle = |t: &u32| origin.get(*t as usize).copied().unwrap_or(false);
        for &t in tris.iter().filter(|t| scan_triangle(t)) {
            let (Some(c), Some(slot)) = (before.face_centroid(t as usize), taken.get_mut(g)) else { continue };
            let d = distance(&c).unwrap_or(0.0);
            slot.fill_deviation = slot.fill_deviation.max(d);
            if !own.contains(&t) && !touches_fold(&t) {
                slot.clean_deviation = slot.clean_deviation.max(d);
            }
        }
    }
}

/// The groups that failed in `out` (triangles from index `from` on are the patches): a short loop of the
/// new boundary that does not run along the old rim is a hole that stayed open; a patch with a triangle
/// without a normal would leave a vertex that no wall can be built on.
fn failed_groups(out: &Mesh, from: usize, rim: &[bool], owner: &[u32]) -> Vec<usize> {
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
/// a group fails when its hole stays open, a patch has a flat triangle, or the fill deviation is above
/// `accept` (mm).
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
    let mut failed = failed_groups(&out, before, &rim, &owner);
    let measured = plans
        .iter()
        .enumerate()
        .map(|(g, p)| if skip.get(g).copied().unwrap_or(false) { Vec::new() } else { p.clone() })
        .collect::<Vec<_>>();
    distances((mesh, &out), origin, &owner, (folds, &measured), &mut taken);
    let strayed = (0..taken.len()).filter(|&g| taken[g].clean_deviation > accept);
    failed.extend(strayed.filter(|g| !skip.get(*g).copied().unwrap_or(false)));
    failed.sort_unstable();
    failed.dedup();
    let mut kept: Vec<bool> = origin.iter().zip(&dead).filter(|(_, d)| !**d).map(|(o, _)| *o).collect();
    kept.extend(std::iter::repeat_n(false, added));
    let removed_original = (0..dead.len() as u32)
        .filter(|&t| dead.get(t as usize).copied().unwrap_or(false) && origin.get(t as usize).copied().unwrap_or(false))
        .collect();
    Attempt { mesh: out, origin: kept, failed, added, taken, removed_original }
}
