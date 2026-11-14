//! What a removal does to the scan, measured against the surface that replaced it.
use super::detect::Site;
use crate::{Bvh, Mesh, Point3};
use std::collections::BTreeSet;

/// What was taken out for one group.
#[derive(Clone, Debug, Default)]
pub(super) struct Taken {
    pub triangles: usize,
    /// Area of the scan's own triangles taken out (patches of earlier hole filling do not count).
    pub area: f64,
    /// Largest distance of a vertex of the fold that is gone from the new surface (mm).
    pub height: f64,
    /// Largest distance of a vertex or triangle centre of the clean surface that went with the margin, from the
    /// new surface; `clean_samples` of them were measured.
    pub clean_deviation: f64,
    pub clean_samples: usize,
    /// The same for what touches the fold as well (without the vertices of the fold itself).
    pub ring_deviation: f64,
    /// Positions of the vertices of the fold that the scan recorded.
    pub fold_points: Vec<Point3>,
}

impl Taken {
    /// Does the fill stray from what it replaced by more than `accept`? A group with clean surface around it is
    /// judged by that; a group whose margin is all tangled with the fold (a few triangles, one ring) by what is
    /// left of the ring: a triangle with a vertex of the fold is bent by at most a third of the fold's height.
    pub(super) fn strays(&self, accept: f64) -> bool {
        if self.clean_samples > 0 {
            self.clean_deviation > accept
        } else {
            self.ring_deviation - self.height / 3.0 > accept
        }
    }
}

fn scan_vertices(mesh: &Mesh, origin: &[bool]) -> Vec<bool> {
    let mut scan = vec![false; mesh.positions.len()];
    for c in mesh.triangles.iter().zip(origin).filter(|(_, o)| **o).map(|(c, _)| c) {
        for &v in c {
            if let Some(x) = scan.get_mut(v as usize) {
                *x = true;
            }
        }
    }
    scan
}

fn vertices_of(mesh: &Mesh, triangles: impl Iterator<Item = u32>) -> BTreeSet<u32> {
    triangles.filter_map(|t| mesh.triangles.get(t as usize)).flatten().copied().collect()
}

/// Where the mesh before and the one after differ for each group: the vertices of a fold that went give the
/// height; the vertices and triangle centres of the clean surface that went with a margin the deviation.
/// Only what the scan recorded counts (`origin`). What touches a vertex of the fold is tangled with it.
pub(super) fn measure(
    (before, after): (&Mesh, &Mesh),
    origin: &[bool],
    owner: &[u32],
    (folds, plans): (&[&Site], &[Vec<u32>]),
    taken: &mut [Taken],
) {
    let scan = scan_vertices(before, origin);
    let mut used = vec![false; before.positions.len()];
    for &v in after.triangles.iter().flatten() {
        if let Some(u) = used.get_mut(v as usize) {
            *u = true;
        }
    }
    let bvh = Bvh::build(after);
    let distance = |p: &Point3| bvh.closest(p, f64::MAX).map(|h| h.distance);
    for (g, (site, plan)) in folds.iter().zip(plans).enumerate() {
        if plan.is_empty() {
            continue;
        }
        let fold = vertices_of(before, site.triangles.iter().copied());
        let touching: BTreeSet<u32> = vertices_of(
            before,
            plan.iter()
                .copied()
                .filter(|&t| before.triangles.get(t as usize).is_some_and(|c| c.iter().any(|v| fold.contains(v)))),
        );
        let own: BTreeSet<u32> = site.triangles.iter().copied().collect();
        let Some(slot) = taken.get_mut(g) else { continue };
        slot.fold_points = fold
            .iter()
            .filter(|&&v| scan.get(v as usize).copied().unwrap_or(false))
            .filter_map(|&v| before.positions.get(v as usize).copied())
            .collect();
        for v in vertices_of(before, plan.iter().copied()) {
            let (Some(&o), Some(false), Some(true), Some(p)) = (
                owner.get(v as usize),
                used.get(v as usize).copied(),
                scan.get(v as usize).copied(),
                before.positions.get(v as usize),
            ) else {
                continue;
            };
            let Some(d) = distance(p).filter(|_| o as usize == g) else { continue };
            if fold.contains(&v) {
                slot.height = slot.height.max(d);
            } else {
                slot.ring_deviation = slot.ring_deviation.max(d);
                if !touching.contains(&v) {
                    slot.clean_deviation = slot.clean_deviation.max(d);
                    slot.clean_samples += 1;
                }
            }
        }
        for &t in plan.iter().filter(|t| !own.contains(t) && origin.get(**t as usize).copied().unwrap_or(false)) {
            let Some(d) = before.face_centroid(t as usize).and_then(|c| distance(&c)) else { continue };
            slot.ring_deviation = slot.ring_deviation.max(d);
            let tangled = before.triangles.get(t as usize).is_some_and(|c| c.iter().any(|v| fold.contains(v)));
            if !tangled {
                slot.clean_deviation = slot.clean_deviation.max(d);
                slot.clean_samples += 1;
            }
        }
    }
}

/// How far all the scan's material that was taken out (`removed`, a soup of triangles; `fold_points`, the
/// vertices of the folds) stands from the surface `after` that is left: `(height of the folds, deviation of
/// everything removed)`, both against the FINAL surface, however many rounds it took.
pub(super) fn against_final(after: &Mesh, removed: &Mesh, fold_points: &[Point3]) -> (f64, f64) {
    let bvh = Bvh::build(after);
    let far = |p: &Point3| bvh.closest(p, f64::MAX).map_or(0.0, |h| h.distance);
    let height = fold_points.iter().map(far).fold(0.0, f64::max);
    let vertices = removed.positions.iter().map(far).fold(0.0, f64::max);
    let centres =
        (0..removed.triangle_count()).filter_map(|t| removed.face_centroid(t)).map(|c| far(&c)).fold(0.0, f64::max);
    (height, vertices.max(centres).max(height))
}
