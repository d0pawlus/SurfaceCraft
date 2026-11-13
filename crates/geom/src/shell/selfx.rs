//! Crossings of the scan with itself (a fold of the sheet where two of its triangles pass through each
//! other, a spike that pierces its neighbours): no offset can be clean there, the sheet that is offset
//! is itself tangled. The vertices on such crossings, and their neighbours, are smoothed
//! (Laplacian smoothing, Taubin and others: moving a vertex towards the mean of its neighbours
//! removes spikes and unfolds small folds) by a bounded distance until the sheet no longer crosses
//! itself, before anything is offset. Vertices of the rim are not moved (the rim of the scan stays where
//! it is), and the largest shift is reported: the judge measures against the scan as it was.
use super::crossing::{self, Pairs};
use super::topology::{Csr, Work};
use crate::{Bvh, Mesh, Point3, Vec3, par};

/// Passes of smoothing at most.
const MAX_PASSES: usize = 12;
/// A vertex moves at most this many median edges from where the scan has it.
const MAX_SHIFT_EDGES: f64 = 1.0;
/// Share of the way to the mean of the neighbours a vertex moves in a pass.
const STEP: f64 = 0.5;
/// Rings of neighbours smoothed around the vertices on a crossing.
const RINGS: usize = 2;
/// A row of the adjacency is read up to this many entries.
const MAX_ROW: usize = 64;

/// The vertices on a crossing among the triangles `ids` of the scan sheet (ascending).
fn crossing_vertices(mesh: &Mesh, bvh: &Bvh, ids: &[u32]) -> Vec<u32> {
    let hits: Vec<Vec<u32>> = par::map(ids.len(), |k| {
        let id = ids.get(k).copied().unwrap_or(0) as usize;
        let Some(xi) = mesh.triangles.get(id) else { return Vec::new() };
        let found = crossing::partners(mesh, bvh, id, Pairs::Exact);
        if found.crossing.is_empty() {
            return Vec::new();
        }
        let mut out: Vec<u32> = xi.to_vec();
        out.extend(found.crossing.iter().filter_map(|&o| mesh.triangles.get(o as usize)).flatten().copied());
        out
    });
    let mut out: Vec<u32> = hits.into_iter().flatten().collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// `seed` and its neighbours up to [`RINGS`] rings, ascending.
fn region(adjacent: &Csr, seed: &[u32]) -> Vec<u32> {
    let mut all: Vec<u32> = seed.to_vec();
    let mut frontier: Vec<u32> = seed.to_vec();
    for _ in 0..RINGS {
        let next: Vec<u32> =
            frontier.iter().flat_map(|&v| adjacent.row(v as usize).iter().copied().take(MAX_ROW)).collect();
        all.extend_from_slice(&next);
        frontier = next;
    }
    all.sort_unstable();
    all.dedup();
    all
}

/// One Jacobi step of smoothing over `region`, each vertex kept within `reach` of where it started.
fn smoothed(
    positions: &[Point3],
    start: &[Point3],
    adjacent: &Csr,
    region: &[u32],
    (reach, pinned): (f64, &[bool]),
) -> Vec<Point3> {
    let mut out = positions.to_vec();
    for &v in region.iter().filter(|&&v| !pinned.get(v as usize).copied().unwrap_or(true)) {
        let ring: Vec<&Point3> =
            adjacent.row(v as usize).iter().take(MAX_ROW).filter_map(|&w| positions.get(w as usize)).collect();
        let (Some(p), Some(home), true) = (positions.get(v as usize), start.get(v as usize), !ring.is_empty()) else {
            continue;
        };
        let mean = ring.iter().map(|q| q.coords).sum::<Vec3>() / ring.len() as f64;
        let moved = Point3::from(p.coords + (mean - p.coords) * STEP);
        let offset = moved - home;
        let kept = if offset.norm() > reach { home + offset * (reach / offset.norm()) } else { moved };
        if let Some(slot) = out.get_mut(v as usize) {
            *slot = kept;
        }
    }
    out
}

/// What the repair did.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(super) struct Repair {
    /// Vertices that moved.
    pub vertices: usize,
    /// The largest distance a vertex moved (mm).
    pub largest_shift: f64,
}

/// Smooths the scan where it crosses itself, leaving the vertices flagged in `rim` where they are. What
/// still crosses afterwards is left to the rest of the thickening (it ends up as unresolved vertices).
pub(super) fn smooth_scan_crossings(work: &mut Work, adjacent: &Csr, median_edge: f64, rim: &[bool]) -> Repair {
    let reach = MAX_SHIFT_EDGES * median_edge;
    let start = work.positions.clone();
    let mut moved: Vec<u32> = Vec::new();
    let mut ids: Vec<u32> = (0..work.triangles.len() as u32).collect();
    for _ in 0..MAX_PASSES {
        let mesh = Mesh { positions: work.positions.clone(), triangles: work.triangles.clone() };
        let bvh = Bvh::build(&mesh);
        let bad = crossing_vertices(&mesh, &bvh, &ids);
        if bad.is_empty() {
            break;
        }
        let area = region(adjacent, &bad);
        work.positions = smoothed(&work.positions, &start, adjacent, &area, (reach, rim));
        moved.extend_from_slice(&area);
        let pairs: Vec<(u32, u32)> =
            work.triangles.iter().enumerate().flat_map(|(i, t)| t.iter().map(move |&v| (v, i as u32))).collect();
        let around = Csr::from_pairs(work.positions.len(), &pairs);
        let mut touched: Vec<u32> = area.iter().flat_map(|&v| around.row(v as usize).iter().copied()).collect();
        touched.sort_unstable();
        touched.dedup();
        ids = touched;
    }
    let shifts: Vec<f64> = work.positions.iter().zip(&start).map(|(p, q)| (p - q).norm()).collect();
    Repair {
        vertices: shifts.iter().filter(|&&d| d > 0.0).count(),
        largest_shift: shifts.iter().copied().fold(0.0, f64::max),
    }
}
