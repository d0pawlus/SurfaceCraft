//! Cutting the candidates out and filling every loop they leave with the smooth hole fill. A candidate
//! is refused when cutting it would split the surface (its loops are not joined outside it), when one
//! of its holes stays open, or when its fill strays from the surface it replaced by more than the bound.
use super::cells::{Candidate, join, root, sets};
use crate::{Bvh, Mesh, edit, holes};

/// Longest loop a candidate leaves that is filled (boundary edges).
pub(super) const FILL_MAX_EDGES: usize = 4000;
/// A fill is refused when more than this share of its points (vertices and triangle centres) lies
/// farther than the bound from the surface it replaced. Not all of it can be near: the fill spans the
/// mouth of the tunnel, where there was no surface.
pub(super) const FILL_FAR_SHARE: f64 = 0.25;

/// Why a candidate was not cut.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Refusal {
    /// A hole it left could not be filled.
    Unfillable,
    /// Too much of its fill lies farther from the surface it replaced than the bound.
    FillFar,
    /// Too much of what it removed lies farther from the result than the bound (the fill does not cover
    /// it: a loop that ran from one sheet to another), or a vertex of its loops is pinched.
    Uncovered,
    /// Cutting it left a piece of the surface loose.
    Loosens,
}

/// One attempt at cutting a set of candidates.
pub(super) struct Attempt {
    pub mesh: Mesh,
    pub failed: Vec<(usize, Refusal)>,
    /// Largest distance of the fill of each candidate from the surface it replaced (0 when not cut).
    pub fill_distance: Vec<f64>,
    pub added: usize,
}

/// The owner (candidate index) of every vertex of a candidate that is cut, `u32::MAX` elsewhere, and
/// the triangles that go.
fn mark(mesh: &Mesh, cands: &[Candidate], skip: &[bool]) -> (Vec<u32>, Vec<bool>) {
    let mut owner = vec![u32::MAX; mesh.positions.len()];
    let mut dead = vec![false; mesh.triangles.len()];
    for (g, c) in cands.iter().enumerate().filter(|(g, _)| !skip.get(*g).copied().unwrap_or(true)) {
        for &t in &c.triangles {
            let (Some(d), Some(tri)) = (dead.get_mut(t as usize), mesh.triangles.get(t as usize)) else { continue };
            *d = true;
            for &v in tri {
                if let Some(o) = owner.get_mut(v as usize) {
                    *o = g as u32;
                }
            }
        }
    }
    (owner, dead)
}

/// The candidates (not skipped) whose loops are not all joined by the triangles that stay.
pub(super) fn splitting(mesh: &Mesh, cands: &[Candidate], skip: &[bool]) -> Vec<usize> {
    let (_, dead) = mark(mesh, cands, skip);
    let mut s = sets(mesh.positions.len());
    for (t, tri) in mesh.triangles.iter().enumerate() {
        if !dead.get(t).copied().unwrap_or(true) {
            join(&mut s, tri[0], tri[1]);
            join(&mut s, tri[1], tri[2]);
        }
    }
    let mut out = Vec::new();
    for (g, c) in cands.iter().enumerate().filter(|(g, _)| !skip.get(*g).copied().unwrap_or(true)) {
        let roots: Vec<u32> = c.loops.iter().map(|&v| root(&mut s, v)).collect();
        if roots.windows(2).any(|w| matches!(w, [a, b] if a != b)) {
            out.push(g);
        }
    }
    out
}

/// The mesh with the candidates not in `skip` cut out and their loops filled, and what failed.
pub(super) fn attempt(mesh: &Mesh, cands: &[Candidate], skip: &[bool], bound: f64) -> Attempt {
    let (owner, dead) = mark(mesh, cands, skip);
    let mut out = mesh.clone();
    let mut k = dead.iter();
    out.triangles.retain(|_| !k.next().copied().unwrap_or(false));
    let before = out.triangles.len();
    let owned = |v: &u32| owner.get(*v as usize).is_some_and(|&o| o != u32::MAX);
    let accept = |_: &Mesh, lp: &[u32]| lp.iter().all(owned);
    holes::fill_holes_filtered(&mut out, FILL_MAX_EDGES, true, true, &accept);
    let added = out.triangles.len() - before;
    let mut failed = unfilled(&out, &owner);
    let (fill_distance, far) = fill_distances(mesh, &out, before, (&owner, &dead), (cands.len(), bound));
    failed.extend(far.iter().enumerate().filter(|(_, f)| **f > FILL_FAR_SHARE).map(|(g, _)| (g, Refusal::FillFar)));
    failed.extend(uncovered(mesh, &out, cands, skip, bound).into_iter().map(|g| (g, Refusal::Uncovered)));
    failed.sort_unstable_by_key(|f| f.0);
    failed.dedup_by_key(|f| f.0);
    Attempt { mesh: out, failed, fill_distance, added }
}

/// Components of the triangles of `m` (through shared vertices), as a union-find to read with [`root`].
fn components(m: &Mesh) -> Vec<u32> {
    let mut s = sets(m.positions.len());
    for t in &m.triangles {
        join(&mut s, t[0], t[1]);
        join(&mut s, t[1], t[2]);
    }
    for v in 0..s.len() as u32 {
        root(&mut s, v);
    }
    s
}

/// The candidates (not skipped) that left a piece of the surface loose: a vertex of theirs that is still
/// used lies in a component of `out` that is not the largest of the component of `work` it came from.
pub(super) fn loosened(work: &Mesh, out: &Mesh, cands: &[Candidate], skip: &[bool]) -> Vec<usize> {
    let mut before = components(work);
    let mut after = components(out);
    let mut size: std::collections::BTreeMap<u32, usize> = std::collections::BTreeMap::new();
    let mut origin: std::collections::BTreeMap<u32, u32> = std::collections::BTreeMap::new();
    for t in &out.triangles {
        let r = root(&mut after, t[0]);
        *size.entry(r).or_default() += 1;
        if let Some(&old) = t.iter().find(|&&v| (v as usize) < work.positions.len()) {
            origin.entry(r).or_insert_with(|| root(&mut before, old));
        }
    }
    let mut main: std::collections::BTreeMap<u32, (usize, u32)> = std::collections::BTreeMap::new();
    for (&r, &n) in &size {
        let Some(&b) = origin.get(&r) else { continue };
        let e = main.entry(b).or_insert((n, r));
        if n > e.0 {
            *e = (n, r);
        }
    }
    let mut used = vec![false; out.positions.len()];
    out.triangles.iter().flatten().for_each(|&v| used.get_mut(v as usize).map_or((), |u| *u = true));
    let mut culprits = Vec::new();
    for (g, c) in cands.iter().enumerate().filter(|(g, _)| !skip.get(*g).copied().unwrap_or(true)) {
        let loose = c.vertices.iter().filter(|&&v| used.get(v as usize).copied().unwrap_or(false)).any(|&v| {
            let r = root(&mut after, v);
            origin.get(&r).and_then(|b| main.get(b)).is_some_and(|&(_, m)| m != r)
        });
        if loose {
            culprits.push(g);
        }
    }
    culprits
}

/// The candidates with a vertex on a loop that stayed open.
fn unfilled(out: &Mesh, owner: &[u32]) -> Vec<(usize, Refusal)> {
    let mut failed = Vec::new();
    for lp in edit::boundary_loops(out) {
        for &v in &lp {
            if let Some(&o) = owner.get(v as usize).filter(|&&o| o != u32::MAX) {
                failed.push((o as usize, Refusal::Unfillable));
            }
        }
    }
    failed
}

/// The candidates (not skipped) of which more than [`FILL_FAR_SHARE`] of the vertices that were cut out
/// lie farther than `bound` from `out`, or with a vertex that stayed and is pinched (its triangles
/// form more than one fan).
fn uncovered(mesh: &Mesh, out: &Mesh, cands: &[Candidate], skip: &[bool], bound: f64) -> Vec<usize> {
    let mut used = vec![false; mesh.positions.len().max(out.positions.len())];
    for &v in out.triangles.iter().flatten() {
        if let Some(u) = used.get_mut(v as usize) {
            *u = true;
        }
    }
    let live: Vec<usize> = (0..cands.len()).filter(|g| !skip.get(*g).copied().unwrap_or(true)).collect();
    let bvh = Bvh::build(out);
    let fans = Fans::new(out);
    let bad = crate::par::map_slice(&live, |&g| {
        let Some(c) = cands.get(g) else { return false };
        let (gone, kept): (Vec<u32>, Vec<u32>) =
            c.vertices.iter().partition(|&&v| !used.get(v as usize).copied().unwrap_or(false));
        let far = gone
            .iter()
            .filter_map(|&v| mesh.positions.get(v as usize))
            .filter(|p| bvh.closest(p, bound).is_none())
            .count();
        far as f64 > FILL_FAR_SHARE * gone.len() as f64 || kept.iter().any(|&v| fans.count(out, v) > 1)
    });
    live.into_iter().zip(bad).filter(|(_, b)| *b).map(|(g, _)| g).collect()
}

/// The triangles around every vertex, to count the fans a vertex has.
struct Fans {
    start: Vec<usize>,
    tris: Vec<u32>,
}

impl Fans {
    fn new(m: &Mesh) -> Self {
        let mut start = vec![0usize; m.positions.len() + 1];
        for &v in m.triangles.iter().flatten() {
            if let Some(s) = start.get_mut(v as usize + 1) {
                *s += 1;
            }
        }
        for i in 1..start.len() {
            let prev = start.get(i - 1).copied().unwrap_or(0);
            if let Some(s) = start.get_mut(i) {
                *s += prev;
            }
        }
        let mut fill = start.clone();
        let mut tris = vec![0u32; start.last().copied().unwrap_or(0)];
        for (t, tri) in m.triangles.iter().enumerate() {
            for &v in tri {
                if let Some(slot) = fill.get_mut(v as usize)
                    && let Some(x) = tris.get_mut(*slot)
                {
                    *x = t as u32;
                    *slot += 1;
                }
            }
        }
        Self { start, tris }
    }

    /// Fans of triangles around `v` (joined through the edges at `v`).
    fn count(&self, m: &Mesh, v: u32) -> usize {
        let (Some(&a), Some(&b)) = (self.start.get(v as usize), self.start.get(v as usize + 1)) else { return 0 };
        let around = self.tris.get(a..b).unwrap_or(&[]);
        let others = |t: u32| -> Vec<u32> {
            m.triangles.get(t as usize).map(|c| c.iter().copied().filter(|&x| x != v).collect()).unwrap_or_default()
        };
        let mut s = sets(around.len());
        for i in 0..around.len() {
            for j in i + 1..around.len() {
                let (Some(&ti), Some(&tj)) = (around.get(i), around.get(j)) else { continue };
                let (oi, oj) = (others(ti), others(tj));
                if oi.iter().any(|x| oj.contains(x)) {
                    join(&mut s, i as u32, j as u32);
                }
            }
        }
        (0..around.len() as u32).filter(|&i| root(&mut s, i) == i).count()
    }
}

/// Per candidate, the largest distance of a point (vertex or triangle centre) of its fill (the
/// triangles of `out` from `from` on) from the triangles it replaced, and the share of those points
/// farther than `bound`. A point belongs to the candidate whose removed triangle is nearest to it.
fn fill_distances(
    mesh: &Mesh,
    out: &Mesh,
    from: usize,
    (owner, dead): (&[u32], &[bool]),
    (n, bound): (usize, f64),
) -> (Vec<f64>, Vec<f64>) {
    let removed: Vec<u32> =
        (0..dead.len() as u32).filter(|&t| dead.get(t as usize).copied().unwrap_or(false)).collect();
    let mut largest = vec![0.0f64; n];
    let mut counts = vec![(0usize, 0usize); n];
    if removed.is_empty() {
        return (largest, vec![0.0; n]);
    }
    let cut = mesh.submesh(&removed);
    let tag: Vec<u32> = removed
        .iter()
        .map(|&t| mesh.triangles.get(t as usize).and_then(|c| owner.get(c[0] as usize)).copied().unwrap_or(u32::MAX))
        .collect();
    let bvh = Bvh::build(&cut);
    let probes: Vec<crate::Point3> = (from..out.triangles.len())
        .flat_map(|t| {
            let c = out.face_centroid(t);
            let corners = out.triangle(t).map(|p| p.to_vec()).unwrap_or_default();
            c.into_iter().chain(corners)
        })
        .collect();
    let hits = crate::par::map_slice(&probes, |p| bvh.closest(p, f64::INFINITY).map(|h| (h.triangle, h.distance)));
    for (t, d) in hits.into_iter().flatten() {
        let Some(&g) = tag.get(t) else { continue };
        if let (Some(f), Some(c)) = (largest.get_mut(g as usize), counts.get_mut(g as usize)) {
            *f = f.max(d);
            c.0 += 1;
            c.1 += usize::from(d > bound);
        }
    }
    let share = counts.iter().map(|&(all, far)| if all == 0 { 0.0 } else { far as f64 / all as f64 }).collect();
    (largest, share)
}
