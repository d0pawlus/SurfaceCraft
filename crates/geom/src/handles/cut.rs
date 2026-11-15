//! Cutting the candidates out and filling every loop they leave with the smooth hole fill. A candidate
//! is refused when cutting it would split the surface (its loops are not joined outside it) or leave a
//! piece loose, when one of its holes stays open, when its fill strays from the surface it replaced, or
//! when what it removed is not covered by the result. What was removed is judged by points sampled
//! over its triangles (corners, edge midpoints, centres and a lattice by size), not by its vertices:
//! the wall of a tunnel can be long triangles without a vertex inside.
use super::cells::{Candidate, join, root, sets};
use crate::{Bvh, Cancel, Mesh, Point3, edit, holes};

/// The lattice on a removed triangle has at most this many steps along an edge.
const MAX_LATTICE: usize = 8;

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
    /// The triangles of the mesh before that were cut out.
    pub dead: Vec<bool>,
}

/// The distances an attempt is judged by (mm).
#[derive(Clone, Copy, Debug)]
pub(super) struct Bounds {
    /// A point of a fill or of what was removed farther than this from the other side counts as far.
    pub tolerance: f64,
    /// No point of what was removed may lie farther than this from the result (a quarter of the ball).
    pub far_limit: f64,
}

/// Points over a triangle: its corners, edge midpoints and centre, and a lattice of steps no longer than
/// `spacing` (at most [`MAX_LATTICE`] along an edge).
pub(super) fn samples(t: [Point3; 3], spacing: f64) -> Vec<Point3> {
    let [a, b, c] = t;
    let longest = (b - a).norm().max((c - b).norm()).max((a - c).norm());
    let steps = if spacing > 0.0 && longest.is_finite() {
        ((longest / spacing).ceil() as usize).clamp(2, MAX_LATTICE)
    } else {
        2
    };
    let n = steps as f64;
    let mut out = Vec::with_capacity((steps + 1) * (steps + 2) / 2 + 1);
    for i in 0..=steps {
        for j in 0..=steps - i {
            let (u, v) = (i as f64 / n, j as f64 / n);
            out.push(a + (b - a) * u + (c - a) * v);
        }
    }
    out.push(Point3::from((a.coords + b.coords + c.coords) / 3.0));
    out
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

/// Candidates refused by [`splitting`] taken back one at a time (the first `max` of `refused`): one
/// stays in when, cut together with the others that are in, no candidate's loops come apart. Cutting
/// all candidates at once can cut a band of sponge in two where cutting most of them does not. Returns
/// those taken back (`skip` is cleared for them).
pub(super) fn readmit(
    mesh: &Mesh,
    cands: &[Candidate],
    skip: &mut [bool],
    refused: &[usize],
    max: usize,
) -> Vec<usize> {
    let mut back = Vec::new();
    for &g in refused.iter().take(max) {
        let Some(s) = skip.get_mut(g) else { continue };
        *s = false;
        if splitting(mesh, cands, skip).is_empty() {
            back.push(g);
        } else if let Some(s) = skip.get_mut(g) {
            *s = true;
        }
    }
    back
}

/// The mesh with the candidates not in `skip` cut out and their loops filled, and what failed; `None`
/// when the caller cancelled.
pub(super) fn attempt(mesh: &Mesh, cands: &[Candidate], skip: &[bool], b: Bounds, cancel: &Cancel) -> Option<Attempt> {
    let (owner, dead) = mark(mesh, cands, skip);
    let mut out = mesh.clone();
    let mut k = dead.iter();
    out.triangles.retain(|_| !k.next().copied().unwrap_or(false));
    let before = out.triangles.len();
    let owned = |v: &u32| owner.get(*v as usize).is_some_and(|&o| o != u32::MAX);
    let accept = |_: &Mesh, lp: &[u32]| lp.iter().all(owned);
    holes::fill_holes_filtered(&mut out, FILL_MAX_EDGES, true, true, &accept);
    if cancel.is_cancelled() {
        return None;
    }
    let added = out.triangles.len() - before;
    let mut failed = unfilled(&out, &owner);
    let (fill_distance, far) = fill_distances(mesh, &out, before, (&owner, &dead), (cands.len(), b.tolerance));
    failed.extend(far.iter().enumerate().filter(|(_, f)| **f > FILL_FAR_SHARE).map(|(g, _)| (g, Refusal::FillFar)));
    if cancel.is_cancelled() {
        return None;
    }
    failed.extend(uncovered(mesh, &out, cands, skip, b).into_iter().map(|g| (g, Refusal::Uncovered)));
    // stable: the first reason found for a candidate (unfillable, fill far, uncovered) is the one kept
    failed.sort_by_key(|f| f.0);
    failed.dedup_by_key(|f| f.0);
    Some(Attempt { mesh: out, failed, fill_distance, added, dead })
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
/// used lies in a component of `out` that is not the largest of the component of `work` it came from,
/// or the component of `work` they lay in has no triangle left in `out`.
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
        // a component of the mesh before that has no triangle left was deleted whole
        let vanished = c.vertices.first().is_some_and(|&v| !main.contains_key(&root(&mut before, v)));
        let loose = vanished
            || c.vertices.iter().filter(|&&v| used.get(v as usize).copied().unwrap_or(false)).any(|&v| {
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
pub(super) fn unfilled(out: &Mesh, owner: &[u32]) -> Vec<(usize, Refusal)> {
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

/// The candidates (not skipped) of which more than [`FILL_FAR_SHARE`] of the points sampled over the
/// triangles they removed lie farther than the tolerance from `out`, or any farther than the far limit,
/// or with a vertex that stayed and is pinched (its triangles form more than one fan).
fn uncovered(mesh: &Mesh, out: &Mesh, cands: &[Candidate], skip: &[bool], b: Bounds) -> Vec<usize> {
    let live: Vec<usize> = (0..cands.len()).filter(|g| !skip.get(*g).copied().unwrap_or(true)).collect();
    let bvh = Bvh::build(out);
    let fans = Fans::new(out);
    let bad = crate::par::map_slice(&live, |&g| {
        let Some(c) = cands.get(g) else { return false };
        let points: Vec<Point3> = c
            .triangles
            .iter()
            .filter_map(|&t| mesh.triangle(t as usize))
            .flat_map(|t| samples(t, 2.0 * b.tolerance))
            .collect();
        let (mut far, mut beyond) = (0usize, false);
        for p in &points {
            let d = bvh.closest(p, b.far_limit.max(b.tolerance) * 1.01).map_or(f64::INFINITY, |h| h.distance);
            far += usize::from(d > b.tolerance);
            beyond |= d > b.far_limit;
        }
        let kept = c.vertices.iter().filter(|&&v| fans.used(v));
        beyond || far as f64 > FILL_FAR_SHARE * points.len() as f64 || kept.into_iter().any(|&v| fans.count(out, v) > 1)
    });
    live.into_iter().zip(bad).filter(|(_, b)| *b).map(|(g, _)| g).collect()
}

/// The triangles around every vertex, to count the fans a vertex has.
pub(super) struct Fans {
    start: Vec<usize>,
    tris: Vec<u32>,
}

impl Fans {
    pub(super) fn new(m: &Mesh) -> Self {
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

    /// Does a triangle use `v`?
    fn used(&self, v: u32) -> bool {
        matches!((self.start.get(v as usize), self.start.get(v as usize + 1)), (Some(a), Some(b)) if b > a)
    }

    /// Fans of triangles around `v` (joined through the edges at `v`), in O(d log d) for d triangles:
    /// the (other corner, triangle) pairs are sorted, and triangles that share an other corner share an
    /// edge at `v`.
    pub(super) fn count(&self, m: &Mesh, v: u32) -> usize {
        let (Some(&a), Some(&b)) = (self.start.get(v as usize), self.start.get(v as usize + 1)) else { return 0 };
        let around = self.tris.get(a..b).unwrap_or(&[]);
        let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(2 * around.len());
        for (i, &t) in around.iter().enumerate() {
            for &x in m.triangles.get(t as usize).into_iter().flatten().filter(|&&x| x != v) {
                pairs.push((x, i as u32));
            }
        }
        pairs.sort_unstable();
        let mut s = sets(around.len());
        for w in pairs.windows(2) {
            if let [(x, i), (y, j)] = w
                && x == y
            {
                join(&mut s, *i, *j);
            }
        }
        (0..around.len() as u32).filter(|&i| root(&mut s, i) == i).count()
    }
}

/// The patches of a fill (the triangles of `out` from `from` on), joined through shared edges: the
/// patch of every fill triangle (indices from 0).
fn patches(out: &Mesh, from: usize) -> Vec<u32> {
    let n = out.triangles.len().saturating_sub(from);
    let mut edges: Vec<((u32, u32), u32)> = Vec::with_capacity(3 * n);
    for (k, t) in out.triangles.iter().skip(from).enumerate() {
        for e in 0..3 {
            let (a, b) = (t[e], t[(e + 1) % 3]);
            edges.push(((a.min(b), a.max(b)), k as u32));
        }
    }
    edges.sort_unstable();
    let mut s = sets(n);
    for w in edges.windows(2) {
        if let [(x, i), (y, j)] = w
            && x == y
        {
            join(&mut s, *i, *j);
        }
    }
    (0..n as u32).map(|k| root(&mut s, k)).collect()
}

/// For every patch of the fill, the candidate that owns a corner of it, and room for two counts.
fn patch_owners(
    out: &Mesh,
    from: usize,
    patch: &[u32],
    owner: &[u32],
) -> std::collections::BTreeMap<u32, (u32, usize, usize)> {
    let mut counts = std::collections::BTreeMap::new();
    for (t, &p) in (from..out.triangles.len()).zip(patch) {
        let g = out
            .triangles
            .get(t)
            .into_iter()
            .flatten()
            .find_map(|&v| owner.get(v as usize).copied().filter(|&o| o != u32::MAX));
        if let Some(g) = g {
            counts.entry(p).or_insert((g, 0, 0)).0 = g;
        }
    }
    counts
}

/// Per candidate, the largest distance of a point (vertex or triangle centre) of its fill from the
/// triangles it replaced, and the largest share, over the patches of its fill (one per loop), of the
/// points farther than `bound`: a patch that is mostly far is a cap over the mouth of a real opening.
/// A patch belongs to the candidate that owns a corner of it.
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
    let mut share = vec![0.0f64; n];
    if removed.is_empty() {
        return (largest, share);
    }
    let bvh = Bvh::build(&mesh.submesh(&removed));
    let patch = patches(out, from);
    let mut counts = patch_owners(out, from, &patch, owner);
    let probes: Vec<(u32, Point3)> = (from..out.triangles.len())
        .zip(&patch)
        .flat_map(|(t, &p)| {
            let corners = out.triangle(t).map(|c| c.to_vec()).unwrap_or_default();
            out.face_centroid(t).into_iter().chain(corners).map(move |q| (p, q))
        })
        .collect();
    let hits =
        crate::par::map_slice(&probes, |(_, q)| bvh.closest(q, f64::INFINITY).map_or(f64::INFINITY, |h| h.distance));
    for ((p, _), d) in probes.iter().zip(hits) {
        if let Some(c) = counts.get_mut(p) {
            c.1 += 1;
            c.2 += usize::from(d > bound);
            if let Some(f) = largest.get_mut(c.0 as usize) {
                *f = f.max(d);
            }
        }
    }
    for &(g, all, far) in counts.values() {
        if let Some(s) = share.get_mut(g as usize)
            && all > 0
        {
            *s = s.max(far as f64 / all as f64);
        }
    }
    (largest, share)
}
