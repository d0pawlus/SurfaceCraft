//! The small holes the pivoting leaves in the seam (where two fans of the band touch at a vertex,
//! or a ball found no free place): loops of open seam edges of at most [`SMALL_LOOP_EDGES`] edges,
//! filled by ear clipping in their mean plane (no new vertex), each triangle through the same guard
//! as the pivoting (on scanned surface, crossing nothing). Loops with an edge of a scan's own border
//! are openings of the part and are never filled.
use super::prep::Scan;
use super::seam::Guard;
use super::union::{Origin, Union, directed};
use crate::Mesh;
use std::collections::HashSet;

/// Loops of open seam edges up to this many edges are filled.
pub const SMALL_LOOP_EDGES: usize = 64;

fn undirected(a: u32, b: u32) -> u64 {
    directed(a.min(b), a.max(b))
}

/// Closed walks over the open seam edges (in the direction of their triangles), each split into
/// simple loops where it passes a vertex twice.
fn seam_loops(u: &Union, scans: &[Scan]) -> Vec<Vec<u32>> {
    let mut edges: Vec<(u32, u32)> = u.front(scans).iter().map(|f| (f[0], f[1])).collect();
    edges.sort_unstable();
    let mut used = vec![false; edges.len()];
    let mut out = Vec::new();
    for first in 0..edges.len() {
        if used.get(first).copied().unwrap_or(true) {
            continue;
        }
        let Some(&(start, mut at)) = edges.get(first) else { continue };
        used[first] = true;
        let mut walk = vec![start];
        let mut closed = false;
        for _ in 0..edges.len() {
            if at == start {
                closed = true;
                break;
            }
            walk.push(at);
            let from = edges.partition_point(|e| e.0 < at);
            let next = (from..edges.len()).take_while(|&i| edges[i].0 == at).find(|&i| !used[i]);
            let Some(i) = next else { break };
            used[i] = true;
            at = edges[i].1;
        }
        if closed {
            out.extend(simple(walk));
        }
    }
    out
}

/// A closed walk split into the simple loops it is made of.
fn simple(walk: Vec<u32>) -> Vec<Vec<u32>> {
    let mut out = Vec::new();
    let mut stack: Vec<u32> = Vec::with_capacity(walk.len());
    for v in walk {
        if let Some(pos) = stack.iter().position(|&w| w == v) {
            out.push(stack.split_off(pos));
        }
        stack.push(v);
    }
    out.push(stack);
    out.retain(|l| l.len() >= 3);
    out
}

/// Fills the small seam loops of `u` through `guard`. Returns (loops filled, triangles added).
pub(super) fn fill_small(u: &mut Union, scans: &[Scan], guard: &mut Guard<'_>) -> (usize, usize) {
    let loops: Vec<Vec<u32>> = seam_loops(u, scans).into_iter().filter(|l| l.len() <= SMALL_LOOP_EDGES).collect();
    if loops.is_empty() {
        return (0, 0);
    }
    let mut taken: HashSet<u64> =
        u.triangles.iter().flat_map(|t| (0..3).map(move |k| undirected(t[k], t[(k + 1) % 3]))).collect();
    let mut work = Mesh { positions: u.positions.clone(), triangles: Vec::new() };
    let (mut filled, mut added) = (0, 0);
    for lp in loops {
        let rev: Vec<u32> = lp.iter().rev().copied().collect();
        let Some(patch) = patch_for(&mut work, &rev, &taken, u, guard) else { continue };
        taken.extend(patch.iter().flat_map(|t| (0..3).map(move |k| undirected(t[k], t[(k + 1) % 3]))));
        added += patch.len();
        filled += 1;
        u.origin.extend(std::iter::repeat_n(Origin::Seam, patch.len()));
        u.triangles.extend(patch);
    }
    (filled, added)
}

/// A fan of the loop `lp` from its vertex `k`.
fn fan(lp: &[u32], k: usize) -> Vec<[u32; 3]> {
    let n = lp.len();
    (1..n - 1).filter_map(|i| Some([*lp.get(k)?, *lp.get((k + i) % n)?, *lp.get((k + i + 1) % n)?])).collect()
}

/// The triangles that fill the loop `lp` (in the fill's direction): the ear clipping, else the first
/// fan from one of its vertices, whose triangles all pass `guard` and use no edge of the mesh as a
/// diagonal.
fn patch_for(
    work: &mut Mesh,
    lp: &[u32],
    taken: &HashSet<u64>,
    u: &Union,
    guard: &mut Guard<'_>,
) -> Option<Vec<[u32; 3]>> {
    work.triangles.clear();
    let mut tries: Vec<Vec<[u32; 3]>> = Vec::with_capacity(lp.len() + 1);
    if crate::ear::ear_clip_avoiding(work, lp, &|a, b| taken.contains(&undirected(a, b))) {
        tries.push(std::mem::take(&mut work.triangles));
    }
    tries.extend((0..lp.len()).map(|k| fan(lp, k)));
    let boundary: HashSet<u64> = lp.iter().zip(lp.iter().cycle().skip(1)).map(|(a, b)| undirected(*a, *b)).collect();
    // every boundary edge once, every diagonal twice (inside the patch) and not an edge of the mesh
    let diagonal_free = |p: &[[u32; 3]]| {
        let mut count: std::collections::BTreeMap<u64, usize> = std::collections::BTreeMap::new();
        for t in p {
            for k in 0..3 {
                *count.entry(undirected(t[k], t[(k + 1) % 3])).or_default() += 1;
            }
        }
        count.iter().all(|(e, &n)| if boundary.contains(e) { n == 1 } else { n == 2 && !taken.contains(e) })
    };
    tries.into_iter().find(|p| diagonal_free(p) && guard.accept_all(&u.positions, p))
}

/// The fans of triangles `tris` around vertex `v` (triangles joined across the edges through `v`).
fn fans(u: &Union, v: u32, tris: &[usize]) -> usize {
    let mut parent: Vec<usize> = (0..tris.len()).collect();
    fn root(p: &mut [usize], mut x: usize) -> usize {
        while let Some(&up) = p.get(x).filter(|&&up| up != x) {
            x = up;
        }
        x
    }
    let others = |i: usize| u.triangles.get(i).map(|t| t.iter().copied().filter(|&w| w != v).collect::<Vec<u32>>());
    for i in 0..tris.len() {
        for j in i + 1..tris.len() {
            let (Some(a), Some(b)) = (others(tris[i]), others(tris[j])) else { continue };
            if a.iter().any(|w| b.contains(w)) {
                let (ri, rj) = (root(&mut parent, i), root(&mut parent, j));
                if let Some(s) = parent.get_mut(ri) {
                    *s = rj;
                }
            }
        }
    }
    (0..tris.len()).filter(|&i| root(&mut parent, i) == i).count()
}

/// Takes back every seam triangle at a vertex where the triangles form more than one fan (a pinch
/// the pivoting made), so that the hole around it can be filled as one loop. Returns how many.
pub(super) fn unpinch(u: &mut Union, guard: &mut Guard<'_>) -> usize {
    let mut at: Vec<Vec<usize>> = vec![Vec::new(); u.positions.len()];
    for (i, t) in u.triangles.iter().enumerate() {
        for &v in t {
            if let Some(list) = at.get_mut(v as usize) {
                list.push(i);
            }
        }
    }
    let seam = |i: &usize| matches!(u.origin.get(*i), Some(Origin::Seam));
    let mut drop = vec![false; u.triangles.len()];
    for (v, tris) in at.iter().enumerate() {
        if tris.iter().any(seam) && fans(u, v as u32, tris) > 1 {
            for &i in tris.iter().filter(|i| seam(i)) {
                if let Some(d) = drop.get_mut(i) {
                    *d = true;
                }
            }
        }
    }
    let gone: Vec<[u32; 3]> = u.triangles.iter().zip(&drop).filter(|(_, d)| **d).map(|(t, _)| *t).collect();
    for &t in &gone {
        guard.forget(&u.positions, t);
    }
    let mut keep = drop.iter().map(|d| !d);
    u.triangles.retain(|_| keep.next().unwrap_or(true));
    let mut keep = drop.iter().map(|d| !d);
    u.origin.retain(|_| keep.next().unwrap_or(true));
    gone.len()
}

/// Drops the pieces of the union other than the largest that lie wholly (the centre of every
/// triangle) within `tol` of facing surface of the rest: a stray triangle of a scan left on the
/// merged surface, a second layer. Returns the kept scan triangles dropped as `(scan, triangle)`.
pub(super) fn drop_islands(u: &mut Union, tol: f64) -> (Vec<(u32, u32)>, usize) {
    let mesh = Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() };
    let parts = crate::edit::components(&mesh);
    if parts.len() < 2 {
        return (Vec::new(), 0);
    }
    let bvh = crate::Bvh::build(&mesh);
    let mut drop = vec![false; u.triangles.len()];
    for part in parts.iter().skip(1) {
        let mine: HashSet<u32> = part.iter().copied().collect();
        let on_rest = part.iter().all(|&t| {
            let (Some(c), Some(n)) = (mesh.face_centroid(t as usize), mesh.face_normal(t as usize)) else {
                return false;
            };
            let keep = |o: usize| {
                !mine.contains(&(o as u32))
                    && mesh.face_normal(o).is_some_and(|m| m.dot(&n) >= super::overlap::COS_FACING)
            };
            bvh.closest_filtered(&c, tol, &keep).is_some()
        });
        if on_rest {
            for &t in part {
                if let Some(d) = drop.get_mut(t as usize) {
                    *d = true;
                }
            }
        }
    }
    let kept: Vec<(u32, u32)> = u
        .origin
        .iter()
        .zip(&drop)
        .filter(|(_, d)| **d)
        .filter_map(|(o, _)| if let Origin::Kept(s, t) = o { Some((*s, *t)) } else { None })
        .collect();
    let count = drop.iter().filter(|d| **d).count();
    let mut keep = drop.iter().map(|d| !d);
    u.triangles.retain(|_| keep.next().unwrap_or(true));
    let mut keep = drop.iter().map(|d| !d);
    u.origin.retain(|_| keep.next().unwrap_or(true));
    (kept, count)
}
