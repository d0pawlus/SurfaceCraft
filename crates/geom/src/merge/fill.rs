//! The small holes the pivoting leaves in the seam (where two fans of the band touch at a vertex,
//! or a ball found no free place): loops of open seam edges of at most [`SMALL_LOOP_EDGES`] edges,
//! filled by ear clipping in their mean plane (no new vertex), each triangle through the same guard
//! as the pivoting (on scanned surface, crossing nothing). Loops with an edge of a scan's own border
//! are openings of the part and are never filled.
use super::guard::{Guard, Pts};
use super::prep::Scan;
use super::union::{Origin, Union, directed};
use crate::Mesh;
use std::collections::HashSet;

/// Loops (or pieces) handled between two looks at the cancellation flag.
const CANCEL_EVERY: usize = 256;
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
    let mut at: std::collections::HashMap<u32, usize> = std::collections::HashMap::new();
    for v in walk {
        if let Some(&pos) = at.get(&v) {
            let part = stack.split_off(pos);
            for w in &part {
                at.remove(w);
            }
            out.push(part);
        }
        at.insert(v, stack.len());
        stack.push(v);
    }
    out.push(stack);
    out.retain(|l| l.len() >= 3);
    out
}

/// Fills the small seam loops of `u` through `guard`. Returns (loops filled, triangles added, the
/// small loops left open).
pub(super) fn fill_small(
    u: &mut Union,
    scans: &[Scan],
    guard: &mut Guard<'_>,
    cancel: &crate::Cancel,
) -> Option<(usize, usize, Vec<Vec<u32>>)> {
    let loops: Vec<Vec<u32>> = seam_loops(u, scans).into_iter().filter(|l| l.len() <= SMALL_LOOP_EDGES).collect();
    let mut left = Vec::new();
    if loops.is_empty() {
        return Some((0, 0, left));
    }
    let mut taken: HashSet<u64> =
        u.triangles.iter().flat_map(|t| (0..3).map(move |k| undirected(t[k], t[(k + 1) % 3]))).collect();
    let mut work = Mesh { positions: u.positions.clone(), triangles: Vec::new() };
    let (mut filled, mut added) = (0, 0);
    for (n, lp) in loops.into_iter().enumerate() {
        if n % CANCEL_EVERY == 0 && cancel.is_cancelled() {
            return None;
        }
        let rev: Vec<u32> = lp.iter().rev().copied().collect();
        let Some(patch) = patch_for(&mut work, &rev, &taken, u, guard) else {
            left.push(lp);
            continue;
        };
        taken.extend(patch.iter().flat_map(|t| (0..3).map(move |k| undirected(t[k], t[(k + 1) % 3]))));
        added += patch.len();
        filled += 1;
        u.origin.extend(std::iter::repeat_n(Origin::Seam, patch.len()));
        u.triangles.extend(patch);
    }
    Some((filled, added, left))
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
    tries.into_iter().find(|p| diagonal_free(p) && guard.accept_all(Pts::of(u), p))
}

/// The fans of triangles `tris` around vertex `v` (triangles joined across the edges through `v`).
fn fans(u: &Union, v: u32, tris: &[usize]) -> usize {
    fn root(p: &mut [usize], mut x: usize) -> usize {
        while let Some(&up) = p.get(x).filter(|&&up| up != x) {
            let grand = p.get(up).copied().unwrap_or(up);
            if let Some(slot) = p.get_mut(x) {
                *slot = grand;
            }
            x = grand;
        }
        x
    }
    let mut parent: Vec<usize> = (0..tris.len()).collect();
    // two triangles of the fan are joined when they share a vertex besides `v` (an edge through `v`)
    let mut first: std::collections::HashMap<u32, usize> = std::collections::HashMap::new();
    for (i, &t) in tris.iter().enumerate() {
        for &w in u.triangles.get(t).into_iter().flatten().filter(|&&w| w != v) {
            match first.get(&w) {
                Some(&j) => {
                    let (ri, rj) = (root(&mut parent, i), root(&mut parent, j));
                    if let Some(slot) = parent.get_mut(ri) {
                        *slot = rj;
                    }
                }
                None => {
                    first.insert(w, i);
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
        if !tris.iter().any(seam) {
            continue;
        }
        // a pinch the scans already had (their own triangles alone form several fans) is not the
        // seam's: it is left as it is
        let own: Vec<usize> = tris.iter().copied().filter(|i| !seam(i)).collect();
        if fans(u, v as u32, tris) > fans(u, v as u32, &own).max(1) {
            for &i in tris.iter().filter(|i| seam(i)) {
                if let Some(d) = drop.get_mut(i) {
                    *d = true;
                }
            }
        }
    }
    take_back(u, guard, &drop)
}

/// Takes back the seam triangles at the vertices of the small loops that could not be filled, so
/// that each hole grows by a ring and can be filled as a whole (a twisted corner of the band, where
/// no triangle over the small loop faces the right way). Returns how many.
pub(super) fn widen(u: &mut Union, guard: &mut Guard<'_>, left: &[Vec<u32>]) -> usize {
    let mut at = vec![false; u.positions.len()];
    for v in left.iter().flatten() {
        if let Some(f) = at.get_mut(*v as usize) {
            *f = true;
        }
    }
    let drop: Vec<bool> = u
        .triangles
        .iter()
        .zip(&u.origin)
        .map(|(t, o)| matches!(o, Origin::Seam) && t.iter().any(|v| at.get(*v as usize).copied().unwrap_or(false)))
        .collect();
    take_back(u, guard, &drop)
}

/// Removes the triangles `drop` flags from `u` and from the guard's record.
fn take_back(u: &mut Union, guard: &mut Guard<'_>, drop: &[bool]) -> usize {
    let gone: Vec<[u32; 3]> = u.triangles.iter().zip(drop).filter(|(_, d)| **d).map(|(t, _)| *t).collect();
    for &t in &gone {
        guard.forget(Pts::of(u), t);
    }
    let mut keep = drop.iter().map(|d| !d);
    u.triangles.retain(|_| keep.next().unwrap_or(true));
    let mut keep = drop.iter().map(|d| !d);
    u.origin.retain(|_| keep.next().unwrap_or(true));
    gone.len()
}

/// A piece the recovery put back that joined nothing: only recovered triangles, less than
/// [`SPECK_SHARE`] of the area of the result. A floating fragment helps no wrap; its surface counts
/// as removed (and uncovered) in the checks.
fn recovered_speck(u: &Union, mesh: &Mesh, part: &[u32]) -> bool {
    let all_recovered = part.iter().all(|&t| matches!(u.origin.get(t as usize), Some(Origin::Recovered(..))));
    let area: f64 = part.iter().map(|&t| mesh.face_area(t as usize)).sum();
    all_recovered && area < SPECK_SHARE * mesh.area()
}

/// Takes back the pieces made only of recovered triangles that joined nothing (see
/// [`recovered_speck`]), before the holes are filled again (a speck in a hole keeps the fill out).
/// Returns how many triangles.
pub(super) fn drop_specks(u: &mut Union, guard: &mut Guard<'_>) -> usize {
    let mesh = Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() };
    let mut drop = vec![false; u.triangles.len()];
    for part in crate::edit::components(&mesh).iter().filter(|p| recovered_speck(u, &mesh, p)) {
        for &t in part {
            if let Some(d) = drop.get_mut(t as usize) {
                *d = true;
            }
        }
    }
    take_back(u, guard, &drop)
}

/// Pieces made only of recovered triangles below this share of the result's area are dropped.
pub const SPECK_SHARE: f64 = 0.01;

/// Drops the pieces of the union other than the largest that lie wholly (the centre of every
/// triangle) within `tol` of facing surface of the rest: a stray triangle of a scan left on the
/// merged surface, a second layer. Returns the kept scan triangles dropped as `(scan, triangle)`, the
/// number dropped and of them the recovered ones. Runs after the guard's last use: the guard is not
/// told (nothing is accepted after it), the vertices left unused stay out of the result's mesh.
pub(super) fn drop_islands(u: &mut Union, tol: f64, cancel: &crate::Cancel) -> Option<(Vec<(u32, u32)>, usize, usize)> {
    let mesh = Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() };
    let parts = crate::edit::components(&mesh);
    if parts.len() < 2 {
        return Some((Vec::new(), 0, 0));
    }
    let bvh = crate::Bvh::build(&mesh);
    let mut drop = vec![false; u.triangles.len()];
    for (n, part) in parts.iter().skip(1).enumerate() {
        if n % CANCEL_EVERY == 0 && cancel.is_cancelled() {
            return None;
        }
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
    let recovered = u.origin.iter().zip(&drop).filter(|(o, d)| **d && matches!(o, Origin::Recovered(..))).count();
    let mut keep = drop.iter().map(|d| !d);
    u.triangles.retain(|_| keep.next().unwrap_or(true));
    let mut keep = drop.iter().map(|d| !d);
    u.origin.retain(|_| keep.next().unwrap_or(true));
    Some((kept, count, recovered))
}
