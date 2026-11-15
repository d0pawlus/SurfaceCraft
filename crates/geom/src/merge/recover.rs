//! Putting back scanned surface the seam could not cover. A removed triangle whose centre has no
//! facing surface of the result within the tolerance (the seam stayed open there, or a long
//! triangle was removed and nothing replaced it) comes back on its own scan's vertices (each corner
//! the scan's vertex in the result, or that vertex added at exactly its place: nothing moves, and a
//! recovered triangle has the corners of one scan only; it joins another scan only where a corner was
//! welded to that scan before). It crosses and covers nothing, uses no directed edge twice and
//! repeats no triangle's corners; the vertices a refused one needed are taken back. Then the small
//! holes are filled again.
use super::guard::{Guard, Pts};
use super::overlap::COS_FACING;
use super::prep::{NONE, Scan};
use super::union::{Origin, Union, directed, sorted};
use crate::{Bvh, Cancel, Mesh};
use std::collections::HashSet;

/// Triangles tried between two looks at the cancellation flag (each try compares at most
/// `MAX_NEAR` triangles, so the work between two looks is bounded).
const CANCEL_EVERY: usize = 64;

/// What the recovery did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Recovered {
    /// Removed triangles whose centre the seam left without facing result surface within the tolerance.
    pub uncovered: usize,
    /// Of them, put back.
    pub triangles: usize,
    /// Of those, taken back again: pieces of recovered triangles only that joined nothing.
    pub specks_dropped: usize,
    /// Rounds of recovery that put something back.
    pub rounds: usize,
}

/// The removed triangles (not restored) whose centre has no facing result surface within `tol`.
fn uncovered(
    u: &Union,
    scans: &[Scan],
    removed: &[Vec<bool>],
    tol: f64,
    cancel: &Cancel,
) -> Option<Vec<(usize, usize)>> {
    let back: HashSet<(u32, u32)> = u
        .origin
        .iter()
        .filter_map(|o| if let Origin::Restored(s, t) | Origin::Recovered(s, t) = o { Some((*s, *t)) } else { None })
        .collect();
    let bvh = Bvh::build(&Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() });
    let mut out = Vec::new();
    for (s, scan) in scans.iter().enumerate() {
        if cancel.is_cancelled() {
            return None;
        }
        let gone = removed.get(s).map_or(&[][..], Vec::as_slice);
        for t in (0..scan.mesh.triangles.len()).filter(|&t| gone.get(t).copied().unwrap_or(false)) {
            if back.contains(&(s as u32, t as u32)) {
                continue;
            }
            let (Some(c), Some(n)) = (scan.mesh.face_centroid(t), scan.mesh.face_normal(t)) else { continue };
            if bvh.closest_facing(&c, tol, &[n], COS_FACING).is_none() {
                out.push((s, t));
            }
        }
    }
    Some(out)
}

/// The union vertex of corner `v` of scan `s`: its own in the result, or added at its place (and
/// listed in `added`).
fn corner(u: &mut Union, scans: &[Scan], s: usize, v: u32, added: &mut Vec<(usize, u32)>) -> Option<u32> {
    if let Some(x) = u.vertex_of.get(s).and_then(|m| m.get(v as usize)).copied().filter(|&x| x != NONE) {
        return Some(x);
    }
    added.push((s, v));
    super::union::add_vertex(u, scans, s, v)
}

/// Puts back the removed triangles the result does not cover (see the module); `None` when
/// cancelled.
pub(super) fn recover(
    u: &mut Union,
    scans: &[Scan],
    removed: &[Vec<bool>],
    guard: &mut Guard<'_>,
    (tol, cancel): (f64, &Cancel),
) -> Option<Recovered> {
    let todo = uncovered(u, scans, removed, tol, cancel)?;
    let mut rep = Recovered { uncovered: todo.len(), ..Recovered::default() };
    let (mut edges, mut sets) = (u.directed_edges(), u.vertex_sets());
    for (n, (s, t)) in todo.into_iter().enumerate() {
        if n % CANCEL_EVERY == 0 && cancel.is_cancelled() {
            return None;
        }
        let Some(&tri) = scans.get(s).and_then(|sc| sc.mesh.triangles.get(t)) else { continue };
        let (len, mut added) = (u.positions.len(), Vec::new());
        let m = tri.map(|v| corner(u, scans, s, v, &mut added));
        let ok = match m {
            [Some(a), Some(b), Some(c)] => {
                let m = [a, b, c];
                let fresh = a != b && b != c && a != c && !sets.contains(&sorted(m));
                let free = !(0..3).any(|k| edges.contains(&directed(m[k], m[(k + 1) % 3])));
                (fresh && free && guard.restorable(Pts::of(u), m)).then_some(m)
            }
            _ => None,
        };
        let Some(m) = ok else {
            u.rollback(len, &added);
            continue;
        };
        edges.extend((0..3).map(|k| directed(m[k], m[(k + 1) % 3])));
        sets.insert(sorted(m));
        u.triangles.push(m);
        u.origin.push(Origin::Recovered(s as u32, t as u32));
        rep.triangles += 1;
    }
    Some(rep)
}
