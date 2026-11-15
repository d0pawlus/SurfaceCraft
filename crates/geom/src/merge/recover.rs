//! Putting back scanned surface the seam could not cover. A removed triangle whose centre has no
//! facing surface of the result within the tolerance (the seam stayed open there, or a long
//! triangle was removed and nothing replaced it) comes back on its own scan's vertices (each corner
//! the scan's vertex in the result, or that vertex added at exactly its place: nothing moves and no
//! two scans are joined here). It passes the guard of the seam (on scanned surface, crossing and
//! covering nothing) and uses no directed edge twice; then the small holes are filled again.
use super::overlap::COS_FACING;
use super::prep::{NONE, Scan};
use super::seam::Guard;
use super::union::{Origin, Union, directed};
use crate::{Bvh, Mesh, Vec3};
use std::collections::HashSet;

/// What the recovery did.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Recovered {
    /// Removed triangles whose centre the seam left without facing result surface within the tolerance.
    pub uncovered: usize,
    /// Of them, put back.
    pub triangles: usize,
}

/// The removed triangles (not restored) whose centre has no facing result surface within `tol`.
fn uncovered(u: &Union, scans: &[Scan], removed: &[Vec<bool>], tol: f64) -> Vec<(usize, usize)> {
    let back: HashSet<(u32, u32)> = u
        .origin
        .iter()
        .filter_map(|o| if let Origin::Restored(s, t) | Origin::Recovered(s, t) = o { Some((*s, *t)) } else { None })
        .collect();
    let bvh = Bvh::build(&Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() });
    let mut out = Vec::new();
    for (s, scan) in scans.iter().enumerate() {
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
    out
}

/// The union vertex of corner `v` of scan `s`: its own in the result, or added at its place.
fn corner(u: &mut Union, scans: &[Scan], s: usize, v: u32) -> Option<u32> {
    if let Some(x) = u.vertex_of.get(s).and_then(|m| m.get(v as usize)).copied().filter(|&x| x != NONE) {
        return Some(x);
    }
    let scan = scans.get(s)?;
    let p = *scan.mesh.positions.get(v as usize)?;
    let x = u.positions.len() as u32;
    u.positions.push(p);
    u.normals.push(scan.normals.get(v as usize).copied().unwrap_or_else(Vec3::zeros));
    u.scan_of.push(s as u32);
    if let Some(slot) = u.vertex_of.get_mut(s).and_then(|m| m.get_mut(v as usize)) {
        *slot = x;
    }
    Some(x)
}

/// Puts back the removed triangles the result does not cover (see the module).
pub(super) fn recover(
    u: &mut Union,
    scans: &[Scan],
    removed: &[Vec<bool>],
    guard: &mut Guard<'_>,
    tol: f64,
) -> Recovered {
    let todo = uncovered(u, scans, removed, tol);
    let mut rep = Recovered { uncovered: todo.len(), ..Recovered::default() };
    let mut edges = u.directed_edges();
    for (s, t) in todo {
        let Some(&tri) = scans.get(s).and_then(|sc| sc.mesh.triangles.get(t)) else { continue };
        let (Some(a), Some(b), Some(c)) =
            (corner(u, scans, s, tri[0]), corner(u, scans, s, tri[1]), corner(u, scans, s, tri[2]))
        else {
            continue;
        };
        let m = [a, b, c];
        let fresh = a != b && b != c && a != c;
        if !fresh || (0..3).any(|k| edges.contains(&directed(m[k], m[(k + 1) % 3]))) || !guard.accept(&u.positions, m) {
            continue;
        }
        edges.extend((0..3).map(|k| directed(m[k], m[(k + 1) % 3])));
        u.triangles.push(m);
        u.origin.push(Origin::Recovered(s as u32, t as u32));
        rep.triangles += 1;
    }
    rep
}
