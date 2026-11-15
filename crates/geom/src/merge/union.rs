//! The kept triangles of all scans as one vertex and triangle list, before the seam is closed:
//! every kept vertex once, at its own position; seam vertices of different scans that coincide
//! (see [`super::weld`]) welded into the one of the scan listed first;
//! removed triangles whose corners all exist in the union at exactly their positions restored
//! (pieces cut from one scan share their vertices and triangles where they overlap: there the seam
//! is a re-weld, not new surface).
use super::guard::Pts;
use super::prep::{NONE, Scan};
use crate::{Point3, Vec3};
use std::collections::{HashMap, HashSet};

/// Passes over the removed triangles when restoring (a triangle may need a neighbour restored first).
const RESTORE_PASSES: usize = 3;

/// Where a triangle of the union comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Origin {
    /// Triangle `1` of scan `0`, kept.
    Kept(u32, u32),
    /// Triangle `1` of scan `0`, removed and restored (its corners all at exactly their places).
    Restored(u32, u32),
    /// Triangle `1` of scan `0`, removed, left uncovered by the seam and put back on its own vertices.
    Recovered(u32, u32),
    /// Built to close the seam.
    Seam,
}

/// The union being built.
pub(super) struct Union {
    pub(super) positions: Vec<Point3>,
    pub(super) normals: Vec<Vec3>,
    /// Scan of every vertex.
    pub(super) scan_of: Vec<u32>,
    pub(super) triangles: Vec<[u32; 3]>,
    pub(super) origin: Vec<Origin>,
    /// Union vertex of every scan vertex (`NONE` when not kept).
    pub(super) vertex_of: Vec<Vec<u32>>,
    /// Vertices moved by the weld and the largest move (mm).
    pub(super) welded: usize,
    pub(super) max_move: f64,
    /// Welds undone because they would have flipped, folded or doubled a triangle.
    pub(super) welds_undone: usize,
    /// Kept triangles `(scan, triangle)` a weld collapsed (two corners welded into one).
    pub(super) collapsed: Vec<(u32, u32)>,
    /// The cut edges `a → b` the seam started from.
    pub(super) initial_front: Vec<[u32; 2]>,
}

pub(super) fn directed(a: u32, b: u32) -> u64 {
    (u64::from(a) << 32) | u64::from(b)
}

/// The kept triangles of every scan, their vertices numbered scan after scan.
pub(super) fn assemble(scans: &[Scan], removed: &[Vec<bool>]) -> Union {
    let mut u = Union {
        positions: Vec::new(),
        normals: Vec::new(),
        scan_of: Vec::new(),
        triangles: Vec::new(),
        origin: Vec::new(),
        vertex_of: Vec::with_capacity(scans.len()),
        welded: 0,
        max_move: 0.0,
        welds_undone: 0,
        collapsed: Vec::new(),
        initial_front: Vec::new(),
    };
    for (s, scan) in scans.iter().enumerate() {
        let gone = removed.get(s).map_or(&[][..], Vec::as_slice);
        let mut map = vec![NONE; scan.mesh.positions.len()];
        for (t, tri) in scan.mesh.triangles.iter().enumerate().filter(|(t, _)| !gone.get(*t).copied().unwrap_or(true)) {
            let mut out = [0u32; 3];
            for (k, &v) in tri.iter().enumerate() {
                let Some(slot) = map.get_mut(v as usize) else { continue };
                if *slot == NONE {
                    *slot = u.positions.len() as u32;
                    u.positions.push(scan.mesh.positions.get(v as usize).copied().unwrap_or_else(Point3::origin));
                    u.normals.push(scan.normals.get(v as usize).copied().unwrap_or_else(Vec3::zeros));
                    u.scan_of.push(s as u32);
                }
                out[k] = *slot;
            }
            u.triangles.push(out);
            u.origin.push(Origin::Kept(s as u32, t as u32));
        }
        u.vertex_of.push(map);
    }
    u
}

impl Union {
    /// Directed edges of all triangles.
    pub(super) fn directed_edges(&self) -> HashSet<u64> {
        self.triangles.iter().flat_map(|t| (0..3).map(move |k| directed(t[k], t[(k + 1) % 3]))).collect()
    }

    /// Is edge `k` of the union triangle `i` on the seam side: an interior edge of the scan it came
    /// from, or an edge of a seam triangle?
    pub(super) fn seam_side(&self, scans: &[Scan], i: usize, k: usize) -> bool {
        match self.origin.get(i) {
            Some(Origin::Kept(s, t) | Origin::Restored(s, t) | Origin::Recovered(s, t)) => {
                scans.get(*s as usize).is_some_and(|sc| !sc.is_border(*t as usize, k))
            }
            Some(Origin::Seam) => true,
            None => false,
        }
    }

    /// The front of the seam: border edges of the union (no triangle on the other side) on the seam
    /// side (see [`Union::seam_side`]), as `[a, b, c]` (edge `a → b` of triangle `(a, b, c)`).
    pub(super) fn front(&self, scans: &[Scan]) -> Vec<[u32; 3]> {
        let edges = self.directed_edges();
        let mut out = Vec::new();
        for (i, t) in self.triangles.iter().enumerate() {
            for k in 0..3 {
                let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
                if !edges.contains(&directed(b, a)) && self.seam_side(scans, i, k) {
                    out.push([a, b, c]);
                }
            }
        }
        out
    }
}

/// Exact position of a point as a key.
fn bits(p: &Point3) -> [u64; 3] {
    [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]
}

/// Where corner `v` of a removed triangle of scan `s` is in the union.
#[derive(Clone, Copy)]
enum Corner {
    /// The scan's own vertex, kept.
    Own(u32),
    /// A vertex of another scan at exactly its position.
    Other(u32),
    /// The scan's own vertex, not lost to another scan, but in no kept triangle: added if needed.
    Unused(u32),
}

fn corner(
    u: &Union,
    at: &HashMap<[u64; 3], u32>,
    scans: &[Scan],
    lost: &[Vec<bool>],
    s: usize,
    v: u32,
) -> Option<Corner> {
    if let Some(x) = u.vertex_of.get(s).and_then(|m| m.get(v as usize)).copied().filter(|&x| x != NONE) {
        return Some(Corner::Own(x));
    }
    let p = scans.get(s)?.mesh.positions.get(v as usize)?;
    if let Some(&x) = at.get(&bits(p)) {
        return Some(if u.scan_of.get(x as usize) == Some(&(s as u32)) { Corner::Own(x) } else { Corner::Other(x) });
    }
    let owned = !lost.get(s).and_then(|l| l.get(v as usize)).copied().unwrap_or(true);
    owned.then_some(Corner::Unused(v))
}

/// Gives vertex `v` of scan `s` a place in the union (at its own position, with its normal).
pub(super) fn add_vertex(u: &mut Union, scans: &[Scan], s: usize, v: u32) -> Option<u32> {
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

impl Union {
    /// Takes back the vertices added since the union had `len` of them (those of scan vertices
    /// `added`): a triangle that needed them was refused.
    pub(super) fn rollback(&mut self, len: usize, added: &[(usize, u32)]) {
        self.positions.truncate(len);
        self.normals.truncate(len);
        self.scan_of.truncate(len);
        for &(s, v) in added {
            if let Some(slot) = self.vertex_of.get_mut(s).and_then(|m| m.get_mut(v as usize)) {
                *slot = NONE;
            }
        }
    }

    /// The vertex sets of the triangles (any winding).
    pub(super) fn vertex_sets(&self) -> HashSet<[u32; 3]> {
        self.triangles.iter().map(|t| sorted(*t)).collect()
    }
}

pub(super) fn sorted(mut t: [u32; 3]) -> [u32; 3] {
    t.sort_unstable();
    t
}

/// The union vertices of the corners of a removed triangle, when it can come back: every corner is
/// in the union (or an unused own vertex, added now and listed in `added`) and at least one is
/// another scan's vertex at exactly its place (a triangle removed for redundancy, with all its
/// corners its own, stays out).
fn corners_of(
    u: &mut Union,
    at: &HashMap<[u64; 3], u32>,
    (scans, lost): (&[Scan], &[Vec<bool>]),
    s: usize,
    tri: [u32; 3],
    added: &mut Vec<(usize, u32)>,
) -> Option<[u32; 3]> {
    let c = tri.map(|v| corner(u, at, scans, lost, s, v));
    let c = [c[0]?, c[1]?, c[2]?];
    if !c.iter().any(|k| matches!(k, Corner::Other(_))) {
        return None;
    }
    let mut m = [0u32; 3];
    for (k, x) in c.iter().enumerate() {
        m[k] = match *x {
            Corner::Own(i) | Corner::Other(i) => i,
            Corner::Unused(v) => {
                added.push((s, v));
                add_vertex(u, scans, s, v)?
            }
        };
    }
    (m[0] != m[1] && m[1] != m[2] && m[0] != m[2]).then_some(m)
}

/// What the restoring keeps between triangles: used positions, directed edges, vertex sets.
struct Restoring {
    at: HashMap<[u64; 3], u32>,
    edges: HashSet<u64>,
    sets: HashSet<[u32; 3]>,
}

impl Restoring {
    /// Restores triangle `t` (corners `tri`) of scan `s` if it can come back (see [`restore`]).
    fn try_one(
        &mut self,
        u: &mut Union,
        (scans, lost): (&[Scan], &[Vec<bool>]),
        (s, t, tri): (usize, usize, [u32; 3]),
        guard: &mut dyn FnMut(Pts<'_>, [u32; 3]) -> bool,
    ) -> bool {
        let (len, mut added) = (u.positions.len(), Vec::new());
        let Some(m) = corners_of(u, &self.at, (scans, lost), s, tri, &mut added) else {
            u.rollback(len, &added);
            return false;
        };
        let taken =
            (0..3).any(|k| self.edges.contains(&directed(m[k], m[(k + 1) % 3]))) || self.sets.contains(&sorted(m));
        if taken || !guard(Pts::of(u), m) {
            u.rollback(len, &added);
            return false;
        }
        for &(sv, v) in &added {
            let x = u.vertex_of.get(sv).and_then(|map| map.get(v as usize)).copied();
            if let (Some(p), Some(x)) = (scans.get(sv).and_then(|sc| sc.mesh.positions.get(v as usize)), x) {
                self.at.entry(bits(p)).or_insert(x);
            }
        }
        self.edges.extend((0..3).map(|k| directed(m[k], m[(k + 1) % 3])));
        self.sets.insert(sorted(m));
        u.triangles.push(m);
        u.origin.push(Origin::Restored(s as u32, t as u32));
        true
    }
}

/// The union vertex of every used scan vertex by exact position (the lowest index of a position).
fn positions_in_use(u: &Union) -> HashMap<[u64; 3], u32> {
    let mut at: HashMap<[u64; 3], u32> = HashMap::new();
    for &i in u.vertex_of.iter().flatten().filter(|&&i| i != NONE) {
        if let Some(p) = u.positions.get(i as usize) {
            let slot = at.entry(bits(p)).or_insert(i);
            *slot = (*slot).min(i);
        }
    }
    at
}

/// Restores the removed triangles whose corners all exist in the union at exactly their positions
/// (see [`corners_of`]), when that uses no directed edge twice, repeats no triangle's corners and
/// `guard` accepts them; the vertices a refused one needed are taken back. Returns how many, or
/// `None` when `cancel` was set.
pub(super) fn restore(
    u: &mut Union,
    scans: &[Scan],
    (removed, lost): (&[Vec<bool>], &[Vec<bool>]),
    guard: &mut dyn FnMut(Pts<'_>, [u32; 3]) -> bool,
    cancel: &crate::Cancel,
) -> Option<usize> {
    let mut st = Restoring { at: positions_in_use(u), edges: u.directed_edges(), sets: u.vertex_sets() };
    let mut done: HashSet<(u32, u32)> = HashSet::new();
    let mut restored = 0;
    for _ in 0..RESTORE_PASSES {
        let before = restored;
        for (s, scan) in scans.iter().enumerate() {
            if cancel.is_cancelled() {
                return None;
            }
            let gone = removed.get(s).map_or(&[][..], Vec::as_slice);
            for (t, &tri) in scan.mesh.triangles.iter().enumerate() {
                if !gone.get(t).copied().unwrap_or(false) || done.contains(&(s as u32, t as u32)) {
                    continue;
                }
                if st.try_one(u, (scans, lost), (s, t, tri), guard) {
                    done.insert((s as u32, t as u32));
                    restored += 1;
                }
            }
        }
        if restored == before {
            break;
        }
    }
    Some(restored)
}
