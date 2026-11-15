//! The kept triangles of all scans as one vertex and triangle list, before the seam is closed:
//! every kept vertex once, at its own position; seam vertices of different scans that coincide
//! (closer than [`WELD_SHARE`] of the tolerance) welded into the one of the scan listed first;
//! removed triangles whose corners all exist in the union at exactly their positions restored
//! (pieces cut from one scan share their vertices and triangles where they overlap: there the seam
//! is a re-weld, not new surface).
use super::prep::{NONE, Scan};
use crate::{KdTree, Point3, Vec3};
use std::collections::{HashMap, HashSet};

/// Seam vertices of different scans closer than this share of the tolerance (and a tenth of the
/// median edge) are one vertex: two copies of one point a scan pair left on either side of the seam.
pub const WELD_SHARE: f64 = 0.25;
/// Rounds of undoing welds that would make an edge run the same way twice.
const WELD_PASSES: usize = 4;
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

    /// Welds front vertices of different scans that coincide (within `eps`) into the vertex of the
    /// scan listed first, drops the triangles that collapse, and returns the front again.
    pub(super) fn weld_front(&mut self, scans: &[Scan], eps: f64) {
        let front = self.front(scans);
        let mut ends: Vec<u32> = front.iter().flat_map(|f| [f[0], f[1]]).collect();
        ends.sort_unstable();
        ends.dedup();
        if ends.is_empty() || !(eps > 0.0) {
            return;
        }
        let mut to = self.weld_targets(&ends, eps);
        // a weld that would make two triangles run one edge the same way (or put three on it) is undone
        for _ in 0..WELD_PASSES {
            let bad = self.clashing(&to);
            if bad.is_empty() {
                break;
            }
            for v in bad {
                if let Some(slot) = to.get_mut(v as usize) {
                    *slot = v;
                }
            }
        }
        for (v, &w) in to.iter().enumerate().filter(|(v, w)| *v as u32 != **w) {
            let moved = match (self.positions.get(v), self.positions.get(w as usize)) {
                (Some(p), Some(q)) => (q - p).norm(),
                _ => 0.0,
            };
            self.max_move = self.max_move.max(moved);
            self.welded += 1;
        }
        self.remap(&to);
    }

    /// For every front vertex in `ends`, the vertex of a scan listed earlier within `eps` it is welded
    /// into (itself when none).
    fn weld_targets(&self, ends: &[u32], eps: f64) -> Vec<u32> {
        let pts: Vec<Point3> = ends.iter().filter_map(|&v| self.positions.get(v as usize).copied()).collect();
        let tree = KdTree::build(&pts);
        let mut to: Vec<u32> = (0..self.positions.len() as u32).collect();
        for (i, &v) in ends.iter().enumerate() {
            let (Some(p), Some(&sv)) = (pts.get(i), self.scan_of.get(v as usize)) else { continue };
            let best = tree
                .within(p, eps)
                .into_iter()
                .filter_map(|j| ends.get(j).copied())
                .filter(|&w| w < v && self.scan_of.get(w as usize).is_some_and(|&sw| sw < sv))
                .min();
            if let Some(w) = best {
                let target = to.get(w as usize).copied().unwrap_or(w);
                if let Some(slot) = to.get_mut(v as usize) {
                    *slot = target;
                }
            }
        }
        to
    }

    /// The welded vertices (`to[v] != v`) on an edge that the weld would leave with two triangles running
    /// it the same way, or with three triangles.
    fn clashing(&self, to: &[u32]) -> Vec<u32> {
        let map = |v: u32| to.get(v as usize).copied().unwrap_or(v);
        // only edges at a welded vertex or a vertex welded into can change
        let mut moved = vec![false; self.positions.len()];
        for (v, &w) in to.iter().enumerate().filter(|(v, w)| *v as u32 != **w) {
            for x in [v, w as usize] {
                if let Some(f) = moved.get_mut(x) {
                    *f = true;
                }
            }
        }
        let touched: Vec<&[u32; 3]> = self
            .triangles
            .iter()
            .filter(|t| t.iter().any(|&v| moved.get(v as usize).copied().unwrap_or(false)))
            .collect();
        let mut count: HashMap<u64, u32> = HashMap::new();
        let mut undirected: HashMap<u64, u32> = HashMap::new();
        for t in &touched {
            let m = t.map(map);
            if m[0] == m[1] || m[1] == m[2] || m[0] == m[2] {
                continue;
            }
            for k in 0..3 {
                let (a, b) = (m[k], m[(k + 1) % 3]);
                *count.entry(directed(a, b)).or_default() += 1;
                *undirected.entry(directed(a.min(b), a.max(b))).or_default() += 1;
            }
        }
        let mut bad: Vec<u32> = Vec::new();
        for t in &touched {
            let m = t.map(map);
            let clash = (0..3).any(|k| {
                let (a, b) = (m[k], m[(k + 1) % 3]);
                count.get(&directed(a, b)).copied().unwrap_or(0) > 1
                    || undirected.get(&directed(a.min(b), a.max(b))).copied().unwrap_or(0) > 2
            });
            if clash {
                bad.extend(t.iter().copied().filter(|&v| map(v) != v));
            }
        }
        bad.sort_unstable();
        bad.dedup();
        bad
    }

    /// Points every triangle at the vertices `to` gives and drops those that collapse.
    fn remap(&mut self, to: &[u32]) {
        let mut tris = Vec::with_capacity(self.triangles.len());
        let mut origin = Vec::with_capacity(self.triangles.len());
        for (t, o) in self.triangles.iter().zip(&self.origin) {
            let m = t.map(|v| to.get(v as usize).copied().unwrap_or(v));
            if m[0] != m[1] && m[1] != m[2] && m[0] != m[2] {
                tris.push(m);
                origin.push(*o);
            }
        }
        for map in &mut self.vertex_of {
            for v in map.iter_mut().filter(|v| **v != NONE) {
                *v = to.get(*v as usize).copied().unwrap_or(*v);
            }
        }
        self.triangles = tris;
        self.origin = origin;
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

/// Gives the unused vertex `v` of scan `s` a place in the union.
fn add_vertex(u: &mut Union, at: &mut HashMap<[u64; 3], u32>, scans: &[Scan], s: usize, v: u32) -> Option<u32> {
    let scan = scans.get(s)?;
    let p = *scan.mesh.positions.get(v as usize)?;
    let x = u.positions.len() as u32;
    u.positions.push(p);
    u.normals.push(scan.normals.get(v as usize).copied().unwrap_or_else(Vec3::zeros));
    u.scan_of.push(s as u32);
    if let Some(slot) = u.vertex_of.get_mut(s).and_then(|m| m.get_mut(v as usize)) {
        *slot = x;
    }
    at.entry(bits(&p)).or_insert(x);
    Some(x)
}

/// The union vertices of the corners of a removed triangle, when it can come back: every corner is
/// in the union (or an unused own vertex, added now) and at least one is another scan's vertex at
/// exactly its place (a triangle removed for redundancy, with all its corners its own, stays out).
fn corners_of(
    u: &mut Union,
    at: &mut HashMap<[u64; 3], u32>,
    scans: &[Scan],
    lost: &[Vec<bool>],
    s: usize,
    tri: [u32; 3],
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
            Corner::Unused(v) => add_vertex(u, at, scans, s, v)?,
        };
    }
    (m[0] != m[1] && m[1] != m[2] && m[0] != m[2]).then_some(m)
}

/// Restores the removed triangles whose corners all exist in the union at exactly their positions
/// (see [`corners_of`]), when that uses no directed edge twice and `guard` accepts them. Returns how
/// many.
pub(super) fn restore(
    u: &mut Union,
    scans: &[Scan],
    (removed, lost): (&[Vec<bool>], &[Vec<bool>]),
    guard: &mut dyn FnMut(&[Point3], [u32; 3]) -> bool,
) -> usize {
    // only vertices still in use (a welded vertex lives on as the one it was welded into)
    let mut at: HashMap<[u64; 3], u32> = HashMap::new();
    for &i in u.vertex_of.iter().flatten().filter(|&&i| i != NONE) {
        if let Some(p) = u.positions.get(i as usize) {
            let slot = at.entry(bits(p)).or_insert(i);
            *slot = (*slot).min(i);
        }
    }
    let mut edges = u.directed_edges();
    let mut done: HashSet<(u32, u32)> = HashSet::new();
    let mut restored = 0;
    for _ in 0..RESTORE_PASSES {
        let before = restored;
        for (s, scan) in scans.iter().enumerate() {
            let gone = removed.get(s).map_or(&[][..], Vec::as_slice);
            for (t, &tri) in scan.mesh.triangles.iter().enumerate() {
                if !gone.get(t).copied().unwrap_or(false) || done.contains(&(s as u32, t as u32)) {
                    continue;
                }
                let Some(m) = corners_of(u, &mut at, scans, lost, s, tri) else { continue };
                if (0..3).any(|k| edges.contains(&directed(m[k], m[(k + 1) % 3]))) || !guard(&u.positions, m) {
                    continue;
                }
                edges.extend((0..3).map(|k| directed(m[k], m[(k + 1) % 3])));
                u.triangles.push(m);
                u.origin.push(Origin::Restored(s as u32, t as u32));
                done.insert((s as u32, t as u32));
                restored += 1;
            }
        }
        if restored == before {
            break;
        }
    }
    restored
}
