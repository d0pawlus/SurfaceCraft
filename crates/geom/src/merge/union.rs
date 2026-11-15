//! The kept triangles of all scans as one vertex and triangle list, before the seam is closed:
//! every kept vertex once, at its own position; seam vertices of different scans that coincide
//! (closer than [`WELD_SHARE`] of the median edge) welded into the one of the scan listed first;
//! removed triangles whose corners all exist in the union at exactly their positions restored
//! (pieces cut from one scan share their vertices and triangles where they overlap: there the seam
//! is a re-weld, not new surface).
use super::prep::{NONE, Scan};
use crate::{KdTree, Point3, Vec3};
use std::collections::{HashMap, HashSet};

/// Seam vertices of different scans closer than this share of the median edge are one vertex.
pub const WELD_SHARE: f64 = 1e-3;
/// Passes over the removed triangles when restoring (a triangle may need a neighbour restored first).
const RESTORE_PASSES: usize = 3;

/// Where a triangle of the union comes from.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Origin {
    /// Triangle `1` of scan `0`, kept.
    Kept(u32, u32),
    /// Triangle `1` of scan `0`, removed and restored.
    Restored(u32, u32),
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
            Some(Origin::Kept(s, t) | Origin::Restored(s, t)) => {
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
                let moved = self.positions.get(target as usize).map_or(0.0, |q| (q - p).norm());
                self.max_move = self.max_move.max(moved);
                self.welded += 1;
            }
        }
        self.remap(&to);
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

/// The union vertex of corner `v` of scan `s`: its own when kept, else one of another scan at exactly
/// its position.
fn corner(u: &Union, at: &HashMap<[u64; 3], u32>, scans: &[Scan], s: usize, v: u32) -> Option<u32> {
    let own = u.vertex_of.get(s).and_then(|m| m.get(v as usize)).copied().filter(|&x| x != NONE);
    own.or_else(|| at.get(&bits(scans.get(s)?.mesh.positions.get(v as usize)?)).copied())
}

/// Restores the removed triangles whose corners all exist in the union at exactly their positions,
/// when that uses no directed edge twice and `guard` accepts them. Returns how many.
pub(super) fn restore(
    u: &mut Union,
    scans: &[Scan],
    removed: &[Vec<bool>],
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
            for (t, tri) in scan.mesh.triangles.iter().enumerate() {
                if !gone.get(t).copied().unwrap_or(false) || done.contains(&(s as u32, t as u32)) {
                    continue;
                }
                let Some(m) = tri.iter().map(|&v| corner(u, &at, scans, s, v)).collect::<Option<Vec<u32>>>() else {
                    continue;
                };
                let m = [m[0], m[1], m[2]];
                let fresh = m[0] != m[1] && m[1] != m[2] && m[0] != m[2];
                if !fresh || (0..3).any(|k| edges.contains(&directed(m[k], m[(k + 1) % 3]))) || !guard(&u.positions, m)
                {
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
