//! Edge bookkeeping of the mesh that is thickened: a compacted copy without degenerate triangles,
//! the checks that make the stitching well defined, the split of vertices where boundary loops
//! touch, and the vertex adjacency the normal smoothing walks.
use super::ThickenError;
use crate::{Mesh, Point3};

/// Triangle and vertex lists the thickening works on.
pub(super) struct Work {
    pub positions: Vec<Point3>,
    pub triangles: Vec<[u32; 3]>,
    /// Triangles dropped because they repeat a corner.
    pub dropped: usize,
    /// Vertices added by splitting the places where boundary loops touch.
    pub pinch_splits: usize,
    /// Triangles removed because they lay on an edge shared by three or more triangles.
    pub removed: usize,
    /// Triangles turned around to make the orientation consistent.
    pub flipped: usize,
}

/// Compressed rows: `row(i)` lists the items of vertex `i`.
pub(super) struct Csr {
    start: Vec<usize>,
    items: Vec<u32>,
}

impl Csr {
    /// Builds the rows from `(row, item)` pairs.
    pub(super) fn from_pairs(rows: usize, pairs: &[(u32, u32)]) -> Csr {
        let mut start = vec![0usize; rows + 1];
        for &(r, _) in pairs {
            if let Some(c) = start.get_mut(r as usize + 1) {
                *c += 1;
            }
        }
        for i in 0..rows {
            let prev = start.get(i).copied().unwrap_or(0);
            if let Some(c) = start.get_mut(i + 1) {
                *c += prev;
            }
        }
        let mut fill = start.clone();
        let mut items = vec![0u32; pairs.len()];
        for &(r, it) in pairs {
            if let Some(f) = fill.get_mut(r as usize)
                && let Some(slot) = items.get_mut(*f)
            {
                *slot = it;
                *f += 1;
            }
        }
        Csr { start, items }
    }

    /// The items of row `i` (empty past the end).
    pub(super) fn row(&self, i: usize) -> &[u32] {
        let (Some(&a), Some(&b)) = (self.start.get(i), self.start.get(i + 1)) else { return &[] };
        self.items.get(a..b).unwrap_or(&[])
    }
}

fn key(a: u32, b: u32) -> u64 {
    (u64::from(a) << 32) | u64::from(b)
}

fn unkey(k: u64) -> (u32, u32) {
    ((k >> 32) as u32, (k & 0xFFFF_FFFF) as u32)
}

/// A copy of `mesh` without triangles that repeat a corner and without unused vertices (the order of
/// the remaining vertices is kept).
pub(super) fn compact(mesh: &Mesh) -> Work {
    let kept: Vec<[u32; 3]> =
        mesh.triangles.iter().copied().filter(|t| t[0] != t[1] && t[1] != t[2] && t[2] != t[0]).collect();
    let dropped = mesh.triangles.len() - kept.len();
    let mut slot = vec![u32::MAX; mesh.positions.len()];
    for t in &kept {
        for &v in t {
            if let Some(s) = slot.get_mut(v as usize) {
                *s = 0;
            }
        }
    }
    let mut positions = Vec::new();
    for (i, s) in slot.iter_mut().enumerate() {
        if *s == 0
            && let Some(p) = mesh.positions.get(i)
        {
            *s = positions.len() as u32;
            positions.push(*p);
        }
    }
    let triangles =
        kept.iter().map(|t| [0, 1, 2].map(|k| slot.get(t[k] as usize).copied().unwrap_or(0))).collect::<Vec<_>>();
    Work { positions, triangles, dropped, pinch_splits: 0, removed: 0, flipped: 0 }
}

/// Drops the vertices no triangle uses (the order of the others is kept).
pub(super) fn drop_unused_vertices(w: &mut Work) {
    let mut slot = vec![u32::MAX; w.positions.len()];
    for t in &w.triangles {
        for &v in t {
            if let Some(s) = slot.get_mut(v as usize) {
                *s = 0;
            }
        }
    }
    let mut positions = Vec::new();
    for (i, s) in slot.iter_mut().enumerate() {
        if *s == 0
            && let Some(p) = w.positions.get(i)
        {
            *s = positions.len() as u32;
            positions.push(*p);
        }
    }
    for t in &mut w.triangles {
        *t = t.map(|v| slot.get(v as usize).copied().unwrap_or(0));
    }
    w.positions = positions;
}

/// The sorted directed edges of the triangles.
pub(super) fn directed_edges(triangles: &[[u32; 3]]) -> Vec<u64> {
    let mut keys: Vec<u64> = Vec::with_capacity(triangles.len() * 3);
    for t in triangles {
        keys.extend([key(t[0], t[1]), key(t[1], t[2]), key(t[2], t[0])]);
    }
    keys.sort_unstable();
    keys
}

/// Edges used by two triangles in the same direction (three or more triangles on an edge, or two of
/// opposite orientation): the mesh is then not an oriented manifold.
pub(super) fn repeated_edges(sorted: &[u64]) -> usize {
    sorted.windows(2).filter(|w| w[0] == w[1]).count()
}

/// The edges (in the direction of their triangle) that have no triangle on the other side.
pub(super) fn boundary_edges(sorted: &[u64]) -> Vec<(u32, u32)> {
    sorted.iter().map(|&k| unkey(k)).filter(|&(a, b)| sorted.binary_search(&key(b, a)).is_err()).collect()
}

/// Splits every vertex where more than one boundary loop passes into one vertex per fan of triangles
/// around it, so that each boundary vertex has one edge in and one out. Returns the vertices added.
pub(super) fn split_pinches(w: &mut Work) -> usize {
    let n = w.positions.len();
    let sorted = directed_edges(&w.triangles);
    let mut out_count = vec![0u32; n];
    for (a, _) in boundary_edges(&sorted) {
        if let Some(c) = out_count.get_mut(a as usize) {
            *c += 1;
        }
    }
    let pinched: Vec<u32> = (0..n as u32).filter(|&v| out_count.get(v as usize).is_some_and(|&c| c > 1)).collect();
    if pinched.is_empty() {
        return 0;
    }
    let pairs: Vec<(u32, u32)> =
        w.triangles.iter().enumerate().flat_map(|(i, t)| t.iter().map(move |&v| (v, i as u32))).collect();
    let around = Csr::from_pairs(n, &pairs);
    let mut added = 0;
    for v in pinched {
        added += split_fans(w, &around, v);
    }
    w.pinch_splits += added;
    added
}

/// Gives each fan of triangles around `v` but the first its own copy of `v`.
fn split_fans(w: &mut Work, around: &Csr, v: u32) -> usize {
    let tris: Vec<u32> = around.row(v as usize).to_vec();
    let mut group: Vec<usize> = (0..tris.len()).collect();
    // triangles that share an edge through `v` are one fan
    let mut via: Vec<(u32, usize)> = Vec::new();
    for (local, &ti) in tris.iter().enumerate() {
        if let Some(t) = w.triangles.get(ti as usize) {
            via.extend(t.iter().filter(|&&x| x != v).map(|&x| (x, local)));
        }
    }
    via.sort_unstable();
    for pair in via.windows(2) {
        if pair[0].0 == pair[1].0 {
            merge(&mut group, pair[0].1, pair[1].1);
        }
    }
    let mut roots: Vec<usize> = Vec::new();
    let mut ids: Vec<u32> = vec![v];
    for (local, &ti) in tris.iter().enumerate() {
        let root = find(&mut group, local);
        let slot = match roots.iter().position(|&r| r == root) {
            Some(s) => s,
            None => {
                roots.push(root);
                if roots.len() > 1
                    && let Some(p) = w.positions.get(v as usize).copied()
                {
                    ids.push(w.positions.len() as u32);
                    w.positions.push(p);
                }
                roots.len() - 1
            }
        };
        let id = ids.get(slot).copied().unwrap_or(v);
        if id != v
            && let Some(t) = w.triangles.get_mut(ti as usize)
        {
            for c in t.iter_mut().filter(|c| **c == v) {
                *c = id;
            }
        }
    }
    ids.len() - 1
}

fn find(parent: &mut [usize], mut i: usize) -> usize {
    while let Some(&p) = parent.get(i) {
        if p == i {
            break;
        }
        i = p;
    }
    i
}

fn merge(parent: &mut [usize], a: usize, b: usize) {
    let (ra, rb) = (find(parent, a), find(parent, b));
    if ra != rb
        && let Some(p) = parent.get_mut(ra.max(rb))
    {
        *p = ra.min(rb);
    }
}

/// Number of closed boundary loops, given the boundary edges (one out-edge per vertex).
pub(super) fn count_loops(vertices: usize, boundary: &[(u32, u32)]) -> usize {
    let mut next = vec![u32::MAX; vertices];
    for &(a, b) in boundary {
        if let Some(n) = next.get_mut(a as usize) {
            *n = b;
        }
    }
    let mut seen = vec![false; vertices];
    let mut loops = 0;
    for &(start, _) in boundary {
        if seen.get(start as usize).copied().unwrap_or(true) {
            continue;
        }
        loops += 1;
        let mut v = start;
        while let Some(s) = seen.get_mut(v as usize) {
            if *s {
                break;
            }
            *s = true;
            v = next.get(v as usize).copied().unwrap_or(u32::MAX);
        }
    }
    loops
}

/// Vertex neighbours over the undirected edges (each neighbour once).
pub(super) fn adjacency(vertices: usize, sorted: &[u64]) -> Csr {
    let mut pairs: Vec<(u32, u32)> = Vec::with_capacity(sorted.len() * 2);
    for &k in sorted {
        let (a, b) = unkey(k);
        if a < b || sorted.binary_search(&key(b, a)).is_err() {
            pairs.push((a, b));
            pairs.push((b, a));
        }
    }
    Csr::from_pairs(vertices, &pairs)
}

/// Checks that `check` fails with [`ThickenError::NonOrientable`] when an edge is repeated.
pub(super) fn require_oriented(sorted: &[u64]) -> Result<(), ThickenError> {
    match repeated_edges(sorted) {
        0 => Ok(()),
        edges => Err(ThickenError::NonOrientable { edges }),
    }
}
