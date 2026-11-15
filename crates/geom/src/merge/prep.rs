//! One scan made ready for the merge: its edges (border, interior, non-manifold), its orientation,
//! its vertex normals, a BVH and the distance of every vertex from the border of the scan.
use crate::{Aabb, Bvh, Cancel, Mesh, Point3, Vec3, par, pieces};
use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// Dijkstra steps between two looks at the cancellation flag.
const CANCEL_EVERY: usize = 1 << 14;

/// No triangle across this edge.
pub(super) const NONE: u32 = u32::MAX;

/// A scan with what the merge needs to know about it.
pub(super) struct Scan {
    /// The scan without the triangles on edges of three or more triangles.
    pub(super) mesh: Mesh,
    /// Triangles left out (duplicates, non-manifold, wound the wrong way).
    pub(super) dropped: Vec<[Point3; 3]>,
    /// Per triangle and edge `k` (`t[k] → t[k + 1]`): the triangle across it, [`NONE`] on the border.
    pub(super) across: Vec<[u32; 3]>,
    /// Per vertex: the area-weighted unit normal (zero for a vertex of no triangle).
    pub(super) normals: Vec<Vec3>,
    /// Per vertex: the distance along the edges to the nearest border vertex (∞ on a closed scan).
    pub(super) border_distance: Vec<f64>,
    /// BVH over [`Scan::mesh`].
    pub(super) bvh: Bvh,
    /// Bounding box.
    pub(super) bbox: Aabb,
    /// Median edge length (mm).
    pub(super) median_edge: f64,
    /// Triangles left out as exact duplicates or for lying on an edge of three or more triangles.
    pub(super) non_manifold: usize,
    /// Triangles left out for being wound against the rest of their piece, and their share of the area.
    pub(super) flipped: (usize, f64),
    /// Closed pieces with a negative volume (turned inside out).
    pub(super) inverted_pieces: usize,
    /// Topology of the scan as given to the merge (after the drop).
    pub(super) topology: Topology,
}

/// Pieces, boundary loops and genus of a mesh.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Topology {
    /// Connected pieces.
    pub pieces: usize,
    /// Boundary loops.
    pub loops: usize,
    /// Genus (sum over the pieces); `None` when a piece has none (pinched, non-manifold, not oriented).
    pub genus: Option<usize>,
    /// Edges of three or more triangles.
    pub non_manifold_edges: usize,
    /// Edges of two triangles that run the same way.
    pub unoriented_edges: usize,
    /// Vertices whose triangles form more than one fan.
    pub pinched_vertices: usize,
}

/// The topology of `mesh` (see [`Topology`]).
#[must_use]
pub fn topology(mesh: &Mesh) -> Topology {
    let ps = pieces::pieces(mesh);
    Topology {
        pieces: ps.len(),
        loops: ps.iter().map(|p| p.loops.len()).sum(),
        genus: ps.iter().map(pieces::Piece::genus).sum(),
        non_manifold_edges: ps.iter().map(|p| p.non_manifold_edges).sum(),
        unoriented_edges: ps.iter().map(|p| p.unoriented_edges).sum(),
        pinched_vertices: ps.iter().map(|p| p.pinched_vertices).sum(),
    }
}

fn undirected(a: u32, b: u32) -> u64 {
    (u64::from(a.min(b)) << 32) | u64::from(a.max(b))
}

/// Every edge `(undirected key, 3 · triangle + k)` of the triangles, sorted.
fn half_edges(mesh: &Mesh) -> Vec<(u64, u32)> {
    let mut half: Vec<(u64, u32)> = Vec::with_capacity(mesh.triangles.len() * 3);
    for (t, tri) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            half.push((undirected(tri[k], tri[(k + 1) % 3]), 3 * t as u32 + k as u32));
        }
    }
    par::sort_unstable(&mut half);
    half
}

/// Groups of equal undirected edges in a sorted list of half edges.
fn groups(half: &[(u64, u32)]) -> impl Iterator<Item = &[(u64, u32)]> {
    half.chunk_by(|x, y| x.0 == y.0)
}

/// The triangles on an edge of three or more triangles, or on a collapsed edge (`a == b`).
fn non_manifold_triangles(mesh: &Mesh) -> Vec<bool> {
    let mut bad = vec![false; mesh.triangles.len()];
    for g in groups(&half_edges(mesh)) {
        let collapsed = g.first().is_some_and(|&(k, _)| (k >> 32) == (k & 0xffff_ffff));
        if g.len() > 2 || collapsed {
            for &(_, h) in g {
                if let Some(b) = bad.get_mut((h / 3) as usize) {
                    *b = true;
                }
            }
        }
    }
    bad
}

/// The triangle across every edge, and the interior edges that run the same way twice.
fn adjacency(mesh: &Mesh) -> (Vec<[u32; 3]>, usize, usize) {
    let mut across = vec![[NONE; 3]; mesh.triangles.len()];
    let (mut interior, mut unoriented) = (0, 0);
    let dir = |h: u32| -> Option<(u32, u32)> {
        let t = mesh.triangles.get((h / 3) as usize)?;
        let k = (h % 3) as usize;
        Some((*t.get(k)?, *t.get((k + 1) % 3)?))
    };
    for g in groups(&half_edges(mesh)) {
        let [(_, x), (_, y)] = g else { continue };
        interior += 1;
        if dir(*x) == dir(*y) {
            unoriented += 1;
        }
        for (from, to) in [(x, y), (y, x)] {
            if let Some(slot) = across.get_mut((from / 3) as usize).and_then(|a| a.get_mut((from % 3) as usize)) {
                *slot = to / 3;
            }
        }
    }
    (across, interior, unoriented)
}

/// Area-weighted vertex normals over the triangles.
fn vertex_normals(mesh: &Mesh) -> Vec<Vec3> {
    let mut n = vec![Vec3::zeros(); mesh.positions.len()];
    for t in 0..mesh.triangles.len() {
        let (Some(tri), Some(corners)) = (mesh.triangles.get(t), mesh.triangle(t)) else { continue };
        let w = crate::tri::cross(&corners[0], &corners[1], &corners[2]);
        for &v in tri {
            if let Some(s) = n.get_mut(v as usize) {
                *s += w;
            }
        }
    }
    n.into_iter().map(|v| crate::normalize(v).unwrap_or_else(Vec3::zeros)).collect()
}

/// Distance in the heap of the border distance (ordered by distance, then vertex).
#[derive(PartialEq)]
struct Step(f64, u32);

impl Eq for Step {}
impl Ord for Step {
    fn cmp(&self, o: &Self) -> Ordering {
        o.0.total_cmp(&self.0).then(o.1.cmp(&self.1))
    }
}
impl PartialOrd for Step {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

/// Vertex adjacency (compressed rows) from the edges of the triangles.
fn neighbours(mesh: &Mesh) -> (Vec<u32>, Vec<u32>) {
    let mut count = vec![0u32; mesh.positions.len() + 1];
    for t in &mesh.triangles {
        for &v in t {
            if let Some(c) = count.get_mut(v as usize + 1) {
                *c += 2;
            }
        }
    }
    for i in 1..count.len() {
        count[i] += count[i - 1];
    }
    let mut fill = count.clone();
    let mut list = vec![0u32; count.last().copied().unwrap_or(0) as usize];
    for t in &mesh.triangles {
        for k in 0..3 {
            let (v, others) = (t[k], [t[(k + 1) % 3], t[(k + 2) % 3]]);
            for o in others {
                if let Some(f) = fill.get_mut(v as usize) {
                    if let Some(slot) = list.get_mut(*f as usize) {
                        *slot = o;
                    }
                    *f += 1;
                }
            }
        }
    }
    (count, list)
}

/// Distance along the edges from every vertex to the nearest vertex of a border edge (Dijkstra);
/// `None` when cancelled.
fn border_distance(mesh: &Mesh, across: &[[u32; 3]], cancel: &Cancel) -> Option<Vec<f64>> {
    let mut dist = vec![f64::INFINITY; mesh.positions.len()];
    let mut heap = BinaryHeap::new();
    for (t, a) in across.iter().enumerate() {
        for k in (0..3).filter(|&k| a[k] == NONE) {
            let Some(tri) = mesh.triangles.get(t) else { continue };
            for v in [tri[k], tri[(k + 1) % 3]] {
                if let Some(d) = dist.get_mut(v as usize).filter(|d| **d > 0.0) {
                    *d = 0.0;
                    heap.push(Step(0.0, v));
                }
            }
        }
    }
    let (start, list) = neighbours(mesh);
    let mut steps = 0usize;
    while let Some(Step(d, v)) = heap.pop() {
        steps += 1;
        if steps.is_multiple_of(CANCEL_EVERY) && cancel.is_cancelled() {
            return None;
        }
        if dist.get(v as usize).is_some_and(|&best| d > best) {
            continue;
        }
        let (Some(&s), Some(&e), Some(p)) =
            (start.get(v as usize), start.get(v as usize + 1), mesh.positions.get(v as usize))
        else {
            continue;
        };
        for &w in list.get(s as usize..e as usize).unwrap_or(&[]) {
            let Some(q) = mesh.positions.get(w as usize) else { continue };
            let nd = d + (q - p).norm();
            if let Some(slot) = dist.get_mut(w as usize).filter(|best| nd < **best) {
                *slot = nd;
                heap.push(Step(nd, w));
            }
        }
    }
    Some(dist)
}

/// Median edge length of the triangles (0 without edges).
pub(super) fn median_edge(mesh: &Mesh) -> f64 {
    let mut l: Vec<f64> = (0..mesh.triangles.len())
        .filter_map(|t| mesh.triangle(t))
        .flat_map(|[a, b, c]| [(b - a).norm(), (c - b).norm(), (a - c).norm()])
        .collect();
    if l.is_empty() {
        return 0.0;
    }
    let k = l.len() / 2;
    *l.select_nth_unstable_by(k, f64::total_cmp).1
}

/// A closed piece smaller than this share of its scan's area is not judged for being inside out
/// (debris).
pub const INVERTED_MIN_SHARE: f64 = 0.01;

/// Signed volume of a piece, measured about the centre of its box.
fn volume(mesh: &Mesh, p: &pieces::Piece) -> f64 {
    let o = p.bbox.center();
    p.triangles
        .iter()
        .filter_map(|&t| mesh.triangle(t as usize))
        .map(|[a, b, c]| (a - o).dot(&(b - o).cross(&(c - o))))
        .sum()
}

/// Does piece `k` lie inside the other closed pieces (an inner skin, whose volume is negative)?
fn nested(mesh: &Mesh, closed: &[&pieces::Piece], k: usize) -> bool {
    let others: Vec<u32> =
        closed.iter().enumerate().filter(|(j, _)| *j != k).flat_map(|(_, q)| q.triangles.iter().copied()).collect();
    let probe = closed.get(k).and_then(|p| p.triangles.first()).and_then(|&t| mesh.triangles.get(t as usize));
    let (Some(tri), false) = (probe, others.is_empty()) else { return false };
    let Some(&p) = mesh.positions.get(tri[0] as usize) else { return false };
    let bvh = Bvh::build(&mesh.submesh(&others));
    crate::nesting::point_inside(&bvh, &p, 1e-9 * mesh.bbox().diagonal().max(f64::MIN_POSITIVE))
}

/// Closed pieces turned inside out: a negative volume, not inside another closed piece of the scan
/// (an inner skin), and at least [`INVERTED_MIN_SHARE`] of the scan's area.
fn inverted_pieces(mesh: &Mesh) -> usize {
    let all = pieces::pieces(mesh);
    let closed: Vec<&pieces::Piece> = all.iter().filter(|p| p.closed()).collect();
    let area = mesh.area();
    (0..closed.len())
        .filter(|&k| {
            closed.get(k).is_some_and(|p| p.area >= INVERTED_MIN_SHARE * area && volume(mesh, p) < 0.0)
                && !nested(mesh, &closed, k)
        })
        .count()
}

/// `mesh` without the triangles `drop` flags, and the corners of those dropped.
fn without(mesh: Mesh, drop: &[bool]) -> (Mesh, Vec<[Point3; 3]>) {
    let gone: Vec<[Point3; 3]> =
        drop.iter().enumerate().filter(|(_, d)| **d).filter_map(|(t, _)| mesh.triangle(t)).collect();
    if gone.is_empty() {
        return (mesh, gone);
    }
    let mut out = mesh;
    let mut keep = drop.iter().map(|d| !d);
    out.triangles.retain(|_| keep.next().unwrap_or(false));
    (out, gone)
}

/// Prepares `given` (already validated); `None` when cancelled. Exact duplicate triangles, triangles
/// on edges of three or more triangles and triangles wound against the rest of their piece are left
/// out (and counted).
pub(super) fn prepare(given: &Mesh, cancel: &Cancel) -> Option<Scan> {
    let area = given.area();
    let dups = super::orient::duplicates(given);
    let (mesh, mut dropped) = without(given.clone(), &dups);
    let bad = non_manifold_triangles(&mesh);
    let (mesh, more) = without(mesh, &bad);
    let non_manifold = dropped.len() + more.len();
    dropped.extend(more);
    let (across, _, _) = adjacency(&mesh);
    let flip = super::orient::flipped(&mesh, &across);
    let (mesh, wrong) = without(mesh, &flip);
    let flipped_area: f64 = wrong.iter().map(|[a, b, c]| crate::tri::area(a, b, c)).sum();
    let flipped = (wrong.len(), if area > 0.0 { flipped_area / area } else { 0.0 });
    dropped.extend(wrong);
    let across = if flipped.0 > 0 { adjacency(&mesh).0 } else { across };
    let border_distance = border_distance(&mesh, &across, cancel)?;
    Some(Scan {
        normals: vertex_normals(&mesh),
        bvh: Bvh::build(&mesh),
        bbox: mesh.bbox(),
        median_edge: median_edge(&mesh),
        inverted_pieces: inverted_pieces(&mesh),
        topology: topology(&mesh),
        mesh,
        dropped,
        non_manifold,
        flipped,
        across,
        border_distance,
    })
}

impl Scan {
    /// Is the edge `k` of triangle `t` on the border of the scan?
    pub(super) fn is_border(&self, t: usize, k: usize) -> bool {
        self.across.get(t).and_then(|a| a.get(k)).is_none_or(|&x| x == NONE)
    }

    /// Unit normal of triangle `t` (zero when degenerate).
    pub(super) fn face_normal(&self, t: usize) -> Vec3 {
        self.mesh.face_normal(t).unwrap_or_else(Vec3::zeros)
    }

    /// Is the closest point `hit` (on triangle `t`) inside the scan: not on one of its border edges?
    /// A point beyond the border of a scan finds its closest point on the border edge.
    pub(super) fn interior(&self, t: usize, hit: &Point3) -> bool {
        let Some(tri) = self.mesh.triangle(t) else { return false };
        let w = barycentric(&tri, hit);
        // the barycentric coordinate opposite edge k (t[k] → t[k + 1]) is that of corner k + 2
        (0..3).all(|k| !(self.is_border(t, k) && w[(k + 2) % 3] <= BORDER_SNAP))
    }

    /// The border distance at `hit` on triangle `t`, interpolated over its corners.
    pub(super) fn distance_at(&self, t: usize, hit: &Point3) -> f64 {
        let (Some(tri), Some(ids)) = (self.mesh.triangle(t), self.mesh.triangles.get(t)) else { return 0.0 };
        let w = barycentric(&tri, hit);
        let d = ids.map(|v| self.border_distance.get(v as usize).copied().unwrap_or(0.0));
        if d.iter().all(|x| x.is_infinite()) {
            return f64::INFINITY;
        }
        // a corner at ∞ with weight 0 would give NaN: corners at ∞ only count with weight
        (0..3).map(|k| if w[k] > 0.0 { w[k] * d[k] } else { 0.0 }).sum()
    }
}

/// A closest point within this barycentric share of an edge lies on it.
const BORDER_SNAP: f64 = 1e-9;

/// Barycentric coordinates of `p` (on or near the triangle) clamped to the triangle.
pub(super) fn barycentric(t: &[Point3; 3], p: &Point3) -> [f64; 3] {
    let (e0, e1, v) = (t[1] - t[0], t[2] - t[0], p - t[0]);
    let (d00, d01, d11) = (e0.dot(&e0), e0.dot(&e1), e1.dot(&e1));
    let (d20, d21) = (v.dot(&e0), v.dot(&e1));
    let den = d00 * d11 - d01 * d01;
    if !(den > 0.0) || !den.is_finite() {
        return [1.0 / 3.0; 3];
    }
    let b = ((d11 * d20 - d01 * d21) / den).clamp(0.0, 1.0);
    let c = ((d00 * d21 - d01 * d20) / den).clamp(0.0, 1.0 - b);
    [(1.0 - b - c).max(0.0), b, c]
}

/// Barycentric coordinates of `p` (in the plane of the triangle) without clamping; all three
/// positive inside. Zeros for a degenerate triangle.
pub(super) fn barycentric_open(t: &[Point3; 3], p: &Point3) -> [f64; 3] {
    let (e0, e1, v) = (t[1] - t[0], t[2] - t[0], p - t[0]);
    let (d00, d01, d11) = (e0.dot(&e0), e0.dot(&e1), e1.dot(&e1));
    let (d20, d21) = (v.dot(&e0), v.dot(&e1));
    let den = d00 * d11 - d01 * d01;
    if !(den > 0.0) || !den.is_finite() {
        return [0.0; 3];
    }
    let b = (d11 * d20 - d01 * d21) / den;
    let c = (d00 * d21 - d01 * d20) / den;
    [1.0 - b - c, b, c]
}
