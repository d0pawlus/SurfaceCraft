//! Crossings of the shell mesh with itself, told apart by where they lie: LOCAL ones (the two
//! triangles are close to each other along the scanned surface: neighbouring offset triangles, the
//! two flanks of a crease, the two sheets of a wall that collapsed, the strip along the rim and the
//! sheets next to it) are a fault of the offset and are relaxed away; NON-LOCAL ones (two different
//! areas of the part closer than the wall: a channel narrower than the wall) are real geometry and
//! stay reported. The predicate is that of [`nesting::sampled_self_intersections`] (two triangles
//! that share no vertex, an edge of one passes through the other), applied to the shell mesh as it is
//! returned, so a sheet triangle and the offset copy of its neighbour are tested against each other.
use super::strip::Rim;
use super::topology::Csr;
use crate::{Aabb, Bvh, Mesh, Point3, nesting, par, tri};

/// Crossing triangles whose vertices are within this many thicknesses of each other along the surface
/// are local; farther ones are left to the report.
const LOCAL_REACH: f64 = 2.0;
/// ...and so are those within this many mean edges (on a coarse mesh two triangles that share no
/// vertex are already an edge or two apart).
const LOCAL_RINGS: f64 = 3.0;

/// The scanned surface the shell was made from: how far apart two triangles of the shell lie along it.
pub(super) struct Surface<'a> {
    positions: &'a [Point3],
    adjacent: &'a Csr,
    rim: &'a Rim,
    reach: f64,
}

impl<'a> Surface<'a> {
    /// The surface with its vertices, their neighbours, the rim ids and the wall `thickness` (mm); `edge`
    /// is the mean edge length (mm).
    pub(super) fn new(positions: &'a [Point3], adjacent: &'a Csr, rim: &'a Rim, thickness: f64, edge: f64) -> Self {
        Surface { positions, adjacent, rim, reach: (LOCAL_REACH * thickness).max(LOCAL_RINGS * edge) }
    }

    /// The vertices within the reach of the scan vertices `from` along the scan (ascending; `from` included).
    fn reach_from(&self, from: &[u32]) -> std::collections::HashMap<u32, f64> {
        let mut best: std::collections::HashMap<u32, f64> = from.iter().map(|&v| (v, 0.0)).collect();
        let mut queue: std::collections::VecDeque<u32> = from.iter().copied().collect();
        while let Some(v) = queue.pop_front() {
            let (Some(&dv), Some(pv)) = (best.get(&v), self.positions.get(v as usize)) else { continue };
            for &w in self.adjacent.row(v as usize) {
                let Some(pw) = self.positions.get(w as usize) else { continue };
                let d = dv + (pw - pv).norm();
                if d <= self.reach && best.get(&w).is_none_or(|&old| d < old - 1e-12) {
                    best.insert(w, d);
                    queue.push_back(w);
                }
            }
        }
        best
    }

    /// The scan vertices within the reach of vertex `v` along the scan, ascending.
    pub(super) fn within_reach(&self, v: u32) -> Vec<u32> {
        let mut out: Vec<u32> = self.reach_from(&[v]).into_keys().collect();
        out.sort_unstable();
        out
    }

    /// Do triangles `x` and `y` of the shell mesh lie within the reach of each other along the scan?
    pub(super) fn near(&self, x: &[u32; 3], y: &[u32; 3]) -> bool {
        let from: Vec<u32> = x.iter().map(|&v| self.rim.origin(v)).collect();
        let best = self.reach_from(&from);
        y.iter().any(|&v| best.contains_key(&self.rim.origin(v)))
    }
}

fn boxes_overlap(a: &Aabb, b: &Aabb) -> bool {
    (0..3).all(|k| a.min[k] <= b.max[k] && b.min[k] <= a.max[k])
}

/// Which pairs of triangles count as crossing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Pairs {
    /// Triangles that share no vertex (the predicate of the sampled estimate in [`nesting`]).
    Disjoint,
    /// ...and triangles that share one vertex and whose opposite edges pass through each other, and
    /// triangles that share an edge and lie over each other (the rows of the strip, a sliver of the
    /// offset): the exact test of the relaxation.
    Exact,
}

/// Parameter along an edge within this distance of an end does not count as piercing a triangle (the
/// edge then starts or ends on it: triangles that touch at a vertex, as at a split pinch).
const END_TOLERANCE: f64 = 1e-9;

/// Does an edge of `x` other than those that end in `skip` pass through `y`?
fn edges_pierce(x: &[Point3; 3], xi: &[u32; 3], y: &[Point3; 3], skip: Option<u32>) -> bool {
    (0..3).any(|k| {
        let (u, v) = (xi.get(k).copied().unwrap_or(0), xi.get((k + 1) % 3).copied().unwrap_or(0));
        let (Some(&p), Some(&q)) = (x.get(k), x.get((k + 1) % 3)) else { return false };
        skip.is_none_or(|s| u != s && v != s)
            && tri::ray_hit(&p, &(q - p), &y[0], &y[1], &y[2])
                .is_some_and(|s| s > END_TOLERANCE && s < 1.0 - END_TOLERANCE)
    })
}

/// Cosine of the angle between the normals of two triangles that share an edge below which they lie
/// over each other (folded back by less than a degree) rather than next to each other.
const FOLD_COS: f64 = -0.9998;

/// Do `x` and `y`, which share the edge `(a, b)` (vertex ids), lie over each other?
fn folded_over(x: &[Point3; 3], xi: &[u32; 3], y: &[Point3; 3], yi: &[u32; 3], edge: [u32; 2]) -> bool {
    let corner = |p: &[Point3; 3], i: &[u32; 3], v: u32| i.iter().position(|&w| w == v).and_then(|k| p.get(k).copied());
    let apex = |p: &[Point3; 3], i: &[u32; 3]| i.iter().position(|w| !edge.contains(w)).and_then(|k| p.get(k).copied());
    let (Some(a), Some(b), Some(ox), Some(oy)) =
        (corner(x, xi, edge[0]), corner(x, xi, edge[1]), apex(x, xi), apex(y, yi))
    else {
        return false;
    };
    let (Some(nx), Some(ny)) = (tri::normal(&x[0], &x[1], &x[2]), tri::normal(&y[0], &y[1], &y[2])) else {
        return false;
    };
    let e = b - a;
    let side = |o: Point3| (o - a) - e * ((o - a).dot(&e) / e.norm_squared().max(f64::MIN_POSITIVE));
    nx.dot(&ny) < FOLD_COS && side(ox).dot(&side(oy)) > 0.0
}

fn pair_crosses(x: &[Point3; 3], xi: &[u32; 3], y: &[Point3; 3], yi: &[u32; 3], pairs: Pairs) -> bool {
    let shared: Vec<u32> = xi.iter().copied().filter(|v| yi.contains(v)).collect();
    match (shared.as_slice(), pairs) {
        ([], Pairs::Disjoint) => nesting::crosses(x, y),
        ([], Pairs::Exact) => edges_pierce(x, xi, y, None) || edges_pierce(y, yi, x, None),
        ([s], Pairs::Exact) => edges_pierce(x, xi, y, Some(*s)) || edges_pierce(y, yi, x, Some(*s)),
        ([a, b], Pairs::Exact) => folded_over(x, xi, y, yi, [*a, *b]),
        _ => false,
    }
}

/// The triangles of `mesh` that triangle `id` crosses (ascending); `bvh` is built over `mesh`.
pub(super) fn partners(mesh: &Mesh, bvh: &Bvh, id: usize, pairs: Pairs) -> Vec<u32> {
    let (Some(x), Some(xi)) = (mesh.triangle(id), mesh.triangles.get(id)) else { return Vec::new() };
    let bx = Aabb::from_points(x.iter());
    bvh.candidates(&|b: &Aabb| boxes_overlap(b, &bx))
        .into_iter()
        .filter(|&o| {
            let (Some(y), Some(yi)) = (mesh.triangle(o as usize), mesh.triangles.get(o as usize)) else { return false };
            o as usize != id && pair_crosses(&x, xi, &y, yi, pairs)
        })
        .collect()
}

/// The vertices of the scan (as ids of the scan, ascending) of the local crossings of the triangles
/// `ids` of the shell mesh: both triangles of every pair.
pub(super) fn local_vertices(mesh: &Mesh, bvh: &Bvh, surface: &Surface, ids: &[u32]) -> Vec<u32> {
    let hits: Vec<Vec<u32>> = par::map(ids.len(), |k| {
        let id = ids.get(k).copied().unwrap_or(0) as usize;
        let Some(xi) = mesh.triangles.get(id) else { return Vec::new() };
        let mut out = Vec::new();
        for o in partners(mesh, bvh, id, Pairs::Exact) {
            let Some(yi) = mesh.triangles.get(o as usize) else { continue };
            if surface.near(xi, yi) {
                out.extend(xi.iter().chain(yi.iter()).map(|&v| surface.rim.origin(v)));
            }
        }
        out
    });
    let mut out: Vec<u32> = hits.into_iter().flatten().collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// What an even sample of the shell's triangles found.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct Sample {
    /// Triangles tested.
    pub tested: usize,
    /// Of them, triangles that cross only triangles close to them along the scan.
    pub local: usize,
    /// ...that cross a triangle far from them along the scan.
    pub nonlocal: usize,
}

/// Tests an even stride of at most `max_samples` triangles of `mesh` against the rest.
pub(super) fn sample(mesh: &Mesh, bvh: &Bvh, surface: &Surface, max_samples: usize) -> Sample {
    let n = mesh.triangle_count();
    let count = max_samples.min(n);
    let kinds: Vec<(bool, bool)> = par::map(count, |k| {
        let id = crate::even_index(k, n, count);
        let Some(xi) = mesh.triangles.get(id) else { return (false, false) };
        let partners = partners(mesh, bvh, id, Pairs::Disjoint);
        let far = partners.iter().any(|&o| mesh.triangles.get(o as usize).is_some_and(|yi| !surface.near(xi, yi)));
        (!partners.is_empty() && !far, far)
    });
    Sample {
        tested: count,
        local: kinds.iter().filter(|k| k.0).count(),
        nonlocal: kinds.iter().filter(|k| k.1).count(),
    }
}
