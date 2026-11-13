//! Crossings of the shell mesh with itself, told apart by where they lie: LOCAL ones (the two
//! triangles are close to each other along the scanned surface: neighbouring offset triangles, the
//! two flanks of a crease, the two sheets of a wall that collapsed, the strip along the rim and the
//! sheets next to it) are a fault of the offset and are relaxed away; NON-LOCAL ones (two different
//! areas of the part closer than the wall: a channel narrower than the wall) are real geometry and
//! stay reported. The predicate is that of `nesting::sampled_self_intersections` (two triangles
//! that share no vertex, an edge of one passes through the other), applied to the shell mesh as it is
//! returned, so a sheet triangle and the offset copy of its neighbour are tested against each other.
use super::strip::Rim;
use super::topology::Csr;
use crate::{Aabb, Bvh, Mesh, Point3, Vec3, par, tri};

/// Crossing triangles whose vertices are within this many thicknesses of each other along the surface
/// are local; farther ones are left to the report.
const LOCAL_REACH: f64 = 2.0;
/// ...and so are those within this many median edges (on a coarse mesh two triangles that share no
/// vertex are already an edge or two apart).
const LOCAL_RINGS: f64 = 3.0;
/// The reach is at most this share of the size (bounding box diagonal) of the part: a wall that is a
/// large part of the part would otherwise make every crossing local.
const REACH_SIZE_SHARE: f64 = 0.5;
/// The flanks of a crease are close in space but farther apart along the surface (a crease of half angle
/// `a`: `1 / sin a` times): two triangles also count as near when the way along the surface is at most
/// this many times the distance between them in space (a channel is much worse: its walls are joined
/// through the floor).
const CREASE_RATIO: f64 = 10.0;
/// A search along the surface gives up after this many vertices.
const MAX_VISITS: usize = 20_000;
/// A row of the adjacency is read up to this many entries (a hub of a fan).
const MAX_ROW: usize = 4_096;

/// How far apart along the scan two things must be to be told apart: two thicknesses or three median
/// edges, at most [`REACH_SIZE_SHARE`] of the `size` of the part (mm).
pub(super) fn reach_for(thickness: f64, edge: f64, size: f64) -> f64 {
    (LOCAL_REACH * thickness).max(LOCAL_RINGS * edge).min(REACH_SIZE_SHARE * size)
}

/// Result of a search along the scan.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Reach {
    /// A target lies within the reach.
    Found,
    /// None does.
    Missed,
    /// The search gave up (a huge neighbourhood): unknown. Callers treat it as within reach, which is
    /// the safe side for the bound and for the relaxation.
    Truncated,
}

/// The scanned surface the shell was made from: how far apart two triangles of the shell lie along it.
pub(super) struct Surface<'a> {
    positions: &'a [Point3],
    adjacent: &'a Csr,
    rim: &'a Rim,
    reach: f64,
    limit: f64,
}

impl<'a> Surface<'a> {
    /// The surface with its vertices, their neighbours, the rim ids and the wall `thickness` (mm); `edge`
    /// is the median edge length and `size` the diagonal of the bounding box (mm).
    pub(super) fn new(
        positions: &'a [Point3],
        adjacent: &'a Csr,
        rim: &'a Rim,
        thickness: f64,
        edge: f64,
        size: f64,
    ) -> Self {
        Surface { positions, adjacent, rim, reach: reach_for(thickness, edge, size), limit: REACH_SIZE_SHARE * size }
    }

    /// Is a vertex for which `target` holds within the reach of the scan vertices `from`?
    pub(super) fn reaches(&self, from: &[u32], target: &dyn Fn(u32) -> bool) -> Reach {
        self.reaches_within(from, target, self.reach)
    }

    fn reaches_within(&self, from: &[u32], target: &dyn Fn(u32) -> bool, reach: f64) -> Reach {
        if from.iter().any(|&v| target(v)) {
            return Reach::Found;
        }
        let mut best: std::collections::HashMap<u32, f64> = from.iter().map(|&v| (v, 0.0)).collect();
        let mut queue: std::collections::VecDeque<u32> = from.iter().copied().collect();
        let mut truncated = false;
        while let Some(v) = queue.pop_front() {
            let (Some(&dv), Some(pv)) = (best.get(&v), self.positions.get(v as usize)) else { continue };
            let row = self.adjacent.row(v as usize);
            truncated |= row.len() > MAX_ROW;
            for &w in row.iter().take(MAX_ROW) {
                let Some(pw) = self.positions.get(w as usize) else { continue };
                let d = dv + (pw - pv).norm();
                if d <= reach && best.get(&w).is_none_or(|&old| d < old - 1e-12) {
                    if target(w) {
                        return Reach::Found;
                    }
                    best.insert(w, d);
                    queue.push_back(w);
                }
            }
            if best.len() > MAX_VISITS {
                return Reach::Truncated;
            }
        }
        if truncated { Reach::Truncated } else { Reach::Missed }
    }

    /// Do triangles `x` and `y` of the shell mesh lie within the reach of each other along the scan?
    /// (Unknown counts as near.)
    pub(super) fn near(&self, x: &[u32; 3], y: &[u32; 3]) -> bool {
        let from: Vec<u32> = x.iter().filter_map(|&v| self.rim.origin(v)).collect();
        let to: Vec<u32> = y.iter().filter_map(|&v| self.rim.origin(v)).collect();
        let gap = from
            .iter()
            .flat_map(|&u| to.iter().map(move |&w| (u, w)))
            .filter_map(|(u, w)| Some((self.positions.get(u as usize)? - self.positions.get(w as usize)?).norm()))
            .fold(f64::INFINITY, f64::min);
        let reach = if gap.is_finite() { self.reach.max(CREASE_RATIO * gap).min(self.limit) } else { self.reach };
        self.reaches_within(&from, &|w| to.contains(&w), reach) != Reach::Missed
    }

    /// The scan vertex of the shell vertex `v`.
    pub(super) fn origin(&self, v: u32) -> Option<u32> {
        self.rim.origin(v)
    }
}

fn boxes_overlap(a: &Aabb, b: &Aabb) -> bool {
    (0..3).all(|k| a.min[k] <= b.max[k] && b.min[k] <= a.max[k])
}

/// Which pairs of triangles count as crossing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Pairs {
    /// Triangles that share no vertex (the predicate of the sampled estimate in `nesting`, with a tolerance
    /// relative to the size of the triangles).
    Disjoint,
    /// ...and triangles that share one vertex and whose opposite edges pass through each other, and
    /// triangles that share an edge and lie over each other (the rows of the strip, a sliver of the
    /// offset): the exact test of the relaxation.
    Exact,
}

/// Parameter along an edge within this distance of an end does not count as piercing a triangle (the
/// edge then starts or ends on it: triangles that touch at a vertex, as at a split pinch).
const END_TOLERANCE: f64 = 1e-9;
/// A segment whose determinant against a triangle is below this share of the product of the lengths
/// (direction and both edges) is parallel to it. Relative, so that the test does not depend on the unit.
const PARALLEL: f64 = 1e-12;

/// Parameter `s` in `[0, 1]` at which the segment `p`-`q` passes through the triangle `a`, `b`, `c`
/// (Möller and Trumbore, "Fast, minimum storage ray-triangle intersection", J. Graphics Tools 2(1),
/// 1997, with a tolerance relative to the lengths).
fn segment_hit(p: &Point3, q: &Point3, a: &Point3, b: &Point3, c: &Point3) -> Option<f64> {
    let dir = q - p;
    let (e1, e2) = (b - a, c - a);
    let pv = dir.cross(&e2);
    let det = e1.dot(&pv);
    if !(det.abs() > PARALLEL * dir.norm() * e1.norm() * e2.norm()) {
        return None;
    }
    let tv = p - a;
    let u = tv.dot(&pv) / det;
    if !(0.0..=1.0).contains(&u) {
        return None;
    }
    let qv = tv.cross(&e1);
    let v = dir.dot(&qv) / det;
    if v < 0.0 || u + v > 1.0 {
        return None;
    }
    let s = e2.dot(&qv) / det;
    (0.0..=1.0).contains(&s).then_some(s)
}

/// Does an edge of one triangle pass through the other (touching counts)? The predicate of
/// [`nesting::crosses`], with the relative tolerance of [`segment_hit`].
fn crosses_scaled(x: &[Point3; 3], y: &[Point3; 3]) -> bool {
    let hit = |s: &[Point3; 3], t: &[Point3; 3]| {
        (0..3).any(|k| {
            let (Some(p), Some(q)) = (s.get(k), s.get((k + 1) % 3)) else { return false };
            segment_hit(p, q, &t[0], &t[1], &t[2]).is_some()
        })
    };
    hit(x, y) || hit(y, x)
}

/// Does an edge of `x` other than those that end in `skip` pass through `y`?
fn edges_pierce(x: &[Point3; 3], xi: &[u32; 3], y: &[Point3; 3], skip: Option<u32>) -> bool {
    (0..3).any(|k| {
        let (u, v) = (xi.get(k).copied().unwrap_or(0), xi.get((k + 1) % 3).copied().unwrap_or(0));
        let (Some(&p), Some(&q)) = (x.get(k), x.get((k + 1) % 3)) else { return false };
        skip.is_none_or(|s| u != s && v != s)
            && segment_hit(&p, &q, &y[0], &y[1], &y[2]).is_some_and(|s| s > END_TOLERANCE && s < 1.0 - END_TOLERANCE)
    })
}

/// Absolute cosine of the angle between the normals of two triangles above which they are coplanar.
const COPLANAR_COS: f64 = 0.9998;

/// Do `x` and `y`, which share the edge `(a, b)` (vertex ids), lie over each other (coplanar, on the same
/// side of the edge, whatever their orientation)?
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
    nx.dot(&ny).abs() > COPLANAR_COS && side(ox).dot(&side(oy)) > 0.0
}

/// Do two triangles that share the vertex `s` and lie in one plane overlap around it (their wedges at
/// `s` overlap)?
fn wedges_overlap(x: &[Point3; 3], xi: &[u32; 3], y: &[Point3; 3], yi: &[u32; 3], s: u32) -> bool {
    let (Some(nx), Some(ny)) = (tri::normal(&x[0], &x[1], &x[2]), tri::normal(&y[0], &y[1], &y[2])) else {
        return false;
    };
    if nx.dot(&ny).abs() <= COPLANAR_COS {
        return false;
    }
    let arms = |p: &[Point3; 3], i: &[u32; 3]| -> Option<[Vec3; 2]> {
        let at = i.iter().position(|&w| w == s)?;
        Some([*p.get((at + 1) % 3)? - *p.get(at)?, *p.get((at + 2) % 3)? - *p.get(at)?])
    };
    let (Some(ax), Some(ay)) = (arms(x, xi), arms(y, yi)) else { return false };
    let turn = |a: &Vec3, b: &Vec3| nx.dot(&a.cross(b));
    let sign = turn(&ax[0], &ax[1]).signum();
    let inside = |d: &Vec3| turn(&ax[0], d) * sign > 0.0 && turn(d, &ax[1]) * sign > 0.0;
    let same = |a: &Vec3, b: &Vec3| turn(a, b).abs() <= 1e-9 * a.norm() * b.norm() && a.dot(b) > 0.0;
    let inside_y = |d: &Vec3| {
        let sy = turn(&ay[0], &ay[1]).signum();
        turn(&ay[0], d) * sy > 0.0 && turn(d, &ay[1]) * sy > 0.0
    };
    ay.iter().any(inside)
        || ax.iter().any(inside_y)
        || (same(&ax[0], &ay[0]) && same(&ax[1], &ay[1]))
        || (same(&ax[0], &ay[1]) && same(&ax[1], &ay[0]))
}

fn pair_crosses(x: &[Point3; 3], xi: &[u32; 3], y: &[Point3; 3], yi: &[u32; 3], pairs: Pairs) -> bool {
    let shared: Vec<u32> = xi.iter().copied().filter(|v| yi.contains(v)).collect();
    match (shared.as_slice(), pairs) {
        ([], Pairs::Disjoint) => crosses_scaled(x, y),
        ([], Pairs::Exact) => edges_pierce(x, xi, y, None) || edges_pierce(y, yi, x, None),
        ([s], Pairs::Exact) => {
            edges_pierce(x, xi, y, Some(*s)) || edges_pierce(y, yi, x, Some(*s)) || wedges_overlap(x, xi, y, yi, *s)
        }
        ([a, b], Pairs::Exact) => folded_over(x, xi, y, yi, [*a, *b]),
        _ => false,
    }
}

/// Candidates tested per triangle at most: a pile of coincident triangles would cost the square of its
/// size. (Such a pile crosses itself everywhere; the first candidates show it.)
const MAX_CANDIDATES: usize = 1_024;

/// The triangles of `mesh` that triangle `id` crosses (ascending); `bvh` is built over `mesh`.
pub(super) fn partners(mesh: &Mesh, bvh: &Bvh, id: usize, pairs: Pairs) -> Vec<u32> {
    let (Some(x), Some(xi)) = (mesh.triangle(id), mesh.triangles.get(id)) else { return Vec::new() };
    let bx = Aabb::from_points(x.iter());
    bvh.candidates_up_to(&|b: &Aabb| boxes_overlap(b, &bx), MAX_CANDIDATES)
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
                out.extend(xi.iter().chain(yi.iter()).filter_map(|&v| surface.origin(v)));
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

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    #[test]
    fn two_coplanar_triangles_that_share_a_vertex_and_overlap_around_it_are_found() {
        // Arrange: wedges 0-60 degrees and 30-90 degrees at the origin, and 90-150 degrees (apart)
        let at = |deg: f64| p(deg.to_radians().cos(), deg.to_radians().sin(), 0.0);
        let x = [p(0.0, 0.0, 0.0), at(0.0), at(60.0)];
        let overlapping = [p(0.0, 0.0, 0.0), at(30.0), at(90.0)];
        let apart = [p(0.0, 0.0, 0.0), at(90.0), at(150.0)];
        let same = [p(0.0, 0.0, 0.0), at(60.0), at(0.0)];
        let ids_x = [0u32, 1, 2];

        // Act and assert
        assert!(wedges_overlap(&x, &ids_x, &overlapping, &[0, 3, 4], 0));
        assert!(!wedges_overlap(&x, &ids_x, &apart, &[0, 3, 4], 0));
        assert!(wedges_overlap(&x, &ids_x, &same, &[0, 3, 4], 0), "identical wedges overlap");
        assert!(pair_crosses(&x, &ids_x, &overlapping, &[0, 3, 4], Pairs::Exact));
        assert!(!pair_crosses(&x, &ids_x, &overlapping, &[0, 3, 4], Pairs::Disjoint));
    }

    #[test]
    fn two_triangles_that_share_an_edge_and_lie_over_each_other_are_found_in_either_orientation() {
        // Arrange: the apexes on the same side of the shared edge, one triangle wound each way
        let (a, b) = (p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0));
        let x = [a, b, p(0.5, 1.0, 0.0)];
        let same_way = [a, b, p(0.2, 0.5, 0.0)];
        let other_way = [b, a, p(0.2, 0.5, 0.0)];
        let across = [b, a, p(0.5, -1.0, 0.0)];

        // Act and assert
        assert!(folded_over(&x, &[0, 1, 2], &same_way, &[0, 1, 3], [0, 1]), "same orientation, same side");
        assert!(folded_over(&x, &[0, 1, 2], &other_way, &[1, 0, 3], [0, 1]), "folded back");
        assert!(!folded_over(&x, &[0, 1, 2], &across, &[1, 0, 3], [0, 1]), "a flat neighbour is no crossing");
    }

    #[test]
    fn the_piercing_test_does_not_depend_on_the_unit() {
        // Arrange: an edge through a triangle, in metres and in micrometres
        for scale in [1e-6, 1e-3, 1.0, 1e3] {
            let tri = [p(0.0, 0.0, 0.0), p(2.0 * scale, 0.0, 0.0), p(0.0, 2.0 * scale, 0.0)];
            let (below, above) = (p(0.5 * scale, 0.5 * scale, -scale), p(0.5 * scale, 0.5 * scale, scale));

            // Act and assert
            let hit = segment_hit(&below, &above, &tri[0], &tri[1], &tri[2]);
            assert!(hit.is_some_and(|s| (s - 0.5).abs() < 1e-9), "scale {scale}: {hit:?}");
        }
    }

    #[test]
    fn the_reach_is_clamped_to_a_share_of_the_size_and_follows_the_thickness_and_the_edges() {
        // Arrange and act and assert
        assert_eq!(reach_for(2.0, 0.1, 1000.0), 4.0);
        assert_eq!(reach_for(0.1, 1.0, 1000.0), 3.0);
        assert_eq!(reach_for(300.0, 1.0, 100.0), REACH_SIZE_SHARE * 100.0, "a wall of 3 diagonals");
    }
}
