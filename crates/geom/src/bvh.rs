//! A bounding-volume hierarchy over the triangles of a [`Mesh`]: closest point on the surface
//! (deviation, ICP point-to-surface) and ray casting (viewport picking).
use crate::{Aabb, Mesh, Point3, Vec3, tri};

const LEAF: usize = 8;

#[derive(Clone, Debug)]
struct Node {
    bbox: Aabb,
    /// Leaf: `start..end` into `order`; inner: children `left`, `right`.
    start: u32,
    end: u32,
    left: u32,
    right: u32,
    leaf: bool,
}

/// Ranges at least this large are split into two parallel tasks.
const PAR_MIN: usize = 32_768;

fn tri_axis_sum(tris: &[[Point3; 3]], ti: u32, axis: usize) -> f64 {
    tris.get(ti as usize).map_or(0.0, |t| t[0][axis] + t[1][axis] + t[2][axis])
}

fn bounds(tris: &[[Point3; 3]], order: &[u32]) -> (Aabb, Aabb) {
    let mut bbox = Aabb::empty();
    let mut cb = Aabb::empty();
    for &ti in order {
        let Some(t) = tris.get(ti as usize) else { continue };
        for p in t {
            bbox.grow(p);
        }
        cb.grow(&Point3::from((t[0].coords + t[1].coords + t[2].coords) / 3.0));
    }
    (bbox, cb)
}

/// Splits `order` at its median along the longest axis of the centroid box; `None` for a leaf.
fn split<'a>(
    tris: &[[Point3; 3]],
    order: &'a mut [u32],
    cb: &Aabb,
    depth: usize,
) -> Option<(&'a mut [u32], &'a mut [u32])> {
    let n = order.len();
    if n <= LEAF || depth > 60 {
        return None;
    }
    let size = cb.size();
    let axis = if size.x >= size.y && size.x >= size.z {
        0
    } else if size.y >= size.z {
        1
    } else {
        2
    };
    let mid = n / 2;
    order.select_nth_unstable_by(mid, |&a, &b| tri_axis_sum(tris, a, axis).total_cmp(&tri_axis_sum(tris, b, axis)));
    Some(order.split_at_mut(mid))
}

/// Sequential subtree: nodes pushed pre-order into `nodes`; returns the id of the subtree root.
fn build_seq(tris: &[[Point3; 3]], order: &mut [u32], offset: usize, depth: usize, nodes: &mut Vec<Node>) -> u32 {
    let n = order.len();
    let (bbox, cb) = bounds(tris, order);
    let id = nodes.len() as u32;
    nodes.push(Node { bbox, start: offset as u32, end: (offset + n) as u32, left: 0, right: 0, leaf: true });
    if let Some((lo, hi)) = split(tris, order, &cb, depth) {
        let mid = lo.len();
        let left = build_seq(tris, lo, offset, depth + 1, nodes);
        let right = build_seq(tris, hi, offset + mid, depth + 1, nodes);
        if let Some(x) = nodes.get_mut(id as usize) {
            x.leaf = false;
            x.left = left;
            x.right = right;
        }
    }
    id
}

/// Nodes of the subtree over `order` (a slice starting at global position `offset`), pre-order,
/// child ids relative to the returned vector. Subtrees of at least [`PAR_MIN`] triangles are built
/// as two parallel tasks and stitched; smaller ones sequentially.
fn build_range(tris: &[[Point3; 3]], order: &mut [u32], offset: usize, depth: usize) -> Vec<Node> {
    let n = order.len();
    if n < PAR_MIN {
        let mut v = Vec::with_capacity(2 * n / LEAF + 2);
        build_seq(tris, order, offset, depth, &mut v);
        return v;
    }
    let (bbox, cb) = bounds(tris, order);
    let mut node = Node { bbox, start: offset as u32, end: (offset + n) as u32, left: 0, right: 0, leaf: true };
    let Some((lo, hi)) = split(tris, order, &cb, depth) else { return vec![node] };
    let mid = lo.len();
    let (left, right) = crate::par::join(
        || build_range(tris, lo, offset, depth + 1),
        || build_range(tris, hi, offset + mid, depth + 1),
    );
    let (l_shift, r_shift) = (1u32, 1 + left.len() as u32);
    node.leaf = false;
    node.left = l_shift;
    node.right = r_shift;
    let mut nodes = Vec::with_capacity(1 + left.len() + right.len());
    nodes.push(node);
    for (sub, shift) in [(left, l_shift), (right, r_shift)] {
        nodes.extend(sub.into_iter().map(|mut x| {
            if !x.leaf {
                x.left += shift;
                x.right += shift;
            }
            x
        }));
    }
    nodes
}

/// The closest point on a mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SurfaceHit {
    /// Triangle index.
    pub triangle: usize,
    /// Closest point.
    pub point: Point3,
    /// Distance from the query.
    pub distance: f64,
}

/// Outcome of [`Bvh::ray_cast_bounded`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RayCast {
    /// The first hit: triangle and ray parameter.
    Hit(usize, f64),
    /// No triangle is hit.
    Miss,
    /// The bound on the number of triangle tests was reached before the search ended.
    Exhausted,
}

/// Triangle BVH. Holds a copy of the triangle corners so it outlives edits to the mesh.
#[derive(Clone, Debug, Default)]
pub struct Bvh {
    tris: Vec<[Point3; 3]>,
    order: Vec<u32>,
    nodes: Vec<Node>,
}

impl Bvh {
    /// Builds over every valid triangle of `mesh`. Large subtrees are built in parallel; the node
    /// array is the same pre-order layout a sequential build gives.
    #[must_use]
    pub fn build(mesh: &Mesh) -> Self {
        let tris: Vec<[Point3; 3]> =
            crate::par::map(mesh.triangle_count(), |i| mesh.triangle(i).unwrap_or([Point3::origin(); 3]));
        let mut order: Vec<u32> = (0..tris.len() as u32).collect();
        let nodes = if order.is_empty() { Vec::new() } else { build_range(&tris, &mut order, 0, 0) };
        Bvh { tris, order, nodes }
    }

    /// Number of triangles.
    #[must_use]
    pub fn len(&self) -> usize {
        self.tris.len()
    }

    /// Is it empty?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.tris.is_empty()
    }

    /// Unit normal of triangle `i` (`None` when degenerate or missing).
    #[must_use]
    pub fn triangle_normal(&self, i: usize) -> Option<Vec3> {
        let [a, b, c] = self.tris.get(i)?;
        tri::normal(a, b, c)
    }

    /// Closest point on the surface to `q` within `max_dist` (∞ for unbounded).
    #[must_use]
    pub fn closest(&self, q: &Point3, max_dist: f64) -> Option<SurfaceHit> {
        self.closest_where(q, max_dist, &|_| true)
    }

    /// [`Bvh::closest`] among the triangles whose normal makes an angle with one of `facing` whose
    /// cosine is at least `min_cos`. On a thin wall the plain closest point can lie on the far
    /// side; this one stays on the side that faces like `facing`.
    #[must_use]
    pub fn closest_facing(&self, q: &Point3, max_dist: f64, facing: &[Vec3], min_cos: f64) -> Option<SurfaceHit> {
        self.closest_where(q, max_dist, &|i| {
            self.triangle_normal(i).is_some_and(|n| facing.iter().any(|f| n.dot(f) >= min_cos))
        })
    }

    /// Closest point among the triangles `keep` accepts.
    fn closest_where(&self, q: &Point3, max_dist: f64, keep: &dyn Fn(usize) -> bool) -> Option<SurfaceHit> {
        if self.nodes.is_empty() || !crate::finite(q) {
            return None;
        }
        let mut best_d2 = if max_dist.is_finite() { max_dist * max_dist } else { f64::INFINITY };
        let mut best: Option<SurfaceHit> = None;
        let mut stack: Vec<u32> = vec![0];
        while let Some(ni) = stack.pop() {
            let Some(node) = self.nodes.get(ni as usize) else { continue };
            if node.bbox.dist2(q) >= best_d2 {
                continue;
            }
            if node.leaf {
                for &ti in self.order.get(node.start as usize..node.end as usize).unwrap_or(&[]) {
                    if let Some([a, b, c]) = self.tris.get(ti as usize) {
                        let p = tri::closest_point(q, a, b, c);
                        let d2 = (p - q).norm_squared();
                        if d2 < best_d2 && keep(ti as usize) {
                            best_d2 = d2;
                            best = Some(SurfaceHit { triangle: ti as usize, point: p, distance: d2.sqrt() });
                        }
                    }
                }
            } else {
                let (l, r) = (node.left, node.right);
                let dl = self.nodes.get(l as usize).map_or(f64::INFINITY, |n| n.bbox.dist2(q));
                let dr = self.nodes.get(r as usize).map_or(f64::INFINITY, |n| n.bbox.dist2(q));
                if dl < dr {
                    stack.push(r);
                    stack.push(l);
                } else {
                    stack.push(l);
                    stack.push(r);
                }
            }
        }
        best
    }

    /// First triangle hit by the ray `origin + t·dir`: `(triangle, t)`.
    #[must_use]
    pub fn ray_cast(&self, origin: &Point3, dir: &Vec3) -> Option<(usize, f64)> {
        self.ray_cast_where(origin, dir, f64::INFINITY, &|_| true)
    }

    /// First hit of the ray `origin + t·dir` with `t < max_t` among the triangles `keep` accepts.
    #[must_use]
    pub fn ray_cast_where(
        &self,
        origin: &Point3,
        dir: &Vec3,
        max_t: f64,
        keep: &dyn Fn(usize) -> bool,
    ) -> Option<(usize, f64)> {
        match self.ray_cast_bounded(origin, dir, max_t, keep, usize::MAX) {
            RayCast::Hit(t, d) => Some((t, d)),
            RayCast::Miss | RayCast::Exhausted => None,
        }
    }

    /// [`Bvh::ray_cast_where`] that gives up after testing `max_tests` triangles
    /// ([`RayCast::Exhausted`]: a pile of coincident triangles would cost the square of its size).
    #[must_use]
    pub fn ray_cast_bounded(
        &self,
        origin: &Point3,
        dir: &Vec3,
        max_t: f64,
        keep: &dyn Fn(usize) -> bool,
        max_tests: usize,
    ) -> RayCast {
        if self.nodes.is_empty() || !crate::finite(origin) || !crate::finite_vec(dir) {
            return RayCast::Miss;
        }
        let inv = Vec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
        let mut best: Option<(usize, f64)> = None;
        let mut best_t = if max_t.is_nan() { f64::INFINITY } else { max_t };
        let (mut stack, mut tests): (Vec<u32>, usize) = (vec![0], 0);
        while let Some(ni) = stack.pop() {
            let Some(node) = self.nodes.get(ni as usize) else { continue };
            if !slab(&node.bbox, origin, &inv, best_t) {
                continue;
            }
            if node.leaf {
                for &ti in self.order.get(node.start as usize..node.end as usize).unwrap_or(&[]) {
                    tests += 1;
                    if tests > max_tests {
                        return RayCast::Exhausted;
                    }
                    if let Some([a, b, c]) = self.tris.get(ti as usize)
                        && let Some(t) = tri::ray_hit(origin, dir, a, b, c)
                        && t < best_t
                        && keep(ti as usize)
                    {
                        best_t = t;
                        best = Some((ti as usize, t));
                    }
                }
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
        best.map_or(RayCast::Miss, |(t, d)| RayCast::Hit(t, d))
    }

    /// [`Bvh::candidates`] that stops after `max` triangles (ascending among those found).
    #[must_use]
    pub fn candidates_up_to(&self, keep: &dyn Fn(&Aabb) -> bool, max: usize) -> Vec<u32> {
        let mut out = Vec::new();
        let mut stack: Vec<u32> = if self.nodes.is_empty() { Vec::new() } else { vec![0] };
        while let Some(ni) = stack.pop() {
            let Some(node) = self.nodes.get(ni as usize) else { continue };
            if !keep(&node.bbox) {
                continue;
            }
            if node.leaf {
                out.extend_from_slice(self.order.get(node.start as usize..node.end as usize).unwrap_or(&[]));
                if out.len() >= max {
                    break;
                }
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
        out.truncate(max);
        out.sort_unstable();
        out
    }

    /// The triangles under every box on their way down that `keep` accepts, ascending. A box that
    /// `keep` rejects prunes all its triangles, so `keep` must accept every box that can hold a
    /// triangle the caller wants (a box is never smaller than what it holds).
    #[must_use]
    pub fn candidates(&self, keep: &dyn Fn(&Aabb) -> bool) -> Vec<u32> {
        let mut out = Vec::new();
        let mut stack: Vec<u32> = if self.nodes.is_empty() { Vec::new() } else { vec![0] };
        while let Some(ni) = stack.pop() {
            let Some(node) = self.nodes.get(ni as usize) else { continue };
            if !keep(&node.bbox) {
                continue;
            }
            if node.leaf {
                out.extend_from_slice(self.order.get(node.start as usize..node.end as usize).unwrap_or(&[]));
            } else {
                stack.push(node.left);
                stack.push(node.right);
            }
        }
        out.sort_unstable();
        out
    }
}

fn slab(b: &Aabb, o: &Point3, inv: &Vec3, tmax: f64) -> bool {
    if b.is_empty() {
        return false;
    }
    let mut t0 = 0.0f64;
    let mut t1 = tmax;
    for i in 0..3 {
        let mut ta = (b.min[i] - o[i]) * inv[i];
        let mut tb = (b.max[i] - o[i]) * inv[i];
        if ta.is_nan() || tb.is_nan() {
            // ray parallel to the slab and on its plane: accept this axis
            continue;
        }
        if ta > tb {
            std::mem::swap(&mut ta, &mut tb);
        }
        t0 = t0.max(ta);
        t1 = t1.min(tb);
        if t0 > t1 {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rng, synth};

    #[test]
    fn closest_matches_brute_force() {
        let m = synth::sphere(Point3::new(1.0, 2.0, 3.0), 5.0, 24, 12);
        let bvh = Bvh::build(&m);
        let mut rng = Rng::new(9);
        for _ in 0..100 {
            let q = Point3::new(rng.range(-8.0, 8.0), rng.range(-8.0, 8.0), rng.range(-8.0, 8.0));
            let brute = (0..m.triangle_count())
                .map(|i| {
                    let [a, b, c] = m.triangle(i).unwrap();
                    (tri::closest_point(&q, &a, &b, &c) - q).norm()
                })
                .fold(f64::INFINITY, f64::min);
            let got = bvh.closest(&q, f64::INFINITY).unwrap();
            assert!((got.distance - brute).abs() < 1e-12);
        }
    }

    #[test]
    fn ray_hits_sphere_front() {
        let m = synth::sphere(Point3::origin(), 5.0, 48, 24);
        let bvh = Bvh::build(&m);
        let (_, t) = bvh.ray_cast(&Point3::new(0.1, 0.2, 20.0), &Vec3::new(0.0, 0.0, -1.0)).unwrap();
        assert!((t - 15.0).abs() < 0.1);
        assert!(bvh.ray_cast(&Point3::new(9.0, 9.0, 20.0), &Vec3::new(0.0, 0.0, -1.0)).is_none());
    }

    #[test]
    fn candidates_keep_every_triangle_whose_centroid_is_in_the_box() {
        let m = synth::sphere(Point3::origin(), 5.0, 48, 24);
        let bvh = Bvh::build(&m);
        let all = bvh.candidates(&|_| true);
        assert_eq!(all, (0..m.triangle_count() as u32).collect::<Vec<u32>>(), "ascending, each once");
        let region = Aabb { min: Point3::new(-1.0, -1.0, 2.0), max: Point3::new(1.0, 1.0, 6.0) };
        let kept = bvh.candidates(&|b| {
            b.min.x <= region.max.x
                && b.max.x >= region.min.x
                && b.min.y <= region.max.y
                && b.max.y >= region.min.y
                && b.min.z <= region.max.z
                && b.max.z >= region.min.z
        });
        for t in 0..m.triangle_count() {
            let c = m.triangle(t).map(|[a, b, c]| Point3::from((a.coords + b.coords + c.coords) / 3.0));
            if c.is_some_and(|c| {
                region.min.x <= c.x
                    && c.x <= region.max.x
                    && region.min.y <= c.y
                    && c.y <= region.max.y
                    && region.min.z <= c.z
                    && c.z <= region.max.z
            }) {
                assert!(kept.contains(&(t as u32)), "triangle {t} lies in the box and was pruned");
            }
        }
        assert!(kept.len() < m.triangle_count() / 4, "the box prunes most of the sphere: {}", kept.len());
        assert!(bvh.candidates(&|_| false).is_empty());
        assert!(Bvh::default().candidates(&|_| true).is_empty());
    }
}
