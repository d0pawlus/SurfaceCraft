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

/// Triangle BVH. Holds a copy of the triangle corners so it outlives edits to the mesh.
#[derive(Clone, Debug, Default)]
pub struct Bvh {
    tris: Vec<[Point3; 3]>,
    order: Vec<u32>,
    nodes: Vec<Node>,
}

impl Bvh {
    /// Builds over every valid triangle of `mesh`.
    #[must_use]
    pub fn build(mesh: &Mesh) -> Self {
        let tris: Vec<[Point3; 3]> =
            (0..mesh.triangle_count()).map(|i| mesh.triangle(i).unwrap_or([Point3::origin(); 3])).collect();
        let order: Vec<u32> = (0..tris.len() as u32).collect();
        let mut b = Bvh { tris, order, nodes: Vec::new() };
        if !b.order.is_empty() {
            let n = b.order.len();
            b.build_node(0, n, 0);
        }
        b
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

    fn tri_box(&self, oi: usize) -> Aabb {
        let t = self.order.get(oi).and_then(|&i| self.tris.get(i as usize));
        t.map_or_else(Aabb::empty, |t| Aabb::from_points(t.iter()))
    }

    fn centroid(&self, ti: u32) -> Point3 {
        self.tris
            .get(ti as usize)
            .map_or_else(Point3::origin, |t| Point3::from((t[0].coords + t[1].coords + t[2].coords) / 3.0))
    }

    fn build_node(&mut self, start: usize, end: usize, depth: usize) -> u32 {
        let mut bbox = Aabb::empty();
        for oi in start..end {
            let b = self.tri_box(oi);
            if !b.is_empty() {
                bbox.grow(&b.min);
                bbox.grow(&b.max);
            }
        }
        let id = self.nodes.len() as u32;
        self.nodes.push(Node { bbox, start: start as u32, end: end as u32, left: 0, right: 0, leaf: true });
        if end - start <= LEAF || depth > 60 {
            return id;
        }
        let mut cb = Aabb::empty();
        for oi in start..end {
            if let Some(&ti) = self.order.get(oi) {
                cb.grow(&self.centroid(ti));
            }
        }
        let size = cb.size();
        let axis = if size.x >= size.y && size.x >= size.z {
            0
        } else if size.y >= size.z {
            1
        } else {
            2
        };
        let mid = start + (end - start) / 2;
        let tris = &self.tris;
        if let Some(slice) = self.order.get_mut(start..end) {
            slice.select_nth_unstable_by(mid - start, |&a, &b| {
                let ca = tris.get(a as usize).map_or(0.0, |t| t[0][axis] + t[1][axis] + t[2][axis]);
                let cb = tris.get(b as usize).map_or(0.0, |t| t[0][axis] + t[1][axis] + t[2][axis]);
                ca.total_cmp(&cb)
            });
        }
        let left = self.build_node(start, mid, depth + 1);
        let right = self.build_node(mid, end, depth + 1);
        if let Some(n) = self.nodes.get_mut(id as usize) {
            n.leaf = false;
            n.left = left;
            n.right = right;
        }
        id
    }

    /// Closest point on the surface to `q` within `max_dist` (∞ for unbounded).
    #[must_use]
    pub fn closest(&self, q: &Point3, max_dist: f64) -> Option<SurfaceHit> {
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
                        if d2 < best_d2 {
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
        if self.nodes.is_empty() || !crate::finite(origin) || !crate::finite_vec(dir) {
            return None;
        }
        let inv = Vec3::new(1.0 / dir.x, 1.0 / dir.y, 1.0 / dir.z);
        let mut best: Option<(usize, f64)> = None;
        let mut best_t = f64::INFINITY;
        let mut stack: Vec<u32> = vec![0];
        while let Some(ni) = stack.pop() {
            let Some(node) = self.nodes.get(ni as usize) else { continue };
            if !slab(&node.bbox, origin, &inv, best_t) {
                continue;
            }
            if node.leaf {
                for &ti in self.order.get(node.start as usize..node.end as usize).unwrap_or(&[]) {
                    if let Some([a, b, c]) = self.tris.get(ti as usize)
                        && let Some(t) = tri::ray_hit(origin, dir, a, b, c)
                        && t < best_t
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
        best
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
}
