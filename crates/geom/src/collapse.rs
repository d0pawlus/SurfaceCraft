//! The edge-collapse loop of the decimation: a heap of candidate edges ordered by quadric cost,
//! each collapse checked (link condition, flipped faces, optionally the measured guard) before it
//! is made. State and steps are split into methods of [`Collapser`].
//!
//! Orientation. A collapse is refused when it turns a triangle against the surface it came from.
//! Every vertex carries the sum of the area-weighted normals of the original triangles it stands
//! for (merged like the quadrics), so a triangle is compared with the original orientation of its
//! whole neighbourhood and not only with its own previous normal: a chain of collapses that each
//! turn a triangle by a few tens of degrees can no longer fold it over the surface (the way a thin
//! wall or a part smaller than the budget is crumpled).
use crate::decimate::Guard;
use crate::{Bvh, Point3, Vec3, par, tri};
use nalgebra::{Matrix4, Vector4};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashSet};

/// A symmetric 4×4 quadric.
pub(crate) type Quadric = Matrix4<f64>;

/// Collapses whose quadric error is below this fraction of the budget are not measured.
const GUARD_SKIP: f64 = 0.25;
/// With a guard, a collapse may not grow a vertex above this many neighbours (or above the valence
/// it already has). Flat regions collapse at zero cost, and without the cap they pile up in a few
/// hubs whose rings make every further collapse linear in the ring (minutes on a 100k-triangle block).
const GUARD_VALENCE: usize = 32;
/// Edges from which on the first candidates are costed in parallel.
const SEED_PARALLEL_MIN: usize = 20_000;

/// The original orientation of every vertex: the sum of the area-weighted normals (the cross
/// products) of the triangles around it.
pub(crate) fn vertex_origins(tris: &[[u32; 3]], pos: &[Point3]) -> Vec<Vec3> {
    let mut origin = vec![Vec3::zeros(); pos.len()];
    for t in tris {
        let n = tri::cross(&pos[t[0] as usize], &pos[t[1] as usize], &pos[t[2] as usize]);
        for &v in t {
            origin[v as usize] += n;
        }
    }
    origin
}

pub(crate) fn plane_quadric(n: &crate::Vec3, p: &Point3, w: f64) -> Quadric {
    let v = Vector4::new(n.x, n.y, n.z, -n.dot(&p.coords));
    v * v.transpose() * w
}

#[derive(Clone, Copy, PartialEq)]
struct Cand {
    cost: f64,
    a: u32,
    b: u32,
    stamp: (u32, u32),
    target: [f64; 3],
}
impl Eq for Cand {}
impl Ord for Cand {
    fn cmp(&self, o: &Self) -> Ordering {
        o.cost.total_cmp(&self.cost).then(o.a.cmp(&self.a)).then(o.b.cmp(&self.b))
    }
}
impl PartialOrd for Cand {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

fn cost_of(q: &Quadric, p: &Point3) -> f64 {
    let v = Vector4::new(p.x, p.y, p.z, 1.0);
    (v.transpose() * q * v)[(0, 0)].max(0.0)
}

/// The position of the merged vertex with the least quadric cost, and that cost.
fn best_target(q: &Quadric, pa: &Point3, pb: &Point3) -> (Point3, f64) {
    let a = q.fixed_view::<3, 3>(0, 0).into_owned();
    let bvec = -q.fixed_view::<3, 1>(0, 3).into_owned();
    let mut cands = vec![*pa, *pb, Point3::from((pa.coords + pb.coords) / 2.0)];
    if a.determinant().abs() > 1e-10
        && let Some(x) = a.lu().solve(&bvec)
    {
        let p = Point3::from(x);
        // only when close to the edge (no runaway solutions in flat regions)
        let l = (pb - pa).norm();
        if crate::finite(&p) && (p - Point3::from((pa.coords + pb.coords) / 2.0)).norm() <= 2.0 * l {
            cands.push(p);
        }
    }
    cands.into_iter().map(|p| (p, cost_of(q, &p))).min_by(|x, y| x.1.total_cmp(&y.1)).unwrap_or((*pa, 0.0))
}

/// Is the surface after a collapse within `limit` of the original? Measured: the new vertex and the
/// centroids of the new triangles (`fan`) against the original surface, and the two vertices
/// that vanish (`old`) against the new triangles.
fn collapse_within(original: &Bvh, limit: f64, p: &Point3, old: &[Point3], fan: &[[Point3; 3]]) -> bool {
    if original.closest(p, limit).is_none() {
        return false;
    }
    for t in fan {
        let centroid = Point3::from((t[0].coords + t[1].coords + t[2].coords) / 3.0);
        if original.closest(&centroid, limit).is_none() {
            return false;
        }
    }
    old.iter().all(|q| fan.iter().any(|t| (tri::closest_point(q, &t[0], &t[1], &t[2]) - q).norm() <= limit))
}

/// A collapse that passed every check.
struct Plan {
    fb: Vec<u32>,
    shared: Vec<u32>,
    to: Point3,
}

/// The state of one decimation.
pub(crate) struct Collapser<'a> {
    tris: Vec<[u32; 3]>,
    pos: Vec<Point3>,
    q: Vec<Quadric>,
    /// Sum of the area-weighted normals of the original triangles each vertex stands for.
    origin: Vec<Vec3>,
    locked: Vec<bool>,
    alive_t: Vec<bool>,
    alive_v: Vec<bool>,
    stamp: Vec<u32>,
    vfaces: Vec<Vec<u32>>,
    heap: BinaryHeap<Cand>,
    seen_pair: HashSet<(u32, u32)>,
    guard: Option<Guard<'a>>,
    live: usize,
    /// Tests switch the orientation check off to show what it prevents, and count its refusals.
    #[cfg(test)]
    pub(crate) orient_guard: bool,
    #[cfg(test)]
    pub(crate) turns_refused: std::sync::atomic::AtomicUsize,
}

impl<'a> Collapser<'a> {
    /// Starts from `tris` over `pos` with the vertex quadrics `q`, the original orientation of each
    /// vertex (`origin`, see [`vertex_origins`]) and the vertices that must not move (`locked`);
    /// every edge is a candidate (costed in parallel when `parallel_seed`).
    pub(crate) fn new(
        tris: Vec<[u32; 3]>,
        pos: Vec<Point3>,
        q: Vec<Quadric>,
        origin: Vec<Vec3>,
        locked: Vec<bool>,
        guard: Option<Guard<'a>>,
        parallel_seed: bool,
    ) -> Self {
        let nv = pos.len();
        let mut vfaces: Vec<Vec<u32>> = vec![Vec::new(); nv];
        for (i, t) in tris.iter().enumerate() {
            for &v in t {
                vfaces[v as usize].push(i as u32);
            }
        }
        let live = tris.len();
        let mut s = Self {
            alive_t: vec![true; live],
            alive_v: vec![true; nv],
            stamp: vec![0; nv],
            heap: BinaryHeap::new(),
            seen_pair: HashSet::with_capacity(live * 3 / 2),
            tris,
            pos,
            q,
            origin,
            locked,
            vfaces,
            guard,
            live,
            #[cfg(test)]
            orient_guard: true,
            #[cfg(test)]
            turns_refused: std::sync::atomic::AtomicUsize::new(0),
        };
        s.seed(parallel_seed);
        s
    }

    fn candidate(&self, a: u32, b: u32) -> Option<Cand> {
        if self.locked[a as usize] || self.locked[b as usize] {
            return None;
        }
        let qq = self.q[a as usize] + self.q[b as usize];
        let (p, c) = best_target(&qq, &self.pos[a as usize], &self.pos[b as usize]);
        Some(Cand { cost: c, a, b, stamp: (self.stamp[a as usize], self.stamp[b as usize]), target: [p.x, p.y, p.z] })
    }

    fn push(&mut self, a: u32, b: u32) {
        if let Some(c) = self.candidate(a, b) {
            self.heap.push(c);
        }
    }

    fn seed(&mut self, parallel: bool) {
        let mut edges: Vec<(u32, u32)> = Vec::with_capacity(self.tris.len() * 3 / 2);
        for t in &self.tris {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if self.seen_pair.insert((a.min(b), a.max(b))) {
                    edges.push((a.min(b), a.max(b)));
                }
            }
        }
        let cands: Vec<Option<Cand>> = if parallel && edges.len() >= SEED_PARALLEL_MIN {
            par::map_slice(&edges, |&(a, b)| self.candidate(a, b))
        } else {
            edges.iter().map(|&(a, b)| self.candidate(a, b)).collect()
        };
        // candidates are totally ordered (cost, a, b), so the heap does not depend on how it is built
        self.heap = BinaryHeap::from(cands.into_iter().flatten().collect::<Vec<_>>());
    }

    /// Collapses the cheapest edges until `target` triangles are left, the heap is empty, or the
    /// cheapest edge costs more than `max_cost`.
    pub(crate) fn run(&mut self, target: usize, max_cost: Option<f64>) {
        while self.live > target {
            let Some(c) = self.heap.pop() else { break };
            let (a, b) = (c.a, c.b);
            if !self.alive_v[a as usize] || !self.alive_v[b as usize] {
                continue;
            }
            if c.stamp != (self.stamp[a as usize], self.stamp[b as usize]) {
                continue;
            }
            if max_cost.is_some_and(|m| c.cost > m) {
                break;
            }
            if let Some(plan) = self.plan(&c) {
                self.apply(&c, plan);
            }
        }
    }

    fn alive_faces(&self, v: u32) -> Vec<u32> {
        self.vfaces[v as usize].iter().copied().filter(|&f| self.alive_t[f as usize]).collect()
    }

    /// The checks of a collapse of `c.b` into `c.a`: shared faces, link condition, valence under a
    /// guard, no flipped or degenerate face around the new vertex, and the measured guard.
    fn plan(&self, c: &Cand) -> Option<Plan> {
        let (a, b) = (c.a, c.b);
        let (fa, fb) = (self.alive_faces(a), self.alive_faces(b));
        let shared: Vec<u32> = fa.iter().copied().filter(|f| fb.contains(f)).collect();
        if shared.is_empty() || shared.len() > 2 {
            return None;
        }
        let nbr = |fs: &[u32], v: u32| -> HashSet<u32> {
            fs.iter().flat_map(|&f| self.tris[f as usize]).filter(|&x| x != v).collect()
        };
        let (na, nb) = (nbr(&fa, a), nbr(&fb, b));
        let common = na.intersection(&nb).count();
        if common != shared.len() {
            return None;
        }
        if self.guard.is_some() && na.len() + nb.len() - common - 2 > GUARD_VALENCE.max(na.len()).max(nb.len()) {
            return None;
        }
        let p = Point3::new(c.target[0], c.target[1], c.target[2]);
        let fan = self.fan_after(&fa, &fb, &shared, (a, b), &p)?;
        if let Some(g) = self.guard.as_ref() {
            let limit = g.budget[a as usize].min(g.budget[b as usize]);
            let old = [self.pos[a as usize], self.pos[b as usize]];
            if c.cost > (GUARD_SKIP * limit).powi(2) && !collapse_within(g.bvh, limit, &p, &old, &fan) {
                return None;
            }
        }
        Some(Plan { fb, shared, to: p })
    }

    /// The triangles around the merged vertex at `p`, or `None` when one is flipped or squashed.
    fn fan_after(
        &self,
        fa: &[u32],
        fb: &[u32],
        shared: &[u32],
        (a, b): (u32, u32),
        p: &Point3,
    ) -> Option<Vec<[Point3; 3]>> {
        let mut fan: Vec<[Point3; 3]> = Vec::new();
        for &f in fa.iter().chain(fb.iter()) {
            if shared.contains(&f) {
                continue;
            }
            let t = self.tris[f as usize];
            let old = tri::cross(&self.pos[t[0] as usize], &self.pos[t[1] as usize], &self.pos[t[2] as usize]);
            let corners: Vec<Point3> =
                t.iter().map(|&v| if v == a || v == b { *p } else { self.pos[v as usize] }).collect();
            let new = tri::cross(&corners[0], &corners[1], &corners[2]);
            fan.push([corners[0], corners[1], corners[2]]);
            if new.dot(&old) <= 0.0 || new.norm() < 1e-3 * old.norm() {
                return None;
            }
            if let (Some(n0), Some(n1)) = (crate::normalize(old), crate::normalize(new))
                && n0.dot(&n1) < 0.2
            {
                return None;
            }
            if self.turns_against_origin(&t, (a, b), &old, &new) {
                #[cfg(test)]
                self.turns_refused.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                return None;
            }
        }
        Some(fan)
    }

    /// Does the collapse of `b` into `a` turn the triangle `t` (normals `old` before, `new` after)
    /// from the side of the original surface its corners stand for to the other? A triangle that
    /// already disagrees with them (noise) is not held to it.
    fn turns_against_origin(&self, t: &[u32; 3], (a, b): (u32, u32), old: &Vec3, new: &Vec3) -> bool {
        #[cfg(test)]
        if !self.orient_guard {
            return false;
        }
        let at = |v: u32| self.origin[v as usize];
        let before: Vec3 = t.iter().map(|&v| at(v)).sum();
        let after: Vec3 = t.iter().map(|&v| if v == a || v == b { at(a) + at(b) } else { at(v) }).sum();
        old.dot(&before) > 0.0 && new.dot(&after) <= 0.0
    }

    /// Makes the collapse of `c.b` into `c.a` and queues the edges around the merged vertex again.
    fn apply(&mut self, c: &Cand, plan: Plan) {
        let (a, b) = (c.a, c.b);
        for &f in &plan.shared {
            self.alive_t[f as usize] = false;
            self.live -= 1;
        }
        for &f in &plan.fb {
            if !self.alive_t[f as usize] {
                continue;
            }
            for v in self.tris[f as usize].iter_mut() {
                if *v == b {
                    *v = a;
                }
            }
            self.vfaces[a as usize].push(f);
        }
        self.vfaces[b as usize].clear();
        let alive = &self.alive_t;
        self.vfaces[a as usize].retain(|&f| alive[f as usize]);
        self.alive_v[b as usize] = false;
        if let Some(g) = self.guard.as_mut() {
            g.budget[a as usize] = g.budget[a as usize].min(g.budget[b as usize]);
        }
        self.pos[a as usize] = plan.to;
        let qb = self.q[b as usize];
        self.q[a as usize] += qb;
        let ob = self.origin[b as usize];
        self.origin[a as usize] += ob;
        self.stamp[a as usize] = self.stamp[a as usize].wrapping_add(1);
        let mut nbs: Vec<u32> =
            self.vfaces[a as usize].iter().flat_map(|&f| self.tris[f as usize]).filter(|&x| x != a).collect();
        nbs.sort_unstable();
        nbs.dedup();
        for &o in &nbs {
            self.stamp[o as usize] = self.stamp[o as usize].wrapping_add(1);
        }
        // re-queue every edge around the neighbours (their stamps changed)
        self.seen_pair.clear();
        for &o in nbs.iter().chain(std::iter::once(&a)) {
            for k in 0..self.vfaces[o as usize].len() {
                let t = self.tris[self.vfaces[o as usize][k] as usize];
                for j in 0..3 {
                    let (x, y) = (t[j], t[(j + 1) % 3]);
                    let key = (x.min(y), x.max(y));
                    if self.seen_pair.insert(key) {
                        self.push(key.0, key.1);
                    }
                }
            }
        }
    }

    /// The surviving triangles, positions, quadrics and original orientations (vertex numbers
    /// unchanged).
    pub(crate) fn finish(self) -> (Vec<[u32; 3]>, Vec<Point3>, Vec<Quadric>, Vec<Vec3>) {
        let Self { mut tris, pos, q, origin, alive_t, .. } = self;
        let mut k = alive_t.iter();
        tris.retain(|_| k.next().copied().unwrap_or(false));
        (tris, pos, q, origin)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tetrahedron whose vertex orientations are `origin`.
    fn tetrahedron(origin: [Vec3; 4]) -> Collapser<'static> {
        let pos = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let tris = vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [0, 3, 2]];
        Collapser::new(tris, pos, vec![Quadric::zeros(); 4], origin.to_vec(), vec![false; 4], None, false)
    }

    fn up() -> Vec3 {
        Vec3::z()
    }

    #[test]
    fn a_triangle_turned_over_the_surface_it_stood_on_is_refused() {
        // Arrange: every corner of the triangle stands for upward triangles
        let c = tetrahedron([up(); 4]);

        // Act / Assert: facing up before; facing down, sideways (89 degrees) or up after
        assert!(c.turns_against_origin(&[0, 2, 1], (0, 3), &up(), &-up()));
        let sideways = Vec3::new(89f64.to_radians().sin(), 0.0, 89f64.to_radians().cos());
        assert!(!c.turns_against_origin(&[0, 2, 1], (0, 3), &up(), &sideways));
        assert!(!c.turns_against_origin(&[0, 2, 1], (0, 3), &up(), &up()));
    }

    #[test]
    fn a_triangle_that_already_faces_away_is_not_held_to_it() {
        let c = tetrahedron([up(); 4]);

        assert!(!c.turns_against_origin(&[0, 2, 1], (0, 3), &-up(), &-up()));
    }

    #[test]
    fn a_merge_that_brings_in_more_of_the_opposite_side_than_the_triangle_stands_for_is_refused() {
        // vertex 3 stands for twice as much downward surface as vertex 0 does upward surface
        let c = tetrahedron([up(), Vec3::zeros(), Vec3::zeros(), -2.0 * up()]);

        assert!(c.turns_against_origin(&[0, 2, 1], (0, 3), &up(), &up()));
    }

    #[test]
    fn vertices_that_stand_for_the_same_side_merge_freely() {
        let c = tetrahedron([up(); 4]);

        assert!(!c.turns_against_origin(&[0, 2, 1], (0, 1), &up(), &Vec3::new(0.3, 0.0, 1.0)));
    }

    #[test]
    fn the_orientation_of_a_vertex_is_the_sum_of_the_normals_of_its_triangles() {
        let pos = [Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0), Point3::new(0.0, 2.0, 0.0)];

        let origin = vertex_origins(&[[0, 1, 2]], &pos);

        // the cross product of the two edges: twice the area, along +z
        assert!(origin.iter().all(|o| (o - Vec3::new(0.0, 0.0, 4.0)).norm() < 1e-12), "{origin:?}");
    }
}
