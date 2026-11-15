//! Ball pivoting: the front of the growing surface, the balls that roll over the points and the triangles they form.

use super::grid::*;
use super::*;

// ------------------------------------------------------------------ ball pivoting

/// Hash for the directed-edge keys (a well mixed `u64`).
#[derive(Default)]
pub(super) struct Mix(u64);

impl Hasher for Mix {
    fn finish(&self) -> u64 {
        let mut x = self.0;
        x ^= x >> 32;
        x = x.wrapping_mul(0x9E37_79B9_7F4A_7C15);
        x ^ (x >> 29)
    }
    fn write(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 = (self.0.rotate_left(8) ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3);
        }
    }
    fn write_u64(&mut self, v: u64) {
        self.0 = v;
    }
}

type EdgeMap = HashMap<u64, u32, BuildHasherDefault<Mix>>;

pub(super) fn key(a: u32, b: u32) -> u64 {
    (u64::from(a) << 32) | u64::from(b)
}

/// An edge of the front: the triangle `(a, b, c)` was built, the ball (centre `center`) rests on
/// it and the edge `a → b` has no neighbour yet.
pub(super) struct FrontEdge {
    pub(super) a: u32,
    pub(super) b: u32,
    pub(super) c: u32,
    pub(super) center: Point3,
}

pub(super) struct Bpa<'a> {
    pub(super) pts: &'a [Point3],
    pub(super) nrm: &'a [Vec3],
    pub(super) used: Vec<bool>,
    /// Boundary edges (directed, without opposite) at each vertex.
    pub(super) open: Vec<u32>,
    /// Directed edge → triangle.
    pub(super) edges: EdgeMap,
    pub(super) tris: Vec<[u32; 3]>,
    pub(super) scratch: Vec<(f64, u32, Point3)>,
    /// A last test of a triangle `(b, a, d)` a pivot would build (band mode, [`super::band`]); every
    /// candidate is accepted without it. A candidate it refuses is passed over for the next one.
    pub(super) accept: Option<&'a mut dyn FnMut([u32; 3]) -> bool>,
}

/// Centre of the ball of radius `rho` resting on the triangle `(p0, p1, p2)` on the side of its
/// geometric normal, and that normal; `None` for degenerate triangles or a circumcircle larger
/// than the ball.
pub(super) fn ball_center(p0: &Point3, p1: &Point3, p2: &Point3, rho: f64) -> Option<(Point3, Vec3)> {
    let e1 = p1 - p0;
    let e2 = p2 - p0;
    let nrm = e1.cross(&e2);
    let den = nrm.norm_squared();
    let (a, b) = (e1.norm_squared(), e2.norm_squared());
    if !(den > 1e-18 * a * b) || !den.is_finite() {
        return None;
    }
    let cc = p0 + (e2 * a - e1 * b).cross(&nrm) / (2.0 * den);
    let r2 = (cc - p0).norm_squared();
    let h2 = rho * rho - r2;
    if h2 < 0.0 {
        return None;
    }
    let nhat = nrm / den.sqrt();
    Some((cc + nhat * h2.sqrt(), nhat))
}

/// In band mode a point counts for the empty-ball test only when its normal and the triangle's make
/// at least this cosine.
const BAND_FACING: f64 = 0.5;

/// Tolerance of the pivot angle below zero, 1e-7 rad (near zero the pseudo angle equals the angle).
const PSEUDO_EPS: f64 = 1e-7;

/// A monotone stand-in for `atan2(y, x) / (π/2)`, in `(-2, 2]` (a quarter turn is 1), from
/// `+ - * /` only: those are exactly rounded everywhere, while `atan2` differs in the last bit
/// between the C libraries of macOS, Linux and Windows, and the pivot picks among candidates
/// (often exactly tied ones, on regular clouds) by this value.
pub(super) fn pseudo_angle(y: f64, x: f64) -> f64 {
    let sum = x.abs() + y.abs();
    if !(sum > 0.0) {
        return 0.0;
    }
    let p = y / sum;
    match (x < 0.0, y < 0.0) {
        (false, _) => p,
        (true, false) => 2.0 - p,
        (true, true) => -2.0 - p,
    }
}

impl<'a> Bpa<'a> {
    pub(super) fn new(pts: &'a [Point3], nrm: &'a [Vec3]) -> Self {
        Self {
            pts,
            nrm,
            used: vec![false; pts.len()],
            open: vec![0; pts.len()],
            edges: EdgeMap::default(),
            tris: Vec::new(),
            scratch: Vec::new(),
            accept: None,
        }
    }

    fn p(&self, i: u32) -> Point3 {
        self.pts.get(i as usize).copied().unwrap_or_else(Point3::origin)
    }

    fn n(&self, i: u32) -> Vec3 {
        self.nrm.get(i as usize).copied().unwrap_or_else(Vec3::z)
    }

    /// Does the geometric normal `ng` of the triangle `(i, j, k)` agree with the vertex normals?
    fn agrees(&self, ng: &Vec3, i: u32, j: u32, k: u32) -> bool {
        let (a, b, c) = (self.n(i), self.n(j), self.n(k));
        ng.dot(&(a + b + c)) > 0.0 && ng.dot(&a) > -0.25 && ng.dot(&b) > -0.25 && ng.dot(&c) > -0.25
    }

    /// Is the ball (almost) free of points other than `skip`? The test ball is a little smaller
    /// than the real one so noise does not leave holes.
    fn ball_empty(&self, grid: &Grid, center: &Point3, rho: f64, skip: [u32; 3]) -> bool {
        self.ball_empty_facing(grid, center, rho, skip, None)
    }

    /// [`Bpa::ball_empty`] where, with `facing` (band mode), only points on the border of what is
    /// built (the band's own points) whose normal faces like it (cosine at least one half) count:
    /// the surface the band joins is all there, its inner vertices are no candidates and must not
    /// keep the ball out (the caller tests that a triangle covers none), and neither must a wall
    /// standing on it (a boss on a face).
    fn ball_empty_facing(&self, grid: &Grid, center: &Point3, rho: f64, skip: [u32; 3], facing: Option<Vec3>) -> bool {
        let r2 = (rho * 0.97) * (rho * 0.97);
        let mut empty = true;
        grid.visit(center, rho, |q| {
            let counts = facing
                .is_none_or(|f| self.n(q).dot(&f) >= BAND_FACING && self.open.get(q as usize).is_some_and(|&o| o > 0));
            if counts && !skip.contains(&q) && (self.p(q) - center).norm_squared() < r2 {
                empty = false;
                return true;
            }
            false
        });
        empty
    }

    pub(super) fn add_triangle(&mut self, t: [u32; 3]) {
        let id = self.tris.len() as u32;
        self.tris.push(t);
        for k in 0..3 {
            let (x, y) = (t[k], t[(k + 1) % 3]);
            self.edges.insert(key(x, y), id);
            let matched = self.edges.contains_key(&key(y, x));
            for v in [x, y] {
                if let Some(o) = self.open.get_mut(v as usize) {
                    *o = if matched { o.saturating_sub(1) } else { *o + 1 };
                }
            }
            if let Some(u) = self.used.get_mut(x as usize) {
                *u = true;
            }
        }
    }

    pub(super) fn push_open_edges(&self, t: [u32; 3], center: Point3, front: &mut VecDeque<FrontEdge>) {
        for k in 0..3 {
            let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            if !self.edges.contains_key(&key(b, a)) {
                front.push_back(FrontEdge { a, b, c, center });
            }
        }
    }

    pub(super) fn pass(&mut self, rho: f64) {
        let grid = Grid::build(self.pts, 2.0 * rho);
        let mut front: VecDeque<FrontEdge> = VecDeque::new();
        // boundary edges of what earlier passes built
        for t in 0..self.tris.len() {
            let Some(&tri) = self.tris.get(t) else { continue };
            for k in 0..3 {
                let (a, b, c) = (tri[k], tri[(k + 1) % 3], tri[(k + 2) % 3]);
                if self.edges.contains_key(&key(b, a)) {
                    continue;
                }
                if let Some((center, _)) = ball_center(&self.p(a), &self.p(b), &self.p(c), rho) {
                    front.push_back(FrontEdge { a, b, c, center });
                }
            }
        }
        self.drain(&grid, rho, &mut front);
        for p in 0..self.pts.len() as u32 {
            if self.used.get(p as usize).copied().unwrap_or(true) {
                continue;
            }
            if self.seed(&grid, p, rho, &mut front) {
                self.drain(&grid, rho, &mut front);
            }
        }
    }

    fn drain(&mut self, grid: &Grid, rho: f64, front: &mut VecDeque<FrontEdge>) {
        while let Some(f) = front.pop_front() {
            self.pivot(grid, rho, &f, front);
        }
    }

    /// A seed triangle of three unused points around `p`.
    fn seed(&mut self, grid: &Grid, p: u32, rho: f64, front: &mut VecDeque<FrontEdge>) -> bool {
        let pp = self.p(p);
        let mut near: Vec<(f64, u32)> = Vec::new();
        grid.visit(&pp, 2.0 * rho, |q| {
            if q != p && !self.used.get(q as usize).copied().unwrap_or(true) {
                let d2 = (self.p(q) - pp).norm_squared();
                if d2 <= 4.0 * rho * rho {
                    near.push((d2, q));
                }
            }
            false
        });
        near.sort_by(|a, b| a.0.total_cmp(&b.0));
        near.truncate(10);
        for i in 0..near.len() {
            for j in i + 1..near.len() {
                let (Some(&(_, q)), Some(&(_, r))) = (near.get(i), near.get(j)) else { continue };
                let ng = (self.p(q) - pp).cross(&(self.p(r) - pp));
                let s = self.n(p) + self.n(q) + self.n(r);
                let t = if ng.dot(&s) >= 0.0 { [p, q, r] } else { [p, r, q] };
                let Some((center, nhat)) = ball_center(&self.p(t[0]), &self.p(t[1]), &self.p(t[2]), rho) else {
                    continue;
                };
                if !self.agrees(&nhat, t[0], t[1], t[2]) || !self.ball_empty(grid, &center, rho, t) {
                    continue;
                }
                self.add_triangle(t);
                self.push_open_edges(t, center, front);
                return true;
            }
        }
        false
    }

    /// The points the ball pivoting around the front edge `f` can reach, with the pivot angle and the
    /// ball centre, in `out` (cleared first), sorted by angle.
    fn candidates(&self, grid: &Grid, rho: f64, f: &FrontEdge, out: &mut Vec<(f64, u32, Point3)>) {
        out.clear();
        let (a, b, c) = (f.a, f.b, f.c);
        let (pa, pb) = (self.p(a), self.p(b));
        let m = Point3::from((pa.coords + pb.coords) * 0.5);
        let Some(u) = crate::normalize(pb - pa) else { return };
        let v0 = f.center - m;
        let lim2 = 4.0 * rho * rho;
        grid.visit(&m, 2.0 * rho, |d| {
            if d == a || d == b || d == c {
                return false;
            }
            let pd = self.p(d);
            if (pd - pa).norm_squared() > lim2 || (pd - pb).norm_squared() > lim2 {
                return false;
            }
            // the new triangle is (b, a, d): it shares the edge in the opposite direction
            let Some((cen, ng)) = ball_center(&pb, &pa, &pd, rho) else { return false };
            if !self.agrees(&ng, b, a, d) {
                return false;
            }
            let v1 = cen - m;
            let mut ang = pseudo_angle(u.dot(&v0.cross(&v1)), v0.dot(&v1));
            if ang < -PSEUDO_EPS {
                ang += 4.0;
            }
            if ang <= 2.0 {
                out.push((ang.max(0.0), d, cen));
            }
            false
        });
        out.sort_by(|x, y| x.0.total_cmp(&y.0));
    }

    /// May the pivot over `a → b` end at `d` with the ball at `cen`? Not at an inner vertex (nor, in
    /// band mode, at a point no triangle uses), not on an edge that exists, with the ball empty and
    /// the caller's test passed.
    fn takes(&mut self, grid: &Grid, rho: f64, (a, b, d): (u32, u32, u32), cen: &Point3) -> bool {
        let used = self.used.get(d as usize).copied().unwrap_or(true);
        if used && self.open.get(d as usize).copied().unwrap_or(0) == 0 {
            return false; // an inner vertex
        }
        if self.accept.is_some() && !used {
            return false; // band mode: only points of the surface that is there
        }
        if self.edges.contains_key(&key(a, d)) || self.edges.contains_key(&key(d, b)) {
            return false;
        }
        let facing = if self.accept.is_some() { crate::tri::normal(&self.p(b), &self.p(a), &self.p(d)) } else { None };
        if !self.ball_empty_facing(grid, cen, rho, [a, b, d], facing) {
            return false;
        }
        self.accept.as_mut().is_none_or(|accept| accept([b, a, d]))
    }

    /// Pivots the ball around the front edge `a → b` to the next point it touches.
    pub(super) fn pivot(&mut self, grid: &Grid, rho: f64, f: &FrontEdge, front: &mut VecDeque<FrontEdge>) {
        let (a, b) = (f.a, f.b);
        if self.edges.contains_key(&key(b, a)) {
            return;
        }
        let mut cands = std::mem::take(&mut self.scratch);
        self.candidates(grid, rho, f, &mut cands);
        let found =
            cands.iter().find(|&&(_, d, cen)| self.takes(grid, rho, (a, b, d), &cen)).map(|&(_, d, cen)| (d, cen));
        self.scratch = cands;
        if let Some((d, cen)) = found {
            let t = [b, a, d];
            self.add_triangle(t);
            self.push_open_edges(t, cen, front);
        }
    }
}
