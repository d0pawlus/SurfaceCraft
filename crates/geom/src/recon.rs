//! Surface reconstruction from a point cloud (P1: scanners that export points, not faces).
//!
//! Pipeline, all mesh-side computation (design doc §5):
//!
//! 1. exact duplicates are dropped; a k-d tree gives the k nearest neighbours of every point;
//! 2. normals: PCA of the neighbourhood (smallest eigenvector of the covariance), oriented
//!    consistently by propagation along a minimum spanning tree of the neighbour graph with edge
//!    weight `1 − |nᵢ·nⱼ|` (Hoppe et al. 1992), each connected piece facing away from its
//!    extreme point; normals given by the file are trusted as they are;
//! 3. ball pivoting (Bernardini et al. 1999) with radii of a few mean nearest-neighbour
//!    spacings: a ball of radius ρ rolls over the points, every three points it touches without
//!    holding another point form a triangle; passes with growing ρ close the gaps the smaller
//!    balls left. The triangle orientation follows the vertex normals, so the result is
//!    consistently oriented and manifold along edges by construction;
//! 4. leftover specks are dropped and small holes are filled ([`crate::edit`]).
//!
//! Neighbour search and normal estimation use all cores outside the browser; ball pivoting is
//! sequential (its front depends on what was built before).
use crate::{Aabb, KdTree, Mesh, Point3, Vec3, cloud::CloudError, cloud::PointCloud, edit};
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap, VecDeque};
use std::f64::consts::{PI, TAU};
use std::hash::{BuildHasherDefault, Hasher};
use std::ops::Range;

/// Reconstruction settings.
#[derive(Clone, Debug, PartialEq)]
pub struct ReconParams {
    /// Neighbours used for normals and orientation (3 – 64).
    pub normals_k: usize,
    /// First ball radius as a multiple of the mean nearest-neighbour spacing (0.5 – 50).
    pub radius_scale: f64,
    /// Number of passes; each doubles the radius (1 – 6).
    pub passes: usize,
    /// Boundary loops of at most this many edges are filled afterwards (0 = none).
    pub fill_holes_max_edges: usize,
    /// Refuse clouds with more points than this.
    pub max_points: usize,
}

impl Default for ReconParams {
    fn default() -> Self {
        Self { normals_k: 12, radius_scale: 2.5, passes: 3, fill_holes_max_edges: 24, max_points: 20_000_000 }
    }
}

/// Why a cloud could not be reconstructed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum ReconError {
    /// A setting is out of range.
    #[error("{0}")]
    Params(String),
    /// The cloud itself is invalid.
    #[error(transparent)]
    Cloud(#[from] CloudError),
    /// Too few distinct points.
    #[error("{0} distinct points are too few to reconstruct a surface")]
    TooFew(usize),
    /// Over the configured point limit.
    #[error("{points} points exceed the limit of {limit}")]
    TooMany {
        /// Points in the cloud.
        points: usize,
        /// Configured limit.
        limit: usize,
    },
    /// Coordinates beyond +-1e9 mm (a thousand kilometres) are not a scan.
    #[error("coordinates beyond 1e9 mm cannot be reconstructed")]
    TooLarge,
    /// No triangle could be built (points too sparse, collinear or without a surface).
    #[error("no surface found: no ball of the chosen radii rests on three points; try a larger radius scale")]
    NoSurface,
}

/// What reconstruction did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ReconReport {
    /// Points in the input.
    pub points: usize,
    /// Exact duplicates dropped.
    pub duplicates_removed: usize,
    /// Mean nearest-neighbour spacing (mm).
    pub spacing: f64,
    /// Ball radii used, in order (mm).
    pub radii: Vec<f64>,
    /// The file's normals were used (otherwise estimated and oriented here).
    pub normals_given: bool,
    /// Triangles after ball pivoting.
    pub pivoted_triangles: usize,
    /// Points left out of the mesh (no ball rested on them).
    pub unused_points: usize,
    /// Isolated specks removed (triangles).
    pub specks_removed: usize,
    /// Holes filled.
    pub holes_filled: usize,
    /// Boundary loops left in the result.
    pub holes_left: usize,
}

/// Reconstructs a triangle mesh from `cloud`.
pub fn reconstruct(cloud: &PointCloud, p: &ReconParams) -> Result<(Mesh, ReconReport), ReconError> {
    if !(3..=64).contains(&p.normals_k) {
        return Err(ReconError::Params("normals_k must be between 3 and 64".into()));
    }
    if !(p.radius_scale.is_finite() && (0.5..=50.0).contains(&p.radius_scale)) {
        return Err(ReconError::Params("radius_scale must be between 0.5 and 50".into()));
    }
    if !(1..=6).contains(&p.passes) {
        return Err(ReconError::Params("passes must be between 1 and 6".into()));
    }
    cloud.validate()?;
    if cloud.len() > p.max_points {
        return Err(ReconError::TooMany { points: cloud.len(), limit: p.max_points });
    }
    if cloud.positions.iter().any(|q| q.x.abs() > 1e9 || q.y.abs() > 1e9 || q.z.abs() > 1e9) {
        return Err(ReconError::TooLarge);
    }
    let (pts, given) = dedupe(cloud);
    let mut rep =
        ReconReport { points: cloud.len(), duplicates_removed: cloud.len() - pts.len(), ..Default::default() };
    if pts.len() < 8 {
        return Err(ReconError::TooFew(pts.len()));
    }
    let k = p.normals_k.min(pts.len() - 1);
    let tree = KdTree::build(&pts);
    let nbr = knn_graph(&tree, &pts, k);
    drop(tree);

    let spacing = mean_spacing(&pts, &nbr, k).ok_or(ReconError::NoSurface)?;
    rep.spacing = spacing;
    let normals = match given {
        Some(n) => {
            rep.normals_given = true;
            n
        }
        None => {
            let mut n = estimate_normals(&pts, &nbr, k);
            orient_normals(&pts, &mut n, &nbr, k);
            n
        }
    };
    drop(nbr);

    let triangles = {
        let mut bpa = Bpa::new(&pts, &normals);
        for j in 0..p.passes {
            let rho = spacing * p.radius_scale * f64::from(1u32 << j);
            rep.radii.push(rho);
            bpa.pass(rho);
        }
        bpa.tris
    };
    rep.pivoted_triangles = triangles.len();
    if triangles.is_empty() {
        return Err(ReconError::NoSurface);
    }
    let mut mesh = Mesh { positions: pts, triangles };
    edit::compact(&mut mesh);
    rep.unused_points = rep.points - rep.duplicates_removed - mesh.positions.len();
    rep.specks_removed = edit::remove_small_components(&mut mesh, 0.0, 8);
    edit::compact(&mut mesh);
    if p.fill_holes_max_edges >= 3 {
        rep.holes_filled = edit::fill_holes(&mut mesh, p.fill_holes_max_edges, false).filled;
    }
    rep.holes_left = edit::boundary_loops(&mesh).len();
    Ok((mesh, rep))
}

/// Drops exact duplicate points (keeping the first); normals follow when every one of them is
/// non-zero (otherwise they are ignored and re-estimated).
fn dedupe(cloud: &PointCloud) -> (Vec<Point3>, Option<Vec<Vec3>>) {
    let mut order: Vec<u32> = (0..cloud.len() as u32).collect();
    order.sort_unstable_by(|&a, &b| {
        let (pa, pb) = (cloud.positions.get(a as usize), cloud.positions.get(b as usize));
        match (pa, pb) {
            (Some(pa), Some(pb)) => {
                pa.x.total_cmp(&pb.x).then(pa.y.total_cmp(&pb.y)).then(pa.z.total_cmp(&pb.z)).then(a.cmp(&b))
            }
            _ => a.cmp(&b),
        }
    });
    let mut keep = vec![true; cloud.len()];
    let mut prev: Option<&Point3> = None;
    for &i in &order {
        let Some(p) = cloud.positions.get(i as usize) else { continue };
        if prev.is_some_and(|q| q == p) {
            if let Some(f) = keep.get_mut(i as usize) {
                *f = false;
            }
        } else {
            prev = Some(p);
        }
    }
    let pts: Vec<Point3> = cloud.positions.iter().zip(&keep).filter_map(|(p, &k)| k.then_some(*p)).collect();
    let normals = cloud.normals.as_ref().and_then(|ns| {
        let unit: Vec<Vec3> = ns.iter().zip(&keep).filter_map(|(n, &k)| k.then_some(*n)).collect();
        let out: Vec<Vec3> = unit.iter().filter_map(|n| crate::normalize(*n)).collect();
        (out.len() == unit.len()).then_some(out)
    });
    (pts, normals)
}

// ------------------------------------------------------------------ parallel helper

/// Runs `f` over `0..n` in chunks on all cores (sequentially in the browser) and concatenates the
/// results in order.
fn par_collect<T: Send, F: Fn(Range<usize>) -> Vec<T> + Sync>(n: usize, f: F) -> Vec<T> {
    #[cfg(not(target_arch = "wasm32"))]
    {
        let threads = std::thread::available_parallelism().map_or(1, std::num::NonZero::get).min(16);
        if threads > 1 && n >= 20_000 {
            let chunk = n.div_ceil(threads);
            let f = &f;
            let parts: Vec<Option<Vec<T>>> = std::thread::scope(|s| {
                let handles: Vec<_> = (0..threads)
                    .map(|t| {
                        let lo = (t * chunk).min(n);
                        let hi = ((t + 1) * chunk).min(n);
                        s.spawn(move || f(lo..hi))
                    })
                    .collect();
                handles.into_iter().map(|h| h.join().ok()).collect()
            });
            if parts.iter().all(Option::is_some) {
                return parts.into_iter().flatten().flatten().collect();
            }
        }
    }
    f(0..n)
}

// ------------------------------------------------------------------ neighbours and normals

/// `k` nearest neighbours of every point, nearest first, flat (`k` per point).
fn knn_graph(tree: &KdTree, pts: &[Point3], k: usize) -> Vec<u32> {
    par_collect(pts.len(), |r| {
        let mut out = Vec::with_capacity(r.len() * k);
        for i in r {
            let Some(p) = pts.get(i) else { continue };
            let before = out.len();
            for (j, _) in tree.knn(p, k + 1) {
                if j != i && out.len() - before < k {
                    out.push(j as u32);
                }
            }
            let pad = if out.len() > before { out.last().copied().unwrap_or(i as u32) } else { i as u32 };
            while out.len() - before < k {
                out.push(pad);
            }
        }
        out
    })
}

/// Mean distance to the nearest neighbour, ignoring the few isolated outliers.
fn mean_spacing(pts: &[Point3], nbr: &[u32], k: usize) -> Option<f64> {
    let mut d: Vec<f64> = (0..pts.len())
        .filter_map(|i| {
            let j = *nbr.get(i * k)? as usize;
            let l = (pts.get(i)? - pts.get(j)?).norm();
            (l.is_finite() && l > 0.0).then_some(l)
        })
        .collect();
    if d.is_empty() {
        return None;
    }
    let mid = d.len() / 2;
    let median = *d.select_nth_unstable_by(mid, f64::total_cmp).1;
    let kept: Vec<f64> = d.into_iter().filter(|&l| l <= 3.0 * median).collect();
    let mean = kept.iter().sum::<f64>() / kept.len().max(1) as f64;
    (mean.is_finite() && mean > 0.0).then_some(mean)
}

/// Unoriented PCA normals.
fn estimate_normals(pts: &[Point3], nbr: &[u32], k: usize) -> Vec<Vec3> {
    par_collect(pts.len(), |r| {
        let mut out = Vec::with_capacity(r.len());
        for i in r {
            let Some(&p0) = pts.get(i) else { continue };
            let ids = nbr.get(i * k..(i + 1) * k).unwrap_or(&[]);
            let mut mean = Vec3::zeros();
            let mut count = 1.0;
            for &j in ids {
                if let Some(q) = pts.get(j as usize) {
                    mean += q - p0;
                    count += 1.0;
                }
            }
            mean /= count;
            let mut cov = nalgebra::Matrix3::<f64>::zeros();
            let d0 = -mean;
            cov += d0 * d0.transpose();
            for &j in ids {
                if let Some(q) = pts.get(j as usize) {
                    let d = (q - p0) - mean;
                    cov += d * d.transpose();
                }
            }
            let eig = nalgebra::SymmetricEigen::new(cov);
            let mut best = 0;
            for a in 1..3 {
                if eig.eigenvalues[a] < eig.eigenvalues[best] {
                    best = a;
                }
            }
            let n: Vec3 = eig.eigenvectors.column(best).into_owned();
            out.push(crate::normalize(n).unwrap_or_else(Vec3::z));
        }
        out
    })
}

/// Flips normals so neighbours agree (MST propagation); each connected piece of the neighbour
/// graph is oriented to face away from its extreme point in +x.
fn orient_normals(pts: &[Point3], normals: &mut [Vec3], nbr: &[u32], k: usize) {
    let n = pts.len();
    let gk = k.min(10);
    // undirected adjacency in CSR form
    let mut deg = vec![0u32; n + 1];
    for i in 0..n {
        for t in 0..gk {
            if let Some(&j) = nbr.get(i * k + t) {
                if let Some(d) = deg.get_mut(i) {
                    *d += 1;
                }
                if let Some(d) = deg.get_mut(j as usize) {
                    *d += 1;
                }
            }
        }
    }
    let mut start = vec![0usize; n + 1];
    for i in 0..n {
        start[i + 1] = start[i] + deg.get(i).copied().unwrap_or(0) as usize;
    }
    let mut fill = start.clone();
    let mut adj = vec![0u32; start[n]];
    for i in 0..n {
        for t in 0..gk {
            if let Some(&j) = nbr.get(i * k + t) {
                if let Some(f) = fill.get_mut(i) {
                    if let Some(a) = adj.get_mut(*f) {
                        *a = j;
                    }
                    *f += 1;
                }
                if let Some(f) = fill.get_mut(j as usize) {
                    if let Some(a) = adj.get_mut(*f) {
                        *a = i as u32;
                    }
                    *f += 1;
                }
            }
        }
    }
    let mut visited = vec![false; n];
    let mut heap: BinaryHeap<Reverse<(u32, u32, u32)>> = BinaryHeap::new();
    let mut piece: Vec<u32> = Vec::new();
    for s in 0..n {
        if visited.get(s).copied().unwrap_or(true) {
            continue;
        }
        piece.clear();
        heap.clear();
        heap.push(Reverse((0, s as u32, s as u32)));
        while let Some(Reverse((_, to, from))) = heap.pop() {
            let (to_i, from_i) = (to as usize, from as usize);
            if visited.get(to_i).copied().unwrap_or(true) {
                continue;
            }
            let nf = normals.get(from_i).copied();
            if let (Some(nf), Some(nt)) = (nf, normals.get_mut(to_i))
                && nf.dot(nt) < 0.0
            {
                *nt = -*nt;
            }
            if let Some(v) = visited.get_mut(to_i) {
                *v = true;
            }
            piece.push(to);
            let nt = normals.get(to_i).copied().unwrap_or_else(Vec3::z);
            for &j in adj.get(start[to_i]..start[to_i + 1]).unwrap_or(&[]) {
                if visited.get(j as usize).copied().unwrap_or(true) {
                    continue;
                }
                let nj = normals.get(j as usize).copied().unwrap_or_else(Vec3::z);
                let w = ((1.0 - nt.dot(&nj).abs()).clamp(0.0, 1.0) * 1_048_576.0) as u32;
                heap.push(Reverse((w, j, to)));
            }
        }
        // the extreme point in +x must face +x
        let extreme = piece
            .iter()
            .copied()
            .max_by(|&a, &b| {
                let xa = pts.get(a as usize).map_or(0.0, |p| p.x);
                let xb = pts.get(b as usize).map_or(0.0, |p| p.x);
                xa.total_cmp(&xb)
            })
            .unwrap_or(s as u32);
        if normals.get(extreme as usize).is_some_and(|v| v.x < 0.0) {
            for &i in &piece {
                if let Some(v) = normals.get_mut(i as usize) {
                    *v = -*v;
                }
            }
        }
    }
}

// ------------------------------------------------------------------ uniform grid

/// Static uniform grid over the points (counting-sorted, dense).
struct Grid {
    min: Point3,
    cell: f64,
    dims: [usize; 3],
    start: Vec<u32>,
    items: Vec<u32>,
}

impl Grid {
    fn build(pts: &[Point3], cell: f64) -> Self {
        let bb = Aabb::from_points(pts);
        let (min, ext) = if bb.is_empty() { (Point3::origin(), Vec3::zeros()) } else { (bb.min, bb.max - bb.min) };
        let mut cell = if cell.is_finite() && cell > 0.0 { cell } else { 1.0 };
        let cap = (pts.len() * 2).max(1 << 20) as f64;
        let dims = loop {
            let d = [0, 1, 2].map(|a| ((ext[a] / cell).floor() as usize).saturating_add(1));
            if (d[0] as f64) * (d[1] as f64) * (d[2] as f64) <= cap {
                break d;
            }
            cell *= 1.5;
        };
        let total = dims[0] * dims[1] * dims[2];
        let mut g = Grid { min, cell, dims, start: vec![0; total + 1], items: vec![0; pts.len()] };
        let ids: Vec<usize> = pts.iter().map(|p| g.cell_index(p)).collect();
        for &c in &ids {
            if let Some(s) = g.start.get_mut(c + 1) {
                *s += 1;
            }
        }
        for c in 0..total {
            g.start[c + 1] += g.start[c];
        }
        let mut fill: Vec<u32> = g.start.clone();
        for (i, &c) in ids.iter().enumerate() {
            if let Some(f) = fill.get_mut(c) {
                if let Some(slot) = g.items.get_mut(*f as usize) {
                    *slot = i as u32;
                }
                *f += 1;
            }
        }
        g
    }

    fn axis_cell(&self, v: f64, a: usize) -> usize {
        let last = self.dims[a] - 1;
        let c = ((v - self.min[a]) / self.cell).floor();
        if c <= 0.0 { 0 } else { (c as usize).min(last) }
    }

    fn cell_index(&self, p: &Point3) -> usize {
        let (x, y, z) = (self.axis_cell(p.x, 0), self.axis_cell(p.y, 1), self.axis_cell(p.z, 2));
        (z * self.dims[1] + y) * self.dims[0] + x
    }

    /// Calls `f` on every point in the cells overlapping the ball `(c, r)` (a superset of the
    /// points inside it); `f` returns true to stop.
    fn visit(&self, c: &Point3, r: f64, mut f: impl FnMut(u32) -> bool) {
        let lo = [0, 1, 2].map(|a| self.axis_cell(c[a] - r, a));
        let hi = [0, 1, 2].map(|a| self.axis_cell(c[a] + r, a));
        for z in lo[2]..=hi[2] {
            for y in lo[1]..=hi[1] {
                let row = (z * self.dims[1] + y) * self.dims[0];
                let (Some(&s), Some(&e)) = (self.start.get(row + lo[0]), self.start.get(row + hi[0] + 1)) else {
                    continue;
                };
                for &i in self.items.get(s as usize..e as usize).unwrap_or(&[]) {
                    if f(i) {
                        return;
                    }
                }
            }
        }
    }
}

// ------------------------------------------------------------------ ball pivoting

/// Hash for the directed-edge keys (a well mixed `u64`).
#[derive(Default)]
struct Mix(u64);

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

fn key(a: u32, b: u32) -> u64 {
    (u64::from(a) << 32) | u64::from(b)
}

/// An edge of the front: the triangle `(a, b, c)` was built, the ball (centre `center`) rests on
/// it and the edge `a → b` has no neighbour yet.
struct FrontEdge {
    a: u32,
    b: u32,
    c: u32,
    center: Point3,
}

struct Bpa<'a> {
    pts: &'a [Point3],
    nrm: &'a [Vec3],
    used: Vec<bool>,
    /// Boundary edges (directed, without opposite) at each vertex.
    open: Vec<u32>,
    /// Directed edge → triangle.
    edges: EdgeMap,
    tris: Vec<[u32; 3]>,
    scratch: Vec<(f64, u32, Point3)>,
}

/// Centre of the ball of radius `rho` resting on the triangle `(p0, p1, p2)` on the side of its
/// geometric normal, and that normal; `None` for degenerate triangles or a circumcircle larger
/// than the ball.
fn ball_center(p0: &Point3, p1: &Point3, p2: &Point3, rho: f64) -> Option<(Point3, Vec3)> {
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

impl<'a> Bpa<'a> {
    fn new(pts: &'a [Point3], nrm: &'a [Vec3]) -> Self {
        Self {
            pts,
            nrm,
            used: vec![false; pts.len()],
            open: vec![0; pts.len()],
            edges: EdgeMap::default(),
            tris: Vec::new(),
            scratch: Vec::new(),
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
        let r2 = (rho * 0.97) * (rho * 0.97);
        let mut empty = true;
        grid.visit(center, rho, |q| {
            if !skip.contains(&q) && (self.p(q) - center).norm_squared() < r2 {
                empty = false;
                return true;
            }
            false
        });
        empty
    }

    fn add_triangle(&mut self, t: [u32; 3]) {
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

    fn push_open_edges(&self, t: [u32; 3], center: Point3, front: &mut VecDeque<FrontEdge>) {
        for k in 0..3 {
            let (a, b, c) = (t[k], t[(k + 1) % 3], t[(k + 2) % 3]);
            if !self.edges.contains_key(&key(b, a)) {
                front.push_back(FrontEdge { a, b, c, center });
            }
        }
    }

    fn pass(&mut self, rho: f64) {
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

    /// Pivots the ball around the front edge `a → b` to the next point it touches.
    fn pivot(&mut self, grid: &Grid, rho: f64, f: &FrontEdge, front: &mut VecDeque<FrontEdge>) {
        let (a, b, c) = (f.a, f.b, f.c);
        if self.edges.contains_key(&key(b, a)) {
            return;
        }
        let (pa, pb) = (self.p(a), self.p(b));
        let m = Point3::from((pa.coords + pb.coords) * 0.5);
        let Some(u) = crate::normalize(pb - pa) else { return };
        let v0 = f.center - m;
        let lim2 = 4.0 * rho * rho;
        let mut cands = std::mem::take(&mut self.scratch);
        cands.clear();
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
            let mut ang = u.dot(&v0.cross(&v1)).atan2(v0.dot(&v1));
            if ang < -1e-7 {
                ang += TAU;
            }
            if ang <= PI {
                cands.push((ang.max(0.0), d, cen));
            }
            false
        });
        cands.sort_by(|x, y| x.0.total_cmp(&y.0));
        let mut found: Option<(u32, Point3)> = None;
        for &(_, d, cen) in &cands {
            if self.used.get(d as usize).copied().unwrap_or(true)
                && self.open.get(d as usize).copied().unwrap_or(0) == 0
            {
                continue; // an interior vertex
            }
            if self.edges.contains_key(&key(a, d)) || self.edges.contains_key(&key(d, b)) {
                continue;
            }
            if !self.ball_empty(grid, &cen, rho, [a, b, d]) {
                continue;
            }
            found = Some((d, cen));
            break;
        }
        self.scratch = cands;
        if let Some((d, cen)) = found {
            let t = [b, a, d];
            self.add_triangle(t);
            self.push_open_edges(t, cen, front);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rng, synth};

    fn cloud_of(mesh: &Mesh, n: usize, sigma: f64, rng: &mut Rng) -> PointCloud {
        let pts = mesh.sample_points(n, rng).into_iter().map(|(p, nrm)| p + nrm * (sigma * rng.gauss())).collect();
        PointCloud::from_points(pts)
    }

    #[test]
    fn sphere_cloud_becomes_a_closed_mesh() {
        let mut rng = Rng::new(7);
        let sphere = synth::sphere(Point3::new(1.0, 2.0, 3.0), 10.0, 64, 32);
        let c = cloud_of(&sphere, 6000, 0.02, &mut rng);
        let (m, rep) = reconstruct(&c, &ReconParams::default()).unwrap();
        assert!(m.validate().is_ok());
        assert!(rep.holes_left <= 2, "{rep:?}");
        for p in &m.positions {
            let r = (p - Point3::new(1.0, 2.0, 3.0)).norm();
            assert!((r - 10.0).abs() < 0.1, "{r}");
        }
        // area of a sphere of radius 10 is 1257 mm²
        assert!((m.area() - 1256.6).abs() < 80.0, "{}", m.area());
        // outward orientation: the signed volume is positive
        let mut vol = 0.0;
        for i in 0..m.triangle_count() {
            if let Some([a, b, c]) = m.triangle(i) {
                vol += a.coords.dot(&b.coords.cross(&c.coords)) / 6.0;
            }
        }
        assert!(vol > 0.0, "{vol}");
    }

    /// Degenerate and extreme clouds are refused or reconstructed, never a panic.
    #[test]
    fn hostile_clouds_never_panic() {
        let mut rng = Rng::new(11);
        let sphere = synth::sphere(Point3::origin(), 10.0, 32, 16);
        let base = cloud_of(&sphere, 800, 0.0, &mut rng);
        let scaled =
            |f: f64| PointCloud::from_points(base.positions.iter().map(|p| Point3::from(p.coords * f)).collect());
        let mut clouds = vec![
            PointCloud::from_points(vec![Point3::new(1.0, 2.0, 3.0); 100]),
            PointCloud::from_points((0..200).map(|i| Point3::new(f64::from(i) * 0.1, 0.0, 0.0)).collect()),
            // a perfectly regular grid: every four neighbours are cospherical
            PointCloud::from_points((0..400).map(|i| Point3::new(f64::from(i % 20), f64::from(i / 20), 0.0)).collect()),
            scaled(1e-9),
            scaled(1e8),
            scaled(1e12),
            PointCloud::from_points((0..500).map(|_| Point3::from(rng.unit_vector() * rng.range(0.0, 50.0))).collect()),
            PointCloud::from_points(
                (0..300)
                    .map(|i| Point3::new(if i % 2 == 0 { 0.0 } else { 1e6 }, 0.0, 0.0) + rng.unit_vector())
                    .collect(),
            ),
        ];
        // normals: all zero, random, and given on a sphere
        let mut c = base.clone();
        c.normals = Some(vec![Vec3::zeros(); base.len()]);
        clouds.push(c);
        let mut c = base.clone();
        c.normals = Some((0..base.len()).map(|_| rng.unit_vector() * 1e-30).collect());
        clouds.push(c);
        for c in &clouds {
            for params in [
                ReconParams::default(),
                ReconParams { radius_scale: 0.5, passes: 1, fill_holes_max_edges: 0, ..ReconParams::default() },
                ReconParams { radius_scale: 50.0, passes: 6, normals_k: 3, ..ReconParams::default() },
                ReconParams { normals_k: 64, ..ReconParams::default() },
            ] {
                if let Ok((m, _)) = reconstruct(c, &params) {
                    assert!(m.validate().is_ok());
                }
            }
        }
    }

    #[test]
    fn given_normals_are_used_and_degenerate_input_is_refused() {
        let mut rng = Rng::new(3);
        let plane = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 20.0, 20.0, 4, 4);
        let s = plane.sample_points(1500, &mut rng);
        let c =
            PointCloud::new(s.iter().map(|(p, _)| *p).collect(), Some(s.iter().map(|(_, n)| *n).collect())).unwrap();
        let (m, rep) = reconstruct(&c, &ReconParams::default()).unwrap();
        assert!(rep.normals_given);
        assert!((m.area() - 400.0).abs() < 40.0, "{}", m.area());
        assert!(matches!(
            reconstruct(&PointCloud::from_points(vec![Point3::origin(); 50]), &ReconParams::default()),
            Err(ReconError::TooFew(1))
        ));
        let line: Vec<Point3> = (0..100).map(|i| Point3::new(f64::from(i), 0.0, 0.0)).collect();
        assert!(reconstruct(&PointCloud::from_points(line), &ReconParams::default()).is_err());
        let bad = ReconParams { radius_scale: f64::NAN, ..ReconParams::default() };
        assert!(reconstruct(&c, &bad).is_err());
    }
}
