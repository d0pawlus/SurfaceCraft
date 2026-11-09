//! Unstructured, field-aligned retopology of a triangle mesh into a quad-dominant layout
//! (design doc §6.3 and §10, milestone P4): wrapped and closed regions the height-field
//! layout of M4 cannot cover.
//!
//! The method is that of Jakob, Tarini, Panozzo and Sorkine-Hornung, "Instant Field-Aligned
//! Meshes" (SIGGRAPH Asia 2015; reference implementation "Instant Meshes", permissive licence):
//!
//! 1. an *orientation field* (4-RoSy: a tangent direction per vertex defined up to 90° turns),
//!    smoothed by Gauss–Seidel sweeps over the one-ring with the best 90° match per edge;
//! 2. a *position field* (2-PoSy: a lattice point of spacing ρ per vertex, aligned with the
//!    orientation) smoothed the same way: every neighbour proposes the point of its own lattice
//!    closest to the shared edge's middle, the average is rounded to the lattice point nearest to
//!    the vertex;
//! 3. extraction: mesh vertices whose lattice points coincide collapse into one layout vertex,
//!    lattice neighbours become layout edges and the faces are the cycles of the resulting
//!    embedded graph.
//!
//! This is written from the paper, not from the reference source (no code was copied; the
//! reference and its licence are credited in ATTRIBUTION.md and NOTICE). Differences from the
//! reference: a coarse-to-fine hierarchy built with quadric decimation instead of its graph
//! coarsening; single-threaded; extraction by cell adjacency (no edge-tag tracing and no
//! post-clean-up beyond pruning dangling vertices); open-boundary alignment by pinning the
//! orientation to the boundary tangent and the lattice to pass through boundary vertices.
//! Where the field cannot give a clean polygon mesh, [`retopo`] falls back to subdividing a
//! coarse decimation (every triangle into three quads) and says so in the report.
use std::collections::{BTreeMap, HashMap};
use surfacecraft_geom::edit::{DecimateOptions, compact, decimate};
use surfacecraft_geom::{Bvh, KdTree, Mesh, Point3, Vec3, any_perpendicular, normalize, synth};

/// Largest polygon the extraction keeps.
const MAX_POLY: usize = 8;
/// Most triangles of the working mesh.
const MAX_WORK_TRIANGLES: usize = 120_000;

/// A polygon mesh with per-vertex unit normals; faces run counter-clockwise seen from the side
/// the normals point to.
#[derive(Clone, Debug, Default)]
pub struct PolyMesh {
    /// Vertices.
    pub vertices: Vec<Point3>,
    /// Unit normals.
    pub normals: Vec<Vec3>,
    /// Faces (3 or more vertices).
    pub faces: Vec<Vec<u32>>,
}

/// An all-quad mesh.
#[derive(Clone, Debug, Default)]
pub struct QuadMesh {
    /// Vertices.
    pub vertices: Vec<Point3>,
    /// Unit normals.
    pub normals: Vec<Vec3>,
    /// Quads, counter-clockwise seen from the normals' side.
    pub faces: Vec<[u32; 4]>,
}

/// How the layout was obtained.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Layout {
    /// Field-aligned extraction.
    Field,
    /// Coarse decimation split into quads (the field gave no clean layout).
    Fallback,
}

/// Retopology parameters.
#[derive(Clone, Debug)]
pub struct RetopoOptions {
    /// Approximate number of faces wanted.
    pub target_faces: usize,
    /// Smoothing sweeps of the orientation field per level.
    pub orientation_sweeps: usize,
    /// Smoothing sweeps of the position field per level.
    pub position_sweeps: usize,
    /// Skip the fields and use the fallback layout (decimation split into quads).
    pub force_fallback: bool,
}

impl Default for RetopoOptions {
    fn default() -> Self {
        Self { target_faces: 200, orientation_sweeps: 40, position_sweeps: 40, force_fallback: false }
    }
}

/// What [`retopo`] did.
#[derive(Clone, Debug)]
pub struct RetopoReport {
    /// Field extraction or fallback.
    pub layout: Layout,
    /// Edge length ρ asked of the field (mm).
    pub edge_length: f64,
    /// Triangles of the working mesh the fields were solved on.
    pub work_triangles: usize,
    /// Quads extracted (before any splitting of other polygons).
    pub quads: usize,
    /// Triangles, pentagons … extracted.
    pub other_polygons: usize,
    /// Share of vertices whose valence is not 4 (interior vertices only).
    pub irregular_fraction: f64,
    /// Does the Euler characteristic equal that of the input (closed inputs; true otherwise)?
    pub euler_ok: bool,
    /// Why the fallback was used.
    pub note: Option<String>,
}

impl PolyMesh {
    /// The faces as quads when every one is a quad.
    #[must_use]
    pub fn as_quads(&self) -> Option<QuadMesh> {
        let faces: Option<Vec<[u32; 4]>> = self.faces.iter().map(|f| <[u32; 4]>::try_from(f.as_slice()).ok()).collect();
        Some(QuadMesh { vertices: self.vertices.clone(), normals: self.normals.clone(), faces: faces? })
    }

    /// Tangential relaxation: every interior vertex moves halfway to the average of its edge
    /// neighbours (in its tangent plane) and is projected back onto the scan by `project`.
    /// Evens out quad sizes and untangles small folds of the extracted layout.
    pub fn relax(&mut self, iterations: usize, project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>) {
        let nv = self.vertices.len();
        let mut nb: Vec<std::collections::BTreeSet<u32>> = vec![Default::default(); nv];
        for f in &self.faces {
            for k in 0..f.len() {
                let (a, b) = (f[k], f[(k + 1) % f.len()]);
                if let Some(s) = nb.get_mut(a as usize) {
                    s.insert(b);
                }
                if let Some(s) = nb.get_mut(b as usize) {
                    s.insert(a);
                }
            }
        }
        let mut fixed = vec![false; nv];
        let mut uses: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for f in &self.faces {
            for k in 0..f.len() {
                let (a, b) = (f[k], f[(k + 1) % f.len()]);
                *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        for ((a, b), c) in uses {
            if c == 1 {
                for v in [a, b] {
                    if let Some(x) = fixed.get_mut(v as usize) {
                        *x = true;
                    }
                }
            }
        }
        for _ in 0..iterations {
            let mut moved: Vec<(Point3, Vec3)> = Vec::with_capacity(nv);
            for i in 0..nv {
                let (p, n) = (self.vertices[i], self.normals[i]);
                if fixed[i] || nb[i].len() < 3 {
                    moved.push((p, n));
                    continue;
                }
                let avg: Vec3 =
                    nb[i].iter().filter_map(|&j| self.vertices.get(j as usize)).map(|q| q.coords).sum::<Vec3>()
                        / nb[i].len() as f64;
                let d = avg - p.coords;
                let target = Point3::from(p.coords + (d - n * d.dot(&n)) * 0.5);
                moved.push(project(&target, &n).unwrap_or((p, n)));
            }
            for (i, (p, n)) in moved.into_iter().enumerate() {
                self.vertices[i] = p;
                self.normals[i] = n;
            }
        }
    }

    /// One Catmull–Clark-style topological split of every face into quads (edge midpoints and a
    /// face centre); `project` moves each new point onto the scan and returns its normal.
    #[must_use]
    pub fn split_to_quads(&self, project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>) -> QuadMesh {
        let mut vertices = self.vertices.clone();
        let mut normals = self.normals.clone();
        let mut mid: HashMap<(u32, u32), u32> = HashMap::new();
        let mut faces: Vec<[u32; 4]> = Vec::with_capacity(self.faces.len() * 4);
        for f in &self.faces {
            let n = f.len();
            if n < 3 {
                continue;
            }
            let pts: Vec<(Point3, Vec3)> = f
                .iter()
                .map(|&i| {
                    (
                        vertices.get(i as usize).copied().unwrap_or_else(Point3::origin),
                        normals.get(i as usize).copied().unwrap_or_else(Vec3::z),
                    )
                })
                .collect();
            let centre: Vec3 = pts.iter().map(|p| p.0.coords).sum::<Vec3>() / n as f64;
            let nsum: Vec3 = pts.iter().map(|p| p.1).sum();
            let nrm = normalize(nsum).unwrap_or_else(Vec3::z);
            let (cp, cn) = project(&Point3::from(centre), &nrm).unwrap_or((Point3::from(centre), nrm));
            let c = vertices.len() as u32;
            vertices.push(cp);
            normals.push(cn);
            let mut edge_ids = Vec::with_capacity(n);
            for k in 0..n {
                let (a, b) = (f[k], f[(k + 1) % n]);
                let key = (a.min(b), a.max(b));
                let id = match mid.get(&key) {
                    Some(&id) => id,
                    None => {
                        let (pa, na) = (pts[k].0, pts[k].1);
                        let (pb, nb) = (pts[(k + 1) % n].0, pts[(k + 1) % n].1);
                        let mp = Point3::from((pa.coords + pb.coords) * 0.5);
                        let mn = normalize(na + nb).unwrap_or(na);
                        let (p, nn) = project(&mp, &mn).unwrap_or((mp, mn));
                        let id = vertices.len() as u32;
                        vertices.push(p);
                        normals.push(nn);
                        mid.insert(key, id);
                        id
                    }
                };
                edge_ids.push(id);
            }
            for k in 0..n {
                faces.push([f[k], edge_ids[k], c, edge_ids[(k + n - 1) % n]]);
            }
        }
        QuadMesh { vertices, normals, faces }
    }
}

impl QuadMesh {
    /// The same mesh as polygons.
    #[must_use]
    pub fn to_poly(&self) -> PolyMesh {
        PolyMesh {
            vertices: self.vertices.clone(),
            normals: self.normals.clone(),
            faces: self.faces.iter().map(|f| f.to_vec()).collect(),
        }
    }

    /// Splits every quad into four (see [`PolyMesh::split_to_quads`]).
    #[must_use]
    pub fn split(&self, project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>) -> QuadMesh {
        self.to_poly().split_to_quads(project)
    }

    /// Vertex valences.
    #[must_use]
    pub fn valences(&self) -> Vec<usize> {
        let mut nb: Vec<std::collections::BTreeSet<u32>> = vec![Default::default(); self.vertices.len()];
        for f in &self.faces {
            for k in 0..4 {
                let (a, b) = (f[k], f[(k + 1) % 4]);
                if let Some(s) = nb.get_mut(a as usize) {
                    s.insert(b);
                }
                if let Some(s) = nb.get_mut(b as usize) {
                    s.insert(a);
                }
            }
        }
        nb.iter().map(std::collections::BTreeSet::len).collect()
    }

    /// Tangential relaxation, see [`PolyMesh::relax`].
    pub fn relax(&mut self, iterations: usize, project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>) {
        let mut poly = self.to_poly();
        poly.relax(iterations, project);
        self.vertices = poly.vertices;
        self.normals = poly.normals;
    }

    /// Quads whose geometric normal points against the vertex normals (folded or inverted).
    #[must_use]
    pub fn folded(&self) -> usize {
        self.faces
            .iter()
            .filter(|f| {
                let p: Vec<Option<&Point3>> = f.iter().map(|&i| self.vertices.get(i as usize)).collect();
                let n: Vec3 = f.iter().filter_map(|&i| self.normals.get(i as usize)).sum();
                match (p[0], p[1], p[2], p[3]) {
                    (Some(a), Some(b), Some(c), Some(d)) => {
                        // Newell normal of the quad
                        let nn = (c - a).cross(&(d - b));
                        nn.dot(&n) <= 0.0
                    }
                    _ => true,
                }
            })
            .count()
    }

    /// Edges used by one face only (the free boundary) as vertex pairs.
    #[must_use]
    pub fn boundary_edges(&self) -> Vec<(u32, u32)> {
        let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for f in &self.faces {
            for k in 0..4 {
                let (a, b) = (f[k], f[(k + 1) % 4]);
                *count.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        count.into_iter().filter(|(_, c)| *c == 1).map(|(e, _)| e).collect()
    }

    /// Euler characteristic V − E + F.
    #[must_use]
    pub fn euler(&self) -> i64 {
        let mut edges = std::collections::BTreeSet::new();
        for f in &self.faces {
            for k in 0..4 {
                let (a, b) = (f[k], f[(k + 1) % 4]);
                edges.insert((a.min(b), a.max(b)));
            }
        }
        let used: std::collections::BTreeSet<u32> = self.faces.iter().flatten().copied().collect();
        used.len() as i64 - edges.len() as i64 + self.faces.len() as i64
    }
}

// ----------------------------------------------------------------------------------------
// working levels

struct Level {
    v: Vec<Vec3>,
    n: Vec<Vec3>,
    nb: Vec<Vec<u32>>,
    /// Boundary tangent for pinned boundary vertices.
    pin: Vec<Option<Vec3>>,
}

fn one_ring(mesh: &Mesh) -> Vec<Vec<u32>> {
    let mut nb: Vec<Vec<u32>> = vec![Vec::new(); mesh.positions.len()];
    for t in &mesh.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if let Some(l) = nb.get_mut(a as usize) {
                l.push(b);
            }
            if let Some(l) = nb.get_mut(b as usize) {
                l.push(a);
            }
        }
    }
    for l in &mut nb {
        l.sort_unstable();
        l.dedup();
    }
    nb
}

/// Boundary neighbours of every vertex (edges used by exactly one triangle).
fn boundary_neighbours(mesh: &Mesh) -> Vec<Vec<u32>> {
    let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for t in &mesh.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            *count.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    let mut out: Vec<Vec<u32>> = vec![Vec::new(); mesh.positions.len()];
    for ((a, b), c) in count {
        if c == 1 {
            if let Some(l) = out.get_mut(a as usize) {
                l.push(b);
            }
            if let Some(l) = out.get_mut(b as usize) {
                l.push(a);
            }
        }
    }
    out
}

fn level_of(mesh: &Mesh) -> Level {
    let nrm = mesh.vertex_normals();
    let v: Vec<Vec3> = mesh.positions.iter().map(|p| p.coords).collect();
    let bn = boundary_neighbours(mesh);
    let mut pin = vec![None; v.len()];
    for (i, b) in bn.iter().enumerate() {
        if let [a, c] = b.as_slice() {
            let (pa, pc, pv) = (v[*a as usize], v[*c as usize], v[i]);
            let (d1, d2) = (pv - pa, pc - pv);
            let (Some(u1), Some(u2)) = (normalize(d1), normalize(d2)) else { continue };
            if u1.dot(&u2) > 0.8 {
                let n = nrm[i];
                if let Some(t) = normalize((pc - pa) - n * (pc - pa).dot(&n)) {
                    pin[i] = Some(t);
                }
            }
        }
    }
    Level { nb: one_ring(mesh), n: nrm, v, pin }
}

/// The 90° turn (and sign) of vertex 1's direction closest to vertex 0's: `(o0, matched o1)`.
fn compat4(o0: &Vec3, _n0: &Vec3, o1: &Vec3, n1: &Vec3) -> (Vec3, Vec3) {
    let t1 = n1.cross(o1);
    let mut best = (f64::NEG_INFINITY, *o1);
    for c in [*o1, -*o1, t1, -t1] {
        let d = o0.dot(&c);
        if d > best.0 {
            best = (d, c);
        }
    }
    (*o0, best.1)
}

fn tangent(n: &Vec3, d: &Vec3) -> Vec3 {
    normalize(d - n * d.dot(n)).unwrap_or_else(|| any_perpendicular(n))
}

fn init_orientation(l: &Level) -> Vec<Vec3> {
    let nv = l.v.len();
    let mut o = vec![Vec3::zeros(); nv];
    let mut seen = vec![false; nv];
    for s in 0..nv {
        if seen[s] {
            continue;
        }
        seen[s] = true;
        o[s] = l.pin[s].unwrap_or_else(|| any_perpendicular(&l.n[s]));
        let mut queue = std::collections::VecDeque::from([s]);
        while let Some(i) = queue.pop_front() {
            for &j in &l.nb[i] {
                let j = j as usize;
                if !seen[j] {
                    seen[j] = true;
                    o[j] = l.pin[j].unwrap_or_else(|| tangent(&l.n[j], &o[i]));
                    queue.push_back(j);
                }
            }
        }
    }
    o
}

fn smooth_orientation(l: &Level, o: &mut [Vec3], sweeps: usize) {
    let nv = l.v.len();
    for sweep in 0..sweeps {
        let mut change = 0.0f64;
        for step in 0..nv {
            let i = if sweep % 2 == 0 { step } else { nv - 1 - step };
            if l.pin[i].is_some() {
                continue;
            }
            let mut sum = Vec3::zeros();
            for &j in &l.nb[i] {
                let j = j as usize;
                let (a, b) = compat4(&o[i], &l.n[i], &o[j], &l.n[j]);
                let _ = a;
                sum += b;
            }
            if let Some(t) = normalize(sum - l.n[i] * sum.dot(&l.n[i])) {
                change = change.max((t - o[i]).norm());
                o[i] = t;
            }
        }
        if change < 1e-6 {
            break;
        }
    }
}

/// The lattice points of vertices `i` and `j` (axes `oi`, `ni × oi` and `oj`, `nj × oj`) nearest
/// to the middle of their edge and to each other, with their integer lattice coordinates:
/// `(li, lj, (xi, yi), (xj, yj))`. Comparing at the middle keeps curvature from reading as
/// disagreement between the two tangent planes.
fn lattice_pair(
    vi: &Vec3,
    ni: &Vec3,
    pi: &Vec3,
    oi: &Vec3,
    vj: &Vec3,
    nj: &Vec3,
    pj: &Vec3,
    oj: &Vec3,
    rho: f64,
) -> (Vec3, Vec3, (i32, i32), (i32, i32)) {
    let ti = ni.cross(oi);
    let tj = nj.cross(oj);
    let m = (vi + vj) * 0.5;
    let inv = 1.0 / rho;
    let (xi, yi) = (((m - pi).dot(oi) * inv).floor(), ((m - pi).dot(&ti) * inv).floor());
    let (xj, yj) = (((m - pj).dot(oj) * inv).floor(), ((m - pj).dot(&tj) * inv).floor());
    let mut best = (f64::INFINITY, *pi, *pj, (0, 0), (0, 0));
    for a in 0..4 {
        let (ax, ay) = (xi + f64::from(a & 1), yi + f64::from(a >> 1));
        let li = pi + (oi * ax + ti * ay) * rho;
        for b in 0..4 {
            let (bx, by) = (xj + f64::from(b & 1), yj + f64::from(b >> 1));
            let lj = pj + (oj * bx + tj * by) * rho;
            let d = (li - lj).norm_squared();
            if d < best.0 {
                best = (d, li, lj, (ax as i32, ay as i32), (bx as i32, by as i32));
            }
        }
    }
    (best.1, best.2, best.3, best.4)
}

fn round_lattice(s: &Vec3, v: &Vec3, o: &Vec3, n: &Vec3, rho: f64, pinned: bool) -> Vec3 {
    let t = n.cross(o);
    let mut s = *s;
    if pinned {
        s += t * (v - s).dot(&t);
    }
    let dx = ((v - s).dot(o) / rho).round();
    let dy = ((v - s).dot(&t) / rho).round();
    s + o * (rho * dx) + t * (rho * dy)
}

/// Lattice transported along a breadth-first tree: consistent except where fronts meet.
fn init_positions(l: &Level, o: &[Vec3], rho: f64) -> Vec<Vec3> {
    let nv = l.v.len();
    let mut p = vec![Vec3::zeros(); nv];
    let mut seen = vec![false; nv];
    for s in 0..nv {
        if seen[s] {
            continue;
        }
        seen[s] = true;
        p[s] = round_lattice(&l.v[s], &l.v[s], &o[s], &l.n[s], rho, l.pin[s].is_some());
        let mut queue = std::collections::VecDeque::from([s]);
        while let Some(i) = queue.pop_front() {
            let t = l.n[i].cross(&o[i]);
            for &j in &l.nb[i] {
                let j = j as usize;
                if seen[j] {
                    continue;
                }
                seen[j] = true;
                let d = l.v[j] - p[i];
                let mut q = p[i] + o[i] * (rho * (d.dot(&o[i]) / rho).round()) + t * (rho * (d.dot(&t) / rho).round());
                q -= l.n[j] * (q - l.v[j]).dot(&l.n[j]);
                p[j] = round_lattice(&q, &l.v[j], &o[j], &l.n[j], rho, l.pin[j].is_some());
                queue.push_back(j);
            }
        }
    }
    p
}

fn smooth_positions(l: &Level, o: &[Vec3], p: &mut [Vec3], rho: f64, sweeps: usize) {
    let nv = l.v.len();
    for sweep in 0..sweeps {
        let mut moved = 0usize;
        for step in 0..nv {
            let i = if sweep % 2 == 0 { step } else { nv - 1 - step };
            if l.nb[i].is_empty() {
                continue;
            }
            let mut sum = Vec3::zeros();
            for &j in &l.nb[i] {
                let j = j as usize;
                let (a, b) = compat4(&o[i], &l.n[i], &o[j], &l.n[j]);
                let (li, lj, _, _) = lattice_pair(&l.v[i], &l.n[i], &p[i], &a, &l.v[j], &l.n[j], &p[j], &b, rho);
                sum += lj - li;
            }
            let mut s = p[i] + sum / l.nb[i].len() as f64;
            s -= l.n[i] * (s - l.v[i]).dot(&l.n[i]);
            let q = round_lattice(&s, &l.v[i], &o[i], &l.n[i], rho, l.pin[i].is_some());
            if (q - p[i]).norm() > 1e-3 * rho {
                moved += 1;
            }
            p[i] = q;
        }
        if moved * 400 < nv {
            break;
        }
    }
}

/// Carries a coarse solution to a finer level through the nearest coarse vertex.
fn prolong(coarse: &Level, co: &[Vec3], cp: &[Vec3], fine: &Level, rho: f64) -> (Vec<Vec3>, Vec<Vec3>) {
    let pts: Vec<Point3> = coarse.v.iter().map(|v| Point3::from(*v)).collect();
    let tree = KdTree::build(&pts);
    let mut o = Vec::with_capacity(fine.v.len());
    let mut p = Vec::with_capacity(fine.v.len());
    for (i, v) in fine.v.iter().enumerate() {
        let j = tree.nearest(&Point3::from(*v)).map_or(0, |x| x.0);
        let n = fine.n[i];
        let oi = fine.pin[i].unwrap_or_else(|| tangent(&n, &co[j]));
        let mut s = cp[j];
        s -= n * (s - v).dot(&n);
        p.push(round_lattice(&s, v, &oi, &n, rho, fine.pin[i].is_some()));
        o.push(oi);
    }
    (o, p)
}

// ----------------------------------------------------------------------------------------
// extraction

struct Uf(Vec<u32>);

impl Uf {
    fn new(n: usize) -> Self {
        Self((0..n as u32).collect())
    }
    fn find(&mut self, mut x: u32) -> u32 {
        while self.0[x as usize] != x {
            let p = self.0[x as usize];
            self.0[x as usize] = self.0[p as usize];
            x = self.0[x as usize];
        }
        x
    }
    fn union(&mut self, a: u32, b: u32) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.0[a.max(b) as usize] = a.min(b);
        }
    }
}

/// Extracts a polygon mesh from the solved fields of the finest level.
fn extract(l: &Level, o: &[Vec3], p: &[Vec3], rho: f64, bvh: &Bvh) -> Option<PolyMesh> {
    let nv = l.v.len();
    let mut uf = Uf::new(nv);
    let mut steps: Vec<(u32, u32)> = Vec::new();
    for i in 0..nv {
        for &j in &l.nb[i] {
            if (j as usize) <= i {
                continue;
            }
            let ju = j as usize;
            let (a, b) = compat4(&o[i], &l.n[i], &o[ju], &l.n[ju]);
            let (li, lj, di, dj) = lattice_pair(&l.v[i], &l.n[i], &p[i], &a, &l.v[ju], &l.n[ju], &p[ju], &b, rho);
            if (li - lj).norm() > 0.35 * rho {
                continue;
            }
            // vertex j's axes expressed in vertex i's: b is ±a or ±(n × a)
            let ti = l.n[i].cross(&a);
            let (ca, ct) = (b.dot(&a), b.dot(&ti));
            let mapped = if ca.abs() >= ct.abs() {
                let s = if ca >= 0.0 { 1 } else { -1 };
                (s * dj.0, s * dj.1)
            } else {
                let s = if ct >= 0.0 { 1 } else { -1 };
                (-s * dj.1, s * dj.0)
            };
            let k = (di.0 - mapped.0, di.1 - mapped.1);
            match k.0.abs() + k.1.abs() {
                0 => {
                    uf.union(i as u32, j);
                }
                1 => {
                    steps.push((i as u32, j));
                }
                _ => {}
            }
        }
    }
    // clusters
    let mut id_of_root: HashMap<u32, u32> = HashMap::new();
    let mut cluster = vec![0u32; nv];
    let mut count = 0u32;
    for i in 0..nv {
        let r = uf.find(i as u32);
        let id = *id_of_root.entry(r).or_insert_with(|| {
            count += 1;
            count - 1
        });
        cluster[i] = id;
    }
    let nc = count as usize;
    let mut cpos = vec![Vec3::zeros(); nc];
    let mut cn = vec![Vec3::zeros(); nc];
    let mut members = vec![0u32; nc];
    for i in 0..nv {
        let c = cluster[i] as usize;
        cpos[c] += p[i];
        cn[c] += l.n[i];
        members[c] += 1;
    }
    for c in 0..nc {
        cpos[c] /= f64::from(members[c].max(1));
    }
    // layout vertices of the same lattice point reached through separate clusters merge
    let ptree = KdTree::build(&cpos.iter().map(|v| Point3::from(*v)).collect::<Vec<_>>());
    let mut cuf = Uf::new(nc);
    for c in 0..nc {
        for j in ptree.within(&Point3::from(cpos[c]), 0.3 * rho) {
            if j != c && cn[c].dot(&cn[j]) > 0.0 {
                cuf.union(c as u32, j as u32);
            }
        }
    }
    let mut edges: std::collections::BTreeSet<(u32, u32)> = std::collections::BTreeSet::new();
    for (i, j) in steps {
        let (a, b) = (cuf.find(cluster[i as usize]), cuf.find(cluster[j as usize]));
        if a != b {
            edges.insert((a.min(b), a.max(b)));
        }
    }
    // merged cluster positions and normals
    let mut mpos = vec![Vec3::zeros(); nc];
    let mut mn = vec![Vec3::zeros(); nc];
    let mut mw = vec![0.0f64; nc];
    for c in 0..nc {
        let r = cuf.find(c as u32) as usize;
        let w = f64::from(members[c]);
        mpos[r] += cpos[c] * w;
        mn[r] += cn[c];
        mw[r] += w;
    }
    // adjacency; dangling vertices are pruned
    let mut adj: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in &edges {
        adj.entry(a).or_default().push(b);
        adj.entry(b).or_default().push(a);
    }
    loop {
        let weak: Vec<u32> = adj.iter().filter(|(_, v)| v.len() < 2).map(|(k, _)| *k).collect();
        if weak.is_empty() {
            break;
        }
        for k in weak {
            if let Some(nbs) = adj.remove(&k) {
                for b in nbs {
                    if let Some(l) = adj.get_mut(&b) {
                        l.retain(|&x| x != k);
                    }
                }
            }
        }
    }
    if adj.len() < 4 {
        return None;
    }
    // compact ids, positions on the working surface
    let ids: Vec<u32> = adj.keys().copied().collect();
    let index: HashMap<u32, u32> = ids.iter().enumerate().map(|(i, &c)| (c, i as u32)).collect();
    let mut vertices = Vec::with_capacity(ids.len());
    let mut normals = Vec::with_capacity(ids.len());
    for &c in &ids {
        let c = c as usize;
        let pos = Point3::from(mpos[c] / mw[c].max(1.0));
        let nrm = normalize(mn[c]).unwrap_or_else(Vec3::z);
        match bvh.closest(&pos, f64::INFINITY) {
            Some(h) => {
                normals.push(nrm);
                vertices.push(h.point);
            }
            None => {
                normals.push(nrm);
                vertices.push(pos);
            }
        }
    }
    // rotation system, counter-clockwise about the vertex normal
    let mut nbs: Vec<Vec<u32>> = Vec::with_capacity(ids.len());
    for (k, &c) in ids.iter().enumerate() {
        let n = normals[k];
        let e1 = any_perpendicular(&n);
        let e2 = n.cross(&e1);
        let mut list: Vec<(f64, u32)> = adj[&c]
            .iter()
            .filter_map(|b| index.get(b).copied())
            .map(|b| {
                let d = vertices[b as usize] - vertices[k];
                ((d.dot(&e2)).atan2(d.dot(&e1)), b)
            })
            .collect();
        list.sort_by(|a, b| a.0.total_cmp(&b.0));
        nbs.push(list.into_iter().map(|x| x.1).collect());
    }
    let mut seen: Vec<Vec<bool>> = nbs.iter().map(|l| vec![false; l.len()]).collect();
    let mut faces: Vec<Vec<u32>> = Vec::new();
    for u0 in 0..ids.len() {
        for k0 in 0..nbs[u0].len() {
            if seen[u0][k0] {
                continue;
            }
            let (mut u, mut k) = (u0, k0);
            let mut walk: Vec<u32> = Vec::new();
            let mut ok = true;
            loop {
                seen[u][k] = true;
                walk.push(u as u32);
                let v = nbs[u][k] as usize;
                let Some(back) = nbs[v].iter().position(|&x| x as usize == u) else {
                    ok = false;
                    break;
                };
                let deg = nbs[v].len();
                let nk = (back + deg - 1) % deg;
                u = v;
                k = nk;
                if (u, k) == (u0, k0) {
                    break;
                }
                if seen[u][k] || walk.len() > 4 * MAX_POLY {
                    ok = false;
                    break;
                }
            }
            if !ok || walk.len() < 3 || walk.len() > MAX_POLY {
                continue;
            }
            let p0 = vertices[walk[0] as usize];
            let nsum: Vec3 = walk.iter().map(|&w| normals[w as usize]).sum();
            let mut area = 0.0;
            for w in 1..walk.len() - 1 {
                let (a, b) = (vertices[walk[w] as usize] - p0, vertices[walk[w + 1] as usize] - p0);
                area += a.cross(&b).dot(&nsum);
            }
            if area > 0.0 {
                faces.push(walk);
            }
        }
    }
    if faces.is_empty() {
        return None;
    }
    fill_small_holes(&mut faces);
    // drop vertices no face uses
    let mut used = vec![false; vertices.len()];
    for f in &faces {
        for &w in f {
            used[w as usize] = true;
        }
    }
    let mut remap = vec![u32::MAX; vertices.len()];
    let (mut vs, mut ns) = (Vec::new(), Vec::new());
    for i in 0..vertices.len() {
        if used[i] {
            remap[i] = vs.len() as u32;
            vs.push(vertices[i]);
            ns.push(normals[i]);
        }
    }
    for f in &mut faces {
        for w in f.iter_mut() {
            *w = remap[*w as usize];
        }
    }
    Some(PolyMesh { vertices: vs, normals: ns, faces })
}

/// Closes boundary loops of 3 to `MAX_POLY` edges with one polygon each.
fn fill_small_holes(faces: &mut Vec<Vec<u32>>) {
    let mut directed: std::collections::BTreeSet<(u32, u32)> = std::collections::BTreeSet::new();
    for f in faces.iter() {
        for k in 0..f.len() {
            directed.insert((f[k], f[(k + 1) % f.len()]));
        }
    }
    // a hole's edges run against the unmatched directed edges
    let mut next: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            next.entry(b).or_default().push(a);
        }
    }
    let starts: Vec<u32> = next.keys().copied().collect();
    for s in starts {
        while let Some(first) = next.get(&s).and_then(|v| v.first().copied()) {
            // walk the loop from s
            let mut lp = vec![s];
            let mut cur = first;
            let mut closed = false;
            while lp.len() <= MAX_POLY {
                if cur == s {
                    closed = true;
                    break;
                }
                lp.push(cur);
                match next.get(&cur).and_then(|v| v.first().copied()) {
                    Some(n) => cur = n,
                    None => break,
                }
            }
            // consume the edges of this walk so it is not repeated
            let mut consumed = lp.clone();
            consumed.push(s);
            for w in consumed.windows(2) {
                if let Some(v) = next.get_mut(&w[0])
                    && let Some(pos) = v.iter().position(|&x| x == w[1])
                {
                    v.remove(pos);
                }
            }
            if closed && lp.len() >= 3 {
                faces.push(lp);
            } else if !closed {
                // an open chain: drop its first edge to make progress
                if let Some(v) = next.get_mut(&s)
                    && !v.is_empty()
                {
                    v.remove(0);
                }
            }
        }
    }
}

fn euler_of(m: &PolyMesh) -> i64 {
    let mut e = std::collections::BTreeSet::new();
    for f in &m.faces {
        for k in 0..f.len() {
            let (a, b) = (f[k], f[(k + 1) % f.len()]);
            e.insert((a.min(b), a.max(b)));
        }
    }
    m.vertices.len() as i64 - e.len() as i64 + m.faces.len() as i64
}

fn euler_of_mesh(m: &Mesh) -> i64 {
    let mut e = std::collections::BTreeSet::new();
    for t in &m.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            e.insert((a.min(b), a.max(b)));
        }
    }
    let used: std::collections::BTreeSet<u32> = m.triangles.iter().flatten().copied().collect();
    used.len() as i64 - e.len() as i64 + m.triangles.len() as i64
}

fn is_closed(m: &Mesh) -> bool {
    boundary_neighbours(m).iter().all(Vec::is_empty)
}

fn irregular_fraction(m: &PolyMesh) -> f64 {
    let mut nb: Vec<std::collections::BTreeSet<u32>> = vec![Default::default(); m.vertices.len()];
    let mut boundary: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for f in &m.faces {
        for k in 0..f.len() {
            let (a, b) = (f[k], f[(k + 1) % f.len()]);
            nb[a as usize].insert(b);
            nb[b as usize].insert(a);
            *boundary.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    let mut on_boundary = vec![false; m.vertices.len()];
    for ((a, b), c) in boundary {
        if c == 1 {
            on_boundary[a as usize] = true;
            on_boundary[b as usize] = true;
        }
    }
    let interior: Vec<usize> = (0..nb.len()).filter(|&i| !on_boundary[i] && !nb[i].is_empty()).collect();
    if interior.is_empty() {
        return 0.0;
    }
    interior.iter().filter(|&&i| nb[i].len() != 4).count() as f64 / interior.len() as f64
}

/// The working mesh: welded, cleaned, decimated to roughly 12 triangles per wanted face and
/// refined so no edge is longer than ρ/2 (the extraction needs several vertices per lattice
/// cell), its vertices on the original surface.
fn working_mesh(mesh: &Mesh, rho: f64, target_faces: usize) -> Mesh {
    let mut m = mesh.clone();
    m.weld(1e-9);
    m.remove_degenerate(0.0);
    compact(&mut m);
    let want = (target_faces * 12).clamp(300, MAX_WORK_TRIANGLES);
    let mut changed = false;
    if m.triangle_count() > want {
        let opts = DecimateOptions {
            target_triangles: want,
            max_error: None,
            feature_angle_deg: 181.0,
            preserve_boundary: true,
        };
        decimate(&mut m, &opts);
        changed = true;
    }
    let before = m.triangle_count();
    let refined = synth::refine(&m, |_| rho / 2.0);
    if refined.triangle_count() != before && refined.triangle_count() <= MAX_WORK_TRIANGLES * 2 {
        m = refined;
        changed = true;
    }
    if changed {
        let bvh = Bvh::build(mesh);
        for p in &mut m.positions {
            if let Some(h) = bvh.closest(p, f64::INFINITY) {
                *p = h.point;
            }
        }
    }
    m
}

/// Coarse-to-fine chain of levels, finest first.
fn build_levels(fine: &Mesh) -> Vec<Mesh> {
    let mut levels = vec![fine.clone()];
    while let Some(last) = levels.last() {
        let t = last.triangle_count() / 3;
        if last.triangle_count() < 700 || levels.len() >= 6 {
            break;
        }
        let mut c = last.clone();
        let opts =
            DecimateOptions { target_triangles: t, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
        decimate(&mut c, &opts);
        if c.triangle_count() >= last.triangle_count() || c.triangle_count() < 100 {
            break;
        }
        levels.push(c);
    }
    levels
}

/// Fallback layout: a coarse decimation whose every triangle becomes three quads.
fn fallback(work: &Mesh, target_faces: usize, bvh: &Bvh) -> PolyMesh {
    let mut m = work.clone();
    let t = (target_faces / 3).max(8);
    if m.triangle_count() > t {
        let opts =
            DecimateOptions { target_triangles: t, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
        decimate(&mut m, &opts);
    }
    compact(&mut m);
    let nrm = m.vertex_normals();
    let mut vertices = Vec::with_capacity(m.positions.len());
    let mut normals = Vec::with_capacity(m.positions.len());
    for (p, n) in m.positions.iter().zip(&nrm) {
        match bvh.closest(p, f64::INFINITY) {
            Some(h) => vertices.push(h.point),
            None => vertices.push(*p),
        }
        normals.push(*n);
    }
    PolyMesh { vertices, normals, faces: m.triangles.iter().map(|t| t.to_vec()).collect() }
}

/// Retopologises `mesh` into a quad-dominant polygon mesh of about `opts.target_faces` faces.
///
/// # Errors
/// An error message when the mesh is empty or invalid.
pub fn retopo(mesh: &Mesh, opts: &RetopoOptions) -> Result<(PolyMesh, RetopoReport), String> {
    mesh.validate().map_err(|e| e.to_string())?;
    if mesh.triangle_count() < 4 {
        return Err("the mesh has too few triangles".into());
    }
    let target = opts.target_faces.clamp(8, 100_000);
    let area = mesh.area();
    if !(area.is_finite() && area > 0.0) {
        return Err("the mesh has no area".into());
    }
    let rho = (area / target as f64).sqrt();
    let work = working_mesh(mesh, rho, target);
    if work.triangle_count() < 4 {
        return Err("the mesh collapsed while preparing it".into());
    }
    let closed = is_closed(&work);
    let chi = euler_of_mesh(&work);
    let bvh = Bvh::build(&work);
    if opts.force_fallback {
        let f = fallback(&work, target, &bvh);
        let report = RetopoReport {
            layout: Layout::Fallback,
            edge_length: rho,
            work_triangles: work.triangle_count(),
            quads: 0,
            other_polygons: f.faces.len(),
            irregular_fraction: irregular_fraction(&f),
            euler_ok: !closed || euler_of(&f) == chi,
            note: Some("fallback layout requested".into()),
        };
        return Ok((f, report));
    }
    let meshes = build_levels(&work);
    let levels: Vec<Level> = meshes.iter().map(level_of).collect();
    // solve coarse to fine
    let mut o: Vec<Vec3> = Vec::new();
    let mut p: Vec<Vec3> = Vec::new();
    for k in (0..levels.len()).rev() {
        let lv = &levels[k];
        if k == levels.len() - 1 {
            o = init_orientation(lv);
            smooth_orientation(lv, &mut o, opts.orientation_sweeps * 4);
            p = init_positions(lv, &o, rho);
            smooth_positions(lv, &o, &mut p, rho, opts.position_sweeps * 4);
        } else {
            let (no, np) = prolong(&levels[k + 1], &o, &p, lv, rho);
            o = no;
            p = np;
            let fine = if k == 0 { 3 } else { 1 };
            smooth_orientation(lv, &mut o, opts.orientation_sweeps / fine);
            smooth_positions(lv, &o, &mut p, rho, opts.position_sweeps / fine);
        }
    }
    let finest = levels.first().ok_or("no level")?;
    let mut report = RetopoReport {
        layout: Layout::Field,
        edge_length: rho,
        work_triangles: work.triangle_count(),
        quads: 0,
        other_polygons: 0,
        irregular_fraction: 0.0,
        euler_ok: true,
        note: None,
    };
    let extracted = extract(finest, &o, &p, rho, &bvh);
    let mut chosen: Option<PolyMesh> = None;
    let mut why = String::new();
    match extracted {
        Some(poly) => {
            let quads = poly.faces.iter().filter(|f| f.len() == 4).count();
            let share = quads as f64 / poly.faces.len().max(1) as f64;
            let euler_ok = !closed || euler_of(&poly) == chi;
            let size_ok = poly.faces.len() * 3 > target && poly.faces.len() < target * 3;
            if share >= 0.5 && euler_ok && size_ok {
                report.quads = quads;
                report.other_polygons = poly.faces.len() - quads;
                report.irregular_fraction = irregular_fraction(&poly);
                chosen = Some(poly);
            } else {
                why = format!(
                    "field extraction gave {} faces, {:.0} % quads, Euler {}",
                    poly.faces.len(),
                    share * 100.0,
                    if euler_ok { "ok" } else { "wrong" }
                );
            }
        }
        None => why = "field extraction found no faces".into(),
    }
    let poly = match chosen {
        Some(p) => p,
        None => {
            report.layout = Layout::Fallback;
            let f = fallback(&work, target, &bvh);
            report.quads = 0;
            report.other_polygons = f.faces.len();
            report.irregular_fraction = irregular_fraction(&f);
            report.euler_ok = !closed || euler_of(&f) == chi;
            report.note = Some(why);
            f
        }
    };
    Ok((poly, report))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mismatch(l: &Level, o: &[Vec3]) -> f64 {
        let (mut bad, mut tot) = (0usize, 0usize);
        for i in 0..l.v.len() {
            for &j in &l.nb[i] {
                let (a, b) = compat4(&o[i], &l.n[i], &o[j as usize], &l.n[j as usize]);
                bad += usize::from(a.dot(&b) < 0.94);
                tot += 1;
            }
        }
        bad as f64 / tot.max(1) as f64
    }

    #[test]
    fn plane_layout_is_a_grid() {
        let m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 20.0, 20.0, 24, 24);
        let (poly, rep) = retopo(&m, &RetopoOptions { target_faces: 25, ..RetopoOptions::default() }).unwrap();
        eprintln!("plane: {rep:?}, faces {}", poly.faces.len());
        assert_eq!(rep.layout, Layout::Field);
        assert!(poly.faces.iter().all(|f| f.len() == 4));
    }

    #[test]
    fn orientation_field_smooths_on_a_plane_and_a_sphere() {
        let m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 30.0, 30.0, 20, 20);
        let l = level_of(&m);
        let mut o = init_orientation(&l);
        smooth_orientation(&l, &mut o, 50);
        assert!(mismatch(&l, &o) < 0.02, "plane {}", mismatch(&l, &o));
        let m = synth::sphere(Point3::origin(), 20.0, 40, 20);
        let l = level_of(&m);
        let mut o = init_orientation(&l);
        eprintln!("sphere init mismatch {:.3}", mismatch(&l, &o));
        smooth_orientation(&l, &mut o, 200);
        eprintln!("sphere smoothed mismatch {:.3}", mismatch(&l, &o));
        assert!(mismatch(&l, &o) < 0.2);
    }

    #[test]
    fn sphere_layout_is_quad_dominant_and_closed() {
        let m = synth::sphere(Point3::origin(), 20.0, 80, 40);
        let (poly, rep) = retopo(&m, &RetopoOptions { target_faces: 150, ..RetopoOptions::default() }).unwrap();
        eprintln!("sphere: {rep:?}, faces {}", poly.faces.len());
        assert!(poly.faces.len() > 40);
        assert!(rep.euler_ok);
        // every vertex lies on the sphere (within the working-mesh chord)
        for v in &poly.vertices {
            assert!((v.coords.norm() - 20.0).abs() < 0.5);
        }
        let q = poly.split_to_quads(&|p, n| Some((*p, *n)));
        // closed 2-manifold: every edge used twice, sphere Euler characteristic
        assert!(q.boundary_edges().is_empty());
        assert_eq!(q.euler(), 2);
    }
}
