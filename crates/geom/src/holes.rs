//! Hole filling: boundary loops triangulated by ear clipping in the loop's mean plane; the smooth
//! variant refines the patch (centroid splits and Delaunay flips) and fairs it with the
//! surrounding ring held fixed (membrane, then thin-plate iterations).
use crate::ear::{ear_clip, fan_fill};
use crate::{Mesh, Point3, Vec3, par, tri};
use std::collections::{HashMap, HashSet};

/// Closed boundary loops as vertex lists, each in the direction of its triangles' edges.
#[must_use]
pub fn boundary_loops(mesh: &Mesh) -> Vec<Vec<u32>> {
    if mesh.validate().is_err() {
        return Vec::new();
    }
    let mut directed: HashSet<(u32, u32)> = HashSet::with_capacity(mesh.triangles.len() * 3);
    for t in &mesh.triangles {
        for k in 0..3 {
            directed.insert((t[k], t[(k + 1) % 3]));
        }
    }
    let mut next: HashMap<u32, Vec<u32>> = HashMap::new();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            next.entry(a).or_default().push(b);
        }
    }
    // hash order is not deterministic; the loops must be (two runs give the same mesh)
    for v in next.values_mut() {
        v.sort_unstable();
    }
    let mut used: HashSet<(u32, u32)> = HashSet::new();
    let mut starts: Vec<u32> = next.keys().copied().collect();
    starts.sort_unstable();
    let mut loops = Vec::new();
    for s in starts {
        let outs = next.get(&s).cloned().unwrap_or_default();
        for first in outs {
            if used.contains(&(s, first)) {
                continue;
            }
            let mut lp = vec![s];
            let (mut a, mut b) = (s, first);
            let mut closed = false;
            for _ in 0..directed.len() {
                used.insert((a, b));
                if b == s {
                    closed = true;
                    break;
                }
                lp.push(b);
                let Some(nb) = next.get(&b).and_then(|v| v.iter().find(|&&c| !used.contains(&(b, c))).copied()) else {
                    break;
                };
                a = b;
                b = nb;
            }
            if closed && lp.len() >= 3 {
                loops.push(lp);
            }
        }
    }
    loops
}

/// Hole-filling result.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FillReport {
    /// Holes filled.
    pub filled: usize,
    /// Holes left (longer than the limit or not triangulable).
    pub skipped: usize,
    /// Triangles added.
    pub added_triangles: usize,
}

/// The longest loop filled (boundary edges). Ear clipping takes about n² steps, so the bound is
/// time, not correctness; see the measurement of a 5 000-edge hole in `crates/io/tests/h2_perf.rs`.
pub const MAX_FILL_EDGES: usize = 20_000;

/// Rings of triangles around a loop copied into the workspace of its patch: more than the
/// sharpening (two rings) and the fairing (one) read.
const WORKSPACE_RINGS: usize = 3;

/// Fills boundary loops with at most `max_edges` edges (and at most [`MAX_FILL_EDGES`]). `smooth`
/// refines and fairs the patch so it follows the surrounding curvature, and keeps the sharp edges
/// and corners the hole lies over; otherwise the patch is flat (ear clipping only). Every hole is
/// filled from the mesh as it was before any was filled, so the holes are independent (they run in
/// parallel) and the result does not depend on the number of threads.
pub fn fill_holes(mesh: &mut Mesh, max_edges: usize, smooth: bool) -> FillReport {
    fill_holes_with(mesh, max_edges, smooth, smooth)
}

/// [`fill_holes`], with the keeping of sharp edges (`sharpen`, only with `smooth`) on its own
/// switch.
pub(crate) fn fill_holes_with(mesh: &mut Mesh, max_edges: usize, smooth: bool, sharpen: bool) -> FillReport {
    let mut rep = FillReport::default();
    let max_edges = max_edges.clamp(3, MAX_FILL_EDGES);
    let (todo, long): (Vec<Vec<u32>>, Vec<Vec<u32>>) =
        boundary_loops(mesh).into_iter().partition(|lp| lp.len() <= max_edges);
    rep.skipped = long.len();
    drop(long);
    if todo.is_empty() {
        return rep;
    }
    let faces = VertexFaces::new(mesh);
    let source: &Mesh = mesh;
    let fill = |lp: &Vec<u32>| fill_in_workspace(source, &faces, lp, smooth, sharpen);
    // a single hole stays on this thread (the crease tests read a thread-local reason)
    let patches: Vec<Patch> =
        if todo.len() < 2 { todo.iter().map(fill).collect() } else { par::map_slice(&todo, fill) };
    for patch in patches {
        rep.filled += 1;
        rep.added_triangles += patch.triangles.len();
        patch.append_to(mesh);
    }
    rep
}

/// The triangles around every vertex of a mesh (compressed rows).
struct VertexFaces {
    start: Vec<u32>,
    faces: Vec<u32>,
}

impl VertexFaces {
    fn new(mesh: &Mesh) -> Self {
        let mut start = vec![0u32; mesh.positions.len() + 1];
        for t in &mesh.triangles {
            for &v in t {
                start[v as usize + 1] += 1;
            }
        }
        for i in 1..start.len() {
            start[i] += start[i - 1];
        }
        let mut next = start.clone();
        let mut faces = vec![0u32; start.last().copied().unwrap_or(0) as usize];
        for (i, t) in mesh.triangles.iter().enumerate() {
            for &v in t {
                faces[next[v as usize] as usize] = i as u32;
                next[v as usize] += 1;
            }
        }
        Self { start, faces }
    }

    fn of(&self, v: u32) -> &[u32] {
        let (a, b) = (self.start[v as usize] as usize, self.start[v as usize + 1] as usize);
        &self.faces[a..b]
    }
}

/// What filling one hole added, in the vertex numbers of its workspace.
struct Patch {
    /// Mesh vertex of every workspace vertex that was in the mesh.
    origin: Vec<u32>,
    /// New vertices (workspace numbers from `origin.len()` on).
    positions: Vec<Point3>,
    /// New triangles.
    triangles: Vec<[u32; 3]>,
}

impl Patch {
    fn append_to(self, mesh: &mut Mesh) {
        let (base, first) = (self.origin.len() as u32, mesh.positions.len() as u32);
        let map = |v: u32| if v < base { self.origin[v as usize] } else { first + (v - base) };
        mesh.triangles.extend(self.triangles.iter().map(|t| t.map(map)));
        mesh.positions.extend(self.positions);
    }
}

/// Fills the loop `lp` in a copy of the triangles around it, which is all the fairing and the
/// sharpening read; the copy numbers its vertices in the order of the mesh's, so the patch is the
/// one the whole mesh would give.
fn fill_in_workspace(mesh: &Mesh, faces: &VertexFaces, lp: &[u32], smooth: bool, sharpen: bool) -> Patch {
    let mut tris: Vec<u32> = Vec::new();
    let mut seen: HashSet<u32> = HashSet::new();
    let mut front: Vec<u32> = lp.to_vec();
    for _ in 0..WORKSPACE_RINGS {
        let mut next = Vec::new();
        for &v in &front {
            for &t in faces.of(v) {
                if seen.insert(t) {
                    tris.push(t);
                    next.extend(mesh.triangles[t as usize]);
                }
            }
        }
        front = next;
    }
    tris.sort_unstable();
    let mut origin: Vec<u32> =
        tris.iter().flat_map(|&t| mesh.triangles[t as usize]).chain(lp.iter().copied()).collect();
    origin.sort_unstable();
    origin.dedup();
    let local = |v: u32| origin.binary_search(&v).map_or(0, |k| k as u32);
    let mut work = Mesh {
        positions: origin.iter().map(|&v| mesh.positions[v as usize]).collect(),
        triangles: tris.iter().map(|&t| mesh.triangles[t as usize].map(local)).collect(),
    };
    let lp: Vec<u32> = lp.iter().map(|&v| local(v)).collect();
    let around = (smooth && sharpen).then(|| crate::crease::Around::new(&work));
    let (first_tri, first_vertex) = (work.triangles.len(), work.positions.len());
    // fill triangles run against the loop direction
    let rev: Vec<u32> = lp.iter().rev().copied().collect();
    if !ear_clip(&mut work, &rev) {
        fan_fill(&mut work, &rev);
    }
    if smooth {
        refine_and_fair(&mut work, first_tri, &lp);
        if let Some(around) = &around {
            crate::crease::sharpen_patch(&mut work, around, first_tri, first_vertex, &lp);
        }
    }
    Patch { origin, positions: work.positions.split_off(first_vertex), triangles: work.triangles.split_off(first_tri) }
}

/// Refines the triangles from `first` on (centroid splits until edges are about the loop's mean
/// edge, Delaunay flips) and fairs their interior vertices with the loop held fixed.
fn refine_and_fair(mesh: &mut Mesh, first: usize, lp: &[u32]) {
    let n = lp.len();
    if n < 3 {
        return;
    }
    let target: f64 =
        (0..n).map(|k| (mesh.positions[lp[k] as usize] - mesh.positions[lp[(k + 1) % n] as usize]).norm()).sum::<f64>()
            / n as f64;
    if !(target > 0.0) {
        return;
    }
    let first_vertex = mesh.positions.len();
    let target_area = target * target * 3f64.sqrt() / 4.0;
    for _round in 0..12 {
        let mut split = false;
        let end = mesh.triangles.len();
        for ti in first..end {
            let t = mesh.triangles[ti];
            let (a, b, c) =
                (mesh.positions[t[0] as usize], mesh.positions[t[1] as usize], mesh.positions[t[2] as usize]);
            if tri::area(&a, &b, &c) > 1.0 * target_area {
                let m = mesh.positions.len() as u32;
                mesh.positions.push(Point3::from((a.coords + b.coords + c.coords) / 3.0));
                mesh.triangles[ti] = [t[0], t[1], m];
                mesh.triangles.push([t[1], t[2], m]);
                mesh.triangles.push([t[2], t[0], m]);
                split = true;
            }
        }
        flip_patch(mesh, first);
        if !split {
            break;
        }
        if mesh.triangles.len() - first > 200_000 {
            break;
        }
    }
    flip_patch(mesh, first);
    fair_patch(mesh, first, first_vertex);
}

/// Delaunay-style edge flips among patch triangles (edges shared by two patch triangles).
fn flip_patch(mesh: &mut Mesh, first: usize) {
    for _pass in 0..8 {
        let mut edge: HashMap<(u32, u32), (usize, usize)> = HashMap::new();
        for ti in first..mesh.triangles.len() {
            let t = mesh.triangles[ti];
            for k in 0..3 {
                edge.insert((t[k], t[(k + 1) % 3]), (ti, k));
            }
        }
        let mut flipped = false;
        let mut done: HashSet<usize> = HashSet::new();
        let mut keys: Vec<(u32, u32)> = edge.keys().copied().collect();
        keys.sort_unstable();
        for (a, b) in keys {
            let (Some(&(t0, k0)), Some(&(t1, k1))) = (edge.get(&(a, b)), edge.get(&(b, a))) else { continue };
            if done.contains(&t0) || done.contains(&t1) {
                continue;
            }
            let c = mesh.triangles[t0][(k0 + 2) % 3];
            let d = mesh.triangles[t1][(k1 + 2) % 3];
            if c == d {
                continue;
            }
            let (pa, pb, pc, pd) = (
                mesh.positions[a as usize],
                mesh.positions[b as usize],
                mesh.positions[c as usize],
                mesh.positions[d as usize],
            );
            let ang = |o: Point3, u: Point3, v: Point3| {
                let (x, y) = (u - o, v - o);
                x.angle(&y)
            };
            if ang(pc, pa, pb) + ang(pd, pa, pb) <= std::f64::consts::PI + 1e-9 {
                continue;
            }
            // the new triangles must keep the orientation of the old pair
            let n_old = tri::cross(&pa, &pb, &pc) + tri::cross(&pb, &pa, &pd);
            let (n1, n2) = (tri::cross(&pc, &pd, &pb), tri::cross(&pd, &pc, &pa));
            if n1.dot(&n_old) <= 0.0 || n2.dot(&n_old) <= 0.0 {
                continue;
            }
            // do not create an edge that already exists
            if edge.contains_key(&(c, d)) || edge.contains_key(&(d, c)) {
                continue;
            }
            mesh.triangles[t0] = [c, d, b];
            mesh.triangles[t1] = [d, c, a];
            done.insert(t0);
            done.insert(t1);
            flipped = true;
        }
        if !flipped {
            break;
        }
    }
}

/// One row of the fairing operator (see [`fair_patch`]).
struct RowOp {
    len: usize,
    cols: Vec<usize>,
    own: Option<usize>,
}

/// Fairs the vertices from `first_vertex` on: membrane (umbrella) iterations, then thin-plate
/// (umbrella of the umbrella) iterations, the rest of the mesh fixed.
fn fair_patch(mesh: &mut Mesh, first_tri: usize, first_vertex: usize) {
    let nv = mesh.positions.len();
    if first_vertex >= nv {
        return;
    }
    // rings of patch vertices and of the boundary vertices they touch (from patch and from the
    // original triangles around the boundary, so the thin-plate step sees the outer surface)
    let mut ring: HashMap<u32, Vec<u32>> = HashMap::new();
    let mut add = |a: u32, b: u32| {
        ring.entry(a).or_default().push(b);
    };
    let mut touched: HashSet<u32> = HashSet::new();
    for t in &mesh.triangles[first_tri..] {
        for &v in t {
            touched.insert(v);
        }
    }
    for t in &mesh.triangles {
        if t.iter().any(|v| touched.contains(v)) {
            for k in 0..3 {
                add(t[k], t[(k + 1) % 3]);
                add(t[(k + 1) % 3], t[k]);
            }
        }
    }
    for r in ring.values_mut() {
        r.sort_unstable();
        r.dedup();
    }
    let free: Vec<u32> = (first_vertex as u32..nv as u32).collect();
    let lap = |pos: &[Point3], v: u32| -> Option<Vec3> {
        let r = ring.get(&v)?;
        if r.is_empty() {
            return None;
        }
        let c = r.iter().fold(Vec3::zeros(), |a, &j| a + pos[j as usize].coords) / r.len() as f64;
        Some(c - pos[v as usize].coords)
    };
    for _ in 0..200 {
        for &v in &free {
            if let Some(l) = lap(&mesh.positions, v) {
                mesh.positions[v as usize] += l;
            }
        }
    }
    // thin plate: minimise Σ_touched |Δx|² over the free vertices (normal equations, CG per axis)
    let rows: Vec<u32> = {
        let mut r: Vec<u32> = touched.iter().copied().collect();
        r.sort_unstable();
        r
    };
    // the free vertices are the last ones, so a vertex's column is its offset from the first
    let col_of = |v: u32| (v as usize >= first_vertex).then(|| v as usize - first_vertex);
    let nf = free.len();
    let empty: Vec<u32> = Vec::new();
    let ring_of = |v: u32| ring.get(&v).unwrap_or(&empty);
    // every row of M once: ring size, columns of its free ring members, its own column
    let ops: Vec<RowOp> = rows
        .iter()
        .map(|&v| {
            let r = ring_of(v);
            RowOp { len: r.len(), cols: r.iter().filter_map(|&j| col_of(j)).collect(), own: col_of(v) }
        })
        .collect();
    // M x (rows) for free values x, fixed ones taken as zero
    let m_apply = |x: &[f64]| -> Vec<f64> {
        ops.iter()
            .map(|o| {
                if o.len == 0 {
                    return 0.0;
                }
                let avg = o.cols.iter().map(|&k| x[k]).sum::<f64>() / o.len as f64;
                avg - o.own.map_or(0.0, |k| x[k])
            })
            .collect()
    };
    let mt_apply = |y: &[f64]| -> Vec<f64> {
        let mut out = vec![0.0; nf];
        for (ri, o) in ops.iter().enumerate() {
            if o.len == 0 {
                continue;
            }
            let w = y[ri] / o.len as f64;
            for &k in &o.cols {
                out[k] += w;
            }
            if let Some(k) = o.own {
                out[k] -= y[ri];
            }
        }
        out
    };
    for axis in 0..3 {
        // constant part: Δ of the fixed vertices alone
        let fixed_lap: Vec<f64> = rows
            .iter()
            .map(|&v| {
                let r = ring_of(v);
                if r.is_empty() {
                    return 0.0;
                }
                let avg =
                    r.iter().filter(|&&j| col_of(j).is_none()).map(|&j| mesh.positions[j as usize][axis]).sum::<f64>()
                        / r.len() as f64;
                avg - if col_of(v).is_some() { 0.0 } else { mesh.positions[v as usize][axis] }
            })
            .collect();
        let b: Vec<f64> = mt_apply(&fixed_lap).iter().map(|x| -x).collect();
        let mut x: Vec<f64> = free.iter().map(|&v| mesh.positions[v as usize][axis]).collect();
        let ax = mt_apply(&m_apply(&x));
        let mut r: Vec<f64> = b.iter().zip(&ax).map(|(b, a)| b - a).collect();
        let mut p = r.clone();
        let mut rr: f64 = r.iter().map(|v| v * v).sum();
        let tol = rr * 1e-20;
        for _ in 0..(4 * nf).clamp(50, 4000) {
            if rr <= tol || rr == 0.0 {
                break;
            }
            let ap = mt_apply(&m_apply(&p));
            let pap: f64 = p.iter().zip(&ap).map(|(a, b)| a * b).sum();
            if !(pap > 0.0) {
                break;
            }
            let alpha = rr / pap;
            for k in 0..nf {
                x[k] += alpha * p[k];
                r[k] -= alpha * ap[k];
            }
            let rr2: f64 = r.iter().map(|v| v * v).sum();
            let beta = rr2 / rr;
            rr = rr2;
            for k in 0..nf {
                p[k] = r[k] + beta * p[k];
            }
        }
        for (k, &v) in free.iter().enumerate() {
            if x[k].is_finite() {
                mesh.positions[v as usize][axis] = x[k];
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rng, synth};

    fn in_pool<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
        rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
    }

    /// A sphere with holes punched around `count` well-spread vertices.
    fn punched(count: usize) -> Mesh {
        let m = synth::sphere(Point3::origin(), 10.0, 120, 60);
        let mut rng = Rng::new(3);
        let seeds: Vec<Point3> = (0..count).map(|_| m.positions[rng.index(m.positions.len())]).collect();
        let triangles = m
            .triangles
            .iter()
            .copied()
            .filter(|t| {
                let c = (m.positions[t[0] as usize].coords
                    + m.positions[t[1] as usize].coords
                    + m.positions[t[2] as usize].coords)
                    / 3.0;
                seeds.iter().all(|s| (c - s.coords).norm() > 1.2)
            })
            .collect();
        Mesh { positions: m.positions, triangles }
    }

    #[test]
    fn filled_holes_are_the_same_on_one_and_many_threads() {
        let holed = punched(12);
        let before = boundary_loops(&holed).len();
        assert!(before >= 6, "{before}");
        for smooth in [false, true] {
            let run = |threads: usize| {
                in_pool(threads, || {
                    let mut m = holed.clone();
                    let rep = fill_holes(&mut m, 4000, smooth);
                    (rep, m)
                })
            };
            let ((r1, m1), (r8, m8)) = (run(1), run(8));
            assert_eq!(r1, r8);
            assert_eq!(m1, m8, "smooth = {smooth}");
            assert_eq!(r1.filled, before);
            assert!(boundary_loops(&m1).is_empty(), "holes left");
            assert!(m1.validate().is_ok());
        }
    }

    /// A disc of 4 500 boundary edges (past the former limit of 4 000) is filled flat.
    #[test]
    fn a_hole_beyond_four_thousand_edges_is_filled() {
        let n = 4500;
        let mut positions: Vec<Point3> = (0..n)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / n as f64;
                let r = 20.0 + (9.0 * a).sin();
                Point3::new(r * a.cos(), r * a.sin(), 0.0)
            })
            .collect();
        positions.push(Point3::origin());
        let triangles = (0..n as u32).map(|k| [n as u32, k, (k + 1) % n as u32]).collect();
        let mut m = Mesh { positions, triangles };
        let loops = boundary_loops(&m);
        assert_eq!(loops.iter().map(Vec::len).collect::<Vec<_>>(), vec![n]);
        let rep = fill_holes(&mut m, 5000, false);
        assert_eq!((rep.filled, rep.skipped, rep.added_triangles), (1, 0, n - 2));
        assert!(boundary_loops(&m).is_empty());
        assert_eq!(fill_holes(&mut Mesh::default(), 5000, false), FillReport::default());
    }
}
