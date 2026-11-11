//! Mesh editing for real scans (P1): components, outliers, holes, decimation, smoothing.
//!
//! Everything here is mesh-side computation (design doc §5). Each function validates the mesh
//! first and does nothing on an invalid one, so later indexing stays in range.
//!
//! - small components: connectivity through shared vertices;
//! - outliers: spikes, i.e. vertices far off the plane of their one-ring compared with the ring's
//!   size (creases and corners stay well below the threshold);
//! - holes: boundary loops triangulated by ear clipping in the loop's mean plane; the smooth
//!   variant refines the patch (centroid splits + Delaunay flips) and fairs it with the
//!   surrounding ring held fixed (membrane, then thin-plate iterations);
//! - decimation: quadric error metric edge collapse (Garland & Heckbert 1997) with boundary and
//!   feature-edge constraint planes, link-condition and normal-flip checks;
//! - smoothing: Taubin λ|μ (1995), boundary and sharp-edge vertices optionally fixed.
use crate::{Bvh, Mesh, Point3, Vec3, tri};
use nalgebra::{Matrix3, Matrix4, Vector4};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// Drops vertices no triangle uses; returns how many were dropped.
pub fn compact(mesh: &mut Mesh) -> usize {
    if mesh.validate().is_err() {
        return 0;
    }
    let mut used = vec![u32::MAX; mesh.positions.len()];
    let mut out = Vec::with_capacity(mesh.positions.len());
    for t in &mut mesh.triangles {
        for v in t.iter_mut() {
            let i = *v as usize;
            if used[i] == u32::MAX {
                used[i] = out.len() as u32;
                out.push(mesh.positions[i]);
            }
            *v = used[i];
        }
    }
    let removed = mesh.positions.len() - out.len();
    mesh.positions = out;
    removed
}

/// Connected components (through shared vertices) as lists of triangle indices, largest first.
#[must_use]
pub fn components(mesh: &Mesh) -> Vec<Vec<u32>> {
    if mesh.validate().is_err() {
        return Vec::new();
    }
    // union-find over vertices
    let mut parent: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    fn find(p: &mut [u32], mut x: u32) -> u32 {
        while p[x as usize] != x {
            let up = p[p[x as usize] as usize];
            p[x as usize] = up;
            x = up;
        }
        x
    }
    for t in &mesh.triangles {
        let a = find(&mut parent, t[0]);
        for &v in &t[1..] {
            let b = find(&mut parent, v);
            if a != b {
                parent[b as usize] = a;
            }
        }
    }
    let mut groups: HashMap<u32, Vec<u32>> = HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        let r = find(&mut parent, t[0]);
        groups.entry(r).or_default().push(i as u32);
    }
    let mut v: Vec<Vec<u32>> = groups.into_values().collect();
    v.sort_by(|a, b| b.len().cmp(&a.len()).then(a.first().cmp(&b.first())));
    v
}

/// Removes components with fewer than `max(min_triangles, min_fraction × largest)` triangles.
/// Returns the number of triangles removed.
pub fn remove_small_components(mesh: &mut Mesh, min_fraction: f64, min_triangles: usize) -> usize {
    let comps = components(mesh);
    let Some(largest) = comps.first().map(Vec::len) else { return 0 };
    let f = if min_fraction.is_finite() { min_fraction.clamp(0.0, 1.0) } else { 0.0 };
    let keep_at = ((largest as f64 * f).ceil() as usize).max(min_triangles).min(largest);
    let mut keep = vec![false; mesh.triangles.len()];
    for c in comps.iter().filter(|c| c.len() >= keep_at) {
        for &t in c {
            keep[t as usize] = true;
        }
    }
    let before = mesh.triangles.len();
    let mut k = keep.iter();
    mesh.triangles.retain(|_| k.next().copied().unwrap_or(false));
    compact(mesh);
    before - mesh.triangles.len()
}

/// Vertex → neighbouring vertices (one ring, unordered, no duplicates).
pub(crate) fn vertex_rings(mesh: &Mesh) -> Vec<Vec<u32>> {
    let mut ring: Vec<Vec<u32>> = vec![Vec::new(); mesh.positions.len()];
    for t in &mesh.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            ring[a as usize].push(b);
            ring[b as usize].push(a);
        }
    }
    for r in &mut ring {
        r.sort_unstable();
        r.dedup();
    }
    ring
}

/// A ring point this many times the median distance from the ring's middle is left out of the
/// ring's plane (it may be a spike itself).
const RING_STRAY_FACTOR: f64 = 2.0;
/// The points a plane is fitted to are spread enough when their second principal extent is at
/// least this share of the first (points nearly in a line give no plane).
const RING_MIN_SPREAD: f64 = 0.2;

/// Least-squares plane of `pts` as (centroid, unit normal, mean distance to the centroid, ratio
/// of the second to the first principal extent).
fn ring_plane(pts: &[Point3]) -> Option<(Vec3, Vec3, f64, f64)> {
    if pts.len() < 3 {
        return None;
    }
    let c = pts.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / pts.len() as f64;
    let mut cov = Matrix3::zeros();
    let mut radius = 0.0;
    for p in pts {
        let d = p.coords - c;
        cov += d * d.transpose();
        radius += d.norm();
    }
    radius /= pts.len() as f64;
    if !(radius > 0.0) {
        return None;
    }
    let e = cov.symmetric_eigen();
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| e.eigenvalues[a].total_cmp(&e.eigenvalues[b]));
    let [least, mid, most] = order;
    let spread = (e.eigenvalues[mid].max(0.0) / e.eigenvalues[most].max(f64::MIN_POSITIVE)).sqrt();
    Some((c, e.eigenvectors.column(least).into_owned(), radius, spread))
}

/// The plane of a one ring that a spike among the ring's own points cannot tilt: ring points
/// more than [`RING_STRAY_FACTOR`] × the median distance from the ring's middle (the median of
/// each coordinate) are left out, as long as the rest still spans a plane (a ring of thin
/// triangles has far points that are no spikes, and the near ones lie almost in a line).
fn steady_ring_plane(pts: &[Point3]) -> Option<(Vec3, Vec3, f64)> {
    let median = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v.get(v.len() / 2).copied().unwrap_or(0.0)
    };
    let mid = Point3::new(
        median(pts.iter().map(|p| p.x).collect()),
        median(pts.iter().map(|p| p.y).collect()),
        median(pts.iter().map(|p| p.z).collect()),
    );
    let reach = RING_STRAY_FACTOR * median(pts.iter().map(|p| (p - mid).norm()).collect());
    let kept: Vec<Point3> = pts.iter().filter(|p| (*p - mid).norm() <= reach).copied().collect();
    let steady = if kept.len() >= 3 && kept.len() < pts.len() {
        ring_plane(&kept).filter(|(_, _, _, spread)| *spread >= RING_MIN_SPREAD)
    } else {
        None
    };
    // the radius stays that of the whole ring about the plane's centre: the far points make the
    // test stricter, they only may not tilt the plane
    match steady {
        Some((c, n, _, _)) => {
            let radius = pts.iter().map(|p| (p.coords - c).norm()).sum::<f64>() / pts.len() as f64;
            Some((c, n, radius))
        }
        None => ring_plane(pts).map(|(c, n, radius, _)| (c, n, radius)),
    }
}

/// Removes spikes: vertices farther from the least-squares plane of their one ring than
/// `ratio` × the ring's radius. Far points of the ring are left out of the plane, so spikes next
/// to each other do not hide one another. Triangles touching them are removed (the holes
/// left behind can be filled with [`fill_holes`]). Returns the number of triangles removed.
pub fn remove_outliers(mesh: &mut Mesh, ratio: f64) -> usize {
    if mesh.validate().is_err() {
        return 0;
    }
    let ratio = if ratio.is_finite() && ratio > 0.0 { ratio } else { 1.5 };
    let ring = vertex_rings(mesh);
    let positions = &mesh.positions;
    let bad: Vec<bool> = crate::par::map(ring.len(), |i| {
        let Some(r) = ring.get(i) else { return false };
        let pts: Vec<Point3> = r.iter().filter_map(|&j| positions.get(j as usize).copied()).collect();
        let (Some((c, n, radius)), Some(p)) = (steady_ring_plane(&pts), positions.get(i)) else { return false };
        (p.coords - c).dot(&n).abs() > ratio * radius
    });
    let before = mesh.triangles.len();
    mesh.triangles.retain(|t| !t.iter().any(|&v| bad[v as usize]));
    compact(mesh);
    before - mesh.triangles.len()
}

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

/// Fills boundary loops with at most `max_edges` edges. `smooth` refines and fairs the patch so
/// it follows the surrounding curvature; otherwise the patch is flat (ear clipping only).
pub fn fill_holes(mesh: &mut Mesh, max_edges: usize, smooth: bool) -> FillReport {
    let mut rep = FillReport::default();
    let max_edges = max_edges.clamp(3, 4000);
    for lp in boundary_loops(mesh) {
        if lp.len() > max_edges {
            rep.skipped += 1;
            continue;
        }
        // fill triangles run against the loop direction
        let rev: Vec<u32> = lp.iter().rev().copied().collect();
        let first_new = mesh.triangles.len();
        let first_vertex = mesh.positions.len();
        let ok = ear_clip(mesh, &rev);
        if !ok {
            fan_fill(mesh, &rev);
        }
        if smooth {
            refine_and_fair(mesh, first_new, &lp);
            crate::crease::sharpen_patch(mesh, first_new, first_vertex, &lp);
        }
        rep.filled += 1;
        rep.added_triangles += mesh.triangles.len() - first_new;
    }
    rep
}

fn loop_frame(mesh: &Mesh, lp: &[u32]) -> (Point3, Vec3, Vec3, Vec3) {
    // Newell normal and centroid
    let mut n = Vec3::zeros();
    let mut c = Vec3::zeros();
    for (k, &a) in lp.iter().enumerate() {
        let p = mesh.positions[a as usize];
        let q = mesh.positions[lp[(k + 1) % lp.len()] as usize];
        n += Vec3::new((p.y - q.y) * (p.z + q.z), (p.z - q.z) * (p.x + q.x), (p.x - q.x) * (p.y + q.y));
        c += p.coords;
    }
    let n = crate::normalize(n).unwrap_or_else(Vec3::z);
    let x = crate::any_perpendicular(&n);
    let y = n.cross(&x);
    (Point3::from(c / lp.len() as f64), x, y, n)
}

/// Ear clipping in the loop's mean plane; false when the projected polygon is not simple enough.
fn ear_clip(mesh: &mut Mesh, lp: &[u32]) -> bool {
    let (o, x, y, _) = loop_frame(mesh, lp);
    let p2: Vec<[f64; 2]> = lp
        .iter()
        .map(|&v| {
            let d = mesh.positions[v as usize] - o;
            [d.dot(&x), d.dot(&y)]
        })
        .collect();
    let mut idx: Vec<usize> = (0..lp.len()).collect();
    // orientation of the projected polygon
    let area2: f64 = (0..p2.len())
        .map(|k| {
            let (a, b) = (p2[k], p2[(k + 1) % p2.len()]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum();
    let sign = if area2 >= 0.0 { 1.0 } else { -1.0 };
    let cross = |a: [f64; 2], b: [f64; 2], c: [f64; 2]| (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    let mut out: Vec<[u32; 3]> = Vec::with_capacity(lp.len());
    let mut guard = 0;
    while idx.len() > 3 {
        guard += 1;
        if guard > lp.len() * lp.len() + 10 {
            return false;
        }
        let m = idx.len();
        let mut best: Option<(usize, f64)> = None;
        for k in 0..m {
            let (ia, ib, ic) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
            let (a, b, c) = (p2[ia], p2[ib], p2[ic]);
            let cr = cross(a, b, c) * sign;
            if cr <= 1e-15 {
                continue;
            }
            let inside = idx.iter().any(|&j| {
                if j == ia || j == ib || j == ic {
                    return false;
                }
                let p = p2[j];
                cross(a, b, p) * sign > 0.0 && cross(b, c, p) * sign > 0.0 && cross(c, a, p) * sign > 0.0
            });
            if inside {
                continue;
            }
            // prefer well-shaped ears: area over squared longest edge
            let l = |u: [f64; 2], v: [f64; 2]| (u[0] - v[0]).powi(2) + (u[1] - v[1]).powi(2);
            let q = cr / l(a, b).max(l(b, c)).max(l(c, a)).max(1e-300);
            if best.is_none_or(|(_, bq)| q > bq) {
                best = Some((k, q));
            }
        }
        let Some((k, _)) = best else { return false };
        let (ia, ib, ic) = (idx[(k + m - 1) % m], idx[k], idx[(k + 1) % m]);
        out.push([lp[ia], lp[ib], lp[ic]]);
        idx.remove(k);
    }
    if let [a, b, c] = idx.as_slice() {
        out.push([lp[*a], lp[*b], lp[*c]]);
    }
    mesh.triangles.extend(out);
    true
}

/// Fan around a new centre vertex (fallback for loops that do not project to a simple polygon).
fn fan_fill(mesh: &mut Mesh, lp: &[u32]) {
    let (o, _, _, _) = loop_frame(mesh, lp);
    let c = mesh.positions.len() as u32;
    mesh.positions.push(o);
    for k in 0..lp.len() {
        mesh.triangles.push([lp[k], lp[(k + 1) % lp.len()], c]);
    }
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
    let col: HashMap<u32, usize> = free.iter().enumerate().map(|(k, &v)| (v, k)).collect();
    let nf = free.len();
    let empty: Vec<u32> = Vec::new();
    let ring_of = |v: u32| ring.get(&v).unwrap_or(&empty);
    // M x (rows) for free values x, fixed ones taken as zero
    let m_apply = |x: &[f64]| -> Vec<f64> {
        rows.iter()
            .map(|&v| {
                let r = ring_of(v);
                if r.is_empty() {
                    return 0.0;
                }
                let avg = r.iter().filter_map(|j| col.get(j).map(|&k| x[k])).sum::<f64>() / r.len() as f64;
                avg - col.get(&v).map_or(0.0, |&k| x[k])
            })
            .collect()
    };
    let mt_apply = |y: &[f64]| -> Vec<f64> {
        let mut out = vec![0.0; nf];
        for (ri, &v) in rows.iter().enumerate() {
            let r = ring_of(v);
            if r.is_empty() {
                continue;
            }
            let w = y[ri] / r.len() as f64;
            for j in r {
                if let Some(&k) = col.get(j) {
                    out[k] += w;
                }
            }
            if let Some(&k) = col.get(&v) {
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
                    r.iter().filter(|j| !col.contains_key(j)).map(|&j| mesh.positions[j as usize][axis]).sum::<f64>()
                        / r.len() as f64;
                avg - if col.contains_key(&v) { 0.0 } else { mesh.positions[v as usize][axis] }
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

/// A symmetric 4×4 quadric.
type Quadric = Matrix4<f64>;

fn plane_quadric(n: &Vec3, p: &Point3, w: f64) -> Quadric {
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

/// Decimation settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DecimateOptions {
    /// Stop at this many triangles.
    pub target_triangles: usize,
    /// Never move the surface more than this (mm, approximate quadric distance); `None` = no limit.
    pub max_error: Option<f64>,
    /// Edges with a dihedral angle above this are kept sharp (degrees).
    pub feature_angle_deg: f64,
    /// Keep boundary edges in place.
    pub preserve_boundary: bool,
}

impl Default for DecimateOptions {
    fn default() -> Self {
        Self { target_triangles: 0, max_error: None, feature_angle_deg: 40.0, preserve_boundary: true }
    }
}

/// Quadric edge-collapse decimation; returns the number of triangles removed.
pub fn decimate(mesh: &mut Mesh, opts: &DecimateOptions) -> usize {
    decimate_guarded(mesh, opts, None)
}

/// Measured guard of a collapse: the reduction of [`crate::reduce`] checks every collapse against
/// the original surface instead of trusting the quadric alone (which overestimates the error of noisy
/// data and ignores the sagitta inside a triangle).
pub(crate) struct Guard<'a> {
    /// The original surface.
    pub bvh: &'a Bvh,
    /// Per-vertex error budget (mm), indexed by the vertices of the mesh before the call; a collapse
    /// is allowed only when the measured deviation is within the smaller budget of its two vertices,
    /// and the merged vertex keeps that smaller budget.
    pub budget: &'a mut Vec<f64>,
}

/// Collapses whose quadric error is below this fraction of the budget are not measured.
const GUARD_SKIP: f64 = 0.25;
/// With a guard, a collapse may not grow a vertex above this many neighbours (or above the valence
/// it already has). Flat regions collapse at zero cost, and without the cap they pile up in a few
/// hubs whose rings make every further collapse linear in the ring (minutes on a 100k-triangle block).
const GUARD_VALENCE: usize = 32;

/// [`decimate`] with an optional measured [`Guard`]; without one it is [`decimate`].
pub(crate) fn decimate_guarded(mesh: &mut Mesh, opts: &DecimateOptions, mut guard: Option<Guard<'_>>) -> usize {
    if guard.as_ref().is_some_and(|g| g.budget.len() != mesh.positions.len()) {
        return 0;
    }
    if mesh.validate().is_err() || mesh.triangles.len() <= opts.target_triangles.max(4) {
        return 0;
    }
    let nv = mesh.positions.len();
    let mut tris = std::mem::take(&mut mesh.triangles);
    let before = tris.len();
    let mut pos = std::mem::take(&mut mesh.positions);
    let mut alive_t = vec![true; tris.len()];
    let mut vfaces: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for (i, t) in tris.iter().enumerate() {
        for &v in t {
            vfaces[v as usize].push(i as u32);
        }
    }
    // quadrics
    let mut q = vec![Quadric::zeros(); nv];
    let mut fnormal: Vec<Vec3> = Vec::with_capacity(tris.len());
    for t in &tris {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        let cr = tri::cross(&a, &b, &c);
        let n = crate::normalize(cr).unwrap_or_else(Vec3::zeros);
        fnormal.push(n);
        let k = plane_quadric(&n, &a, 1.0);
        for &v in t {
            q[v as usize] += k;
        }
    }
    // boundary and feature edges get constraint planes
    let cos_feat = opts.feature_angle_deg.clamp(1.0, 179.0).to_radians().cos();
    let mut edge_faces: HashMap<(u32, u32), Vec<u32>> = HashMap::with_capacity(tris.len() * 3 / 2);
    for (i, t) in tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edge_faces.entry((a.min(b), a.max(b))).or_default().push(i as u32);
        }
    }
    let mut locked = vec![false; nv];
    let mut edge_list: Vec<(&(u32, u32), &Vec<u32>)> = edge_faces.iter().collect();
    edge_list.sort_unstable_by_key(|(k, _)| **k);
    for (&(a, b), fs) in edge_list {
        let (pa, pb) = (pos[a as usize], pos[b as usize]);
        let e = pb - pa;
        let len2 = e.norm_squared();
        let boundary = fs.len() == 1;
        let feature = fs.len() == 2 && fnormal[fs[0] as usize].dot(&fnormal[fs[1] as usize]) < cos_feat;
        if fs.len() > 2 {
            locked[a as usize] = true;
            locked[b as usize] = true;
            continue;
        }
        if (boundary && opts.preserve_boundary) || feature {
            for &f in fs {
                let fnn = fnormal[f as usize];
                if let Some(side) = crate::normalize(e.cross(&fnn)) {
                    let k = plane_quadric(&side, &pa, 1000.0 * len2.max(1e-12));
                    q[a as usize] += k;
                    q[b as usize] += k;
                }
            }
        }
    }
    drop(edge_faces);
    let mut stamp = vec![0u32; nv];
    let mut alive_v = vec![true; nv];
    let cost_of = |q: &Quadric, p: &Point3| {
        let v = Vector4::new(p.x, p.y, p.z, 1.0);
        (v.transpose() * q * v)[(0, 0)].max(0.0)
    };
    let best_target = |q: &Quadric, pa: &Point3, pb: &Point3| -> (Point3, f64) {
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
    };
    let mut heap: BinaryHeap<Cand> = BinaryHeap::with_capacity(tris.len() * 3 / 2);
    let push =
        |heap: &mut BinaryHeap<Cand>, a: u32, b: u32, pos: &[Point3], q: &[Quadric], stamp: &[u32], locked: &[bool]| {
            if locked[a as usize] || locked[b as usize] {
                return;
            }
            let qq = q[a as usize] + q[b as usize];
            let (p, c) = best_target(&qq, &pos[a as usize], &pos[b as usize]);
            heap.push(Cand { cost: c, a, b, stamp: (stamp[a as usize], stamp[b as usize]), target: [p.x, p.y, p.z] });
        };
    let mut seen_pair: HashSet<(u32, u32)> = HashSet::with_capacity(tris.len() * 3 / 2);
    for t in &tris {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if seen_pair.insert((a.min(b), a.max(b))) {
                push(&mut heap, a.min(b), a.max(b), &pos, &q, &stamp, &locked);
            }
        }
    }
    let mut live = before;
    let max_cost = opts.max_error.filter(|e| e.is_finite() && *e > 0.0).map(|e| e * e);
    let target = opts.target_triangles;
    while live > target {
        let Some(c) = heap.pop() else { break };
        let (a, b) = (c.a, c.b);
        if !alive_v[a as usize] || !alive_v[b as usize] {
            continue;
        }
        if c.stamp != (stamp[a as usize], stamp[b as usize]) {
            continue;
        }
        if max_cost.is_some_and(|m| c.cost > m) {
            break;
        }
        // faces of a, b; shared faces are the ones removed
        let fa: Vec<u32> = vfaces[a as usize].iter().copied().filter(|&f| alive_t[f as usize]).collect();
        let fb: Vec<u32> = vfaces[b as usize].iter().copied().filter(|&f| alive_t[f as usize]).collect();
        let shared: Vec<u32> = fa.iter().copied().filter(|f| fb.contains(f)).collect();
        if shared.is_empty() || shared.len() > 2 {
            continue;
        }
        // link condition: common neighbours are exactly the opposite corners of shared faces
        let nbr = |fs: &[u32], v: u32| -> HashSet<u32> {
            fs.iter().flat_map(|&f| tris[f as usize]).filter(|&x| x != v).collect()
        };
        let (na, nb) = (nbr(&fa, a), nbr(&fb, b));
        let common = na.intersection(&nb).count();
        if common != shared.len() {
            continue;
        }
        if guard.is_some() && na.len() + nb.len() - common - 2 > GUARD_VALENCE.max(na.len()).max(nb.len()) {
            continue;
        }
        let p = Point3::new(c.target[0], c.target[1], c.target[2]);
        // no flipped or degenerate faces around the new vertex
        let mut ok = true;
        let mut fan: Vec<[Point3; 3]> = Vec::new();
        for &f in fa.iter().chain(fb.iter()) {
            if shared.contains(&f) {
                continue;
            }
            let t = tris[f as usize];
            let old = tri::cross(&pos[t[0] as usize], &pos[t[1] as usize], &pos[t[2] as usize]);
            let corners: Vec<Point3> = t.iter().map(|&v| if v == a || v == b { p } else { pos[v as usize] }).collect();
            let new = tri::cross(&corners[0], &corners[1], &corners[2]);
            fan.push([corners[0], corners[1], corners[2]]);
            if new.dot(&old) <= 0.0 || new.norm() < 1e-3 * old.norm() {
                ok = false;
                break;
            }
            if let (Some(n0), Some(n1)) = (crate::normalize(old), crate::normalize(new))
                && n0.dot(&n1) < 0.2
            {
                ok = false;
                break;
            }
        }
        if !ok {
            continue;
        }
        if let Some(g) = guard.as_ref() {
            let limit = g.budget[a as usize].min(g.budget[b as usize]);
            if c.cost > (GUARD_SKIP * limit).powi(2)
                && !collapse_within(g.bvh, limit, &p, &[pos[a as usize], pos[b as usize]], &fan)
            {
                continue;
            }
        }
        // collapse b into a
        for &f in &shared {
            alive_t[f as usize] = false;
            live -= 1;
        }
        for &f in &fb {
            if !alive_t[f as usize] {
                continue;
            }
            for v in tris[f as usize].iter_mut() {
                if *v == b {
                    *v = a;
                }
            }
            vfaces[a as usize].push(f);
        }
        vfaces[b as usize].clear();
        vfaces[a as usize].retain(|&f| alive_t[f as usize]);
        alive_v[b as usize] = false;
        if let Some(g) = guard.as_mut() {
            g.budget[a as usize] = g.budget[a as usize].min(g.budget[b as usize]);
        }
        pos[a as usize] = p;
        let qb = q[b as usize];
        q[a as usize] += qb;
        stamp[a as usize] = stamp[a as usize].wrapping_add(1);
        let mut nbs: Vec<u32> = vfaces[a as usize].iter().flat_map(|&f| tris[f as usize]).filter(|&x| x != a).collect();
        nbs.sort_unstable();
        nbs.dedup();
        for &o in &nbs {
            stamp[o as usize] = stamp[o as usize].wrapping_add(1);
        }
        // re-queue every edge around the neighbours (their stamps changed)
        seen_pair.clear();
        for &o in nbs.iter().chain(std::iter::once(&a)) {
            for &f in &vfaces[o as usize] {
                let t = tris[f as usize];
                for k in 0..3 {
                    let (x, y) = (t[k], t[(k + 1) % 3]);
                    let key = (x.min(y), x.max(y));
                    if seen_pair.insert(key) {
                        push(&mut heap, key.0, key.1, &pos, &q, &stamp, &locked);
                    }
                }
            }
        }
    }
    let mut k = alive_t.iter();
    tris.retain(|_| k.next().copied().unwrap_or(false));
    mesh.positions = pos;
    mesh.triangles = tris;
    compact(mesh);
    before - mesh.triangles.len()
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

/// Taubin smoothing; `fix_boundary` keeps boundary vertices, `feature_angle_deg` keeps vertices on
/// edges sharper than that (None = smooth everything). Returns the largest displacement (mm).
pub fn smooth(mesh: &mut Mesh, iterations: usize, fix_boundary: bool, feature_angle_deg: Option<f64>) -> f64 {
    if mesh.validate().is_err() {
        return 0.0;
    }
    let ring = vertex_rings(mesh);
    let mut fixed = vec![false; mesh.positions.len()];
    let nb = mesh.triangle_neighbours();
    let cos = feature_angle_deg.map(|a| a.clamp(1.0, 179.0).to_radians().cos());
    for (ti, t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            let o = nb[ti][k];
            let edge = [t[k], t[(k + 1) % 3]];
            let mark = if o == u32::MAX {
                fix_boundary
            } else if let Some(c) = cos {
                match (mesh.face_normal(ti), mesh.face_normal(o as usize)) {
                    (Some(a), Some(b)) => a.dot(&b) < c,
                    _ => false,
                }
            } else {
                false
            };
            if mark {
                for v in edge {
                    fixed[v as usize] = true;
                }
            }
        }
    }
    let start = mesh.positions.clone();
    let step = |pos: &mut Vec<Point3>, f: f64| {
        let old = pos.clone();
        for (i, r) in ring.iter().enumerate() {
            if fixed[i] || r.is_empty() {
                continue;
            }
            let c = r.iter().fold(Vec3::zeros(), |a, &j| a + old[j as usize].coords) / r.len() as f64;
            pos[i] = old[i] + (c - old[i].coords) * f;
        }
    };
    for _ in 0..iterations.min(1000) {
        step(&mut mesh.positions, 0.5);
        step(&mut mesh.positions, -0.53);
    }
    mesh.positions.iter().zip(&start).map(|(a, b)| (a - b).norm()).fold(0.0, f64::max)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rng, synth};

    #[test]
    fn small_components_and_compact() {
        let mut m = synth::sphere(Point3::origin(), 10.0, 32, 16);
        m.weld(1e-9);
        let n = m.triangle_count();
        let mut debris = synth::plane(Point3::new(50.0, 0.0, 0.0), Vec3::x(), Vec3::y(), 1.0, 1.0, 2, 2);
        debris.weld(1e-9);
        m.append(&debris);
        assert_eq!(components(&m).len(), 2);
        assert_eq!(remove_small_components(&mut m, 0.01, 10), 8);
        assert_eq!(m.triangle_count(), n);
        assert!(m.positions.iter().all(|p| p.x < 20.0));
    }

    #[test]
    fn spikes_go_creases_stay() {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 8);
        m.weld(1e-9);
        let n = m.triangle_count();
        // creases and corners are not outliers
        assert_eq!(remove_outliers(&mut m.clone(), 1.5), 0);
        // a spike in the middle of a face is
        let top = m
            .positions
            .iter()
            .position(|p| (p.z - 20.0).abs() < 1e-9 && (p.x - 10.0).abs() < 1e-9 && (p.y - 10.0).abs() < 1e-9);
        let i = top.unwrap();
        m.positions[i].z += 15.0;
        let removed = remove_outliers(&mut m, 1.5);
        assert!(removed > 0 && removed <= 8, "{removed}");
        assert_eq!(m.triangle_count(), n - removed);
        // the hole it leaves is one loop, filled flat back to a closed box
        let loops = boundary_loops(&m);
        assert_eq!(loops.len(), 1);
        let r = fill_holes(&mut m, 100, false);
        assert_eq!(r.filled, 1);
        assert!(boundary_loops(&m).is_empty());
        assert!((m.area() - 6.0 * 400.0).abs() < 1e-6, "{}", m.area());
    }

    /// Two spikes next to each other are in each other's ring; neither may hide the other.
    #[test]
    fn neighbouring_spikes_both_go() {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 8);
        m.weld(1e-9);
        for (x, lift) in [(10.0, 10.4), (12.5, 7.8)] {
            let i = m
                .positions
                .iter()
                .position(|p| (p.z - 20.0).abs() < 1e-9 && (p.x - x).abs() < 1e-9 && (p.y - 10.0).abs() < 1e-9)
                .unwrap();
            m.positions[i].z += lift;
        }
        assert!(remove_outliers(&mut m, 1.5) > 0);
        let high = m.positions.iter().filter(|p| p.z > 20.5).count();
        assert_eq!(high, 0, "{high} spike vertices are left");
        // creases and corners of the clean box are still not outliers
        let mut clean = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 8);
        clean.weld(1e-9);
        assert_eq!(remove_outliers(&mut clean, 1.5), 0);
    }

    #[test]
    fn smooth_fill_follows_the_sphere() {
        let r = 20.0;
        let mut m = synth::sphere(Point3::origin(), r, 64, 32);
        m.weld(1e-9);
        // cut a cap out near the equator
        let keep: Vec<u32> = (0..m.triangle_count() as u32)
            .filter(|&t| {
                let c = m.face_centroid(t as usize).unwrap();
                !(c.x > 15.0)
            })
            .collect();
        let mut cut = m.submesh(&keep);
        let loops = boundary_loops(&cut);
        assert_eq!(loops.len(), 1);
        let mut flat = cut.clone();
        // deterministic: the same input gives the same fill
        let (mut x, mut y) = (cut.clone(), cut.clone());
        fill_holes(&mut x, 1000, true);
        fill_holes(&mut y, 1000, true);
        assert_eq!(x, y);
        fill_holes(&mut flat, 1000, false);
        let rep = fill_holes(&mut cut, 1000, true);
        assert_eq!(rep.filled, 1);
        assert!(boundary_loops(&cut).is_empty());
        let err = |mm: &Mesh| {
            (0..mm.triangle_count())
                .filter_map(|t| mm.face_centroid(t))
                .map(|p| (p.coords.norm() - r).abs())
                .fold(0.0, f64::max)
        };
        // a flat cap misses the sphere by its sagitta (≈ 5 mm); the faired cap follows it
        assert!(err(&flat) > 3.0, "{}", err(&flat));
        // a thin plate over the hole is a paraboloid-like cap, well inside the flat error
        assert!(err(&cut) < 0.35 * err(&flat), "smooth fill error {} vs flat {}", err(&cut), err(&flat));
    }

    #[test]
    fn decimation_keeps_shape_and_creases() {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(20.0, 10.0, 5.0), 24);
        m.weld(1e-9);
        let before = m.triangle_count();
        let removed = decimate(&mut m, &DecimateOptions { target_triangles: 200, ..Default::default() });
        assert!(removed > 0);
        assert!(m.triangle_count() <= before / 4, "{}", m.triangle_count());
        // flat faces collapse freely; the box stays a box (every vertex on its surface, corners kept)
        for p in &m.positions {
            let d =
                [p.x, 20.0 - p.x, p.y, 10.0 - p.y, p.z, 5.0 - p.z].into_iter().map(f64::abs).fold(f64::MAX, f64::min);
            assert!(d < 1e-6, "{p}");
        }
        assert!((m.area() - 2.0 * (200.0 + 100.0 + 50.0)).abs() < 1e-6);
        assert!(boundary_loops(&m).is_empty());
        // a noisy sphere: 10 % of the triangles stay within 0.3 mm of the radius
        let mut s = synth::sphere(Point3::origin(), 20.0, 128, 64);
        s.weld(1e-9);
        let n = s.triangle_count();
        decimate(&mut s, &DecimateOptions { target_triangles: n / 10, ..Default::default() });
        assert!(s.triangle_count() <= n / 10 + 2);
        let err = s.positions.iter().map(|p| (p.coords.norm() - 20.0).abs()).fold(0.0, f64::max);
        assert!(err < 0.3, "{err}");
    }

    #[test]
    fn taubin_reduces_noise_without_shrinking() {
        let mut m = synth::sphere(Point3::origin(), 20.0, 96, 48);
        m.weld(1e-9);
        synth::add_noise(&mut m, 0.1, &mut Rng::new(3));
        let err = |mm: &Mesh| {
            let v: Vec<f64> = mm.positions.iter().map(|p| p.coords.norm() - 20.0).collect();
            let mean = v.iter().sum::<f64>() / v.len() as f64;
            let rms = (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt();
            (mean, rms)
        };
        let (_, rms0) = err(&m);
        smooth(&mut m, 10, true, None);
        let (mean, rms1) = err(&m);
        assert!(rms1 < 0.5 * rms0, "{rms0} → {rms1}");
        assert!(mean.abs() < 0.05, "shrinkage {mean}");
    }

    #[test]
    fn invalid_meshes_are_left_alone() {
        let mut m = Mesh { positions: vec![Point3::origin()], triangles: vec![[0, 1, 7]] };
        assert_eq!(compact(&mut m), 0);
        assert!(components(&m).is_empty());
        assert_eq!(remove_outliers(&mut m, 1.5), 0);
        assert_eq!(fill_holes(&mut m, 10, true), FillReport::default());
        assert_eq!(decimate(&mut m, &DecimateOptions::default()), 0);
        assert_eq!(smooth(&mut m, 3, true, Some(30.0)), 0.0);
        assert_eq!(m.triangles, vec![[0, 1, 7]]);
    }
}
