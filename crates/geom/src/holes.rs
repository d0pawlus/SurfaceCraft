//! Hole filling: boundary loops triangulated by ear clipping in the loop's mean plane; the smooth
//! variant refines the patch (centroid splits and Delaunay flips) and fairs it with the
//! surrounding ring held fixed (membrane, then thin-plate iterations).
use crate::{Mesh, Point3, Vec3, tri};
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

/// Fills boundary loops with at most `max_edges` edges. `smooth` refines and fairs the patch so
/// it follows the surrounding curvature, and keeps the sharp edges and corners the hole lies
/// over; otherwise the patch is flat (ear clipping only).
pub fn fill_holes(mesh: &mut Mesh, max_edges: usize, smooth: bool) -> FillReport {
    fill_holes_with(mesh, max_edges, smooth, smooth)
}

/// [`fill_holes`], with the keeping of sharp edges (`sharpen`, only with `smooth`) on its own
/// switch.
pub(crate) fn fill_holes_with(mesh: &mut Mesh, max_edges: usize, smooth: bool, sharpen: bool) -> FillReport {
    let mut rep = FillReport::default();
    let max_edges = max_edges.clamp(3, 4000);
    let loops = boundary_loops(mesh);
    // the surface around every hole is read from the mesh as it is before any is filled
    let around =
        (smooth && sharpen && loops.iter().any(|lp| lp.len() <= max_edges)).then(|| crate::crease::Around::new(mesh));
    for lp in loops {
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
            if let Some(around) = &around {
                crate::crease::sharpen_patch(mesh, around, first_new, first_vertex, &lp);
            }
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
