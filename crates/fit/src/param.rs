//! Flattening of a disc-like mesh region to the plane: a (u, v) for every vertex, to start a
//! surface fit from (`surface.patch`).
//!
//! The boundary loop is laid on a convex polygon by chord length, the interior follows from the
//! mean-value weights of Floater (a linear system with one row per interior vertex, solved by
//! BiCGSTAB on the adjacency lists), then local/global steps of "as rigid as possible" (Liu, Zhang,
//! Xu, Gotsman, Gortler 2008) relax the map towards an isometry with a free boundary. The map is
//! only ever replaced by one with no more reversed triangles than the one before it, so a region
//! that flattens without folds never comes back folded.
//!
//! Mesh-side computation only; no dense matrix is formed (the solves run on adjacency lists).
use std::f64::consts::TAU;
use surfacecraft_geom::{Mesh, Vec3};

/// Most triangles flattened in one call.
const MAX_TRIANGLES: usize = 400_000;
/// Most local/global steps whatever the caller asks for.
const MAX_ARAP_ITERATIONS: usize = 10_000;
/// A boundary vertex whose direction turns by more than this (degrees) is a corner of the polygon.
const CORNER_TURN_DEGREES: f64 = 30.0;
/// Upper bound of `tan(angle / 2)` in the mean-value weights (angles near 180 degrees).
const MAX_HALF_TAN: f64 = 1e6;
/// Cotangent weights of the rigid-fit step are kept in this range: positive, so the system stays
/// positive semi-definite on obtuse triangles, and bounded, so slivers do not dominate it.
const COT_RANGE: (f64, f64) = (1e-3, 1e3);
/// A triangle whose doubled area is below this share of its longest edge squared is degenerate.
const DEGENERATE_AREA: f64 = 1e-10;
/// Relative residual the linear solves aim at.
const SOLVE_TOLERANCE: f64 = 1e-12;
/// Largest relative residual of the interior (harmonic) system that is still accepted.
const HARMONIC_ACCEPT: f64 = 1e-8;
/// Largest relative residual of a rigid-fit global step that is still accepted.
const ARAP_ACCEPT: f64 = 1e-6;
/// The rigid-fit iteration stops when no vertex moves more than this share of the region's size.
const ARAP_STALL: f64 = 1e-13;

/// A flattening of a mesh region.
#[derive(Clone, Debug, PartialEq)]
pub struct Flattening {
    /// One (u, v) per vertex of the mesh; `None` for vertices not used by `triangles`.
    pub uv: Vec<Option<[f64; 2]>>,
    /// The boundary loop (vertex ids, counter-clockwise in (u, v)).
    pub boundary: Vec<u32>,
    /// Triangles whose orientation is reversed in (u, v) (0 for a valid flattening).
    pub flipped: usize,
    /// Largest edge-length distortion |l_uv / l_3d - 1| after uniform scaling.
    pub distortion_max: f64,
    /// Mean edge-length distortion |l_uv / l_3d - 1| after uniform scaling.
    pub distortion_mean: f64,
}

/// The region as a compact indexed mesh: local vertex ids `0..n` in the order of the global ids.
struct Topology {
    /// Global vertex id of each local vertex (ascending).
    ids: Vec<u32>,
    /// Triangles in local ids, as given (counter-clockwise).
    tris: Vec<[usize; 3]>,
    /// Each undirected edge once, smaller id first.
    edges: Vec<(usize, usize)>,
    /// Neighbours of each vertex, ascending.
    nbr: Vec<Vec<usize>>,
    /// The boundary loop, interior on its left.
    boundary: Vec<usize>,
    /// Is the vertex on the boundary?
    is_boundary: Vec<bool>,
}

/// Finds the root of `x` with path halving.
fn find_root(parent: &mut [usize], mut x: usize) -> usize {
    while parent[x] != x {
        parent[x] = parent[parent[x]];
        x = parent[x];
    }
    x
}

/// Checks the triangles of `mesh` listed in `triangles` and builds the compact topology and the
/// positions of its vertices.
fn build_topology(mesh: &Mesh, triangles: &[u32]) -> Result<(Topology, Vec<Vec3>), String> {
    if triangles.is_empty() {
        return Err("no triangles to flatten".to_string());
    }
    if triangles.len() > MAX_TRIANGLES {
        return Err(format!("too many triangles to flatten ({}, at most {MAX_TRIANGLES})", triangles.len()));
    }
    let mut corners: Vec<[u32; 3]> = Vec::with_capacity(triangles.len());
    for &t in triangles {
        let tri = mesh
            .triangles
            .get(t as usize)
            .ok_or_else(|| format!("triangle id {t} is out of range ({} triangles)", mesh.triangles.len()))?;
        corners.push(*tri);
    }
    let mut ids: Vec<u32> = corners.iter().flatten().copied().collect();
    ids.sort_unstable();
    ids.dedup();
    let mut pos: Vec<Vec3> = Vec::with_capacity(ids.len());
    for &v in &ids {
        let p = mesh
            .positions
            .get(v as usize)
            .ok_or_else(|| format!("vertex {v} is out of range ({} vertices)", mesh.positions.len()))?;
        if !surfacecraft_geom::finite(p) {
            return Err(format!("vertex {v} has a non-finite coordinate"));
        }
        pos.push(p.coords);
    }
    let nv = ids.len();
    let mut tris: Vec<[usize; 3]> = Vec::with_capacity(corners.len());
    for c in &corners {
        let mut t = [0usize; 3];
        for (slot, v) in t.iter_mut().zip(c.iter()) {
            *slot = ids.binary_search(v).map_err(|_| "internal error: unknown vertex".to_string())?;
        }
        if t[0] == t[1] || t[1] == t[2] || t[0] == t[2] {
            return Err("a triangle repeats a vertex".to_string());
        }
        tris.push(t);
    }

    // directed half-edges: a repeat is a non-manifold edge or two triangles of opposite orientation
    let mut half: Vec<(usize, usize)> = Vec::with_capacity(tris.len() * 3);
    for t in &tris {
        for k in 0..3 {
            half.push((t[k], t[(k + 1) % 3]));
        }
    }
    half.sort_unstable();
    if half.windows(2).any(|w| w[0] == w[1]) {
        return Err("non-manifold or inconsistently oriented edge in the region".to_string());
    }
    let mut edges: Vec<(usize, usize)> = Vec::with_capacity(half.len());
    let mut next = vec![usize::MAX; nv];
    let mut is_boundary = vec![false; nv];
    for &(a, b) in &half {
        let has_reverse = half.binary_search(&(b, a)).is_ok();
        if !has_reverse {
            if next[a] != usize::MAX {
                return Err("the boundary of the region touches itself at a vertex (not a disc)".to_string());
            }
            next[a] = b;
            is_boundary[a] = true;
            is_boundary[b] = true;
        }
        if a < b || !has_reverse {
            edges.push((a.min(b), a.max(b)));
        }
    }
    let mut nbr: Vec<Vec<usize>> = vec![Vec::new(); nv];
    for &(a, b) in &edges {
        nbr[a].push(b);
        nbr[b].push(a);
    }
    for list in &mut nbr {
        list.sort_unstable();
    }

    let mut parent: Vec<usize> = (0..nv).collect();
    for &(a, b) in &edges {
        let (ra, rb) = (find_root(&mut parent, a), find_root(&mut parent, b));
        if ra != rb {
            parent[ra] = rb;
        }
    }
    let components = (0..nv).filter(|&i| find_root(&mut parent, i) == i).count();
    if components > 1 {
        return Err(format!("the region is not connected ({components} parts)"));
    }

    let mut loops: Vec<Vec<usize>> = Vec::new();
    let mut visited = vec![false; nv];
    for s in 0..nv {
        if !is_boundary[s] || visited[s] {
            continue;
        }
        let mut lp = Vec::new();
        let mut v = s;
        loop {
            visited[v] = true;
            lp.push(v);
            let n = next[v];
            if n == usize::MAX {
                return Err("internal error: open boundary chain".to_string());
            }
            if n == s {
                break;
            }
            if visited[n] {
                return Err("internal error: boundary chain revisits a vertex".to_string());
            }
            v = n;
        }
        loops.push(lp);
    }
    let Some(first) = loops.first() else {
        return Err("the region is a closed surface (no boundary loop to flatten from)".to_string());
    };
    if loops.len() > 1 {
        return Err(format!("the region has {} boundary loops (it must be a disc: one loop, no holes)", loops.len()));
    }
    let euler = nv as i64 - edges.len() as i64 + tris.len() as i64;
    if euler != 1 {
        return Err(format!("the region is not a topological disc (Euler characteristic {euler}, expected 1)"));
    }
    let boundary = first.clone();
    Ok((Topology { ids, tris, edges, nbr, boundary, is_boundary }, pos))
}

/// Everything the solves need that depends on the 3-D shape only.
struct Geometry {
    /// Isometric planar coordinates of each triangle's corners.
    local: Vec<[[f64; 2]; 3]>,
    /// Cotangent weight (clamped) of the edge opposite each corner.
    cot: Vec<[f64; 3]>,
    /// Mean-value weights, rows normalised to sum 1, parallel to `Topology::nbr`.
    mean_value: Vec<Vec<f64>>,
    /// Summed cotangent weights per edge, parallel to `Topology::nbr`.
    laplacian: Vec<Vec<f64>>,
    /// Row sums of `laplacian`.
    laplacian_diag: Vec<f64>,
    /// Total 3-D area.
    area: f64,
}

/// Adds `value` to the weight of the pair (`i`, `j`).
fn add_weight(weights: &mut [Vec<f64>], nbr: &[Vec<usize>], i: usize, j: usize, value: f64) {
    if let Some(s) = nbr.get(i).and_then(|l| l.binary_search(&j).ok())
        && let Some(w) = weights.get_mut(i).and_then(|row| row.get_mut(s))
    {
        *w += value;
    }
}

/// Weights and local frames of every triangle.
fn build_geometry(topo: &Topology, pos: &[Vec3]) -> Result<Geometry, String> {
    let nv = topo.nbr.len();
    let mut mean_value: Vec<Vec<f64>> = topo.nbr.iter().map(|l| vec![0.0; l.len()]).collect();
    let mut laplacian = mean_value.clone();
    let mut local = Vec::with_capacity(topo.tris.len());
    let mut cot = Vec::with_capacity(topo.tris.len());
    let mut area = 0.0;
    for t in &topo.tris {
        let p = [pos[t[0]], pos[t[1]], pos[t[2]]];
        let lens2 = [(p[1] - p[0]).norm_squared(), (p[2] - p[1]).norm_squared(), (p[0] - p[2]).norm_squared()];
        let longest2 = lens2.iter().copied().fold(0.0, f64::max);
        let normal = (p[1] - p[0]).cross(&(p[2] - p[0]));
        let doubled = normal.norm();
        if !(doubled.is_finite() && doubled > DEGENERATE_AREA * longest2) {
            return Err("a triangle of the region is degenerate (zero area)".to_string());
        }
        area += doubled / 2.0;

        // isometric planar frame: first edge along x, counter-clockwise
        let e1 = p[1] - p[0];
        let e2 = p[2] - p[0];
        let ex = e1 / e1.norm();
        let ey = (normal / doubled).cross(&ex);
        local.push([[0.0, 0.0], [e1.norm(), 0.0], [e2.dot(&ex), e2.dot(&ey)]]);

        let mut cots = [0.0; 3];
        for k in 0..3 {
            let (j, l) = ((k + 1) % 3, (k + 2) % 3);
            let (d1, d2) = (p[j] - p[k], p[l] - p[k]);
            let (n1, n2) = (d1.norm(), d2.norm());
            let dot = d1.dot(&d2);
            let half_tan = (doubled / (n1 * n2 + dot).max(f64::MIN_POSITIVE)).min(MAX_HALF_TAN);
            add_weight(&mut mean_value, &topo.nbr, t[k], t[j], half_tan / n1);
            add_weight(&mut mean_value, &topo.nbr, t[k], t[l], half_tan / n2);
            let c = (dot / doubled).clamp(COT_RANGE.0, COT_RANGE.1);
            if let Some(slot) = cots.get_mut(k) {
                *slot = c;
            }
            add_weight(&mut laplacian, &topo.nbr, t[j], t[l], c);
            add_weight(&mut laplacian, &topo.nbr, t[l], t[j], c);
        }
        cot.push(cots);
    }
    for (i, row) in mean_value.iter_mut().enumerate() {
        let sum: f64 = row.iter().sum();
        if !(sum.is_finite() && sum > 0.0) {
            return Err(format!("vertex {i} of the region has no usable mean-value weights"));
        }
        for w in row.iter_mut() {
            *w /= sum;
        }
    }
    let laplacian_diag: Vec<f64> = laplacian.iter().map(|row| row.iter().sum()).collect();
    if laplacian_diag.len() != nv || laplacian_diag.iter().any(|d| !(d.is_finite() && *d > 0.0)) {
        return Err("the region has an isolated vertex".to_string());
    }
    Ok(Geometry { local, cot, mean_value, laplacian, laplacian_diag, area })
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

fn norm(a: &[f64]) -> f64 {
    dot(a, a).sqrt()
}

/// Relative residual `|b - A x| / |b|` (`0` for a zero right-hand side).
fn relative_residual(apply: &dyn Fn(&[f64], &mut [f64]), b: &[f64], x: &[f64]) -> f64 {
    let bn = norm(b);
    if !bn.is_finite() {
        return f64::NAN;
    }
    if bn == 0.0 {
        return 0.0;
    }
    let mut ax = vec![0.0; b.len()];
    apply(x, &mut ax);
    let r: f64 = b.iter().zip(&ax).map(|(bi, ai)| (bi - ai) * (bi - ai)).sum::<f64>().sqrt();
    r / bn
}

/// Preconditioned conjugate gradients (Jacobi) for a symmetric positive semi-definite operator
/// with a consistent right-hand side. `x` is the start and receives the result. Returns the
/// relative residual reached.
fn conjugate_gradient(
    apply: &dyn Fn(&[f64], &mut [f64]),
    inv_diag: &[f64],
    b: &[f64],
    x: &mut [f64],
    max_iter: usize,
) -> f64 {
    let n = b.len();
    let bn = norm(b);
    if !bn.is_finite() {
        return f64::NAN;
    }
    if bn == 0.0 {
        return 0.0;
    }
    let mut ap = vec![0.0; n];
    apply(x, &mut ap);
    let mut r: Vec<f64> = b.iter().zip(&ap).map(|(bi, ai)| bi - ai).collect();
    let mut z: Vec<f64> = r.iter().zip(inv_diag).map(|(ri, d)| ri * d).collect();
    let mut p = z.clone();
    let mut rz = dot(&r, &z);
    for _ in 0..max_iter {
        if norm(&r) <= SOLVE_TOLERANCE * bn {
            break;
        }
        apply(&p, &mut ap);
        let pap = dot(&p, &ap);
        if !(pap.is_finite() && pap > 0.0) {
            break;
        }
        let alpha = rz / pap;
        for i in 0..n {
            x[i] += alpha * p[i];
            r[i] -= alpha * ap[i];
            z[i] = r[i] * inv_diag[i];
        }
        let rz_new = dot(&r, &z);
        if !(rz.is_finite() && rz != 0.0 && rz_new.is_finite()) {
            break;
        }
        let beta = rz_new / rz;
        rz = rz_new;
        for i in 0..n {
            p[i] = z[i] + beta * p[i];
        }
    }
    relative_residual(apply, b, x)
}

/// BiCGSTAB for a general sparse operator (the mean-value system is not symmetric). `x` is the
/// start and receives the result. Returns the relative residual reached.
fn bicgstab(apply: &dyn Fn(&[f64], &mut [f64]), b: &[f64], x: &mut [f64], max_iter: usize) -> f64 {
    let n = b.len();
    let bn = norm(b);
    if !bn.is_finite() {
        return f64::NAN;
    }
    if bn == 0.0 {
        return 0.0;
    }
    let mut v = vec![0.0; n];
    apply(x, &mut v);
    let mut r: Vec<f64> = b.iter().zip(&v).map(|(bi, ai)| bi - ai).collect();
    let r_hat = r.clone();
    let (mut rho, mut alpha, mut omega) = (1.0f64, 1.0f64, 1.0f64);
    v.iter_mut().for_each(|e| *e = 0.0);
    let mut p = vec![0.0; n];
    let mut s = vec![0.0; n];
    let mut t = vec![0.0; n];
    for _ in 0..max_iter {
        if norm(&r) <= SOLVE_TOLERANCE * bn {
            break;
        }
        let rho_new = dot(&r_hat, &r);
        if !(rho_new.is_finite() && rho_new != 0.0 && omega != 0.0) {
            break;
        }
        let beta = (rho_new / rho) * (alpha / omega);
        for i in 0..n {
            p[i] = r[i] + beta * (p[i] - omega * v[i]);
        }
        apply(&p, &mut v);
        let rv = dot(&r_hat, &v);
        if !(rv.is_finite() && rv != 0.0) {
            break;
        }
        alpha = rho_new / rv;
        for i in 0..n {
            s[i] = r[i] - alpha * v[i];
        }
        if norm(&s) <= SOLVE_TOLERANCE * bn {
            for i in 0..n {
                x[i] += alpha * p[i];
            }
            break;
        }
        apply(&s, &mut t);
        let tt = dot(&t, &t);
        if !(tt.is_finite() && tt > 0.0) {
            break;
        }
        omega = dot(&t, &s) / tt;
        if !(omega.is_finite() && omega != 0.0) {
            break;
        }
        for i in 0..n {
            x[i] += alpha * p[i] + omega * s[i];
            r[i] = s[i] - omega * t[i];
        }
        rho = rho_new;
    }
    relative_residual(apply, b, x)
}

/// Positions of the boundary loop on a convex polygon inscribed in the unit circle: the corners
/// (turning angle above [`CORNER_TURN_DEGREES`]) at the angle of their chord-length fraction, the
/// vertices between two corners on the straight edge between them in proportion to chord length.
/// Fewer than three corners: every vertex on the circle itself.
fn boundary_polygon(lp: &[usize], pos: &[Vec3]) -> Result<Vec<[f64; 2]>, String> {
    let m = lp.len();
    let at = |k: usize| lp.get(k % m).and_then(|&v| pos.get(v)).copied().unwrap_or_else(Vec3::zeros);
    let lens: Vec<f64> = (0..m).map(|k| (at(k + 1) - at(k)).norm()).collect();
    let total: f64 = lens.iter().sum();
    if !(total.is_finite() && total > 1e-12) {
        return Err("the boundary of the region has no length".to_string());
    }
    let mut start = Vec::with_capacity(m);
    let mut acc = 0.0;
    for &l in &lens {
        start.push(acc);
        acc += l;
    }
    let cos_limit = CORNER_TURN_DEGREES.to_radians().cos();
    let corners: Vec<usize> = (0..m)
        .filter(|&k| {
            let d1 = at(k) - at(k + m - 1);
            let d2 = at(k + 1) - at(k);
            let (n1, n2) = (d1.norm(), d2.norm());
            n1 > 1e-12 && n2 > 1e-12 && d1.dot(&d2) / (n1 * n2) < cos_limit
        })
        .collect();
    let on_circle = |k: usize| {
        let a = TAU * start.get(k).copied().unwrap_or(0.0) / total;
        [a.cos(), a.sin()]
    };
    let mut out = vec![[0.0; 2]; m];
    if corners.len() < 3 {
        for (k, o) in out.iter_mut().enumerate() {
            *o = on_circle(k);
        }
        return Ok(out);
    }
    for (q, &a) in corners.iter().enumerate() {
        let b = corners.get((q + 1) % corners.len()).copied().unwrap_or(a);
        let count = (b + m - a) % m;
        let seg: f64 = (0..count).map(|st| lens.get((a + st) % m).copied().unwrap_or(0.0)).sum();
        let (pa, pb) = (on_circle(a), on_circle(b));
        let mut run = 0.0;
        for st in 0..count {
            let k = (a + st) % m;
            let f = if seg > 0.0 { run / seg } else { 0.0 };
            if let Some(o) = out.get_mut(k) {
                *o = [pa[0] + (pb[0] - pa[0]) * f, pa[1] + (pb[1] - pa[1]) * f];
            }
            run += lens.get(k).copied().unwrap_or(0.0);
        }
    }
    Ok(out)
}

/// Doubled signed area of a triangle in the plane.
fn area2(uv: &[[f64; 2]], t: &[usize; 3]) -> f64 {
    let (a, b, c) = (uv[t[0]], uv[t[1]], uv[t[2]]);
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

/// Triangles that are reversed (or collapsed) in the plane.
fn flipped_count(tris: &[[usize; 3]], uv: &[[f64; 2]]) -> usize {
    tris.iter().filter(|t| area2(uv, t) <= 0.0).count()
}

/// Signed area of the whole map.
fn uv_area(tris: &[[usize; 3]], uv: &[[f64; 2]]) -> f64 {
    tris.iter().map(|t| area2(uv, t)).sum::<f64>() / 2.0
}

fn all_finite(uv: &[[f64; 2]]) -> bool {
    uv.iter().all(|p| p[0].is_finite() && p[1].is_finite())
}

/// Scales `uv` uniformly so that its area equals `area`.
fn scale_to_area(tris: &[[usize; 3]], uv: &mut [[f64; 2]], area: f64) -> Result<(), String> {
    let a = uv_area(tris, uv);
    if !(a.is_finite() && a > 0.0) {
        return Err("the flattening has no positive area".to_string());
    }
    let s = (area / a).sqrt();
    for p in uv.iter_mut() {
        p[0] *= s;
        p[1] *= s;
    }
    Ok(())
}

/// Mean-value (harmonic-like) map of the interior for the given boundary positions.
fn harmonic(topo: &Topology, geo: &Geometry, polygon: &[[f64; 2]]) -> Result<Vec<[f64; 2]>, String> {
    let n = topo.nbr.len();
    let mut uv = vec![[0.0; 2]; n];
    for (&v, p) in topo.boundary.iter().zip(polygon) {
        uv[v] = *p;
    }
    let apply = |x: &[f64], y: &mut [f64]| {
        for i in 0..n {
            if topo.is_boundary[i] {
                y[i] = x[i];
            } else {
                let mut acc = x[i];
                for (&j, &w) in topo.nbr[i].iter().zip(&geo.mean_value[i]) {
                    acc -= w * x[j];
                }
                y[i] = acc;
            }
        }
    };
    let max_iter = 2000 + 20 * (n as f64).sqrt() as usize;
    for c in 0..2 {
        let b: Vec<f64> = (0..n).map(|i| if topo.is_boundary[i] { uv[i][c] } else { 0.0 }).collect();
        let mut x = b.clone();
        let residual = bicgstab(&apply, &b, &mut x, max_iter);
        if !(residual <= HARMONIC_ACCEPT) || x.iter().any(|e| !e.is_finite()) {
            return Err(format!("the interior system did not converge (relative residual {residual:e})"));
        }
        for i in 0..n {
            uv[i][c] = x[i];
        }
    }
    Ok(uv)
}

/// Up to `iterations` local/global steps of the rigid-fit relaxation of `uv`. A step is kept only
/// when its result is finite and has no more reversed triangles than the map before it.
fn relax_arap(topo: &Topology, geo: &Geometry, uv: &mut [[f64; 2]], iterations: usize) {
    let n = uv.len();
    let scale = geo.area.sqrt();
    let apply = |x: &[f64], y: &mut [f64]| {
        for i in 0..n {
            let mut acc = geo.laplacian_diag[i] * x[i];
            for (&j, &w) in topo.nbr[i].iter().zip(&geo.laplacian[i]) {
                acc -= w * x[j];
            }
            y[i] = acc;
        }
    };
    let inv_diag: Vec<f64> = geo.laplacian_diag.iter().map(|d| 1.0 / d).collect();
    let max_iter = 500 + 10 * (n as f64).sqrt() as usize;
    let mut cur: Vec<[f64; 2]> = uv.to_vec();
    let mut cur_flipped = flipped_count(&topo.tris, &cur);
    for _ in 0..iterations.min(MAX_ARAP_ITERATIONS) {
        // local step: the rotation closest to the Jacobian of every triangle
        let mut rhs = vec![[0.0f64; 2]; n];
        for (ti, t) in topo.tris.iter().enumerate() {
            let x = &geo.local[ti];
            let w = &geo.cot[ti];
            let mut jac = [[0.0f64; 2]; 2];
            for k in 0..3 {
                let (ia, ib) = ((k + 1) % 3, (k + 2) % 3);
                let du = [cur[t[ia]][0] - cur[t[ib]][0], cur[t[ia]][1] - cur[t[ib]][1]];
                let dx = [x[ia][0] - x[ib][0], x[ia][1] - x[ib][1]];
                for r in 0..2 {
                    for c in 0..2 {
                        jac[r][c] += w[k] * du[r] * dx[c];
                    }
                }
            }
            // maximises tr(R^T J) over rotations R = [[c, -s], [s, c]]
            let (cx, sy) = (jac[0][0] + jac[1][1], jac[1][0] - jac[0][1]);
            let len = cx.hypot(sy);
            let (c, s) = if len > 1e-300 { (cx / len, sy / len) } else { (1.0, 0.0) };
            for k in 0..3 {
                let (ia, ib) = ((k + 1) % 3, (k + 2) % 3);
                let dx = [x[ia][0] - x[ib][0], x[ia][1] - x[ib][1]];
                let d = [w[k] * (c * dx[0] - s * dx[1]), w[k] * (s * dx[0] + c * dx[1])];
                rhs[t[ia]][0] += d[0];
                rhs[t[ia]][1] += d[1];
                rhs[t[ib]][0] -= d[0];
                rhs[t[ib]][1] -= d[1];
            }
        }
        // global step: one Poisson-type solve per coordinate, translation left as it was
        let mut next = cur.clone();
        let mut ok = true;
        for c in 0..2 {
            let mut b: Vec<f64> = rhs.iter().map(|r| r[c]).collect();
            let mean = b.iter().sum::<f64>() / n as f64;
            for e in &mut b {
                *e -= mean;
            }
            let mut x: Vec<f64> = cur.iter().map(|p| p[c]).collect();
            let residual = conjugate_gradient(&apply, &inv_diag, &b, &mut x, max_iter);
            if !(residual <= ARAP_ACCEPT) || x.iter().any(|e| !e.is_finite()) {
                ok = false;
                break;
            }
            for i in 0..n {
                next[i][c] = x[i];
            }
        }
        if !ok || !all_finite(&next) {
            break;
        }
        let next_flipped = flipped_count(&topo.tris, &next);
        if next_flipped > cur_flipped {
            break;
        }
        let moved =
            cur.iter().zip(&next).map(|(a, b)| (a[0] - b[0]).abs().max((a[1] - b[1]).abs())).fold(0.0f64, f64::max);
        cur = next;
        cur_flipped = next_flipped;
        if moved <= ARAP_STALL * scale {
            break;
        }
    }
    uv.copy_from_slice(&cur);
}

/// Moves `uv` so that its vertex mean is the origin.
fn center(uv: &mut [[f64; 2]]) {
    if uv.is_empty() {
        return;
    }
    let n = uv.len() as f64;
    let (sx, sy) = uv.iter().fold((0.0, 0.0), |(x, y), p| (x + p[0], y + p[1]));
    for p in uv.iter_mut() {
        p[0] -= sx / n;
        p[1] -= sy / n;
    }
}

/// Rotates the centred `uv` to the orientation of the centred `reference` (least squares).
fn align_rotation(uv: &mut [[f64; 2]], reference: &[[f64; 2]]) {
    let (mut cross, mut dotp) = (0.0, 0.0);
    for (a, h) in uv.iter().zip(reference) {
        cross += a[0] * h[1] - a[1] * h[0];
        dotp += a[0] * h[0] + a[1] * h[1];
    }
    let len = cross.hypot(dotp);
    if !(len.is_finite() && len > 1e-300) {
        return;
    }
    let (s, c) = (cross / len, dotp / len);
    for p in uv.iter_mut() {
        *p = [c * p[0] - s * p[1], s * p[0] + c * p[1]];
    }
}

/// Largest and mean `|l_uv / l_3d - 1|` over the edges.
fn edge_distortion(topo: &Topology, pos: &[Vec3], uv: &[[f64; 2]]) -> (f64, f64) {
    let (mut max, mut sum, mut count) = (0.0f64, 0.0f64, 0usize);
    for &(a, b) in &topo.edges {
        let l3 = (pos[a] - pos[b]).norm();
        if l3 > 1e-12 {
            let luv = (uv[a][0] - uv[b][0]).hypot(uv[a][1] - uv[b][1]);
            let d = (luv / l3 - 1.0).abs();
            max = max.max(d);
            sum += d;
            count += 1;
        }
    }
    (max, if count > 0 { sum / count as f64 } else { 0.0 })
}

/// Flattens the triangles `triangles` of `mesh` (one disc: one boundary loop, no holes).
///
/// The result has the uv area of the region equal to its 3-D area. `arap_iterations` is the
/// number of local/global relaxation steps after the mean-value map (0 keeps that map).
///
/// # Errors
///
/// No triangles, more than 400 000 triangles, an id or vertex out of range, a non-finite
/// vertex, a degenerate triangle, a non-manifold or inconsistently oriented edge, a region that
/// is not connected or has no boundary (a closed surface), more than one boundary loop, a
/// boundary touching itself, or a linear system that does not converge.
pub fn flatten(mesh: &Mesh, triangles: &[u32], arap_iterations: usize) -> Result<Flattening, String> {
    let (topo, pos) = build_topology(mesh, triangles)?;
    let geo = build_geometry(&topo, &pos)?;
    let polygon = boundary_polygon(&topo.boundary, &pos)?;
    let mut uv = harmonic(&topo, &geo, &polygon)?;
    scale_to_area(&topo.tris, &mut uv, geo.area)?;
    if !all_finite(&uv) {
        return Err("the flattening is not finite".to_string());
    }
    let mut reference = uv.clone();
    center(&mut reference);
    if arap_iterations > 0 {
        relax_arap(&topo, &geo, &mut uv, arap_iterations);
        center(&mut uv);
        align_rotation(&mut uv, &reference);
        scale_to_area(&topo.tris, &mut uv, geo.area)?;
    } else {
        center(&mut uv);
    }
    if !all_finite(&uv) {
        return Err("the flattening is not finite".to_string());
    }
    let flipped = flipped_count(&topo.tris, &uv);
    let (distortion_max, distortion_mean) = edge_distortion(&topo, &pos, &uv);
    let mut out: Vec<Option<[f64; 2]>> = vec![None; mesh.positions.len()];
    for (local, &id) in topo.ids.iter().enumerate() {
        if let Some(slot) = out.get_mut(id as usize) {
            *slot = uv.get(local).copied();
        }
    }
    let boundary = topo.boundary.iter().filter_map(|&v| topo.ids.get(v).copied()).collect();
    Ok(Flattening { uv: out, boundary, flipped, distortion_max, distortion_mean })
}

#[cfg(test)]
mod tests;
