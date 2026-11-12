//! Orientation and position fields of the retopology (coarse-to-fine working levels).
use std::collections::BTreeMap;
use surfacecraft_geom::{KdTree, Mesh, Point3, Vec3, any_perpendicular, normalize};

pub(super) struct Level {
    pub(super) v: Vec<Vec3>,
    pub(super) n: Vec<Vec3>,
    pub(super) nb: Vec<Vec<u32>>,
    /// Boundary tangent for pinned boundary vertices.
    pub(super) pin: Vec<Option<Vec3>>,
}

pub(super) fn one_ring(mesh: &Mesh) -> Vec<Vec<u32>> {
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
pub(super) fn boundary_neighbours(mesh: &Mesh) -> Vec<Vec<u32>> {
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

pub(super) fn level_of(mesh: &Mesh) -> Level {
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
pub(super) fn compat4(o0: &Vec3, _n0: &Vec3, o1: &Vec3, n1: &Vec3) -> (Vec3, Vec3) {
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

pub(super) fn tangent(n: &Vec3, d: &Vec3) -> Vec3 {
    normalize(d - n * d.dot(n)).unwrap_or_else(|| any_perpendicular(n))
}

pub(super) fn init_orientation(l: &Level) -> Vec<Vec3> {
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

pub(super) fn smooth_orientation(l: &Level, o: &mut [Vec3], sweeps: usize) {
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
pub(super) fn lattice_pair(
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

pub(super) fn round_lattice(s: &Vec3, v: &Vec3, o: &Vec3, n: &Vec3, rho: f64, pinned: bool) -> Vec3 {
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
pub(super) fn init_positions(l: &Level, o: &[Vec3], rho: f64) -> Vec<Vec3> {
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

pub(super) fn smooth_positions(l: &Level, o: &[Vec3], p: &mut [Vec3], rho: f64, sweeps: usize) {
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
pub(super) fn prolong(coarse: &Level, co: &[Vec3], cp: &[Vec3], fine: &Level, rho: f64) -> (Vec<Vec3>, Vec<Vec3>) {
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
