//! Fairing of a refined patch with the rest of the mesh held fixed: membrane (umbrella)
//! iterations, then, for [`Fairing::ThinPlate`], thin-plate (umbrella of the umbrella) iterations.
use crate::{Mesh, Point3, Vec3};
use std::collections::{HashMap, HashSet};

/// How far a patch is faired.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Fairing {
    /// Membrane only: every vertex the mean of its neighbours, so the patch stays within the convex
    /// hull of its loop.
    Membrane,
    /// Membrane, then thin plate: the patch follows the curvature of the surface around the loop
    /// (it may leave that hull, and on a curved rim pass through the mesh).
    ThinPlate,
}

/// One row of the fairing operator (see [`fair_patch`]).
struct RowOp {
    len: usize,
    cols: Vec<usize>,
    own: Option<usize>,
}

/// Fairs the vertices from `first_vertex` on as `fairing` says, the rest of the mesh fixed.
pub(super) fn fair_patch(mesh: &mut Mesh, first_tri: usize, first_vertex: usize, fairing: Fairing) {
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
    if fairing == Fairing::Membrane {
        return;
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
