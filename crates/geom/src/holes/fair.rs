//! Fairing of a hole patch with the mesh around it held fixed: membrane (umbrella) iterations,
//! then a thin-plate solve (umbrella of the umbrella, least squares by conjugate gradients).
//! The three axes are independent (each reads and writes only its own coordinate), so they are
//! solved at the same time; each solve does its arithmetic in the order a lone one would.
use crate::{Mesh, Point3, Vec3, par};
use std::collections::{HashMap, HashSet};

/// Membrane sweeps before the thin-plate solve.
const MEMBRANE_SWEEPS: usize = 200;
/// Bounds on the conjugate-gradient iterations (four per free vertex in between).
const CG_MIN_ITERATIONS: usize = 50;
const CG_MAX_ITERATIONS: usize = 4000;
/// Squared residual, relative to the first one, at which a solve stops.
const CG_TOLERANCE: f64 = 1e-20;

/// Fairs the vertices from `first_vertex` on (the patch's own), the rest of the mesh fixed.
pub(super) fn fair_patch(mesh: &mut Mesh, first_tri: usize, first_vertex: usize) {
    let nv = mesh.positions.len();
    if first_vertex >= nv {
        return;
    }
    let (ring, touched) = rings(mesh, first_tri);
    let free: Vec<u32> = (first_vertex as u32..nv as u32).collect();
    membrane(&mut mesh.positions, &ring, &free);
    let mut rows: Vec<u32> = touched.into_iter().collect();
    rows.sort_unstable();
    let plate = Plate::new(ring, rows, free, first_vertex);
    let positions = &mesh.positions;
    let solved: Vec<Vec<f64>> = par::map(3, |axis| plate.solve(positions, axis));
    for (axis, x) in solved.iter().enumerate() {
        for (&v, &value) in plate.free.iter().zip(x) {
            if value.is_finite() {
                mesh.positions[v as usize][axis] = value;
            }
        }
    }
}

/// Rings of the patch vertices and of the vertices around the hole they touch (from the patch
/// and from the original triangles there, so the thin-plate step sees the outer surface), sorted
/// and without repeats; and the touched vertices.
fn rings(mesh: &Mesh, first_tri: usize) -> (HashMap<u32, Vec<u32>>, HashSet<u32>) {
    let mut ring: HashMap<u32, Vec<u32>> = HashMap::new();
    let mut add = |a: u32, b: u32| {
        ring.entry(a).or_default().push(b);
    };
    let mut touched: HashSet<u32> = HashSet::new();
    for t in mesh.triangles.get(first_tri..).unwrap_or(&[]) {
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
    (ring, touched)
}

/// Gauss–Seidel umbrella sweeps over the free vertices, in their order.
fn membrane(pos: &mut [Point3], ring: &HashMap<u32, Vec<u32>>, free: &[u32]) {
    // looked up once, not on every sweep
    let rings: Vec<&[u32]> = free.iter().map(|v| ring.get(v).map_or(&[][..], Vec::as_slice)).collect();
    for _ in 0..MEMBRANE_SWEEPS {
        for (&v, r) in free.iter().zip(&rings) {
            if r.is_empty() {
                continue;
            }
            let c = r.iter().fold(Vec3::zeros(), |a, &j| a + pos[j as usize].coords) / r.len() as f64;
            let l = c - pos[v as usize].coords;
            pos[v as usize] += l;
        }
    }
}

/// The thin-plate problem: minimise Σ over the touched vertices of |Δx|², the free vertices
/// unknown (normal equations Mᵀ M x = −Mᵀ (Δ of the fixed vertices alone), per axis).
struct Plate {
    ring: HashMap<u32, Vec<u32>>,
    rows: Vec<u32>,
    free: Vec<u32>,
    first_vertex: usize,
    op: Operator,
}

impl Plate {
    fn new(ring: HashMap<u32, Vec<u32>>, rows: Vec<u32>, free: Vec<u32>, first_vertex: usize) -> Self {
        let mut p = Self { ring, rows, free, first_vertex, op: Operator::default() };
        p.op = Operator::new(&p);
        p
    }

    fn ring_of(&self, v: u32) -> &[u32] {
        self.ring.get(&v).map_or(&[], Vec::as_slice)
    }

    /// The column of a free vertex: the free vertices are the last ones, so its offset.
    fn col_of(&self, v: u32) -> Option<u32> {
        (v as usize >= self.first_vertex).then(|| v - self.first_vertex as u32)
    }

    /// Δ of the fixed vertices alone, per row, on `axis`.
    fn fixed_lap(&self, positions: &[Point3], axis: usize) -> Vec<f64> {
        self.rows
            .iter()
            .map(|&v| {
                let r = self.ring_of(v);
                if r.is_empty() {
                    return 0.0;
                }
                let avg =
                    r.iter().filter(|&&j| self.col_of(j).is_none()).map(|&j| positions[j as usize][axis]).sum::<f64>()
                        / r.len() as f64;
                avg - if self.col_of(v).is_some() { 0.0 } else { positions[v as usize][axis] }
            })
            .collect()
    }

    /// The free vertices' coordinate on `axis` (conjugate gradients from where they are).
    fn solve(&self, positions: &[Point3], axis: usize) -> Vec<f64> {
        let nf = self.free.len();
        let mut my = vec![0.0; self.rows.len()];
        let mut ap = vec![0.0; nf];
        let mut b = vec![0.0; nf];
        self.op.apply_t(&self.fixed_lap(positions, axis), &mut b);
        for v in &mut b {
            *v = -*v;
        }
        let mut x: Vec<f64> = self.free.iter().map(|&v| positions[v as usize][axis]).collect();
        self.op.apply(&x, &mut my);
        self.op.apply_t(&my, &mut ap);
        let mut r: Vec<f64> = b.iter().zip(&ap).map(|(b, a)| b - a).collect();
        let mut p = r.clone();
        let mut rr: f64 = r.iter().map(|v| v * v).sum();
        let tol = rr * CG_TOLERANCE;
        for _ in 0..(4 * nf).clamp(CG_MIN_ITERATIONS, CG_MAX_ITERATIONS) {
            if rr <= tol || rr == 0.0 {
                break;
            }
            self.op.apply(&p, &mut my);
            self.op.apply_t(&my, &mut ap);
            let pap: f64 = p.iter().zip(&ap).map(|(a, b)| a * b).sum();
            if !(pap > 0.0) {
                break;
            }
            let alpha = rr / pap;
            for ((xk, rk), (&pk, &apk)) in x.iter_mut().zip(r.iter_mut()).zip(p.iter().zip(&ap)) {
                *xk += alpha * pk;
                *rk -= alpha * apk;
            }
            let rr2: f64 = r.iter().map(|v| v * v).sum();
            let beta = rr2 / rr;
            rr = rr2;
            for (pk, &rk) in p.iter_mut().zip(&r) {
                *pk = rk + beta * *pk;
            }
        }
        x
    }
}

/// The rows of the thin-plate operator M, one per touched vertex: row `r` is the mean of the
/// vertex's ring (`len[r]` members, the free ones at the columns `cols[start[r]..start[r + 1]]`)
/// minus the vertex itself (column `own[r]` when it is free).
#[derive(Default)]
struct Operator {
    len: Vec<usize>,
    start: Vec<usize>,
    cols: Vec<u32>,
    own: Vec<Option<u32>>,
}

impl Operator {
    fn new(p: &Plate) -> Self {
        let mut op = Self { start: vec![0], ..Self::default() };
        for &v in &p.rows {
            let r = p.ring_of(v);
            op.len.push(r.len());
            op.cols.extend(r.iter().filter_map(|&j| p.col_of(j)));
            op.start.push(op.cols.len());
            op.own.push(p.col_of(v));
        }
        op
    }

    fn cols_of(&self, row: usize) -> &[u32] {
        match (self.start.get(row), self.start.get(row + 1)) {
            (Some(&a), Some(&b)) => self.cols.get(a..b).unwrap_or(&[]),
            _ => &[],
        }
    }

    /// `y = M x` for the free values `x` (fixed ones taken as zero).
    fn apply(&self, x: &[f64], y: &mut [f64]) {
        for (row, (out, &len)) in y.iter_mut().zip(&self.len).enumerate() {
            if len == 0 {
                *out = 0.0;
                continue;
            }
            let avg = self.cols_of(row).iter().map(|&k| x[k as usize]).sum::<f64>() / len as f64;
            *out = avg - self.own[row].map_or(0.0, |k| x[k as usize]);
        }
    }

    /// `out = Mᵀ y`.
    fn apply_t(&self, y: &[f64], out: &mut [f64]) {
        out.fill(0.0);
        for (row, (&yr, &len)) in y.iter().zip(&self.len).enumerate() {
            if len == 0 {
                continue;
            }
            let w = yr / len as f64;
            for &k in self.cols_of(row) {
                out[k as usize] += w;
            }
            if let Some(k) = self.own[row] {
                out[k as usize] -= yr;
            }
        }
    }
}
