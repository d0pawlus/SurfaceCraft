//! APP-GEOMETRY: stands in for a solidcore constrained variational surface fit (n-sided boundary, G0/G1/G2 sides, interior points and curves).
//!
//! One bicubic B-spline patch with clamped uniform knots on a rectangular parameter domain,
//! fitted by minimising a thin-plate bending energy plus weighted data residuals under hard
//! linear conditions (positions, tangent-plane rows, normal-curvature rows). The conditions are
//! linear in the control points, so the solve is dense linear algebra: the hard rows are reduced
//! (QR) and decomposed (SVD) for rank and a particular solution, and the energy is minimised on
//! the null space of the hard rows (Cholesky). Basis functions, their derivatives and the surface
//! derivatives come from solidcore (`basis_derivs`, `Surface::derivs`).
//!
//! Sources: Piegl and Tiller, *The NURBS Book* (basis derivatives); Welch and Witkin, *Variational
//! surface modeling* (1992), and Celniker and Gossard (1991) for the energy and the linear
//! conditions; Golub and Van Loan, *Matrix Computations* (null-space method).
//!
//! Positions are mapped to a unit box before assembly (centre and diagonal of every given point)
//! and mapped back afterwards, so the result does not depend on units. Hard rows are scaled to unit
//! norm before the rank decision.
use crate::record::brep::Surface;
use crate::{KernelError, Result, op_err, poll};
use cells::active_cells;
use nalgebra::DMatrix;
use solidcore::nurbs::basis_basic::basis_derivs;
use solidcore::nurbs::knots::{KnotVector, Side};
use solidcore::nurbs::math::{CPoint, NOW};
use solidcore::nurbs::srf_basic::Surface as Nurb;
use solve::constrained_min;
use surfacecraft_geom::Cancel;

mod assemble;
mod cells;
mod cost;
mod solve;

use assemble::{assemble_soft, control_net, expand_coupled, hard_residual, stiffness};
use cost::check_cost;
pub use cost::{MAX_SOLVE_SECONDS, solve_seconds};

/// Fewest control points per direction.
pub const MIN_CONTROL: usize = 4;
/// Most control points per direction.
pub const MAX_CONTROL: usize = 24;
/// Degree of the patch in both directions.
pub const DEGREE: usize = 3;

const MAX_DATA: usize = 50_000;
const MAX_ROWS: usize = 20_000;
/// Data points or rows between two looks at the cancellation flag.
const POLL_EVERY: usize = 512;
/// Weight of the diagonal that keeps control points of inactive cells determined.
const REGULARIZE: f64 = 1e-8;
/// Weight of the diagonal added everywhere.
const DIAG_EPS: f64 = 1e-12;
/// Singular values below this fraction of the largest one count as zero.
const RANK_TOL: f64 = 1e-9;
/// Largest magnitude accepted for coordinates, parameters and vector components.
const LIMIT: f64 = 1e100;
/// Largest weight or stiffness accepted.
const WEIGHT_LIMIT: f64 = 1e50;
/// Vectors whose components are all below this are treated as zero.
const TINY: f64 = 1e-100;
/// Narrowest knot cell the basis routines handle reliably.
const MIN_CELL: f64 = 1e-11;

const GAUSS_NODES: [f64; 4] = [-0.8611363115940526, -0.3399810435848563, 0.3399810435848563, 0.8611363115940526];
const GAUSS_WEIGHTS: [f64; 4] = [0.3478548451374538, 0.6521451548625461, 0.6521451548625461, 0.3478548451374538];

/// A scan/data sample with its parameters.
#[derive(Clone, Copy, Debug)]
pub struct DataPoint {
    /// Parameters of the sample on the patch.
    pub uv: [f64; 2],
    /// The sampled position.
    pub at: [f64; 3],
    /// Weight of the squared distance (> 0).
    pub weight: f64,
}

/// One linear condition on the patch `S`.
#[derive(Clone, Debug)]
pub enum Row {
    /// `S(uv) = at` (3 scalar rows).
    Position {
        /// Parameters.
        uv: [f64; 2],
        /// Required position.
        at: [f64; 3],
    },
    /// `S_d(uv) · normal = 0`, `S_d = dir[0]·S_u + dir[1]·S_v` (1 scalar row).
    Tangent {
        /// Parameters.
        uv: [f64; 2],
        /// Direction in the parameter plane.
        dir: [f64; 2],
        /// Normal the derivative must be orthogonal to.
        normal: [f64; 3],
    },
    /// `S_dd(uv) · normal = value`, `S_dd = dir0²·S_uu + 2·dir0·dir1·S_uv + dir1²·S_vv` (1 scalar row).
    Curvature {
        /// Parameters.
        uv: [f64; 2],
        /// Direction in the parameter plane.
        dir: [f64; 2],
        /// Normal the second derivative is projected on.
        normal: [f64; 3],
        /// Required value of the projection.
        value: f64,
    },
}

/// The variational problem.
#[derive(Clone, Debug)]
pub struct Problem {
    /// `[[u0, u1], [v0, v1]]`, `u0 < u1`, `v0 < v1`.
    pub domain: [[f64; 2]; 2],
    /// Control points per direction.
    pub control: [usize; 2],
    /// Conditions met exactly (up to dependent rows, which are dropped).
    pub hard: Vec<Row>,
    /// Conditions met in the least-squares sense with the given weight (> 0). A tangent or curvature
    /// row is scaled to unit Euclidean length (its right-hand side with it) before the weight applies,
    /// so the weight does not depend on the units of the parameters; a position row is a squared
    /// distance in the problem's unit box.
    pub soft: Vec<(Row, f64)>,
    /// Scan samples.
    pub data: Vec<DataPoint>,
    /// Closed polygon in (u, v): bending energy is integrated over knot cells that touch or lie
    /// inside it plus one ring of cells; other cells get only `REGULARIZE`. Empty = whole domain.
    pub outline: Vec<[f64; 2]>,
    /// Relative stiffness (> 0): energy weight = smoothing · trace(data normal matrix) / trace(K).
    /// Only the data points (with their weights) enter that trace; soft rows never do. With no data
    /// but soft rows the trace of the soft rows' normal matrix is used instead; with neither the
    /// energy weight is 1.
    pub smoothing: f64,
}

/// The fitted patch and how the hard rows fared.
#[derive(Clone, Debug)]
pub struct Solution {
    /// `Surface::Spline`, clamped uniform knots on the domain.
    pub surface: Surface,
    /// Numerical rank of the hard rows (scalar rows).
    pub rank: usize,
    /// Hard scalar rows found dependent on others.
    pub dropped: usize,
    /// Largest `|row·x − rhs|` over the hard rows after the solve (mm or mm-derivative units).
    pub hard_residual: f64,
}

fn arg(msg: impl Into<String>) -> KernelError {
    KernelError::Argument(msg.into())
}

fn bounded(vals: &[f64]) -> bool {
    vals.iter().all(|v| v.is_finite() && v.abs() <= LIMIT)
}

fn nonzero(vals: &[f64]) -> bool {
    vals.iter().any(|v| v.abs() > TINY)
}

fn in_domain(dom: &[[f64; 2]; 2], uv: [f64; 2]) -> bool {
    (0..2).all(|d| {
        let [lo, hi] = dom[d];
        let tol = 1e-9 * (hi - lo);
        uv[d] >= lo - tol && uv[d] <= hi + tol
    })
}

fn check_uv(dom: &[[f64; 2]; 2], uv: [f64; 2], what: &str) -> Result<()> {
    if bounded(&uv) && in_domain(dom, uv) {
        Ok(())
    } else {
        Err(arg(format!("{what}: the parameters ({}, {}) are not finite or lie outside the domain", uv[0], uv[1])))
    }
}

fn check_row(dom: &[[f64; 2]; 2], row: &Row) -> Result<()> {
    match row {
        Row::Position { uv, at } => {
            check_uv(dom, *uv, "position row")?;
            if !bounded(at) {
                return Err(arg("position row: the position must be finite"));
            }
        }
        Row::Tangent { uv, dir, normal } => {
            check_uv(dom, *uv, "tangent row")?;
            if !bounded(dir) || !bounded(normal) || !nonzero(dir) || !nonzero(normal) {
                return Err(arg("tangent row: direction and normal must be finite and not zero"));
            }
        }
        Row::Curvature { uv, dir, normal, value } => {
            check_uv(dom, *uv, "curvature row")?;
            if !bounded(dir) || !bounded(normal) || !nonzero(dir) || !nonzero(normal) || !bounded(&[*value]) {
                return Err(arg(
                    "curvature row: direction, normal and value must be finite, direction and normal not zero",
                ));
            }
        }
    }
    Ok(())
}

fn validate(p: &Problem) -> Result<()> {
    let [[u0, u1], [v0, v1]] = p.domain;
    if !bounded(&[u0, u1, v0, v1]) || u0 >= u1 || v0 >= v1 {
        return Err(arg("the parameter domain must be finite and not empty"));
    }
    if p.control.iter().any(|n| *n < MIN_CONTROL || *n > MAX_CONTROL) {
        return Err(arg(format!(
            "the control net must have {MIN_CONTROL} to {MAX_CONTROL} points per direction, not {} × {}",
            p.control[0], p.control[1]
        )));
    }
    if !p.smoothing.is_finite() || p.smoothing <= 0.0 || p.smoothing > WEIGHT_LIMIT {
        return Err(arg("the smoothing must be finite and positive"));
    }
    if p.data.len() > MAX_DATA {
        return Err(arg(format!("at most {MAX_DATA} data points are accepted, not {}", p.data.len())));
    }
    if p.hard.len() + p.soft.len() > MAX_ROWS {
        return Err(arg(format!("at most {MAX_ROWS} rows are accepted, not {}", p.hard.len() + p.soft.len())));
    }
    for d in &p.data {
        check_uv(&p.domain, d.uv, "data point")?;
        if !bounded(&d.at) {
            return Err(arg("data point: the position must be finite"));
        }
        if !d.weight.is_finite() || d.weight <= 0.0 || d.weight > WEIGHT_LIMIT {
            return Err(arg("data point: the weight must be finite and positive"));
        }
    }
    for r in &p.hard {
        check_row(&p.domain, r)?;
    }
    for (r, w) in &p.soft {
        check_row(&p.domain, r)?;
        if !w.is_finite() || *w <= 0.0 || *w > WEIGHT_LIMIT {
            return Err(arg("soft row: the weight must be finite and positive"));
        }
    }
    if !p.outline.is_empty() {
        if p.outline.len() < 3 {
            return Err(arg("the outline needs at least 3 corners"));
        }
        if p.outline.iter().any(|c| !bounded(c)) {
            return Err(arg("the outline must be finite"));
        }
    }
    Ok(())
}

/// Knot cell boundaries of `cells` equal cells on `[a, b]`.
fn bounds(cells: usize, a: f64, b: f64) -> Vec<f64> {
    (0..=cells)
        .map(|i| {
            if i == 0 {
                a
            } else if i == cells {
                b
            } else {
                a + (b - a) * (i as f64) / (cells as f64)
            }
        })
        .collect()
}

/// Clamped knot vector of degree [`DEGREE`] whose distinct knots are `bounds`.
fn clamped(bounds: &[f64]) -> Vec<f64> {
    let first = bounds.first().copied().unwrap_or(0.0);
    let last = bounds.last().copied().unwrap_or(1.0);
    let mut k = Vec::with_capacity(bounds.len() + 2 * DEGREE);
    k.extend(std::iter::repeat_n(first, DEGREE));
    k.extend_from_slice(bounds);
    k.extend(std::iter::repeat_n(last, DEGREE));
    k
}

/// Clamped uniform knot vector for `n` control points on `[a, b]`.
#[cfg(test)]
fn clamped_uniform(n: usize, a: f64, b: f64) -> Vec<f64> {
    clamped(&bounds(n.saturating_sub(DEGREE).max(1), a, b))
}

/// Parameter grid of the patch: knot vectors and knot cells.
struct Grid {
    domain: [[f64; 2]; 2],
    control: [usize; 2],
    cells: [usize; 2],
    bounds: [Vec<f64>; 2],
    knots: [Vec<f64>; 2],
}

impl Grid {
    fn new(domain: [[f64; 2]; 2], control: [usize; 2]) -> Result<Self> {
        let cells = [control[0].saturating_sub(DEGREE).max(1), control[1].saturating_sub(DEGREE).max(1)];
        let bu = bounds(cells[0], domain[0][0], domain[0][1]);
        let bv = bounds(cells[1], domain[1][0], domain[1][1]);
        if ![&bu, &bv].iter().all(|b| b.windows(2).all(|w| matches!(w, [lo, hi] if hi - lo > MIN_CELL))) {
            return Err(arg("the parameter domain is too small for the number of knot cells"));
        }
        let knots = [clamped(&bu), clamped(&bv)];
        Ok(Self { domain, control, cells, bounds: [bu, bv], knots })
    }

    fn clamp(&self, uv: [f64; 2]) -> [f64; 2] {
        // the domain is validated finite with lo < hi, so `clamp` cannot panic
        [uv[0].clamp(self.domain[0][0], self.domain[0][1]), uv[1].clamp(self.domain[1][0], self.domain[1][1])]
    }

    fn count(&self) -> usize {
        self.control[0] * self.control[1]
    }
}

/// Non-vanishing cubic basis functions and their derivatives up to second order at one parameter.
#[derive(Clone, Copy, Debug)]
struct Local {
    /// Index of the control point of the first non-vanishing function.
    first: usize,
    /// `d[k][r]`: `k`-th derivative of the function `first + r`.
    d: [[f64; 4]; 3],
}

fn local_basis(knots: &[f64], t: f64) -> Result<Local> {
    let kv = KnotVector::from_slice(knots).map_err(|e| op_err("KnotVector::from_slice", format!("{e:?}")))?;
    let bd = basis_derivs(&kv, DEGREE, t, Side::Left, 2).map_err(|e| op_err("basis_derivs", format!("{e:?}")))?;
    let first = bd.span.checked_sub(DEGREE).ok_or_else(|| arg("knot span index below the degree"))?;
    let mut d = [[0.0; 4]; 3];
    for (k, row) in d.iter_mut().enumerate() {
        for (r, slot) in row.iter_mut().enumerate() {
            *slot = bd
                .derivs
                .get(k)
                .and_then(|x| x.get(r))
                .copied()
                .ok_or_else(|| arg("basis derivative table is incomplete"))?;
        }
    }
    Ok(Local { first, d })
}

/// The 4 × 4 control points that influence the patch at one parameter pair, with the basis there.
struct Stencil {
    bu: Local,
    bv: Local,
}

impl Stencil {
    fn new(g: &Grid, uv: [f64; 2]) -> Result<Self> {
        let bu = local_basis(&g.knots[0], uv[0])?;
        let bv = local_basis(&g.knots[1], uv[1])?;
        if bu.first + DEGREE >= g.control[0] || bv.first + DEGREE >= g.control[1] {
            return Err(arg("the knot span lies outside the control net"));
        }
        Ok(Self { bu, bv })
    }

    /// Weights of the 16 control points for the derivative of order `du` in u and `dv` in v.
    fn coefficient(&self, du: usize, dv: usize) -> [f64; 16] {
        std::array::from_fn(|i| {
            let (r, s) = (i / 4, i % 4);
            let a = self.bu.d.get(du).map_or(0.0, |row| row[r]);
            let b = self.bv.d.get(dv).map_or(0.0, |row| row[s]);
            a * b
        })
    }

    /// Flat indices (`i·m + j`) of the 16 control points, matching [`Stencil::coefficient`].
    fn indices(&self, g: &Grid) -> [usize; 16] {
        std::array::from_fn(|i| (self.bu.first + i / 4) * g.control[1] + self.bv.first + i % 4)
    }
}

/// Maps positions to a unit box and back.
struct Scale {
    center: [f64; 3],
    factor: f64,
}

impl Scale {
    fn new(p: &Problem) -> Result<Self> {
        let mut pts: Vec<[f64; 3]> = p.data.iter().map(|d| d.at).collect();
        for r in &p.hard {
            if let Row::Position { at, .. } = r {
                pts.push(*at);
            }
        }
        for (r, _) in &p.soft {
            if let Row::Position { at, .. } = r {
                pts.push(*at);
            }
        }
        if pts.is_empty() {
            return Ok(Self { center: [0.0; 3], factor: 1.0 });
        }
        let mut lo = [f64::INFINITY; 3];
        let mut hi = [f64::NEG_INFINITY; 3];
        for q in &pts {
            for c in 0..3 {
                lo[c] = lo[c].min(q[c]);
                hi[c] = hi[c].max(q[c]);
            }
        }
        let diag = (0..3).map(|c| (hi[c] - lo[c]).powi(2)).sum::<f64>().sqrt();
        let factor = if diag > TINY { 1.0 / diag } else { 1.0 };
        let center = [0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1]), 0.5 * (lo[2] + hi[2])];
        if !factor.is_finite() || !bounded(&center) {
            return Err(arg("the given positions cannot be scaled"));
        }
        Ok(Self { center, factor })
    }

    fn forward(&self, p: [f64; 3]) -> [f64; 3] {
        std::array::from_fn(|c| (p[c] - self.center[c]) * self.factor)
    }
}

/// One scalar row `Σ a_j x_j = rhs`; `rhs` has three entries in the decoupled layout (one per
/// coordinate) and only the first is used in the coupled layout.
struct LinRow {
    cols: Vec<(usize, f64)>,
    rhs: [f64; 3],
}

fn couples(r: &Row) -> bool {
    !matches!(r, Row::Position { .. })
}

fn directional_row(g: &Grid, st: &Stencil, coef: &[f64; 16], normal: [f64; 3], rhs: f64) -> LinRow {
    let idx = st.indices(g);
    let mut cols = Vec::with_capacity(48);
    for (p, &ip) in idx.iter().enumerate() {
        for (q, nq) in normal.iter().enumerate() {
            cols.push((3 * ip + q, coef[p] * nq));
        }
    }
    LinRow { cols, rhs: [rhs, 0.0, 0.0] }
}

/// The scalar rows of one condition. `coupled` selects the layout `x[3·idx + c]` (rows over the
/// three coordinates) against one row per control point with three right-hand sides.
fn build_rows(g: &Grid, scale: &Scale, row: &Row, coupled: bool) -> Result<Vec<LinRow>> {
    match row {
        Row::Position { uv, at } => {
            let st = Stencil::new(g, g.clamp(*uv))?;
            let phi = st.coefficient(0, 0);
            let idx = st.indices(g);
            let at = scale.forward(*at);
            if coupled {
                Ok((0..3)
                    .map(|q| LinRow {
                        cols: idx.iter().zip(phi.iter()).map(|(&i, &w)| (3 * i + q, w)).collect(),
                        rhs: [at[q], 0.0, 0.0],
                    })
                    .collect())
            } else {
                Ok(vec![LinRow { cols: idx.iter().zip(phi.iter()).map(|(&i, &w)| (i, w)).collect(), rhs: at }])
            }
        }
        Row::Tangent { uv, dir, normal } => {
            let st = Stencil::new(g, g.clamp(*uv))?;
            let (su, sv) = (st.coefficient(1, 0), st.coefficient(0, 1));
            let coef: [f64; 16] = std::array::from_fn(|i| dir[0] * su[i] + dir[1] * sv[i]);
            Ok(vec![directional_row(g, &st, &coef, *normal, 0.0)])
        }
        Row::Curvature { uv, dir, normal, value } => {
            let st = Stencil::new(g, g.clamp(*uv))?;
            let (suu, suv, svv) = (st.coefficient(2, 0), st.coefficient(1, 1), st.coefficient(0, 2));
            let coef: [f64; 16] = std::array::from_fn(|i| {
                dir[0] * dir[0] * suu[i] + 2.0 * dir[0] * dir[1] * suv[i] + dir[1] * dir[1] * svv[i]
            });
            Ok(vec![directional_row(g, &st, &coef, *normal, value * scale.factor)])
        }
    }
}

/// Adds `w·|φ·x − at|²` (per coordinate) to the scalar normal matrix `a` and right-hand side `b`.
fn accumulate(a: &mut DMatrix<f64>, b: &mut DMatrix<f64>, idx: &[usize; 16], phi: &[f64; 16], at: [f64; 3], w: f64) {
    for (p, &ip) in idx.iter().enumerate() {
        for (q, &iq) in idx.iter().enumerate() {
            if let Some(slot) = a.get_mut((ip, iq)) {
                *slot += w * phi[p] * phi[q];
            }
        }
        for (c, v) in at.iter().enumerate() {
            if let Some(slot) = b.get_mut((ip, c)) {
                *slot += w * phi[p] * v;
            }
        }
    }
}

fn gauss_tables(knots: &[f64], cell_bounds: &[f64]) -> Result<Vec<Vec<(Local, f64)>>> {
    let mut out = Vec::new();
    for w in cell_bounds.windows(2) {
        let [lo, hi] = w else { continue };
        let (mid, half) = (0.5 * (lo + hi), 0.5 * (hi - lo));
        let mut cell = Vec::with_capacity(4);
        for (xi, wt) in GAUSS_NODES.iter().zip(GAUSS_WEIGHTS.iter()) {
            cell.push((local_basis(knots, mid + half * xi)?, half * wt));
        }
        out.push(cell);
    }
    Ok(out)
}

/// Scalar bending-energy matrix `K` (`∫∫ S_uu² + 2 S_uv² + S_vv²` per coordinate) over the active
/// cells, and the control points that belong to an inactive cell.
fn energy_matrix(g: &Grid, outline: &[[f64; 2]]) -> Result<(DMatrix<f64>, Vec<bool>)> {
    let [cu, cv] = g.cells;
    let m = g.control[1];
    let nn = g.count();
    let active = active_cells(g, outline);
    if !active.iter().any(|a| *a) {
        return Err(arg("the outline does not meet the parameter domain"));
    }
    let gu = gauss_tables(&g.knots[0], &g.bounds[0])?;
    let gv = gauss_tables(&g.knots[1], &g.bounds[1])?;
    let mut k = DMatrix::<f64>::zeros(nn, nn);
    let mut inactive = vec![false; nn];
    for a in 0..cu {
        for b in 0..cv {
            if !active.get(a * cv + b).copied().unwrap_or(true) {
                for p in 0..16 {
                    if let Some(slot) = inactive.get_mut((a + p / 4) * m + b + p % 4) {
                        *slot = true;
                    }
                }
                continue;
            }
            let (Some(tu), Some(tv)) = (gu.get(a), gv.get(b)) else { continue };
            for (lu, wu) in tu {
                for (lv, wv) in tv {
                    let w = wu * wv;
                    let guu: [f64; 16] = std::array::from_fn(|i| lu.d[2][i / 4] * lv.d[0][i % 4]);
                    let guv: [f64; 16] = std::array::from_fn(|i| lu.d[1][i / 4] * lv.d[1][i % 4]);
                    let gvv: [f64; 16] = std::array::from_fn(|i| lu.d[0][i / 4] * lv.d[2][i % 4]);
                    let idx: [usize; 16] = std::array::from_fn(|i| (lu.first + i / 4) * m + lv.first + i % 4);
                    for (p, &ip) in idx.iter().enumerate() {
                        for (q, &iq) in idx.iter().enumerate() {
                            if let Some(slot) = k.get_mut((ip, iq)) {
                                *slot += w * (guu[p] * guu[q] + 2.0 * guv[p] * guv[q] + gvv[p] * gvv[q]);
                            }
                        }
                    }
                }
            }
        }
    }
    Ok((k, inactive))
}

/// Fits the patch.
///
/// Errors (`KernelError::Argument`) for non-finite input, a control net outside
/// [`MIN_CONTROL`]..=[`MAX_CONTROL`], an empty or inverted domain, parameters outside the domain,
/// non-positive weights, zero directions or normals, too many data points or rows, and a solve
/// estimated above [`MAX_SOLVE_SECONDS`] (see [`solve_seconds`]).
pub fn solve(p: &Problem) -> Result<Solution> {
    solve_cancellable(p, &Cancel::new())
}

/// [`solve`] that gives up with [`KernelError::Cancelled`] when `cancel` is set: checked between the
/// stages and inside the loops over the data and the rows, so a stop waits for one dense
/// factorisation at most (a few seconds at the largest accepted size).
pub fn solve_cancellable(p: &Problem, cancel: &Cancel) -> Result<Solution> {
    validate(p)?;
    check_cost(p)?;
    let g = Grid::new(p.domain, p.control)?;
    let nn = g.count();
    let scale = Scale::new(p)?;
    let coupled = p.is_coupled();

    let (kmat, inactive) = energy_matrix(&g, &p.outline)?;
    poll(cancel)?;
    let tr_k: f64 = (0..nn).filter_map(|i| kmat.get((i, i))).sum();
    if !tr_k.is_finite() || tr_k <= 0.0 {
        return Err(op_err("vpatch", "the bending energy matrix is empty"));
    }
    let soft = assemble_soft(&g, &scale, p, cancel)?;
    // with data: relative to the data; with soft rows only: relative to them; with neither: 1
    let tr_scale = if soft.tr_data > 0.0 { soft.tr_data } else { soft.tr_soft };
    let lambda = if tr_scale > 0.0 { p.smoothing * tr_scale / (3.0 * tr_k) } else { 1.0 };
    if !lambda.is_finite() {
        return Err(arg("the smoothing weight is not finite"));
    }
    let h_s = stiffness(&soft.a, &kmat, lambda, &inactive);
    let mut hard_rows: Vec<LinRow> = Vec::new();
    for r in &p.hard {
        hard_rows.extend(build_rows(&g, &scale, r, coupled)?);
    }

    let (x, rank, unit) = if coupled {
        let (h3, b3) = expand_coupled(&h_s, &soft.b, &soft.coupled, cancel)?;
        let (x, rank) = constrained_min(&h3, &b3, &hard_rows, cancel)?;
        (x, rank, 1)
    } else {
        let (x, rank) = constrained_min(&h_s, &soft.b, &hard_rows, cancel)?;
        (x, rank, 3)
    };
    let residual = hard_residual(&hard_rows, &x, unit, &scale)?;
    let points = control_net(&g, &x, coupled, &scale)?;
    let total = hard_rows.len() * unit;
    Ok(Solution {
        surface: Surface::Spline { u_knots: g.knots[0].clone(), v_knots: g.knots[1].clone(), points },
        rank: rank * unit,
        dropped: total.saturating_sub(rank * unit),
        hard_residual: residual,
    })
}

fn spline_to_nurb(s: &Surface) -> Result<Nurb> {
    let Surface::Spline { u_knots, v_knots, points } = s else {
        return Err(arg("expected a spline surface"));
    };
    let nu = points.len();
    let nv = points.first().map_or(0, Vec::len);
    if nu < 2 || nv < 2 || points.iter().any(|r| r.len() != nv) {
        return Err(arg("spline control net must be rectangular"));
    }
    if points.iter().flatten().any(|c| !crate::finite_all(c))
        || !crate::finite_all(u_knots)
        || !crate::finite_all(v_knots)
    {
        return Err(arg("spline surface must be finite"));
    }
    let p = u_knots.len().checked_sub(nu + 1).ok_or_else(|| arg("u knots too short"))?;
    let q = v_knots.len().checked_sub(nv + 1).ok_or_else(|| arg("v knots too short"))?;
    if p == 0 || q == 0 {
        return Err(arg("invalid spline surface"));
    }
    let net: Vec<Vec<CPoint>> =
        points.iter().map(|r| r.iter().map(|c| CPoint { x: c[0], y: c[1], z: c[2], w: NOW }).collect()).collect();
    Nurb::new(net, p, q, u_knots.clone(), v_knots.clone())
        .map_err(|e| op_err("srf_basic::Surface::new", format!("{e:?}")))
}

/// `S`, `S_u`, `S_v`, `S_uu`, `S_uv`, `S_vv` at `uv` (used by the driver for measurements).
pub fn derivatives(s: &Surface, uv: [f64; 2]) -> Result<[[f64; 3]; 6]> {
    if !crate::finite_all(&uv) {
        return Err(arg("parameters must be finite"));
    }
    let nurb = spline_to_nurb(s)?;
    let sd = nurb
        .derivs(uv[0], uv[1], Side::Left, Side::Left, true, 2, 2)
        .map_err(|e| op_err("srf_nurb::Surface::derivs", format!("{e:?}")))?;
    let at = |k: usize, l: usize| -> Result<[f64; 3]> {
        let v = sd.get(k).and_then(|r| r.get(l)).ok_or_else(|| arg("the derivative table is incomplete"))?;
        Ok([v.x, v.y, v.z])
    };
    Ok([at(0, 0)?, at(1, 0)?, at(0, 1)?, at(2, 0)?, at(1, 1)?, at(0, 2)?])
}

#[cfg(test)]
mod tests;
