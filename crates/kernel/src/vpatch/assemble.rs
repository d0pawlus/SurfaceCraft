//! Assembly of the normal equations and the read-out of the solution: data and soft rows, the
//! stiffness, the coupled layout, the hard residual and the control net.
use super::{
    DIAG_EPS, Grid, LinRow, POLL_EVERY, Problem, REGULARIZE, Row, Scale, Stencil, TINY, accumulate, bounded, build_rows,
};
use crate::{Result, op_err, poll};
use nalgebra::DMatrix;
use surfacecraft_geom::Cancel;

/// Data and soft rows: the scalar normal matrix and right-hand sides of the positions, the
/// coupled soft rows, and the traces that scale the bending energy.
pub(super) struct SoftTerms {
    pub(super) a: DMatrix<f64>,
    pub(super) b: DMatrix<f64>,
    pub(super) coupled: Vec<(LinRow, f64)>,
    /// Trace of the data's normal matrix.
    pub(super) tr_data: f64,
    /// Trace of the soft rows' normal matrix.
    pub(super) tr_soft: f64,
}

/// A soft tangent or curvature row at unit Euclidean length (its right-hand side with it): its
/// coefficients scale with 1/cell and 1/cell², and its weight against the positions must not
/// depend on the units of the parameters. `None` for a row without length.
fn unit_row(mut lr: LinRow) -> Option<LinRow> {
    let len = lr.cols.iter().map(|(_, a)| a * a).sum::<f64>().sqrt();
    if !(len.is_finite() && len > TINY) {
        return None;
    }
    for c in lr.cols.iter_mut() {
        c.1 /= len;
    }
    lr.rhs[0] /= len;
    Some(lr)
}

pub(super) fn assemble_soft(g: &Grid, scale: &Scale, p: &Problem, cancel: &Cancel) -> Result<SoftTerms> {
    let nn = g.count();
    let mut a = DMatrix::<f64>::zeros(nn, nn);
    let mut b = DMatrix::<f64>::zeros(nn, 3);
    // the point's contribution to a trace; the caller decides whether it counts as data
    let mut add_position = |uv: [f64; 2], at: [f64; 3], w: f64| -> Result<f64> {
        let st = Stencil::new(g, g.clamp(uv))?;
        let phi = st.coefficient(0, 0);
        accumulate(&mut a, &mut b, &st.indices(g), &phi, scale.forward(at), w);
        Ok(3.0 * w * phi.iter().map(|v| v * v).sum::<f64>())
    };
    // the energy weight is scaled by the trace of the DATA normal matrix only; soft rows (which
    // may be numerous and heavy) must not stiffen the fit, so they are traced separately
    let mut tr_data = 0.0;
    for (i, d) in p.data.iter().enumerate() {
        if i % POLL_EVERY == 0 {
            poll(cancel)?;
        }
        tr_data += add_position(d.uv, d.at, d.weight)?;
    }
    let mut coupled: Vec<(LinRow, f64)> = Vec::new();
    let mut tr_soft = 0.0;
    for (i, (row, w)) in p.soft.iter().enumerate() {
        if i % POLL_EVERY == 0 {
            poll(cancel)?;
        }
        if let Row::Position { uv, at } = row {
            tr_soft += add_position(*uv, *at, *w)?;
            continue;
        }
        for lr in build_rows(g, scale, row, true)?.into_iter().filter_map(unit_row) {
            tr_soft += *w;
            coupled.push((lr, *w));
        }
    }
    Ok(SoftTerms { a, b, coupled, tr_data, tr_soft })
}

/// `H = A + λK` with the diagonal that keeps the unknowns of inactive cells and the whole matrix definite.
pub(super) fn stiffness(a: &DMatrix<f64>, kmat: &DMatrix<f64>, lambda: f64, inactive: &[bool]) -> DMatrix<f64> {
    let mut h = a + kmat * lambda;
    let nn = h.nrows();
    let diag = |m: &DMatrix<f64>, i: usize| m.get((i, i)).copied().unwrap_or(0.0);
    let mean_h = (0..nn).map(|i| diag(&h, i)).sum::<f64>() / nn.max(1) as f64;
    let base = if mean_h.is_finite() && mean_h > 0.0 { mean_h } else { 1.0 };
    let mean_e = lambda * (0..nn).map(|i| diag(kmat, i)).sum::<f64>() / nn.max(1) as f64;
    let reg_inactive = REGULARIZE * if mean_e.is_finite() && mean_e > 0.0 { mean_e } else { base };
    for i in 0..nn {
        let extra = DIAG_EPS * base + if inactive.get(i).copied().unwrap_or(false) { reg_inactive } else { 0.0 };
        if let Some(slot) = h.get_mut((i, i)) {
            *slot += extra;
        }
    }
    h
}

/// The coupled layout `x[3·i + c]`: `H` repeated on the three coordinates, the soft tangent and
/// curvature rows added; returns the matrix and the right-hand side.
pub(super) fn expand_coupled(
    h: &DMatrix<f64>,
    b: &DMatrix<f64>,
    soft: &[(LinRow, f64)],
    cancel: &Cancel,
) -> Result<(DMatrix<f64>, DMatrix<f64>)> {
    let nn = h.nrows();
    let c = 3 * nn;
    let mut h3 = DMatrix::<f64>::zeros(c, c);
    let mut b3 = DMatrix::<f64>::zeros(c, 1);
    for a in 0..nn {
        poll(cancel)?;
        for bb in 0..nn {
            let v = h.get((a, bb)).copied().unwrap_or(0.0);
            if v == 0.0 {
                continue;
            }
            for q in 0..3 {
                if let Some(slot) = h3.get_mut((3 * a + q, 3 * bb + q)) {
                    *slot = v;
                }
            }
        }
        for q in 0..3 {
            if let (Some(slot), Some(v)) = (b3.get_mut((3 * a + q, 0)), b.get((a, q))) {
                *slot = *v;
            }
        }
    }
    for (row, w) in soft {
        for &(i, ai) in &row.cols {
            for &(j, aj) in &row.cols {
                if let Some(slot) = h3.get_mut((i, j)) {
                    *slot += w * ai * aj;
                }
            }
            if let Some(slot) = b3.get_mut((i, 0)) {
                *slot += w * row.rhs[0] * ai;
            }
        }
    }
    Ok((h3, b3))
}

/// Largest `|row·x − rhs|` over the hard rows, in the units of the input.
pub(super) fn hard_residual(rows: &[LinRow], x: &DMatrix<f64>, unit: usize, scale: &Scale) -> Result<f64> {
    let mut residual = 0.0_f64;
    for r in rows {
        for q in 0..unit {
            let lhs: f64 = r.cols.iter().map(|&(j, a)| a * x.get((j, q)).copied().unwrap_or(f64::NAN)).sum();
            let d = (lhs - r.rhs[q]).abs();
            if !d.is_finite() {
                return Err(op_err("vpatch", "the hard rows could not be evaluated"));
            }
            residual = residual.max(d);
        }
    }
    Ok(residual / scale.factor)
}

/// The control net from the solution `x`, back in the units of the input.
pub(super) fn control_net(g: &Grid, x: &DMatrix<f64>, coupled: bool, scale: &Scale) -> Result<Vec<Vec<[f64; 3]>>> {
    let (n, m) = (g.control[0], g.control[1]);
    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        let mut line = Vec::with_capacity(m);
        for j in 0..m {
            let a = i * m + j;
            let mut pt = [0.0; 3];
            for (q, slot) in pt.iter_mut().enumerate() {
                let at = if coupled { (3 * a + q, 0) } else { (a, q) };
                let v = x.get(at).copied().ok_or_else(|| op_err("vpatch", "the solution is too short"))?;
                *slot = v / scale.factor + scale.center[q];
            }
            if !bounded(&pt) {
                return Err(op_err("vpatch", "the fitted control point is not finite"));
            }
            line.push(pt);
        }
        points.push(line);
    }
    Ok(points)
}
