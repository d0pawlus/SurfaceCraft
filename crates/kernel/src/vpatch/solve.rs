//! Dense linear algebra of the constrained minimisation: rank-revealing reduction of the hard rows and the solve in their null space. Pure matrix code; the geometry (basis rows, energy) is assembled in the parent module.

use super::{LinRow, RANK_TOL};
use crate::{Result, op_err, poll};
use surfacecraft_geom::Cancel;
use nalgebra::{DMatrix, SVD};

fn all_finite(m: &DMatrix<f64>) -> bool {
    m.iter().all(|v| v.is_finite())
}

/// Solves `A x = b` for a symmetric positive (semi)definite `A`: Cholesky with two refinement
/// steps, SVD pseudo-inverse when the factorisation fails.
fn spd_solve(a: &DMatrix<f64>, b: &DMatrix<f64>) -> Result<DMatrix<f64>> {
    if !all_finite(a) || !all_finite(b) {
        return Err(op_err("vpatch", "the normal matrix is not finite"));
    }
    let sym = (a + a.transpose()) * 0.5;
    let x = match sym.clone().cholesky() {
        Some(ch) => {
            let mut x = ch.solve(b);
            for _ in 0..2 {
                let res = b - &sym * &x;
                x += ch.solve(&res);
            }
            x
        }
        None => {
            let niter = 64 * (sym.nrows() + 8);
            let svd = SVD::try_new_unordered(sym.clone(), true, true, f64::EPSILON, niter)
                .ok_or_else(|| op_err("vpatch", "the SVD of the normal matrix did not converge"))?;
            let smax = svd.singular_values.iter().copied().fold(0.0_f64, f64::max);
            svd.solve(b, smax * 1e-13).map_err(|e| op_err("vpatch", e))?
        }
    };
    if all_finite(&x) { Ok(x) } else { Err(op_err("vpatch", "the solution is not finite")) }
}

/// The rows (scaled to unit norm) and their right-hand sides reduced to at most `c` rows with the
/// same normal equations: `[C | D]` with `C` over the `c` unknowns and `D` the `k` right-hand sides.
fn reduce_rows(rows: &[LinRow], c: usize, k: usize, cancel: &Cancel) -> Result<DMatrix<f64>> {
    let w = c + k;
    let usable: Vec<(&LinRow, f64)> = rows
        .iter()
        .filter_map(|r| {
            let norm = r.cols.iter().map(|(_, a)| a * a).sum::<f64>().sqrt();
            if norm.is_finite() && norm > 0.0 { Some((r, 1.0 / norm)) } else { None }
        })
        .collect();
    let mut acc = DMatrix::<f64>::zeros(0, w);
    for chunk in usable.chunks(c.max(1)) {
        poll(cancel)?;
        let top = acc.nrows();
        let mut st = DMatrix::<f64>::zeros(top + chunk.len(), w);
        if top > 0 {
            st.view_mut((0, 0), (top, w)).copy_from(&acc);
        }
        for (q, (row, inv)) in chunk.iter().enumerate() {
            for &(j, a) in &row.cols {
                if let Some(slot) = st.get_mut((top + q, j)) {
                    *slot += a * inv;
                }
            }
            for t in 0..k {
                if let (Some(slot), Some(v)) = (st.get_mut((top + q, c + t)), row.rhs.get(t)) {
                    *slot = v * inv;
                }
            }
        }
        acc = if st.nrows() > c {
            let r = st.qr().r();
            r.rows(0, c.min(r.nrows())).into_owned()
        } else {
            st
        };
    }
    Ok(acc)
}

/// Singular values, left vectors and right vectors (as rows) of the wide matrix `a` (`m × c`,
/// `m ≤ c`), through a QR of `aᵀ` and the SVD of its small triangular factor: `aᵀ = Q R`,
/// `Rᵀ = U S Wᵀ`, so `a = U S (Q W)ᵀ`. The same singular system as the SVD of `a`, at the cost of
/// `m²·c` instead of a bidiagonalisation that carries `c`-wide factors.
fn row_space_svd(a: &DMatrix<f64>) -> Result<(Vec<f64>, DMatrix<f64>, DMatrix<f64>)> {
    let m = a.nrows();
    let qr = a.transpose().qr();
    let (q, r) = (qr.q(), qr.r());
    let niter = 64 * (m + 8);
    let svd = SVD::try_new_unordered(r.transpose(), true, true, f64::EPSILON, niter)
        .ok_or_else(|| op_err("vpatch", "the SVD of the constraint rows did not converge"))?;
    let (Some(u), Some(wt)) = (svd.u, svd.v_t) else {
        return Err(op_err("vpatch", "the SVD of the constraint rows has no singular vectors"));
    };
    let vt = wt * q.transpose();
    Ok((svd.singular_values.iter().copied().collect(), u, vt))
}

/// Minimises `½ xᵀ H x − bᵀ x` subject to the rows (`Σ a_j x_j = rhs`, right-hand sides in
/// columns); returns `x` (one column per right-hand side) and the rank of the rows.
pub(super) fn constrained_min(
    h: &DMatrix<f64>,
    b: &DMatrix<f64>,
    rows: &[LinRow],
    cancel: &Cancel,
) -> Result<(DMatrix<f64>, usize)> {
    let c = h.nrows();
    let k = b.ncols();
    let m = reduce_rows(rows, c, k, cancel)?;
    if m.nrows() == 0 {
        return Ok((spd_solve(h, b)?, 0));
    }
    if !all_finite(&m) {
        return Err(op_err("vpatch", "the constraint rows are not finite"));
    }
    let cm = m.columns(0, c).into_owned();
    let dm = m.columns(c, k).into_owned();
    poll(cancel)?;
    let (sig, u, vt) = row_space_svd(&cm)?;
    poll(cancel)?;
    let smax = sig.iter().copied().fold(0.0_f64, f64::max);
    let keep: Vec<usize> = (0..sig.len()).filter(|&i| sig[i] > RANK_TOL * smax && sig[i] > 0.0).collect();
    let r = keep.len();
    if r == 0 {
        return Ok((spd_solve(h, b)?, 0));
    }
    let sig_r: Vec<f64> = keep.iter().map(|&i| sig[i]).collect();
    let vr = DMatrix::from_fn(r, c, |i, j| vt[(keep[i], j)]);
    let ur = DMatrix::from_fn(cm.nrows(), r, |i, j| u[(i, keep[j])]);
    // pseudo-inverse of the (reduced) rows applied to a right-hand side
    let pinv = |rhs: &DMatrix<f64>| -> DMatrix<f64> {
        let mut t = ur.transpose() * rhs;
        for (i, s) in sig_r.iter().enumerate() {
            for q in 0..k {
                t[(i, q)] /= s;
            }
        }
        vr.transpose() * t
    };
    let mut xp = pinv(&dm);
    for _ in 0..2 {
        let res = &dm - &cm * &xp;
        xp += pinv(&res);
    }
    // minimise on x = xp + z with V z = 0 (V: row space of the rows): P H P z = P (b − H xp),
    // P = I − VᵀV; the block γ·VᵀV makes the matrix definite without touching the null space
    let vt_r = vr.transpose();
    poll(cancel)?;
    let gamma = {
        let g = (0..c).map(|i| h[(i, i)]).sum::<f64>() / c as f64;
        if g.is_finite() && g > 0.0 { g } else { 1.0 }
    };
    let w = &vr * h;
    let t1 = &vt_r * &w;
    let wv = &w * &vt_r;
    let t3 = &vt_r * &(&wv * &vr);
    let vv = &vt_r * &vr;
    let g = h - &t1 - t1.transpose() + &t3 + &vv * gamma;
    let r0 = b - h * &xp;
    let rp = &r0 - &vt_r * &(&vr * &r0);
    poll(cancel)?;
    let z = spd_solve(&g, &rp)?;
    let z = &z - &vt_r * &(&vr * &z);
    let x = &xp + &z;
    if !all_finite(&x) {
        return Err(op_err("vpatch", "the constrained solution is not finite"));
    }
    Ok((x, r))
}
