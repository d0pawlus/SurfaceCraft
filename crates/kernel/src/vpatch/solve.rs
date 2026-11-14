//! Dense linear algebra of the constrained minimisation: rank-revealing reduction of the hard rows and the solve in their null space. Pure matrix code; the geometry (basis rows, energy) is assembled in the parent module.

use super::cost::svd_fallback_seconds;
use super::{LinRow, MAX_SOLVE_SECONDS, RANK_TOL};
use crate::{Result, op_err, poll};
use nalgebra::{DMatrix, SVD};
use surfacecraft_geom::Cancel;

fn all_finite(m: &DMatrix<f64>) -> bool {
    m.iter().all(|v| v.is_finite())
}

/// Solves `A x = b` for a symmetric positive (semi)definite `A`: Cholesky with two refinement
/// steps; when the factorisation fails, an SVD pseudo-inverse, which is only run where its estimated
/// cost ([`svd_fallback_seconds`]) is within [`MAX_SOLVE_SECONDS`] (otherwise an error says the matrix
/// is singular beyond what the regularisation covers).
pub(super) fn spd_solve(a: &DMatrix<f64>, b: &DMatrix<f64>, cancel: &Cancel) -> Result<DMatrix<f64>> {
    if !all_finite(a) || !all_finite(b) {
        return Err(op_err("vpatch", "the normal matrix is not finite"));
    }
    poll(cancel)?;
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
            let c = sym.nrows();
            let est = svd_fallback_seconds(c);
            if est > MAX_SOLVE_SECONDS {
                return Err(op_err(
                    "vpatch",
                    format!(
                        "the normal matrix of {c} unknowns is singular beyond what the regularisation covers, and the \
                         fallback factorisation would take about {est:.1} s (limit {MAX_SOLVE_SECONDS:.1} s): \
                         use a smaller control net or other conditions"
                    ),
                ));
            }
            poll(cancel)?;
            let niter = 64 * (c + 8);
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

/// The row space of the reduced hard rows: the reduced rows `cm`, their right-hand sides `dm`, and
/// the part of their SVD above the rank tolerance (`vr`: right vectors as rows, `ur`: left vectors,
/// `sig`: singular values).
struct RowSpace {
    cm: DMatrix<f64>,
    dm: DMatrix<f64>,
    vr: DMatrix<f64>,
    ur: DMatrix<f64>,
    sig: Vec<f64>,
}

impl RowSpace {
    fn rank(&self) -> usize {
        self.sig.len()
    }

    /// Pseudo-inverse of the reduced rows applied to a right-hand side.
    fn pinv(&self, rhs: &DMatrix<f64>) -> DMatrix<f64> {
        let mut t = self.ur.transpose() * rhs;
        for (i, s) in self.sig.iter().enumerate() {
            for q in 0..t.ncols() {
                if let Some(slot) = t.get_mut((i, q)) {
                    *slot /= s;
                }
            }
        }
        self.vr.transpose() * t
    }

    /// The least-squares solution of the rows of least norm (two refinement steps).
    fn particular(&self) -> DMatrix<f64> {
        let mut xp = self.pinv(&self.dm);
        for _ in 0..2 {
            let res = &self.dm - &self.cm * &xp;
            xp += self.pinv(&res);
        }
        xp
    }
}

/// Reduces the hard rows and decomposes them; `None` when there are none or none above the tolerance.
fn row_space(rows: &[LinRow], c: usize, k: usize, cancel: &Cancel) -> Result<Option<RowSpace>> {
    let m = reduce_rows(rows, c, k, cancel)?;
    if m.nrows() == 0 {
        return Ok(None);
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
    let keep: Vec<usize> =
        (0..sig.len()).filter(|&i| sig.get(i).is_some_and(|s| *s > RANK_TOL * smax && *s > 0.0)).collect();
    if keep.is_empty() {
        return Ok(None);
    }
    // `keep` lists indices of `sig`; `u` and `vt` come from the SVD of `cm` (dimensions as above)
    let at = |m: &DMatrix<f64>, i: usize, j: usize| m.get((i, j)).copied().unwrap_or(0.0);
    let vr = DMatrix::from_fn(keep.len(), c, |i, j| at(&vt, keep.get(i).copied().unwrap_or(0), j));
    let ur = DMatrix::from_fn(cm.nrows(), keep.len(), |i, j| at(&u, i, keep.get(j).copied().unwrap_or(0)));
    let sig_r: Vec<f64> = keep.iter().filter_map(|&i| sig.get(i).copied()).collect();
    Ok(Some(RowSpace { cm, dm, vr, ur, sig: sig_r }))
}

/// Minimises on `x = xp + z` with `V z = 0` (`V`: row space of the rows): `P H P z = P (b − H xp)`,
/// `P = I − VᵀV`; the block `γ·VᵀV` makes the matrix definite without touching the null space.
/// The cancellation flag is looked at between the dense products.
fn null_space_min(
    h: &DMatrix<f64>,
    b: &DMatrix<f64>,
    vr: &DMatrix<f64>,
    xp: &DMatrix<f64>,
    cancel: &Cancel,
) -> Result<DMatrix<f64>> {
    let c = h.nrows();
    let vt_r = vr.transpose();
    let gamma = {
        let g = (0..c).filter_map(|i| h.get((i, i))).sum::<f64>() / c as f64;
        if g.is_finite() && g > 0.0 { g } else { 1.0 }
    };
    poll(cancel)?;
    let w = vr * h;
    poll(cancel)?;
    let t1 = &vt_r * &w;
    poll(cancel)?;
    let wv = &w * &vt_r;
    poll(cancel)?;
    let t3 = &vt_r * &(&wv * vr);
    poll(cancel)?;
    let vv = &vt_r * vr;
    poll(cancel)?;
    let g = h - &t1 - t1.transpose() + &t3 + &vv * gamma;
    let r0 = b - h * xp;
    let rp = &r0 - &vt_r * &(vr * &r0);
    let z = spd_solve(&g, &rp, cancel)?;
    Ok(&z - &vt_r * &(vr * &z))
}

/// Minimises `½ xᵀ H x − bᵀ x` subject to the rows (`Σ a_j x_j = rhs`, right-hand sides in
/// columns); returns `x` (one column per right-hand side) and the rank of the rows.
///
/// The longest stretch without a look at the cancellation flag is the QR and the SVD of the reduced
/// rows (`row_space_svd`) or one dense product: measured about 2.7 s at the largest accepted net
/// (24 × 24 coupled, 714 hard rows, `ci` profile, aarch64), the rest under 1 s each.
pub(super) fn constrained_min(
    h: &DMatrix<f64>,
    b: &DMatrix<f64>,
    rows: &[LinRow],
    cancel: &Cancel,
) -> Result<(DMatrix<f64>, usize)> {
    let Some(rs) = row_space(rows, h.nrows(), b.ncols(), cancel)? else {
        return Ok((spd_solve(h, b, cancel)?, 0));
    };
    let xp = rs.particular();
    let z = null_space_min(h, b, &rs.vr, &xp, cancel)?;
    let x = &xp + &z;
    if !all_finite(&x) {
        return Err(op_err("vpatch", "the constrained solution is not finite"));
    }
    Ok((x, rs.rank()))
}
