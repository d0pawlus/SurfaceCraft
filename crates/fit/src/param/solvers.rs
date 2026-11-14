//! Sparse iterative solvers of the flattening: Jacobi-preconditioned conjugate gradients and
//! BiCGSTAB on operators given as closures, both stopping early when the caller cancels.
use super::{dot, norm};
use surfacecraft_geom::Cancel;

/// Iterations of a linear solve between two looks at the cancellation flag.
const POLL_EVERY: usize = 16;
/// Relative residual the linear solves aim at.
const SOLVE_TOLERANCE: f64 = 1e-12;

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
pub(super) fn conjugate_gradient(
    apply: &dyn Fn(&[f64], &mut [f64]),
    inv_diag: &[f64],
    b: &[f64],
    x: &mut [f64],
    max_iter: usize,
    cancel: &Cancel,
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
    for k in 0..max_iter {
        if k % POLL_EVERY == 0 && cancel.is_cancelled() {
            return f64::NAN;
        }
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
pub(super) fn bicgstab(
    apply: &dyn Fn(&[f64], &mut [f64]),
    b: &[f64],
    x: &mut [f64],
    max_iter: usize,
    cancel: &Cancel,
) -> f64 {
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
    for k in 0..max_iter {
        if k % POLL_EVERY == 0 && cancel.is_cancelled() {
            return f64::NAN;
        }
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
