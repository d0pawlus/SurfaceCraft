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

/// Working vectors of [`conjugate_gradient`] between iterations.
struct CgState {
    n: usize,
    r: Vec<f64>,
    z: Vec<f64>,
    p: Vec<f64>,
    ap: Vec<f64>,
    rz: f64,
}

impl CgState {
    /// The residual, the preconditioned residual and the first search direction at the start `x`.
    fn start(apply: &dyn Fn(&[f64], &mut [f64]), inv_diag: &[f64], b: &[f64], x: &[f64]) -> Self {
        let n = b.len();
        let mut ap = vec![0.0; n];
        apply(x, &mut ap);
        let r: Vec<f64> = b.iter().zip(&ap).map(|(bi, ai)| bi - ai).collect();
        let z: Vec<f64> = r.iter().zip(inv_diag).map(|(ri, d)| ri * d).collect();
        let p = z.clone();
        let rz = dot(&r, &z);
        Self { n, r, z, p, ap, rz }
    }

    /// One iteration; false on a breakdown (the iteration stops).
    fn step(&mut self, apply: &dyn Fn(&[f64], &mut [f64]), inv_diag: &[f64], x: &mut [f64]) -> bool {
        apply(&self.p, &mut self.ap);
        let pap = dot(&self.p, &self.ap);
        if !(pap.is_finite() && pap > 0.0) {
            return false;
        }
        let alpha = self.rz / pap;
        for i in 0..self.n {
            x[i] += alpha * self.p[i];
            self.r[i] -= alpha * self.ap[i];
            self.z[i] = self.r[i] * inv_diag[i];
        }
        let rz_new = dot(&self.r, &self.z);
        if !(self.rz.is_finite() && self.rz != 0.0 && rz_new.is_finite()) {
            return false;
        }
        let beta = rz_new / self.rz;
        self.rz = rz_new;
        for i in 0..self.n {
            self.p[i] = self.z[i] + beta * self.p[i];
        }
        true
    }
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
    let bn = norm(b);
    if !bn.is_finite() {
        return f64::NAN;
    }
    if bn == 0.0 {
        return 0.0;
    }
    let mut st = CgState::start(apply, inv_diag, b, x);
    for k in 0..max_iter {
        if k % POLL_EVERY == 0 && cancel.is_cancelled() {
            return f64::NAN;
        }
        if norm(&st.r) <= SOLVE_TOLERANCE * bn {
            break;
        }
        if !st.step(apply, inv_diag, x) {
            break;
        }
    }
    relative_residual(apply, b, x)
}

/// Working vectors and scalars of [`bicgstab`] between iterations.
struct BicgState {
    n: usize,
    r: Vec<f64>,
    r_hat: Vec<f64>,
    v: Vec<f64>,
    p: Vec<f64>,
    s: Vec<f64>,
    t: Vec<f64>,
    rho: f64,
    alpha: f64,
    omega: f64,
}

impl BicgState {
    /// The residual at the start `x` (also the shadow residual) and zeroed work vectors.
    fn start(apply: &dyn Fn(&[f64], &mut [f64]), b: &[f64], x: &[f64]) -> Self {
        let n = b.len();
        let mut v = vec![0.0; n];
        apply(x, &mut v);
        let r: Vec<f64> = b.iter().zip(&v).map(|(bi, ai)| bi - ai).collect();
        let r_hat = r.clone();
        v.iter_mut().for_each(|e| *e = 0.0);
        let (p, s, t) = (vec![0.0; n], vec![0.0; n], vec![0.0; n]);
        Self { n, r, r_hat, v, p, s, t, rho: 1.0, alpha: 1.0, omega: 1.0 }
    }

    /// The new search direction `p`, `v = A p`, the step `alpha` and `s = r - alpha v`; false on a
    /// breakdown.
    fn direction(&mut self, apply: &dyn Fn(&[f64], &mut [f64]), rho_new: f64) -> bool {
        let beta = (rho_new / self.rho) * (self.alpha / self.omega);
        for i in 0..self.n {
            self.p[i] = self.r[i] + beta * (self.p[i] - self.omega * self.v[i]);
        }
        apply(&self.p, &mut self.v);
        let rv = dot(&self.r_hat, &self.v);
        if !(rv.is_finite() && rv != 0.0) {
            return false;
        }
        self.alpha = rho_new / rv;
        for i in 0..self.n {
            self.s[i] = self.r[i] - self.alpha * self.v[i];
        }
        true
    }

    /// The stabilising step `omega` along `s`, then the update of `x` and `r`; false on a breakdown.
    fn stabilise(&mut self, apply: &dyn Fn(&[f64], &mut [f64]), x: &mut [f64]) -> bool {
        apply(&self.s, &mut self.t);
        let tt = dot(&self.t, &self.t);
        if !(tt.is_finite() && tt > 0.0) {
            return false;
        }
        self.omega = dot(&self.t, &self.s) / tt;
        if !(self.omega.is_finite() && self.omega != 0.0) {
            return false;
        }
        for i in 0..self.n {
            x[i] += self.alpha * self.p[i] + self.omega * self.s[i];
            self.r[i] = self.s[i] - self.omega * self.t[i];
        }
        true
    }

    /// One iteration; false when the iteration stops (converged on `s`, or a breakdown).
    fn step(&mut self, apply: &dyn Fn(&[f64], &mut [f64]), bn: f64, x: &mut [f64]) -> bool {
        let rho_new = dot(&self.r_hat, &self.r);
        if !(rho_new.is_finite() && rho_new != 0.0 && self.omega != 0.0) {
            return false;
        }
        if !self.direction(apply, rho_new) {
            return false;
        }
        if norm(&self.s) <= SOLVE_TOLERANCE * bn {
            for i in 0..self.n {
                x[i] += self.alpha * self.p[i];
            }
            return false;
        }
        if !self.stabilise(apply, x) {
            return false;
        }
        self.rho = rho_new;
        true
    }
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
    let bn = norm(b);
    if !bn.is_finite() {
        return f64::NAN;
    }
    if bn == 0.0 {
        return 0.0;
    }
    let mut st = BicgState::start(apply, b, x);
    for k in 0..max_iter {
        if k % POLL_EVERY == 0 && cancel.is_cancelled() {
            return f64::NAN;
        }
        if norm(&st.r) <= SOLVE_TOLERANCE * bn {
            break;
        }
        if !st.step(apply, bn, x) {
            break;
        }
    }
    relative_residual(apply, b, x)
}
