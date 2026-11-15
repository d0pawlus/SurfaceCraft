//! A small Levenberg–Marquardt solver for geometric fits (design doc §10: chosen over the
//! `levenberg-marquardt` and `argmin` crates — a dozen parameters at most, dense normal
//! equations, analytic residuals; one screen of code instead of a dependency).
use nalgebra::{DMatrix, DVector};

/// Result of a minimisation.
#[derive(Clone, Debug, PartialEq)]
pub struct LmResult {
    /// Final parameters.
    pub params: DVector<f64>,
    /// Final sum of squared residuals.
    pub cost: f64,
    /// Iterations run.
    pub iterations: usize,
}

/// Working state of [`minimize`] between iterations.
struct LmState {
    p: DVector<f64>,
    r: DVector<f64>,
    rp: DVector<f64>,
    rm: DVector<f64>,
    jac: DMatrix<f64>,
    cost: f64,
    lambda: f64,
}

impl LmState {
    /// Fills the Jacobian with central differences scaled to each parameter.
    fn jacobian(&mut self, residuals: &mut impl FnMut(&DVector<f64>, &mut DVector<f64>)) {
        for j in 0..self.p.len() {
            let h = 1e-7 * (1.0 + self.p[j].abs());
            let mut q = self.p.clone();
            q[j] += h;
            residuals(&q, &mut self.rp);
            q[j] -= 2.0 * h;
            residuals(&q, &mut self.rm);
            let col = (&self.rp - &self.rm) / (2.0 * h);
            self.jac.set_column(j, &col);
        }
    }

    /// Tries increasingly damped steps until one lowers the cost; true when the iterations go on
    /// (a step was taken and it was not negligible).
    fn damped_step(
        &mut self,
        jtj: &DMatrix<f64>,
        g: &DVector<f64>,
        residuals: &mut impl FnMut(&DVector<f64>, &mut DVector<f64>),
    ) -> bool {
        let n = self.p.len();
        for _ in 0..30 {
            let mut a = jtj.clone();
            for k in 0..n {
                a[(k, k)] += self.lambda * (jtj[(k, k)].abs() + 1e-12);
            }
            let Some(step) = a.lu().solve(&(-g)) else {
                self.lambda *= 10.0;
                continue;
            };
            let q = &self.p + &step;
            residuals(&q, &mut self.rp);
            let c = self.rp.norm_squared();
            if c.is_finite() && c < self.cost {
                let small = step.norm() < 1e-14 * (1.0 + self.p.norm());
                let rel = (self.cost - c) / self.cost.max(1e-300);
                self.p = q;
                std::mem::swap(&mut self.r, &mut self.rp);
                self.cost = c;
                self.lambda = (self.lambda * 0.3).max(1e-15);
                return !(small || rel < 1e-15);
            }
            self.lambda *= 10.0;
            if self.lambda > 1e16 {
                break;
            }
        }
        false
    }
}

/// Minimises `Σ rᵢ(p)²`. `residuals(p, out)` fills `out` (length `m`); the Jacobian is taken by
/// central differences scaled to each parameter. Returns `None` when the residuals are not finite
/// at the start.
pub fn minimize(
    p0: DVector<f64>,
    m: usize,
    max_iter: usize,
    mut residuals: impl FnMut(&DVector<f64>, &mut DVector<f64>),
) -> Option<LmResult> {
    let n = p0.len();
    let mut r = DVector::zeros(m);
    residuals(&p0, &mut r);
    let cost = r.norm_squared();
    if !cost.is_finite() {
        return None;
    }
    let (rp, rm, jac) = (DVector::zeros(m), DVector::zeros(m), DMatrix::zeros(m, n));
    let mut s = LmState { p: p0, r, rp, rm, jac, cost, lambda: 1e-3 };
    let mut it = 0;
    while it < max_iter {
        it += 1;
        s.jacobian(&mut residuals);
        let jt = s.jac.transpose();
        let jtj = &jt * &s.jac;
        let g = &jt * &s.r;
        if g.amax() < 1e-15 * (1.0 + s.cost) {
            break;
        }
        if !s.damped_step(&jtj, &g, &mut residuals) {
            break;
        }
    }
    crate::stats::lm_run(it);
    Some(LmResult { params: s.p, cost: s.cost, iterations: it })
}

/// The Huber-robustified form of a residual: `r` while `|r| ≤ k`, then `sign(r)·√(k(2|r| − k))`,
/// so that the squared residual grows linearly (the Huber loss) and the first derivative is
/// continuous. A solver that minimises `Σ rᵢ²` of these residuals minimises the Huber loss.
#[must_use]
pub fn robust_residual(r: f64, k: f64) -> f64 {
    let a = r.abs();
    if !(k > 0.0) || a <= k || !a.is_finite() {
        return r;
    }
    (k * (2.0 * a - k)).sqrt().copysign(r)
}

/// Settings of [`minimize_bounded`].
#[derive(Clone, Debug)]
pub struct BoundedOptions {
    /// Lower bound of each parameter.
    pub lower: Vec<f64>,
    /// Upper bound of each parameter.
    pub upper: Vec<f64>,
    /// Finite-difference step of each parameter (the residuals may be only piecewise smooth, as
    /// a replayed CAD model is, so this is an absolute step, not a tiny relative one).
    pub step: Vec<f64>,
    /// Iterations (Jacobians) at most.
    pub max_iter: usize,
    /// Damping retries per iteration at most.
    pub max_trials: usize,
    /// Central differences (two evaluations per parameter) instead of one-sided ones.
    pub central: bool,
    /// Stop when no parameter moves more than this.
    pub tol_step: f64,
    /// Stop when the cost falls by less than this fraction.
    pub tol_cost: f64,
}

/// Result of [`minimize_bounded`].
#[derive(Clone, Debug, PartialEq)]
pub struct BoundedResult {
    /// Final parameters (inside the bounds).
    pub params: Vec<f64>,
    /// Sum of squared residuals at the start.
    pub initial_cost: f64,
    /// Sum of squared residuals at the end.
    pub cost: f64,
    /// Iterations run.
    pub iterations: usize,
    /// Residual evaluations made.
    pub evaluations: usize,
    /// Per parameter, the root-mean-square change of a residual per unit of the parameter at the
    /// last Jacobian (zero: the residuals do not depend on it).
    pub sensitivity: Vec<f64>,
    /// Did it stop because the step or the cost change fell below its tolerance (or the gradient
    /// vanished), not because it ran out of iterations?
    pub converged: bool,
}

/// A residual function that counts its evaluations.
struct Counted<F> {
    f: F,
    evaluations: usize,
}

impl<F: FnMut(&DVector<f64>, &mut DVector<f64>)> Counted<F> {
    /// Evaluates the residuals at `q` into `out`.
    fn eval(&mut self, q: &DVector<f64>, out: &mut DVector<f64>) {
        (self.f)(q, out);
        self.evaluations += 1;
    }

    /// Evaluates the residuals at `p` with parameter `j` moved by `delta`; true when all are finite.
    fn eval_shifted(&mut self, p: &DVector<f64>, j: usize, delta: f64, out: &mut DVector<f64>) -> bool {
        let mut q = p.clone();
        q[j] += delta;
        self.eval(&q, out);
        out.iter().all(|v| v.is_finite())
    }
}

/// Are the sizes and the bounds of a bounded minimisation consistent?
fn bounded_settings_valid(n: usize, m: usize, opts: &BoundedOptions) -> bool {
    if n == 0 || m == 0 || opts.lower.len() != n || opts.upper.len() != n || opts.step.len() != n {
        return false;
    }
    let (lo, hi) = (&opts.lower, &opts.upper);
    !(0..n).any(|k| !(lo[k] <= hi[k]) || !(opts.step[k] > 0.0) || !lo[k].is_finite() || !hi[k].is_finite())
}

/// Clamps `v` into the box of `opts`.
fn clamp_box(opts: &BoundedOptions, v: &DVector<f64>) -> DVector<f64> {
    DVector::from_fn(v.len(), |k, _| v[k].clamp(opts.lower[k], opts.upper[k]))
}

/// Working state of [`minimize_bounded`] between iterations.
struct BoundedLm<'a, F> {
    opts: &'a BoundedOptions,
    res: Counted<F>,
    p: DVector<f64>,
    r: DVector<f64>,
    rp: DVector<f64>,
    rm: DVector<f64>,
    jac: DMatrix<f64>,
    cost: f64,
    initial_cost: f64,
    lambda: f64,
    converged: bool,
}

impl<'a, F: FnMut(&DVector<f64>, &mut DVector<f64>)> BoundedLm<'a, F> {
    /// Evaluates the clamped start; `None` when the residuals are not finite there.
    fn start(opts: &'a BoundedOptions, m: usize, p0: &[f64], f: F) -> Option<Self> {
        let p = clamp_box(opts, &DVector::from_column_slice(p0));
        let mut res = Counted { f, evaluations: 0 };
        let mut r = DVector::zeros(m);
        res.eval(&p, &mut r);
        let cost = r.norm_squared();
        if !cost.is_finite() {
            return None;
        }
        let (rp, rm, jac) = (DVector::zeros(m), DVector::zeros(m), DMatrix::zeros(m, p0.len()));
        Some(Self { opts, res, p, r, rp, rm, jac, cost, initial_cost: cost, lambda: 1e-3, converged: false })
    }

    /// Fills the Jacobian with one-sided or central differences that stay inside the bounds.
    fn jacobian(&mut self) {
        let m = self.r.len();
        for j in 0..self.p.len() {
            let h = self.opts.step[j];
            let up = self.p[j] + h <= self.opts.upper[j] + 1e-12;
            let down = self.p[j] - h >= self.opts.lower[j] - 1e-12;
            let fwd = up && self.res.eval_shifted(&self.p, j, h, &mut self.rp);
            let bwd = down && (self.opts.central || !fwd) && self.res.eval_shifted(&self.p, j, -h, &mut self.rm);
            let col = match (fwd, bwd) {
                (true, true) if self.opts.central => (&self.rp - &self.rm) / (2.0 * h),
                (true, _) => (&self.rp - &self.r) / h,
                (false, true) => (&self.r - &self.rm) / h,
                _ => DVector::zeros(m),
            };
            self.jac.set_column(j, &col);
        }
    }

    /// The parameters not pressed against a bound by the descent direction `g`; `None` when there
    /// are none or the gradient on them has vanished.
    fn free_set(&self, g: &DVector<f64>) -> Option<Vec<usize>> {
        let (lo, hi) = (&self.opts.lower, &self.opts.upper);
        let p = &self.p;
        let free: Vec<usize> = (0..p.len())
            .filter(|&k| !((p[k] <= lo[k] + 1e-12 && g[k] > 0.0) || (p[k] >= hi[k] - 1e-12 && g[k] < 0.0)))
            .collect();
        let gnorm = free.iter().map(|&k| g[k].abs()).fold(0.0, f64::max);
        if free.is_empty() || gnorm < 1e-12 * (1.0 + self.cost) { None } else { Some(free) }
    }

    /// Solves the damped normal equations restricted to the free parameters.
    fn solve_step(&self, jtj: &DMatrix<f64>, g: &DVector<f64>, free: &[usize]) -> Option<DVector<f64>> {
        let nf = free.len();
        let mut a = DMatrix::zeros(nf, nf);
        let mut b = DVector::zeros(nf);
        for (ia, &ka) in free.iter().enumerate() {
            b[ia] = -g[ka];
            for (ib, &kb) in free.iter().enumerate() {
                a[(ia, ib)] = jtj[(ka, kb)];
            }
            a[(ia, ia)] += self.lambda * (jtj[(ka, ka)].abs() + 1e-12);
        }
        a.lu().solve(&b)
    }

    /// The trial point: the free parameters moved by `step`, clamped into the box.
    fn trial_point(&self, step: &DVector<f64>, free: &[usize]) -> DVector<f64> {
        let mut q = self.p.clone();
        for (ia, &ka) in free.iter().enumerate() {
            q[ka] += step[ia];
        }
        clamp_box(self.opts, &q)
    }

    /// Takes the trial point `q` of cost `c`, relaxes the damping and checks the tolerances.
    fn accept(&mut self, q: DVector<f64>, c: f64) {
        let moved = (&q - &self.p).amax();
        let rel = (self.cost - c) / self.cost.max(1e-300);
        self.p = q;
        std::mem::swap(&mut self.r, &mut self.rp);
        self.cost = c;
        self.lambda = (self.lambda * 0.3).max(1e-9);
        if moved < self.opts.tol_step || rel < self.opts.tol_cost {
            self.converged = true;
        }
    }

    /// Tries increasingly damped steps until one lowers the cost; true when one was taken.
    fn damped_step(&mut self, jtj: &DMatrix<f64>, g: &DVector<f64>, free: &[usize]) -> bool {
        for _ in 0..self.opts.max_trials.max(1) {
            let Some(step) = self.solve_step(jtj, g, free) else {
                self.lambda *= 10.0;
                continue;
            };
            let q = self.trial_point(&step, free);
            self.res.eval(&q, &mut self.rp);
            let c = self.rp.norm_squared();
            if c.is_finite() && c < self.cost {
                self.accept(q, c);
                return true;
            }
            self.lambda *= 10.0;
            if self.lambda > 1e12 {
                break;
            }
        }
        false
    }

    /// The result after `it` iterations.
    fn finish(self, it: usize) -> BoundedResult {
        let m = self.r.len();
        let sensitivity: Vec<f64> = (0..self.p.len()).map(|k| self.jac.column(k).norm() / (m as f64).sqrt()).collect();
        crate::stats::lm_run(it);
        BoundedResult {
            params: self.p.as_slice().to_vec(),
            initial_cost: self.initial_cost,
            cost: self.cost,
            iterations: it,
            evaluations: self.res.evaluations,
            sensitivity,
            converged: self.converged,
        }
    }
}

/// Levenberg–Marquardt with box bounds for expensive, piecewise-smooth residuals (a replayed
/// feature tree): the Jacobian is one-sided or central differences with an absolute step that
/// never leaves the bounds, parameters held at a bound by the gradient drop out of the step
/// (an active set), and the step is clamped into the box. `residuals(p, out)` fills `out`
/// (length `m`) and writes a non-finite value when the model cannot be evaluated at `p`: such a
/// trial is rejected, and a Jacobian column falls back to the other side or to zero. Returns
/// `None` when the start cannot be evaluated or the settings do not fit.
pub fn minimize_bounded(
    p0: &[f64],
    m: usize,
    opts: &BoundedOptions,
    mut residuals: impl FnMut(&[f64], &mut [f64]),
) -> Option<BoundedResult> {
    if !bounded_settings_valid(p0.len(), m, opts) {
        return None;
    }
    let wrapped = |p: &DVector<f64>, out: &mut DVector<f64>| residuals(p.as_slice(), out.as_mut_slice());
    let mut s = BoundedLm::start(opts, m, p0, wrapped)?;
    let mut it = 0;
    while it < opts.max_iter {
        it += 1;
        s.jacobian();
        let jt = s.jac.transpose();
        let jtj = &jt * &s.jac;
        let g = &jt * &s.r;
        // parameters pressed against a bound by the descent direction are held
        let Some(free) = s.free_set(&g) else {
            s.converged = true;
            break;
        };
        if !s.damped_step(&jtj, &g, &free) {
            // no step lowers the cost: a minimum as far as these differences can tell
            s.converged = true;
        }
        if s.converged {
            break;
        }
    }
    Some(s.finish(it))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn robust_residual_is_the_huber_loss() {
        assert_eq!(robust_residual(0.3, 0.5), 0.3);
        // continuous at k and linear in the loss beyond it: ρ(r) = k|r| − k²/2
        assert!((robust_residual(0.5, 0.5) - 0.5).abs() < 1e-12);
        let r = robust_residual(-4.0, 0.5);
        assert!((0.5 * r * r - (0.5 * 4.0 - 0.125)).abs() < 1e-12 && r < 0.0);
    }

    #[test]
    fn bounded_lm_respects_bounds_and_fits_a_line() {
        // y = 2x + 1 fitted for slope a and intercept b, with a capped at 1.5
        let xs: Vec<f64> = (0..20).map(f64::from).collect();
        let ys: Vec<f64> = xs.iter().map(|x| 2.0 * x + 1.0).collect();
        let res = |p: &[f64], out: &mut [f64]| {
            for (i, (x, y)) in xs.iter().zip(&ys).enumerate() {
                out[i] = p[0] * x + p[1] - y;
            }
        };
        let mk = |hi_a: f64| BoundedOptions {
            lower: vec![0.0, -10.0],
            upper: vec![hi_a, 10.0],
            step: vec![0.01, 0.01],
            max_iter: 50,
            max_trials: 12,
            central: false,
            tol_step: 1e-9,
            tol_cost: 1e-12,
        };
        let free = minimize_bounded(&[1.0, 0.0], 20, &mk(5.0), res).unwrap();
        assert!((free.params[0] - 2.0).abs() < 1e-6 && (free.params[1] - 1.0).abs() < 1e-5, "{free:?}");
        assert!(free.converged && free.cost < 1e-9 && free.sensitivity[0] > free.sensitivity[1]);
        let capped = minimize_bounded(&[1.0, 0.0], 20, &mk(1.5), res).unwrap();
        assert!((capped.params[0] - 1.5).abs() < 1e-12, "{capped:?}");
        assert!(capped.params[1] > 1.0, "the intercept makes up for the capped slope");
    }

    #[test]
    fn bounded_lm_survives_unevaluable_trials() {
        // the model fails (NaN) beyond p = 3: the minimum at 5 is out of reach, the solver stops at the edge
        let res = |p: &[f64], out: &mut [f64]| {
            out[0] = if p[0] > 3.0 { f64::NAN } else { p[0] - 5.0 };
        };
        let opts = BoundedOptions {
            lower: vec![0.0],
            upper: vec![10.0],
            step: vec![0.01],
            max_iter: 30,
            max_trials: 8,
            central: true,
            tol_step: 1e-9,
            tol_cost: 1e-12,
        };
        let r = minimize_bounded(&[1.0], 1, &opts, res).unwrap();
        assert!(r.params[0] <= 3.0 && r.params[0] > 1.0 && r.cost < 16.0, "{r:?}");
        let none = minimize_bounded(&[4.0], 1, &opts, res);
        assert!(none.is_none());
    }

    #[test]
    fn rosenbrock() {
        let r = minimize(DVector::from_vec(vec![-1.2, 1.0]), 2, 200, |p, out| {
            out[0] = 10.0 * (p[1] - p[0] * p[0]);
            out[1] = 1.0 - p[0];
        })
        .unwrap();
        assert!((r.params[0] - 1.0).abs() < 1e-8 && (r.params[1] - 1.0).abs() < 1e-8, "{r:?}");
    }
}
