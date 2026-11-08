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
    let mut p = p0;
    let mut r = DVector::zeros(m);
    residuals(&p, &mut r);
    let mut cost = r.norm_squared();
    if !cost.is_finite() {
        return None;
    }
    let mut lambda = 1e-3;
    let mut jac = DMatrix::zeros(m, n);
    let mut rp = DVector::zeros(m);
    let mut rm = DVector::zeros(m);
    let mut it = 0;
    while it < max_iter {
        it += 1;
        for j in 0..n {
            let h = 1e-7 * (1.0 + p[j].abs());
            let mut q = p.clone();
            q[j] += h;
            residuals(&q, &mut rp);
            q[j] -= 2.0 * h;
            residuals(&q, &mut rm);
            let col = (&rp - &rm) / (2.0 * h);
            jac.set_column(j, &col);
        }
        let jt = jac.transpose();
        let jtj = &jt * &jac;
        let g = &jt * &r;
        if g.amax() < 1e-15 * (1.0 + cost) {
            break;
        }
        let mut improved = false;
        for _ in 0..30 {
            let mut a = jtj.clone();
            for k in 0..n {
                a[(k, k)] += lambda * (jtj[(k, k)].abs() + 1e-12);
            }
            let Some(step) = a.lu().solve(&(-&g)) else {
                lambda *= 10.0;
                continue;
            };
            let q = &p + &step;
            residuals(&q, &mut rp);
            let c = rp.norm_squared();
            if c.is_finite() && c < cost {
                let small = step.norm() < 1e-14 * (1.0 + p.norm());
                let rel = (cost - c) / cost.max(1e-300);
                p = q;
                std::mem::swap(&mut r, &mut rp);
                cost = c;
                lambda = (lambda * 0.3).max(1e-15);
                improved = true;
                if small || rel < 1e-15 {
                    crate::stats::lm_run(it);
                    return Some(LmResult { params: p, cost, iterations: it });
                }
                break;
            }
            lambda *= 10.0;
            if lambda > 1e16 {
                break;
            }
        }
        if !improved {
            break;
        }
    }
    crate::stats::lm_run(it);
    Some(LmResult { params: p, cost, iterations: it })
}

#[cfg(test)]
mod tests {
    use super::*;

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
