//! The signed volume enclosed by Bézier patches, from the patches themselves (P9-O3).
//!
//! The divergence theorem gives `V = 1/3 ∮ (p - c) · (S_u × S_v) du dv` for a closed, outward
//! oriented surface and any reference point `c`. For a degree `n` patch the integrand is a
//! polynomial of degree `3n - 1` in each parameter, so Gauss-Legendre quadrature with
//! `ceil(3n/2)` points is exact up to rounding; one more point is used. This is a check of
//! orientation inside the kernel adapter (the sign of the volume), not a construction or an
//! evaluation the application relies on elsewhere; the volume of a body is still measured by
//! solidcore. Coordinates are taken relative to a reference point inside the network and the
//! terms are summed with Neumaier compensation, so a thin body far from the origin keeps its
//! sign; the sum of the magnitudes of the terms is returned to bound the rounding.

/// A running sum with Neumaier compensation and the sum of the magnitudes of its terms.
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct Sum {
    total: f64,
    carry: f64,
    /// The sum of `|term|` over all terms: the scale of the rounding errors.
    pub magnitude: f64,
}

impl Sum {
    pub(crate) fn add(&mut self, term: f64) {
        let t = self.total + term;
        self.carry += if self.total.abs() >= term.abs() { (self.total - t) + term } else { (term - t) + self.total };
        self.total = t;
        self.magnitude += term.abs();
    }

    pub(crate) fn value(&self) -> f64 {
        self.total + self.carry
    }
}

/// Gauss-Legendre nodes and weights on `[0, 1]` (Newton iteration on the Legendre polynomial).
fn gauss_legendre(m: usize) -> Vec<(f64, f64)> {
    let mf = m as f64;
    (0..m)
        .map(|i| {
            let mut x = (std::f64::consts::PI * (i as f64 + 0.75) / (mf + 0.5)).cos();
            let mut dp = 1.0;
            for _ in 0..100 {
                let (mut p0, mut p1) = (1.0, x);
                for k in 2..=m {
                    let kf = k as f64;
                    (p0, p1) = (p1, ((2.0 * kf - 1.0) * x * p1 - (kf - 1.0) * p0) / kf);
                }
                // p1 = P_m(x), p0 = P_(m-1)(x)
                dp = mf * (x * p1 - p0) / (x * x - 1.0);
                let step = p1 / dp;
                x -= step;
                if step.abs() < 1e-15 {
                    break;
                }
            }
            ((x + 1.0) / 2.0, 1.0 / ((1.0 - x * x) * dp * dp))
        })
        .collect()
}

/// Bernstein basis of degree `n` at `t`, and its derivative.
fn bernstein(n: usize, t: f64) -> (Vec<f64>, Vec<f64>) {
    let basis = |deg: usize| -> Vec<f64> {
        let mut b = vec![1.0];
        for _ in 0..deg {
            let mut next = vec![0.0; b.len() + 1];
            for (i, v) in b.iter().enumerate() {
                next[i] += v * (1.0 - t);
                next[i + 1] += v * t;
            }
            b = next;
        }
        b
    };
    let low = basis(n.saturating_sub(1));
    let at = |i: isize| -> f64 { usize::try_from(i).ok().and_then(|i| low.get(i)).copied().unwrap_or(0.0) };
    let d = (0..=n as isize).map(|i| n as f64 * (at(i - 1) - at(i))).collect();
    (basis(n), d)
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// The quadrature rule for degree `n` patches.
pub(crate) struct Rule {
    nodes: Vec<(f64, f64)>,
    bases: Vec<(Vec<f64>, Vec<f64>)>,
}

impl Rule {
    pub(crate) fn for_degree(n: usize) -> Self {
        let nodes = gauss_legendre((3 * n).div_ceil(2) + 1);
        let bases = nodes.iter().map(|&(t, _)| bernstein(n, t)).collect();
        Self { nodes, bases }
    }

    /// Adds `sign` times `1/3 ∫ (p - centre) · (S_u × S_v)` over the patch to `sum`.
    pub(crate) fn add_patch(&self, sum: &mut Sum, net: &[Vec<[f64; 3]>], centre: [f64; 3], sign: f64) {
        for (&(_, wu), (bu, dbu)) in self.nodes.iter().zip(&self.bases) {
            for (&(_, wv), (bv, dbv)) in self.nodes.iter().zip(&self.bases) {
                let (mut p, mut su, mut sv) = ([0.0; 3], [0.0; 3], [0.0; 3]);
                for ((&bi, &dbi), row) in bu.iter().zip(dbu).zip(net) {
                    for ((&bj, &dbj), q) in bv.iter().zip(dbv).zip(row) {
                        for k in 0..3 {
                            let c = q[k] - centre[k];
                            p[k] += bi * bj * c;
                            su[k] += dbi * bj * c;
                            sv[k] += bi * dbj * c;
                        }
                    }
                }
                let n = cross(su, sv);
                sum.add(sign * wu * wv * (p[0] * n[0] + p[1] * n[1] + p[2] * n[2]) / 3.0);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn gauss_rule_integrates_polynomials_exactly() {
        for m in 1..=14 {
            let rule = gauss_legendre(m);
            let total: f64 = rule.iter().map(|w| w.1).sum();
            assert!((total - 1.0).abs() < 1e-13, "m {m}: weights {total}");
            // exact up to degree 2m - 1
            let degree = (2 * m - 1) as i32;
            let integral: f64 = rule.iter().map(|&(t, w)| w * t.powi(degree)).sum();
            assert!((integral - 1.0 / (degree as f64 + 1.0)).abs() < 1e-13, "m {m}");
        }
    }

    #[test]
    fn unit_cube_of_flat_patches_has_volume_one() {
        let flat = |a: [f64; 3], u: [f64; 3], v: [f64; 3]| -> Vec<Vec<[f64; 3]>> {
            (0..2)
                .map(|i| (0..2).map(|j| std::array::from_fn(|k| a[k] + u[k] * i as f64 + v[k] * j as f64)).collect())
                .collect()
        };
        let (x, y, z) = ([1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]);
        let o = [0.0; 3];
        let faces = [flat(o, y, x), flat(z, x, y), flat(o, x, z), flat(y, z, x), flat(o, z, y), flat(x, y, z)];
        let rule = Rule::for_degree(1);
        let mut sum = Sum::default();
        faces.iter().for_each(|f| rule.add_patch(&mut sum, f, [0.5; 3], 1.0));
        assert!((sum.value() - 1.0).abs() < 1e-13, "{}", sum.value());
    }
}
