//! Moving-least-squares projection onto a scan (point cloud with normals): a point near the
//! scan is moved onto a local quadric fitted to the nearby scan vertices, which also gives a
//! smooth normal. Averaging over many vertices makes the result far less noisy than snapping to
//! the closest vertex or triangle (design doc §6.3, P4: quad layout vertices and patch edges).
use nalgebra::{Matrix3, Matrix6, SymmetricEigen, Vector6};
use surfacecraft_geom::{KdTree, Point3, Vec3, normalize};

/// A scan prepared for projection.
pub struct ScanSurface {
    pts: Vec<Point3>,
    tree: KdTree,
}

impl ScanSurface {
    /// The scan vertices (an empty or non-finite set projects nothing).
    #[must_use]
    pub fn new(points: &[Point3]) -> Self {
        let pts: Vec<Point3> = points.iter().copied().filter(surfacecraft_geom::finite).collect();
        Self { tree: KdTree::build(&pts), pts }
    }

    /// Number of scan points.
    #[must_use]
    pub fn len(&self) -> usize {
        self.pts.len()
    }

    /// No points?
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.pts.is_empty()
    }

    /// Nearest scan vertex (used as a first guess and as a distance estimate).
    #[must_use]
    pub fn nearest(&self, q: &Point3) -> Option<(Point3, f64)> {
        let (i, d2) = self.tree.nearest(q)?;
        Some((*self.pts.get(i)?, d2.sqrt()))
    }

    /// Projects `q` onto the scan with a Gaussian-weighted local quadric of support `radius`;
    /// `hint` orients the returned unit normal (it points to the same side). The support grows
    /// (up to 3 times) when fewer than 12 vertices are inside it.
    #[must_use]
    pub fn project(&self, q: &Point3, radius: f64, hint: &Vec3) -> Option<(Point3, Vec3)> {
        if !(radius.is_finite() && radius > 0.0) || self.pts.is_empty() {
            return None;
        }
        let mut cur = *q;
        let mut out: Option<(Point3, Vec3)> = None;
        for it in 0..3 {
            let mut r = radius;
            let mut idx = self.tree.within(&cur, r);
            let mut grow = 0;
            while idx.len() < 12 && grow < 3 {
                r *= 1.6;
                idx = self.tree.within(&cur, r);
                grow += 1;
            }
            if idx.len() < 6 {
                return out;
            }
            let (c, n, e1, e2) = local_frame(&self.pts, &idx, &cur, r, hint)?;
            // quadric z(x, y) in the frame centred on c
            let mut a = Matrix6::<f64>::zeros();
            let mut b = Vector6::<f64>::zeros();
            let s2 = (r * 0.5) * (r * 0.5);
            for &i in &idx {
                let Some(p) = self.pts.get(i) else { continue };
                let d = p - c;
                let (x, y, z) = (d.dot(&e1), d.dot(&e2), d.dot(&n));
                let w = (-d.norm_squared() / s2).exp();
                let row = Vector6::new(1.0, x, y, x * x, x * y, y * y);
                a += row * row.transpose() * w;
                b += row * (z * w);
            }
            for k in 0..6 {
                a[(k, k)] += 1e-9 * (1.0 + a[(k, k)]);
            }
            let Some(coef) = a.lu().solve(&b) else { return out };
            if !coef.iter().all(|v| v.is_finite()) {
                return out;
            }
            // the surface point above the foot of `cur` (not above the centroid): evaluate at the
            // in-plane position of `cur`
            let d = cur - c;
            let (x, y) = (d.dot(&e1), d.dot(&e2));
            let z = coef[0] + coef[1] * x + coef[2] * y + coef[3] * x * x + coef[4] * x * y + coef[5] * y * y;
            let (zx, zy) = (coef[1] + 2.0 * coef[3] * x + coef[4] * y, coef[2] + coef[4] * x + 2.0 * coef[5] * y);
            let p = c + e1 * x + e2 * y + n * z;
            let nn = normalize(n - e1 * zx - e2 * zy)?;
            let nn = if nn.dot(hint) < 0.0 { -nn } else { nn };
            out = Some((p, nn));
            if (p - cur).norm() < 1e-9 * (1.0 + radius) && it > 0 {
                break;
            }
            cur = p;
        }
        out
    }
}

/// Weighted centroid, PCA normal (oriented by `hint`) and tangent axes of the points `idx`.
fn local_frame(pts: &[Point3], idx: &[usize], at: &Point3, r: f64, hint: &Vec3) -> Option<(Point3, Vec3, Vec3, Vec3)> {
    let s2 = (r * 0.5) * (r * 0.5);
    let mut wsum = 0.0;
    let mut c = Vec3::zeros();
    for &i in idx {
        let p = pts.get(i)?;
        let w = (-(p - at).norm_squared() / s2).exp();
        wsum += w;
        c += p.coords * w;
    }
    if wsum <= 0.0 {
        return None;
    }
    let c = Point3::from(c / wsum);
    let mut cov = Matrix3::<f64>::zeros();
    for &i in idx {
        let p = pts.get(i)?;
        let d = p - c;
        let w = (-(p - at).norm_squared() / s2).exp();
        cov += d * d.transpose() * w;
    }
    let eig = SymmetricEigen::new(cov);
    let mut k = 0;
    for j in 1..3 {
        if eig.eigenvalues[j] < eig.eigenvalues[k] {
            k = j;
        }
    }
    let mut n = normalize(eig.eigenvectors.column(k).into_owned())?;
    if n.dot(hint) < 0.0 {
        n = -n;
    }
    let e1 = surfacecraft_geom::any_perpendicular(&n);
    let e2 = n.cross(&e1);
    Some((c, n, e1, e2))
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    #[test]
    fn projection_onto_a_noisy_sphere_averages_the_noise() {
        let mut m = synth::sphere(Point3::origin(), 20.0, 120, 60);
        synth::add_noise(&mut m, 0.03, &mut Rng::new(3));
        let s = ScanSurface::new(&m.positions);
        let mut worst: f64 = 0.0;
        let mut rng = Rng::new(9);
        for _ in 0..200 {
            let d = rng.unit_vector();
            let q = Point3::from(d * (20.0 + rng.range(-0.3, 0.3)));
            let (p, n) = s.project(&q, 5.0, &d).unwrap();
            worst = worst.max((p.coords.norm() - 20.0).abs());
            assert!(n.dot(&d) > 0.999, "normal off by {}", n.dot(&d).acos());
        }
        // σ = 0.03 per vertex; the average of ~100 vertices is far closer to the truth
        assert!(worst < 0.03, "worst {worst}");
    }
}
