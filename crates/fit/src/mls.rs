//! Moving-least-squares projection onto a scan (point cloud with normals): a point near the
//! scan is moved onto a local quadric fitted to the nearby scan vertices, which also gives a
//! smooth normal. Averaging over many vertices makes the result far less noisy than snapping to
//! the closest vertex or triangle (design doc §6.3, P4: quad layout vertices and patch edges).
use nalgebra::{Matrix3, Matrix6, SymmetricEigen, Vector6};
use surfacecraft_geom::{KdTree, Point3, Vec3, normalize};

/// Scan vertices whose normal makes less than about 72 degrees with the hint lie on the same
/// side of the part as the point being projected.
const SAME_SIDE_COS: f64 = 0.3;
/// A projection that moves the point farther than this many supports is a runaway of the quadric.
const MAX_MOVE_SUPPORTS: f64 = 2.0;

/// A scan prepared for projection.
pub struct ScanSurface {
    pts: Vec<Point3>,
    /// Normal of every point; empty when the scan was given without normals.
    normals: Vec<Vec3>,
    tree: KdTree,
}

impl ScanSurface {
    /// The scan vertices (an empty or non-finite set projects nothing).
    #[must_use]
    pub fn new(points: &[Point3]) -> Self {
        let pts: Vec<Point3> = points.iter().copied().filter(surfacecraft_geom::finite).collect();
        Self { tree: KdTree::build(&pts), pts, normals: Vec::new() }
    }

    /// The scan vertices with their normals. A projection then uses only the vertices that face
    /// the same way as its hint, so the two sides of a thin wall (both inside one support) are
    /// not averaged into one surface. Lengths that differ give a scan without normals.
    #[must_use]
    pub fn oriented(points: &[Point3], normals: &[Vec3]) -> Self {
        if points.len() != normals.len() {
            return Self::new(points);
        }
        let (pts, normals): (Vec<Point3>, Vec<Vec3>) = points
            .iter()
            .zip(normals)
            .filter(|(p, n)| surfacecraft_geom::finite(p) && n.iter().all(|v| v.is_finite()))
            .map(|(p, n)| (*p, *n))
            .unzip();
        Self { tree: KdTree::build(&pts), pts, normals }
    }

    /// The vertices within `r` of `at` on the side `hint` points to (all of them without normals).
    fn support(&self, at: &Point3, r: f64, hint: &Vec3) -> Vec<usize> {
        let mut idx = self.tree.within(at, r);
        if !self.normals.is_empty() {
            idx.retain(|&i| self.normals.get(i).is_some_and(|n| n.dot(hint) > SAME_SIDE_COS));
        }
        idx
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
            let mut idx = self.support(&cur, r, hint);
            let mut grow = 0;
            while idx.len() < 12 && grow < 3 {
                r *= 1.6;
                idx = self.support(&cur, r, hint);
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
            if !surfacecraft_geom::finite(&p) || (p - q).norm() > MAX_MOVE_SUPPORTS * r {
                return out;
            }
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
    fn a_thin_wall_projects_onto_the_side_the_hint_faces() {
        // two parallel sheets 1 mm apart, both inside the 5 mm support
        let mut pts = Vec::new();
        let mut normals = Vec::new();
        for i in 0..40 {
            for j in 0..40 {
                let (x, y) = (f64::from(i) * 0.5, f64::from(j) * 0.5);
                pts.push(Point3::new(x, y, 0.5));
                normals.push(Vec3::z());
                pts.push(Point3::new(x, y, -0.5));
                normals.push(-Vec3::z());
            }
        }
        let q = Point3::new(10.0, 10.0, 0.2);
        let blind = ScanSurface::new(&pts).project(&q, 5.0, &Vec3::z()).unwrap().0;
        assert!(blind.z.abs() < 0.2, "without normals the two sheets average out: z = {}", blind.z);
        let s = ScanSurface::oriented(&pts, &normals);
        let top = s.project(&q, 5.0, &Vec3::z()).unwrap().0;
        let bottom = s.project(&q, 5.0, &-Vec3::z()).unwrap().0;
        assert!((top.z - 0.5).abs() < 1e-6 && (bottom.z + 0.5).abs() < 1e-6, "top {} bottom {}", top.z, bottom.z);
    }

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
