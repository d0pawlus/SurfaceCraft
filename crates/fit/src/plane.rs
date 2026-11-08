//! Least-squares plane (PCA): the plane through the centroid normal to the direction of least
//! spread.
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Mat3, Point3, Vec3};

/// A fitted plane.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PlaneFit {
    /// Centroid of the points (on the plane).
    pub point: Point3,
    /// Unit normal.
    pub normal: Vec3,
    /// RMS distance of the points to the plane.
    pub rms: f64,
    /// Largest distance.
    pub max: f64,
}

/// Fits a plane to `points` (at least 3, not collinear). `orient`, when given, flips the
/// normal to point the same way.
#[must_use]
pub fn fit_plane(points: &[Point3], orient: Option<&Vec3>) -> Option<PlaneFit> {
    let pts: Vec<&Point3> = points.iter().filter(|p| surfacecraft_geom::finite(p)).collect();
    if pts.len() < 3 {
        return None;
    }
    let n = pts.len() as f64;
    let c: Vec3 = pts.iter().map(|p| p.coords).sum::<Vec3>() / n;
    let mut cov = Mat3::zeros();
    for p in &pts {
        let d = p.coords - c;
        cov += d * d.transpose();
    }
    let eig = cov.symmetric_eigen();
    let mut order = [0usize, 1, 2];
    order.sort_by(|&i, &j| eig.eigenvalues[i].total_cmp(&eig.eigenvalues[j]));
    // collinear points: the two smallest eigenvalues vanish
    let largest = eig.eigenvalues[order[2]].max(1e-300);
    if eig.eigenvalues[order[1]] / largest < 1e-14 {
        return None;
    }
    let mut normal: Vec3 = eig.eigenvectors.column(order[0]).into_owned();
    normal = surfacecraft_geom::normalize(normal)?;
    if let Some(o) = orient
        && normal.dot(o) < 0.0
    {
        normal = -normal;
    }
    let mut sq = 0.0;
    let mut max: f64 = 0.0;
    for p in &pts {
        let d = (p.coords - c).dot(&normal);
        sq += d * d;
        max = max.max(d.abs());
    }
    Some(PlaneFit { point: Point3::from(c), normal, rms: (sq / n).sqrt(), max })
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::Rng;

    #[test]
    fn fits_a_tilted_plane() {
        let n = Vec3::new(1.0, -2.0, 3.0).normalize();
        let u = surfacecraft_geom::any_perpendicular(&n);
        let v = n.cross(&u);
        let mut rng = Rng::new(2);
        let pts: Vec<Point3> = (0..500)
            .map(|_| Point3::new(1.0, 2.0, 3.0) + u * rng.range(-10.0, 10.0) + v * rng.range(-10.0, 10.0))
            .collect();
        let f = fit_plane(&pts, Some(&n)).unwrap();
        assert!((f.normal - n).norm() < 1e-12);
        assert!(f.rms < 1e-12);
        let line: Vec<Point3> = (0..5).map(|i| Point3::new(f64::from(i), 0.0, 0.0)).collect();
        assert!(fit_plane(&line, None).is_none());
    }
}
