//! Kabsch / Umeyama without scale: the proper rotation and translation minimising
//! `Σ wᵢ |R·pᵢ + t − qᵢ|²`.
use crate::AlignError;
use surfacecraft_geom::{Mat3, Point3, Transform, Vec3};

/// The rigid motion mapping `from[i]` onto `to[i]` in the least-squares sense, with optional
/// weights. Needs at least 3 non-collinear pairs.
pub fn kabsch(from: &[Point3], to: &[Point3], weights: Option<&[f64]>) -> Result<Transform, AlignError> {
    let n = from.len().min(to.len());
    if n < 3 {
        return Err(AlignError::NotEnough(format!("{n} point pairs; at least 3 are needed")));
    }
    let w = |i: usize| weights.and_then(|w| w.get(i)).copied().unwrap_or(1.0).max(0.0);
    let mut sw = 0.0;
    let mut cp = Vec3::zeros();
    let mut cq = Vec3::zeros();
    for i in 0..n {
        let (Some(p), Some(q)) = (from.get(i), to.get(i)) else { continue };
        if !surfacecraft_geom::finite(p) || !surfacecraft_geom::finite(q) {
            return Err(AlignError::Degenerate(format!("pair {i} is not finite")));
        }
        sw += w(i);
        cp += p.coords * w(i);
        cq += q.coords * w(i);
    }
    if sw <= 0.0 {
        return Err(AlignError::Degenerate("all weights are zero".into()));
    }
    cp /= sw;
    cq /= sw;
    let mut h = Mat3::zeros();
    let mut spread = 0.0;
    for i in 0..n {
        let (Some(p), Some(q)) = (from.get(i), to.get(i)) else { continue };
        let a = p.coords - cp;
        let b = q.coords - cq;
        h += a * b.transpose() * w(i);
        spread += a.norm_squared() * w(i);
    }
    let svd = h.svd(true, true);
    let (Some(u), Some(vt)) = (svd.u, svd.v_t) else {
        return Err(AlignError::Degenerate("SVD did not converge".into()));
    };
    // collinear input leaves the rotation about the line undetermined
    let s = svd.singular_values;
    let scale = spread.max(1e-300);
    if s[1] / scale < 1e-12 {
        return Err(AlignError::Degenerate("the points are collinear".into()));
    }
    let v = vt.transpose();
    let d = (v * u.transpose()).determinant().signum();
    let fix = Mat3::from_diagonal(&Vec3::new(1.0, 1.0, if d < 0.0 { -1.0 } else { 1.0 }));
    let r = v * fix * u.transpose();
    let t = cq - r * cp;
    Ok(Transform::from_parts(&r, t))
}

/// Root-mean-square residual of `t` over the pairs.
#[must_use]
pub fn rms(t: &Transform, from: &[Point3], to: &[Point3]) -> f64 {
    let n = from.len().min(to.len());
    if n == 0 {
        return 0.0;
    }
    let s: f64 = from.iter().zip(to).map(|(p, q)| (t.apply(p) - q).norm_squared()).sum();
    (s / n as f64).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::Rng;

    #[test]
    fn recovers_a_known_motion_exactly() {
        let mut rng = Rng::new(5);
        let truth = Transform::rotation_axis_angle(&Vec3::new(0.3, -1.0, 0.7), 1.1)
            .then_after(&Transform::translation(Vec3::new(12.0, -40.0, 7.5)));
        let from: Vec<Point3> = (0..50)
            .map(|_| Point3::new(rng.range(-50.0, 50.0), rng.range(-50.0, 50.0), rng.range(-50.0, 50.0)))
            .collect();
        let to: Vec<Point3> = from.iter().map(|p| truth.apply(p)).collect();
        let t = kabsch(&from, &to, None).unwrap();
        let err = t.max_displacement(&truth, &from);
        assert!(err < 1e-9, "max displacement {err}");
    }

    #[test]
    fn reflection_is_never_returned() {
        let from = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let to: Vec<Point3> = from.iter().map(|p| Point3::new(-p.x, p.y, p.z)).collect();
        let t = kabsch(&from, &to, None).unwrap();
        assert!((t.rotation.matrix().determinant() - 1.0).abs() < 1e-12);
    }

    #[test]
    fn degenerate_inputs_are_errors() {
        let line: Vec<Point3> = (0..5).map(|i| Point3::new(f64::from(i), 0.0, 0.0)).collect();
        assert!(kabsch(&line, &line, None).is_err());
        assert!(kabsch(&line[..2], &line[..2], None).is_err());
        let bad = vec![Point3::new(f64::NAN, 0.0, 0.0); 3];
        assert!(kabsch(&bad, &bad, None).is_err());
    }
}
