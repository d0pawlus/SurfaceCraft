//! The 24 poses that put the principal frame of one scan on the principal frame of the other.
//!
//! A scan's principal axes are defined up to sign and, for near-equal extents, order. Trying every
//! proper signed permutation of the axes covers both without having to trust the eigenvalue order
//! or a skewness sign; the scoring stage tells the right one.
use surfacecraft_geom::{Mat3, Point3, Transform, Vec3};

/// The 24 rotations that map the axes to the axes: signed permutations with determinant +1.
pub(super) fn axis_rotations() -> Vec<Mat3> {
    const PERMS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
    let mut out = Vec::with_capacity(24);
    for perm in PERMS {
        for signs in 0..8u8 {
            let mut m = Mat3::zeros();
            for (row, &col) in perm.iter().enumerate() {
                let sign = if signs >> row & 1 == 1 { -1.0 } else { 1.0 };
                m[(row, col)] = sign;
            }
            if m.determinant() > 0.0 {
                out.push(m);
            }
        }
    }
    out
}

/// Centroid and right-handed principal axes (columns, largest spread first) of a point set.
pub(super) fn principal_frame(pts: &[Point3]) -> Option<(Vec3, Mat3)> {
    if pts.len() < 4 {
        return None;
    }
    let n = pts.len() as f64;
    let c: Vec3 = pts.iter().map(|p| p.coords).sum::<Vec3>() / n;
    let cov = pts.iter().fold(Mat3::zeros(), |acc, p| {
        let d = p.coords - c;
        acc + d * d.transpose()
    }) / n;
    if cov.iter().any(|v| !v.is_finite()) {
        return None;
    }
    let eig = cov.symmetric_eigen();
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| eig.eigenvalues[j].total_cmp(&eig.eigenvalues[i]));
    let x: Vec3 = eig.eigenvectors.column(idx[0]).into_owned();
    let y: Vec3 = eig.eigenvectors.column(idx[1]).into_owned();
    let z = x.cross(&y);
    Some((c, Mat3::from_columns(&[x, y, z])))
}

/// The candidate motions (moving → fixed), none when either scan has no usable frame.
pub(super) fn principal_candidates(moving: &[Point3], fixed: &[Point3]) -> Vec<Transform> {
    let (Some((cm, fm)), Some((cf, ff))) = (principal_frame(moving), principal_frame(fixed)) else {
        return Vec::new();
    };
    axis_rotations()
        .iter()
        .map(|d| {
            let r = ff * d * fm.transpose();
            Transform::from_parts(&r, cf - r * cm)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn there_are_24_distinct_proper_rotations() {
        let r = axis_rotations();
        assert_eq!(r.len(), 24);
        for (i, a) in r.iter().enumerate() {
            assert!((a.determinant() - 1.0).abs() < 1e-12);
            assert!(r.iter().skip(i + 1).all(|b| (a - b).norm() > 0.5));
        }
    }

    #[test]
    fn one_candidate_recovers_a_rotated_box_of_points() {
        let mut pts = Vec::new();
        for i in 0..8 {
            for j in 0..5 {
                for k in 0..3 {
                    pts.push(Point3::new(f64::from(i) * 7.0, f64::from(j) * 5.0, f64::from(k) * 3.0));
                }
            }
        }
        let g = Transform::rotation_axis_angle(&Vec3::new(1.0, 2.0, 0.5), 2.2)
            .then_after(&Transform::translation(Vec3::new(30.0, -5.0, 8.0)));
        let moved: Vec<Point3> = pts.iter().map(|p| g.apply(p)).collect();
        let best = principal_candidates(&moved, &pts)
            .iter()
            .map(|t| moved.iter().zip(&pts).map(|(m, p)| (t.apply(m) - p).norm()).fold(0.0, f64::max))
            .fold(f64::INFINITY, f64::min);
        assert!(best < 1e-9, "{best}");
        assert!(principal_candidates(&moved[..2], &pts).is_empty());
    }
}
