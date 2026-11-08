//! Principal axes of a mesh surface ("align to world axes"): the centroid goes to the origin,
//! the direction of largest spread to X, the second to Y, the smallest to Z. Signs follow the
//! third moment along each axis, so the result does not depend on vertex order.
use crate::AlignError;
use surfacecraft_geom::{Mat3, Mesh, Transform, Vec3};

/// The motion taking the mesh's principal frame to the world frame.
pub fn principal_frame(mesh: &Mesh) -> Result<Transform, AlignError> {
    let area = mesh.area();
    if !(area > 0.0) {
        return Err(AlignError::NotEnough("the mesh has no area".into()));
    }
    let c = mesh.centroid().coords;
    // area-weighted second moment of each triangle (exact for a uniform triangle)
    let mut cov = Mat3::zeros();
    for i in 0..mesh.triangle_count() {
        let Some([a, b, cc]) = mesh.triangle(i) else { continue };
        let w = mesh.face_area(i);
        let (a, b, cc) = (a.coords - c, b.coords - c, cc.coords - c);
        let s = a + b + cc;
        cov += (s * s.transpose() + a * a.transpose() + b * b.transpose() + cc * cc.transpose()) * (w / 12.0);
    }
    let eig = cov.symmetric_eigen();
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&i, &j| eig.eigenvalues[j].total_cmp(&eig.eigenvalues[i]));
    let mut axes: Vec<Vec3> = idx.iter().map(|&i| eig.eigenvectors.column(i).into_owned()).collect();
    // sign by skewness along each axis
    for ax in axes.iter_mut().take(2) {
        let mut m3 = 0.0;
        for p in &mesh.positions {
            let d = (p.coords - c).dot(ax);
            m3 += d * d * d;
        }
        if m3 < 0.0 {
            *ax = -*ax;
        }
    }
    let (Some(x), Some(y)) = (axes.first().copied(), axes.get(1).copied()) else {
        return Err(AlignError::Degenerate("no principal axes".into()));
    };
    let z = x.cross(&y);
    let r = Mat3::from_rows(&[x.transpose(), y.transpose(), z.transpose()]);
    Ok(Transform::from_parts(&r, -(r * c)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Point3, synth};

    #[test]
    fn box_axes_become_world_axes() {
        let m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 30.0, 10.0), 6);
        let truth = Transform::rotation_axis_angle(&Vec3::new(1.0, 1.0, 0.3), 0.8)
            .then_after(&Transform::translation(Vec3::new(5.0, 6.0, 7.0)));
        let moved = m.transformed(&truth);
        let t = principal_frame(&moved).unwrap();
        let aligned = moved.transformed(&t);
        let b = aligned.bbox();
        let s = b.size();
        assert!((s.x - 60.0).abs() < 1e-6 && (s.y - 30.0).abs() < 1e-6 && (s.z - 10.0).abs() < 1e-6, "{s:?}");
        assert!(b.center().coords.norm() < 1e-6);
        assert!(principal_frame(&Mesh::default()).is_err());
    }
}
