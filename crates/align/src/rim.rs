//! Open flat rims: a planar boundary loop of the mesh stands for the face a scan did not capture
//! (the bottom of a part scanned standing on the table), and is voted for with the area the loop
//! encloses.
use crate::faces::{FaceSet, PlaneFace};
use nalgebra::SymmetricEigen;
use surfacecraft_geom::{Mat3, Mesh, Point3, Vec3, edit::boundary_loops, normalize};

/// A loop encloses at least this share of the surface area to count as a missing face.
const MIN_SHARE: f64 = 0.02;
/// A loop is flat when its points deviate from the plane by at most this share of the
/// equivalent diameter of the enclosed area (RMS).
const PLANAR_TOLERANCE: f64 = 0.02;
/// Slab half width of a rim as a share of the bounding-box diagonal.
const HALF_WIDTH: f64 = 0.003;

/// The flat rims of `mesh`, largest first.
pub(crate) fn rim_faces(mesh: &Mesh, set: &FaceSet) -> Vec<PlaneFace> {
    let mut rims: Vec<PlaneFace> = boundary_loops(mesh)
        .iter()
        .filter(|lp| lp.len() >= 3)
        .filter_map(|lp| {
            let pts: Vec<Point3> = lp.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()).collect();
            (pts.len() == lp.len()).then(|| rim_of(&pts, set)).flatten()
        })
        .collect();
    rims.sort_by(|a, b| b.area.total_cmp(&a.area));
    rims
}

/// The rim a closed polygon of points makes, when it is large and flat enough.
fn rim_of(pts: &[Point3], set: &FaceSet) -> Option<PlaneFace> {
    let n = pts.len() as f64;
    let mean: Vec3 = pts.iter().map(|p| p.coords).sum::<Vec3>() / n;
    let area_vec: Vec3 =
        (0..pts.len()).map(|i| (pts[i].coords - mean).cross(&(pts[(i + 1) % pts.len()].coords - mean))).sum::<Vec3>()
            * 0.5;
    if !(area_vec.norm() >= MIN_SHARE * set.total) {
        return None;
    }
    let mut cov = Mat3::zeros();
    for p in pts {
        let d = p.coords - mean;
        cov += d * d.transpose();
    }
    let eig = SymmetricEigen::new(cov);
    let lowest = (0..3).min_by(|&i, &j| eig.eigenvalues[i].total_cmp(&eig.eigenvalues[j]))?;
    let mut normal = normalize(eig.eigenvectors.column(lowest).into_owned())?;
    if normal.dot(&area_vec) < 0.0 {
        normal = -normal;
    }
    let area = area_vec.dot(&normal);
    let rms = (eig.eigenvalues[lowest].max(0.0) / n).sqrt();
    let equivalent_diameter = 2.0 * (area / std::f64::consts::PI).sqrt();
    if !(area >= MIN_SHARE * set.total) || rms > PLANAR_TOLERANCE * equivalent_diameter {
        return None;
    }
    Some(PlaneFace {
        normal,
        point: Point3::from(polygon_centroid(pts, &mean, &normal)),
        area,
        half_width: HALF_WIDTH * set.diag,
        rim: true,
    })
}

/// Area centroid of the polygon: a fan from its first point, triangles signed along `normal`.
fn polygon_centroid(pts: &[Point3], mean: &Vec3, normal: &Vec3) -> Vec3 {
    let Some(p0) = pts.first() else { return *mean };
    let (mut acc, mut total) = (Vec3::zeros(), 0.0);
    for w in pts.windows(2).skip(1) {
        let signed = 0.5 * (w[0] - p0).cross(&(w[1] - p0)).dot(normal);
        acc += (p0.coords + w[0].coords + w[1].coords) / 3.0 * signed;
        total += signed;
    }
    if total.abs() > 0.0 && acc.iter().all(|v| v.is_finite()) { acc / total } else { *mean }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    fn open_box(sx: f64, sy: f64, sz: f64) -> Mesh {
        let mut m = synth::cuboid(Point3::new(5.0, 6.0, 7.0), Vec3::new(sx, sy, sz), 10);
        let keep: Vec<u32> = (0..m.triangle_count())
            .filter(|&i| {
                !(m.face_centroid(i).is_some_and(|c| (c.z - 7.0).abs() < 1e-9)
                    && m.face_normal(i).is_some_and(|n| n.z < -0.9))
            })
            .map(|i| i as u32)
            .collect();
        m = m.submesh(&keep);
        m
    }

    #[test]
    fn the_missing_bottom_is_a_rim_with_its_area_and_centre() {
        let m = open_box(60.0, 40.0, 20.0);
        let set = FaceSet::new(&m).unwrap();
        let rims = rim_faces(&m, &set);
        assert_eq!(rims.len(), 1);
        let r = &rims[0];
        assert!(r.rim);
        assert!((r.area - 2400.0).abs() < 1e-6, "{}", r.area);
        assert!(r.normal.z.abs() > 0.999_999, "{:?}", r.normal);
        assert!((r.point - Point3::new(35.0, 26.0, 7.0)).norm() < 1e-6, "{:?}", r.point);
    }

    #[test]
    fn a_closed_mesh_and_a_tiny_hole_have_no_rim() {
        let closed = synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 10.0), 4);
        assert!(rim_faces(&closed, &FaceSet::new(&closed).unwrap()).is_empty());
        let mut holed = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), 12);
        let keep: Vec<u32> = (1..holed.triangle_count() as u32).collect();
        holed = holed.submesh(&keep);
        assert!(rim_faces(&holed, &FaceSet::new(&holed).unwrap()).is_empty());
    }
}
