//! Symmetry plane of a mesh: the plane minimising the distance between the mesh and its mirror
//! image (design doc §6.2).
//!
//! For each principal plane through the centroid: mirror the mesh, ICP the mirror onto the
//! original (rigid), giving an improper isometry `M = T ∘ R`. For a true symmetry every point
//! `p` and its image `M p` are symmetric about the plane, so the plane is fitted to the midpoints
//! `(p + Mp) / 2` with normal along `Mp − p`. The candidate with the lowest ICP residual wins.
use crate::icp::{IcpOptions, icp};
use crate::{AlignError, principal_frame};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Bvh, Mesh, Point3, Vec3};

/// A symmetry plane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct SymmetryPlane {
    /// A point on the plane.
    pub point: [f64; 3],
    /// Unit normal.
    pub normal: [f64; 3],
    /// RMS distance between the mesh and its mirror image (mm).
    pub rms: f64,
}

fn reflect(p: &Point3, o: &Vec3, n: &Vec3) -> Point3 {
    p - n * (2.0 * (p.coords - o).dot(n))
}

/// Finds the best symmetry plane of `mesh`.
pub fn symmetry_plane(mesh: &Mesh) -> Result<SymmetryPlane, AlignError> {
    let frame = principal_frame(mesh)?;
    let inv = frame.inverse();
    let c = inv.apply(&Point3::origin()).coords;
    let bvh = Bvh::build(mesh);
    let opts = IcpOptions { tolerance: 1e-5, ..IcpOptions::default() };
    let mut best: Option<SymmetryPlane> = None;
    for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
        let n0 = inv.apply_vec(&axis);
        let mirrored = Mesh {
            positions: mesh.positions.iter().map(|p| reflect(p, &c, &n0)).collect(),
            triangles: mesh.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect(),
        };
        let Ok(rep) = icp(&mirrored, &bvh, &surfacecraft_geom::Transform::identity(), &opts) else { continue };
        // midpoints of p and M p
        let stride = mesh.vertex_count().div_ceil(5000).max(1);
        let mut mids = Vec::new();
        let mut dir = Vec3::zeros();
        for p in mesh.positions.iter().step_by(stride) {
            let q = rep.transform.apply(&reflect(p, &c, &n0));
            let d = q - p;
            mids.push(Point3::from((p.coords + q.coords) / 2.0));
            dir += if d.dot(&n0) >= 0.0 { d } else { -d };
        }
        let orient = if dir.norm() > 0.0 { dir } else { n0 };
        let Some(pl) = crate::fit_plane_for_symmetry(&mids, &orient) else { continue };
        let cand = SymmetryPlane { point: pl.0.into(), normal: pl.1.into(), rms: rep.rms };
        if best.as_ref().is_none_or(|b| cand.rms < b.rms) {
            best = Some(cand);
        }
    }
    best.ok_or_else(|| AlignError::Degenerate("no symmetry plane found".into()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, Transform, synth};

    #[test]
    fn finds_the_plane_of_a_symmetric_part() {
        // a box with a boss centred in x: symmetric about x = 30 only
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), 12);
        m.append(&synth::cylinder(Point3::new(30.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
        let t = Transform::rotation_axis_angle(&Vec3::new(0.3, 1.0, 0.2), 0.5)
            .then_after(&Transform::translation(Vec3::new(4.0, -3.0, 9.0)));
        let mut moved = m.transformed(&t);
        synth::add_noise(&mut moved, 0.02, &mut Rng::new(2));
        let s = symmetry_plane(&moved).unwrap();
        let n_true = t.apply_vec(&Vec3::x());
        let n = Vec3::from(s.normal);
        assert!(n.dot(&n_true).abs() > 0.9999, "normal {n:?} vs {n_true:?}");
        let on_plane = t.apply(&Point3::new(30.0, 0.0, 0.0));
        assert!(((on_plane.coords - Vec3::from(s.point)).dot(&n)).abs() < 0.05);
        assert!(s.rms < 0.05, "rms {}", s.rms);
    }
}
