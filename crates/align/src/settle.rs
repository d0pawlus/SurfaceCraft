//! Settling a part on its mirror plane: the pose that puts the symmetry plane on a world plane.
use crate::symmetry::{SymmetryPlane, symmetry_plane};
use crate::{AlignError, rms_to_surface};
use nalgebra::Rotation3;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Bvh, Mesh, Point3, Transform, Vec3, normalize};

/// Mirrored vertices measured for the residual (strided down to this many).
const RESIDUAL_SAMPLES: usize = 20_000;

/// The world plane the mirror plane is put on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum WorldPlane {
    /// The plane x = 0.
    #[default]
    Yz,
    /// The plane y = 0.
    Xz,
    /// The plane z = 0.
    Xy,
}

impl WorldPlane {
    /// Unit normal of the plane.
    #[must_use]
    pub fn normal(self) -> Vec3 {
        match self {
            WorldPlane::Yz => Vec3::x(),
            WorldPlane::Xz => Vec3::y(),
            WorldPlane::Xy => Vec3::z(),
        }
    }
}

/// The motion that settles a part on its mirror plane.
#[derive(Clone, Debug)]
pub struct Settled {
    /// The motion (model → aligned).
    pub transform: Transform,
    /// The mirror plane found, in the scan's frame.
    pub plane: SymmetryPlane,
    /// RMS distance (mm) from the mirror image of the mesh, reflected on `plane`, to the mesh.
    pub residual_rms: f64,
}

fn reflect(p: &Point3, plane: &SymmetryPlane) -> Point3 {
    let (o, n) = (Vec3::from(plane.point), Vec3::from(plane.normal));
    p - n * (2.0 * (p.coords - o).dot(&n))
}

/// Finds the mirror plane of `mesh` and the motion that puts it on `target`: the normal turns to
/// the target axis by the shortest arc (the other axes stay as close as they can to where they
/// are) and the part slides along that axis only.
///
/// # Errors
/// The errors of [`symmetry_plane`].
pub fn settle_symmetry(mesh: &Mesh, target: WorldPlane) -> Result<Settled, AlignError> {
    let plane = symmetry_plane(mesh)?;
    let axis = target.normal();
    let mut n = normalize(Vec3::from(plane.normal))
        .ok_or_else(|| AlignError::Degenerate("the symmetry plane has no normal".into()))?;
    if n.dot(&axis) < 0.0 {
        n = -n;
    }
    let rotation = Rotation3::rotation_between(&n, &axis)
        .ok_or_else(|| AlignError::Degenerate("the symmetry plane normal is opposite to the axis".into()))?;
    let along = (rotation * Vec3::from(plane.point)).dot(&axis);
    let transform = Transform { rotation, translation: -axis * along };
    let stride = mesh.vertex_count().div_ceil(RESIDUAL_SAMPLES).max(1);
    let mirrored: Vec<Point3> = mesh.positions.iter().step_by(stride).map(|p| reflect(p, &plane)).collect();
    let bvh = Bvh::build(mesh);
    let residual_rms = rms_to_surface(&bvh, &mirrored, &Transform::identity())
        .ok_or_else(|| AlignError::NotEnough("there is nothing to measure the symmetry on".into()))?;
    Ok(Settled { transform, plane, residual_rms })
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    fn part() -> Mesh {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), 12);
        m.append(&synth::cylinder(Point3::new(30.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
        m
    }

    #[test]
    fn the_mirror_plane_lands_on_each_world_plane() {
        let pose = Transform::rotation_axis_angle(&Vec3::new(0.2, 1.0, -0.3), 0.6)
            .then_after(&Transform::translation(Vec3::new(7.0, -3.0, 11.0)));
        let mut moved = part().transformed(&pose);
        synth::add_noise(&mut moved, 0.02, &mut Rng::new(5));
        for plane in [WorldPlane::Yz, WorldPlane::Xz, WorldPlane::Xy] {
            let s = settle_symmetry(&moved, plane).unwrap();
            let on_plane = s.transform.apply(&pose.apply(&Point3::new(30.0, 5.0, 2.0)));
            assert!(on_plane.coords.dot(&plane.normal()).abs() < 0.02, "{plane:?}: {on_plane:?}");
            assert!(s.residual_rms > 0.0 && s.residual_rms < 0.06, "{}", s.residual_rms);
        }
    }

    #[test]
    fn a_mesh_without_symmetry_data_is_an_error() {
        assert!(settle_symmetry(&Mesh::default(), WorldPlane::Yz).is_err());
    }
}
