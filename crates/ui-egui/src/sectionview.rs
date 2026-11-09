//! The viewport's section plane: a movable axis-aligned plane that clips the drawing (the GPU
//! discards fragments on its far side) and whose outline on the visible meshes comes from
//! `geom::section`. The maths here is plain and tested without a GPU.
use surfacecraft_geom::{Mesh, Point3, Vec3, section::Polyline};

/// The axis a section plane is perpendicular to.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SectionAxis {
    /// Plane x = offset.
    X,
    /// Plane y = offset.
    Y,
    /// Plane z = offset.
    #[default]
    Z,
}

impl SectionAxis {
    /// All axes.
    pub const ALL: [SectionAxis; 3] = [SectionAxis::X, SectionAxis::Y, SectionAxis::Z];

    /// The unit direction.
    #[must_use]
    pub fn dir(self) -> Vec3 {
        match self {
            SectionAxis::X => Vec3::x(),
            SectionAxis::Y => Vec3::y(),
            SectionAxis::Z => Vec3::z(),
        }
    }

    /// The catalogue key of the axis name.
    #[must_use]
    pub fn key(self) -> &'static str {
        match self {
            SectionAxis::X => "section.axis_x",
            SectionAxis::Y => "section.axis_y",
            SectionAxis::Z => "section.axis_z",
        }
    }

    /// The component of `p` along this axis.
    #[must_use]
    pub fn coord(self, p: &Point3) -> f64 {
        match self {
            SectionAxis::X => p.x,
            SectionAxis::Y => p.y,
            SectionAxis::Z => p.z,
        }
    }
}

/// A world clip plane `[nx, ny, nz, d]` for vertices stored relative to `shift` (`q = p - shift`):
/// `n · q > d - n · shift` removes the same points. The shader's form.
#[must_use]
pub fn shifted_plane(plane: &[f64; 4], shift: &Point3) -> [f32; 4] {
    let d = plane[3] - (plane[0] * shift.x + plane[1] * shift.y + plane[2] * shift.z);
    [plane[0] as f32, plane[1] as f32, plane[2] as f32, d as f32]
}

/// The section plane of the viewport.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SectionPlane {
    /// Whether the viewport is clipped.
    pub enabled: bool,
    /// What the plane is perpendicular to.
    pub axis: SectionAxis,
    /// Position along the axis (model units).
    pub offset: f64,
    /// Keep the other side.
    pub flip: bool,
}

impl SectionPlane {
    /// The plane normal; the side it points to is removed.
    #[must_use]
    pub fn normal(&self) -> Vec3 {
        let n = self.axis.dir();
        if self.flip { -n } else { n }
    }

    /// A point on the plane.
    #[must_use]
    pub fn origin(&self) -> Point3 {
        Point3::from(self.axis.dir() * self.offset)
    }

    /// Signed distance of `p` from the plane (positive on the removed side).
    #[must_use]
    pub fn signed_distance(&self, p: &Point3) -> f64 {
        (p - self.origin()).dot(&self.normal())
    }

    /// Whether `p` survives the clip.
    #[must_use]
    pub fn keeps(&self, p: &Point3) -> bool {
        !self.enabled || self.signed_distance(p) <= 0.0
    }

    /// The clip plane in world coordinates `[nx, ny, nz, d]`: a point is removed when
    /// `n · p > d`. All zero when the clip is off.
    #[must_use]
    pub fn world_plane(&self) -> [f64; 4] {
        if !self.enabled {
            return [0.0; 4];
        }
        let n = self.normal();
        [n.x, n.y, n.z, n.dot(&self.origin().coords)]
    }

    /// The plane as the shader takes it, for vertices stored relative to `shift`.
    #[must_use]
    pub fn shader_plane(&self, shift: &Point3) -> [f32; 4] {
        shifted_plane(&self.world_plane(), shift)
    }

    /// The outline of the plane on `meshes`.
    #[must_use]
    pub fn outline(&self, meshes: &[&Mesh]) -> Vec<Polyline> {
        meshes.iter().flat_map(|m| surfacecraft_geom::section::section(m, &self.origin(), &self.normal())).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    #[test]
    fn clip_keeps_the_side_opposite_the_normal() {
        let mut s = SectionPlane { enabled: true, axis: SectionAxis::Y, offset: 3.0, flip: false };
        assert!(s.keeps(&Point3::new(9.0, 2.9, 1.0)));
        assert!(!s.keeps(&Point3::new(9.0, 3.1, 1.0)));
        assert!((s.signed_distance(&Point3::new(0.0, 5.0, 0.0)) - 2.0).abs() < 1e-12);
        s.flip = true;
        assert!(!s.keeps(&Point3::new(9.0, 2.9, 1.0)));
        assert!(s.keeps(&Point3::new(9.0, 3.1, 1.0)));
        s.enabled = false;
        assert!(s.keeps(&Point3::new(0.0, 1e6, 0.0)));
    }

    /// The shader plane, evaluated at vertices shifted by the scene origin, agrees with `keeps`.
    #[test]
    fn shader_plane_agrees_with_the_world_clip_for_shifted_vertices() {
        let shift = Point3::new(100.0, -40.0, 7.0);
        for flip in [false, true] {
            let s = SectionPlane { enabled: true, axis: SectionAxis::X, offset: 104.0, flip };
            let [nx, ny, nz, d] = s.shader_plane(&shift);
            for p in [Point3::new(103.0, 0.0, 0.0), Point3::new(105.0, 3.0, 2.0), Point3::new(104.2, -1.0, 9.0)] {
                let q = p - shift.coords;
                let discarded = nx * q.x as f32 + ny * q.y as f32 + nz * q.z as f32 - d > 0.0;
                assert_eq!(discarded, !s.keeps(&p), "{p:?} flip {flip}");
            }
        }
        assert_eq!(SectionPlane::default().shader_plane(&shift), [0.0; 4]);
    }

    /// The outline of a plane through a sphere's middle is a closed circle of the sphere's radius.
    #[test]
    fn outline_of_a_sphere_is_its_great_circle() {
        let mesh = synth::sphere(Point3::origin(), 10.0, 48, 24);
        let s = SectionPlane { enabled: true, axis: SectionAxis::Z, offset: 0.0, flip: false };
        let lines = s.outline(&[&mesh]);
        assert_eq!(lines.len(), 1);
        assert!(lines[0].closed);
        for p in &lines[0].points {
            assert!(p.z.abs() < 1e-9);
            assert!((p.coords.norm() - 10.0).abs() < 0.15, "{p:?}");
        }
        // a plane off the body has no outline
        let off = SectionPlane { offset: 50.0, ..s };
        assert!(off.outline(&[&mesh]).is_empty());
    }
}
