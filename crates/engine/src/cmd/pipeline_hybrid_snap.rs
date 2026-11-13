//! The prepared copy of the scan that the hybrid method wraps (P9-O6): the vertices of every region
//! that becomes an analytic face are put on a surface parallel to its primitive, a little beyond it
//! on the side the primitive's cut removes (the void side), or a little short of it for an addition.
//!
//! The freeform stock is made from that copy. Where it lies beyond the primitive the kernel trims it
//! back to the primitive exactly and the face of the result is the primitive; where an addition
//! (boss, dome) is united with it, the stock lies inside the addition and the face is the addition's.
//! Without the offset the stock would lie on the primitive within noise and fit error, and the cut
//! would leave a patchwork of primitive and patches.
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Mesh, Point3, Vec3};
use surfacecraft_kernel::hybrid::Shape;

/// A region to put on the offset surface of its primitive.
pub(super) struct SnapRegion {
    /// Triangles of the region.
    pub triangles: Vec<u32>,
    /// The primitive as the kernel will cut with it.
    pub primitive: Primitive,
    /// The material lies inside the shape (else the void does: a hole).
    pub convex: bool,
    /// The kernel adds the shape to the stock (a boss, a dome) instead of cutting the stock with it.
    pub is_addition: bool,
}

/// The primitive of a kernel shape (the same parameters).
pub(super) fn primitive_of(shape: &Shape) -> Primitive {
    match shape {
        Shape::Plane { point, normal } => Primitive::Plane { point: *point, normal: *normal },
        Shape::Cylinder { point, axis, radius } => Primitive::Cylinder { point: *point, axis: *axis, radius: *radius },
        Shape::Cone { apex, axis, half_angle } => Primitive::Cone { apex: *apex, axis: *axis, half_angle: *half_angle },
        Shape::Sphere { center, radius } => Primitive::Sphere { center: *center, radius: *radius },
        Shape::Torus { center, axis, major_radius, minor_radius } => {
            Primitive::Torus { center: *center, axis: *axis, major_radius: *major_radius, minor_radius: *minor_radius }
        }
    }
}

impl SnapRegion {
    /// Where `p` goes: onto the primitive, then `delta` toward the void for a cut, away from it for
    /// an addition. `None` when the primitive has no direction at `p`.
    fn target(&self, p: &Point3, delta: f64) -> Option<Point3> {
        let on = self.primitive.project(p);
        let outward: Vec3 = self.primitive.normal(p);
        let toward_void = if self.convex { outward } else { -outward };
        let offset = if self.is_addition { -delta } else { delta };
        let q = on + toward_void * offset;
        (q.coords.iter().all(|c| c.is_finite())).then_some(q)
    }
}

/// A copy of `mesh` with the vertices of each region moved onto its offset surface. A vertex that
/// belongs to several regions goes with the first of them (regions are given largest first).
pub(super) fn snapped(mesh: &Mesh, regions: &[SnapRegion], delta: f64) -> Mesh {
    let mut positions = mesh.positions.clone();
    let mut claimed = vec![false; positions.len()];
    for region in regions {
        for &t in &region.triangles {
            let Some(tri) = mesh.triangles.get(t as usize) else { continue };
            for &v in tri {
                let i = v as usize;
                let (Some(done), Some(p)) = (claimed.get_mut(i), mesh.positions.get(i)) else { continue };
                if *done {
                    continue;
                }
                *done = true;
                if let (Some(q), Some(slot)) = (region.target(p, delta), positions.get_mut(i)) {
                    *slot = q;
                }
            }
        }
    }
    Mesh { positions, triangles: mesh.triangles.clone() }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two triangles of a unit square in z = 0 and one triangle off it, sharing vertex 1.
    fn mesh() -> Mesh {
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        Mesh {
            positions: vec![
                p(0.0, 0.0, 0.01),
                p(1.0, 0.0, -0.01),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
                p(2.0, 0.0, 5.0),
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3], [1, 4, 2]],
        }
    }

    fn plane_up() -> Primitive {
        Primitive::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] }
    }

    fn region(triangles: &[u32], is_addition: bool) -> SnapRegion {
        SnapRegion { triangles: triangles.to_vec(), primitive: plane_up(), convex: true, is_addition }
    }

    #[test]
    fn a_cut_region_lies_beyond_its_plane_on_the_void_side() {
        let out = snapped(&mesh(), &[region(&[0, 1], false)], 0.08);
        for v in 0..4 {
            assert!((out.positions[v].z - 0.08).abs() < 1e-12, "vertex {v}: {}", out.positions[v].z);
        }
        assert!((out.positions[0].x - 0.0).abs() < 1e-12, "moved along the normal only");
        assert_eq!(out.positions[4], mesh().positions[4], "vertices of no region stay");
    }

    #[test]
    fn an_addition_lies_short_of_its_surface_on_the_material_side() {
        let out = snapped(&mesh(), &[region(&[0, 1], true)], 0.08);
        assert!((out.positions[2].z + 0.08).abs() < 1e-12);
    }

    #[test]
    fn a_hole_wall_moves_into_the_hole_and_a_shared_vertex_goes_with_the_first_region() {
        let hole = SnapRegion {
            triangles: vec![0],
            primitive: Primitive::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: 1.0 },
            convex: false,
            is_addition: false,
        };
        let m = Mesh {
            positions: vec![Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0), Point3::new(0.0, 1.0, 1.0)],
            triangles: vec![[0, 1, 2]],
        };
        let out = snapped(&m, &[hole, region(&[0], false)], 0.1);
        // the hole's void is inside the cylinder: the wall moves toward the axis
        assert!((out.positions[0].x - 0.9).abs() < 1e-9, "{:?}", out.positions[0]);
        assert!((out.positions[1].y - 0.9).abs() < 1e-9, "the plane region did not take the vertex");
    }

    #[test]
    fn hostile_triangle_indices_and_nan_are_skipped() {
        let out = snapped(&mesh(), &[region(&[99, 0], false)], f64::NAN);
        assert_eq!(out.positions, mesh().positions, "a NaN offset moves nothing");
        let out = snapped(&mesh(), &[region(&[99], false)], 0.1);
        assert_eq!(out.positions, mesh().positions);
    }
}
