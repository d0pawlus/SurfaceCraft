//! The prepared copy of the scan that the hybrid method wraps (P9-O6b): the vertices of every
//! protected region that lie within a distance of their primitive are put exactly on it (projected by the
//! kernel's `put_on_all`, which the merge's snap uses too, never pushed beyond), so the patches fitted over
//! the region lie on the primitive and can be merged into one analytic face. The rest of the scan is untouched. The judge never sees this copy:
//! the body is measured against the original scan.
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Mesh, Point3};
use surfacecraft_kernel::hybrid::Shape;
use surfacecraft_kernel::merge::placed;

/// The primitive of a kernel shape and back: the same parameters.
pub(super) fn shape_of(p: &Primitive) -> Shape {
    match p {
        Primitive::Plane { point, normal } => Shape::Plane { point: *point, normal: *normal },
        Primitive::Cylinder { point, axis, radius } => Shape::Cylinder { point: *point, axis: *axis, radius: *radius },
        Primitive::Cone { apex, axis, half_angle } => Shape::Cone { apex: *apex, axis: *axis, half_angle: *half_angle },
        Primitive::Sphere { center, radius } => Shape::Sphere { center: *center, radius: *radius },
        Primitive::Torus { center, axis, major_radius, minor_radius } => {
            Shape::Torus { center: *center, axis: *axis, major_radius: *major_radius, minor_radius: *minor_radius }
        }
    }
}

/// A copy of `mesh` with the vertices of each region (given as triangle lists with their primitive)
/// moved onto their primitives where that moves them by `max_move` at most: onto the primitive of the
/// region, and onto all of them where several regions meet (a sharp edge, so that the patches on
/// both sides end on it). Hostile input (triangle indices out of range, NaN, a negative or NaN
/// `max_move`) moves nothing it cannot account for.
pub(super) fn projected(mesh: &Mesh, regions: &[(&[u32], &Primitive)], max_move: f64) -> Mesh {
    let mut owners: Vec<Vec<usize>> = vec![Vec::new(); mesh.positions.len()];
    for (k, (triangles, _)) in regions.iter().enumerate() {
        for &t in *triangles {
            let Some(tri) = mesh.triangles.get(t as usize) else { continue };
            for &v in tri {
                if let Some(list) = owners.get_mut(v as usize)
                    && list.last() != Some(&k)
                {
                    list.push(k);
                }
            }
        }
    }
    // the kernel's bounded projection onto all of a vertex's primitives (the nearest one where they do not meet),
    // with the fit's closest point
    let mut positions = mesh.positions.clone();
    for (slot, (list, p)) in positions.iter_mut().zip(owners.iter().zip(&mesh.positions)) {
        let prims: Vec<&Primitive> = list.iter().filter_map(|&k| regions.get(k).map(|r| r.1)).collect();
        let closest = |i: usize, q: [f64; 3]| {
            let c = prims.get(i)?.project(&Point3::new(q[0], q[1], q[2]));
            Some([c.x, c.y, c.z])
        };
        if let Some((q, _)) = placed(&closest, prims.len(), [p.x, p.y, p.z], max_move) {
            *slot = Point3::new(q[0], q[1], q[2]);
        }
    }
    Mesh { positions, triangles: mesh.triangles.clone() }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two triangles of a unit square near z = 0 and one triangle off it, sharing vertex 1.
    fn mesh() -> Mesh {
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        Mesh {
            positions: vec![
                p(0.0, 0.0, 0.01),
                p(1.0, 0.0, -0.01),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.3),
                p(2.0, 0.0, 5.0),
            ],
            triangles: vec![[0, 1, 2], [0, 2, 3], [1, 4, 2]],
        }
    }

    fn plane() -> Primitive {
        Primitive::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] }
    }

    #[test]
    fn vertices_within_the_distance_go_exactly_onto_the_primitive_and_the_rest_stay() {
        let plane = plane();
        let out = projected(&mesh(), &[(&[0, 1], &plane)], 0.1);
        for v in [0, 1, 2] {
            assert!(out.positions[v].z.abs() < 1e-15, "vertex {v}: {}", out.positions[v].z);
        }
        assert_eq!(out.positions[0].x, 0.0, "moved along the normal only");
        assert_eq!(out.positions[3], mesh().positions[3], "0.3 away is beyond the distance: it stays");
        assert_eq!(out.positions[4], mesh().positions[4], "vertices of no region stay");
    }

    #[test]
    fn a_vertex_of_two_regions_goes_onto_the_intersection_of_their_primitives() {
        let a = plane();
        let b = Primitive::Plane { point: [0.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0] };
        let m = Mesh {
            positions: vec![Point3::new(0.05, 0.0, 0.07), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            triangles: vec![[0, 1, 2], [0, 2, 1]],
        };
        let out = projected(&m, &[(&[0], &a), (&[1], &b)], 1.0);
        assert!(out.positions[0].x.abs() < 1e-6 && out.positions[0].z.abs() < 1e-6, "{:?}", out.positions[0]);
    }

    #[test]
    fn a_vertex_of_two_parallel_regions_stays_where_it_is() {
        let (a, b) = (plane(), Primitive::Plane { point: [0.0, 0.0, 0.5], normal: [0.0, 0.0, 1.0] });
        let out = projected(&mesh(), &[(&[0], &a), (&[1], &b)], 1.0);
        // no point is on both planes: the shared vertices are not moved (parallel surfaces are refused, not projected
        // onto the first)
        assert_eq!((out.positions[0], out.positions[2]), (mesh().positions[0], mesh().positions[2]));
        assert!((out.positions[3].z - 0.5).abs() < 1e-15, "the vertex only the second region has");
    }

    #[test]
    fn hostile_indices_nan_and_negative_distances_move_nothing() {
        let plane = plane();
        for max_move in [f64::NAN, -1.0, 0.0] {
            let out = projected(&mesh(), &[(&[99, 0], &plane)], max_move);
            assert_eq!(out.positions[1], mesh().positions[1], "{max_move}");
        }
        assert_eq!(projected(&mesh(), &[(&[99], &plane)], 1.0).positions, mesh().positions);
        assert_eq!(projected(&mesh(), &[], 1.0).positions, mesh().positions);
    }
}
