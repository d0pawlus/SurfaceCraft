//! Faired patches that pass through the mesh.
//!
//! The thin-plate fairing makes a patch follow the curvature of the surface around its loop; over
//! a curved rim (the square cap of a sphere) it continues that surface past the rim until the patch
//! passes through the mesh and through itself. A mesh closed by such a patch crosses itself: its
//! volume is still measured, but a solid wrapped around it has no outside a ray can tell, and the
//! kernel's healer turned the whole solid inside out. Such a patch is faired again as a membrane,
//! which stays within the convex hull of its loop.
use crate::nesting::{crosses, overlap};
use crate::{Aabb, Bvh, Mesh, Point3};

/// The triangles a patch must not cross: those of the mesh it fills, in a tree built once and
/// shared by the holes filled at the same time. (Built before they start: a tree built lazily by
/// the first hole, itself in parallel, could make that thread take up another hole that waits for
/// the same tree.)
pub(super) struct Crossings<'a> {
    mesh: &'a Mesh,
    tree: Bvh,
}

impl<'a> Crossings<'a> {
    pub(super) fn new(mesh: &'a Mesh) -> Self {
        Self { mesh, tree: Bvh::build(mesh) }
    }

    /// Does a triangle of the patch (the triangles of the workspace `work` from `first_tri` on)
    /// cross a triangle of the mesh, or another triangle of the patch, that it shares no vertex
    /// with? `origin` is the mesh vertex of every workspace vertex that was in the mesh; the
    /// patch's own vertices come after them.
    pub(super) fn patch_crosses(&self, origin: &[u32], work: &Mesh, first_tri: usize) -> bool {
        let patch = work.triangles.get(first_tri..).unwrap_or_default();
        if patch.is_empty() {
            return false;
        }
        let own = Bvh::build(&Mesh { positions: work.positions.clone(), triangles: patch.to_vec() });
        let corners = |t: &[u32; 3]| -> Option<[Point3; 3]> {
            Some([
                *work.positions.get(t[0] as usize)?,
                *work.positions.get(t[1] as usize)?,
                *work.positions.get(t[2] as usize)?,
            ])
        };
        patch.iter().enumerate().any(|(i, t)| {
            let Some(x) = corners(t) else { return false };
            let bx = Aabb::from_points(x.iter());
            let in_mesh = t.map(|v| origin.get(v as usize).copied());
            let crosses_mesh = self.tree.candidates(&|b| overlap(b, &bx)).into_iter().any(|o| {
                let (Some(y), Some(oi)) = (self.mesh.triangle(o as usize), self.mesh.triangles.get(o as usize)) else {
                    return false;
                };
                !in_mesh.iter().flatten().any(|v| oi.contains(v)) && crosses(&x, &y)
            });
            crosses_mesh
                || own.candidates(&|b| overlap(b, &bx)).into_iter().any(|o| {
                    let (Some(oi), Some(y)) = (patch.get(o as usize), patch.get(o as usize).and_then(corners)) else {
                        return false;
                    };
                    o as usize != i && !t.iter().any(|v| oi.contains(v)) && crosses(&x, &y)
                })
        })
    }
}

#[cfg(test)]
mod tests {
    use crate::holes::fill_holes;
    use crate::{Bvh, Mesh, Point3, Vec3, nesting, synth};

    const RADIUS: f64 = 30.0;
    const HALF: f64 = 18.0;
    const CELLS: usize = 30;

    /// A square cap of a sphere seen from its convex side: its rim rises from the corners to the
    /// middle of each side, and the thin-plate fairing continued the dome past the rim until the
    /// patch passed through the dome near the corners.
    fn convex_dish() -> Mesh {
        let corner = Point3::new(-HALF, -HALF, 0.0);
        let mut m = synth::plane(corner, Vec3::x(), Vec3::y(), 2.0 * HALF, 2.0 * HALF, CELLS, CELLS);
        for p in &mut m.positions {
            p.z = (RADIUS * RADIUS - p.x * p.x - p.y * p.y).sqrt();
        }
        m
    }

    fn crossing_triangles(m: &Mesh) -> usize {
        let bvh = Bvh::build(m);
        nesting::sampled_self_intersections(m, &bvh, m.triangle_count()).crossing
    }

    #[test]
    fn the_smooth_cap_of_a_convex_dish_does_not_pass_through_the_dish() {
        // Arrange
        let mut m = convex_dish();
        let rim_edges = 4 * CELLS;

        // Act
        let rep = fill_holes(&mut m, 2 * rim_edges, true);

        // Assert
        assert_eq!((rep.filled, rep.skipped), (1, 0), "{rep:?}");
        assert_eq!(crossing_triangles(&m), 0);
    }
}
