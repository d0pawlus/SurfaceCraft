//! What filling one hole adds (a patch, in the numbers of its workspace), and what it measures.
use crate::{Mesh, Point3};

/// What filling one hole added, in the vertex numbers of its workspace.
pub(super) struct Patch {
    /// Mesh vertex of every workspace vertex that was in the mesh.
    pub(super) origin: Vec<u32>,
    /// New vertices (workspace numbers from `origin.len()` on).
    pub(super) positions: Vec<Point3>,
    /// New triangles.
    pub(super) triangles: Vec<[u32; 3]>,
}

impl Patch {
    /// The triangles in the numbers of a mesh whose next new vertex is `first`.
    pub(super) fn mapped(&self, first: u32) -> Vec<[u32; 3]> {
        let base = self.origin.len() as u32;
        let map = |v: u32| match self.origin.get(v as usize) {
            Some(&m) if v < base => m,
            _ => first + (v - base),
        };
        self.triangles.iter().map(|t| t.map(map)).collect()
    }

    pub(super) fn append_to(self, mesh: &mut Mesh) {
        let triangles = self.mapped(mesh.positions.len() as u32);
        mesh.triangles.extend(triangles);
        mesh.positions.extend(self.positions);
    }

    /// The position of workspace vertex `v`: a mesh vertex or one of the patch's own.
    fn point(&self, mesh: &Mesh, v: u32) -> Option<Point3> {
        let base = self.origin.len() as u32;
        match self.origin.get(v as usize) {
            Some(&m) if v < base => mesh.positions.get(m as usize).copied(),
            _ => self.positions.get((v - base) as usize).copied(),
        }
    }

    /// Six times the signed volume over `o` and the area of the patch's triangles.
    pub(super) fn volume_and_area(&self, mesh: &Mesh, o: Point3) -> (f64, f64) {
        self.triangles.iter().fold((0.0, 0.0), |(v, a), t| {
            let corners = [self.point(mesh, t[0]), self.point(mesh, t[1]), self.point(mesh, t[2])];
            match corners {
                [Some(p), Some(q), Some(r)] => {
                    let (dv, da) = volume_and_area(o, [p, q, r]);
                    (v + dv, a + da)
                }
                _ => (v, a),
            }
        })
    }
}

/// Six times the signed volume of the tetrahedron from `o` over the triangle, and the triangle's area.
pub(super) fn volume_and_area(o: Point3, [a, b, c]: [Point3; 3]) -> (f64, f64) {
    let (a, b, c) = (a - o, b - o, c - o);
    (a.dot(&b.cross(&c)), 0.5 * (b - a).cross(&(c - a)).norm())
}
