//! Fills that would close a piece of the mesh inside out.
//!
//! A scanner records the side of the surface it looked from: the triangles' normals point away from
//! the material. When the fills of a piece close it, the closed piece encloses a volume, and a solid
//! made of it is turned outward (positive volume). If the piece, closed by its patches, has a
//! clearly negative volume, the scanned face points into what was enclosed: the patches cap the
//! space in front of the scanned face, and every scan point would face away from the solid. The
//! mesh cannot tell why: an open sheet seen from its hollow side, or a sheet whose normals were
//! stored inward; either way the loop is its open edge, not a hole, and the closed solid would be
//! invented material. Such a fill is
//! refused when the patches are a large share of the piece: they invent a face rather than repair a
//! hole. A closed scan stored inside out with small holes is still filled (its patches are small),
//! so that it is still told to be inside out, not left open.
use super::Patch;
use crate::{Mesh, Point3};
use std::collections::{HashMap, HashSet};

/// The patches that close a piece inside out are refused when their area is at least this share of
/// the area of the piece's own triangles. A hole in a scan is a few percent of it; the cap over the
/// open edge of a sheet is about as large as the sheet's outline (about 90 % for the square cap of a
/// sphere in the tests).
const INWARD_PATCH_SHARE: f64 = 0.25;

/// A closed piece whose six-fold volume is below this share of its area to the power 3/2 is a
/// nearly flat sheet capped nearly flat: it encloses no volume worth an orientation (its sign is the
/// scanner noise) and it is filled. The square cap of a sphere in the tests reads about 0.4, a
/// sphere 0.56.
const NO_VOLUME_SHARE: f64 = 1e-3;

/// Connected pieces of `mesh`: the root vertex of each vertex (vertices joined through triangles).
/// The mesh is valid (every index in range).
fn vertex_pieces(mesh: &Mesh) -> Vec<u32> {
    fn find(parent: &mut [u32], mut x: u32) -> u32 {
        while parent[x as usize] != x {
            let up = parent[parent[x as usize] as usize];
            parent[x as usize] = up;
            x = up;
        }
        x
    }
    let mut parent: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    for t in &mesh.triangles {
        let a = find(&mut parent, t[0]);
        for &v in &t[1..] {
            let b = find(&mut parent, v);
            if a != b {
                parent[b as usize] = a;
            }
        }
    }
    (0..parent.len() as u32).map(|v| find(&mut parent, v)).collect()
}

/// Six times the signed volume of the tetrahedron from `o` over the triangle, and the triangle's area.
fn volume_and_area(o: Point3, [a, b, c]: [Point3; 3]) -> (f64, f64) {
    let (a, b, c) = (a - o, b - o, c - o);
    (a.dot(&b.cross(&c)), 0.5 * (b - a).cross(&(c - a)).norm())
}

impl Patch {
    /// The position of workspace vertex `v`: a mesh vertex or one of the patch's own.
    fn point(&self, mesh: &Mesh, v: u32) -> Option<Point3> {
        let base = self.origin.len() as u32;
        match self.origin.get(v as usize) {
            Some(&m) if v < base => mesh.positions.get(m as usize).copied(),
            _ => self.positions.get((v - base) as usize).copied(),
        }
    }

    /// Six times the signed volume over `o` and the area of the patch's triangles.
    fn volume_and_area(&self, mesh: &Mesh, o: Point3) -> (f64, f64) {
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

/// For every loop of `todo` (whose patches are `patches`): is it left open because, with all the
/// patches in, its piece would be closed inside out by patches that are a large share of it (see the
/// module)? Of such a piece only the loop with the largest patch is refused; the piece then stays
/// open and its other holes are filled. A piece with a loop in `left` (not filled) stays open and is
/// not judged. The mesh is valid.
pub(super) fn refused(mesh: &Mesh, todo: &[Vec<u32>], patches: &[Patch], left: &[Vec<u32>]) -> Vec<bool> {
    let mut refused = vec![false; todo.len()];
    let piece = vertex_pieces(mesh);
    let piece_of = |lp: &[u32]| lp.first().and_then(|&v| piece.get(v as usize).copied());
    let open: HashSet<u32> = left.iter().filter_map(|lp| piece_of(lp)).collect();
    let mut loops_of: HashMap<u32, Vec<usize>> = HashMap::new();
    for (k, lp) in todo.iter().enumerate() {
        if let Some(p) = piece_of(lp).filter(|p| !open.contains(p)) {
            loops_of.entry(p).or_default().push(k);
        }
    }
    if loops_of.is_empty() {
        return refused;
    }
    // the sums are taken over a point of the piece: the closed volume does not depend on it, and a
    // point nearby keeps the sum well conditioned
    let reference: HashMap<u32, Point3> = loops_of
        .iter()
        .filter_map(|(&p, ks)| {
            let v = ks.first().and_then(|&k| todo.get(k)).and_then(|lp| lp.first())?;
            Some((p, *mesh.positions.get(*v as usize)?))
        })
        .collect();
    let mut sums: HashMap<u32, (f64, f64)> = HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        let p = piece.get(t[0] as usize).copied().unwrap_or(u32::MAX);
        let (Some(&o), Some(corners)) = (reference.get(&p), mesh.triangle(i)) else { continue };
        let (dv, da) = volume_and_area(o, corners);
        let s = sums.entry(p).or_insert((0.0, 0.0));
        *s = (s.0 + dv, s.1 + da);
    }
    for (p, ks) in &loops_of {
        let (Some(&o), Some(&(mut volume, scan_area))) = (reference.get(p), sums.get(p)) else { continue };
        let mut patch_area = 0.0;
        let mut largest: Option<(f64, usize)> = None;
        for &k in ks {
            let Some(patch) = patches.get(k) else { continue };
            let (dv, da) = patch.volume_and_area(mesh, o);
            volume += dv;
            patch_area += da;
            if largest.is_none_or(|(a, _)| da > a) {
                largest = Some((da, k));
            }
        }
        let enclosed = volume.abs() > NO_VOLUME_SHARE * (scan_area + patch_area).powf(1.5);
        let inward = volume < 0.0 && enclosed;
        if inward
            && patch_area >= INWARD_PATCH_SHARE * scan_area
            && let Some((_, k)) = largest
            && let Some(slot) = refused.get_mut(k)
        {
            *slot = true;
        }
    }
    refused
}

#[cfg(test)]
mod tests {
    use crate::holes::{boundary_loops, fill_holes};
    use crate::{Mesh, Point3, Vec3, synth};

    /// A square cap of a sphere of radius 30, normals away from the centre (seen from its convex
    /// side) or, `hollow`, into the hollow (seen from inside, as a scanner looking into a dish).
    fn dish(hollow: bool) -> Mesh {
        let mut m = synth::plane(Point3::new(-18.0, -18.0, 0.0), Vec3::x(), Vec3::y(), 36.0, 36.0, 24, 24);
        for p in &mut m.positions {
            p.z = (900.0 - p.x * p.x - p.y * p.y).sqrt();
        }
        if hollow {
            m.triangles.iter_mut().for_each(|t| t.swap(1, 2));
        }
        m
    }

    #[test]
    fn the_rim_of_a_dish_seen_from_its_hollow_side_is_left_open() {
        for smooth in [false, true] {
            // Arrange
            let mut m = dish(true);
            let before = m.clone();

            // Act
            let rep = fill_holes(&mut m, 1000, smooth);

            // Assert
            assert_eq!((rep.filled, rep.skipped, rep.inward), (0, 1, 1), "smooth {smooth}");
            assert_eq!(m, before, "smooth {smooth}: nothing added");
        }
    }

    #[test]
    fn the_rim_of_a_dish_seen_from_its_convex_side_is_filled() {
        for smooth in [false, true] {
            // Arrange
            let mut m = dish(false);

            // Act
            let rep = fill_holes(&mut m, 1000, smooth);

            // Assert
            assert_eq!((rep.filled, rep.skipped, rep.inward), (1, 0, 0), "smooth {smooth}");
            assert!(boundary_loops(&m).is_empty());
        }
    }

    /// The other holes of a piece whose rim stays open are filled.
    #[test]
    fn only_the_rim_stays_open_the_holes_inside_the_dish_are_filled() {
        // Arrange: two small holes away from the rim
        let m = dish(true);
        let near = |c: Point3| [(-6.0, -6.0), (7.0, 5.0)].iter().any(|&(x, y)| (c.x - x).hypot(c.y - y) < 2.0);
        let kept: Vec<u32> =
            (0..m.triangle_count() as u32).filter(|&t| m.face_centroid(t as usize).is_some_and(|c| !near(c))).collect();
        let mut m = m.submesh(&kept);
        assert_eq!(boundary_loops(&m).len(), 3);

        // Act
        let rep = fill_holes(&mut m, 1000, true);

        // Assert
        assert_eq!((rep.filled, rep.skipped, rep.inward), (2, 1, 1));
        assert_eq!(boundary_loops(&m).len(), 1);
    }

    /// A closed ball stored inside out with a small hole is repaired: its patch is small beside it.
    #[test]
    fn a_small_hole_of_a_ball_stored_inside_out_is_filled() {
        // Arrange
        let mut ball = synth::sphere(Point3::origin(), 15.0, 48, 24);
        ball.triangles.iter_mut().for_each(|t| t.swap(1, 2));
        let middle = ball.triangle_count() as u32 / 2;
        let kept: Vec<u32> = (0..ball.triangle_count() as u32).filter(|&t| t != middle).collect();
        let mut ball = ball.submesh(&kept);

        // Act
        let rep = fill_holes(&mut ball, 1000, true);

        // Assert
        assert_eq!((rep.filled, rep.inward), (1, 0));
        assert!(boundary_loops(&ball).is_empty());
    }
}
