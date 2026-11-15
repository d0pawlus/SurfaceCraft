//! Fills that would close a piece of the mesh inside out.
//!
//! A scanner records the side of the surface it looked from: the triangles' normals point away from
//! the material. When the fills of a piece close it, the closed piece encloses a volume, and a solid
//! made of it is turned outward (positive volume). If the piece, closed by its patches, has a
//! clearly negative volume, the scanned face points into what was enclosed: the patches cap the
//! space in front of the scanned face, and every scan point would face away from the solid. The
//! mesh cannot tell why: an open sheet seen from its hollow side, or a sheet whose normals were
//! stored inward; either way a loop whose patch alone is a large share of the piece is an open edge,
//! not a hole, and its patch would invent material. Every such loop of an inside-out piece is
//! refused, so no open edge is capped on one side and left open on the other (a tube's two ends).
//! Smaller patches repair holes and are added: a closed scan stored inside out with small holes,
//! however many, is filled, so that it is still told to be inside out, not left open.
use super::Patch;
use super::patch::volume_and_area;
use crate::{Mesh, Point3};
use std::collections::{HashMap, HashSet};

/// A loop of a piece its patches would close inside out is refused when its own patch is at least
/// this share of the area of the piece's own triangles. A hole in a scan is a few percent of it;
/// the cap over the open edge of a sheet is about as large as the sheet's outline (about 90 % for
/// the square cap of a sphere in the tests).
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

/// For every loop of `todo` (whose patches are `patches`): is it left open because, with all the
/// patches in, its piece would be closed inside out, and its own patch is a large share of the piece
/// (see the module)? A piece with a loop in `left` (not filled) stays open and is not judged. The
/// judgement takes every patch of the piece as added; one that `append_if_admitted` drops later would
/// leave the piece open anyway, so the only effect is a large patch refused on a piece that was not
/// going to close: conservative, never an inside-out solid. The mesh is valid.
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
        let (Some(&o), Some(&(volume, scan_area))) = (reference.get(p), sums.get(p)) else { continue };
        let measured: Vec<(usize, f64, f64)> = ks
            .iter()
            .filter_map(|&k| patches.get(k).map(|patch| (k, patch.volume_and_area(mesh, o))))
            .map(|(k, (dv, da))| (k, dv, da))
            .collect();
        if !inside_out(volume, scan_area, &measured) {
            continue;
        }
        for &(k, _, area) in &measured {
            if area >= INWARD_PATCH_SHARE * scan_area
                && let Some(slot) = refused.get_mut(k)
            {
                *slot = true;
            }
        }
    }
    refused
}

/// Whether a piece of six-fold volume `volume` and area `scan_area` (its own triangles), closed by
/// patches of six-fold volumes and areas `(_, volume, area)`, encloses a volume clearly below zero.
/// A volume that is not a number is not judged.
fn inside_out(volume: f64, scan_area: f64, patches: &[(usize, f64, f64)]) -> bool {
    let closed = volume + patches.iter().map(|p| p.1).sum::<f64>();
    let area = scan_area + patches.iter().map(|p| p.2).sum::<f64>();
    closed < 0.0 && closed.abs() > NO_VOLUME_SHARE * area.powf(1.5)
}

#[cfg(test)]
mod tests {
    use crate::holes::{boundary_loops, fill_holes, fill_holes_allowing_inward};
    use crate::{Mesh, Point3, Vec3, synth};

    fn reversed(mut m: Mesh) -> Mesh {
        m.triangles.iter_mut().for_each(|t| t.swap(1, 2));
        m
    }

    fn signed_volume(m: &Mesh) -> f64 {
        (0..m.triangle_count())
            .filter_map(|i| m.triangle(i))
            .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)))
            .sum()
    }

    /// Keeps the triangles whose centroid `keep` accepts.
    fn kept(m: &Mesh, keep: impl Fn(Point3) -> bool) -> Mesh {
        let ids: Vec<u32> =
            (0..m.triangle_count() as u32).filter(|&t| m.face_centroid(t as usize).is_some_and(&keep)).collect();
        m.submesh(&ids)
    }

    /// A ball of radius 15 stored inside out (normals toward the centre).
    fn ball_inside_out() -> Mesh {
        let ball = synth::sphere(Point3::origin(), 15.0, 48, 24);
        assert!(signed_volume(&ball) > 0.0);
        reversed(ball)
    }

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

    /// Twelve holes of about 3 % of a ball stored inside out each, 36 % in all: every one repairs a
    /// hole, so all are filled and the closed scan is still inside out (none is left open on its own).
    #[test]
    fn many_small_holes_of_a_ball_stored_inside_out_are_all_filled() {
        // Arrange: discs around the vertices of an icosahedron, well apart
        let g = (1.0 + 5f64.sqrt()) / 2.0;
        let mut centres = Vec::new();
        for (a, b) in [(1.0, g), (-1.0, g), (1.0, -g), (-1.0, -g)] {
            centres.extend([Vec3::new(0.0, a, b), Vec3::new(a, b, 0.0), Vec3::new(b, 0.0, a)]);
        }
        let centres: Vec<Point3> = centres.iter().map(|c| Point3::from(c.normalize() * 15.0)).collect();
        let mut ball = kept(&ball_inside_out(), |c| centres.iter().all(|h| (c - h).norm() > 5.2));
        assert_eq!(boundary_loops(&ball).len(), 12);

        // Act
        let rep = fill_holes(&mut ball, 1000, true);

        // Assert
        assert_eq!((rep.filled, rep.inward), (12, 0));
        assert!(boundary_loops(&ball).is_empty());
        assert!(signed_volume(&ball) < 0.0, "still inside out, to be reported so");
    }

    /// A tube stored inside out, both ends about a third of its side: both ends stay open, neither is
    /// capped alone.
    #[test]
    fn both_ends_of_a_tube_stored_inside_out_stay_open() {
        // Arrange
        let tube = synth::cylinder(Point3::origin(), Vec3::z(), 10.0, 15.0, 48, 12);
        let mut capped = tube.clone();
        fill_holes_allowing_inward(&mut capped, 1000, false);
        let mut tube = if signed_volume(&capped) > 0.0 { reversed(tube) } else { tube };
        let before = tube.clone();

        // Act
        let rep = fill_holes(&mut tube, 1000, true);

        // Assert
        assert_eq!((rep.filled, rep.skipped, rep.inward), (0, 2, 2));
        assert_eq!(tube, before);
    }

    /// A ball stored inside out with a hole of about 30 % of it: that patch would invent as much as a
    /// sheet's cap, so it is refused and counted in `inward` (the caller reports it).
    #[test]
    fn a_hole_of_a_third_of_a_ball_stored_inside_out_is_refused_and_counted() {
        // Arrange
        let mut ball = kept(&ball_inside_out(), |c| c.z < 6.0);
        assert_eq!(boundary_loops(&ball).len(), 1);

        // Act
        let rep = fill_holes(&mut ball, 1000, true);

        // Assert
        assert_eq!((rep.filled, rep.inward), (0, 1));
        assert_eq!(boundary_loops(&ball).len(), 1);
    }

    /// Two pieces in one mesh are judged each on its own.
    #[test]
    fn a_hollow_and_a_convex_dish_in_one_mesh_are_judged_apart() {
        // Arrange
        let mut both = dish(true);
        let convex = dish(false);
        let offset = both.positions.len() as u32;
        both.positions.extend(convex.positions.iter().map(|p| p + Vec3::new(100.0, 0.0, 0.0)));
        both.triangles.extend(convex.triangles.iter().map(|t| t.map(|v| v + offset)));

        // Act
        let rep = fill_holes(&mut both, 1000, true);

        // Assert
        assert_eq!((rep.filled, rep.inward), (1, 1));
        let left = boundary_loops(&both);
        assert_eq!(left.len(), 1);
        assert!(left[0].iter().all(|&v| v < offset), "the hollow dish keeps its rim");
    }

    /// A caller who wants the cap (a mould of the dish) gets it.
    #[test]
    fn the_rim_of_a_hollow_dish_is_capped_when_inward_fills_are_allowed() {
        // Arrange
        let mut m = dish(true);

        // Act
        let rep = fill_holes_allowing_inward(&mut m, 1000, true);

        // Assert
        assert_eq!((rep.filled, rep.inward), (1, 0));
        assert!(boundary_loops(&m).is_empty());
    }

    /// Positions that are not numbers or that all coincide never panic and are never judged inward.
    #[test]
    fn hostile_positions_are_not_judged_inward() {
        let mut nan = dish(true);
        nan.positions[300] = Point3::new(f64::NAN, 0.0, f64::INFINITY);
        let mut point = dish(true);
        point.positions.iter_mut().for_each(|p| *p = Point3::origin());
        for (name, mut m) in [("nan", nan), ("point", point)] {
            // Act
            let rep = fill_holes(&mut m, 1000, true);

            // Assert
            assert_eq!(rep.inward, 0, "{name}");
        }
    }
}
