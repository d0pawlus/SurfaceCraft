//! Faired patches that pass through the mesh.
//!
//! The thin-plate fairing makes a patch follow the curvature of the surface around its loop; over
//! a curved rim (the square cap of a sphere) it continues that surface past the rim until the patch
//! passes through the mesh and through itself. A mesh closed by such a patch crosses itself: its
//! volume is still measured, but a solid wrapped around it has no well-defined outside, and the
//! kernel's healer turned the whole solid inside out. Such a patch is faired again as a membrane,
//! which stays within the convex hull of its loop, and the membrane patch is kept when it does not
//! cross (or when the thin-plate patch is not finite).
//!
//! Not checked: a patch against the patches of the other holes filled in the same call, or against
//! a patch accepted before it is made again (the tree holds the mesh as it was); a thin-plate patch
//! that bulges farther than its loop's diagonal from the loop's box, against the mesh beyond that
//! (the tree holds only the triangles within it); and the patch after the sharpening, which runs on
//! the patch that was kept.
use crate::nesting::{crosses, overlap};
use crate::{Aabb, Bvh, Mesh, Point3, finite, par, tri};

/// Which fairing a smooth patch kept (tests read it).
#[cfg(test)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Fairing {
    ThinPlate,
    Membrane,
}

#[cfg(test)]
thread_local! {
    /// The fairing the last smooth patch on this thread kept.
    pub(super) static LAST_FAIRING: std::cell::Cell<Option<Fairing>> = const { std::cell::Cell::new(None) };
}

#[cfg(test)]
fn kept(f: Fairing) {
    LAST_FAIRING.with(|c| c.set(Some(f)));
}

/// The triangles a patch must not cross: those of the mesh near the loops about to be filled (whose
/// box, grown by the loop's diameter, they overlap), as a mesh of their own with a tree over it.
/// Built once before the holes are filled in parallel and kept for the refills: it owns its data.
pub(super) struct Crossings {
    near: Mesh,
    /// The mesh vertex of every vertex of `near`.
    global: Vec<u32>,
    tree: Bvh,
}

/// A vertex of a triangle under test: its number in the mesh (`None` for a patch's own vertex,
/// which no mesh triangle has).
type Corner = Option<u32>;

impl Crossings {
    pub(super) fn new(mesh: &Mesh, loops: &[Vec<u32>]) -> Self {
        let picked = near_loops(mesh, loops);
        let mut local: std::collections::HashMap<u32, u32> = std::collections::HashMap::new();
        let mut global: Vec<u32> = Vec::new();
        let mut positions: Vec<Point3> = Vec::new();
        let mut triangles: Vec<[u32; 3]> = Vec::with_capacity(picked.len());
        for t in picked {
            let (Some(idx), Some(corners)) = (mesh.triangles.get(t), mesh.triangle(t)) else { continue };
            let mut out = [0u32; 3];
            for ((slot, &v), p) in out.iter_mut().zip(idx).zip(corners) {
                *slot = *local.entry(v).or_insert_with(|| {
                    global.push(v);
                    positions.push(p);
                    (positions.len() - 1) as u32
                });
            }
            triangles.push(out);
        }
        let near = Mesh { positions, triangles };
        let tree = Bvh::build(&near);
        Self { near, global, tree }
    }

    /// Triangles in the tree.
    #[cfg(test)]
    pub(super) fn len(&self) -> usize {
        self.near.triangle_count()
    }

    /// Keeps the thin-plate patch of the workspace `work` (its triangles from `first_tri` on, its
    /// own faired vertices from `first_new` on) unless it crosses; then puts the `membrane`
    /// positions of those vertices in its place, and keeps them unless they cross too and the
    /// thin-plate patch is finite. `origin` is the mesh vertex of every workspace vertex that was in
    /// the mesh.
    pub(super) fn settle(
        &self,
        origin: &[u32],
        work: &mut Mesh,
        (first_tri, first_new): (usize, usize),
        mut membrane: Vec<Point3>,
    ) {
        if membrane.is_empty() || first_new >= work.positions.len() {
            return; // no interior vertices: nothing was faired
        }
        let thin_finite = work.positions.get(first_new..).is_some_and(|p| p.iter().all(finite));
        let membrane_kept = !(thin_finite && !self.patch_crosses(origin, work, first_tri))
            && self.try_membrane(origin, work, (first_tri, first_new), &mut membrane, thin_finite);
        #[cfg(test)]
        kept(if membrane_kept { Fairing::Membrane } else { Fairing::ThinPlate });
        #[cfg(not(test))]
        let _ = membrane_kept;
    }

    /// Puts the `membrane` positions in place of the patch's own vertices and keeps them unless they
    /// cross too and the thin-plate patch is finite (then the thin-plate positions go back). A
    /// membrane that is not finite is never put in. Returns whether the membrane was kept.
    fn try_membrane(
        &self,
        origin: &[u32],
        work: &mut Mesh,
        (first_tri, first_new): (usize, usize),
        membrane: &mut [Point3],
        thin_finite: bool,
    ) -> bool {
        if !membrane.iter().all(finite) {
            return false;
        }
        let Some(own) = work.positions.get_mut(first_new..).filter(|p| p.len() == membrane.len()) else { return false };
        own.swap_with_slice(membrane);
        if thin_finite && self.patch_crosses(origin, work, first_tri) {
            if let Some(own) = work.positions.get_mut(first_new..) {
                own.swap_with_slice(membrane);
            }
            return false;
        }
        true
    }

    /// Does a triangle of the patch (the triangles of the workspace `work` from `first_tri` on)
    /// cross a triangle of the mesh near it or another triangle of the patch (see [`pair_crosses`])?
    /// A patch with a corner that is not finite counts as crossing.
    pub(super) fn patch_crosses(&self, origin: &[u32], work: &Mesh, first_tri: usize) -> bool {
        let patch = work.triangles.get(first_tri..).unwrap_or_default();
        let corners: Option<Vec<[Point3; 3]>> = patch
            .iter()
            .map(|t| {
                Some([
                    *work.positions.get(t[0] as usize)?,
                    *work.positions.get(t[1] as usize)?,
                    *work.positions.get(t[2] as usize)?,
                ])
            })
            .collect();
        let Some(corners) = corners else { return true };
        if corners.iter().flatten().any(|p| !finite(p)) {
            return true;
        }
        let n = corners.len() as u32;
        let own = Bvh::build_owned(Mesh {
            positions: corners.iter().flatten().copied().collect(),
            triangles: (0..n).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect(),
        });
        let in_mesh = |t: &[u32; 3]| -> [Corner; 3] { t.map(|v| origin.get(v as usize).copied()) };
        patch.iter().zip(&corners).enumerate().any(|(i, (t, x))| {
            let bx = Aabb::from_points(x.iter());
            let kx = in_mesh(t);
            let with_mesh = self.tree.candidates(&|b| overlap(b, &bx)).into_iter().any(|o| {
                let (Some(oi), Some(y)) = (self.near.triangles.get(o as usize), self.near.triangle(o as usize)) else {
                    return false;
                };
                pair_crosses((x, kx), (&y, oi.map(|v| self.global.get(v as usize).copied())))
            });
            with_mesh
                || own.candidates(&|b| overlap(b, &bx)).into_iter().any(|o| {
                    let (Some(oi), Some(y)) = (patch.get(o as usize), corners.get(o as usize)) else { return false };
                    o as usize != i && pair_crosses((x, t.map(Some)), (y, oi.map(Some)))
                })
        })
    }
}

/// The triangles of `mesh` whose box overlaps the box of a loop grown by the loop's diameter (a
/// patch stays near its loop), ascending.
fn near_loops(mesh: &Mesh, loops: &[Vec<u32>]) -> Vec<usize> {
    let boxes: Vec<Aabb> = loops
        .iter()
        .map(|lp| grown(Aabb::from_points(lp.iter().filter_map(|&v| mesh.positions.get(v as usize)))))
        .filter(|b| !b.is_empty())
        .collect();
    let mut all = Aabb::empty();
    for b in &boxes {
        all.grow(&b.min);
        all.grow(&b.max);
    }
    // a tree over the boxes: each box is the degenerate triangle on its two corners
    let n = boxes.len() as u32;
    let box_tree = Bvh::build_owned(Mesh {
        positions: boxes.iter().flat_map(|b| [b.min, b.max, b.max]).collect(),
        triangles: (0..n).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect(),
    });
    let keep = par::map(mesh.triangle_count(), |t| {
        let Some(corners) = mesh.triangle(t) else { return false };
        let tb = Aabb::from_points(corners.iter());
        overlap(&all, &tb)
            && box_tree
                .candidates(&|b| overlap(b, &tb))
                .into_iter()
                .any(|k| boxes.get(k as usize).is_some_and(|b| overlap(b, &tb)))
    });
    keep.into_iter().enumerate().filter_map(|(t, k)| k.then_some(t)).collect()
}

/// The box grown by its diagonal on every side.
fn grown(b: Aabb) -> Aabb {
    if b.is_empty() {
        return b;
    }
    let d = b.diagonal();
    let pad = crate::Vec3::new(d, d, d);
    Aabb { min: b.min - pad, max: b.max + pad }
}

/// Does triangle `x` cross triangle `y`? Each comes with its corners' vertex numbers (`None` never
/// matches). Without a shared corner: an edge of one passes through the other. With one: the edge
/// of either opposite the shared corner passes through the other (a patch folded across its rim at
/// a loop vertex). With two: no; two triangles in different planes that share an edge meet only
/// along it, however small the angle between them (the knife edge where the cap of a thin dish
/// meets its rim is a few degrees, and is not a crossing).
fn pair_crosses((x, kx): (&[Point3; 3], [Corner; 3]), (y, ky): (&[Point3; 3], [Corner; 3])) -> bool {
    let mut shared: Vec<(usize, usize)> = Vec::with_capacity(3);
    for (i, a) in kx.iter().enumerate() {
        for (j, b) in ky.iter().enumerate() {
            if a.is_some() && a == b {
                shared.push((i, j));
            }
        }
    }
    match shared.as_slice() {
        [] => crosses(x, y),
        [(i, j)] => opposite_edge_hits(x, *i, y) || opposite_edge_hits(y, *j, x),
        _ => false,
    }
}

/// Does the edge of `s` opposite its corner `i` pass through triangle `t`?
fn opposite_edge_hits(s: &[Point3; 3], i: usize, t: &[Point3; 3]) -> bool {
    let (Some(p), Some(q)) = (s.get((i + 1) % 3), s.get((i + 2) % 3)) else { return false };
    let [a, b, c] = t;
    tri::ray_hit(p, &(q - p), a, b, c).is_some_and(|u| u <= 1.0)
}

#[cfg(test)]
mod tests {
    use super::{Crossings, Fairing, LAST_FAIRING, pair_crosses};
    use crate::holes::patch::Patch;
    use crate::holes::{EdgeUses, VertexFaces, boundary_loops, fill_holes, fill_in_workspace};
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

    /// A square sheet of `n` x `n` unit cells around the origin, lifted by `lift`, with the middle
    /// `hole` x `hole` cells taken out.
    fn sheet_with_hole(n: usize, hole: usize, lift: impl Fn(f64, f64) -> f64) -> Mesh {
        let half = n as f64 / 2.0;
        let mut m = synth::plane(Point3::new(-half, -half, 0.0), Vec3::x(), Vec3::y(), n as f64, n as f64, n, n);
        for p in &mut m.positions {
            p.z = lift(p.x, p.y);
        }
        let h = hole as f64 / 2.0;
        let kept: Vec<u32> = (0..m.triangle_count() as u32)
            .filter(|&t| {
                let c = m.triangle(t as usize).unwrap();
                let (x, y) = ((c[0].x + c[1].x + c[2].x) / 3.0, (c[0].y + c[1].y + c[2].y) / 3.0);
                x.abs() > h || y.abs() > h
            })
            .collect();
        m.submesh(&kept)
    }

    /// A closed box from `lo` to `hi`, outward.
    fn closed_box(lo: Point3, hi: Point3) -> Mesh {
        let v: Vec<Point3> = (0..8)
            .map(|i| {
                Point3::new(
                    if i & 1 == 0 { lo.x } else { hi.x },
                    if i & 2 == 0 { lo.y } else { hi.y },
                    if i & 4 == 0 { lo.z } else { hi.z },
                )
            })
            .collect();
        let quads = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [1, 3, 7, 5], [3, 2, 6, 7], [2, 0, 4, 6]];
        let triangles = quads.iter().flat_map(|q| [[q[0], q[1], q[2]], [q[0], q[2], q[3]]]).collect();
        Mesh { positions: v, triangles }
    }

    fn crossing_triangles(m: &Mesh) -> usize {
        let bvh = Bvh::build(m);
        nesting::sampled_self_intersections(m, &bvh, m.triangle_count()).crossing
    }

    /// The patch of the shortest boundary loop, with or without the crossing check.
    fn fill_shortest(mesh: &Mesh, smooth: bool, check: bool) -> (Patch, Option<Fairing>) {
        let lp = boundary_loops(mesh).into_iter().min_by_key(Vec::len).unwrap();
        let loops = std::slice::from_ref(&lp);
        let (faces, uses) = (VertexFaces::new(mesh), EdgeUses::new(mesh, loops));
        let crossings = check.then(|| Crossings::new(mesh, loops));
        LAST_FAIRING.with(|c| c.set(None));
        let patch = fill_in_workspace(mesh, &faces, &uses, &lp, smooth, smooth, crossings.as_ref());
        (patch, LAST_FAIRING.with(std::cell::Cell::get))
    }

    #[test]
    fn the_smooth_cap_of_a_convex_dish_does_not_pass_through_the_dish() {
        // Arrange
        let mut m = convex_dish();
        let before = m.positions.clone();
        let rim_edges = 4 * CELLS;

        // Act
        LAST_FAIRING.with(|c| c.set(None));
        let rep = fill_holes(&mut m, 2 * rim_edges, true);

        // Assert
        assert_eq!((rep.filled, rep.skipped), (1, 0), "{rep:?}");
        assert_eq!(LAST_FAIRING.with(std::cell::Cell::get), Some(Fairing::Membrane));
        assert_eq!(crossing_triangles(&m), 0);
        assert_eq!(m.positions.get(..before.len()), Some(&before[..]), "the scan and its rim stay where they were");
    }

    #[test]
    fn a_patch_that_crosses_as_a_membrane_too_keeps_its_thin_plate_shape() {
        // Arrange: a flat sheet with a hole, and a closed box standing through the hole
        let mut m = sheet_with_hole(12, 4, |_, _| 0.0);
        m.append(&closed_box(Point3::new(-0.1, -0.1, -1.0), Point3::new(0.1, 0.1, 1.0)));

        // Act
        let (checked, fairing) = fill_shortest(&m, true, true);
        let (unchecked, _) = fill_shortest(&m, true, false);

        // Assert
        assert_eq!(fairing, Some(Fairing::ThinPlate));
        assert_eq!((checked.positions, checked.triangles), (unchecked.positions, unchecked.triangles));
    }

    #[test]
    fn a_smooth_patch_that_crosses_nothing_and_a_flat_fill_are_as_without_the_check() {
        // Arrange: a gently curved sheet with a hole
        let m = sheet_with_hole(12, 4, |x, y| 0.02 * (x * x + y * y));

        for smooth in [true, false] {
            // Act
            let (checked, fairing) = fill_shortest(&m, smooth, true);
            let (unchecked, _) = fill_shortest(&m, smooth, false);

            // Assert
            assert_eq!(fairing, smooth.then_some(Fairing::ThinPlate), "smooth {smooth}");
            assert_eq!(
                (checked.positions, checked.triangles),
                (unchecked.positions, unchecked.triangles),
                "smooth {smooth}"
            );
        }
    }

    #[test]
    fn the_tree_holds_only_the_triangles_near_a_small_hole_of_a_large_mesh() {
        // Arrange: 300 x 300 unit cells (180 000 triangles), the middle 2 x 2 cells taken out
        let m = sheet_with_hole(300, 2, |_, _| 0.0);
        let hole = vec![boundary_loops(&m).into_iter().min_by_key(Vec::len).unwrap()];
        assert_eq!(hole[0].len(), 8);

        // Act
        let crossings = Crossings::new(&m, &hole);

        // Assert: the hole's box grown by its diagonal covers 8 x 8 cells (120 triangles)
        assert!(crossings.len() > 0);
        assert!(crossings.len() <= 400, "{} of {} triangles", crossings.len(), m.triangle_count());
    }

    #[test]
    fn triangles_sharing_a_corner_or_an_edge_cross_only_where_they_pass_through_each_other() {
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        let flat = [p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0), p(0.0, 2.0, 0.0)];
        let keys = [Some(0), Some(1), Some(2)];
        // sharing corner 0: a triangle whose far edge passes down through `flat`, one beside it
        let through = [p(0.0, 0.0, 0.0), p(0.5, 0.5, 1.0), p(0.5, 0.5, -1.0)];
        let beside = [p(0.0, 0.0, 0.0), p(-1.0, 0.0, 0.5), p(0.0, -1.0, 0.5)];
        let corner = [Some(0), None, None];
        assert!(pair_crosses((&through, corner), (&flat, keys)));
        assert!(pair_crosses((&flat, keys), (&through, corner)));
        assert!(!pair_crosses((&beside, corner), (&flat, keys)));
        // sharing edge 0-1: a knife edge of a few degrees meets `flat` only along the edge
        let knife = [p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0), p(0.5, 1.0, 0.05)];
        let edge = [Some(0), Some(1), None];
        assert!(!pair_crosses((&knife, edge), (&flat, keys)));
    }

    /// The crossing case (a flat sheet with a hole, a closed box standing through it) with a fan
    /// patch over the hole whose one interior vertex is at `apex`: the tree, the workspace and where
    /// the patch starts (`(first_tri, first_new)`).
    fn fan_over_the_crossing_hole(apex: Point3) -> (Crossings, Vec<u32>, Mesh, (usize, usize)) {
        let mut m = sheet_with_hole(12, 4, |_, _| 0.0);
        m.append(&closed_box(Point3::new(-0.1, -0.1, -1.0), Point3::new(0.1, 0.1, 1.0)));
        let lp = boundary_loops(&m).into_iter().min_by_key(Vec::len).unwrap();
        let crossings = Crossings::new(&m, std::slice::from_ref(&lp));
        let (origin, mut work) = crate::holes::workspace(&m, &VertexFaces::new(&m), &lp);
        let first_tri = work.triangles.len();
        let base = work.positions.len() as u32;
        let a = lp.iter().map(|&v| origin.binary_search(&v).unwrap() as u32).collect::<Vec<_>>();
        work.positions.push(apex);
        for k in 0..a.len() {
            work.triangles.push([a[k], a[(k + 1) % a.len()], base]);
        }
        (crossings, origin, work, (first_tri, base as usize))
    }

    #[test]
    fn a_thin_plate_patch_that_is_not_finite_gives_way_to_a_finite_membrane_even_one_that_crosses() {
        // Arrange: the thin plate not finite; the membrane at the middle of the hole crosses the box
        let (crossings, origin, mut work, at) = fan_over_the_crossing_hole(Point3::new(f64::NAN, 0.0, 0.0));
        let membrane = Point3::new(0.0, 0.0, 0.0);

        // Act
        assert!(crossings.patch_crosses(&origin, &work, at.0), "a NaN corner counts as crossing");
        crossings.settle(&origin, &mut work, at, vec![membrane]);

        // Assert
        assert_eq!(work.positions.last(), Some(&membrane));
        assert_eq!(LAST_FAIRING.with(std::cell::Cell::get), Some(Fairing::Membrane));
    }

    #[test]
    fn a_membrane_that_is_not_finite_never_replaces_the_thin_plate() {
        // Arrange: a finite thin plate that crosses the box
        let thin = Point3::new(0.0, 0.0, 0.0);
        let (crossings, origin, mut work, at) = fan_over_the_crossing_hole(thin);
        assert!(crossings.patch_crosses(&origin, &work, at.0));

        // Act
        crossings.settle(&origin, &mut work, at, vec![Point3::new(f64::NAN, 0.0, 0.0)]);

        // Assert
        assert_eq!(work.positions.last(), Some(&thin));
        assert_eq!(LAST_FAIRING.with(std::cell::Cell::get), Some(Fairing::ThinPlate));
    }

    #[test]
    fn when_neither_patch_is_finite_the_thin_plate_stays_and_says_so() {
        // Arrange: both not finite, told apart by their finite coordinates
        let (crossings, origin, mut work, at) = fan_over_the_crossing_hole(Point3::new(f64::NAN, 1.0, 0.0));

        // Act
        crossings.settle(&origin, &mut work, at, vec![Point3::new(f64::NAN, 2.0, 0.0)]);

        // Assert: the patch is left as the fairing gave it (the caller's checks see a NaN patch)
        assert_eq!(work.positions.last().map(|p| p.y), Some(1.0));
        assert_eq!(LAST_FAIRING.with(std::cell::Cell::get), Some(Fairing::ThinPlate));
    }
}
