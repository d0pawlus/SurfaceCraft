//! Peeling the border of a group that cannot be merged whole (P9-O6e): a group whose border snap would fold a
//! neighbouring patch and whose border is too far from the primitive to be merged as it is loses only the border
//! patches concerned, and the rest becomes one analytic face. The fixtures are synthetic; every count asserted is
//! the topology of the fixture, every distance a multiple of the fixture's own figures.
mod common;

use common::network_from_fns;
use surfacecraft_kernel::hybrid::Shape;
use surfacecraft_kernel::merge::{Batch, GroupSpec, merge_faces_onto_shapes};
use surfacecraft_kernel::nurbs::sheet_from_bezier_network;
use surfacecraft_kernel::record::brep::{Brep, Surface};

type P = [f64; 3];

/// The gap the merge is allowed in every test (mm).
const GAP: f64 = 0.1;

/// A sheet of bilinear patches over the quads `quads` of the points `points` (each quad is four point indices,
/// counter-clockwise seen from above, the first two along the patch's first parameter).
fn sheet(points: &[P], quads: &[[usize; 4]]) -> Brep {
    let nets: Vec<Vec<Vec<P>>> =
        quads.iter().map(|q| vec![vec![points[q[0]], points[q[3]]], vec![points[q[1]], points[q[2]]]]).collect();
    let corners: Vec<[u32; 4]> = quads.iter().map(|q| q.map(|i| i as u32)).collect();
    sheet_from_bezier_network(&nets, &corners).unwrap()
}

/// A row of bilinear patches along x (y from 0 to 10), column `i` at `xs[i]` and height `zs[i]`; patch `k` lies
/// between columns `k` and `k + 1`, its normal up.
fn row(xs: &[f64], zs: &[f64]) -> Brep {
    let points: Vec<P> = (0..xs.len()).flat_map(|i| [[xs[i], 0.0, zs[i]], [xs[i], 10.0, zs[i]]]).collect();
    let quads: Vec<[usize; 4]> = (0..xs.len() - 1).map(|k| [2 * k, 2 * k + 2, 2 * k + 3, 2 * k + 1]).collect();
    sheet(&points, &quads)
}

fn plane() -> Shape {
    Shape::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] }
}

fn merge(body: &Brep, faces: &[usize], shape: &Shape) -> Batch {
    merge_faces_onto_shapes(body, &[GroupSpec { faces, shape }], GAP, &|| false).unwrap()
}

fn net(b: &Brep, face: usize) -> Vec<Vec<P>> {
    match &b.faces[face].surface {
        Surface::Spline { points, .. } => points.clone(),
        other => panic!("face {face} is not a patch: {other:?}"),
    }
}

/// The patches `faces` of `before` are patches of `after`, exactly where they were.
fn unchanged(before: &Brep, after: &Batch, faces: &[usize]) {
    for &f in faces {
        let k = after.face_map[f].unwrap_or_else(|| panic!("patch {f} stays"));
        let off = net(before, f)
            .iter()
            .flatten()
            .zip(net(&after.brep, k).iter().flatten())
            .map(|(p, q)| (0..3).map(|d| (p[d] - q[d]).abs()).fold(0.0, f64::max))
            .fold(0.0, f64::max);
        assert!(off < 1e-9, "patch {f} moved by {off}");
    }
}

/// The sliver at the end of the rows below: from a column 0.11 above the plane (1.1 gaps, within the slack of 1.25
/// gaps that admits a border to the snap, beyond the gap a border that is not snapped may have) to one 0.03 mm
/// further and 0.05 up. Snapping the high column onto the plane turns the sliver over: its normal goes from
/// (0.06, 0.03) to (-0.05, 0.03) in (x, z), a dot product of -0.0021.
const HIGH: f64 = 0.11;
const SLIVER: (f64, f64) = (0.03, 0.05);

/// Four patches of a plane whose right border is 1.1 gaps above it, and a sliver next to that border that the
/// snap would turn over. Merged whole the border cannot be snapped (the sliver) and is too far to stay as it is,
/// so the whole group was refused; peeled, the patch along the far border (which has a corner on the sliver)
/// stays a patch and the other three become one plane on the plane. The patches that stay are where they were.
#[test]
fn a_plane_group_whose_snap_folds_a_neighbour_loses_only_its_border_patch() {
    let body = row(&[0.0, 4.0, 8.0, 12.0, 16.0, 16.0 + SLIVER.0], &[0.0, 0.0, 0.0, 0.0, HIGH, SLIVER.1]);
    let batch = merge(&body, &[0, 1, 2, 3], &plane());
    let g = batch.groups[0].as_ref().unwrap_or_else(|e| panic!("the peeled group is merged: {e}"));
    assert_eq!(g.peeled, 1, "the patch at the sliver's corners");
    assert!(g.gap < 0.1 * GAP, "the border of the face is on the plane: {}", g.gap);
    assert!(g.snap_note.is_none(), "the peeled group is snapped: {:?}", g.snap_note);
    assert_eq!(batch.brep.faces.len(), 3, "one plane and the two patches that stay");
    assert!(matches!(batch.brep.faces[g.face].surface, Surface::Plane { .. } | Surface::PlaneThrough { .. }));
    unchanged(&body, &batch, &[3, 4]);
}

/// The same on a cylinder of radius 10: four quadratic patches of 0.4 rad each, the last column 1.1 gaps outside
/// the cylinder, and a sliver 0.003 rad (0.03 mm) wide that comes back to 0.05 mm outside it.
#[test]
fn a_cylinder_group_whose_snap_folds_a_neighbour_loses_only_its_border_patch() {
    let r = 10.0;
    let angles = [0.5, 0.9, 1.3, 1.7, 2.1, 2.1 + SLIVER.0 / r];
    let radii = [r, r, r, r, r + HIGH, r + SLIVER.1];
    let patches: Vec<Box<dyn Fn(f64, f64) -> P>> = (0..5)
        .map(|k| {
            let (a0, a1, r0, r1) = (angles[k], angles[k + 1], radii[k], radii[k + 1]);
            Box::new(move |u: f64, v: f64| {
                let (a, rho) = (a0 + (a1 - a0) * u, r0 + (r1 - r0) * u);
                [rho * a.cos(), rho * a.sin(), 10.0 * v]
            }) as Box<dyn Fn(f64, f64) -> P>
        })
        .collect();
    let refs: Vec<&dyn Fn(f64, f64) -> P> = patches.iter().map(|b| b.as_ref()).collect();
    let net = network_from_fns(&refs, 2);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    let shape = Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: r };
    let batch = merge(&body, &[0, 1, 2, 3], &shape);
    let g = batch.groups[0].as_ref().unwrap_or_else(|e| panic!("the peeled group is merged: {e}"));
    assert_eq!(g.peeled, 1);
    assert!(g.gap < 0.1 * GAP, "{}", g.gap);
    assert!(matches!(batch.brep.faces[g.face].surface, Surface::Cylinder { .. }));
    assert_eq!(batch.brep.faces.len(), 3);
    unchanged(&body, &batch, &[3, 4]);
}

/// A refused group leaves the body as it was and says why.
fn refused_as_it_was(body: &Brep, batch: &Batch, why: &str) {
    let e = batch.groups[0].as_ref().expect_err("refused");
    assert!(e.contains(why), "{e}");
    assert_eq!(&batch.brep, body, "nothing is lost");
}

/// Seven patches in a row and a sliver on the far side of the middle one, whose border there is 1.1 gaps above the
/// plane: peeling the patches at the sliver's corners and along the far border would cut the group in two, so it is
/// not peeled and is refused, the body as it was.
#[test]
fn a_group_that_peeling_would_cut_in_two_is_refused_and_left_as_it_was() {
    let top = |i: usize| if i == 3 || i == 4 { HIGH } else { 0.0 };
    let mut points: Vec<P> =
        (0..8).flat_map(|i| [[4.0 * i as f64, 0.0, 0.0], [4.0 * i as f64, 10.0, top(i)]]).collect();
    let mut quads: Vec<[usize; 4]> = (0..7).map(|k| [2 * k, 2 * k + 2, 2 * k + 3, 2 * k + 1]).collect();
    // the sliver over the top edge of patch 3, from (12, 10) to (16, 10), 0.03 mm wide in y
    points.push([16.0, 10.0 + SLIVER.0, SLIVER.1]);
    points.push([12.0, 10.0 + SLIVER.0, SLIVER.1]);
    quads.push([7, 9, 16, 17]);
    let body = sheet(&points, &quads);
    let batch = merge(&body, &(0..7).collect::<Vec<_>>(), &plane());
    refused_as_it_was(&body, &batch, "2 pieces");
}

/// Two patches whose peel would leave one: refused, the body as it was.
#[test]
fn a_group_that_peeling_would_leave_a_single_patch_is_refused() {
    let body = row(&[0.0, 4.0, 8.0, 8.0 + SLIVER.0], &[0.0, 0.0, HIGH, SLIVER.1]);
    let batch = merge(&body, &[0, 1], &plane());
    refused_as_it_was(&body, &batch, "1 patch(es)");
}

/// A group every patch of which is on the far border and at the sliver: nothing would be left, so it is refused
/// and the body is as it was (the peeling ends; nothing is merged and nothing moves).
#[test]
fn a_group_that_fails_everywhere_is_refused_and_nothing_is_lost() {
    let body = row(&[0.0, 4.0, 8.0, 12.0, 12.0 + SLIVER.0], &[HIGH, HIGH, HIGH, HIGH, SLIVER.1]);
    let batch = merge(&body, &[0, 1, 2], &plane());
    refused_as_it_was(&body, &batch, "0 pieces");
}

/// A group whose snap folds the sliver but whose border is within the gap is merged as it was (as before the peel:
/// nothing is peeled off a group that can be merged whole), with the reason it was not snapped.
#[test]
fn a_group_within_the_gap_is_merged_whole_and_not_peeled() {
    let body = row(&[0.0, 4.0, 8.0, 8.0 + SLIVER.0], &[0.0, 0.0, 0.08, 0.02]);
    let batch = merge(&body, &[0, 1], &plane());
    let g = batch.groups[0].as_ref().unwrap_or_else(|e| panic!("merged whole: {e}"));
    assert_eq!(g.peeled, 0);
    assert!(g.snap_note.as_deref().is_some_and(|n| n.contains("fold")), "{:?}", g.snap_note);
}
