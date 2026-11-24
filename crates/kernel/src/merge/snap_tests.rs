//! Tests of `snap.rs`.
use super::*;

fn p(x: f64, y: f64, z: f64) -> V {
    [x, y, z]
}

#[test]
fn a_fit_through_points_on_a_cubic_gives_that_cubic() {
    let cubic = [p(0.0, 0.0, 0.0), p(1.0, 2.0, 0.5), p(3.0, 2.0, -0.5), p(4.0, 0.0, 0.0)];
    let samples: Vec<V> = (0..9)
        .map(|i| {
            let t = f64::from(i) / 8.0;
            let w = [(1.0 - t).powi(3), 3.0 * t * (1.0 - t).powi(2), 3.0 * t * t * (1.0 - t), t.powi(3)];
            (0..3).map(|d| cubic.iter().zip(&w).map(|(q, b)| q[d] * b).sum::<f64>()).collect::<Vec<_>>()
        })
        .map(|v| [v[0], v[1], v[2]])
        .collect();
    let fit = least_squares(3, &samples).unwrap();
    for (a, b) in fit.iter().zip(&cubic) {
        assert!(norm(sub(*a, *b)) < 1e-9, "{a:?} {b:?}");
    }
}

#[test]
fn a_line_has_no_interior_unknowns_and_the_fallback_keeps_its_ends() {
    assert!(least_squares(1, &[p(0.0, 0.0, 0.0), p(0.5, 0.0, 0.0), p(1.0, 0.0, 0.0)]).is_none());
    let moved = follow(&[p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(2.0, 0.0, 0.0)], [p(0.0, 1.0, 0.0), p(2.0, 3.0, 0.0)]);
    assert_eq!(moved.first(), Some(&p(0.0, 1.0, 0.0)));
    assert_eq!(moved.last(), Some(&p(2.0, 3.0, 0.0)));
    assert!((moved[1][1] - 2.0).abs() < 1e-12, "the middle moves by the mean of the end displacements");
}

#[test]
fn a_border_is_matched_to_its_edge_either_way_round_and_a_stranger_is_not() {
    let net = vec![vec![p(0.0, 0.0, 0.0), p(0.0, 1.0, 0.0)], vec![p(1.0, 0.0, 0.0), p(1.0, 1.0, 0.0)]];
    let old = vec![p(0.0, 0.0, 0.0), p(0.0, 1.0, 0.0)];
    let new = vec![p(0.0, 0.0, 0.1), p(0.0, 1.0, 0.1)];
    fn edit<'a>(old: &'a [V], new: &'a [V]) -> Edit<'a> {
        Edit { old, new, side: None }
    }
    let moved = moved_net(&net, &[edit(&old, &new)]).unwrap();
    assert_eq!((moved[0][0][2], moved[0][1][2], moved[1][0][2]), (0.1, 0.1, 0.0));
    let backwards = vec![p(0.0, 1.0, 0.0), p(0.0, 0.0, 0.0)];
    let reversed: Vec<V> = new.iter().rev().copied().collect();
    assert!(moved_net(&net, &[edit(&backwards, &reversed)]).is_some());
    assert!(moved_net(&net, &[edit(&[p(5.0, 5.0, 5.0), p(6.0, 5.0, 5.0)], &new)]).is_none());
}

fn plane_gauge(normal: V, d: f64) -> Gauge {
    Gauge::new(&crate::hybrid::Shape::Plane { point: scale(normal, d), normal }).unwrap()
}

#[test]
fn a_point_goes_onto_its_primitives_only_within_the_allowed_move() {
    let (a, b) = (plane_gauge([0.0, 0.0, 1.0], 0.0), plane_gauge([1.0, 0.0, 0.0], 0.0));
    let q = put_on_all(&[&a, &b], p(0.05, 3.0, 0.07), 0.2).unwrap();
    assert!(q[0].abs() < 1e-5 && q[2].abs() < 1e-5, "{q:?}");
    assert!(put_on_all(&[&a, &b], p(0.05, 3.0, 0.07), 0.05).is_none(), "0.086 away is farther than 0.05");
    assert!(put_on_all(&[&a], p(0.0, 0.0, 0.3), 0.2).is_none());
    assert!(put_on_all(&[&a], p(0.0, 0.0, 0.1), f64::NAN).is_none());
    assert!(put_on_all(&[], p(0.0, 0.0, 0.1), 1.0).is_none());
}

/// Two planes 10 degrees apart: a point between them, near both, is far from their line when it is close to each; the
/// move onto the line is refused beyond the limit. At 10 degrees the alternation (a factor of 0.97 a round) does not
/// reach a nanometre in 200 rounds, so `put_on_all` reports it, and `placed` gives the point nearest both, flagged as
/// not on both; at 30 degrees it converges and is on both.
#[test]
fn a_point_near_two_planes_at_a_shallow_angle_that_meet_far_away_is_refused() {
    let plane_at = |deg: f64| {
        let t = deg.to_radians();
        (plane_gauge([0.0, 0.0, 1.0], 0.0), plane_gauge([-t.sin(), 0.0, t.cos()], 0.0))
    };
    let (a, b) = plane_at(30.0);
    assert!(put_on_all(&[&a, &b], p(0.1, 0.0, 0.05), 0.3).is_some());
    assert!(put_on_all(&[&a, &b], p(5.7, 0.0, 0.1), 0.3).is_none(), "far from the line");
    let (a, b) = plane_at(10.0);
    assert!(put_on_all(&[&a, &b], p(0.5, 0.0, 0.1), 10.0).is_none(), "not converged in 200 rounds");
    let (q, on_all) = placed(&|i, q| [&a, &b][i].closest(q), 2, p(0.5, 0.0, 0.1), 10.0).unwrap();
    assert!(!on_all);
    let worst = [&a, &b].iter().filter_map(|g| g.distance(q)).fold(0.0_f64, f64::max);
    assert!(worst <= 0.1 + 1e-9, "nearer both than the point was: {worst}");
}

fn two_patches() -> Brep {
    let net = |x: f64| vec![vec![p(x, 0.0, 0.0), p(x, 1.0, 0.0)], vec![p(x + 1.0, 0.0, 0.0), p(x + 1.0, 1.0, 0.0)]];
    crate::nurbs::sheet_from_bezier_network(&[net(0.0), net(1.0)], &[[0, 1, 2, 3], [1, 4, 5, 2]]).unwrap()
}

#[test]
fn a_body_whose_edges_lost_a_use_does_not_have_its_topology_kept() {
    let before = two_patches();
    assert!(topology_kept(&before, &before));
    let mut after = before.clone();
    after.edges[0].uses.clear();
    assert!(!topology_kept(&before, &after));
}

#[test]
fn a_patch_that_moved_too_far_or_turned_over_is_not_sound() {
    let before = two_patches();
    let changed = |after: &Brep| {
        let edges = vec![0u32];
        moved_patch_ok(&before, after, &Changed { face: 1, edges, asked: Groups::new() }, 0.1)
    };
    let mut far = before.clone();
    if let Some(Surface::Spline { points, .. }) = far.faces.get_mut(1).map(|f| &mut f.surface) {
        points[1][1][2] = 0.5;
    }
    assert!(!changed(&far), "a corner 0.5 mm away");
    let mut turned = before.clone();
    if let Some(Surface::Spline { points, .. }) = turned.faces.get_mut(1).map(|f| &mut f.surface) {
        let (a, b) = (points[0][1], points[1][1]);
        points[0][1] = b;
        points[1][1] = a;
    }
    assert!(!changed(&turned), "folded over");
    let mut nan = before.clone();
    if let Some(Surface::Spline { points, .. }) = nan.faces.get_mut(1).map(|f| &mut f.surface) {
        points[1][1][2] = f64::NAN;
    }
    assert!(!changed(&nan), "a NaN displacement is no displacement a maximum would see");
}

/// A border collapsed to a point runs along a collapsed edge either way round: the side the loop says it is
/// decides which end of the new edge is which.
#[test]
fn a_collapsed_border_is_moved_by_the_side_of_the_loop_it_is() {
    let at = p(1.0, 1.0, 0.0);
    let net = vec![vec![at, at, at], vec![p(2.0, 0.0, 0.0), p(2.0, 1.0, 0.0), p(2.0, 2.0, 0.0)]];
    let old = vec![at, at, at];
    let new = vec![p(1.0, 1.0, 0.5), p(1.0, 1.0, 0.25), p(1.0, 1.0, 0.0)];
    // the border net[0][..] is the fourth side backwards: a loop that walks the edge forward on it runs the
    // border against the edge
    let moved = |forward: bool| {
        let edit = Edit { old: &old, new: &new, side: side_of(3, forward) };
        moved_net(&net, &[edit]).unwrap()[0].iter().map(|q| q[2]).collect::<Vec<f64>>()
    };
    assert_eq!(moved(true), vec![0.0, 0.25, 0.5]);
    assert_eq!(moved(false), vec![0.5, 0.25, 0.0]);
}

#[test]
fn parallel_primitives_are_reported_as_not_met_and_not_settled() {
    let (a, b) = (plane_gauge([0.0, 0.0, 1.0], 0.0), plane_gauge([0.0, 0.0, 1.0], 0.5));
    assert!(put_on_all(&[&a, &b], p(1.0, 1.0, 0.2), 10.0).is_none(), "no point is on both");
    assert!(put_on_all(&[&a, &a], p(1.0, 1.0, 0.2), 10.0).is_some(), "the same plane twice is one plane");
    let at = p(1.0, 1.0, 0.2);
    assert!(
        placed(&|i, q| [&a, &b][i].closest(q), 2, at, 10.0).is_none(),
        "parallel planes place nothing, not even on one"
    );
}

/// A collapsed border is moved only where the loop says it is: a loop that starts elsewhere than the net was laid
/// out (healed, renumbered) leads to a border that is not collapsed, and the face is refused, not guessed.
#[test]
fn a_collapsed_border_that_the_loop_does_not_lead_to_is_refused() {
    let at = p(1.0, 1.0, 0.0);
    let net = vec![vec![at, at, at], vec![p(2.0, 0.0, 0.0), p(2.0, 1.0, 0.0), p(2.0, 2.0, 0.0)]];
    let (old, new) = (vec![at, at, at], vec![p(1.0, 1.0, 0.5), p(1.0, 1.0, 0.25), p(1.0, 1.0, 0.0)]);
    let edit = |side| Edit { old: &old, new: &new, side };
    assert!(moved_net(&net, &[edit(side_of(3, true))]).is_some());
    assert!(moved_net(&net, &[edit(side_of(0, true))]).is_none(), "side 0 is net[..][0], not collapsed");
    assert!(moved_net(&net, &[edit(None)]).is_none(), "no side, no guess");
}

#[test]
fn an_edge_with_more_control_points_than_samples_is_refused_before_the_fit() {
    let body = two_patches();
    let g = plane_gauge([0.0, 0.0, 1.0], 0.0);
    let many: Vec<V> = (0..40).map(|i| p(f64::from(i), 0.0, 0.0)).collect();
    assert!(refit_edge(&body, 0, &[&g], [many[0], many[39]], &many, 1.0).is_err());
    assert!(least_squares(30, &many[..10]).is_none());
}

#[test]
fn a_cancelled_snap_snaps_nothing() {
    use super::super::geometry::group_of;
    let body = two_patches();
    let group = group_of(&body, &[0]).unwrap();
    let gauge = plane_gauge([0.0, 0.0, 1.0], 0.0);
    let target = [Target { faces: &[0], group: &group, gauge: &gauge }];
    let done = snap_boundaries(&body, &target, 0.3, &|| false);
    assert!(done.snapped[0]);
    let cancelled = snap_boundaries(&body, &target, 0.3, &|| true);
    assert!(!cancelled.snapped[0]);
    assert_eq!(cancelled.body, body);
}

/// One bilinear patch with the corners `[p00, p10, p11, p01]` (rows of its net `u`, columns `v`).
fn one_patch(c: [V; 4]) -> Brep {
    let [p00, p10, p11, p01] = c;
    crate::nurbs::sheet_from_bezier_network(&[vec![vec![p00, p01], vec![p10, p11]]], &[[0, 1, 2, 3]]).unwrap()
}

/// A parallelogram 20 mm long and 1 mm wide whose sides meet at a sine of 0.0995: its normal can be read nowhere
/// (the bound is 0.4), as on the long thin patches along the border of an open scan.
fn skewed() -> [V; 4] {
    [p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0), p(20.0, 1.0, 0.0), p(10.0, 1.0, 0.0)]
}

fn patch_ok(before: &Brep, after: &Brep, max_move: f64) -> bool {
    moved_patch_ok(before, after, &Changed { face: 0, edges: Vec::new(), asked: Groups::new() }, max_move)
}

/// The neighbours that refused snaps on real scans: no readable normal before the snap, a move of 0.30 to 1.13 of
/// their width, and the patch as a whole not turned (measured on scans 4, 6 and 10). A move of 0.3 of the width that
/// tilts the patch without folding it does not make it worse, so it is taken.
#[test]
fn a_neighbour_whose_normal_cannot_be_read_takes_a_move_that_does_not_make_it_worse() {
    let before = one_patch(skewed());
    assert_eq!(normals_at_grid(&before, 0, MOVED_GRID).unwrap().iter().flatten().count(), 0, "unreadable");
    let mut c = skewed();
    c[2][2] = 0.3;
    let after = one_patch(c);
    assert!(patch_ok(&before, &after, 0.5), "a corner lifted by 0.3 of the width");
}

/// The same patch is still refused where the move turns it over as a whole or folds it, whatever its normals.
#[test]
fn a_neighbour_whose_normal_cannot_be_read_is_refused_when_the_move_turns_or_folds_it() {
    let before = one_patch(skewed());
    let mirrored = one_patch([p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0), p(20.0, -1.0, 0.0), p(10.0, -1.0, 0.0)]);
    assert!(!patch_ok(&before, &mirrored, 100.0), "turned over as a whole");
    let bow_tie = one_patch([p(0.0, 0.0, 0.0), p(10.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(10.0, 1.0, 0.0)]);
    assert!(!patch_ok(&before, &bow_tie, 100.0), "folded across itself");
}

/// A neighbour that is sound before the snap (its normal read at every point, all one way) keeps the absolute test:
/// a move that turns its normal at any point of the grid is refused.
#[test]
fn a_sound_neighbour_keeps_the_absolute_test() {
    let flat = [p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(1.0, 1.0, 0.0), p(0.0, 1.0, 0.0)];
    let before = one_patch(flat);
    let n0 = normals_at_grid(&before, 0, MOVED_GRID).unwrap();
    assert!(sound_before(&n0), "read everywhere, all one way");
    // one corner pulled across the patch, past the opposite diagonal: its corner of the patch turns over
    let mut c = flat;
    c[2] = p(0.2, 0.2, 0.0);
    let after = one_patch(c);
    assert!(!patch_ok(&before, &after, 100.0), "turned over at a corner");
}

/// A group whose border meets a long thin patch with no readable normal (the border of an open scan) is snapped: the
/// patch is tilted by the move (0.2 mm, two thirds of its 0.3 mm width) but not folded or turned over.
#[test]
fn a_group_next_to_a_neighbour_whose_normal_cannot_be_read_is_snapped() {
    use super::super::geometry::group_of;
    let group = vec![vec![p(0.0, 0.0, 0.0), p(0.0, 1.0, 0.0)], vec![p(1.0, 0.0, 0.2), p(1.0, 1.0, 0.2)]];
    let thin = vec![vec![p(1.0, 0.0, 0.2), p(1.0, 1.0, 0.2)], vec![p(1.3, 3.0, 0.2), p(1.3, 4.0, 0.2)]];
    let body = crate::nurbs::sheet_from_bezier_network(&[group, thin], &[[0, 1, 2, 3], [1, 4, 5, 2]]).unwrap();
    assert_eq!(normals_at_grid(&body, 1, MOVED_GRID).unwrap().iter().flatten().count(), 0, "unreadable");
    let g = group_of(&body, &[0]).unwrap();
    let gauge = plane_gauge([0.0, 0.0, 1.0], 0.0);
    let done = snap_boundaries(&body, &[Target { faces: &[0], group: &g, gauge: &gauge }], 0.5, &|| false);
    assert!(done.snapped[0], "{:?}", done.why_not);
    let shared = done.body.vertices.get(1).unwrap().point;
    assert!(shared[2].abs() < 1e-6, "the shared border is on the plane: {shared:?}");
}
