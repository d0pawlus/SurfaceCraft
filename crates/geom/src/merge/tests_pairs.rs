//! Tests of the pair verdicts: scans displaced along their normals (refused when nothing ties them,
//! reported as `LayersApart` when they meet somewhere), the ties between scans, small overlaps and
//! unjudged samples.
use super::tests_round2::{codes, flipped, moved, report, sheet, warns};
use super::*;
use crate::{Point3, Transform, Vec3, synth};

#[test]
fn flat_sheets_displaced_along_their_normal_are_refused_and_sheets_apart_are_a_union() {
    let tol = 0.1;
    let a = sheet(40, 40, (0.0, 0.0), 1.0);
    let b = sheet(32, 32, (10.0, 5.0), 1.25);
    for times in [10.0, 20.0, 100.0] {
        let rep = report(&[a.clone(), moved(&b, Vec3::new(0.0, 0.0, times * tol))], tol);
        assert!(rep.refused, "{times} tolerances: {:?}", rep.issues);
        assert!(codes(&rep).contains(&"Displaced".to_string()), "{:?}", rep.issues);
    }
    // beside each other, sharing nothing: a union of two pieces with the notice
    let rep = report(&[a.clone(), moved(&b, Vec3::new(60.0, 0.0, 0.0))], tol);
    assert!(!rep.refused && codes(&rep).contains(&"Disjoint".to_string()), "{:?}", rep.issues);
}

#[test]
fn opposite_sides_of_a_wall_and_a_hollow_part_are_not_refused() {
    let tol = 0.1;
    // the front of a wall seen by one scan, its back (2 mm behind, facing away) by the other
    let front = sheet(20, 20, (0.0, 0.0), 1.0);
    let back = flipped(&moved(&sheet(20, 20, (2.0, 2.0), 1.0), Vec3::new(0.0, 0.0, -2.0)));
    let rep = report(&[front, back], tol);
    assert!(!rep.refused && !warns(&rep), "{:?}", rep.issues);
    // a hollow sphere (outer skin and inner skin facing in) with a view of its outer skin
    let mut hollow = synth::sphere(Point3::origin(), 20.0, 40, 20);
    hollow.append(&flipped(&synth::sphere(Point3::origin(), 16.0, 32, 16)));
    let view = synth::sphere(Point3::origin(), 20.0, 44, 22);
    let cap = Mesh {
        positions: view.positions.clone(),
        triangles: (0..view.triangle_count())
            .filter(|&t| view.face_centroid(t).unwrap().x > 5.0)
            .map(|t| view.triangles[t])
            .collect(),
    };
    let rep = report(&[hollow, cap], 0.3);
    assert!(!rep.refused && !warns(&rep), "{:?}", rep.issues);
}

#[test]
fn a_shelf_over_a_floor_seen_by_different_scans_that_share_a_wall_is_not_refused() {
    // a wall in the plane x = 40 (facing -x), both scans see it; one also sees the floor (z = 0), the
    // other a shelf over most of it (z = 10, facing up)
    let wall = |h: f64| {
        let s = sheet(20, 10, (0.0, 0.0), h * 2.0);
        Mesh {
            positions: s.positions.iter().map(|p| Point3::new(40.0, p.x, p.y)).collect(),
            triangles: s.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect(),
        }
    };
    let mut floor = sheet(40, 40, (0.0, 0.0), 1.0);
    floor.append(&wall(1.0));
    let mut shelf = moved(&sheet(24, 32, (0.0, 0.0), 1.25), Vec3::new(0.0, 0.0, 10.0));
    shelf.append(&wall(0.9));
    let rep = report(&[floor, shelf], 0.1);
    assert!(!rep.refused, "{:?}", rep.issues);
}

#[test]
fn a_stray_ray_hit_does_not_join_scans_that_share_nothing() {
    let r = |a: usize, b: usize, over: usize| PairResidual { a, b, samples: 20_000, over, ..PairResidual::default() };
    let issues = pair_issues(2, &[r(0, 1, 201), r(1, 0, 0)], 0.1);
    assert!(issues.iter().any(|i| matches!(i, Issue::Disjoint { .. })), "{issues:?}");
    let displaced = pair_issues(2, &[r(0, 1, 5000), r(1, 0, 5000)], 0.1);
    assert!(
        displaced.iter().any(|i| matches!(i, Issue::Misaligned { .. } | Issue::Displaced { .. }))
            && !displaced.iter().any(|i| matches!(i, Issue::Disjoint { .. }))
    );
}

#[test]
fn a_pile_of_triangles_at_one_place_does_not_hang_the_comparison() {
    // 200 000 triangles of distinct vertices at the same three places: no duplicate by index
    let mut pile = Mesh::default();
    for k in 0..200_000u32 {
        pile.positions.extend([Point3::new(0.0, 0.0, 5.0), Point3::new(10.0, 0.0, 5.0), Point3::new(10.0, 10.0, 5.0)]);
        pile.triangles.push([3 * k, 3 * k + 1, 3 * k + 2]);
    }
    let r = merge(&[sheet(4, 4, (0.0, 0.0), 1.0), pile], &Params { tolerance: 0.1 }, &Cancel::new(), &mut |_| {});
    let (_, rep) = r.unwrap();
    // the pile lies flat 50 tolerances over the sheet, facing the same way: displaced, refused; the
    // sheet's samples under the pile's box but beside its triangles (y > x) meet 200 000 boxes and
    // no triangle: their casts run out and they are said to be unjudged, not taken as "not over"
    let displaced = rep.issues.iter().any(|i| matches!(i, Issue::Displaced { a: 1, b: 0, .. }));
    assert!(rep.refused && displaced, "{:?}", rep.issues);
    let unjudged = rep.issues.iter().find_map(|i| match i {
        Issue::Unjudged { a: 0, b: 1, samples } => Some(*samples),
        _ => None,
    });
    assert!(unjudged.is_some_and(|n| n > 0), "{:?}", rep.issues);
}

#[test]
fn two_pieces_over_each_other_tied_by_a_third_are_the_part_not_a_misalignment() {
    // 0 and 2 share nothing and lie over each other displaced (a step); 1 agrees with both
    let r = |a: usize, b: usize, near: usize, over: usize| PairResidual {
        a,
        b,
        samples: 20_000,
        near,
        within: near,
        over,
        ..PairResidual::default()
    };
    let pairs =
        [r(0, 1, 6000, 0), r(1, 0, 4700, 0), r(1, 2, 4400, 0), r(2, 1, 10_000, 0), r(0, 2, 0, 900), r(2, 0, 0, 2200)];
    let issues = pair_issues(3, &pairs, 0.1);
    assert!(issues.is_empty(), "{issues:?}");
    // the same two alone are refused
    let alone = pair_issues(2, &[r(0, 1, 0, 900), r(1, 0, 0, 2200)], 0.1);
    assert!(alone.iter().any(|i| matches!(i, Issue::Misaligned { .. } | Issue::Displaced { .. })), "{alone:?}");
}

/// An L: a floor (z = 0, facing up) of `n` cells of `h` and a wall at its end (x = n h, facing -x).
fn l_part(n: usize, h: f64) -> Mesh {
    let mut m = sheet(n, n, (0.0, 0.0), h);
    let w = sheet(n, n, (0.0, 0.0), h);
    let x = n as f64 * h;
    m.append(&Mesh {
        positions: w.positions.iter().map(|p| Point3::new(x, p.x, p.y)).collect(),
        triangles: w.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect(),
    });
    m
}

#[test]
fn an_l_part_displaced_along_its_floor_normal_never_merges_silently() {
    let tol = 0.1;
    // the second view's wall slides along the first's (within the tolerance), its floor lies 20
    // tolerances above the first's
    let a = l_part(20, 1.0);
    let b = moved(&l_part(16, 1.25), Vec3::new(0.0, 0.0, 20.0 * tol));
    let rep = report(&[a, b], tol);
    let flagged = rep.refused
        || rep
            .issues
            .iter()
            .any(|i| matches!(i, Issue::LayersApart { offset, .. } if (offset - 20.0 * tol).abs() < tol));
    assert!(flagged, "{:?}", rep.issues);
}

/// Is the merge of `scans` refused or does it say the scans lie over each other?
fn flagged(scans: &[Mesh], tol: f64) -> (bool, Vec<Issue>) {
    let rep = report(scans, tol);
    let hit = rep.refused || rep.issues.iter().any(|i| matches!(i, Issue::LayersApart { .. }));
    (hit, rep.issues)
}

#[test]
fn a_small_displaced_patch_is_never_merged_silently() {
    let tol = 0.1;
    // a 5 mm L patch on a 20 mm L, its wall on the other's wall, its floor 2 mm (20 tol) above
    let b = moved(&l_part(4, 1.25), Vec3::new(15.0, 0.0, 20.0 * tol));
    // measured over share 0.50 each way, agreement 0.50 (the walls): tied, so warned, not refused
    let rep = report(&[l_part(20, 1.0), b], tol);
    assert!(!rep.refused && warns(&rep), "{:?}", rep.issues);
}

#[test]
fn a_patch_displaced_by_a_small_rotation_is_never_merged_silently() {
    let tol = 0.1;
    // turned by 6 degrees about the foot of the wall: the floor rises to 2 mm at its far end
    let turn = Transform::translation(Vec3::new(20.0, 0.0, 0.0))
        .then_after(&Transform::rotation_axis_angle(&Vec3::y(), -6.0_f64.to_radians()))
        .then_after(&Transform::translation(Vec3::new(-20.0, 0.0, 0.0)));
    let b = l_part(16, 1.25).transformed(&turn);
    // measured over share 0.61 and agreement 0.047 of the related samples: most near samples lie
    // beyond the tolerance, so the pair is refused through its beyond share
    let rep = report(&[l_part(20, 1.0), b], tol);
    assert!(rep.refused && rep.issues.iter().any(|i| matches!(i, Issue::Misaligned { .. })), "{:?}", rep.issues);
}

#[test]
fn a_displaced_curved_surface_is_never_merged_silently() {
    let tol = 0.1;
    // the top of a sphere and a smaller cap of another tessellation 2 mm higher: they meet near the
    // rim of the cap (where the shift runs along the surface), not at the top
    let top = |m: Mesh, z: f64| Mesh {
        positions: m.positions.clone(),
        triangles: (0..m.triangle_count())
            .filter(|&t| m.face_centroid(t).unwrap().z > z)
            .map(|t| m.triangles[t])
            .collect(),
    };
    let a = top(synth::sphere(Point3::origin(), 20.0, 64, 32), 0.0);
    let b = moved(&top(synth::sphere(Point3::origin(), 20.0, 56, 28), 12.0), Vec3::new(0.0, 0.0, 20.0 * tol));
    // measured over share 1.0 each way and nothing within the tolerance: nothing ties them, refused
    let (hit, issues) = flagged(&[a, b], tol);
    assert!(hit, "{issues:?}");
}

#[test]
fn a_displaced_scan_seen_from_one_side_only_is_never_merged_silently() {
    let tol = 0.1;
    // the second scan sees the wall and only a strip of floor along it (the rest hidden from it)
    let mut b = sheet(4, 16, (15.0, 0.0), 1.25);
    let w = sheet(16, 16, (0.0, 0.0), 1.25);
    b.append(&Mesh {
        positions: w.positions.iter().map(|p| Point3::new(20.0, p.x, p.y)).collect(),
        triangles: w.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect(),
    });
    // measured over share 0.2174 against LAYERS_SHARE 0.14 (1.55 times): warned
    let (hit, issues) = flagged(&[l_part(20, 1.0), moved(&b, Vec3::new(0.0, 0.0, 20.0 * tol))], tol);
    assert!(hit, "{issues:?}");
}

#[test]
fn only_clean_agreement_ties_two_scans() {
    let r = |a: usize, b: usize, near: usize, within: usize, over: usize| PairResidual {
        a,
        b,
        samples: 20_000,
        near,
        within,
        over,
        ..PairResidual::default()
    };
    // 0 and 2 lie over each other and share nothing; 1 agrees with 2, and with 0 as given
    let pairs = |n: usize, w: usize| {
        [
            r(0, 1, n, w, 0),
            r(1, 0, n, w, 0),
            r(1, 2, 8000, 8000, 0),
            r(2, 1, 8000, 8000, 0),
            r(0, 2, 0, 0, 3000),
            r(2, 0, 0, 0, 3000),
        ]
    };
    let displaced = |n, w| pair_issues(3, &pairs(n, w), 0.1).iter().any(|i| matches!(i, Issue::Displaced { .. }));
    // 40 of 45 near samples agree: fewer than MIN_SHARED_SAMPLES, no tie, the displaced pair refused
    assert!(displaced(45, 40));
    // 60 of 60 agree, a small share of what 0 and 1 relate: a clean tie however small, no refusal
    assert!(!displaced(60, 60));
    // 9500 of 10 000 agree (5 % beyond, not misaligned): a tie
    assert!(!displaced(10_000, 9500));
}

/// A flat sheet of `w` x `h` mm (1 mm cells) at `origin`, spanned by `u` and `v` (its normal u x v).
fn plate(origin: Point3, (u, v): (Vec3, Vec3), (w, h): (usize, usize)) -> Mesh {
    let s = sheet(w, h, (0.0, 0.0), 1.0);
    Mesh { positions: s.positions.iter().map(|p| origin + u * p.x + v * p.y).collect(), triangles: s.triangles.clone() }
}

#[test]
fn a_stepped_part_linked_by_a_close_up_merges_and_is_not_refused() {
    let tol = 0.1;
    // A: the floor (z = 0); C: a shelf at z = 10 over its left half, sharing nothing with A; B: a
    // close-up of the shelf's edge: a 20 mm strip of shelf, the riser at x = 20 (facing +x) and a
    // 5 mm strip of floor
    let floor = plate(Point3::new(-20.0, 0.0, 0.0), (Vec3::x(), Vec3::y()), (80, 20));
    let shelf = plate(Point3::new(-20.0, 0.0, 10.0), (Vec3::x(), Vec3::y()), (40, 20));
    let mut close = plate(Point3::new(0.0, 0.0, 10.0), (Vec3::x(), Vec3::y()), (20, 20));
    close.append(&plate(Point3::new(20.0, 0.0, 10.0), (-Vec3::z(), Vec3::y()), (10, 20)));
    close.append(&plate(Point3::new(20.0, 0.0, 0.0), (Vec3::x(), Vec3::y()), (5, 20)));
    let rep = report(&[floor, close, shelf], tol);
    // tied through the close-up (it agrees with both); A and B also lie over each other where the
    // shelf strip is over the floor: warned (`layers_apart`), never refused
    assert!(!rep.refused, "{:?}", rep.issues);
    assert!(warns(&rep), "{:?}", rep.issues);
}

/// A documented limit, not a detection: a closed rod slid along its own axis.
#[test]
fn a_closed_rod_slid_along_its_axis_is_a_documented_limit() {
    let tol = 0.1;
    // a 10 x 10 x 100 mm rod (L/D 10) and another tessellation of it slid 20 tolerances along its axis:
    // the sides slide along themselves and agree; only the end caps lie over each other, at one
    // offset, about 5 % of the related samples, under LAYERS_SHARE. A tight-cluster rule would catch
    // it, but a correct scan of the local corpus has as tight a cluster at a share within 1.4 times:
    // left out, listed in the guide's limits. The merge is not refused and raises no layers_apart.
    let a = synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 100.0), 20);
    let b = synth::cuboid(Point3::new(0.0, 0.0, 20.0 * tol), Vec3::new(10.0, 10.0, 100.0), 24);
    let rep = report(&[a, b], tol);
    assert!(!rep.refused && !warns(&rep), "{:?}", rep.issues);
}

#[test]
fn scans_that_touch_by_too_little_are_reported() {
    let r = |a: usize, b: usize, near: usize, within: usize, over: usize| PairResidual {
        a,
        b,
        samples: 20_000,
        near,
        within,
        over,
        ..PairResidual::default()
    };
    let issues = pair_issues(2, &[r(0, 1, 30, 0, 10), r(1, 0, 20, 0, 0)], 0.1);
    let small = issues.iter().find(|i| matches!(i, Issue::SmallOverlap { .. }));
    assert_eq!(small, Some(&Issue::SmallOverlap { a: 0, b: 1, near: [30, 20], over: [10, 0] }), "{issues:?}");
    // with 50 near samples a verdict is drawn: no such notice
    let judged = pair_issues(2, &[r(0, 1, 60, 60, 0), r(1, 0, 60, 60, 0)], 0.1);
    assert!(!judged.iter().any(|i| matches!(i, Issue::SmallOverlap { .. })), "{judged:?}");
}

#[test]
fn unjudged_samples_leave_the_verdict_to_the_judged_ones() {
    // 19 000 of 20 000 samples unjudged: the share is of the 1000 judged, 60 over is 6 % of them
    let r = PairResidual { a: 0, b: 1, samples: 20_000, unknown: 19_000, over: 60, ..PairResidual::default() };
    assert!(r.displaced(MIN_SHARED_SAMPLES));
    // all but 40 unjudged: fewer than the minimum over, no verdict drawn
    let none = PairResidual { a: 0, b: 1, samples: 20_000, unknown: 19_960, over: 40, ..PairResidual::default() };
    assert!(!none.displaced(MIN_SHARED_SAMPLES));
    let issues =
        pair_issues(2, &[none.clone(), PairResidual { a: 1, b: 0, samples: 20_000, ..PairResidual::default() }], 0.1);
    assert!(issues.iter().any(|i| matches!(i, Issue::Unjudged { samples: 19_960, .. })), "{issues:?}");
}
