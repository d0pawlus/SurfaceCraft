//! Tests of `pipeline_parts.rs`: which pieces of a scan are parts.
use super::*;
use crate::cmd::scan_split::{self, MIN_SHARE};
use surfacecraft_geom::{Point3, synth};

const TOL: f64 = 0.2;

fn ball(at: f64, r: f64, res: usize) -> Mesh {
    let mut m = synth::sphere(Point3::new(at, 0.0, 0.0), r, res, res / 2);
    m.weld(1e-9);
    m
}

fn pieces_of(m: &Mesh) -> (Pieces, usize) {
    let a = scan_split::analyse(m, MIN_SHARE, true);
    let listed = a.parts.len();
    (Pieces::of(m, a), listed)
}

fn only_piece(m: &Mesh) -> (Piece, f64) {
    let p = pieces(m).remove(0);
    let v = signed_volume(m, &p);
    (p, v)
}

/// A ball of 16 x 8 (224 triangles) and 5 mm radius is a part; the same ball of 1.5 mm radius, or a
/// ball of 60 triangles, is debris; a cut-open ball is open; a ball turned inside out faces inward;
/// a ball with one triangle turned has no consistent inside.
#[test]
fn a_piece_is_a_small_part_by_what_it_is() {
    let (p, v) = only_piece(&ball(0.0, 5.0, 16));
    assert_eq!(reason_of(&p, v, TOL), None);
    let (p, v) = only_piece(&ball(0.0, 1.5, 16));
    assert_eq!(reason_of(&p, v, TOL), Some(LeftOutReason::Debris), "5.2 mm across is below 10 mm");
    assert_eq!(reason_of(&p, v, 0.02), None, "the floor is in tolerances: at 0.02 mm it is a part");
    let (p, v) = only_piece(&ball(0.0, 5.0, 8));
    assert!(p.triangles.len() < MIN_PART_TRIANGLES);
    assert_eq!(reason_of(&p, v, TOL), Some(LeftOutReason::Debris));
    let mut open = ball(0.0, 5.0, 16);
    open.triangles.truncate(open.triangles.len() - 16);
    let (p, v) = only_piece(&open);
    assert_eq!(reason_of(&p, v, TOL), Some(LeftOutReason::Open));
    let mut inside_out = ball(0.0, 5.0, 16);
    inside_out.triangles.iter_mut().for_each(|t| t.swap(1, 2));
    let (p, v) = only_piece(&inside_out);
    assert_eq!(reason_of(&p, v, TOL), Some(LeftOutReason::InsideOut));
    let mut turned = ball(0.0, 5.0, 16);
    if let Some(t) = turned.triangles.get_mut(40) {
        t.swap(1, 2);
    }
    let (p, v) = only_piece(&turned);
    assert_eq!(reason_of(&p, v, TOL), Some(LeftOutReason::NotOriented));
    assert_eq!(reason_of(&p, f64::NAN, TOL), Some(LeftOutReason::NotOriented));
    let (p, _) = only_piece(&ball(0.0, 5.0, 16));
    assert_eq!(reason_of(&p, f64::NAN, TOL), Some(LeftOutReason::InsideOut), "no volume is no part");
}

/// One large ball and twelve small closed balls below a tenth of its area: one co-equal part and
/// twelve small parts, largest first; asking for a number takes co-equal parts only.
#[test]
fn small_closed_solids_are_planned_after_the_co_equal_parts() {
    let mut m = ball(0.0, 40.0, 96);
    for k in 0..12 {
        m.append(&ball(100.0 + 30.0 * f64::from(k), 5.0 + 0.1 * f64::from(k), 16));
    }
    let (p, _) = pieces_of(&m);
    let plan = plan_of(&p, (PART_SHARE, TOL), None);
    assert_eq!((plan.chosen.len(), plan.co_equal), (13, 1));
    assert!(plan.left_out.is_empty() && plan.beyond_cap.is_empty());
    let areas: Vec<f64> = plan.chosen.iter().skip(1).map(|k| p.area(*k)).collect();
    assert!(areas.windows(2).all(|w| w[0] >= w[1]), "small parts by area: {areas:?}");
    assert!(worth_splitting(&plan));
    let asked = plan_of(&p, (PART_SHARE, TOL), Some(1));
    assert_eq!((asked.chosen.len(), asked.not_requested.len()), (1, 12));
}

/// More parts than the cap, all equal: each is co-equal by its area (the split listing stops at 64,
/// the co-equal rule does not); the cap models the first 128 and counts the rest.
#[test]
fn the_cap_counts_what_it_leaves_out() {
    let mut m = Mesh::default();
    for k in 0..(MAX_COMPONENTS + 12) {
        m.append(&ball(30.0 * k as f64, 5.0, 16));
    }
    let (p, _) = pieces_of(&m);
    let plan = plan_of(&p, (PART_SHARE, TOL), None);
    assert_eq!((plan.chosen.len(), plan.beyond_cap.len()), (MAX_COMPONENTS, 12));
    assert_eq!(plan.co_equal, MAX_COMPONENTS, "more co-equal parts than the cap: the cap takes co-equal ones only");
}

/// Seventy equal balls: past the 64 the split lists, every one is still co-equal; a ball just above a
/// tenth of the largest area is co-equal, one just below is a small part.
#[test]
fn co_equal_is_decided_by_area_not_by_the_listing() {
    let mut m = Mesh::default();
    for k in 0..70 {
        m.append(&ball(30.0 * f64::from(k), 5.0, 16));
    }
    let (p, listed) = pieces_of(&m);
    assert_eq!(listed, scan_split::MAX_PARTS);
    let plan = plan_of(&p, (PART_SHARE, TOL), None);
    assert_eq!((plan.co_equal, plan.chosen.len()), (70, 70));
    // r = 20: area 5027; r = 6.4: 515 (10.2 %); r = 6.2: 483 (9.6 %)
    let mut two = ball(0.0, 20.0, 96);
    two.append(&ball(100.0, 6.4, 96));
    two.append(&ball(200.0, 6.2, 96));
    let (p, _) = pieces_of(&two);
    let plan = plan_of(&p, (PART_SHARE, TOL), None);
    assert_eq!((plan.co_equal, plan.chosen.len()), (2, 3));
}

/// One part and debris (small specks): nothing to split.
#[test]
fn one_part_with_debris_is_not_worth_a_split() {
    let mut m = ball(0.0, 40.0, 96);
    for k in 0..30 {
        m.append(&ball(100.0 + 10.0 * f64::from(k), 1.5, 8));
    }
    let (p, listed) = pieces_of(&m);
    let plan = plan_of(&p, (PART_SHARE, TOL), None);
    assert_eq!(listed, 1);
    assert_eq!(plan.left_out.len(), 30);
    assert!(plan.left_out.iter().all(|(_, r)| *r == LeftOutReason::Debris));
    assert!(!worth_splitting(&plan));
}

/// The cleaning protects exactly the small parts below its default: debris only gives nothing; a
/// closed ball of 224 triangles is protected, an open fragment of the same size beside it is not.
#[test]
fn the_cleaning_protects_exactly_the_small_parts_below_its_default() {
    let mut m = ball(0.0, 40.0, 240);
    assert!(m.triangle_count() / 100 > 224, "a 224-triangle ball is below 1 % of the largest");
    for k in 0..5 {
        m.append(&ball(100.0 + 10.0 * f64::from(k), 1.5, 16));
    }
    assert!(protected_triangles(&m, TOL).is_empty(), "debris only");
    let mut open = ball(300.0, 6.0, 20);
    open.triangles.truncate(open.triangles.len() - 40);
    m.append(&open);
    assert!(protected_triangles(&m, TOL).is_empty(), "an open fragment is not protected");
    let before = m.triangle_count() as u32;
    m.append(&ball(400.0, 5.0, 16));
    let kept = protected_triangles(&m, TOL);
    assert_eq!(kept.len(), 224);
    assert!(kept.iter().all(|t| *t >= before), "exactly the triangles of the ball");
    assert!(protected_triangles(&Mesh::default(), TOL).is_empty());
}

/// The volume of a piece far from the origin keeps its digits; a piece with an infinite coordinate
/// has no volume and is no part.
#[test]
fn the_volume_is_taken_about_the_piece_and_must_be_finite() {
    let near = only_piece(&ball(0.0, 5.0, 16)).1;
    let mut far = ball(0.0, 5.0, 16);
    far.positions.iter_mut().for_each(|p| *p += surfacecraft_geom::Vec3::new(1e6, 1e6, 1e6));
    let (p, v) = only_piece(&far);
    assert!((v - near).abs() < 1e-6 * near, "{v} against {near}");
    assert_eq!(reason_of(&p, v, TOL), None);
    let mut broken = ball(0.0, 5.0, 16);
    if let Some(q) = broken.positions.get_mut(3) {
        q.x = f64::INFINITY;
    }
    let ps = pieces(&broken);
    assert!(ps.iter().all(|p| reason_of(p, signed_volume(&broken, p), TOL).is_some()), "{}", ps.len());
}

/// Co-equal counting for the shell check is unchanged: two equal balls are two co-equal parts, a
/// ball beside a small one is one.
#[test]
fn co_equal_counting_ignores_small_parts() {
    let mut two = ball(0.0, 10.0, 32);
    two.append(&ball(40.0, 10.0, 32));
    assert_eq!(coequal_parts(&two), 2);
    let mut one = ball(0.0, 40.0, 96);
    one.append(&ball(100.0, 5.0, 16));
    assert_eq!(coequal_parts(&one), 1);
}
