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
    let (p, listed) = pieces_of(&m);
    let plan = plan_of(&p, listed, (PART_SHARE, TOL), None);
    assert_eq!((plan.chosen.len(), plan.co_equal), (13, 1));
    assert!(plan.left_out.is_empty() && plan.beyond_cap.is_empty());
    let areas: Vec<f64> = plan.chosen.iter().skip(1).map(|k| p.area(*k)).collect();
    assert!(areas.windows(2).all(|w| w[0] >= w[1]), "small parts by area: {areas:?}");
    assert!(worth_splitting(&plan));
    let asked = plan_of(&p, listed, (PART_SHARE, TOL), Some(1));
    assert_eq!((asked.chosen.len(), asked.not_requested.len()), (1, 12));
}

/// More parts than the cap: the cap keeps the co-equal ones first and counts the rest.
#[test]
fn the_cap_counts_what_it_leaves_out() {
    let mut m = Mesh::default();
    for k in 0..(MAX_COMPONENTS + 12) {
        m.append(&ball(30.0 * k as f64, 5.0, 16));
    }
    let (p, listed) = pieces_of(&m);
    let plan = plan_of(&p, listed, (PART_SHARE, TOL), None);
    assert_eq!((plan.chosen.len(), plan.beyond_cap.len()), (MAX_COMPONENTS, 12));
    // each ball holds under 1 % of the scan, so none is a listed part: the largest stands in as the
    // co-equal one and the others are small parts
    assert_eq!((listed, plan.co_equal), (0, 1));
}

/// One part and debris (small specks): nothing to split.
#[test]
fn one_part_with_debris_is_not_worth_a_split() {
    let mut m = ball(0.0, 40.0, 96);
    for k in 0..30 {
        m.append(&ball(100.0 + 10.0 * f64::from(k), 1.5, 8));
    }
    let (p, listed) = pieces_of(&m);
    let plan = plan_of(&p, listed, (PART_SHARE, TOL), None);
    assert_eq!(listed, 1);
    assert_eq!(plan.left_out.len(), 30);
    assert!(plan.left_out.iter().all(|(_, r)| *r == LeftOutReason::Debris));
    assert!(!worth_splitting(&plan));
}

/// The cleaning keeps a small part that its default (a share of the largest piece) would drop, and
/// keeps its defaults when there is none.
#[test]
fn the_cleaning_keeps_small_parts_below_its_default() {
    let mut m = ball(0.0, 40.0, 240);
    assert!(m.triangle_count() / 100 > 224, "a 224-triangle ball is below 1 % of the largest");
    for k in 0..5 {
        m.append(&ball(100.0 + 10.0 * f64::from(k), 1.5, 16));
    }
    assert_eq!(keep_components_from(&m, TOL), None, "debris only");
    m.append(&ball(300.0, 5.0, 16));
    assert_eq!(keep_components_from(&m, TOL), Some(224));
    assert_eq!(keep_components_from(&Mesh::default(), TOL), None);
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
