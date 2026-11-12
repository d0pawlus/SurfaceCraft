use super::*;
use surfacecraft_geom::Rng;

/// The square [0, 10]², region on the left of the travel direction, four corners.
fn square() -> Outline {
    let pts = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
    let h = std::f64::consts::FRAC_1_SQRT_2;
    let outward = [(-h, -h), (h, -h), (h, h), (-h, h)];
    let lp = OutlineLoop {
        points: pts.iter().map(|p| Point3::new(p.0, p.1, 0.0)).collect(),
        corners: vec![0, 1, 2, 3],
        outward: outward.iter().map(|o| Vec3::new(o.0, o.1, 0.0)).collect(),
    };
    Outline { loops: vec![lp], slack: 0.0 }
}

fn poly(points: &[(f64, f64)]) -> PolyMesh {
    PolyMesh {
        vertices: points.iter().map(|p| Point3::new(p.0, p.1, 0.0)).collect(),
        normals: vec![Vec3::z(); points.len()],
        faces: vec![(0..points.len() as u32).collect()],
    }
}

fn up(_: &Point3, _: &Vec3) -> Option<Vec3> {
    Some(Vec3::z())
}

fn down(_: &Point3, _: &Vec3) -> Option<Vec3> {
    Some(-Vec3::z())
}

fn flat(m: &PolyMesh, f: &[u32]) -> Vec<(f64, f64)> {
    f.iter().map(|&v| (m.vertices[v as usize].x, m.vertices[v as usize].y)).collect()
}

/// Positive area, no zero edge, no self-intersection.
fn assert_simple(m: &PolyMesh, f: &[u32]) {
    let p = flat(m, f);
    let n = p.len();
    let area: f64 = (0..n).map(|k| p[k].0 * p[(k + 1) % n].1 - p[(k + 1) % n].0 * p[k].1).sum::<f64>() / 2.0;
    assert!(area > 1e-9, "area {area} of {p:?}");
    for i in 0..n {
        assert!((p[i].0 - p[(i + 1) % n].0).hypot(p[i].1 - p[(i + 1) % n].1) > 1e-9, "zero edge in {p:?}");
        for j in i + 2..n {
            if (j + 1) % n != i {
                assert!(!segments_cross(p[i], p[(i + 1) % n], p[j], p[(j + 1) % n]), "self-intersection {p:?}");
            }
        }
    }
}

/// Every free loop runs once around the square with strictly increasing arcs and `gap` between
/// neighbours (also across the closing pair).
fn assert_ordered(m: &PolyMesh, outline: &Outline, gap: f64) {
    let lp = &outline.loops[0];
    let (_, total) = lp.arc_lengths();
    for ids in free_loops(&m.faces) {
        let arcs: Vec<f64> = ids.iter().map(|&v| lp.closest(&m.vertices[v as usize]).unwrap().0).collect();
        let diffs: Vec<f64> =
            (0..arcs.len()).map(|k| (arcs[(k + 1) % arcs.len()] - arcs[k]).rem_euclid(total)).collect();
        assert!(diffs.iter().all(|&d| d >= 0.99 * gap), "gaps {diffs:?} arcs {arcs:?}");
        assert!((diffs.iter().sum::<f64>() - total).abs() < 1e-6, "not one revolution: {diffs:?}");
    }
}

fn assert_valid(m: &PolyMesh) {
    for f in &m.faces {
        assert_simple(m, f);
    }
}

/// The outer ring of eight points around [0, 10]²; two neighbours of each corner project onto it.
const RING: [(f64, f64); 8] =
    [(-0.1, 0.0), (0.0, -0.1), (10.0, -0.1), (10.1, 0.0), (10.1, 10.0), (10.0, 10.1), (0.0, 10.1), (-0.1, 10.0)];

/// Neighbours projecting to one outline point leave no zero-length edge, whichever vertex starts
/// the loop (the closing pair included).
#[test]
fn neighbours_projecting_to_one_point_leave_no_zero_length_edge() {
    for start in 0..RING.len() {
        let pts: Vec<(f64, f64)> = (0..RING.len()).map(|k| RING[(start + k) % RING.len()]).collect();
        let mut m = poly(&pts);
        let report = conform_to_outline(&mut m, &square(), 4.0, &up);
        assert_eq!(report.conformed + report.rolled_back, 1, "start {start}: {report:?}");
        assert_valid(&m);
        if report.conformed == 1 {
            assert_ordered(&m, &square(), 0.05 * 4.0);
        }
    }
}

/// An interval that wraps past the start of the loop gets its corners in arc order.
#[test]
fn wrapped_ring_intervals_insert_corners_in_arc_order() {
    let mut m = poly(&[(5.0, 8.0), (5.0, 2.0), (8.0, 2.0), (8.0, 8.0)]);
    let report = conform_to_outline(&mut m, &square(), 2.0, &up);
    assert_eq!(report.conformed, 1, "{report:?}");
    assert!(m.faces.len() > 1);
    assert_valid(&m);
}

/// Projected arcs 9.96, 9.97, 9.99 and a corner at 10: the vertex that becomes the corner is not
/// overtaken by its neighbours, which keep the minimum gap behind it.
#[test]
fn a_forced_corner_is_not_overtaken_by_its_neighbours() {
    let mut m = poly(&[
        (0.5, -0.1),
        (9.96, -0.1),
        (9.97, -0.1),
        (9.99, -0.1),
        (10.1, 0.5),
        (10.1, 9.5),
        (9.5, 10.1),
        (0.5, 10.1),
        (-0.1, 9.5),
        (-0.1, 0.5),
    ]);
    let report = conform_to_outline(&mut m, &square(), 1.0, &up);
    assert_eq!(report.conformed, 1, "{report:?}");
    assert_valid(&m);
    assert_ordered(&m, &square(), 0.05);
}

/// Planar ring quad whose new vertices would carry the opposite wall's normal (summed normal
/// zero): the loop is rolled back and reported, nothing is dropped silently.
#[test]
fn an_undefined_face_normal_rolls_the_loop_back_and_says_so() {
    let mut m = poly(&[(1.0, 1.0), (9.0, 1.0), (9.0, 9.0), (1.0, 9.0)]);
    let before = (m.vertices.clone(), m.faces.clone());
    let report = conform_to_outline(&mut m, &square(), 1.0, &down);
    assert_eq!(report.rolled_back, 1, "{report:?}");
    assert!(report.reason.is_some());
    assert_eq!((m.vertices, m.faces), before);
}

/// More vertices between two corners than their arc can hold with the minimum gap: not forced.
#[test]
fn a_span_with_more_vertices_than_fit_is_rolled_back() {
    let mut pts = vec![(0.0, -0.1)];
    pts.extend((1..=39).map(|k| (k as f64 * 0.25, -0.1)));
    pts.extend([(10.1, 0.0), (10.1, 10.0), (0.0, 10.1), (-0.1, 5.0)]);
    let mut m = poly(&pts);
    let before = (m.vertices.clone(), m.faces.clone());
    let report = conform_to_outline(&mut m, &square(), 10.0, &up);
    assert_eq!(report.rolled_back, 1, "{report:?}");
    assert_eq!(report.reason, Some("a span between corners is too short for its vertices"));
    assert_eq!((m.vertices, m.faces), before);
}

/// Random star-shaped vertex sets around the square: every loop is either reported unconformed and
/// unchanged, or runs once around the outline with the minimum gap and only valid faces.
#[test]
fn random_layouts_are_unchanged_or_conformed_with_gaps_and_valid_faces() {
    let mut rng = Rng::new(7);
    let (mut conformed, mut left) = (0, 0);
    for _ in 0..300 {
        let m_count = 3 + rng.index(9);
        let mut angles: Vec<f64> = (0..m_count).map(|_| rng.range(0.0, std::f64::consts::TAU)).collect();
        angles.sort_by(f64::total_cmp);
        let pts: Vec<(f64, f64)> = angles
            .iter()
            .map(|a| {
                let r = rng.range(2.0, 7.0);
                (5.0 + r * a.cos(), 5.0 + r * a.sin())
            })
            .collect();
        let edge = rng.range(0.5, 4.0);
        let mut m = poly(&pts);
        let before = (m.vertices.clone(), m.faces.clone());
        let report = conform_to_outline(&mut m, &square(), edge, &up);
        if report.conformed == 0 {
            assert_eq!((m.vertices.clone(), m.faces.clone()), before, "{pts:?} {report:?}");
            left += 1;
        } else {
            assert_valid(&m);
            assert_ordered(&m, &square(), 0.05 * edge);
            conformed += 1;
        }
    }
    assert!(conformed > 20 && left > 0, "conformed {conformed}, left {left}");
}
