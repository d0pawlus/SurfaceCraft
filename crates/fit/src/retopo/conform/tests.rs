use super::*;
use surfacecraft_geom::Rng;

/// The square [0, 10]², region on the left of the travel direction, four corners.
fn square() -> Outline {
    square_of(10.0)
}

fn square_of(size: f64) -> Outline {
    let pts = [(0.0, 0.0), (size, 0.0), (size, size), (0.0, size)];
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

/// Improper quads among those the layout is converted to (fan of every face, as `quad.retopo` does).
fn improper_quads(m: &PolyMesh) -> usize {
    invalid_quads(&m.split_to_quads(&|p, n| Some((*p, *n))))
}

/// Bottom-edge vertices at `n` equal steps between two forced corners, `n - 1` of them in between.
fn bottom_run(between: usize) -> PolyMesh {
    let mut pts = vec![(0.0, -0.1)];
    pts.extend((1..=between).map(|k| (10.0 * k as f64 / (between + 1) as f64, -0.1)));
    pts.extend([(10.0, -0.1), (10.1, 10.0), (0.0, 10.1), (-0.1, 5.0)]);
    poly(&pts)
}

/// A span that holds its vertices exactly (199 × 0.05 + 0.05 = 10) must not panic in the bounds;
/// one vertex more is rolled back, one less is conformed.
#[test]
fn an_exactly_full_span_neither_panics_nor_overflows() {
    for between in [198usize, 199, 200] {
        let mut m = bottom_run(between);
        let report = conform_to_outline(&mut m, &square(), 1.0, &up);
        assert_eq!(report.conformed + report.rolled_back + report.skipped, 1, "{between}: {report:?}");
        if between == 200 {
            assert_eq!(report.rolled_back, 1, "{report:?}");
        }
        if between == 198 {
            assert_eq!(report.conformed, 1, "{report:?}");
        }
    }
}

/// An accepted dent polygon whose fan from the arithmetic centre would invert a quad: what the
/// layout is converted to must be proper, so the loop is rolled back (or decomposed safely).
#[test]
fn a_concave_dent_polygon_never_converts_to_an_inverted_quad() {
    let mut m = poly(&[
        (10.0, 0.6),
        (20.0, 20.0),
        (40.0, 20.0),
        (50.0, 1.1),
        (60.0, 20.0),
        (80.0, 20.0),
        (90.0, 0.6),
        (99.5, 50.0),
        (90.0, 99.4),
        (10.0, 99.4),
        (0.5, 50.0),
    ]);
    let before = (improper_quads(&m), m.vertices.clone(), m.faces.clone());
    let report = conform_to_outline(&mut m, &square_of(100.0), 1.0, &up);
    // conforming must not add an improper quad; here it is rolled back and the layout is as it was
    assert!(improper_quads(&m) <= before.0, "{report:?}");
    assert_eq!(report.rolled_back, 1, "{report:?}");
    assert_eq!((m.vertices, m.faces), (before.1, before.2));
}

/// After a split, the re-snap must not put two neighbouring vertices on one outline point.
#[test]
fn re_snapping_a_split_layout_keeps_its_vertices_apart() {
    let layout = super::super::QuadMesh {
        vertices: [(-0.1, 0.0), (0.0, -0.1), (1.0, 0.0), (0.0, 1.0)]
            .iter()
            .map(|p| Point3::new(p.0, p.1, 0.0))
            .collect(),
        normals: vec![Vec3::z(); 4],
        faces: vec![[0, 1, 2, 3]],
    };
    let split = layout.split(&|p, n| Some((*p, *n)));
    let (snapped, _) = snap_to_outline(&split, &square(), 1.0, 0.5);
    for (i, a) in snapped.vertices.iter().enumerate() {
        for b in &snapped.vertices[i + 1..] {
            assert!((a - b).norm() > 1e-9, "{a:?} {b:?}");
        }
    }
}

// ---------------------------------------------------------------------------------------------
// property test over three outlines on a gently curved surface

fn curved(x: f64, y: f64) -> Point3 {
    Point3::new(x, y, 0.01 * (x * x + y * y))
}

fn curved_normal(x: f64, y: f64) -> Vec3 {
    Vec3::new(-0.02 * x, -0.02 * y, 1.0).normalize()
}

/// A counter-clockwise polygon on the curved surface as an outline; `corner` marks its sharp vertices.
fn outline_of(pts: &[(f64, f64)], corner: &dyn Fn(usize) -> bool) -> Outline {
    let n = pts.len();
    let out_of = |a: (f64, f64), b: (f64, f64)| {
        let (dx, dy) = (b.0 - a.0, b.1 - a.1);
        let l = dx.hypot(dy);
        (dy / l, -dx / l)
    };
    let outward: Vec<Vec3> = (0..n)
        .map(|k| {
            let (a, b) = (out_of(pts[(k + n - 1) % n], pts[k]), out_of(pts[k], pts[(k + 1) % n]));
            Vec3::new(a.0 + b.0, a.1 + b.1, 0.0).normalize()
        })
        .collect();
    let lp = OutlineLoop {
        points: pts.iter().map(|p| curved(p.0, p.1)).collect(),
        corners: (0..n).filter(|&k| corner(k)).collect(),
        outward,
    };
    Outline { loops: vec![lp], slack: 0.0 }
}

fn shapes() -> Vec<(Outline, (f64, f64), f64)> {
    let square = [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)];
    let l_shape = [(0.0, 0.0), (10.0, 0.0), (10.0, 4.0), (4.0, 4.0), (4.0, 10.0), (0.0, 10.0)];
    let circle: Vec<(f64, f64)> = (0..24)
        .map(|k| {
            let a = std::f64::consts::TAU * k as f64 / 24.0;
            (5.0 + 5.0 * a.cos(), 5.0 + 5.0 * a.sin())
        })
        .collect();
    vec![
        (outline_of(&square, &|_| true), (5.0, 5.0), 5.0),
        (outline_of(&l_shape, &|_| true), (3.0, 3.0), 4.0),
        (outline_of(&circle, &|_| false), (5.0, 5.0), 5.0),
    ]
}

fn curved_poly(points: &[(f64, f64)]) -> PolyMesh {
    PolyMesh {
        vertices: points.iter().map(|p| curved(p.0, p.1)).collect(),
        normals: points.iter().map(|p| curved_normal(p.0, p.1)).collect(),
        faces: vec![(0..points.len() as u32).collect()],
    }
}

/// The quads `quad.retopo` ends with: the layout converted, split once and re-snapped to the outline.
fn pipeline(poly: &PolyMesh, outline: &Outline, edge: f64) -> super::super::QuadMesh {
    let id = |p: &Point3, n: &Vec3| Some((*p, *n));
    let quads = poly.as_quads().unwrap_or_else(|| poly.split_to_quads(&id));
    let split = quads.split(&id);
    snap_to_outline(&split, outline, 0.5 * edge, 0.4 * edge).0
}

/// What a random layout exercises, counted over all cases.
#[derive(Default, Debug)]
struct Seen {
    seam: usize,
    dent: usize,
    forced: usize,
    rolled_back: usize,
    conformed: usize,
}

fn classify(poly: &PolyMesh, outline: &Outline, edge: f64, seen: &mut Seen) {
    let lp = &outline.loops[0];
    let raw: Vec<f64> = poly.vertices.iter().map(|p| lp.closest(p).unwrap().0).collect();
    let n = raw.len();
    seen.seam += usize::from((0..n - 1).any(|k| raw[k + 1] < raw[k]));
    seen.dent += usize::from(poly.vertices.iter().any(|p| lp.closest(p).unwrap().2 > edge));
    let (arcs, total) = lp.arc_lengths();
    let near_corner = |a: f64| {
        lp.corners.iter().any(|&c| {
            let d = (a - arcs[c]).rem_euclid(total);
            d.min(total - d) <= 0.75 * edge
        })
    };
    seen.forced += usize::from(raw.iter().any(|&a| near_corner(a)));
}

/// Random layouts around a square, an L and a circle-like outline on a curved surface: conform,
/// split and re-snap never panic, never add an improper quad compared with not conforming, and a
/// loop is either conformed (ordered, gapped arcs) or reported and left exactly as it was.
#[test]
fn random_layouts_on_three_outlines_conform_split_and_resnap_safely() {
    let mut rng = Rng::new(11);
    let mut seen = Seen::default();
    for case in 0..900 {
        let (outline, centre, size) = &shapes()[case % 3];
        let m_count = 3 + rng.index(10);
        let mut angles: Vec<f64> = (0..m_count).map(|_| rng.range(0.0, std::f64::consts::TAU)).collect();
        angles.sort_by(f64::total_cmp);
        let pts: Vec<(f64, f64)> = angles
            .iter()
            .map(|a| {
                let r = rng.range(0.4, 1.25) * size;
                (centre.0 + r * a.cos(), centre.1 + r * a.sin())
            })
            .collect();
        let edge = rng.range(0.3, 3.0);
        let mut poly = curved_poly(&pts);
        let before = (poly.vertices.clone(), poly.faces.clone());
        let plain = pipeline(&poly, outline, edge);
        classify(&poly, outline, edge, &mut seen);
        let report =
            conform_to_outline(&mut poly, outline, edge, &|p: &Point3, _: &Vec3| Some(curved_normal(p.x, p.y)));
        if report.conformed == 0 {
            assert_eq!((poly.vertices.clone(), poly.faces.clone()), before, "case {case}: {report:?}");
        } else {
            assert_ordered(&poly, outline, 0.05 * edge);
        }
        seen.rolled_back += report.rolled_back;
        seen.conformed += report.conformed;
        let last = pipeline(&poly, outline, edge);
        assert!(invalid_quads(&last) <= invalid_quads(&plain), "case {case}: {report:?}");
    }
    assert!(
        seen.seam > 0 && seen.dent > 0 && seen.forced > 0 && seen.rolled_back > 0 && seen.conformed > 0,
        "{seen:?}"
    );
}
