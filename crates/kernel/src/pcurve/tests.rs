use super::*;
use crate::record::brep::{EdgeUseRef, OrientedEdge};

const TOL: f64 = 1e-9;

fn frame() -> Frame {
    Frame { origin: [1.0, -2.0, 0.5], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] }
}

fn add(a: [f64; 3], b: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] + k * b[0], a[1] + k * b[1], a[2] + k * b[2]]
}

fn dist(a: [f64; 3], b: [f64; 3]) -> f64 {
    add(a, b, -1.0).iter().map(|c| c * c).sum::<f64>().sqrt()
}

/// The unit radial direction of `f` at angle `u`.
fn radial(f: &Frame, u: f64) -> [f64; 3] {
    add(add([0.0; 3], f.x_axis, u.cos()), f.y_axis, u.sin())
}

fn axis(f: &Frame) -> [f64; 3] {
    cross3(f.x_axis, f.y_axis)
}

fn dummy() -> Curve2 {
    Curve2::Spline { knots: vec![0.0, 0.0, 1.0, 1.0], points: vec![[0.0; 2], [1.0; 2]] }
}

fn circle(origin: [f64; 3], x: [f64; 3], y: [f64; 3], radius: f64) -> Curve {
    Curve::Circle { frame: Frame { origin, x_axis: x, y_axis: y }, radius }
}

/// The four edges of the box `u × v` on `s` as exact circles and lines, and the face on them:
/// bottom →, right ↑, top ←, left ↓ in the record's (u, v). `bottom`/`top` are the v-iso
/// curves of the surface at v0 and v1, `left`/`right` the u-iso ones at u0 and u1.
struct Patch {
    face: Face,
    edges: Vec<Edge>,
    curves: Vec<Curve>,
}

fn patch(
    surface: Surface,
    [u0, u1]: [f64; 2],
    [v0, v1]: [f64; 2],
    v_shift: f64,
    iso_u: impl Fn(f64) -> Curve,
    iso_v: impl Fn(f64) -> Curve,
) -> Patch {
    let curves = vec![iso_v(v0), iso_u(u1), iso_v(v1), iso_u(u0)];
    let intervals = [[u0, u1], [v0 + v_shift, v1 + v_shift], [u0, u1], [v0 + v_shift, v1 + v_shift]];
    let forward = [true, true, false, false];
    let edges: Vec<Edge> = curves
        .iter()
        .zip(intervals)
        .enumerate()
        .map(|(i, (c, interval))| Edge {
            curve: c.clone(),
            nurbs: None,
            interval,
            vertices: [0, 0],
            tolerance: 1e-9,
            uses: vec![EdgeUseRef { face: 0, r#loop: 0, position: i as u32, upper: !forward[i] }],
        })
        .collect();
    let uses = (0..4).map(|i| OrientedEdge { edge: i as u32, forward: forward[i], pcurve: dummy() }).collect();
    let face = Face { surface, nurbs: None, domain: [[u0, u1], [v0, v1]], loops: vec![Loop::Edges { edges: uses }] };
    Patch { face, edges, curves }
}

fn cylinder_patch() -> Patch {
    let f = frame();
    let r = 3.0;
    let (z, f1) = (axis(&f), f.clone());
    let s = Surface::Cylinder { frame: f.clone(), radius: r };
    let iso_v = move |v: f64| circle(add(f1.origin, z, v), f1.x_axis, f1.y_axis, r);
    let sa = s.clone();
    let iso_u = move |u: f64| Curve::Line {
        start: surface_point(&sa, [u, 0.0]).unwrap(),
        end: surface_point(&sa, [u, 1.0]).unwrap(),
    };
    patch(s, [0.4, 2.2], [0.5, 4.0], 0.0, iso_u, iso_v)
}

fn cone_patch(half_angle: f64) -> Patch {
    let f = frame();
    let (r0, z) = (2.0, axis(&f));
    let s = Surface::Cone { frame: f.clone(), radius_at_origin: r0, half_angle };
    let f1 = f.clone();
    let iso_v = move |v: f64| circle(add(f1.origin, z, v), f1.x_axis, f1.y_axis, r0 + v * half_angle.tan());
    let sa = s.clone();
    let iso_u = move |u: f64| Curve::Line {
        start: surface_point(&sa, [u, 0.0]).unwrap(),
        end: surface_point(&sa, [u, 1.0]).unwrap(),
    };
    patch(s, [-1.0, 0.9], [0.3, 1.7], 0.0, iso_u, iso_v)
}

fn sphere_patch() -> Patch {
    let f = frame();
    let (r, z) = (4.0, axis(&f));
    let s = Surface::Sphere { frame: f.clone(), radius: r };
    let f1 = f.clone();
    let iso_v = move |v: f64| circle(add(f1.origin, z, r * v.sin()), f1.x_axis, f1.y_axis, r * v.cos());
    let f2 = f.clone();
    let iso_u = move |u: f64| circle(f2.origin, radial(&f2, u), z, r);
    patch(s, [0.3, 2.9], [-0.6, 0.8], 0.0, iso_u, iso_v)
}

fn torus_patch(tube_start: f64) -> Patch {
    let f = frame();
    let (big, small, z) = (5.0, 1.5, axis(&f));
    let s = Surface::Torus { frame: f.clone(), major_radius: big, minor_radius: small, tube_start };
    let f1 = f.clone();
    let iso_v = move |v: f64| {
        let w = v + tube_start;
        circle(add(f1.origin, z, small * w.sin()), f1.x_axis, f1.y_axis, big + small * w.cos())
    };
    let f2 = f.clone();
    let iso_u = move |u: f64| circle(add(f2.origin, radial(&f2, u), big), radial(&f2, u), z, small);
    patch(s, [0.2, 2.0], [0.3, 1.9], tube_start, iso_u, iso_v)
}

/// Each use's pcurve lies on its edge and consecutive uses meet; returns the largest gap.
fn check_patch(p: &Patch) -> f64 {
    let a = analytic_face(&p.face, &p.edges).unwrap();
    let row = &a.pcurves[0];
    assert_eq!(row.len(), 4);
    let mut worst: f64 = 0.0;
    for (i, pc) in row.iter().enumerate() {
        let gap = pcurve_gap(&a.surface, pc, &p.curves[i], p.edges[i].interval).unwrap();
        worst = worst.max(gap);
    }
    // chain: the traversal end of use i is the traversal start of use i + 1, in uv
    let ends = |i: usize| {
        let k = solidcore_convert::curve2(&row[i]);
        let [t0, t1] = k.interval().unwrap();
        let (a, b) = (k.eval(t0).unwrap(), k.eval(t1).unwrap());
        if [true, true, false, false][i] { (a, b) } else { (b, a) }
    };
    for i in 0..4 {
        let (end, next) = (ends(i).1, ends((i + 1) % 4).0);
        assert!((end[0] - next[0]).abs() < TOL && (end[1] - next[1]).abs() < TOL, "use {i}: {end:?} vs {next:?}");
    }
    worst
}

#[test]
fn analytic_pcurves_lie_on_their_edges() {
    let cases = [
        ("cylinder", cylinder_patch()),
        ("cone", cone_patch(0.3)),
        ("cone narrowing", cone_patch(-0.2)),
        ("sphere", sphere_patch()),
        ("torus", torus_patch(0.0)),
        ("torus with tube start", torus_patch(0.7)),
    ];
    for (name, p) in &cases {
        let gap = check_patch(p);
        eprintln!("h1 pcurve gap on a {name} patch: {gap:e} mm");
        assert!(gap <= 1e-9, "{name}: {gap}");
    }
}

#[test]
fn a_cone_narrowing_along_z_is_written_with_a_positive_half_angle() {
    let f = frame();
    let narrowing = Surface::Cone { frame: f, radius_at_origin: 2.0, half_angle: -0.3 };
    let written = written_surface(&narrowing).unwrap();
    let Surface::Cone { half_angle, .. } = &written else { panic!("{written:?}") };
    assert!(*half_angle > 0.0);
    for (u, v) in [(0.3, 0.5), (2.0, 1.2), (-1.0, 0.1)] {
        let a = surface_point(&narrowing, [u, v]).unwrap();
        let b = surface_point(&written, [-u, -v]).unwrap();
        assert!(dist(a, b) < TOL, "{a:?} vs {b:?}");
    }
}

#[test]
fn a_torus_tube_start_shifts_v() {
    let f = frame();
    let shifted = Surface::Torus { frame: f, major_radius: 5.0, minor_radius: 1.5, tube_start: 0.7 };
    let written = written_surface(&shifted).unwrap();
    let a = surface_point(&shifted, [0.4, 0.2]).unwrap();
    let b = surface_point(&written, [0.4, 0.9]).unwrap();
    assert!(dist(a, b) < TOL);
}

#[test]
fn a_plane_with_skew_scaled_axes_maps_exactly() {
    let (o, a, b) = ([1.0, 2.0, 3.0], [2.0, 0.0, 0.0], [1.0, 3.0, 0.5]);
    let plane = step_plane(o, a, b).unwrap();
    let rec = Surface::Plane { origin: o, u_axis: a, v_axis: b };
    let line = Curve2::Spline { knots: vec![0.0, 0.0, 1.0, 1.0], points: vec![[0.2, 0.1], [1.3, -0.7]] };
    let mapped = plane.pcurve(&line);
    for t in [0.0, 0.3, 1.0] {
        let uv = solidcore_convert::curve2(&line).eval(t).unwrap();
        let st = solidcore_convert::curve2(&mapped).eval(t).unwrap();
        let (p, q) = (surface_point(&rec, uv).unwrap(), surface_point(&plane.surface, st).unwrap());
        assert!(dist(p, q) < TOL, "{p:?} vs {q:?}");
    }
    let rational = Curve2::RationalSpline {
        knots: vec![0.0, 0.0, 1.0, 1.0],
        homogeneous: vec![[0.4, 0.2, 2.0], [2.6, -1.4, 2.0]],
    };
    let m = plane.pcurve(&rational);
    let (uv, st) =
        (solidcore_convert::curve2(&rational).eval(0.5).unwrap(), solidcore_convert::curve2(&m).eval(0.5).unwrap());
    assert!(dist(surface_point(&rec, uv).unwrap(), surface_point(&plane.surface, st).unwrap()) < TOL);
    assert!(step_plane(o, a, a).is_err());
}

#[test]
fn an_edge_that_is_not_an_iso_curve_is_refused_and_bad_surfaces_are_rejected() {
    let mut p = cylinder_patch();
    // a slanted section: the circle tilted about x
    if let Some(e) = p.edges.get_mut(0) {
        e.curve = circle([1.0, -2.0, 1.0], [1.0, 0.0, 0.0], [0.0, 0.6, 0.8], 3.0);
    }
    let err = analytic_face(&p.face, &p.edges).unwrap_err().to_string();
    assert!(err.contains("not supported yet") || err.contains("does not lie"), "{err}");
    let f = frame();
    assert!(written_surface(&Surface::Cylinder { frame: f.clone(), radius: 0.0 }).is_err());
    assert!(
        written_surface(&Surface::Torus { frame: f.clone(), major_radius: 1.0, minor_radius: 2.0, tube_start: 0.0 })
            .is_err()
    );
    assert!(written_surface(&Surface::Cone { frame: f.clone(), radius_at_origin: 1.0, half_angle: 0.0 }).is_err());
    assert!(written_surface(&Surface::Cone { frame: f, radius_at_origin: -1.0, half_angle: 0.3 }).is_err());
}

#[test]
fn kernel_cylinder_side_has_a_seam_one_period_apart() {
    let b = crate::solid::make_cylinder([0.0; 3], 5.0, 10.0).unwrap();
    let face = &b.faces[0];
    assert!(is_analytic(&face.surface));
    let a = analytic_face(face, &b.edges).unwrap();
    assert!(!a.flip_sense, "SMLib's cylinder form has the analytic normal");
    let row = &a.pcurves[0];
    let uv = |i: usize, t: f64| solidcore_convert::curve2(&row[i]).eval(t).unwrap();
    // the loop is seam ↓ (edge 0 reversed), bottom circle →, seam ↑, top circle ←: the two seam
    // uses sit a turn apart
    // The loop runs counter-clockwise in (u, v) seen along the normal: up the seam at the larger
    // u (the forward use, position 2), down it at the smaller (the reversed use, position 0). The
    // frame's x axis meets the seam, so the two sides are exactly u = 0 and u = 2π.
    let (s0, s1) = (uv(0, 5.0), uv(2, 5.0));
    assert!(s0[0].abs() < TOL, "reversed use at u = 0: {s0:?}");
    assert!((s1[0] - std::f64::consts::TAU).abs() < TOL, "forward use at u = 2π: {s1:?}");
    for (i, pc) in row.iter().enumerate() {
        let Loop::Edges { edges: uses } = &face.loops[0] else { panic!() };
        let e = &b.edges[uses[i].edge as usize];
        let gap = pcurve_gap(&a.surface, pc, e.nurbs.as_ref().unwrap_or(&e.curve), e.interval).unwrap();
        eprintln!("h1 kernel cylinder use {i}: gap {gap:e} mm");
        assert!(gap <= 1e-9, "{gap}");
    }
}

/// A whole sphere: one face whose meridian seam is used twice, meeting itself at the poles.
#[test]
fn kernel_sphere_seam_uses_are_a_turn_apart() {
    let b = crate::solid::make_sphere([0.0; 3], 5.0).unwrap();
    let face = &b.faces[0];
    let a = analytic_face(face, &b.edges).unwrap();
    let row = &a.pcurves[0];
    assert_eq!(row.len(), 2);
    let u_of = |i: usize| mid_of(&row[i]).unwrap()[0];
    // the loop is [reversed use, forward use]: down the meridian on the left, up it on the right,
    // so the forward use is the later one, a turn after the reversed one (signed, not absolute)
    assert!((u_of(1) - u_of(0) - std::f64::consts::TAU).abs() < TOL, "{} {}", u_of(0), u_of(1));
    assert!(u_of(0).abs() <= std::f64::consts::PI, "the first side starts in the principal range");
    let Loop::Edges { edges: uses } = &face.loops[0] else { panic!() };
    for (i, pc) in row.iter().enumerate() {
        let e = &b.edges[uses[i].edge as usize];
        let gap = pcurve_gap(&a.surface, pc, e.nurbs.as_ref().unwrap_or(&e.curve), e.interval).unwrap();
        eprintln!("h1 kernel sphere use {i}: gap {gap:e} mm");
        assert!(gap <= 1e-9, "{gap}");
    }
}

/// A record's tolerance is input: a huge one must not make an off-surface edge "lie on" the
/// surface, and a NaN one counts as the least.
#[test]
fn a_hostile_edge_tolerance_is_capped() {
    for tolerance in [1e6, f64::INFINITY, f64::NAN, -1.0] {
        let mut p = cylinder_patch();
        if let Some(e) = p.edges.get_mut(0) {
            e.curve = circle([1.0, -2.0, 1.0], [1.0, 0.0, 0.0], [0.0, 0.6, 0.8], 3.0);
            e.tolerance = tolerance;
        }
        assert!(analytic_face(&p.face, &p.edges).is_err(), "tolerance {tolerance}");
    }
    let mut p = cylinder_patch();
    p.edges[0].tolerance = 1e6;
    assert!(analytic_face(&p.face, &p.edges).is_ok(), "an honest edge is still accepted");
    assert!(edge_tolerance(&p.edges[0]) <= MAX_TOLERANCE_MM);
}

#[test]
fn an_empty_edge_interval_is_refused() {
    let mut p = cylinder_patch();
    let t = p.edges[1].interval[0];
    p.edges[1].interval = [t, t];
    assert!(analytic_face(&p.face, &p.edges).is_err());
}
