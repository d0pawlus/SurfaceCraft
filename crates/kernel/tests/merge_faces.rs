//! Merging patches that lie on a primitive into one analytic face (P9-O6b): the kernel finding.
mod common;

use common::{Network, cube_sphere, network_from_fns};
use surfacecraft_kernel::hybrid::Shape;
use surfacecraft_kernel::merge::merge_faces_onto_shape;
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network};
use surfacecraft_kernel::record::brep::{Brep, Surface};
use surfacecraft_kernel::{solid, step, tess};

type Fun = Box<dyn Fn(f64, f64) -> [f64; 3]>;

/// A face `f(s, t)` over the unit square split into `n x n` bilinear patches.
fn split(f: impl Fn(f64, f64) -> [f64; 3] + Copy + 'static, n: usize) -> Vec<Fun> {
    let mut out: Vec<Fun> = Vec::new();
    for i in 0..n {
        for j in 0..n {
            let k = n as f64;
            out.push(Box::new(move |u, v| f((i as f64 + u) / k, (j as f64 + v) / k)));
        }
    }
    out
}

/// A 30 x 30 x 10 box whose six faces are all 3 x 3 patches; faces 0..9 are the top, which
/// bulges by `lift` (0 = flat).
fn box_of_patches(lift: f64) -> Network {
    let top = move |u: f64, v: f64| [30.0 * u, 30.0 * v, 10.0 + lift * u * (1.0 - u) * v * (1.0 - v)];
    let bottom = |u: f64, v: f64| [30.0 * v, 30.0 * u, 0.0];
    let front = |u: f64, v: f64| [30.0 * u, 0.0, 10.0 * v];
    let back = |u: f64, v: f64| [30.0 * v, 30.0, 10.0 * u];
    let left = |u: f64, v: f64| [0.0, 30.0 * v, 10.0 * u];
    let right = |u: f64, v: f64| [30.0, 30.0 * u, 10.0 * v];
    let mut all: Vec<Fun> = split(top, 3);
    all.extend(split(bottom, 3));
    all.extend(split(front, 3));
    all.extend(split(back, 3));
    all.extend(split(left, 3));
    all.extend(split(right, 3));
    let refs: Vec<&dyn Fn(f64, f64) -> [f64; 3]> = all.iter().map(|b| b.as_ref()).collect();
    network_from_fns(&refs, 1)
}

fn kinds(b: &Brep) -> Vec<&'static str> {
    b.faces
        .iter()
        .map(|f| match f.surface {
            Surface::Plane { .. } | Surface::PlaneThrough { .. } => "plane",
            Surface::Cylinder { .. } => "cylinder",
            Surface::Cone { .. } => "cone",
            Surface::Sphere { .. } => "sphere",
            Surface::Spline { .. } | Surface::RationalSpline { .. } => "spline",
            _ => "other",
        })
        .collect()
}

/// The finding: merged body is a valid, closed solid that tessellates, exports and re-imports.
fn assert_sound(before: &Brep, after: &Brep, volume_tol: f64) {
    assert_eq!(is_closed(before), is_closed(after));
    // the record is valid, its order canonical (a face used backward is walked from its other side)
    after.validate().expect("a valid, canonical record");
    let (v0, v1) = (solid::volume(before).unwrap(), solid::volume(after).unwrap());
    assert!((v0 - v1).abs() <= volume_tol * v0, "volume {v0} -> {v1}");
    let mesh = tess::tessellate_all(after, None).expect("tessellates");
    if is_closed(after) {
        // the triangles of every face run outward: their signed volume is the body's
        let signed: f64 = (0..mesh.triangle_count())
            .filter_map(|t| mesh.triangle(t))
            .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
            .sum();
        assert!((signed - v1).abs() <= 5e-3 * v1, "mesh volume {signed}, body volume {v1}");
        // and so do those of the display mesh the viewport and the judge use, at a fine chord
        let display = tess::tessellate(after, Some(0.05)).expect("tessellates");
        let signed: f64 = (0..display.triangle_count())
            .filter_map(|t| display.triangle(t))
            .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
            .sum();
        assert!((signed - v1).abs() <= 5e-3 * v1, "display mesh volume {signed}, body volume {v1}");
    }
    let text = step::export_solids(&[("merged", after)]).expect("exports");
    let back = step::import(text.as_bytes()).expect("re-imports");
    assert_eq!(back.len(), 1);
    assert_eq!(back[0].faces.len(), after.faces.len());
    let mut a = kinds(after);
    let mut b = kinds(&back[0]);
    a.sort_unstable();
    b.sort_unstable();
    assert_eq!(a, b, "surface kinds survive STEP");
    let v2 = solid::volume(&back[0]).unwrap();
    assert!((v2 - v1).abs() <= 1e-6 * v1, "volume after STEP {v1} -> {v2}");
}

#[test]
fn nine_flat_patches_become_one_plane() {
    let net = box_of_patches(0.0);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    assert!(is_closed(&body));
    let shape = Shape::Plane { point: [0.0, 0.0, 10.0], normal: [0.0, 0.0, 1.0] };
    let merged = merge_faces_onto_shape(&body, &(0..9).collect::<Vec<_>>(), &shape, 0.01).unwrap();
    assert_eq!(merged.brep.faces.len(), body.faces.len() - 8);
    assert_eq!(kinds(&merged.brep).iter().filter(|k| **k == "plane").count(), 1);
    assert!(merged.gap < 1e-9, "gap {}", merged.gap);
    assert_sound(&body, &merged.brep, 1e-9);
}

#[test]
fn nearly_flat_patches_become_one_plane_within_their_gap() {
    for lift in [0.0004, 0.004, 0.04] {
        let net = box_of_patches(lift);
        let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
        let shape = Shape::Plane { point: [0.0, 0.0, 10.0], normal: [0.0, 0.0, 1.0] };
        let r = merge_faces_onto_shape(&body, &(0..9).collect::<Vec<_>>(), &shape, 0.1);
        assert!(r.is_ok(), "lift {lift}: {:?}", r.map(|m| m.gap).map_err(|e| e.to_string()));
    }
}

#[test]
fn a_cube_sphere_side_becomes_one_sphere_face() {
    let net = cube_sphere(6, 50.0);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    let group: Vec<usize> = (36..72).collect();
    let shape = Shape::Sphere { center: [0.0; 3], radius: 50.0 };
    let merged = merge_faces_onto_shape(&body, &group, &shape, 0.3).unwrap();
    println!(
        "sphere gap {} kinds {:?}",
        merged.gap,
        kinds(&merged.brep).iter().filter(|k| **k != "spline").collect::<Vec<_>>()
    );
    assert_eq!(kinds(&merged.brep).iter().filter(|k| **k == "sphere").count(), 1);
    assert_sound(&body, &merged.brep, 3e-3);
}

/// An open 3 x 3 sheet of quadratic patches over `f(s, t)`, the unit square split in thirds.
fn sheet(f: impl Fn(f64, f64) -> [f64; 3] + Copy + 'static) -> Network {
    let all = split(f, 3);
    let refs: Vec<&dyn Fn(f64, f64) -> [f64; 3]> = all.iter().map(|b| b.as_ref()).collect();
    network_from_fns(&refs, 2)
}

/// Whether the shell uses `face` along its natural normal.
fn used_forward(b: &Brep, face: usize) -> Option<bool> {
    b.shells.iter().find_map(|s| match s {
        surfacecraft_kernel::record::brep::Shell::Faces { faces, .. } => {
            faces.iter().find(|o| o.face as usize == face).map(|o| o.forward)
        }
        _ => None,
    })
}

/// A patch of an open sheet becomes one analytic face of the right kind: the open scan of gate 2.
fn merge_sheet(net: &Network, shape: &Shape, kind: &str, gap: f64) {
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    assert!(!is_closed(&body));
    let merged = merge_faces_onto_shape(&body, &(0..9).collect::<Vec<_>>(), shape, 0.05).unwrap();
    println!("{kind}: gap {}", merged.gap);
    assert!(merged.gap < gap, "gap {}", merged.gap);
    assert_eq!(merged.brep.faces.len(), 1);
    assert_eq!(kinds(&merged.brep), vec![kind]);
    assert!(!is_closed(&merged.brep));
    tess::tessellate_all(&merged.brep, None).expect("tessellates");
    // the record survives serde (the project file) and the kernel
    let json = serde_json::to_string(&merged.brep).unwrap();
    let back: Brep = serde_json::from_str(&json).unwrap();
    assert_eq!(kinds(&back), vec![kind]);
    tess::tessellate_all(&back, None).expect("tessellates again");
}

#[test]
fn an_open_cylinder_sheet_becomes_one_cylinder_face() {
    let (r, a0, a1) = (10.0_f64, 1.0_f64, 2.2_f64);
    let net = sheet(move |s, t| {
        let a = a0 + (a1 - a0) * s;
        [r * a.cos(), r * a.sin(), 20.0 * t]
    });
    merge_sheet(&net, &Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: r }, "cylinder", 1e-3);
}

/// The same sheets seen from the axis (their normals point inward): the kernel keeps only the `same` use of
/// a sheet's faces, so a merged face that looks against its surface's natural normal would come back
/// reversed (measured on a real scan: every normal of such faces against the patches' ones). Such a group
/// is refused, with the reason; the same group in a closed body is merged
/// (`a_concave_cylinder_wall_becomes_one_cylinder_face_used_backward`).
#[test]
fn a_concave_face_is_not_merged_into_an_open_sheet() {
    let (r, a0, a1) = (10.0_f64, 1.0_f64, 2.2_f64);
    let cyl = sheet(move |s, t| {
        let a = a0 + (a1 - a0) * t;
        [r * a.cos(), r * a.sin(), 20.0 * s]
    });
    let (k, b0, b1) = (0.5_f64, 0.5_f64, 1.7_f64);
    let cone = sheet(move |s, t| {
        let (a, h) = (b0 + (b1 - b0) * t, 10.0 + 10.0 * s);
        [k * h * a.cos(), k * h * a.sin(), h]
    });
    let (rs, c0, c1) = (20.0_f64, 0.3_f64, 1.1_f64);
    let ball = sheet(move |s, t| {
        let (a, b) = (c0 + (c1 - c0) * t, -0.4 + 0.8 * s);
        [rs * b.cos() * a.cos(), rs * b.cos() * a.sin(), rs * b.sin()]
    });
    let shapes = [
        (cyl, Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: r }),
        (cone, Shape::Cone { apex: [0.0; 3], axis: [0.0, 0.0, 1.0], half_angle: k.atan() }),
        (ball, Shape::Sphere { center: [0.0; 3], radius: rs }),
    ];
    for (net, shape) in shapes {
        let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
        let why = merge_faces_onto_shape(&body, &(0..9).collect::<Vec<_>>(), &shape, 0.05).unwrap_err().to_string();
        assert!(why.contains("open sheet"), "{why}");
    }
}

#[test]
fn an_open_cone_sheet_becomes_one_cone_face() {
    let (k, a0, a1) = (0.5_f64, 0.5_f64, 1.7_f64);
    let net = sheet(move |s, t| {
        let a = a0 + (a1 - a0) * s;
        let h = 10.0 + 10.0 * t;
        [k * h * a.cos(), k * h * a.sin(), h]
    });
    let shape = Shape::Cone { apex: [0.0; 3], axis: [0.0, 0.0, 1.0], half_angle: k.atan() };
    merge_sheet(&net, &shape, "cone", 1e-3);
}

#[test]
fn an_open_sphere_sheet_becomes_one_sphere_face() {
    let (r, a0, a1) = (20.0_f64, 0.3_f64, 1.1_f64);
    let net = sheet(move |s, t| {
        let (a, b) = (a0 + (a1 - a0) * s, -0.4 + 0.8 * t);
        [r * b.cos() * a.cos(), r * b.cos() * a.sin(), r * b.sin()]
    });
    merge_sheet(&net, &Shape::Sphere { center: [0.0; 3], radius: r }, "sphere", 1e-3);
}

/// A group with a hole (the middle patch left out) becomes a face with an inner loop.
#[test]
fn a_ring_of_patches_becomes_a_face_with_a_hole() {
    let net = sheet(|s, t| [30.0 * s, 30.0 * t, 0.0]);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    let ring: Vec<usize> = (0..9).filter(|&f| f != 4).collect();
    let shape = Shape::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] };
    let merged = merge_faces_onto_shape(&body, &ring, &shape, 0.05).unwrap();
    assert_eq!(merged.brep.faces.len(), 2, "the face and the middle patch");
    let loops = merged.brep.faces.iter().map(|f| f.loops.len()).max();
    assert_eq!(loops, Some(2), "an outer loop and the hole");
    tess::tessellate_all(&merged.brep, None).expect("tessellates");
    let area: f64 = tess::tessellate_all(&merged.brep, None).unwrap().area();
    assert!((area - 900.0).abs() < 1e-6, "the face and the patch cover the square: {area}");
}

/// Faces that touch only at a corner, a repeated face and a gap over the limit are refused, and nothing
/// is changed.
#[test]
fn groups_the_kernel_cannot_merge_are_refused() {
    let net = sheet(|s, t| [30.0 * s, 30.0 * t, 0.0]);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    let plane = Shape::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] };
    assert!(merge_faces_onto_shape(&body, &[0, 4], &plane, 0.05).is_err(), "corner to corner");
    assert!(merge_faces_onto_shape(&body, &[1, 1], &plane, 0.05).is_err(), "a repeated face");
    assert!(merge_faces_onto_shape(&body, &[], &plane, 0.05).is_err(), "an empty group");
    assert!(merge_faces_onto_shape(&body, &[0, 99], &plane, 0.05).is_err(), "a face out of range");
    let tilted = Shape::Plane { point: [0.0, 0.0, 1.0], normal: [0.0, 0.0, 1.0] };
    assert!(merge_faces_onto_shape(&body, &[0, 1], &tilted, 0.05).is_err(), "a gap of 1 mm over a limit of 0.05");
    for bad in [0.0, f64::NAN, -1.0, 10.0] {
        assert!(merge_faces_onto_shape(&body, &[0, 1], &plane, bad).is_err(), "gap {bad}");
    }
    let nan = Shape::Plane { point: [f64::NAN; 3], normal: [0.0, 0.0, 1.0] };
    assert!(merge_faces_onto_shape(&body, &[0, 1], &nan, 0.05).is_err());
    for radius in [f64::NAN, f64::INFINITY, -1.0, 0.0] {
        let cylinder = Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius };
        assert!(merge_faces_onto_shape(&body, &[0, 1], &cylinder, 0.05).is_err(), "radius {radius}");
    }
    let skew = Shape::Cylinder { point: [0.0; 3], axis: [f64::NAN, 0.0, 1.0], radius: 5.0 };
    assert!(merge_faces_onto_shape(&body, &[0, 1], &skew, 0.05).is_err(), "a NaN axis");
    let blunt = Shape::Cone { apex: [0.0; 3], axis: [0.0, 0.0, 1.0], half_angle: f64::NAN };
    assert!(merge_faces_onto_shape(&body, &[0, 1], &blunt, 0.05).is_err(), "a NaN half angle");
    let torus = Shape::Torus { center: [0.0; 3], axis: [0.0, 0.0, 1.0], major_radius: 20.0, minor_radius: 5.0 };
    assert!(merge_faces_onto_shape(&body, &[0, 1], &torus, 0.05).is_err(), "a torus is not placed");
}

/// A curved slab: the part of the annulus `r..big` over the angles `a0..a1`, height 20, every face a
/// 3 x 3 patch network. Faces 0..9 are the outer wall (convex), 9..18 the inner wall (concave: the
/// body's outside is the axis side), then the two radial walls, the top and the bottom.
fn curved_slab(r: f64, big: f64, a0: f64, a1: f64) -> Network {
    let h = 20.0;
    let ang = move |s: f64| a0 + (a1 - a0) * s;
    let rho = move |t: f64| r + (big - r) * t;
    let outer = move |s: f64, t: f64| [big * ang(s).cos(), big * ang(s).sin(), h * t];
    let inner = move |s: f64, t: f64| [r * ang(s).cos(), r * ang(s).sin(), h * t];
    let wall0 = move |s: f64, t: f64| [rho(s) * a0.cos(), rho(s) * a0.sin(), h * t];
    let wall1 = move |s: f64, t: f64| [rho(s) * a1.cos(), rho(s) * a1.sin(), h * t];
    let top = move |s: f64, t: f64| [rho(t) * ang(s).cos(), rho(t) * ang(s).sin(), h];
    let bottom = move |s: f64, t: f64| [rho(t) * ang(s).cos(), rho(t) * ang(s).sin(), 0.0];
    let mut all: Vec<Fun> = split(outer, 3);
    all.extend(split(inner, 3));
    all.extend(split(wall0, 3));
    all.extend(split(wall1, 3));
    all.extend(split(top, 3));
    all.extend(split(bottom, 3));
    let refs: Vec<&dyn Fn(f64, f64) -> [f64; 3]> = all.iter().map(|b| b.as_ref()).collect();
    network_from_fns(&refs, 2)
}

/// The concave wall of a bore: its shell use runs against the surface's natural normal, the volume and
/// the STEP round trip are those of the patches.
#[test]
fn a_concave_cylinder_wall_becomes_one_cylinder_face_used_backward() {
    let (r, big) = (10.0_f64, 14.0_f64);
    let net = curved_slab(r, big, 1.0, 2.2);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    assert!(is_closed(&body));
    let shape = Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: r };
    let merged = merge_faces_onto_shape(&body, &(9..18).collect::<Vec<_>>(), &shape, 0.05).unwrap();
    assert_eq!(merged.brep.faces.len(), body.faces.len() - 8);
    assert_eq!(kinds(&merged.brep).iter().filter(|k| **k == "cylinder").count(), 1);
    assert_sound(&body, &merged.brep, 2e-3);
    assert_eq!(
        used_forward(&merged.brep, merged.face),
        Some(false),
        "the bore's wall is used against its natural normal"
    );
}

/// A 30 x 30 x 10 box whose top face (faces 0..9, 3 x 3 patches) is the saddle
/// `z = 10 + rise (1 + (x / 30 - 1/2)(y / 30 - 1/2))` plus a bump `16 bump u (1 - u) v (1 - v)` (zero on the
/// border), so that its whole border is between 0.75 and 1.25 `rise` off the plane z = 10; the walls follow it.
fn box_with_bumped_top(rise: f64, bump: f64) -> Network {
    let z = move |x: f64, y: f64| {
        let (u, v) = (x / 30.0, y / 30.0);
        10.0 + rise * (1.0 + (u - 0.5) * (v - 0.5)) + 16.0 * bump * u * (1.0 - u) * v * (1.0 - v)
    };
    let top = move |u: f64, v: f64| [30.0 * u, 30.0 * v, z(30.0 * u, 30.0 * v)];
    let bottom = |u: f64, v: f64| [30.0 * v, 30.0 * u, 0.0];
    let front = move |u: f64, v: f64| [30.0 * u, 0.0, v * z(30.0 * u, 0.0)];
    let back = move |u: f64, v: f64| [30.0 * v, 30.0, u * z(30.0 * v, 30.0)];
    let left = move |u: f64, v: f64| [0.0, 30.0 * v, u * z(0.0, 30.0 * v)];
    let right = move |u: f64, v: f64| [30.0, 30.0 * u, v * z(30.0, 30.0 * u)];
    let mut all: Vec<Fun> = split(top, 3);
    all.extend(split(bottom, 3));
    all.extend(split(front, 3));
    all.extend(split(back, 3));
    all.extend(split(left, 3));
    all.extend(split(right, 3));
    let refs: Vec<&dyn Fn(f64, f64) -> [f64; 3]> = all.iter().map(|b| b.as_ref()).collect();
    network_from_fns(&refs, 2)
}

fn box_with_saddle_top(rise: f64) -> Network {
    box_with_bumped_top(rise, 0.0)
}

/// A group whose border is off its plane by up to 0.05 mm gets a border on the plane (the gap is what the
/// least-squares fit of a cubic leaves, nothing here), and the walls that share the border move with it:
/// the body stays a watertight solid of nearly the volume it had.
#[test]
fn the_border_of_a_group_is_put_on_its_primitive_and_its_neighbours_follow() {
    let net = box_with_saddle_top(0.04);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    assert!(is_closed(&body));
    let shape = Shape::Plane { point: [0.0, 0.0, 10.0], normal: [0.0, 0.0, 1.0] };
    let merged = merge_faces_onto_shape(&body, &(0..9).collect::<Vec<_>>(), &shape, 0.1).unwrap();
    assert!(merged.gap < 1e-6, "the border is on the plane: {}", merged.gap);
    assert_eq!(merged.brep.faces.len(), body.faces.len() - 8);
    assert_sound(&body, &merged.brep, 0.01);
    // the 12 vertices on the border of the top moved onto the plane (the walls' top corners with them)
    let on_plane = merged.brep.vertices.iter().filter(|v| (v.point[2] - 10.0).abs() < 1e-9).count();
    assert_eq!(on_plane, 12, "the vertices of the border");
}

/// The same on a curved primitive: a cylinder wall whose border is 0.04 mm off the cylinder.
#[test]
fn the_border_of_a_cylinder_group_is_put_on_the_cylinder() {
    let (r, big) = (10.0_f64, 14.0_f64);
    let net = curved_slab(r + 0.04, big, 1.0, 2.2);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    let shape = Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: r };
    let merged = merge_faces_onto_shape(&body, &(9..18).collect::<Vec<_>>(), &shape, 0.1).unwrap();
    assert!(merged.gap < 0.1 * 0.1, "{}", merged.gap);
    assert_sound(&body, &merged.brep, 0.01);
}

/// A ring: the annulus `r..big` over the whole turn, height 20, in `n` angular steps. Faces `0..n` are the
/// outer wall (convex), `n..2n` the inner wall (a bore: concave), then the top and the bottom annulus.
fn ring_body(r: f64, big: f64, n: usize) -> Network {
    let da = std::f64::consts::TAU / n as f64;
    let mut all: Vec<Fun> = Vec::new();
    let wall = |radius: f64, i: usize| -> Fun {
        Box::new(move |u, v| {
            let a = (i as f64 + u) * da;
            [radius * a.cos(), radius * a.sin(), 20.0 * v]
        })
    };
    let annulus = |z: f64, i: usize| -> Fun {
        Box::new(move |u, v| {
            let (a, rho) = ((i as f64 + u) * da, r + (big - r) * v);
            [rho * a.cos(), rho * a.sin(), z]
        })
    };
    all.extend((0..n).map(|i| wall(big, i)));
    all.extend((0..n).map(|i| wall(r, i)));
    all.extend((0..n).map(|i| annulus(20.0, i)));
    all.extend((0..n).map(|i| annulus(0.0, i)));
    let refs: Vec<&dyn Fn(f64, f64) -> [f64; 3]> = all.iter().map(|b| b.as_ref()).collect();
    network_from_fns(&refs, 2)
}

/// A wall that goes all the way round (a boss or a bore) is cut into sectors, each one face on the
/// cylinder; the bore's faces are used against their natural normal. The body stays a sound solid.
#[test]
fn walls_that_go_round_the_axis_become_a_face_per_sector() {
    use surfacecraft_kernel::merge::{GroupSpec, merge_faces_onto_shapes, split_round};
    let (r, big, n) = (10.0_f64, 14.0_f64, 24);
    let net = ring_body(r, big, n);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    assert!(is_closed(&body));
    for (first, radius, concave) in [(0, big, false), (n, r, true)] {
        let shape = Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius };
        let wall: Vec<usize> = (first..first + n).collect();
        let sectors = split_round(&body, &wall, &shape);
        assert_eq!(sectors.len(), 3, "a turn is cut in three");
        assert!(sectors.iter().all(|g| g.len() == n / 3));
        let specs: Vec<GroupSpec> = sectors.iter().map(|g| GroupSpec { faces: g, shape: &shape }).collect();
        let batch = merge_faces_onto_shapes(&body, &specs, 0.05, &|| false).unwrap();
        let why: Vec<String> = batch.groups.iter().filter_map(|g| g.as_ref().err().cloned()).collect();
        assert!(why.is_empty(), "{why:?}");
        assert_eq!(kinds(&batch.brep).iter().filter(|k| **k == "cylinder").count(), 3);
        assert_sound(&body, &batch.brep, 2e-3);
        let backward =
            batch.groups.iter().flatten().filter(|g| used_forward(&batch.brep, g.face) == Some(false)).count();
        assert_eq!(backward, if concave { 3 } else { 0 });
    }
}

/// A group that leaves a gap in angle is not cut, and planes and spheres are never cut.
#[test]
fn an_arc_and_a_plane_are_not_cut_into_sectors() {
    use surfacecraft_kernel::merge::split_round;
    let net = curved_slab(10.0, 14.0, 1.0, 2.2);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    let wall: Vec<usize> = (9..18).collect();
    let cyl = Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: 10.0 };
    assert_eq!(split_round(&body, &wall, &cyl), vec![wall.clone()]);
    let plane = Shape::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] };
    assert_eq!(split_round(&body, &wall, &plane), vec![wall.clone()]);
    let sphere = Shape::Sphere { center: [0.0; 3], radius: 10.0 };
    assert_eq!(split_round(&body, &wall, &sphere), vec![wall]);
}

/// The boundary lists its loops by edge number, which says nothing of which is the outer one: with the
/// hole's patch numbered first the hole's edges come first, and the face still has the hole inside it.
#[test]
fn a_hole_numbered_before_the_outer_border_is_still_a_hole() {
    let mut all = split(|s, t| [30.0 * s, 30.0 * t, 0.0], 3);
    all.swap(0, 4);
    let refs: Vec<&dyn Fn(f64, f64) -> [f64; 3]> = all.iter().map(|b| b.as_ref()).collect();
    let net = network_from_fns(&refs, 2);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    let ring: Vec<usize> = (1..9).collect();
    let shape = Shape::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] };
    let merged = merge_faces_onto_shape(&body, &ring, &shape, 0.05).unwrap();
    let face = &merged.brep.faces[merged.face];
    assert_eq!(face.loops.len(), 2);
    let area: f64 = tess::tessellate_all(&merged.brep, None).unwrap().area();
    assert!((area - 900.0).abs() < 1e-6, "the ring and the middle patch cover the square: {area}");
}

/// Two patches of a concave wall (a part of it, the rest stays patches) become one face used against
/// its natural normal; the mesh of the whole body stays outward.
#[test]
fn a_part_of_a_concave_wall_is_merged_with_the_right_sense() {
    let (r, big) = (10.0_f64, 14.0_f64);
    let net = curved_slab(r, big, 1.0, 2.2);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    let shape = Shape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: r };
    for group in [vec![9, 10], vec![13, 14, 16], vec![9, 10, 11, 12]] {
        let merged = merge_faces_onto_shape(&body, &group, &shape, 0.05).unwrap();
        assert_eq!(used_forward(&merged.brep, merged.face), Some(false), "{group:?}");
        assert_sound(&body, &merged.brep, 2e-3);
    }
}

/// A row of bilinear patches along x (y from 0 to 10), column `i` at `xs[i]` and height `zs[i]`; patch `k`
/// lies between columns `k` and `k + 1`, its normal up. Vertex `(i, j)` is `2 i + j`.
fn row_sheet(xs: &[f64], zs: &[f64]) -> Brep {
    let pt = |i: usize, j: usize| [xs[i], 10.0 * j as f64, zs[i]];
    let nets: Vec<Vec<Vec<[f64; 3]>>> =
        (0..xs.len() - 1).map(|i| vec![vec![pt(i, 0), pt(i, 1)], vec![pt(i + 1, 0), pt(i + 1, 1)]]).collect();
    let id = |i: usize, j: usize| (2 * i + j) as u32;
    let corners: Vec<[u32; 4]> = (0..xs.len() - 1).map(|i| [id(i, 0), id(i + 1, 0), id(i + 1, 1), id(i, 1)]).collect();
    sheet_from_bezier_network(&nets, &corners).unwrap()
}

fn plane_through(point: [f64; 3], normal: [f64; 3]) -> Shape {
    Shape::Plane { point, normal }
}

/// Two planes that meet at about 10 degrees (z = 0 and z = tan(10 deg) (x - 10)) and a fold between two groups of
/// patches whose vertices are within the gap of both planes but 1.1 mm from their line of intersection: moving
/// the fold onto that line would drag the patches (and, in a scan, the neighbours' border rows) by several
/// tolerances. Neither group is snapped, and both are merged as they were, with the reason said.
#[test]
fn a_vertex_that_would_move_several_gaps_onto_two_primitives_is_not_moved() {
    use surfacecraft_kernel::merge::{GroupSpec, merge_faces_onto_shapes};
    let t = 10.0_f64.to_radians().tan();
    let zb = |x: f64| t * (x - 10.0);
    let (xs, zs) = ([0.0, 5.57, 11.14, 16.57, 22.0], [0.0, 0.05, 0.1, zb(16.57), zb(22.0)]);
    let body = row_sheet(&xs, &zs);
    let (a, b) = ([0usize, 1], [2usize, 3]);
    let (pa, pb) = (plane_through([0.0; 3], [0.0, 0.0, 1.0]), plane_through([10.0, 0.0, 0.0], [-t, 0.0, 1.0]));
    let specs = [GroupSpec { faces: &a, shape: &pa }, GroupSpec { faces: &b, shape: &pb }];
    let batch = merge_faces_onto_shapes(&body, &specs, 0.15, &|| false).unwrap();
    for g in &batch.groups {
        let g = g.as_ref().expect("merged as it was");
        let note = g.snap_note.as_deref().expect("says why it was not snapped");
        assert!(note.contains("move more than"), "{note}");
    }
    // the fold is where the scan has it
    let fold: Vec<f64> =
        batch.brep.vertices.iter().filter(|v| (v.point[0] - 11.14).abs() < 0.5).map(|v| v.point[2]).collect();
    assert_eq!(fold.len(), 2);
    assert!(fold.iter().all(|z| (z - 0.1).abs() < 1e-9), "{fold:?}");
    assert_eq!(kinds(&batch.brep).iter().filter(|k| **k == "plane").count(), 2);
}

/// The same groups with the fold within reach of the intersection (the planes meet 0.05 mm from it): the
/// vertices go onto it.
#[test]
fn a_vertex_within_reach_of_two_primitives_goes_onto_their_intersection() {
    use surfacecraft_kernel::merge::{GroupSpec, merge_faces_onto_shapes};
    let t = 10.0_f64.to_radians().tan();
    let zb = |x: f64| t * (x - 10.0);
    // the fold at x = 10.2, z = 0.03: 0.2 mm and 0.03 mm from the planes' line
    let (xs, zs) = ([0.0, 5.0, 10.2, 16.0, 22.0], [0.0, 0.01, 0.03, zb(16.0), zb(22.0)]);
    let body = row_sheet(&xs, &zs);
    let (a, b) = ([0usize, 1], [2usize, 3]);
    let (pa, pb) = (plane_through([0.0; 3], [0.0, 0.0, 1.0]), plane_through([10.0, 0.0, 0.0], [-t, 0.0, 1.0]));
    let specs = [GroupSpec { faces: &a, shape: &pa }, GroupSpec { faces: &b, shape: &pb }];
    let batch = merge_faces_onto_shapes(&body, &specs, 0.15, &|| false).unwrap();
    assert!(batch.groups.iter().all(|g| g.as_ref().is_ok_and(|g| g.snap_note.is_none())), "{:?}", batch.groups);
    let fold: Vec<[f64; 3]> =
        batch.brep.vertices.iter().filter(|v| (v.point[0] - 10.0).abs() < 0.001).map(|v| v.point).collect();
    assert_eq!(fold.len(), 2, "the fold went to x = 10, z = 0: {:?}", batch.brep.vertices);
    assert!(fold.iter().all(|p| p[2].abs() < 1e-4), "{fold:?}");
}

/// One edge that is not a Bézier span (two spans) at the border of one group turns the snap off for that group
/// only: the other group is snapped, the first is merged as it was and the reason is reported.
#[test]
fn an_edge_that_is_not_a_bezier_span_limits_the_failure_to_its_groups() {
    use surfacecraft_kernel::merge::{GroupSpec, merge_faces_onto_shapes};
    use surfacecraft_kernel::record::brep::Curve;
    let xs = [0.0, 5.0, 10.0, 15.0, 20.0, 25.0];
    let zs = [0.03, -0.02, 0.02, 0.03, -0.03, 0.02];
    let mut body = row_sheet(&xs, &zs);
    // the edge between vertices (3, 0) and (3, 1): the left border of the second group, split in two spans
    let e = body.edges.iter().position(|e| e.vertices == [6, 7] || e.vertices == [7, 6]).unwrap();
    let (p, q) = (body.vertices[6].point, body.vertices[7].point);
    let mid = [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0, (p[2] + q[2]) / 2.0];
    body.edges[e].curve = Curve::Spline { knots: vec![0.0, 0.0, 0.5, 1.0, 1.0], points: vec![p, mid, q] };
    let (first, second) = ([0usize, 1], [3usize, 4]);
    let flat = plane_through([0.0; 3], [0.0, 0.0, 1.0]);
    let specs = [GroupSpec { faces: &first, shape: &flat }, GroupSpec { faces: &second, shape: &flat }];
    let batch = merge_faces_onto_shapes(&body, &specs, 0.1, &|| false).unwrap();
    let notes: Vec<Option<String>> =
        batch.groups.iter().map(|g| g.as_ref().expect("merged").snap_note.clone()).collect();
    assert!(notes[0].is_none(), "the group without the edge is snapped: {notes:?}");
    assert!(notes[1].as_deref().is_some_and(|n| n.contains("Bézier")), "{notes:?}");
    let z_of = |x: f64| -> Vec<f64> {
        batch.brep.vertices.iter().filter(|v| (v.point[0] - x).abs() < 1e-9).map(|v| v.point[2]).collect()
    };
    assert!(z_of(0.0).iter().all(|z| z.abs() < 1e-9), "the first group's vertices are on the plane: {:?}", z_of(0.0));
    assert!(z_of(20.0).iter().any(|z| (z + 0.03).abs() < 1e-9), "the second group's stay: {:?}", z_of(20.0));
}

/// A group whose face the checks refuse after its border was snapped (its middle bulges 0.3 mm over a plane
/// that its border is 0.075 mm from) leaves the body as the wrap had it: the walls that shared its border
/// are exactly as they were, and the body is the one the other group alone gives.
#[test]
fn a_group_that_is_refused_after_its_snap_leaves_its_neighbours_as_they_were() {
    use surfacecraft_kernel::merge::{GroupSpec, merge_faces_onto_shapes};
    let net = box_with_bumped_top(0.06, 0.3);
    let body = sheet_from_bezier_network(&net.patches, &net.corners).unwrap();
    assert!(is_closed(&body));
    let (top, front): (Vec<usize>, Vec<usize>) = ((0..9).collect(), (18..27).collect());
    let (ptop, pfront) = (plane_through([0.0, 0.0, 10.0], [0.0, 0.0, 1.0]), plane_through([0.0; 3], [0.0, -1.0, 0.0]));
    let both = [GroupSpec { faces: &top, shape: &ptop }, GroupSpec { faces: &front, shape: &pfront }];
    let batch = merge_faces_onto_shapes(&body, &both, 0.1, &|| false).unwrap();
    assert!(batch.groups[0].is_err(), "the bulging top is refused: {:?}", batch.groups[0]);
    assert!(batch.groups[1].is_ok(), "{:?}", batch.groups[1]);
    let alone = merge_faces_onto_shapes(&body, &both[1..], 0.1, &|| false).unwrap();
    assert!(alone.groups[0].is_ok());
    assert_eq!(batch.brep, alone.brep, "the refused group's snap was taken back");
    for f in &top {
        let k = batch.face_map[*f].expect("the top patches stay");
        let (Surface::Spline { points: now, .. }, Surface::Spline { points: was, .. }) =
            (&batch.brep.faces[k].surface, &body.faces[*f].surface)
        else {
            panic!("patches")
        };
        let off = now
            .iter()
            .flatten()
            .zip(was.iter().flatten())
            .map(|(p, q)| (0..3).map(|d| (p[d] - q[d]).abs()).fold(0.0, f64::max));
        assert!(off.fold(0.0, f64::max) < 1e-9, "patch {f} is as the wrap had it");
    }
}

/// A patch with a collapsed side (a triangle: two of its corners at one point, with their own vertices) next to
/// a group: when the shared vertex moves onto the plane, the collapsed side moves with it, its interior control
/// point included, and the body still builds.
#[test]
fn a_patch_with_a_collapsed_side_moves_with_its_vertex() {
    use surfacecraft_kernel::record::brep::Surface;
    let lerp = |a: [f64; 3], b: [f64; 3], t: f64| {
        [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t]
    };
    // quadratic nets of the bilinear patch through p00, p01 (along v), p10 (along u), p11
    let net = |p00: [f64; 3], p01: [f64; 3], p10: [f64; 3], p11: [f64; 3]| -> Vec<Vec<[f64; 3]>> {
        (0..3)
            .map(|i| {
                (0..3)
                    .map(|j| lerp(lerp(p00, p10, i as f64 / 2.0), lerp(p01, p11, i as f64 / 2.0), j as f64 / 2.0))
                    .collect()
            })
            .collect()
    };
    let corner = |i: usize, j: usize| [4.0 * i as f64, 4.0 * j as f64, if (i, j) == (1, 1) { 0.03 } else { 0.0 }];
    let id = |i: usize, j: usize| (i * 2 + j) as u32;
    let mut nets = Vec::new();
    let mut corners = Vec::new();
    for i in 0..2 {
        nets.push(net(corner(i, 0), corner(i, 1), corner(i + 1, 0), corner(i + 1, 1)));
        corners.push([id(i, 0), id(i + 1, 0), id(i + 1, 1), id(i, 1)]);
    }
    // the triangle (4, 4), (8, 4), (8, 8) over the top edge of patch 1; its fourth corner d (vertex 6 + 1) is at
    // (4, 4) too, so its side from d to (1, 1) is collapsed
    let (p0, p1, p2) = (corner(1, 1), corner(2, 1), [8.0, 8.0, 0.0]);
    nets.push(net(p0, p0, p1, p2));
    corners.push([id(1, 1), id(2, 1), 6, 7]);
    let body = sheet_from_bezier_network(&nets, &corners).unwrap();
    let flat = plane_through([0.0; 3], [0.0, 0.0, 1.0]);
    let merged = merge_faces_onto_shape(&body, &[0, 1], &flat, 0.1).expect("merged");
    let q = merged.face_map[2].expect("the triangle stays");
    let Surface::Spline { points, .. } = &merged.brep.faces[q].surface else { panic!("a patch") };
    let heights: Vec<f64> = points.iter().flatten().map(|p| p[2]).collect();
    assert!(
        heights.iter().any(|z| z.abs() < 1e-9),
        "the corner at the group's vertex moved onto the plane: {heights:?}"
    );
    assert!(points.iter().flatten().all(|p| p[2] <= 0.03 + 1e-9), "{heights:?}");
    assert!(tess::tessellate_all(&merged.brep, None).is_ok());
}
