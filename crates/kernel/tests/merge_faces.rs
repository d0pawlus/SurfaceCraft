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
    let (v0, v1) = (solid::volume(before).unwrap(), solid::volume(after).unwrap());
    assert!((v0 - v1).abs() <= volume_tol * v0, "volume {v0} -> {v1}");
    tess::tessellate_all(after, None).expect("tessellates");
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
    let torus = Shape::Torus { center: [0.0; 3], axis: [0.0, 0.0, 1.0], major_radius: 20.0, minor_radius: 5.0 };
    assert!(merge_faces_onto_shape(&body, &[0, 1], &torus, 0.05).is_err(), "a torus is not placed");
}
