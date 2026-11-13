//! P9-O6b gates: `scan.to_solid` with the hybrid method, which wraps the scan in patches and merges the
//! patches that lie on a primitive into one analytic face each. The part is a body of revolution whose
//! plane, cylinder and sphere blend smoothly into each other (no sharp edge): a flat base, a round, a
//! cylindrical wall, a round and a spherical cap, scanned with noise. Assertions are properties with stated
//! margins (which patches form a group depends on the patch layout, which differs a little between
//! platforms): the faces of the right kind at the right place, the share of the area, the deviation.
use serde_json::{Value, json};
use std::f64::consts::{FRAC_PI_2, TAU};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, synth};
use surfacecraft_kernel::record::brep::{Brep, Surface};

const RADIUS: f64 = 8.0;
const ROUND: f64 = 2.0;
/// The wall runs from the lower round to this height, where the upper round starts.
const WALL_TOP: f64 = 14.0;
/// The cap: a sphere of this radius centred on the axis at this height; its round to the wall is
/// tangent to both, which puts to the wall (see `profile`).
const CAP_RADIUS: f64 = 12.0;
const CAP_CENTRE: f64 = 6.0;
const NOISE: f64 = 0.03;
/// How far the analytic faces may be from the part's plane, cylinder and sphere (mm): the noise is 0.03.
const GEOMETRY_TOLERANCE: f64 = 0.1;

/// The profile of the part from the centre of the base to the pole of the cap: (distance from the axis, height).
fn profile() -> Vec<(f64, f64)> {
    let mut p = vec![(0.0, 0.0)];
    let base = RADIUS - ROUND;
    let mut r = 1.0;
    while r < base {
        p.push((r, 0.0));
        r += 1.0;
    }
    for k in 0..=8 {
        let a = -FRAC_PI_2 + FRAC_PI_2 * f64::from(k) / 8.0;
        p.push((base + ROUND * a.cos(), ROUND + ROUND * a.sin()));
    }
    let mut z = ROUND + 1.0;
    while z < WALL_TOP {
        p.push((RADIUS, z));
        z += 1.0;
    }
    // the upper round: tangent to the wall at (RADIUS, WALL_TOP) and inside the cap's sphere, its
    // centre on the line from the sphere's centre
    let centre = (RADIUS - ROUND, WALL_TOP);
    let toward = (centre.0, centre.1 - CAP_CENTRE);
    let reach = toward.0.hypot(toward.1);
    let turn = (toward.1).atan2(toward.0);
    for k in 0..=8 {
        let a = turn * f64::from(k) / 8.0;
        p.push((centre.0 + ROUND * a.cos(), centre.1 + ROUND * a.sin()));
    }
    debug_assert!((reach + ROUND - CAP_RADIUS).abs() < 1e-9, "the round is tangent to the cap");
    let start = turn;
    for k in 1..24 {
        let a = start + (FRAC_PI_2 - start) * f64::from(k) / 24.0;
        p.push((CAP_RADIUS * a.cos(), CAP_CENTRE + CAP_RADIUS * a.sin()));
    }
    p.push((0.0, CAP_CENTRE + CAP_RADIUS));
    p
}

/// The part as a closed mesh: the profile turned about the z axis.
fn part_mesh() -> Mesh {
    const STEPS: usize = 120;
    let prof = profile();
    let mut m = Mesh::default();
    let mut ring_start = Vec::new();
    for &(rho, z) in &prof {
        ring_start.push(m.positions.len() as u32);
        if rho < 1e-9 {
            m.positions.push(Point3::new(0.0, 0.0, z));
        } else {
            for i in 0..STEPS {
                let a = TAU * i as f64 / STEPS as f64;
                m.positions.push(Point3::new(rho * a.cos(), rho * a.sin(), z));
            }
        }
    }
    for (k, w) in ring_start.windows(2).enumerate() {
        let (a, b) = (w[0], w[1]);
        let (pa, pb) = (prof[k].0 < 1e-9, prof[k + 1].0 < 1e-9);
        for i in 0..STEPS as u32 {
            let j = (i + 1) % STEPS as u32;
            let (a0, a1) = if pa { (a, a) } else { (a + i, a + j) };
            let (b0, b1) = if pb { (b, b) } else { (b + i, b + j) };
            if !pa {
                m.triangles.push([a0, b0, a1]);
            }
            if !pb {
                m.triangles.push([a1, b0, b1]);
            }
        }
    }
    // the triangles run outward when the signed volume is positive
    let volume: f64 = (0..m.triangle_count())
        .filter_map(|t| m.triangle(t))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum();
    if volume < 0.0 {
        for t in &mut m.triangles {
            t.swap(1, 2);
        }
    }
    m
}

/// A session holding the noisy scan "scan" of the part (or of `mesh`).
fn scanned_session(mesh: &Mesh, seed: u64) -> Session {
    let mut s = Session::new();
    let mut cad = mesh.clone();
    cad.weld(1e-6);
    let d = synth::ScanDefects { spacing: 0.7, noise: NOISE, holes: 0, spikes: 0.0, debris: 0, ..Default::default() };
    s.doc.add_mesh("scan", synth::scanify(&cad, &d, &mut Rng::new(seed)));
    s
}

fn to_solid(s: &mut Session, method: &str) -> Value {
    s.execute("scan.to_solid", json!({"scan": "scan", "method": method, "tolerance": 0.2, "name": method})).unwrap()
}

/// [`to_solid`] that leaves the holes of the scan open.
fn to_solid_open(s: &mut Session, method: &str) -> Value {
    let p = json!({"scan": "scan", "method": method, "tolerance": 0.2, "name": method, "fill_holes_max_edges": 10});
    s.execute("scan.to_solid", p).unwrap()
}

fn body_of(s: &Session, r: &Value) -> Brep {
    let id = ObjectId(r["body"]["id"].as_u64().expect("a body"));
    (*s.doc.body(id).expect("the body is in the document").brep).clone()
}

fn unit(v: [f64; 3]) -> [f64; 3] {
    let n = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    [v[0] / n, v[1] / n, v[2] / n]
}

fn cross(a: [f64; 3], c: [f64; 3]) -> [f64; 3] {
    [a[1] * c[2] - a[2] * c[1], a[2] * c[0] - a[0] * c[2], a[0] * c[1] - a[1] * c[0]]
}

/// Counts of the analytic faces by kind (plane, cylinder, sphere, other analytic) and of the splines.
fn kinds(b: &Brep) -> (usize, usize, usize, usize, usize) {
    let mut k = (0, 0, 0, 0, 0);
    for f in &b.faces {
        match f.surface {
            Surface::Plane { .. } | Surface::PlaneThrough { .. } => k.0 += 1,
            Surface::Cylinder { .. } => k.1 += 1,
            Surface::Sphere { .. } => k.2 += 1,
            Surface::Spline { .. } | Surface::RationalSpline { .. } => k.4 += 1,
            _ => k.3 += 1,
        }
    }
    k
}

/// The analytic faces are the part's: planes normal to z at the base (z = 0), cylinders of the wall's
/// radius about the z axis, spheres of the cap's radius centred on the axis at the cap's centre.
fn assert_geometry(b: &Brep) {
    for f in &b.faces {
        match &f.surface {
            Surface::Plane { origin, u_axis, v_axis } => {
                let n = unit(cross(*u_axis, *v_axis));
                assert!(n[2].abs() > 0.999, "a plane normal {n:?} is not along z");
                assert!(origin[2].abs() < GEOMETRY_TOLERANCE, "the plane is at z = {}", origin[2]);
            }
            Surface::Cylinder { frame, radius } => {
                assert!((radius - RADIUS).abs() < GEOMETRY_TOLERANCE, "radius {radius}");
                let axis = unit(cross(frame.x_axis, frame.y_axis));
                assert!(axis[2].abs() > 0.999, "axis {axis:?}");
                assert!(frame.origin[0].hypot(frame.origin[1]) < GEOMETRY_TOLERANCE, "axis through {:?}", frame.origin);
            }
            Surface::Sphere { frame, radius } => {
                assert!((radius - CAP_RADIUS).abs() < GEOMETRY_TOLERANCE, "radius {radius}");
                let c = frame.origin;
                assert!(
                    c[0].hypot(c[1]) < GEOMETRY_TOLERANCE && (c[2] - CAP_CENTRE).abs() < GEOMETRY_TOLERANCE,
                    "{c:?}"
                );
            }
            _ => {}
        }
    }
}

/// Gate 1: the plane, the cylinder and the sphere of a smooth part are analytic faces of one closed,
/// passing body; the share of the area is reported; the body survives STEP and is one undo step.
#[test]
fn the_hybrid_method_puts_the_plane_cylinder_and_sphere_of_a_smooth_part_on_analytic_faces() {
    let mut s = scanned_session(&part_mesh(), 5);
    let docs_before = (s.doc.bodies.len(), s.doc.primitives.len());
    let r = to_solid(&mut s, "hybrid");
    assert_eq!(r["method"], "hybrid", "{r}");
    assert_eq!(r["verdict"], "pass", "{r}");
    assert_eq!(s.doc.bodies.len(), docs_before.0 + 1, "one body was added");
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!((s.doc.bodies.len(), s.doc.primitives.len()), docs_before, "one undo step takes the run back");
    s.execute("edit.redo", Value::Null).unwrap();
    let brep = body_of(&s, &r);
    let (planes, cylinders, spheres, others, splines) = kinds(&brep);
    eprintln!(
        "smooth part: {planes} planes, {cylinders} cylinders, {spheres} spheres, {others} others, {splines} splines; {}",
        r["analytic"]
    );
    // the cap and the wall are merged; the base is one flat disc whose patches all touch the round, so it
    // may stay patches (the border band of a region is never merged): at least two of the three kinds
    assert!(spheres >= 1, "the cap is an analytic face");
    assert!(usize::from(planes >= 1) + usize::from(cylinders >= 1) + usize::from(spheres >= 1) >= 2, "{r}");
    assert_geometry(&brep);
    assert!(splines >= 10, "the rounds and the border bands are still patches: {splines}");
    let a = &r["analytic"];
    assert_eq!(a["faces"].as_u64().unwrap() as usize, planes + cylinders + spheres + others);
    assert_eq!(a["total_faces"].as_u64().unwrap() as usize, brep.faces.len());
    let share = a["area_share"].as_f64().unwrap();
    // the base, the wall and the cap are about 1 000 of the part's 1 200 mm2; the rounds and the border
    // bands of patches are not merged
    assert!((0.15..0.95).contains(&share), "analytic area share {share}");
    for m in a["merged"].as_array().unwrap() {
        assert!(m["max_edge_gap"].as_f64().unwrap() <= 0.2 + 1e-9, "{m}");
    }
    assert!(r["body"]["volume"].as_f64().is_some(), "a closed solid has a volume");
    assert!(r["deviation"]["within"].as_f64().unwrap() >= 0.99, "{r}");
    // STEP keeps the face count, the kinds of surface and the volume
    let id = r["body"]["id"].as_u64().unwrap();
    let e = s.execute("export.step", json!({"objects": [id]})).unwrap();
    let back = s.execute("import.step", json!({"data": e["data"], "name": "reimported"})).unwrap();
    assert_eq!(back["faces"][0].as_u64().unwrap() as usize, brep.faces.len());
    let reimported = (*s.doc.body(ObjectId(back["bodies"][0].as_u64().unwrap())).unwrap().brep).clone();
    assert_eq!(kinds(&reimported), kinds(&brep), "the kinds of surface survive STEP");
    let v = r["body"]["volume"].as_f64().unwrap();
    let v2 = s.execute("body.info", json!({"object": back["bodies"][0]})).unwrap()["volume"].as_f64().unwrap();
    assert!((v2 - v).abs() / v < 1e-6, "volume {v2} after the STEP round trip, {v} before");
    // the freeform method on the same scan has no analytic face
    let f = to_solid(&mut s, "freeform");
    assert!(f["analytic"].is_null());
    let (fp, fc, fs, fo, _) = kinds(&body_of(&s, &f));
    assert_eq!((fp, fc, fs, fo), (0, 0, 0, 0), "a patch network has no analytic face");
}

/// Gate 2: the same part as an open scan (a wedge cut away) gives a sheet with analytic faces, not a failure.
#[test]
fn an_open_scan_gives_a_sheet_with_analytic_faces() {
    let mut part = part_mesh();
    let keep: Vec<u32> = (0..part.triangle_count())
        .filter(|&t| part.face_centroid(t).is_some_and(|c| c.x < 1.0))
        .map(|t| t as u32)
        .collect();
    part = part.submesh(&keep);
    let mut s = scanned_session(&part, 7);
    let r = to_solid_open(&mut s, "hybrid");
    assert_eq!(r["method"], "hybrid", "{r}");
    let codes: Vec<&str> = r["issues"].as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap()).collect();
    assert!(!codes.iter().any(|c| c.starts_with("hybrid_")), "{codes:?}: {r}");
    assert!(r["body"]["volume"].is_null(), "an open scan gives a sheet: {r}");
    let brep = body_of(&s, &r);
    let (planes, cylinders, spheres, ..) = kinds(&brep);
    eprintln!(
        "open part: {planes} planes, {cylinders} cylinders, {spheres} spheres; verdict {}; {}",
        r["verdict"], r["analytic"]
    );
    assert!(planes + cylinders + spheres >= 2, "{r}");
    assert_geometry(&brep);
    let freeform = to_solid_open(&mut s, "freeform");
    assert_eq!(freeform["verdict"], r["verdict"], "the verdict freeform gives for the open scan");
}

/// The organic blob of the earlier gates: a closed freeform part.
fn blob_mesh() -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, 200, 100);
    for p in &mut m.positions {
        let d = p.coords;
        let r = 20.0
            * (1.0
                + 0.22 * (5.0 * d.x).sin() * (4.0 * d.y).cos()
                + 0.2 * (4.0 * d.z + 1.2 * d.x).sin() * (5.0 * d.y).cos()
                + 0.16 * (6.0 * d.z).cos() * (3.0 * d.x).sin());
        p.coords = d * r;
    }
    m
}

/// A scan in which no group of patches merges gives the freeform verdict with no analytic face; when it
/// has no region a primitive fits at all, the hybrid body is the freeform body (same counts and volume).
#[test]
fn a_scan_without_a_merged_group_gives_the_freeform_result() {
    let mut s = scanned_session(&blob_mesh(), 5);
    let h = to_solid(&mut s, "hybrid");
    let f = to_solid(&mut s, "freeform");
    assert_eq!(h["method"], "hybrid", "{h}");
    assert_eq!(h["verdict"], f["verdict"], "{h}");
    assert_eq!(h["analytic"]["faces"], 0, "{h}");
    assert!(h["analytic"]["merged"].as_array().unwrap().is_empty());
    if h["modelled"].as_array().unwrap().is_empty() {
        assert_eq!(h["patches"], f["patches"]);
        assert_eq!(h["body"]["faces"], f["body"]["faces"]);
        let (v, w) = (h["body"]["volume"].as_f64().unwrap(), f["body"]["volume"].as_f64().unwrap());
        assert!((v - w).abs() <= 1e-9 * w.abs(), "volume {v} against {w}");
    }
}

/// A scan whose one region is the whole scan (a sphere): the cap is 360 degrees, which the parameters of a
/// sphere cannot hold, so it stays patches and is reported; the body is the valid wrap.
#[test]
fn a_region_that_cannot_be_merged_stays_patches_and_is_reported() {
    let mut s = scanned_session(&synth::sphere(Point3::origin(), 15.0, 80, 40), 3);
    let r = to_solid(&mut s, "hybrid");
    assert_eq!(r["method"], "hybrid", "{r}");
    assert_ne!(r["verdict"], "fail", "{r}");
    assert!(r["body"]["volume"].as_f64().is_some(), "{r}");
    let a = &r["analytic"];
    assert_eq!(a["faces"], a["merged"].as_array().unwrap().len(), "{a}");
    for g in a["left_as_patches"].as_array().unwrap() {
        assert!(g["reason"].as_str().is_some_and(|t| !t.is_empty()), "{g}");
    }
}

/// A call cancelled before it starts ends as a cancellation and leaves the document as it was.
#[test]
fn a_cancelled_hybrid_call_changes_nothing() {
    let mut s = scanned_session(&part_mesh(), 5);
    let before = (s.doc.bodies.len(), s.doc.primitives.len(), s.doc.meshes.len());
    s.cancel.cancel();
    let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "hybrid", "tolerance": 0.2}));
    assert!(r.is_err(), "{r:?}");
    assert_eq!(before.0, s.doc.bodies.len());
    assert_eq!(before.2, s.doc.meshes.len());
}

/// Hostile tolerances are refused or give a result, never a panic.
#[test]
fn hostile_tolerances_never_panic() {
    for tol in [0.0, -1.0, 1e12, f64::NAN] {
        let mut s = scanned_session(&synth::sphere(Point3::origin(), 15.0, 40, 20), 3);
        let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "hybrid", "tolerance": tol}));
        if let Ok(v) = r {
            assert_eq!(v["method"], "hybrid");
        }
    }
}
