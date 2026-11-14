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
    turned(&profile())
}

/// A closed mesh: `prof` (distance from the axis, height), from the axis to the axis, turned about z.
fn turned(prof: &[(f64, f64)]) -> Mesh {
    const STEPS: usize = 120;
    let mut m = Mesh::default();
    let mut ring_start = Vec::new();
    for &(rho, z) in prof {
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
    // the cap and the wall are merged (the base is one flat disc whose patches all touch the round, so
    // it may stay patches: the border band of a region is never merged, and planes are not asserted); every
    // analytic face is the part's own (`assert_geometry`), and the areas are asserted below
    assert!(spheres >= 1 && cylinders >= 1, "the cap and the wall have analytic faces: {r}");
    assert_geometry(&brep);
    assert!(splines >= 10, "the rounds and the border bands are still patches: {splines}");
    let a = &r["analytic"];
    assert_eq!(a["faces"].as_u64().unwrap() as usize, planes + cylinders + spheres + others);
    assert_eq!(a["total_faces"].as_u64().unwrap() as usize, brep.faces.len());
    let area_of = |kind: &str| -> f64 {
        a["merged"].as_array().unwrap().iter().filter(|m| m["kind"] == kind).map(|m| m["area"].as_f64().unwrap()).sum()
    };
    // true areas of the part: the cap 2 pi R^2 (1 - cos 36.87 deg) = 181 mm2, the wall 2 pi 8 x 12 = 603 mm2.
    // Measured on the part (platform and seed move them by a few percent): the cap 183 mm2 in one face, the wall
    // 401 mm2 in six, the share 0.53. The bounds below are at least 1.5 times from those on the low side, and
    // the true area (and 5 %) on the high one, so a face that covers 5 % of its wall does not pass.
    let (cap, wall) = (area_of("sphere"), area_of("cylinder"));
    assert!((120.0..=190.0).contains(&cap), "the cap's analytic area {cap} mm2 (181 in all)");
    assert!((260.0..=633.0).contains(&wall), "the wall's analytic area {wall} mm2 (603 in all)");
    let share = a["area_share"].as_f64().unwrap();
    assert!((0.33..0.95).contains(&share), "analytic area share {share}");
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

/// How many analytic faces are exactly the part's plane, cylinder or sphere (an open scan can also give
/// faces on primitives fitted to a round or a rim, which are within the tolerance of their patches only).
fn faces_on_the_part(b: &Brep) -> usize {
    b.faces
        .iter()
        .filter(|f| match &f.surface {
            Surface::Plane { origin, u_axis, v_axis } => {
                unit(cross(*u_axis, *v_axis))[2].abs() > 0.999 && origin[2].abs() < GEOMETRY_TOLERANCE
            }
            Surface::Cylinder { frame, radius } => {
                (radius - RADIUS).abs() < GEOMETRY_TOLERANCE
                    && frame.origin[0].hypot(frame.origin[1]) < GEOMETRY_TOLERANCE
            }
            Surface::Sphere { frame, radius } => {
                (radius - CAP_RADIUS).abs() < GEOMETRY_TOLERANCE
                    && (frame.origin[2] - CAP_CENTRE).abs() < GEOMETRY_TOLERANCE
            }
            _ => false,
        })
        .count()
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
    // informational codes (`hybrid_edge_gap`, `hybrid_merge_rejected`) may be there; a failure may not
    let failures = ["hybrid_wrap_failed", "hybrid_merge_failed", "hybrid_no_analytic_face", "hybrid_failed"];
    assert!(!codes.iter().any(|c| failures.contains(c)), "{codes:?}: {r}");
    assert!(!r["body"].is_null(), "a body: {r}");
    assert!(r["body"]["volume"].is_null(), "an open scan gives a sheet: {r}");
    let brep = body_of(&s, &r);
    let (planes, cylinders, spheres, ..) = kinds(&brep);
    eprintln!(
        "open part: {planes} planes, {cylinders} cylinders, {spheres} spheres; verdict {}; {}",
        r["verdict"], r["analytic"]
    );
    assert!(planes + cylinders + spheres >= 2, "{r}");
    // measured: six faces are the part's own (the plane of the base and five cylinders of the wall) and the wall's
    // cylinders cover 171 mm2 of its 603; the bounds are 1.5 times from those
    assert!(faces_on_the_part(&brep) >= 4, "too few analytic faces are the part's own: {r}");
    let wall: f64 = r["analytic"]["merged"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|m| m["kind"] == "cylinder")
        .map(|m| m["area"].as_f64().unwrap())
        .sum();
    assert!((110.0..=633.0).contains(&wall), "the wall's analytic area {wall} mm2 (603 in all)");
    let freeform = to_solid_open(&mut s, "freeform");
    assert_eq!(freeform["verdict"], r["verdict"], "the verdict freeform gives for the open scan");
}

/// The cup: a body of revolution with an outer wall of radius 14 (z from 2 to 18), a flat base and a flat
/// rim, and a bore of radius 6 (z from 5 to 18) with a flat floor at z = 3; every edge is rounded by 2. The
/// bore wall is a concave cylinder, the commonest analytic feature of a machined part that is not convex.
const CUP_OUTER: f64 = 14.0;
const CUP_BORE: f64 = 6.0;
const CUP_HEIGHT: f64 = 20.0;

fn cup_profile() -> Vec<(f64, f64)> {
    let arc = |c: (f64, f64), from: f64, to: f64| -> Vec<(f64, f64)> {
        (0..=8)
            .map(|k| {
                let a = from + (to - from) * f64::from(k) / 8.0;
                (c.0 + ROUND * a.cos(), c.1 + ROUND * a.sin())
            })
            .collect()
    };
    let mut p = vec![(0.0, 0.0)];
    p.extend((1..12).map(|r| (f64::from(r), 0.0)));
    p.extend(arc((CUP_OUTER - ROUND, ROUND), -FRAC_PI_2, 0.0));
    p.extend((3..18).map(|z| (CUP_OUTER, f64::from(z))));
    p.extend(arc((CUP_OUTER - ROUND, CUP_HEIGHT - ROUND), 0.0, FRAC_PI_2));
    p.extend((9..12).rev().map(|r| (f64::from(r), CUP_HEIGHT)));
    p.extend(arc((CUP_BORE + ROUND, CUP_HEIGHT - ROUND), FRAC_PI_2, std::f64::consts::PI));
    p.extend((6..18).rev().map(|z| (CUP_BORE, f64::from(z))));
    p.extend(arc((CUP_BORE - ROUND, 5.0), 0.0, -FRAC_PI_2));
    p.extend((1..4).rev().map(|r| (f64::from(r), 3.0)));
    p.push((0.0, 3.0));
    p
}

/// Whether the shell of `b` uses face `face` against its natural normal.
fn used_backward(b: &Brep, face: usize) -> bool {
    use surfacecraft_kernel::record::brep::Shell;
    b.shells.iter().any(|s| match s {
        Shell::Faces { faces, .. } => faces.iter().any(|o| o.face as usize == face && !o.forward),
        _ => false,
    })
}

/// Gate 3: the concave wall of a bore is merged into analytic cylinder faces used against their natural
/// normal; the outer wall and the planes are too; the body stays a passing closed solid that survives STEP.
#[test]
fn the_bore_of_a_cup_becomes_concave_cylinder_faces() {
    let mut s = scanned_session(&turned(&cup_profile()), 5);
    let r = to_solid(&mut s, "hybrid");
    assert_eq!(r["method"], "hybrid", "{r}");
    assert_eq!(r["verdict"], "pass", "{r}");
    let brep = body_of(&s, &r);
    let a = &r["analytic"];
    eprintln!("cup: {}", a);
    let cylinders: Vec<(usize, f64)> = brep
        .faces
        .iter()
        .enumerate()
        .filter_map(|(i, f)| match &f.surface {
            Surface::Cylinder { frame, radius } => {
                let axis = unit(cross(frame.x_axis, frame.y_axis));
                (axis[2].abs() > 0.999 && frame.origin[0].hypot(frame.origin[1]) < GEOMETRY_TOLERANCE)
                    .then_some((i, *radius))
            }
            _ => None,
        })
        .collect();
    let bore: Vec<usize> =
        cylinders.iter().filter(|(_, r)| (r - CUP_BORE).abs() < GEOMETRY_TOLERANCE).map(|(i, _)| *i).collect();
    let outer: Vec<usize> =
        cylinders.iter().filter(|(_, r)| (r - CUP_OUTER).abs() < GEOMETRY_TOLERANCE).map(|(i, _)| *i).collect();
    let area_of_faces = |faces: &[usize]| -> f64 {
        a["merged"]
            .as_array()
            .unwrap()
            .iter()
            .filter(|m| faces.contains(&(m["face"].as_u64().unwrap() as usize)))
            .map(|m| m["area"].as_f64().unwrap())
            .sum()
    };
    let (bore_area, outer_area) = (area_of_faces(&bore), area_of_faces(&outer));
    eprintln!("cup: bore faces {bore:?} {bore_area}, outer faces {outer:?} {outer_area}");
    assert!(!bore.is_empty() && !outer.is_empty(), "the bore and the outer wall have faces: {r}");
    // true areas: the bore wall 2 pi 6 x 13 = 490 mm2, the outer wall 2 pi 14 x 16 = 1 407 mm2; measured 160 and
    // 848; the bounds are 1.5 times from those on the low side and the true area (and 5 %) on the high one
    assert!((100.0..=515.0).contains(&bore_area), "the bore's analytic area {bore_area} mm2 (490 in all)");
    assert!((550.0..=1478.0).contains(&outer_area), "the outer wall's analytic area {outer_area} mm2 (1 407 in all)");
    assert!(bore.iter().all(|&f| used_backward(&brep, f)), "the bore's faces are used against their natural normal");
    assert!(outer.iter().all(|&f| !used_backward(&brep, f)), "the outer wall's faces are used along it");
}

/// A disc of radius 30 and height 6 with a rim rounded by 2: two flat faces of 2 463 mm2 each, the large
/// planes of a plate.
const DISC_RADIUS: f64 = 30.0;
const DISC_HEIGHT: f64 = 6.0;

fn disc_profile() -> Vec<(f64, f64)> {
    let corner = |c: (f64, f64), from: f64, to: f64| -> Vec<(f64, f64)> {
        (0..=8)
            .map(|k| {
                let a = from + (to - from) * f64::from(k) / 8.0;
                (c.0 + ROUND * a.cos(), c.1 + ROUND * a.sin())
            })
            .collect()
    };
    let mut p = vec![(0.0, 0.0)];
    p.extend((1..28).map(|r| (f64::from(r), 0.0)));
    p.extend(corner((DISC_RADIUS - ROUND, ROUND), -FRAC_PI_2, 0.0));
    p.extend(corner((DISC_RADIUS - ROUND, DISC_HEIGHT - ROUND), 0.0, FRAC_PI_2));
    p.extend((1..28).rev().map(|r| (f64::from(r), DISC_HEIGHT)));
    p.push((0.0, DISC_HEIGHT));
    p
}

/// Gate 4: the flat faces of a plate are analytic faces that cover most of their area (a large plane is the
/// easy case: one rebuild of many patches).
#[test]
fn the_large_flat_faces_of_a_disc_are_analytic_faces() {
    let mut s = scanned_session(&turned(&disc_profile()), 5);
    let r = to_solid(&mut s, "hybrid");
    assert_eq!(r["method"], "hybrid", "{r}");
    assert_eq!(r["verdict"], "pass", "{r}");
    let brep = body_of(&s, &r);
    let a = &r["analytic"];
    let planes: Vec<f64> = a["merged"]
        .as_array()
        .unwrap()
        .iter()
        .filter(|m| m["kind"] == "plane")
        .map(|m| m["area"].as_f64().unwrap())
        .collect();
    eprintln!("disc: plane areas {planes:?}; {}", a["stages_ms"]);
    // measured: 2 407 and 2 078 mm2 of the 2 463 of each flat face; the bounds are 1.5 times from those
    let mut sorted = planes.clone();
    sorted.sort_by(|a, b| b.total_cmp(a));
    assert!(sorted.len() >= 2, "both flat faces have an analytic face: {r}");
    assert!(sorted[0] >= 1600.0 && sorted[0] <= 2586.0, "the largest plane's area {} (2 463 in all)", sorted[0]);
    assert!(sorted[1] >= 1350.0 && sorted[1] <= 2586.0, "the second plane's area {} (2 463 in all)", sorted[1]);
    let at = |z: f64| {
        brep.faces
            .iter()
            .any(|f| matches!(&f.surface, Surface::Plane { origin, .. } if (origin[2] - z).abs() < GEOMETRY_TOLERANCE))
    };
    assert!(at(0.0) && at(DISC_HEIGHT), "a plane at the bottom and one at the top");
    assert_geometry_disc(&brep);
}

/// The analytic faces of the disc are planes normal to z at z = 0 or at its height.
fn assert_geometry_disc(b: &Brep) {
    for f in &b.faces {
        if let Surface::Plane { origin, u_axis, v_axis } = &f.surface {
            let n = unit(cross(*u_axis, *v_axis));
            assert!(n[2].abs() > 0.999, "{n:?}");
            let z = origin[2];
            assert!(z.abs() < GEOMETRY_TOLERANCE || (z - DISC_HEIGHT).abs() < GEOMETRY_TOLERANCE, "z = {z}");
        }
    }
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

/// With no region large enough for a group the hybrid method makes no second wrap: it returns the
/// freeform result and says why (`min_region_triangles` leaves no protected region).
#[test]
fn with_no_region_large_enough_the_hybrid_method_returns_the_freeform_result_at_once() {
    let mut s = scanned_session(&part_mesh(), 5);
    let p =
        |m: &str| json!({"scan": "scan", "method": m, "tolerance": 0.2, "name": m, "min_region_triangles": 1_000_000});
    let h = s.execute("scan.to_solid", p("hybrid")).unwrap();
    let f = s.execute("scan.to_solid", p("freeform")).unwrap();
    assert_eq!(h["method"], "hybrid", "{h}");
    let codes: Vec<&str> = h["issues"].as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap()).collect();
    assert!(codes.contains(&"hybrid_no_analytic_face"), "{codes:?}");
    assert_eq!(h["analytic"]["faces"], 0);
    assert_eq!((&h["patches"], &h["verdict"]), (&f["patches"], &f["verdict"]), "the freeform result");
}
