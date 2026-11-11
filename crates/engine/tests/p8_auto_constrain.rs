//! P8 gates: auto-constraints beyond parallel and perpendicular (tangency, symmetry about a plane,
//! equal distances) and the sigma check, through the command layer, on synthetic scans with
//! noise sigma = 0.03 mm. The thresholds of design doc §11.1 are unchanged.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

const SIGMA: f64 = 0.03;

fn noisy(mut m: Mesh, seed: u64) -> Mesh {
    synth::add_noise(&mut m, SIGMA, &mut Rng::new(seed));
    m
}

/// Half a cylinder along z: `theta0 .. theta0 + π` around the axis through `(cx, cy)`.
fn half_cylinder(cx: f64, cy: f64, r: f64, h: f64, theta0: f64) -> Mesh {
    let (nu, nv) = (40_usize, 10_usize);
    let mut positions = Vec::new();
    for j in 0..=nv {
        for i in 0..=nu {
            let th = theta0 + PI * i as f64 / nu as f64;
            positions.push(Point3::new(cx + r * th.cos(), cy + r * th.sin(), h * j as f64 / nv as f64));
        }
    }
    let cols = (nu + 1) as u32;
    let mut triangles = Vec::new();
    for j in 0..nv as u32 {
        for i in 0..nu as u32 {
            let a = j * cols + i;
            let (b, c, d) = (a + 1, a + cols + 1, a + cols);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { positions, triangles }
}

/// Adds a scanned surface: a mesh, a region over all of it and the primitive fitted to the region.
fn add(s: &mut Session, name: &str, mesh: Mesh, kind: &str) {
    let all: Vec<u32> = (0..mesh.triangle_count() as u32).collect();
    s.doc.add_mesh(&format!("scan {name}"), mesh);
    s.execute(
        "region.create",
        json!({"mesh": format!("scan {name}"), "triangles": all, "name": format!("region {name}")}),
    )
    .unwrap();
    s.execute("primitive.fit", json!({"region": format!("region {name}"), "kind": kind, "name": name})).unwrap();
}

fn prim(s: &Session, name: &str) -> Primitive {
    s.doc.primitives.iter().find(|p| p.name == name).unwrap().primitive.clone()
}

fn count(v: &Value, kind: &str) -> usize {
    v["relations"].as_array().unwrap().iter().filter(|r| r["kind"] == kind).count()
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

/// A slot: two parallel walls joined by two half-cylindrical ends (radii 6.0 and 6.05, so the
/// walls cannot be tangent to both until the radii are made equal).
fn slot(s: &mut Session) {
    let (r, l, h) = (6.0, 40.0, 20.0);
    let wall = |y: f64, seed: u64| {
        noisy(synth::plane(Point3::new(-l / 2.0, y, 0.0), Vec3::x(), Vec3::z(), l, h, 40, 10), seed)
    };
    add(s, "wall top", wall(r, 1), "plane");
    add(s, "wall bottom", wall(-r, 2), "plane");
    add(s, "end right", noisy(half_cylinder(l / 2.0, 0.0, r, h, -PI / 2.0), 3), "cylinder");
    add(s, "end left", noisy(half_cylinder(-l / 2.0, 0.0, r + 0.05, h, PI / 2.0), 4), "cylinder");
}

#[test]
fn gate_p8_slot_gets_tangency_and_equal_radii() {
    let mut s = Session::new();
    slot(&mut s);
    let fitted: Vec<Primitive> = s.doc.primitives.iter().map(|p| p.primitive.clone()).collect();

    // report only: nothing moves, the relations and the sigma are reported
    let dry = s.execute("primitive.auto_constrain", json!({"apply": false})).unwrap();
    assert_eq!(dry["applied"], false);
    assert_eq!(count(&dry, "tangent"), 4, "{dry}");
    assert_eq!(count(&dry, "equal_radius"), 1, "{dry}");
    assert_eq!(s.doc.primitives.iter().map(|p| p.primitive.clone()).collect::<Vec<_>>(), fitted);
    for r in dry["relations"].as_array().unwrap().iter().filter(|r| r["kind"] == "tangent") {
        let e = r["error"].as_f64().unwrap();
        assert!(e > 0.0 && e < 0.2, "{r}");
    }

    let out = s.execute("primitive.auto_constrain", json!({})).unwrap();
    eprintln!(
        "p8 slot: tangent {}, equal radius {}, sigma {}, limit {}, rejected {}",
        count(&out, "tangent"),
        count(&out, "equal_radius"),
        out["sigma"],
        out["rms_limit"],
        out["rejected"].as_array().unwrap().len()
    );
    assert_eq!(count(&out, "tangent"), 4, "{out}");
    assert_eq!(count(&out, "equal_radius"), 1, "{out}");
    assert!(out["rejected"].as_array().unwrap().is_empty(), "{out}");
    assert!(out["max_length_error"].as_f64().unwrap() < 1e-9, "{out}");
    assert!(out["max_angle_error_deg"].as_f64().unwrap() < 1e-6, "{out}");

    // exactly equal radii, and every wall tangent to every end
    let (Primitive::Cylinder { radius: right, .. }, Primitive::Cylinder { radius: left, .. }) =
        (prim(&s, "end right"), prim(&s, "end left"))
    else {
        panic!("not cylinders")
    };
    assert!((right - left).abs() < 1e-9, "{right} {left}");
    assert!((5.99..6.06).contains(&right), "{right}");
    for wall in ["wall top", "wall bottom"] {
        let Primitive::Plane { point, normal } = prim(&s, wall) else { panic!("not a plane") };
        for end in ["end right", "end left"] {
            let Primitive::Cylinder { point: c, radius, .. } = prim(&s, end) else { panic!("not a cylinder") };
            let d = dot(normal, sub(c, point));
            assert!((d.abs() - radius).abs() < 1e-9, "{wall} / {end}: distance {} radius {radius}", d.abs());
        }
    }

    // sigma before and after: the snap stays within the noise
    let rows = out["primitives"].as_array().unwrap();
    assert_eq!(rows.len(), 4);
    for p in rows {
        let (b, a) = (p["rms_before"].as_f64().unwrap(), p["rms_after"].as_f64().unwrap());
        assert!(b < 0.06 && a <= 0.1, "{p}");
    }
    let sigma = out["sigma"].as_f64().unwrap();
    assert!(sigma > 0.01 && sigma < 0.06, "{sigma}");
    assert!((out["rms_limit"].as_f64().unwrap() - 0.1).abs() < 1e-12);
}

#[test]
fn gate_p8_three_planes_snap_to_equal_spacing() {
    let mut s = Session::new();
    for (name, z, seed) in [("plane a", 0.0, 11), ("plane b", 10.02, 12), ("plane c", 19.99, 13)] {
        let m = synth::plane(Point3::new(0.0, 0.0, z), Vec3::x(), Vec3::y(), 40.0, 40.0, 20, 20);
        add(&mut s, name, noisy(m, seed), "plane");
    }
    let out = s.execute("primitive.auto_constrain", json!({})).unwrap();
    let eq: Vec<&Value> =
        out["relations"].as_array().unwrap().iter().filter(|r| r["kind"] == "equal_distance").collect();
    assert_eq!(eq.len(), 1, "{out}");
    // the spacings are 10.02 and 9.97: they differ by 0.05
    assert!((eq[0]["error"].as_f64().unwrap() - 0.05).abs() < 0.02, "{out}");
    assert!(eq[0]["c"].is_number(), "an equal distance names its third primitive: {out}");
    assert!(out["rejected"].as_array().unwrap().is_empty(), "{out}");
    assert!(out["max_length_error"].as_f64().unwrap() < 1e-9, "{out}");

    let Primitive::Plane { normal, .. } = prim(&s, "plane a") else { panic!("not a plane") };
    let along = |name: &str| {
        let Primitive::Plane { point, .. } = prim(&s, name) else { panic!("not a plane") };
        dot(normal, point)
    };
    let (g1, g2) = (along("plane b") - along("plane a"), along("plane c") - along("plane b"));
    assert!((g1 - g2).abs() < 1e-9, "spacings {g1} and {g2}");
    assert!((g1.abs() - 10.0).abs() < 0.05, "{g1}");
    for p in out["primitives"].as_array().unwrap() {
        assert!(p["rms_after"].as_f64().unwrap() <= 0.1, "{p}");
    }
}

fn mirrored_part(s: &mut Session) {
    // the right post is 0.1 mm off the mirror image of the left one
    let post = |x: f64, seed: u64| noisy(synth::cylinder(Point3::new(x, 5.0, 0.0), Vec3::z(), 6.0, 30.0, 48, 12), seed);
    add(s, "post left", post(-20.0, 21), "cylinder");
    add(s, "post right", post(20.1, 22), "cylinder");
}

fn cylinders(s: &Session) -> (([f64; 3], [f64; 3], f64), ([f64; 3], [f64; 3], f64)) {
    let (
        Primitive::Cylinder { point: pl, axis: al, radius: rl },
        Primitive::Cylinder { point: pr, axis: ar, radius: rr },
    ) = (prim(s, "post left"), prim(s, "post right"))
    else {
        panic!("not cylinders")
    };
    ((pl, al, rl), (pr, ar, rr))
}

#[test]
fn gate_p8_mirrored_part_gets_its_symmetric_pair() {
    // the mirror plane x = 0 is given
    let mut s = Session::new();
    mirrored_part(&mut s);
    s.doc.add_primitive(
        "mirror",
        Primitive::Plane { point: [0.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0] },
        None,
        0.0,
        0.0,
        Mesh::default(),
    );
    let out = s.execute("primitive.auto_constrain", json!({"symmetry_planes": ["mirror"]})).unwrap();
    assert_eq!(count(&out, "symmetric"), 1, "{out}");
    assert_eq!(out["primitives"].as_array().unwrap().len(), 2, "the mirror plane is a reference: {out}");
    assert!(out["rejected"].as_array().unwrap().is_empty(), "{out}");
    assert!(out["max_length_error"].as_f64().unwrap() < 1e-9, "{out}");
    assert!(out["max_angle_error_deg"].as_f64().unwrap() < 1e-6, "{out}");
    let ((pl, al, rl), (pr, ar, rr)) = cylinders(&s);
    assert!((pl[0] + pr[0]).abs() < 1e-9, "{pl:?} {pr:?}");
    assert!((rl - rr).abs() < 1e-9, "{rl} {rr}");
    // the axes are mirror images (the sign of an axis is arbitrary)
    let mirrored = [-al[0], al[1], al[2]];
    assert!((1.0 - dot(mirrored, ar).abs()).abs() < 1e-12, "{al:?} {ar:?}");
    // and the mirror image of the left axis is the right axis line
    let w = sub([-pl[0], pl[1], pl[2]], pr);
    let along = dot(w, ar);
    let off = sub(w, [ar[0] * along, ar[1] * along, ar[2] * along]);
    assert!(dot(off, off).sqrt() < 1e-9, "{off:?}");
    // the given plane itself is untouched
    assert_eq!(prim(&s, "mirror"), Primitive::Plane { point: [0.0, 0.0, 0.0], normal: [1.0, 0.0, 0.0] });

    // the plane found among the principal planes of the model
    let mut s = Session::new();
    mirrored_part(&mut s);
    let out = s.execute("primitive.auto_constrain", json!({})).unwrap();
    assert_eq!(count(&out, "symmetric"), 1, "{out}");
    assert!(out["rejected"].as_array().unwrap().is_empty(), "{out}");
    assert!(out["max_length_error"].as_f64().unwrap() < 1e-9, "{out}");
    let ((pl, _, rl), (pr, _, rr)) = cylinders(&s);
    assert!((rl - rr).abs() < 1e-9, "{rl} {rr}");
    assert!((pl[0] + pr[0]).abs() < 0.2, "symmetric about a plane near x = 0: {pl:?} {pr:?}");
    // without symmetry detection the pair is left alone
    let mut s = Session::new();
    mirrored_part(&mut s);
    let out = s.execute("primitive.auto_constrain", json!({"detect_symmetry": false})).unwrap();
    assert_eq!(count(&out, "symmetric"), 0, "{out}");
}

#[test]
fn gate_p8_plane_four_degrees_off_is_rejected() {
    let mut s = Session::new();
    let tilt = 4.0_f64.to_radians();
    let flat = |z: f64| synth::plane(Point3::new(0.0, 0.0, z), Vec3::x(), Vec3::y(), 40.0, 40.0, 20, 20);
    add(&mut s, "plane a", noisy(flat(0.0), 31), "plane");
    add(
        &mut s,
        "plane b",
        noisy(
            synth::plane(
                Point3::new(0.0, 0.0, 20.0),
                Vec3::x(),
                Vec3::new(0.0, tilt.cos(), tilt.sin()),
                40.0,
                40.0,
                20,
                20,
            ),
            32,
        ),
        "plane",
    );
    add(&mut s, "plane c", noisy(flat(40.0), 33), "plane");
    let b = prim(&s, "plane b");

    // a tolerance of 5 degrees finds the 4 degree relations; snapping them would cost far more
    // than max(0.1 mm, 2 sigma), so they are rejected and plane b stays where it is
    let out = s.execute("primitive.auto_constrain", json!({"angle_tol_deg": 5.0})).unwrap();
    let rejected = out["rejected"].as_array().unwrap();
    eprintln!("p8 rejected: {}", rejected.len());
    assert!(
        rejected.iter().any(|r| r["relation"]["kind"] == "parallel"
            && r["relation"]["error"].as_f64().unwrap() > 3.5
            && r["reason"].as_str().unwrap().contains("RMS")),
        "{out}"
    );
    assert_eq!(count(&out, "parallel"), 1, "only the flat pair is related: {out}");
    assert_eq!(count(&out, "equal_distance"), 0, "{out}");
    assert_eq!(count(&out, "symmetric"), 0, "{out}");
    let (Primitive::Plane { point: p0, normal: n0 }, Primitive::Plane { point: p1, normal: n1 }) =
        (b, prim(&s, "plane b"))
    else {
        panic!("not planes")
    };
    assert_eq!(p0, p1);
    assert!(dot(n0, n1).clamp(-1.0, 1.0).abs().acos().to_degrees() < 1e-4, "{n0:?} {n1:?}");
    let row = out["primitives"].as_array().unwrap().iter().find(|p| p["name"] == "plane b").unwrap();
    assert!((row["rms_after"].as_f64().unwrap() - row["rms_before"].as_f64().unwrap()).abs() < 1e-3, "{row}");
    // with the tolerance at the default 3 degrees the 4 degree relations are not even found
    let out = s.execute("primitive.auto_constrain", json!({})).unwrap();
    assert_eq!(count(&out, "parallel"), 1, "{out}");
}

#[test]
fn p8_auto_constrain_hostile_parameters_never_panic() {
    let mut s = Session::new();
    for (name, z, seed) in [("plane a", 0.0, 41), ("plane b", 10.0, 42)] {
        let m = synth::plane(Point3::new(0.0, 0.0, z), Vec3::x(), Vec3::y(), 40.0, 40.0, 10, 10);
        add(&mut s, name, noisy(m, seed), "plane");
    }
    add(
        &mut s,
        "post",
        noisy(synth::cylinder(Point3::new(10.0, 10.0, 0.0), Vec3::z(), 5.0, 20.0, 24, 6), 43),
        "cylinder",
    );
    for v in [
        json!({"noise_sigma": -1.0}),
        json!({"noise_sigma": 1e308}),
        json!({"noise_sigma": "x"}),
        json!({"symmetry_planes": [99999]}),
        json!({"symmetry_planes": ["post"]}),
        json!({"symmetry_planes": []}),
        json!({"symmetry_planes": ["plane a"]}),
        json!({"length_tol": 0.0, "angle_tol_deg": 0.0}),
        json!({"length_tol": 1e308}),
        json!({"length_tol": 1e308, "angle_tol_deg": 20.0, "apply": false}),
        json!({"objects": []}),
        json!({"objects": ["nope"]}),
        json!({"objects": ["post", "post"]}),
        json!({"detect_symmetry": false, "noise_sigma": 0.0}),
        json!({"detect_symmetry": 3}),
    ] {
        let r = s.execute("primitive.auto_constrain", v.clone());
        assert!(!matches!(r, Err(EngineError::Internal(_))), "panicked on {v}");
    }
}
