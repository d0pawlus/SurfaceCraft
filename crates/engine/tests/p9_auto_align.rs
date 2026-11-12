//! P9-M1: automatic alignment, through the command layer: `align.auto` (a pose from the scan
//! alone), `align.symmetry` (the mirror plane on a world plane) and `mesh.scale_from_length`.
//!
//! The ground truth of every frame check is the part's own frame before the random pose was
//! applied: the base plane, the longest side and the axis are known there, so the check follows
//! them through `net = A ∘ G` (the proposed motion after the pose) instead of re-measuring the
//! aligned scan with the method under test.
use serde_json::{Value, json};
use std::hash::{DefaultHasher, Hash, Hasher};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Transform, Vec3, synth};
use surfacecraft_kernel::feature::{BoolKind, boolean};
use surfacecraft_kernel::solid;

/// Five fixed starting poses (axis, degrees, translation): large rotations, offsets far from 0.
fn poses() -> Vec<Transform> {
    let table: [([f64; 3], f64, [f64; 3]); 5] = [
        ([0.3, -0.8, 0.5], 37.0, [120.0, -40.0, 300.0]),
        ([1.0, 0.2, -0.4], 121.0, [-15.0, 220.0, 8.0]),
        ([-0.6, 0.1, 0.9], 203.0, [0.0, 0.0, -75.0]),
        ([0.1, 1.0, 0.1], 88.0, [500.0, 12.0, -3.0]),
        ([0.7, 0.7, -0.2], 311.0, [-260.0, -90.0, 41.0]),
    ];
    table
        .iter()
        .map(|(a, deg, t)| {
            Transform::rotation_axis_angle(&Vec3::from(*a), deg.to_radians())
                .then_after(&Transform::translation(Vec3::from(*t)))
        })
        .collect()
}

fn matrix_of(out: &Value) -> Transform {
    let rows: [[f64; 4]; 4] = serde_json::from_value(out["matrix"].clone()).unwrap();
    Transform::from_rows(&rows).unwrap()
}

fn tilt_deg(a: &Vec3, b: &Vec3) -> f64 {
    (a.dot(b) / (a.norm() * b.norm())).clamp(-1.0, 1.0).acos().to_degrees()
}

/// Angle between the line of `a` and the line of `b` (ignores the sign).
fn line_tilt_deg(a: &Vec3, b: &Vec3) -> f64 {
    tilt_deg(a, b).min(tilt_deg(a, &-b))
}

fn mesh_hash(s: &Session, name: &str) -> u64 {
    let (id, _) = s.doc.find_by_name(name).unwrap();
    let mut h = DefaultHasher::new();
    for p in &s.doc.mesh(id).unwrap().mesh.positions {
        for c in p.coords.iter() {
            c.to_bits().hash(&mut h);
        }
    }
    h.finish()
}

fn session_with(mesh: &Mesh, pose: &Transform) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("part", mesh.transformed(pose));
    s
}

fn noisy(mut m: Mesh, sigma: f64, seed: u64) -> Mesh {
    synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    m
}

/// The frame the truth demands: base plane on z = 0 (outward normal −z), long side along x.
struct Truth {
    /// Faces the part may lie on: a point of each and its outward normal (equal-area opposite
    /// faces of a symmetric part both qualify).
    bases: Vec<(Point3, Vec3)>,
    long_dir: Vec3,
    /// Symmetries of the ideal part in the aligned frame (a pose is right up to one of them).
    symmetries: Vec<Transform>,
}

fn assert_frame(net: &Transform, t: &Truth, label: &str) {
    let ok = t.bases.iter().any(|(point, normal)| {
        let tilt = tilt_deg(&net.apply_vec(normal), &-Vec3::z());
        tilt < 0.1 && net.apply(point).z.abs() < 0.05
    });
    let report: Vec<(f64, f64)> =
        t.bases.iter().map(|(p, n)| (tilt_deg(&net.apply_vec(n), &-Vec3::z()), net.apply(p).z)).collect();
    assert!(ok, "{label}: no base face on z = 0 (tilt deg, height mm per candidate: {report:?})");
    let x = net.apply_vec(&t.long_dir);
    let along = line_tilt_deg(&x, &Vec3::x());
    eprintln!("{label}: base (tilt deg, z mm) {report:?}, long side {along:.4} deg off x");
    assert!(along < 0.2, "{label}: longest side {along:.4} deg off x");
}

/// Runs `align.auto` on the part under all five poses and checks the frame and that the five
/// proposals are one pose (up to the part's symmetries).
fn check_part(mesh: &Mesh, truth: &Truth, label: &str) {
    let mut nets = Vec::new();
    for (k, pose) in poses().iter().enumerate() {
        let mut s = session_with(mesh, pose);
        let out = s.execute("align.auto", json!({"object": "part", "apply": true})).unwrap();
        assert_eq!(out["applied"], true, "{out}");
        assert_eq!(out["method"], "planar_face", "{label} pose {k}: {out}");
        let net = matrix_of(&out).then_after(pose);
        assert_frame(&net, truth, &format!("{label} pose {k}"));
        nets.push(net);
    }
    for (k, net) in nets.iter().enumerate().skip(1) {
        let worst = truth
            .symmetries
            .iter()
            .map(|sym| net.max_displacement(&sym.then_after(&nets[0]), &mesh.positions))
            .fold(f64::INFINITY, f64::min);
        assert!(worst < 0.1, "{label}: pose {k} differs from pose 0 by {worst:.4} mm even up to symmetry");
    }
}

/// The identity and the half turns about the axes through `c` (a box-like part's symmetries).
fn half_turns(c: Point3) -> Vec<Transform> {
    let about = |axis: Vec3| {
        Transform::translation(c.coords)
            .then_after(&Transform::rotation_axis_angle(&axis, std::f64::consts::PI))
            .then_after(&Transform::translation(-c.coords))
    };
    vec![Transform::identity(), about(Vec3::x()), about(Vec3::y()), about(Vec3::z())]
}

// ---- parts ------------------------------------------------------------------------------

/// A 40 × 30 × 15 block with its four vertical edges filleted r = 5, tessellated, welded and made
/// to look scanned (density ramp, noise σ, spikes, debris; no holes, so the part stays symmetric
/// about its centre to within the noise).
fn scanned_block(sigma: f64) -> Mesh {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [40.0, 30.0, 15.0], "name": "block"})).unwrap();
    let b = s.doc.bodies.iter().find(|b| b.name == "block").unwrap();
    let edges: Vec<usize> = b
        .brep
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            let (p, q) = (b.brep.vertices[e.vertices[0] as usize].point, b.brep.vertices[e.vertices[1] as usize].point);
            (p[0] - q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9 && (p[2] - q[2]).abs() > 1.0
        })
        .map(|(i, _)| i)
        .collect();
    s.execute("body.fillet", json!({"object": "block", "edges": edges, "size": 5.0})).unwrap();
    scanify_body(&mut s, "block", sigma, 11)
}

fn scanify_body(s: &mut Session, body: &str, sigma: f64, seed: u64) -> Mesh {
    s.execute("body.to_mesh", json!({"object": body, "chord": 0.01, "name": "cad"})).unwrap();
    let id = s.doc.find_by_name("cad").unwrap().0;
    let mut cad = (*s.doc.mesh(id).unwrap().mesh).clone();
    cad.weld(1e-6);
    let d = synth::ScanDefects { spacing: 0.7, noise: sigma, holes: 0, ..synth::ScanDefects::default() };
    synth::scanify(&cad, &d, &mut Rng::new(seed))
}

/// The scan-corpus pin: cylinder r = 12, h = 30, 3 mm chamfer on the top edge, through hole r = 4.
fn scanned_pin_with_hole(sigma: f64) -> Mesh {
    let mut s = Session::new();
    s.execute("body.cylinder", json!({"radius": 12.0, "height": 30.0, "name": "pin"})).unwrap();
    let b = s.doc.bodies.iter().find(|b| b.name == "pin").unwrap();
    let top = b
        .brep
        .edges
        .iter()
        .position(|e| e.vertices.iter().all(|&v| (b.brep.vertices[v as usize].point[2] - 30.0).abs() < 1e-9))
        .unwrap();
    s.execute("body.chamfer", json!({"object": "pin", "edges": [top], "size": 3.0})).unwrap();
    let hole = solid::make_cylinder([0.0, 0.0, -1.0], 4.0, 32.0).unwrap();
    let pin = (*s.doc.body(s.doc.find_by_name("pin").unwrap().0).unwrap().brep).clone();
    let part = boolean(&pin, &hole, BoolKind::Subtract).unwrap();
    s.doc.add_body("part", part);
    scanify_body(&mut s, "part", sigma, 8)
}

fn bracket_without_bottom(sigma: f64, seed: u64) -> (Mesh, Mesh) {
    let mut clean = synth::bracket();
    clean.weld(1e-9);
    let keep: Vec<u32> = (0..clean.triangle_count())
        .filter(|&i| {
            let on_bottom =
                clean.face_centroid(i).is_some_and(|c| c.z < 1e-6) && clean.face_normal(i).is_some_and(|n| n.z < -0.99);
            !on_bottom
        })
        .map(|i| i as u32)
        .collect();
    assert!(keep.len() < clean.triangle_count());
    let closed = noisy(clean, sigma, seed);
    let open = closed.submesh(&keep);
    (closed, open)
}

fn bracket_truth() -> Truth {
    Truth { bases: vec![(Point3::origin(), -Vec3::z())], long_dir: Vec3::x(), symmetries: vec![Transform::identity()] }
}

// ---- align.auto -------------------------------------------------------------------------

/// Gate: the synthetic bracket (σ = 0.05), five poses: base on z = 0 within 0.05 mm and 0.1°,
/// long side on x within 0.2°, one pose for all five.
#[test]
fn gate_p9_auto_bracket_five_poses() {
    let mesh = noisy(synth::bracket(), 0.05, 3);
    check_part(&mesh, &bracket_truth(), "bracket");
}

/// The bracket scanned for real (density ramp, σ = 0.05, holes, spikes, debris): same gate.
#[test]
fn gate_p9_auto_scanned_bracket_five_poses() {
    let mut clean = synth::bracket();
    clean.weld(1e-9);
    let d = synth::ScanDefects { spacing: 0.8, noise: 0.05, ..synth::ScanDefects::default() };
    let mesh = synth::scanify(&clean, &d, &mut Rng::new(21));
    check_part(&mesh, &bracket_truth(), "scanned bracket");
}

/// Gate: the scanned filleted block (σ = 0.03): top and bottom tie, so the pose is right up to
/// the block's half turns (the documented symmetry choice: the least motion from where it is).
#[test]
fn gate_p9_auto_scanned_block_five_poses() {
    let mesh = scanned_block(0.03);
    let b = mesh.bbox();
    assert!((b.min.z).abs() < 0.2, "the CAD base is z = 0: {b:?}");
    let truth = Truth {
        bases: vec![(Point3::new(20.0, 15.0, 0.0), -Vec3::z()), (Point3::new(20.0, 15.0, 15.0), Vec3::z())],
        long_dir: Vec3::x(),
        symmetries: half_turns(Point3::new(0.0, 0.0, 7.5)),
    };
    check_part(&mesh, &truth, "block");
}

/// Gate: a turned part (chamfered pin with a bore): axis on z within 0.1°, the big end face on
/// z = 0 within 0.05 mm.
#[test]
fn gate_p9_auto_turned_part() {
    let mesh = scanned_pin_with_hole(0.03);
    for (k, pose) in poses().iter().take(3).enumerate() {
        let mut s = session_with(&mesh, pose);
        let out = s.execute("align.auto", json!({"object": "part", "apply": true})).unwrap();
        assert_eq!(out["method"], "turned", "{out}");
        let net = matrix_of(&out).then_after(pose);
        let axis = line_tilt_deg(&net.apply_vec(&Vec3::z()), &Vec3::z());
        eprintln!(
            "turned pose {k}: axis {axis:.4} deg, end face z {:.4}, confidence {}",
            net.apply(&Point3::origin()).z,
            out["confidence"]
        );
        assert!(axis < 0.1, "pose {k}: axis {axis:.4} deg off z");
        let end = net.apply(&Point3::origin());
        assert!(end.z.abs() < 0.05, "pose {k}: end face at z = {:.4}", end.z);
        let down = tilt_deg(&net.apply_vec(&-Vec3::z()), &-Vec3::z());
        assert!(down < 0.1, "pose {k}: the big end must face down ({down:.3} deg)");
        // own check (not a spec threshold): the axis goes through the origin
        assert!(end.x.hypot(end.y) < 0.3, "pose {k}: axis misses the origin by {:.3}", end.x.hypot(end.y));
    }
}

/// Gate: a bracket scanned without its bottom face: the open rim stands for it and the pose equals
/// the closed bracket's (0.1 mm, 0.2°).
#[test]
fn gate_p9_auto_open_rim_equals_closed() {
    let (closed, open) = bracket_without_bottom(0.03, 5);
    let pose = &poses()[1];
    let run = |m: &Mesh| {
        let mut s = session_with(m, pose);
        let out = s.execute("align.auto", json!({"object": "part"})).unwrap();
        (matrix_of(&out), out)
    };
    let (a_closed, out_closed) = run(&closed);
    let (a_open, out_open) = run(&open);
    assert_eq!(out_closed["method"], "planar_face", "{out_closed}");
    assert_eq!(out_open["method"], "open_rim", "{out_open}");
    let delta = a_open.then_after(&a_closed.inverse());
    eprintln!("open vs closed: {:.4} deg, {:.4} mm", delta.angle().to_degrees(), delta.translation.norm());
    assert!(delta.angle().to_degrees() < 0.2, "rotation differs by {:.4} deg", delta.angle().to_degrees());
    assert!(delta.translation.norm() < 0.1, "translation differs by {:.4} mm", delta.translation.norm());
}

/// An open box (bottom deleted) with a tie between its top and the missing bottom: the rim is the
/// base, so the box ends up open side down.
#[test]
fn p9_auto_open_box_stands_on_its_rim() {
    let mut clean = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), 12);
    clean.weld(1e-9);
    let keep: Vec<u32> = (0..clean.triangle_count())
        .filter(|&i| {
            !(clean.face_centroid(i).is_some_and(|c| c.z < 1e-6) && clean.face_normal(i).is_some_and(|n| n.z < -0.9))
        })
        .map(|i| i as u32)
        .collect();
    let open = noisy(clean, 0.03, 9).submesh(&keep);
    let pose = &poses()[3];
    let mut s = session_with(&open, pose);
    let out = s.execute("align.auto", json!({"object": "part", "apply": true})).unwrap();
    assert_eq!(out["method"], "open_rim", "{out}");
    let (id, _) = s.doc.find_by_name("part").unwrap();
    let b = s.doc.mesh(id).unwrap().mesh.bbox();
    assert!(b.min.z.abs() < 0.2 && (b.max.z - 20.0).abs() < 0.2, "box must stand open side down: {b:?}");
}

/// A smooth blob has no flat face: the proposal is the principal frame at low confidence, not an
/// error.
#[test]
fn p9_auto_organic_blob_is_low_confidence() {
    let mut m = synth::sphere(Point3::origin(), 20.0, 48, 24);
    for p in &mut m.positions {
        let lump = 1.0 + 0.12 * (0.21 * p.x).sin() * (0.17 * p.y).cos() + 0.08 * (0.3 * p.z).sin();
        *p = Point3::new(p.x * 1.6 * lump, p.y * lump, p.z * 0.7 * lump);
    }
    let m = noisy(m, 0.03, 4);
    let mut s = session_with(&m, &poses()[2]);
    let out = s.execute("align.auto", json!({"object": "part"})).unwrap();
    assert_eq!(out["method"], "principal_fallback", "{out}");
    assert_eq!(out["confident"], false, "{out}");
    let c = out["confidence"].as_f64().unwrap();
    eprintln!("blob: method {}, confidence {c:.3}", out["method"]);
    assert!(c < 0.5, "confidence {c} must be below the documented threshold 0.5");
}

/// `largest_face` puts the base exactly on z = 0 and leaves the tilted top tilted; `all_faces`
/// shares the 1 degree between the faces by area, so the base is no longer flat on the table.
#[test]
fn p9_auto_levelling_modes_differ_on_a_tilted_top() {
    let mut m = synth::bracket();
    let tan = 1.0_f64.to_radians().tan();
    for p in &mut m.positions {
        p.z += p.z / 20.0 * (p.x - 30.0) * tan; // bottom stays flat, the top tilts 1 degree about y
    }
    let m = noisy(m, 0.03, 6);
    let pose = &poses()[0];
    let base_n = -Vec3::z();
    let top_n = Vec3::new(-tan, 0.0, 1.0);
    let measure = |mode: &str| {
        let mut s = session_with(&m, pose);
        let out = s.execute("align.auto", json!({"object": "part", "mode": mode})).unwrap();
        let net = matrix_of(&out).then_after(pose);
        (tilt_deg(&net.apply_vec(&base_n), &base_n), tilt_deg(&net.apply_vec(&top_n), &Vec3::z()), out)
    };
    let (lf_base, lf_top, lf_out) = measure("largest_face");
    let (af_base, af_top, af_out) = measure("all_faces");
    eprintln!("largest_face: base {lf_base:.3} top {lf_top:.3}; all_faces: base {af_base:.3} top {af_top:.3}");
    assert!(lf_base < 0.05, "largest_face keeps the base flat: {lf_base}");
    assert!((lf_top - 1.0).abs() < 0.1, "largest_face leaves the top tilted 1 degree: {lf_top}");
    assert!(af_base > 0.15 && af_base < 0.6, "all_faces tilts the base by a compromise: {af_base}");
    assert!(af_top < lf_top - 0.15, "all_faces brings the top closer to level: {af_top} vs {lf_top}");
    assert_eq!(lf_out["mode"], "largest_face");
    assert_eq!(af_out["mode"], "all_faces");
}

/// `apply: false` (the default) proposes and changes nothing; `apply: true` moves the part in one
/// undo step.
#[test]
fn p9_auto_proposes_then_applies_in_one_undo_step() {
    let m = noisy(synth::bracket(), 0.03, 2);
    let mut s = session_with(&m, &poses()[4]);
    let before = mesh_hash(&s, "part");
    let proposal = s.execute("align.auto", json!({"object": "part"})).unwrap();
    assert_eq!(proposal["applied"], false);
    assert_eq!(mesh_hash(&s, "part"), before, "a proposal must not touch the document");
    assert!(proposal["translation_mm"].as_f64().unwrap() > 100.0, "{proposal}");
    assert!(proposal["angle_degrees"].as_f64().unwrap() > 1.0);
    let applied = s.execute("align.auto", json!({"object": "part", "apply": true})).unwrap();
    assert_eq!(applied["applied"], true);
    assert_eq!(applied["matrix"], proposal["matrix"], "the proposal is what apply does");
    assert_ne!(mesh_hash(&s, "part"), before);
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(mesh_hash(&s, "part"), before, "one undo restores the part");
    // the preview of the GUI sees the moved part without touching this session
    let preview = s.preview("align.auto", json!({"object": "part", "apply": true})).unwrap();
    assert_ne!(mesh_hash(&preview, "part"), before);
    assert_eq!(mesh_hash(&s, "part"), before);
}

#[test]
fn p9_auto_rejects_bad_input_without_panicking() {
    let mut s = Session::new();
    assert!(s.execute("align.auto", json!({"object": "nothing"})).is_err());
    s.doc.add_mesh("empty", Mesh::default());
    assert!(s.execute("align.auto", json!({"object": "empty"})).is_err());
    let m = noisy(synth::bracket(), 0.0, 1);
    s.doc.add_mesh("part", m);
    assert!(s.execute("align.auto", json!({"object": "part", "mode": "sideways"})).is_err());
}

/// Meshes no scanner would write: non-finite and huge coordinates, collapsed triangles, one
/// triangle, a flat sheet. Every command answers with a value or an error, never a panic.
#[test]
fn p9_commands_survive_hostile_meshes() {
    let tri = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| Mesh {
        positions: vec![Point3::from(a), Point3::from(b), Point3::from(c)],
        triangles: vec![[0, 1, 2]],
    };
    let mut nan = synth::bracket();
    nan.positions[3] = Point3::new(f64::NAN, 0.0, 0.0);
    let mut huge = synth::bracket();
    huge.scale(1e300);
    let mut far = synth::bracket();
    far.positions[7] = Point3::new(f64::INFINITY, 1.0, 1.0);
    let hostile = vec![
        ("nan", nan),
        ("huge", huge),
        ("inf", far),
        ("one triangle", tri([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])),
        ("collapsed", tri([0.0; 3], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0])),
        ("sheet", synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 50.0, 30.0, 8, 8)),
        ("bad index", Mesh { positions: vec![Point3::origin()], triangles: vec![[0, 5, 9]] }),
    ];
    for (name, mesh) in hostile {
        for cmd in ["align.auto", "align.symmetry"] {
            let mut s = Session::new();
            s.doc.add_mesh("part", mesh.clone());
            let r = s.execute(cmd, json!({"object": "part", "apply": true}));
            assert!(!matches!(r, Err(surfacecraft_engine::EngineError::Internal(_))), "{cmd} panicked on {name}");
        }
        let mut s = Session::new();
        s.doc.add_mesh("part", mesh);
        let r = s.execute(
            "mesh.scale_from_length",
            json!({"object": "part", "from": [0, 0, 0], "to": [1, 0, 0], "length": 2.0}),
        );
        assert!(!matches!(r, Err(surfacecraft_engine::EngineError::Internal(_))), "scale panicked on {name}");
    }
}

// ---- align.symmetry ---------------------------------------------------------------------

fn mirrored_part() -> Mesh {
    // symmetric about x = 30 only: a block with an off-centre boss at x = 30
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), 12);
    m.append(&synth::cylinder(Point3::new(30.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 48, 6));
    m
}

/// Gate: the mirror plane of a part under a known pose lands on x = 0 within 0.02 mm and 0.05°.
#[test]
fn gate_p9_symmetry_plane_to_x0() {
    let sigma = 0.02;
    let mesh = noisy(mirrored_part(), sigma, 7);
    for (k, pose) in poses().iter().take(3).enumerate() {
        let mut s = session_with(&mesh, pose);
        let proposal = s.execute("align.symmetry", json!({"object": "part"})).unwrap();
        assert_eq!(proposal["applied"], false);
        let before = mesh_hash(&s, "part");
        let out = s.execute("align.symmetry", json!({"object": "part", "apply": true})).unwrap();
        assert_ne!(mesh_hash(&s, "part"), before);
        let net = matrix_of(&out).then_after(pose);
        let n = net.apply_vec(&Vec3::x());
        let tilt = line_tilt_deg(&n, &Vec3::x());
        let off = net.apply(&Point3::new(30.0, 7.0, 3.0)).x;
        assert!(tilt < 0.05, "pose {k}: plane normal {tilt:.4} deg off x");
        assert!(off.abs() < 0.02, "pose {k}: plane at x = {off:.4}");
        let rms = out["residual_rms"].as_f64().unwrap();
        eprintln!("symmetry pose {k}: normal {tilt:.4} deg, offset {off:.5} mm, residual {rms:.4}");
        assert!(rms > 0.0 && rms < 3.0 * sigma, "pose {k}: residual {rms} should be near the noise {sigma}");
        // the other axes stay close to where they were: a shortest-arc motion
        assert!(matrix_of(&out).angle().to_degrees() < 179.0);
    }
}

#[test]
fn p9_symmetry_other_planes_and_errors() {
    let mesh = noisy(mirrored_part(), 0.02, 7);
    let mut s = session_with(&mesh, &poses()[1]);
    let out = s.execute("align.symmetry", json!({"object": "part", "plane": "xz", "apply": true})).unwrap();
    let net = matrix_of(&out).then_after(&poses()[1]);
    // the mirror plane normal (original x) is now y
    assert!(line_tilt_deg(&net.apply_vec(&Vec3::x()), &Vec3::y()) < 0.05);
    assert!(s.execute("align.symmetry", json!({"object": "part", "plane": "diagonal"})).is_err());
    s.doc.add_mesh("empty", Mesh::default());
    assert!(s.execute("align.symmetry", json!({"object": "empty"})).is_err());
}

// ---- mesh.scale_from_length -------------------------------------------------------------

/// Gate: a scan scaled by 0.3731 returns to true size within 1e-9 relative, given the two points.
#[test]
fn gate_p9_scale_from_length_restores_size() {
    let truth = synth::bracket();
    let f = 0.3731;
    let mut s = Session::new();
    let mut small = truth.clone();
    small.scale(f);
    s.doc.add_mesh("scan", small);
    // two corners of the bounding box: the true distance is known exactly
    let (a, b) = (truth.bbox().min, truth.bbox().max);
    let length = (b - a).norm();
    let out = s
        .execute(
            "mesh.scale_from_length",
            json!({"object": "scan", "from": [a.x * f, a.y * f, a.z * f], "to": [b.x * f, b.y * f, b.z * f],
                   "length": length, "origin": [0.0, 0.0, 0.0]}),
        )
        .unwrap();
    assert!((out["factor"].as_f64().unwrap() * f - 1.0).abs() < 1e-9, "{out}");
    let (id, _) = s.doc.find_by_name("scan").unwrap();
    let m = s.doc.mesh(id).unwrap().mesh.clone();
    let worst = m
        .positions
        .iter()
        .zip(&truth.positions)
        .map(|(p, q)| (p - q).norm() / q.coords.norm().max(1.0))
        .fold(0.0, f64::max);
    assert!(worst < 1e-9, "relative error {worst}");
    // about the default origin (the centroid) the size is right too, and the centroid stays put
    let mut s2 = Session::new();
    let mut small2 = truth.clone();
    small2.scale(f);
    let c0 = small2.centroid();
    s2.doc.add_mesh("scan", small2);
    s2.execute(
        "mesh.scale_from_length",
        json!({"object": "scan", "from": [a.x * f, a.y * f, a.z * f], "to": [b.x * f, b.y * f, b.z * f], "length": length}),
    )
    .unwrap();
    let (id2, _) = s2.doc.find_by_name("scan").unwrap();
    let m2 = s2.doc.mesh(id2).unwrap().mesh.clone();
    let size = m2.bbox().size();
    let want = truth.bbox().size();
    assert!(((size - want).norm() / want.norm()) < 1e-9);
    assert!((m2.centroid() - c0).norm() < 1e-9);
}

#[test]
fn p9_scale_from_length_rejects_hostile_input() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", synth::bracket());
    let before = mesh_hash(&s, "scan");
    let ok = json!({"object": "scan", "from": [0, 0, 0], "to": [10, 0, 0], "length": 20.0});
    let mut bad = |patch: Value| {
        let mut p = ok.clone();
        for (k, v) in patch.as_object().unwrap() {
            p[k] = v.clone();
        }
        assert!(s.execute("mesh.scale_from_length", p.clone()).is_err(), "must reject {p}");
    };
    bad(json!({"length": 0.0}));
    bad(json!({"length": -5.0}));
    bad(json!({"length": null}));
    bad(json!({"length": "ten"}));
    bad(json!({"to": [0, 0, 0]}));
    bad(json!({"to": [1e-300, 0, 0]}));
    bad(json!({"to": [1e-9, 0, 0], "length": 1e308}));
    bad(json!({"from": [0, 0]}));
    bad(json!({"origin": [0.0, 0.0, f64::INFINITY]}));
    bad(json!({"object": "nothing"}));
    assert_eq!(mesh_hash(&s, "scan"), before, "a rejected scale leaves the mesh alone");
    // the legal call works and is one undo step
    let out = s.execute("mesh.scale_from_length", ok).unwrap();
    assert!((out["factor"].as_f64().unwrap() - 2.0).abs() < 1e-12);
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(mesh_hash(&s, "scan"), before);
}

/// Measurement, not a gate: `align.auto` on a ~1.5 M triangle mesh. Run with
/// `cargo test --release -p surfacecraft-engine --test p9_auto_align perf -- --ignored --nocapture`.
#[test]
#[ignore = "measurement"]
fn perf_auto_align_large_mesh() {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), 350);
    m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, 600, 200));
    let m = noisy(m, 0.03, 1);
    let mut s = session_with(&m, &poses()[0]);
    let t = std::time::Instant::now();
    let out = s.execute("align.auto", json!({"object": "part"})).unwrap();
    eprintln!("align.auto on {} triangles: {:?} ({})", m.triangle_count(), t.elapsed(), out["method"]);
}
