//! M1 through the command layer: the same calls the GUI, CLI and MCP make.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Point3, Transform, Vec3, synth};

fn truth() -> Transform {
    Transform::rotation_axis_angle(&Vec3::new(0.2, 0.9, -0.4), 4.0_f64.to_radians())
        .then_after(&Transform::translation(Vec3::new(1.5, -2.0, 1.0)))
}

fn matrix(t: &Transform) -> Value {
    serde_json::to_value(t.to_rows()).unwrap()
}

fn mesh_points(s: &Session, name: &str) -> Vec<Point3> {
    let (id, _) = s.doc.find_by_name(name).unwrap();
    s.doc.mesh(id).unwrap().mesh.positions.clone()
}

fn max_dev(a: &[Point3], b: &[Point3]) -> f64 {
    a.iter().zip(b).map(|(p, q)| (p - q).norm()).fold(0.0, f64::max)
}

/// Gate M1: ICP through `align.icp` recovers a known motion on noise-free data to ≤ 1e-6 mm
/// and reports convergence below 1e-4 mm per iteration.
#[test]
fn gate_m1_icp_command_recovers_known_motion() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "name": "reference"})).unwrap();
    s.execute("mesh.synth", json!({"shape": "bracket", "name": "scan"})).unwrap();
    s.execute("mesh.transform", json!({"object": "scan", "matrix": matrix(&truth().inverse())})).unwrap();
    let out = s.execute("align.icp", json!({"object": "scan", "target": "reference"})).unwrap();
    assert_eq!(out["converged"], true, "{out}");
    assert!(out["last_delta"].as_f64().unwrap() < 1e-4);
    let err = max_dev(&mesh_points(&s, "scan"), &mesh_points(&s, "reference"));
    assert!(err <= 1e-6, "scan vertices are {err} mm from the reference after ICP");
}

/// Gate M1: N-point (Kabsch) through `align.n_point` is exact on noise-free picks.
#[test]
fn gate_m1_n_point_command_recovers_known_motion() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "name": "scan"})).unwrap();
    let reference = mesh_points(&s, "scan");
    let t = truth();
    s.execute("mesh.transform", json!({"object": "scan", "matrix": matrix(&t)})).unwrap();
    let idx = [0usize, 17, 230, 511, 900];
    let from: Vec<[f64; 3]> = idx.iter().map(|&i| t.apply(&reference[i]).into()).collect();
    let to: Vec<[f64; 3]> = idx.iter().map(|&i| reference[i].into()).collect();
    let out = s.execute("align.n_point", json!({"object": "scan", "from": from, "to": to})).unwrap();
    assert!(out["rms"].as_f64().unwrap() < 1e-9);
    assert!(max_dev(&mesh_points(&s, "scan"), &reference) <= 1e-6);
}

/// 3-2-1 from planes fitted to triangles of the displaced block's faces.
#[test]
fn datum_from_triangles_aligns_block() {
    let mut s = Session::new();
    let block = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), 4);
    // triangles of the z=0, x=0 and y=0 faces, chosen by their normals before moving
    let pick = |n: Vec3| -> Vec<u32> {
        (0..block.triangle_count())
            .filter(|&i| block.face_normal(i).unwrap().dot(&n) > 0.99)
            .map(|i| i as u32)
            .collect()
    };
    let (bottom, left, front) = (pick(-Vec3::z()), pick(-Vec3::x()), pick(-Vec3::y()));
    let data = surfacecraft_doc::base64::encode(&surfacecraft_io::write_mesh(&block, surfacecraft_io::MeshFormat::Ply));
    s.execute("mesh.import", json!({"data": data, "format": "ply", "name": "block"})).unwrap();
    s.execute("mesh.transform", json!({"object": "block", "matrix": matrix(&truth())})).unwrap();
    // outward normals of these faces are −Z, −X, −Y; the datum frame flips them to +Z, +X
    let out = s
        .execute(
            "align.datum",
            json!({"object": "block",
                "primary": {"plane_from_triangles": bottom},
                "secondary": {"plane_from_triangles": left},
                "tertiary": {"plane_from_triangles": front}}),
        )
        .unwrap();
    assert_eq!(out["applied"], true);
    let b = s.doc.mesh(s.doc.find_by_name("block").unwrap().0).unwrap().mesh.bbox();
    // the block now has a corner at the origin and edges along the axes (orientation flipped)
    let size = b.size();
    let mut dims = [size.x, size.y, size.z];
    dims.sort_by(f64::total_cmp);
    assert!(
        (dims[0] - 20.0).abs() < 1e-6 && (dims[1] - 40.0).abs() < 1e-6 && (dims[2] - 60.0).abs() < 1e-6,
        "{size:?}"
    );
    assert!(b.min.coords.abs().min() < 1e-6 || b.max.coords.abs().min() < 1e-6);
}

#[test]
fn principal_and_center_and_undo() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [60, 30, 10], "center": [500, 500, 500]})).unwrap();
    s.execute("align.center", json!({"object": "box"})).unwrap();
    let b = s.doc.bbox();
    assert!(b.center().coords.norm() < 1e-9);
    // a proposal takes no undo step: one undo takes back the centring
    s.execute("align.principal", json!({"object": "box", "apply": false})).unwrap();
    s.execute("edit.undo", Value::Null).unwrap();
    assert!((s.doc.bbox().center().x - 530.0).abs() < 1e-9);
}
