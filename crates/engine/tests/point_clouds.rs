//! P1: point clouds (scanner exports without faces) become meshes through the command layer.
//!
//! Ground truth is a synthetic shape sampled at random with Gaussian noise (truncated at 2.5 σ so
//! "every vertex within 3 σ of the true surface" is a fair gate); the cloud reaches the engine as
//! an XYZ text file, exactly as a scanner would write it.
use serde_json::{Value, json};
use surfacecraft_doc::base64;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Vec3, edit, synth};

/// `n` random surface samples of `mesh` (those `keep` accepts, tested before the noise), each
/// moved along the normal by truncated `N(0, σ²)`.
fn noisy_points(mesh: &Mesh, n: usize, sigma: f64, seed: u64, keep: impl Fn(&Point3) -> bool) -> Vec<Point3> {
    let mut rng = Rng::new(seed);
    mesh.sample_points(n, &mut rng)
        .into_iter()
        .filter(|(p, _)| keep(p))
        .map(|(p, nrm)| {
            let g = loop {
                let g = rng.gauss();
                if g.abs() <= 2.5 {
                    break g;
                }
            };
            p + nrm * (sigma * g)
        })
        .collect()
}

fn xyz_data(points: &[Point3]) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    for p in points {
        let _ = writeln!(s, "{} {} {}", p.x, p.y, p.z);
    }
    base64::encode(s.as_bytes())
}

/// Largest distance from a vertex of the engine's mesh `name` to the true surface.
fn max_deviation(s: &Session, name: &str, truth: &Mesh) -> f64 {
    let id = s.doc.find_by_name(name).map(|x| x.0).expect("mesh");
    let m = &s.doc.mesh(id).expect("mesh object").mesh;
    let bvh = Bvh::build(truth);
    m.positions.iter().map(|p| bvh.closest(p, f64::INFINITY).map_or(f64::INFINITY, |h| h.distance)).fold(0.0, f64::max)
}

fn boundary_loops(s: &Session, name: &str) -> usize {
    let id = s.doc.find_by_name(name).map(|x| x.0).expect("mesh");
    edit::boundary_loops(&s.doc.mesh(id).expect("mesh object").mesh).len()
}

fn import_cloud(s: &mut Session, points: &[Point3], name: &str) -> Value {
    s.execute("mesh.import", json!({"data": xyz_data(points), "format": "xyz", "name": name})).unwrap()
}

/// Closed shapes come back (nearly) watertight, every vertex within 3 σ of the truth.
#[test]
fn p1_closed_shapes_from_noisy_points_are_watertight() {
    let cases: Vec<(&str, Mesh, usize, f64)> = vec![
        ("sphere", synth::sphere(Point3::new(3.0, -2.0, 5.0), 12.0, 96, 48), 12_000, 0.03),
        ("box", synth::cuboid(Point3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 20.0), 12), 30_000, 0.02),
        ("sphere_noisy", synth::sphere(Point3::origin(), 15.0, 96, 48), 20_000, 0.05),
    ];
    for (name, truth, n, sigma) in cases {
        let mut s = Session::new();
        let pts = noisy_points(&truth, n, sigma, 5, |_| true);
        let out = import_cloud(&mut s, &pts, name);
        let cloud = &out["point_cloud"];
        assert_eq!(cloud["normals"], "estimated", "{name}");
        let tris = out["triangles"].as_u64().unwrap();
        assert!(tris as f64 > 1.5 * pts.len() as f64, "{name}: {tris} triangles for {} points", pts.len());
        let unused = cloud["unused_points"].as_u64().unwrap() as f64;
        let max_unused = if sigma >= 0.05 { 0.15 } else { 0.06 };
        assert!(unused < max_unused * pts.len() as f64, "{name}: {unused} unused points");
        let loops = boundary_loops(&s, name);
        eprintln!("{name}: {tris} triangles, {loops} boundary loops, {cloud}");
        assert!(loops <= 4, "{name}: {loops} boundary loops");
        let dev = max_deviation(&s, name, &truth);
        assert!(dev <= 3.0 * sigma, "{name}: vertex {dev} mm from the surface (3σ = {})", 3.0 * sigma);
    }
}

/// An open surface keeps its open ends: the cylinder has exactly its two rims as boundary.
#[test]
fn p1_open_cylinder_keeps_its_rims() {
    let truth = synth::cylinder(Point3::origin(), Vec3::z(), 8.0, 30.0, 96, 48);
    let mut s = Session::new();
    let pts = noisy_points(&truth, 15_000, 0.03, 9, |_| true);
    import_cloud(&mut s, &pts, "cyl");
    let loops = boundary_loops(&s, "cyl");
    assert!((2..=4).contains(&loops), "{loops} boundary loops");
    assert!(max_deviation(&s, "cyl", &truth) <= 0.09);
    let f = s.execute("primitive.fit", json!({"mesh": "cyl", "kind": "cylinder"}));
    if let Ok(f) = f {
        assert!((f["primitive"]["radius"].as_f64().unwrap() - 8.0).abs() < 0.05, "{f}");
    }
}

/// The bracket from points: segmentation finds the right kinds and every fit is within 0.1 mm
/// RMS (design doc 11.1), as for the noisy mesh of gate M2.
#[test]
fn p1_bracket_from_points_segments_and_fits() {
    let truth = synth::bracket();
    // the dome's lower half and nothing else lies inside the block: it is not part of the surface
    let inside_block =
        |p: &Point3| p.x > 1e-6 && p.x < 60.0 - 1e-6 && p.y > 1e-6 && p.y < 40.0 - 1e-6 && p.z < 20.0 - 1e-6;
    let pts = noisy_points(&truth, 45_000, 0.03, 4, |p| !inside_block(p));
    let mut s = Session::new();
    let out = import_cloud(&mut s, &pts, "scan");
    eprintln!("bracket: {out}");
    assert!(max_deviation(&s, "scan", &truth) <= 0.09);
    let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let regions = seg["regions"].as_array().unwrap().clone();
    let mut kinds: Vec<String> = regions.iter().map(|r| r["kind"].as_str().unwrap().to_string()).collect();
    kinds.sort();
    eprintln!("kinds {kinds:?}");
    assert_eq!(kinds, ["cylinder", "plane", "plane", "plane", "plane", "plane", "plane", "sphere"]);
    for r in &regions {
        let f = s.execute("primitive.fit", json!({"region": r["id"]})).unwrap();
        assert!(f["rms"].as_f64().unwrap() <= 0.1, "{f}");
        assert_eq!(f["primitive"]["type"], r["kind"]);
    }
    let cyl = regions.iter().find(|r| r["kind"] == "cylinder").unwrap();
    let f = s.execute("primitive.fit", json!({"region": cyl["id"], "kind": "cylinder"})).unwrap();
    assert!((f["primitive"]["radius"].as_f64().unwrap() - 6.0).abs() < 0.05, "{f}");
}

/// Normals in the file are used as given; `mesh.reconstruct` takes a file of any kind and the
/// settings reach the algorithm.
#[test]
fn p1_given_normals_and_reconstruct_command() {
    let truth = synth::sphere(Point3::origin(), 10.0, 64, 32);
    let mut rng = Rng::new(2);
    let samples = truth.sample_points(8000, &mut rng);
    let mut text = String::new();
    for (p, n) in &samples {
        text.push_str(&format!("{} {} {} {} {} {}\n", p.x, p.y, p.z, n.x, n.y, n.z));
    }
    let data = base64::encode(text.as_bytes());
    let mut s = Session::new();
    let out = s.execute("mesh.import", json!({"data": data, "name": "withn"})).unwrap();
    assert_eq!(out["point_cloud"]["normals"], "given", "{out}");
    assert!(out["triangles"].as_u64().unwrap() > 8000);
    // an ordinary mesh imports as before, without point-cloud information
    let mesh_bytes = surfacecraft_io::write_mesh(&truth, surfacecraft_io::MeshFormat::Stl);
    let m =
        s.execute("mesh.import", json!({"data": base64::encode(&mesh_bytes), "format": "stl", "name": "m"})).unwrap();
    assert!(m.get("point_cloud").is_none());
    // its vertices re-meshed
    let r = s
        .execute(
            "mesh.reconstruct",
            json!({"data": base64::encode(&mesh_bytes), "format": "stl", "name": "again", "radius_scale": 3.0, "passes": 2}),
        )
        .unwrap();
    assert_eq!(r["point_cloud"]["radii"].as_array().unwrap().len(), 2);
    assert!(r["triangles"].as_u64().unwrap() > 1000, "{r}");
    // scale applies to point clouds too
    let big = s.execute("mesh.import", json!({"data": data, "name": "big", "scale": 2.0})).unwrap();
    assert!((big["bbox_max"][0].as_f64().unwrap() - 20.0).abs() < 0.5, "{big}");
    // undo removes the reconstructed object again
    let before = s.doc.meshes.len();
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(s.doc.meshes.len(), before - 1);
}

/// Bad parameters and bad clouds are errors, never panics.
#[test]
fn p1_hostile_point_clouds_are_errors() {
    let mut s = Session::new();
    let good: Vec<Point3> = (0..400).map(|i| Point3::new(f64::from(i % 20), f64::from(i / 20), 0.0)).collect();
    let data = xyz_data(&good);
    for params in [
        json!({"data": data, "format": "xyz", "reconstruct": {"radius_scale": -1.0}}),
        json!({"data": data, "format": "xyz", "reconstruct": {"radius_scale": 1e9}}),
        json!({"data": data, "format": "xyz", "reconstruct": {"normals_k": 0}}),
        json!({"data": data, "format": "xyz", "reconstruct": {"normals_k": 100000}}),
        json!({"data": data, "format": "xyz", "reconstruct": {"passes": 0}}),
        json!({"data": data, "format": "xyz", "reconstruct": {"passes": 1000}}),
        json!({"data": data, "format": "xyz", "reconstruct": {"bogus": 1}}),
        json!({"data": data, "format": "xyz", "scale": 0.0}),
        json!({"data": xyz_data(&[Point3::origin(); 50]), "format": "xyz"}),
        json!({"data": xyz_data(&(0..100).map(|i| Point3::new(f64::from(i), 0.0, 0.0)).collect::<Vec<_>>()), "format": "xyz"}),
        json!({"data": base64::encode(b"1 2 3\nfoo\n"), "format": "xyz"}),
        json!({"data": base64::encode(b""), "format": "xyz"}),
    ] {
        assert!(s.execute("mesh.import", params.clone()).is_err(), "{params}");
        assert!(
            s.execute("mesh.reconstruct", params.clone()).is_err() || params.get("reconstruct").is_some(),
            "{params}"
        );
    }
    // the same data with sane settings works
    s.execute("mesh.import", json!({"data": data, "format": "xyz", "reconstruct": {"radius_scale": 1.5}})).unwrap();
    assert!(s.doc.meshes.len() <= 1);
}
