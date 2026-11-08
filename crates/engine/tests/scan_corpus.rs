//! P1 corpus level 1 in CI: CAD parts built with the kernel, tessellated and made to look like raw
//! scans (`synth::scanify`: dense non-uniform sampling, noise, holes, spikes, debris), then
//! cleaned, segmented and fitted through the command layer. The ground truth is the CAD model,
//! so recovered dimensions are checked exactly (design doc §11.1 thresholds unchanged).
//!
//! `tangency-labs/models` holds no models yet (checked 2026-11-08), so the CC0 level-1 corpus is
//! generated here rather than taken from a separate repository; nothing binary is committed.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Rng, synth};

/// Vertical edges of a body (both ends share x, y and differ in z).
fn vertical_edges(s: &Session, body: &str) -> Vec<usize> {
    let b = s.doc.bodies.iter().find(|b| b.name == body).unwrap();
    b.brep
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            let (p, q) = (b.brep.vertices[e.vertices[0] as usize].point, b.brep.vertices[e.vertices[1] as usize].point);
            (p[0] - q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9 && (p[2] - q[2]).abs() > 1.0
        })
        .map(|(i, _)| i)
        .collect()
}

/// Tessellates `body`, scanifies it and adds it as mesh "scan"; returns the clean CAD mesh.
fn scan_of(s: &mut Session, body: &str, defects: &synth::ScanDefects, seed: u64) -> Mesh {
    s.execute("body.to_mesh", json!({"object": body, "chord": 0.01, "name": "cad"})).unwrap();
    let id = s.doc.find_by_name("cad").unwrap().0;
    let mut cad = (*s.doc.mesh(id).unwrap().mesh).clone();
    cad.weld(1e-6);
    let scan = synth::scanify(&cad, defects, &mut Rng::new(seed));
    s.doc.add_mesh("scan", scan);
    cad
}

fn regions(out: &Value) -> Vec<Value> {
    out["regions"].as_array().unwrap().clone()
}

fn kinds(rs: &[Value]) -> Vec<String> {
    let mut k: Vec<String> = rs.iter().map(|r| r["kind"].as_str().unwrap_or("none").to_string()).collect();
    k.sort();
    k
}

/// Clean-up: debris and spikes go, holes are filled, and the result stays on the CAD surface.
fn clean_and_check(s: &mut Session, cad: &Mesh, sigma: f64) {
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    assert!(c["removed_triangles"].as_u64().unwrap() > 0, "{c}");
    assert!(c["holes_filled"].as_u64().unwrap() >= 1, "{c}");
    assert_eq!(c["holes_left"], 0, "{c}");
    let id = s.doc.find_by_name("scan").unwrap().0;
    let m = s.doc.mesh(id).unwrap().mesh.clone();
    assert!(surfacecraft_geom::edit::boundary_loops(&m).is_empty(), "holes remain");
    assert_eq!(surfacecraft_geom::edit::components(&m).len(), 1, "debris remains");
    // every vertex within 6 σ of the CAD surface: no spike survived (filled holes included)
    let bvh = Bvh::build(cad);
    let worst =
        surfacecraft_deviation::against_mesh(&m.positions, &bvh, 10.0).iter().map(|d| d.abs()).fold(0.0, f64::max);
    eprintln!("clean: {c}, worst vertex {worst:.4} mm");
    assert!(worst < 6.0 * sigma, "a vertex is {worst} mm off the CAD surface");
    let n = s.execute("deviation.noise", json!({"object": "scan"})).unwrap();
    let est = n["sigma"].as_f64().unwrap();
    eprintln!("noise estimate {est:.4} (true {sigma})");
    assert!((0.6 * sigma..1.5 * sigma).contains(&est), "noise estimate {est}");
}

/// Part A: a 40 × 30 × 15 block with its four vertical edges filleted r = 5 (6 planes, 4
/// cylinders), scanned with a density ramp ×2, σ = 0.03 mm, holes, spikes and debris.
#[test]
fn p1_scanned_filleted_block() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [40.0, 30.0, 15.0], "name": "block"})).unwrap();
    let edges = vertical_edges(&s, "block");
    assert_eq!(edges.len(), 4);
    s.execute("body.fillet", json!({"object": "block", "edges": edges, "size": 5.0})).unwrap();
    let d = synth::ScanDefects { spacing: 0.7, noise: 0.03, ..synth::ScanDefects::default() };
    let cad = scan_of(&mut s, "block", &d, 11);
    clean_and_check(&mut s, &cad, 0.03);
    let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let rs = regions(&seg);
    let big: Vec<Value> = rs.iter().filter(|r| r["triangles"].as_u64().unwrap() > 400).cloned().collect();
    eprintln!("block regions: {} (big {}), kinds {:?}", rs.len(), big.len(), kinds(&big));
    assert_eq!(
        kinds(&big),
        ["cylinder", "cylinder", "cylinder", "cylinder", "plane", "plane", "plane", "plane", "plane", "plane"]
    );
    for r in &big {
        let f = s.execute("primitive.fit", json!({"region": r["id"]})).unwrap();
        assert!(f["rms"].as_f64().unwrap() <= 0.1, "{f}");
        if r["kind"] == "cylinder" {
            let rad = f["primitive"]["radius"].as_f64().unwrap();
            // a quarter arc under σ = 0.03 noise pins the radius less tightly than a full turn:
            // 2 % (this is a dimension check of the test, not a design doc §11.1 threshold)
            eprintln!("fillet radius {rad:.4}");
            assert!((rad - 5.0).abs() < 0.1, "fillet radius {rad}");
        }
    }
}

/// Part B: a cylinder r = 12, h = 30 with a 3 mm chamfer on the top edge (cylinder, cone, two
/// planes), scanned the same way.
#[test]
fn p1_scanned_chamfered_cylinder() {
    let mut s = Session::new();
    s.execute("body.cylinder", json!({"radius": 12.0, "height": 30.0, "name": "pin"})).unwrap();
    let b = s.doc.bodies.iter().find(|b| b.name == "pin").unwrap();
    // the top circle: an edge whose vertices all sit at z = 30
    let top = b
        .brep
        .edges
        .iter()
        .position(|e| e.vertices.iter().all(|&v| (b.brep.vertices[v as usize].point[2] - 30.0).abs() < 1e-9))
        .unwrap();
    s.execute("body.chamfer", json!({"object": "pin", "edges": [top], "size": 3.0})).unwrap();
    let d = synth::ScanDefects { spacing: 0.7, noise: 0.03, ..synth::ScanDefects::default() };
    let cad = scan_of(&mut s, "pin", &d, 5);
    clean_and_check(&mut s, &cad, 0.03);
    let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let rs = regions(&seg);
    let big: Vec<Value> = rs.iter().filter(|r| r["triangles"].as_u64().unwrap() > 400).cloned().collect();
    eprintln!("pin regions: {} (big {}), kinds {:?}", rs.len(), big.len(), kinds(&big));
    assert_eq!(kinds(&big), ["cone", "cylinder", "plane", "plane"]);
    for r in &big {
        let f = s.execute("primitive.fit", json!({"region": r["id"]})).unwrap();
        assert!(f["rms"].as_f64().unwrap() <= 0.1, "{f}");
        match r["kind"].as_str().unwrap() {
            "cylinder" => {
                let rad = f["primitive"]["radius"].as_f64().unwrap();
                assert!((rad - 12.0).abs() < 0.03, "radius {rad}");
            }
            "cone" => {
                let a = f["primitive"]["half_angle"].as_f64().unwrap().to_degrees();
                assert!((a - 45.0).abs() < 0.5, "chamfer angle {a}");
            }
            _ => {}
        }
    }
}

/// The mesh-edit commands one by one: decimation keeps the shape within its error bound and
/// drops the regions of the mesh; smoothing lowers the noise; flat and smooth hole fills.
#[test]
fn p1_mesh_edit_commands() {
    let mut s = Session::new();
    s.execute(
        "mesh.synth",
        json!({"shape": "sphere", "size": [20.0], "resolution": 128, "noise": 0.05, "name": "ball"}),
    )
    .unwrap();
    s.execute("region.auto_segment", json!({"mesh": "ball"})).unwrap();
    assert!(!s.doc.regions.is_empty());
    let d = s.execute("mesh.decimate", json!({"object": "ball", "ratio": 0.25})).unwrap();
    assert!(d["triangles"].as_u64().unwrap() <= d["triangles_before"].as_u64().unwrap() / 4 + 2, "{d}");
    assert!(d["regions_dropped"].as_u64().unwrap() >= 1);
    assert!(s.doc.regions.is_empty());
    let id = s.doc.find_by_name("ball").unwrap().0;
    let err = |s: &Session| {
        let m = &s.doc.mesh(id).unwrap().mesh;
        let v: Vec<f64> = m.positions.iter().map(|p| p.coords.norm() - 20.0).collect();
        (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt()
    };
    let before = err(&s);
    let sm = s.execute("mesh.smooth", json!({"object": "ball", "iterations": 5})).unwrap();
    assert!(sm["max_displacement"].as_f64().unwrap() > 0.0);
    assert!(err(&s) < before, "smoothing did not reduce the noise");
    // undo restores the decimated, unsmoothed mesh
    s.execute("edit.undo", Value::Null).unwrap();
    assert!((err(&s) - before).abs() < 1e-12);
    // holes: cut a cap, fill flat then smooth (undo between)
    let m = (*s.doc.mesh(id).unwrap().mesh).clone();
    let keep: Vec<u32> =
        (0..m.triangle_count() as u32).filter(|&t| m.face_centroid(t as usize).unwrap().z < 16.0).collect();
    let cut = s.doc.add_mesh("cut", m.submesh(&keep));
    let _ = cut;
    let flat = s.execute("mesh.fill_holes", json!({"object": "cut", "smooth": false})).unwrap();
    assert_eq!(flat["holes_filled"], 1, "{flat}");
    s.execute("edit.undo", Value::Null).unwrap();
    let small = s.execute("mesh.fill_holes", json!({"object": "cut", "max_edges": 5})).unwrap();
    assert_eq!(small["holes_left"], 1, "{small}");
    let sm = s.execute("mesh.fill_holes", json!({"object": "cut"})).unwrap();
    assert_eq!(sm["holes_filled"], 1, "{sm}");
    assert!(sm["added_triangles"].as_u64().unwrap() > flat["added_triangles"].as_u64().unwrap());
    // bad parameters are refused, not panicked on
    for bad in [
        json!({"object": "ball", "ratio": 0.0}),
        json!({"object": "ball", "ratio": 2.0}),
        json!({"object": "ball", "max_error": -1.0}),
    ] {
        assert!(s.execute("mesh.decimate", bad).is_err());
    }
    assert!(s.execute("mesh.clean", json!({"object": "ball", "min_component_fraction": 3.0})).is_err());
    assert!(s.execute("mesh.remove_outliers", json!({"object": "ball", "ratio": -1.0})).is_err());
}
