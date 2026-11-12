//! `scan.to_solid`: one command from a raw scan to a verified solid (plan
//! `2026-11-11-mcp-agent-modelling`). The reference part is built with the kernel, so its volume and
//! face count are the ground truth; it is tessellated and made to look like a raw scan
//! (`synth::scanify`, sigma = 0.03 mm, holes, spikes, debris). Nothing binary is committed.
use serde_json::{Value, json};
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_geom::{Rng, synth};
use surfacecraft_kernel::feature::{BoolKind, boolean};
use surfacecraft_kernel::{record::brep::Brep, solid};

/// Share of the CAD volume the rebuilt solid may differ by.
const MAX_VOLUME_ERROR: f64 = 0.005;

fn body_brep(s: &Session, name: &str) -> Brep {
    let id = s.doc.find_by_name(name).unwrap().0;
    (*s.doc.body(id).unwrap().brep).clone()
}

/// A pin r = 12, h = 30 with a 3 mm chamfer on the top edge and a through hole r = 4: two planes,
/// a cylinder, a cone and a concave cylinder. Returns the CAD part.
fn chamfered_pin(s: &mut Session) -> Brep {
    s.execute("body.cylinder", json!({"radius": 12.0, "height": 30.0, "name": "pin"})).unwrap();
    let b = body_brep(s, "pin");
    let top = b
        .edges
        .iter()
        .position(|e| e.vertices.iter().all(|&v| (b.vertices[v as usize].point[2] - 30.0).abs() < 1e-9))
        .unwrap();
    s.execute("body.chamfer", json!({"object": "pin", "edges": [top], "size": 3.0})).unwrap();
    let hole = solid::make_cylinder([0.0, 0.0, -1.0], 4.0, 32.0).unwrap();
    boolean(&body_brep(s, "pin"), &hole, BoolKind::Subtract).unwrap()
}

/// A session holding only the raw scan "scan" of `cad`.
fn raw_scan_session(cad: &Brep, seed: u64) -> Session {
    let mut s = Session::new();
    let part = s.doc.add_body("part", cad.clone());
    s.execute("body.to_mesh", json!({"object": part.0, "chord": 0.01, "name": "cad"})).unwrap();
    let id = s.doc.find_by_name("cad").unwrap().0;
    let mut mesh = (*s.doc.mesh(id).unwrap().mesh).clone();
    mesh.weld(1e-6);
    let defects = synth::ScanDefects { spacing: 0.7, noise: 0.03, ..synth::ScanDefects::default() };
    let scan = synth::scanify(&mesh, &defects, &mut Rng::new(seed));
    let mut out = Session::new();
    out.doc.add_mesh("scan", scan);
    out
}

fn codes(list: &Value) -> Vec<String> {
    list.as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap().to_string()).collect()
}

#[test]
fn gate_scan_to_solid_rebuilds_a_chamfered_pin_in_one_call() {
    // Arrange
    let mut scratch = Session::new();
    let cad = chamfered_pin(&mut scratch);
    let cad_volume = solid::volume(&cad).unwrap();
    let mut s = raw_scan_session(&cad, 8);
    let raw_triangles = s.doc.meshes[0].mesh.triangle_count();

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "name": "pin solid"})).unwrap();

    // Assert
    eprintln!("scan.to_solid: {}", serde_json::to_string_pretty(&out).unwrap());
    assert_eq!(out["verdict"], "pass", "{out}");
    assert!(out["issues"].as_array().unwrap().is_empty(), "{out}");
    assert_eq!(out["body"]["faces"], cad.faces.len(), "{out}");
    let volume = out["body"]["volume"].as_f64().unwrap();
    assert!((volume - cad_volume).abs() / cad_volume < MAX_VOLUME_ERROR, "volume {volume} vs CAD {cad_volume}");
    assert!(out["deviation"]["within"].as_f64().unwrap() >= 0.99, "{out}");
    assert!(out["deviation"]["coverage"].as_f64().unwrap() >= 0.9, "{out}");
    assert_eq!(out["modelled"].as_array().unwrap().len(), 5, "{out}");
    assert_eq!(out["next_steps"][0]["command"], "export.step", "{out}");
    // the suggested step runs as it is
    let step = &out["next_steps"][0];
    let exported = s.execute("export.step", step["params"].clone()).unwrap();
    assert!(exported["data"].as_str().is_some_and(|d| !d.is_empty()), "{exported}");
    // one undo step takes the whole run back
    s.execute("edit.undo", Value::Null).unwrap();
    assert!(s.doc.bodies.is_empty() && s.doc.primitives.is_empty() && s.doc.regions.is_empty());
    assert_eq!(s.doc.meshes[0].mesh.triangle_count(), raw_triangles);
}

#[test]
fn a_scan_that_does_not_close_gets_a_verdict_and_next_steps_not_an_error() {
    // Arrange: three separate shells (a box, a boss and a ball), which no stock trimming closes
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "noise": 0.03, "name": "scan"})).unwrap();

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();

    // Assert
    eprintln!("verdict {} issues {}", out["verdict"], out["issues"]);
    assert_ne!(out["verdict"], "pass", "{out}");
    assert!(!codes(&out["issues"]).is_empty(), "{out}");
    // what was made stays for the agent to continue from
    assert!(!s.doc.regions.is_empty() && !s.doc.primitives.is_empty());
    let steps = out["next_steps"].as_array().unwrap();
    assert!(!steps.is_empty(), "{out}");
    assert!(steps.iter().all(|st| st["command"].is_string() && st["why"].is_string()), "{out}");
    assert_eq!(out["verdict"], "fail", "{out}");
    assert!(codes(&out["issues"]).contains(&"no_solid".to_string()), "{out}");
    // every suggested step has parameters its command accepts
    for st in steps {
        let command = st["command"].as_str().unwrap();
        let r = s.execute(command, st["params"].clone());
        assert!(!matches!(r, Err(EngineError::InvalidParams(_) | EngineError::UnknownCommand(_))), "{st}: {r:?}");
    }
}

#[test]
fn reuse_regions_models_the_regions_the_scan_already_has() {
    // Arrange
    let mut scratch = Session::new();
    let cad = chamfered_pin(&mut scratch);
    let mut s = raw_scan_session(&cad, 8);
    s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let regions = s.doc.regions.len();

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "reuse_regions": true})).unwrap();

    // Assert
    assert_eq!(out["verdict"], "pass", "{out}");
    assert_eq!(s.doc.regions.len(), regions, "the regions are kept as they were");
}

#[test]
fn bad_parameters_are_refused_and_change_nothing() {
    // Arrange
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "resolution": 16, "name": "scan"})).unwrap();

    // Act and assert
    for params in [
        json!({"scan": "missing"}),
        json!({"scan": "scan", "tolerance": -1.0}),
        json!({"scan": "scan", "min_within": 1.5}),
        json!({"scan": "scan", "reuse_regions": true}),
        json!({"scan": "scan", "unknown": 1}),
    ] {
        let r = s.execute("scan.to_solid", params.clone());
        assert!(matches!(r, Err(EngineError::InvalidParams(_))), "{params}: {r:?}");
    }
    assert_eq!(s.doc.meshes.len(), 1);
    assert!(s.doc.regions.is_empty() && s.doc.primitives.is_empty() && s.doc.bodies.is_empty());
}

#[test]
fn a_freeform_scan_is_named_as_such_and_pointed_at_the_surface_guide() {
    // Arrange: a ball with a wavy skin; no plane, cylinder, cone, sphere or torus fits it
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "resolution": 48, "name": "scan"})).unwrap();
    let id = s.doc.find_by_name("scan").unwrap().0;
    let mut mesh = (*s.doc.mesh(id).unwrap().mesh).clone();
    for v in &mut mesh.positions {
        let k = 1.0 + 0.15 * (0.9 * v.x).sin() * (1.1 * v.y).cos() + 0.1 * (1.3 * v.z).sin();
        *v *= k;
    }
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();

    // Assert
    assert_eq!(out["verdict"], "fail", "{out}");
    assert!(codes(&out["issues"]).contains(&"mostly_freeform".to_string()), "{out}");
    let steps = out["next_steps"].as_array().unwrap();
    assert!(steps.iter().any(|st| st["params"]["topic"] == "fit-surfaces"), "{out}");
    assert!(steps.iter().all(|st| st["params"]["reuse_regions"] != true), "{out}");
    for st in steps {
        let r = s.execute(st["command"].as_str().unwrap(), st["params"].clone());
        assert!(!matches!(r, Err(EngineError::InvalidParams(_) | EngineError::UnknownCommand(_))), "{st}: {r:?}");
    }
}

#[test]
fn a_scan_with_non_manifold_edges_gets_a_verdict_not_an_error() {
    // Arrange: the pin scan with a fin, a third triangle on an edge of the surface
    let mut scratch = Session::new();
    let cad = chamfered_pin(&mut scratch);
    let mut s = raw_scan_session(&cad, 8);
    s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    let id = s.doc.find_by_name("scan").unwrap().0;
    let mut mesh = (*s.doc.mesh(id).unwrap().mesh).clone();
    let [a, b, c] = mesh.triangles[0];
    let apex = mesh.positions[a as usize] + (mesh.positions[b as usize] - mesh.positions[c as usize]);
    let tip = mesh.positions.len() as u32;
    mesh.positions.push(apex);
    mesh.triangles.push([a, b, tip]);
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    let qa = s.execute("scan.qa", json!({"object": "scan"})).unwrap();
    assert!(qa["non_manifold_edges"].as_u64().unwrap() > 0, "{qa}");

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "clean": false})).unwrap();

    // Assert
    assert!(out["scan_report"]["non_manifold_edges"].as_u64().unwrap() > 0, "{out}");
    if out["verdict"] != "pass" {
        assert!(codes(&out["issues"]).contains(&"non_manifold_edges".to_string()), "{out}");
    }
}
