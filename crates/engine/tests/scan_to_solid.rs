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
    // `auto` (the default) keeps a prismatic scan on the primitives method: genuine primitives
    // lost to freeform regions would push the primitive share below half and flip it to the wrap
    assert_eq!(out["method"], "primitives", "{out}");
    assert_eq!(out["attempts"].as_array().unwrap().len(), 1, "a pass with the first method is one attempt: {out}");
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

/// `auto` on the cleaned bracket: the primitives do not close a solid, the freeform wrap does, and
/// `auto` returns it (the twin of the next test, which forces the primitives method to see a verdict
/// that is not a pass). The attempts say what was tried.
#[test]
fn auto_on_the_three_shell_scan_passes_with_the_freeform_wrap() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "noise": 0.03, "name": "scan"})).unwrap();
    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();
    assert_eq!((out["method"].clone(), out["verdict"].clone()), (json!("freeform"), json!("pass")), "{out}");
    let attempts = out["attempts"].as_array().unwrap();
    assert!(attempts.last().is_some_and(|a| a["method"] == "freeform" && a["returned"] == true), "{out}");
    assert!(attempts.iter().filter(|a| a["returned"] == true).count() == 1, "{out}");
}

#[test]
fn a_scan_that_does_not_close_gets_a_verdict_and_next_steps_not_an_error() {
    // Arrange: three separate shells (a box, a boss and a ball), which no stock trimming closes
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "noise": 0.03, "name": "scan"})).unwrap();

    // Act: the primitives method, since `auto` now (P9-O7) takes the cleaned boss, whose filled
    // holes the segmentation consolidates into a freeform area, for a freeform scan and wraps it
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "primitives"})).unwrap();

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
    // no solid, or one that holds a part of the scan (the boss is left out as a freeform area)
    let found = codes(&out["issues"]);
    assert!(found.contains(&"no_solid".to_string()) || found.contains(&"low_coverage".to_string()), "{out}");
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
        json!({"scan": "scan", "method": "magic"}),
    ] {
        let r = s.execute("scan.to_solid", params.clone());
        assert!(matches!(r, Err(EngineError::InvalidParams(_))), "{params}: {r:?}");
    }
    assert_eq!(s.doc.meshes.len(), 1);
    assert!(s.doc.regions.is_empty() && s.doc.primitives.is_empty() && s.doc.bodies.is_empty());
}

/// A ball r = 20 with a wavy skin (bumps of up to 3 mm, about 25 mm apart): no plane, cylinder,
/// cone, sphere or torus fits it. Returned as a raw scan (noise sigma = 0.03 mm).
fn organic_scan_session() -> Session {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "resolution": 64, "name": "scan"})).unwrap();
    let id = s.doc.find_by_name("scan").unwrap().0;
    let mut mesh = (*s.doc.mesh(id).unwrap().mesh).clone();
    for v in &mut mesh.positions {
        let k = 1.0 + 0.1 * (0.25 * v.x).sin() * (0.3 * v.y).cos() + 0.05 * (0.35 * v.z).sin();
        *v *= k;
    }
    synth::add_noise(&mut mesh, 0.03, &mut Rng::new(5));
    let mut out = Session::new();
    out.doc.add_mesh("scan", mesh);
    out
}

#[test]
fn gate_scan_to_solid_wraps_an_organic_scan_in_patches() {
    // Arrange
    let mut s = organic_scan_session();

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();

    // Assert: auto chose the freeform method and the solid passed
    eprintln!("organic: {}", serde_json::to_string(&json!({"verdict": out["verdict"], "method": out["method"], "patches": out["patches"], "body": out["body"], "deviation": out["deviation"], "issues": out["issues"]})).unwrap());
    assert_eq!(out["method"], "freeform", "{out}");
    assert_eq!(out["verdict"], "pass", "{out}");
    assert_eq!(out["attempts"].as_array().unwrap().len(), 1, "an organic scan that passes is one attempt: {out}");
    assert!(out["patches"].as_u64().unwrap() > 0, "{out}");
    assert!(out["body"]["volume"].as_f64().is_some_and(|v| v > 0.0), "a closed solid: {out}");
    assert!(out["deviation"]["within"].as_f64().unwrap() >= 0.99, "{out}");
    assert!(out["deviation"]["coverage"].as_f64().unwrap() >= 0.9, "{out}");
    // the suggested export runs as it is
    let step = &out["next_steps"][0];
    assert_eq!(step["command"], "export.step", "{out}");
    let exported = s.execute("export.step", step["params"].clone()).unwrap();
    assert!(exported["data"].as_str().is_some_and(|d| !d.is_empty()));
    // one undo step takes the whole run back
    s.execute("edit.undo", Value::Null).unwrap();
    assert!(s.doc.bodies.is_empty());
}

#[test]
fn method_primitives_on_an_organic_scan_says_it_is_freeform() {
    // Arrange
    let mut s = organic_scan_session();

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "primitives"})).unwrap();

    // Assert
    assert_eq!(out["method"], "primitives", "{out}");
    assert_eq!(out["verdict"], "fail", "{out}");
    assert!(codes(&out["issues"]).contains(&"mostly_freeform".to_string()), "{out}");
    let steps = out["next_steps"].as_array().unwrap();
    assert!(steps.iter().any(|st| st["command"] == "scan.to_solid" && st["params"]["method"] == "freeform"), "{out}");
    for st in steps {
        let r = s.execute(st["command"].as_str().unwrap(), st["params"].clone());
        assert!(!matches!(r, Err(EngineError::InvalidParams(_) | EngineError::UnknownCommand(_))), "{st}: {r:?}");
    }
}

#[test]
fn an_open_organic_scan_becomes_a_sheet_under_review() {
    // Arrange: the upper part of the organic scan, one large open boundary
    let s = organic_scan_session();
    let id = s.doc.find_by_name("scan").unwrap().0;
    let mut mesh = (*s.doc.mesh(id).unwrap().mesh).clone();
    let keep: Vec<[u32; 3]> =
        mesh.triangles.iter().copied().filter(|t| t.iter().all(|&v| mesh.positions[v as usize].z > -5.0)).collect();
    mesh.triangles = keep;
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);

    // Act
    let out =
        s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "fill_holes_max_edges": 10})).unwrap();

    // Assert
    assert_eq!(out["verdict"], "review", "{out}");
    assert!(codes(&out["issues"]).contains(&"open_surface".to_string()), "{out}");
    assert!(out["body"]["volume"].as_f64().is_none_or(|v| v == 0.0), "a sheet has no volume: {out}");
    assert!(out["deviation"]["within"].as_f64().unwrap() >= 0.99, "{out}");
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
