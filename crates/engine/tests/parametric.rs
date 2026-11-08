//! M5 through the command layer: sketch + solver, features, history replay/undo, STEP/IGES.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_engine::Session;

fn vol(v: &Value) -> f64 {
    v["volume"].as_f64().unwrap()
}

/// A 40 × 20 rectangle sketch, fully constrained.
fn plate_sketch(s: &mut Session) -> Value {
    s.execute("sketch.create", json!({"name": "base"})).unwrap();
    s.execute("sketch.add", json!({"sketch": "base", "entities": [{"polygon": {"points": [[0.2, -0.1], [39.0, 0.4], [40.5, 20.2], [0.1, 19.0]]}}]}))
        .unwrap();
    s.execute(
        "sketch.constrain",
        json!({"sketch": "base", "constraints": [
            {"type": "fix", "point": 0, "at": [0.0, 0.0]},
            {"type": "horizontal", "line": 0}, {"type": "vertical", "line": 1},
            {"type": "horizontal", "line": 2}, {"type": "vertical", "line": 3},
            {"type": "length", "line": 0, "value": 40.0}, {"type": "length", "line": 1, "value": 20.0}
        ]}),
    )
    .unwrap()
}

/// Gate M5: the solver reports the sketch fully defined, then detects over-definition.
#[test]
fn gate_m5_sketch_solver_through_commands() {
    let mut s = Session::new();
    let r = plate_sketch(&mut s);
    assert_eq!(r["report"]["status"], "fully_defined", "{r}");
    assert_eq!(r["report"]["dof"], 0);
    assert_eq!(r["profiles"], 1);
    let over = s
        .execute(
            "sketch.constrain",
            json!({"sketch": "base", "constraints": [{"type": "length", "line": 2, "value": 30.0}]}),
        )
        .unwrap();
    assert_eq!(over["report"]["status"], "over_defined", "{over}");
    assert!(over["report"]["message"].as_str().unwrap().contains("overdefined"));
    assert_eq!(over["report"]["conflicting"][0], 7);
}

/// Gate M5: features in a history; editing a sketch dimension or a feature parameter replays
/// everything after it, and undo/redo restore each state.
#[test]
fn gate_m5_features_dag_replay_and_undo() {
    let mut s = Session::new();
    plate_sketch(&mut s);
    let ex = s.execute("feature.extrude", json!({"sketch": "base", "distance": 10.0, "name": "plate"})).unwrap();
    assert!((vol(&ex) - 8000.0).abs() < 1e-3, "{ex}");
    // a hole: circle sketch on the top face, cut through
    s.execute(
        "sketch.create",
        json!({"name": "hole", "plane": {"origin": [0.0, 0.0, 10.0], "normal": [0.0, 0.0, 1.0]}}),
    )
    .unwrap();
    let hole_sk = s.doc.sketches.iter().find(|k| k.name == "hole").unwrap().sketch.plane.clone();
    let c = hole_sk.to_2d(&surfacecraft_geom::Point3::new(20.0, 10.0, 10.0));
    s.execute("sketch.add", json!({"sketch": "hole", "entities": [{"circle": {"center": c, "radius": 4.0}}]})).unwrap();
    let cut = s
        .execute("feature.extrude", json!({"sketch": "hole", "distance": -10.0, "mode": "cut", "target": "plate"}))
        .unwrap();
    let with_hole = 8000.0 - PI * 16.0 * 10.0;
    assert!((vol(&cut) - with_hole).abs() < 1e-2, "{cut}");
    // fillet a vertical outer edge (pick the edge whose ends differ only in z)
    let body = s.doc.bodies.iter().find(|b| b.name == "plate").unwrap();
    let edge = body
        .brep
        .edges
        .iter()
        .position(|e| {
            let (a, b) =
                (body.brep.vertices[e.vertices[0] as usize].point, body.brep.vertices[e.vertices[1] as usize].point);
            (a[0] - b[0]).abs() < 1e-9
                && (a[1] - b[1]).abs() < 1e-9
                && (a[2] - b[2]).abs() > 9.0
                && a[0].abs() < 1e-9
                && a[1].abs() < 1e-9
        })
        .unwrap();
    let fil = s.execute("feature.fillet", json!({"body": "plate", "edges": [edge], "size": 2.0})).unwrap();
    let filleted = with_hole - (1.0 - PI / 4.0) * 4.0 * 10.0;
    assert!((vol(&fil) - filleted).abs() < 1e-2, "{fil}");
    // the history: 4 features (extrude, cut, fillet + no base), dependencies resolved
    let tree = s.execute("feature.tree", Value::Null).unwrap();
    let nodes = tree["features"].as_array().unwrap();
    assert_eq!(nodes.len(), 3);
    assert_eq!(nodes[2]["depends_on"][0], nodes[1]["id"]);
    // edit the sketch: 40 → 50 long; the extrude, cut and fillet replay
    let e = s
        .execute(
            "sketch.edit_constraint",
            json!({"sketch": "base", "index": 5, "constraint": {"type": "length", "line": 0, "value": 50.0}}),
        )
        .unwrap();
    assert_eq!(e["failed_features"], 0, "{e}");
    let v50 =
        surfacecraft_kernel::solid::volume(&s.doc.bodies.iter().find(|b| b.name == "plate").unwrap().brep).unwrap();
    assert!((v50 - (filleted + 10.0 * 20.0 * 10.0)).abs() < 1e-2, "volume after edit {v50}");
    // edit a feature parameter: extrude depth 10 → 15 (the hole cut of 10 then leaves a blind hole)
    let fe = s.execute("feature.edit", json!({"feature": "extrude", "set": {"distance": 15.0}})).unwrap();
    assert_eq!(fe["failed_features"], 0, "{fe}");
    // undo twice returns to the 40-long, 10-deep part
    s.execute("edit.undo", Value::Null).unwrap();
    s.execute("edit.undo", Value::Null).unwrap();
    let back =
        surfacecraft_kernel::solid::volume(&s.doc.bodies.iter().find(|b| b.name == "plate").unwrap().brep).unwrap();
    assert!((back - filleted).abs() < 1e-2, "after undo {back}");
    s.execute("edit.redo", Value::Null).unwrap();
    let again =
        surfacecraft_kernel::solid::volume(&s.doc.bodies.iter().find(|b| b.name == "plate").unwrap().brep).unwrap();
    assert!((again - v50).abs() < 1e-6);
    // suppress the fillet: the corner is sharp again
    s.execute("feature.suppress", json!({"feature": "fillet", "suppressed": true})).unwrap();
    let sharp =
        surfacecraft_kernel::solid::volume(&s.doc.bodies.iter().find(|b| b.name == "plate").unwrap().brep).unwrap();
    assert!((sharp - (with_hole + 2000.0)).abs() < 1e-2, "{sharp}");
}

/// Gate M5: STEP of a parametric solid round-trips (solidcore writer and reader) with the same
/// volume and topology; IGES is written for the same solid.
#[test]
fn gate_m5_step_roundtrip_and_iges() {
    let mut s = Session::new();
    plate_sketch(&mut s);
    s.execute("feature.extrude", json!({"sketch": "base", "distance": 10.0, "name": "plate"})).unwrap();
    s.execute("feature.fillet", json!({"body": "plate", "edges": [0], "size": 2.0})).unwrap();
    let body = s.doc.bodies.iter().find(|b| b.name == "plate").unwrap().clone();
    let v0 = surfacecraft_kernel::solid::volume(&body.brep).unwrap();
    let e = s.execute("export.step", json!({"objects": ["plate"]})).unwrap();
    assert_eq!(e["content"], "solids");
    let back = s.execute("import.step", json!({"data": e["data"], "name": "reread"})).unwrap();
    let rb = s.doc.bodies.iter().find(|b| b.name == "reread").unwrap();
    let v1 = surfacecraft_kernel::solid::volume(&rb.brep).unwrap();
    eprintln!(
        "gate_m5 STEP round-trip: volume {v0:.6} → {v1:.6}, faces {} → {}",
        body.brep.faces.len(),
        back["faces"][0]
    );
    assert!((v0 - v1).abs() < 1e-6 * v0.max(1.0), "{v0} vs {v1}");
    assert_eq!(back["faces"][0].as_u64().unwrap() as usize, body.brep.faces.len());
    let ig = s.execute("export.iges", json!({"objects": ["plate"]})).unwrap();
    assert!(ig["bytes"].as_u64().unwrap() > 1000, "{ig}");
}

#[test]
fn section_sketch_pattern_shell_boolean_and_suggestion() {
    let mut s = Session::new();
    // a scan of a 30 × 20 × 10 box: section at mid height → rectangle → extrude back
    s.execute("mesh.synth", json!({"shape": "box", "size": [30, 20, 10], "resolution": 40, "name": "scan"})).unwrap();
    let sec = s
        .execute(
            "sketch.from_section",
            json!({"mesh": "scan", "origin": [0, 0, 5], "normal": [0, 0, 1], "name": "sec"}),
        )
        .unwrap();
    assert_eq!(sec["entities"], 4, "{sec}");
    assert_eq!(sec["profiles"], 1);
    let ex = s.execute("feature.extrude", json!({"sketch": "sec", "distance": 5.0, "name": "slab"})).unwrap();
    assert!((vol(&ex) - 30.0 * 20.0 * 5.0).abs() < 1e-3, "{ex}");
    // pattern of a pin, shell of a box, boolean
    s.execute("body.cylinder", json!({"radius": 2.0, "height": 4.0, "name": "pin"})).unwrap();
    let p = s.execute("feature.pattern", json!({"body": "pin", "count": 3, "translate": [10.0, 0.0, 0.0]})).unwrap();
    assert!((vol(&p) - 3.0 * PI * 16.0).abs() < 1e-3, "{p}");
    s.execute("body.box", json!({"size": [20.0, 10.0, 5.0], "name": "cup"})).unwrap();
    let sh = s.execute("feature.shell", json!({"body": "cup", "faces": [0], "thickness": 1.0})).unwrap();
    assert!((vol(&sh) - 694.0).abs() < 1e-3, "{sh}");
    s.execute("body.box", json!({"origin": [0.0, 0.0, 20.0], "size": [10.0, 10.0, 10.0], "name": "a"})).unwrap();
    s.execute("body.box", json!({"origin": [5.0, 0.0, 20.0], "size": [10.0, 10.0, 10.0], "name": "b"})).unwrap();
    let bo = s.execute("feature.boolean", json!({"body": "a", "tool": "b", "op": "union"})).unwrap();
    assert!((vol(&bo) - 1500.0).abs() < 1e-3, "{bo}");
    // redesign assistant: scan of a box with a 3 mm round on one edge suggests r ≈ 3
    s.execute("body.box", json!({"origin": [0.0, 50.0, 0.0], "size": [20.0, 10.0, 5.0], "name": "ref"})).unwrap();
    s.execute("body.box", json!({"origin": [0.0, 50.0, 0.0], "size": [20.0, 10.0, 5.0], "name": "rounded"})).unwrap();
    s.execute("body.fillet", json!({"object": "rounded", "edges": [0], "size": 3.0})).unwrap();
    s.execute("body.to_mesh", json!({"object": "rounded", "chord": 0.01, "name": "rscan"})).unwrap();
    let sug =
        s.execute("feature.suggest_fillet", json!({"body": "ref", "edge": 0, "mesh": "rscan", "search": 6.0})).unwrap();
    assert!((sug["radius"].as_f64().unwrap() - 3.0).abs() < 0.05, "{sug}");
}
