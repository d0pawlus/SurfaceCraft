//! Gate M6 (web subset, design doc §11): without file access — as in the browser and the
//! sandboxed MCP server — the whole pipeline runs on inline data.
use serde_json::{Value, json};
use surfacecraft_engine::Session;

#[test]
fn gate_m6_web_subset_runs_without_files() {
    let mut s = Session::new();
    s.allow_files = false;
    // a scan arrives as bytes (the browser's file picker), base64 inline
    let mesh = surfacecraft_geom::synth::bracket();
    let data = surfacecraft_doc::base64::encode(&surfacecraft_io::write_mesh(&mesh, surfacecraft_io::MeshFormat::Stl));
    let steps: Vec<(&str, Value)> = vec![
        ("mesh.import", json!({"data": data, "format": "stl", "name": "scan"})),
        ("align.principal", json!({"object": "scan"})),
        ("region.auto_segment", json!({"mesh": "scan"})),
        ("mesh.synth", json!({"shape": "plane", "name": "sheet"})),
        ("mesh.thicken", json!({"object": "sheet", "thickness": 1.0})),
        ("body.box", json!({"size": [10, 10, 10]})),
        ("feature.fillet", json!({"body": "box", "edges": [0], "size": 1.0})),
        ("export.step", json!({"objects": ["box"]})),
        ("export.iges", json!({"objects": ["box"]})),
        ("mesh.export", json!({"object": "scan", "format": "stl"})),
        ("mesh.synth", json!({"shape": "sphere", "name": "ball", "resolution": 40})),
        ("scan.split", json!({"object": "ball"})),
        ("mesh.remesh", json!({"object": "ball", "voxel": 1.0})),
    ];
    for (cmd, p) in steps {
        let r = s.execute(cmd, p);
        assert!(r.is_ok(), "{cmd}: {r:?}");
        if let Ok(v) = r
            && v.get("bytes").is_some()
        {
            assert!(v["data"].as_str().is_some_and(|d| !d.is_empty()), "{cmd} returns its file inline");
        }
    }
    assert!(s.execute("doc.save", json!({"path": "x.scproj"})).is_err());
}
