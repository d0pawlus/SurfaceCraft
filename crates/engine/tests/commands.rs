//! Engine behaviour every front end relies on.
use serde_json::{Value, json};
use surfacecraft_engine::{EngineError, Session, registry};

#[test]
fn script_runs_and_undo_redo_restore() {
    let mut s = Session::new();
    let report = s
        .run_script(&json!({"commands": [
            {"command": "mesh.synth", "params": {"shape": "sphere", "size": [10.0], "resolution": 16}},
            {"command": "body.box", "params": {"size": [20.0, 10.0, 5.0]}},
            {"command": "body.info", "params": {"object": "box"}},
        ]}))
        .unwrap();
    assert!(report.ok, "{report:?}");
    let vol = report.steps[2].result.as_ref().unwrap()["volume"].as_f64().unwrap();
    assert!((vol - 1000.0).abs() < 1e-6);
    assert_eq!(s.doc.summaries().len(), 2);
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(s.doc.summaries().len(), 1);
    s.execute("edit.redo", Value::Null).unwrap();
    assert_eq!(s.doc.summaries().len(), 2);
    assert!(matches!(s.execute("edit.redo", Value::Null), Err(EngineError::Unavailable(_))));
    // a failing step stops the script and leaves the document as it was
    let report = s
        .run_script(&json!({"commands": [
            {"command": "body.fillet", "params": {"object": "box", "edges": [999], "size": 1.0}},
            {"command": "doc.new"},
        ]}))
        .unwrap();
    assert!(!report.ok);
    assert_eq!(report.steps.len(), 1);
    assert_eq!(s.doc.summaries().len(), 2);
}

#[test]
fn kernel_boundary_through_commands() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [20.0, 10.0, 5.0]})).unwrap();
    let out = s.execute("body.fillet", json!({"object": "box", "edges": [0], "size": 1.0})).unwrap();
    assert_eq!(out["faces"], 7);
    let items = s.display_items();
    assert_eq!(items.len(), 1);
    assert!(items[0].mesh.triangle_count() > 12);
}

#[test]
fn mesh_import_export_inline_roundtrip() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [4.0, 5.0, 6.0], "resolution": 8})).unwrap();
    for fmt in ["stl", "obj", "ply"] {
        let out = s.execute("mesh.export", json!({"object": "box", "format": fmt})).unwrap();
        let data = out["data"].as_str().unwrap().to_string();
        let info = s.execute("mesh.import", json!({"data": data, "format": fmt, "name": fmt})).unwrap();
        assert_eq!(info["triangles"], s.execute("mesh.info", json!({"object": "box"})).unwrap()["triangles"]);
    }
    let t = s
        .execute(
            "mesh.transform",
            json!({"object": "box", "rotate": {"axis": [0, 0, 1], "degrees": 90}, "translate": [1, 0, 0]}),
        )
        .unwrap();
    let m = t["matrix"].as_array().unwrap();
    assert!((m[0][3].as_f64().unwrap() - 1.0).abs() < 1e-12);
}

#[test]
fn project_save_open() {
    let dir = std::env::temp_dir().join(format!("sc-test-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("p.scproj").to_string_lossy().to_string();
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "torus", "resolution": 16})).unwrap();
    s.execute("body.sphere", json!({"radius": 3.0})).unwrap();
    s.execute("doc.save", json!({"path": path})).unwrap();
    let mut t = Session::new();
    let out = t.execute("doc.open", json!({"path": path})).unwrap();
    assert_eq!(out["objects"].as_array().unwrap().len(), 2);
    let _ = std::fs::remove_dir_all(dir);
}

fn hostile_values() -> Vec<Value> {
    vec![
        Value::Null,
        json!({}),
        json!([]),
        json!(1),
        json!("x"),
        json!({"object": -1}),
        json!({"object": 1e300}),
        json!({"object": "nope"}),
        json!({"size": [f64::MAX, -1.0, 0.0]}),
        json!({"size": [1e308, 1e308, 1e308]}),
        json!({"edges": [usize::MAX], "object": 1, "size": 1.0}),
        json!({"data": "!!!!", "format": "stl"}),
        json!({"data": "AAAA", "format": "ply"}),
        json!({"path": "/definitely/not/here.stl"}),
        json!({"shape": "sphere", "size": [-1.0]}),
        json!({"shape": "box", "resolution": 100000000, "size": [1.0, 1.0, 1.0]}),
        json!({"matrix": [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], "object": 1}),
        json!({"rotate": {"axis": [0, 0, 0], "degrees": 1}, "object": 1}),
        json!({"id": "unknown.command"}),
        json!({"filter": 3}),
        json!({"object": 1, "thickness": 1e308, "symmetric": true}),
        json!({"object": 1, "thickness": -1e-300}),
        json!({"scan": 1, "thickness": 1e308, "symmetric": true, "keep_thickened": true}),
        json!({"scan": 1, "thickness": 0.0}),
    ]
}

/// Every command, given hostile parameters, returns an error or a value but never panics
/// (spike §3.2 item 7, `hostile_params_never_panic`).
#[test]
fn hostile_params_never_panic() {
    for spec in registry() {
        for v in hostile_values() {
            let mut s = Session::new();
            s.allow_files = false;
            s.execute("mesh.synth", json!({"shape": "box", "resolution": 8})).unwrap();
            s.execute("body.box", json!({"size": [1.0, 1.0, 1.0]})).unwrap();
            let r = s.execute(spec.id, v.clone());
            assert!(!matches!(r, Err(EngineError::Internal(_))), "{} panicked on {v}", spec.id);
        }
    }
}

#[test]
fn unknown_command_is_an_error() {
    let mut s = Session::new();
    assert!(matches!(s.execute("no.such", Value::Null), Err(EngineError::UnknownCommand(_))));
    assert!(s.run_script(&json!({"steps": []})).is_err());
}
