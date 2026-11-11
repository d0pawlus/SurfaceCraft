//! P8 (G1): `mesh.import_stream` and `mesh.store_info`. A scan is streamed from a file into a chunk
//! store on disk; the commands report what the store holds, refuse paths where files are not
//! allowed, and turn a damaged store into an error.
use serde_json::{Value, json};
use std::path::{Path, PathBuf};
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_geom::{Point3, Vec3, synth};
use surfacecraft_io::{MeshFormat, write_mesh};

struct TempDir(PathBuf);

impl TempDir {
    fn new(name: &str) -> Self {
        let p = std::env::temp_dir().join(format!("sc-engine-stream-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&p);
        std::fs::create_dir_all(&p).unwrap();
        TempDir(p)
    }
    fn path(&self) -> &Path {
        &self.0
    }
    fn s(&self, name: &str) -> String {
        self.0.join(name).to_string_lossy().into_owned()
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

fn torus_file(dir: &TempDir, format: MeshFormat, name: &str) -> (String, usize) {
    let mut m = synth::torus(Point3::origin(), Vec3::z(), 30.0, 7.0, 120, 60);
    m.weld(1e-9);
    let n = m.triangle_count();
    std::fs::write(dir.path().join(name), write_mesh(&m, format)).unwrap();
    (dir.s(name), n)
}

#[test]
fn import_stream_builds_a_store_and_store_info_reads_it_back() {
    let dir = TempDir::new("ok");
    let (path, n) = torus_file(&dir, MeshFormat::Stl, "t.stl");
    let mut s = Session::new();
    let store = dir.s("store");
    let r = s
        .execute("mesh.import_stream", json!({"path": path, "store_dir": store, "chunk_triangles": 1024, "levels": 3}))
        .unwrap();
    assert_eq!(r["source_triangles"].as_u64().unwrap(), n as u64, "{r}");
    assert_eq!(r["triangles"].as_u64().unwrap(), n as u64);
    assert!(r["chunks"].as_u64().unwrap() >= (n / 1024) as u64);
    let levels = r["levels"].as_array().unwrap();
    assert_eq!(levels.len(), 4);
    for l in &levels[1..] {
        assert!(l["max_error"].as_f64().unwrap() <= l["error_bound"].as_f64().unwrap() + 1e-9, "{l}");
    }
    assert!(r["build"]["peak_buffered_triangles"].as_u64().unwrap() <= n as u64);
    assert!(r["build"]["max_chunk_triangles"].as_u64().unwrap() <= 1024);
    assert!(r["build"]["memory_bound_mib"].as_f64().unwrap() > 0.0);
    // the document is untouched: a store is not a mesh object
    assert!(s.doc.find_by_name("t").is_none());
    // the size in x of a torus with R = 30, r = 7 is 74 mm
    let (lo, hi) = (r["bbox_min"][0].as_f64().unwrap(), r["bbox_max"][0].as_f64().unwrap());
    assert!((hi - lo - 74.0).abs() < 0.5, "{lo} {hi}");
    // store_info agrees, with and without verification
    let info = s.execute("mesh.store_info", json!({"store_dir": store})).unwrap();
    assert_eq!(info["triangles"], r["triangles"]);
    assert_eq!(info["verified"], false);
    let verified = s.execute("mesh.store_info", json!({"store_dir": store, "verify": true})).unwrap();
    assert_eq!(verified["verified"], true);
    // PLY works too, an existing store needs `overwrite`
    let (ply, _) = torus_file(&dir, MeshFormat::Ply, "t.ply");
    assert!(s.execute("mesh.import_stream", json!({"path": ply, "store_dir": store})).is_err());
    let again = s
        .execute("mesh.import_stream", json!({"path": ply, "store_dir": store, "overwrite": true, "scale": 2.0}))
        .unwrap();
    assert!((again["bbox_max"][0].as_f64().unwrap() - again["bbox_min"][0].as_f64().unwrap() - 148.0).abs() < 1.0);
}

#[test]
fn a_damaged_store_is_an_error_and_files_are_refused_where_not_allowed() {
    let dir = TempDir::new("bad");
    let (path, _) = torus_file(&dir, MeshFormat::Stl, "t.stl");
    let store = dir.s("store");
    let mut s = Session::new();
    s.execute("mesh.import_stream", json!({"path": path, "store_dir": store, "chunk_triangles": 512, "levels": 1}))
        .unwrap();
    let f = dir.path().join("store").join("mesh-0.dat");
    let mut bytes = std::fs::read(&f).unwrap();
    let at = bytes.len() / 3;
    bytes[at] ^= 0x40;
    std::fs::write(&f, bytes).unwrap();
    // the index alone does not see a flipped bit in a data file, verification does
    assert!(s.execute("mesh.store_info", json!({"store_dir": store})).is_ok());
    let err = s.execute("mesh.store_info", json!({"store_dir": store, "verify": true})).unwrap_err();
    assert!(err.to_string().contains("damaged"), "{err}");
    std::fs::write(dir.path().join("store").join("index.scx"), b"nonsense").unwrap();
    assert!(s.execute("mesh.store_info", json!({"store_dir": store})).is_err());
    // no file access (web, MCP sandbox)
    let mut web = Session::new();
    web.allow_files = false;
    for (id, p) in [
        ("mesh.import_stream", json!({"path": path, "store_dir": dir.s("other")})),
        ("mesh.store_info", json!({"store_dir": store})),
    ] {
        assert!(matches!(web.execute(id, p), Err(EngineError::Unavailable(_))), "{id}");
    }
    assert!(!dir.path().join("other").exists());
}

#[test]
fn import_stream_rejects_bad_parameters_without_touching_the_disk() {
    let dir = TempDir::new("params");
    let (path, _) = torus_file(&dir, MeshFormat::Stl, "t.stl");
    let mut s = Session::new();
    for p in [
        json!({"path": path, "store_dir": dir.s("a"), "chunk_triangles": 3}),
        json!({"path": path, "store_dir": dir.s("a"), "chunk_triangles": usize::MAX}),
        json!({"path": path, "store_dir": dir.s("a"), "levels": 99}),
        json!({"path": path, "store_dir": dir.s("a"), "scale": -2.0}),
        json!({"path": path, "store_dir": dir.s("a"), "scale": 0.0}),
        json!({"path": dir.s("missing.stl"), "store_dir": dir.s("a")}),
        json!({"path": path}),
        json!({"store_dir": dir.s("a")}),
        json!({"path": path, "store_dir": dir.s("a"), "bogus": 1}),
        Value::Null,
    ] {
        assert!(s.execute("mesh.import_stream", p.clone()).is_err(), "{p}");
    }
    assert!(!dir.path().join("a").exists());
}

#[test]
fn the_commands_have_schemas_and_are_listed() {
    for id in ["mesh.import_stream", "mesh.store_info"] {
        let c = surfacecraft_engine::cmd::find(id).unwrap();
        assert!((c.input)()["properties"].is_object(), "{id}");
        assert!((c.output)().is_object());
        assert!(!c.undoable);
        assert!(surfacecraft_engine::cmd::is_heavy(id));
    }
}
