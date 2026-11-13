//! Shared helpers of the P9-U1 engine tests: temporary directories, meshes written in other units,
//! and the reading of results.
#![allow(dead_code)]
use serde_json::{Value, json};
use std::path::PathBuf;
use surfacecraft_engine::Session;
use surfacecraft_geom::Mesh;

/// Removes its directory when dropped.
pub struct TempDir(PathBuf);

impl TempDir {
    pub fn new(tag: &str) -> Self {
        let nanos = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0, |d| d.as_nanos());
        let dir = std::env::temp_dir().join(format!("surfacecraft-p9-units-{tag}-{}-{nanos}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        Self(dir)
    }

    pub fn file(&self, name: &str) -> String {
        self.0.join(name).to_string_lossy().to_string()
    }
}

impl Drop for TempDir {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

/// The mesh object `name` of `s`, copied out.
pub fn mesh_of(s: &Session, name: &str) -> Mesh {
    let (id, _) = s.doc.find_by_name(name).unwrap();
    s.doc.mesh(id).unwrap().mesh.as_ref().clone()
}

/// Largest distance between corresponding vertices, relative to the diagonal of `reference`.
pub fn worst_gap(a: &Mesh, reference: &Mesh) -> f64 {
    assert_eq!(a.positions.len(), reference.positions.len(), "vertex counts differ");
    let diag = reference.bbox().diagonal();
    a.positions.iter().zip(&reference.positions).map(|(p, q)| (p - q).norm() / diag).fold(0.0, f64::max)
}

/// Largest distance between the corners of corresponding triangles, relative to the largest
/// coordinate of `reference`. STL re-indexes the vertices by first use, so the triangles are
/// compared, not the vertex lists. Single precision is judged against the size of the coordinates.
pub fn worst_corner_gap(a: &Mesh, reference: &Mesh) -> f64 {
    assert_eq!(a.triangles.len(), reference.triangles.len(), "triangle counts differ");
    let magnitude = reference.positions.iter().map(|p| p.coords.amax()).fold(0.0, f64::max);
    let corners = |m: &Mesh, t: &[u32; 3]| t.map(|v| m.positions[v as usize]);
    a.triangles
        .iter()
        .zip(&reference.triangles)
        .flat_map(|(ta, tb)| corners(a, ta).into_iter().zip(corners(reference, tb)))
        .map(|(p, q)| (p - q).norm() / magnitude)
        .fold(0.0, f64::max)
}

/// A bracket as the test reference (millimetres), in a session of its own.
pub fn bracket_session() -> Session {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "name": "part"})).unwrap();
    s
}

/// The bracket written to `dir` with every length multiplied by `factor` (the file in another unit).
pub fn scaled_file(dir: &TempDir, name: &str, factor: f64) -> String {
    let mut s = bracket_session();
    s.execute(
        "mesh.scale_from_length",
        json!({"object": "part", "from": [0, 0, 0], "to": [1, 0, 0], "length": factor, "origin": [0, 0, 0]}),
    )
    .unwrap();
    let path = dir.file(name);
    s.execute("mesh.export", json!({"object": "part", "path": path})).unwrap();
    path
}

/// Coordinates per millimetre of each unit: a file in `unit` holds this many units per mm.
pub fn units_per_mm(unit: &str) -> f64 {
    match unit {
        "m" => 0.001,
        "cm" => 0.1,
        "in" => 1.0 / 25.4,
        "ft" => 1.0 / 304.8,
        "um" => 1000.0,
        _ => 1.0,
    }
}

pub fn import_as(s: &mut Session, path: &str, units: Option<&str>, name: &str) -> Value {
    let mut params = json!({"path": path, "name": name});
    if let Some(u) = units {
        params["units"] = json!(u);
    }
    s.execute("mesh.import", params).unwrap()
}

/// A plain box of eight corners as an OBJ file (no outlier: a few vertices are measured exactly).
pub fn plain_box(dir: &TempDir, name: &str, size: f64) -> String {
    let h = size / 2.0;
    let corners = [[-h, -h, -h], [h, -h, -h], [h, h, -h], [-h, h, -h], [-h, -h, h], [h, -h, h], [h, h, h], [-h, h, h]];
    let mut text: String = corners.iter().map(|c| format!("v {} {} {}\n", c[0], c[1], c[2])).collect();
    text.push_str("f 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n");
    let path = dir.file(name);
    std::fs::write(&path, text).unwrap();
    path
}

pub fn warning_code(v: &Value) -> Option<&str> {
    v["scale_warning"]["code"].as_str()
}

pub fn info_of(s: &mut Session, name: &str) -> Value {
    s.execute("mesh.info", json!({"object": name})).unwrap()
}

/// A flat grid of 12 × 12 vertices (over 100) of `x` × `y` millimetres, with one far outlier.
pub fn plane_with_outlier(dir: &TempDir, name: &str, x: f64, y: f64) -> String {
    let mut text = String::new();
    for i in 0..12 {
        for j in 0..12 {
            text.push_str(&format!("v {} {} 0\n", x * i as f64 / 11.0, y * j as f64 / 11.0));
        }
    }
    text.push_str("v 1000000 0 0\n");
    let path = dir.file(name);
    std::fs::write(&path, text).unwrap();
    path
}

/// The file at `src` with every coordinate multiplied by `factor`, written to `name` in `dir`.
pub fn scaled_file_of(dir: &TempDir, name: &str, src: &str, factor: f64) -> String {
    let mut s = Session::new();
    s.execute("mesh.import", json!({"path": src, "name": "src"})).unwrap();
    s.execute(
        "mesh.scale_from_length",
        json!({"object": "src", "from": [0, 0, 0], "to": [1, 0, 0], "length": factor, "origin": [0, 0, 0]}),
    )
    .unwrap();
    let path = dir.file(name);
    s.execute("mesh.export", json!({"object": "src", "path": path})).unwrap();
    path
}

/// An ASCII STL of a flat grid of `n` × `n` cells over `x` × `y` millimetres (two triangles a cell),
/// plus, when `outlier`, one triangle a million millimetres away.
pub fn grid_stl(dir: &TempDir, name: &str, x: f64, y: f64, n: usize, outlier: bool) -> String {
    let mut text = String::from("solid grid\n");
    let facet = |a: [f64; 3], b: [f64; 3], c: [f64; 3]| {
        format!(
            "facet normal 0 0 1\nouter loop\nvertex {} {} {}\nvertex {} {} {}\nvertex {} {} {}\nendloop\nendfacet\n",
            a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]
        )
    };
    for i in 0..n {
        for j in 0..n {
            let p = |a: usize, b: usize| [x * a as f64 / n as f64, y * b as f64 / n as f64, 0.0];
            text.push_str(&facet(p(i, j), p(i + 1, j), p(i + 1, j + 1)));
            text.push_str(&facet(p(i, j), p(i + 1, j + 1), p(i, j + 1)));
        }
    }
    if outlier {
        text.push_str(&facet([1e6, 0.0, 0.0], [1e6 + 1.0, 0.0, 0.0], [1e6, 1.0, 0.0]));
    }
    text.push_str("endsolid grid\n");
    let path = dir.file(name);
    std::fs::write(&path, text).unwrap();
    path
}
