#![cfg(not(target_arch = "wasm32"))]
//! How much memory importing a large scan from the interface takes: the measurement behind the
//! browser's import limit (`read_limits` in the engine). Off by default; run it once per size, in
//! its own process, under a tool that reports the peak resident memory, e.g. on macOS:
//!
//! ```text
//! SURFACECRAFT_IMPORT_TRIANGLES=10000000 /usr/bin/time -l \
//!     target/ci/deps/import_memory-<hash> --ignored --exact peak_of_a_picked_import
//! ```
//!
//! It walks the browser's path: the picked file's bytes handed to the app, the import run as a
//! background job, then what the viewport builds on the CPU before it uploads the mesh. The peak
//! of a run without `SURFACECRAFT_IMPORT_TRIANGLES` (nothing imported) is the baseline to subtract.
use serde_json::json;
use std::time::{Duration, Instant};
use surfacecraft_ui_egui::SurfaceCraftApp;
use surfacecraft_ui_egui::import_reduce::ImportChoice;

/// Longest the import may take.
const PATIENCE: Duration = Duration::from_secs(1800);
/// Bytes of a binary STL header before its triangle count.
const STL_HEADER: usize = 80;
/// Bytes of one binary STL facet: normal, three corners, attribute.
const STL_FACET: usize = 50;

/// A binary STL of a wavy square grid with at least `triangles` triangles (two per cell, corners
/// shared like a scanner's mesh).
fn grid_stl(triangles: usize) -> Vec<u8> {
    let cells = (triangles as f64 / 2.0).sqrt().ceil() as usize;
    let count = 2 * cells * cells;
    let mut bytes = Vec::with_capacity(STL_HEADER + 4 + count * STL_FACET);
    bytes.resize(STL_HEADER, 0);
    bytes.extend_from_slice(&u32::try_from(count).expect("count fits a u32").to_le_bytes());
    let corner = |i: usize, j: usize| {
        let (x, y) = (i as f32 * 0.1, j as f32 * 0.1);
        [x, y, (x * 0.7).sin() * (y * 0.3).cos()]
    };
    let mut facet = |a: [f32; 3], b: [f32; 3], c: [f32; 3]| {
        bytes.extend_from_slice(&[0u8; 12]);
        for p in [a, b, c] {
            for v in p {
                bytes.extend_from_slice(&v.to_le_bytes());
            }
        }
        bytes.extend_from_slice(&[0u8; 2]);
    };
    for i in 0..cells {
        for j in 0..cells {
            let (p00, p10, p01, p11) = (corner(i, j), corner(i + 1, j), corner(i, j + 1), corner(i + 1, j + 1));
            facet(p00, p10, p11);
            facet(p00, p11, p01);
        }
    }
    bytes
}

/// What the viewport builds on the CPU before it uploads a mesh (`viewport/upload.rs`): vertex
/// normals, positions with normals in `f32`, colours, a flat index list.
fn upload_buffers(mesh: &surfacecraft_geom::Mesh) -> usize {
    let normals = mesh.vertex_normals();
    let verts: Vec<[f32; 6]> = mesh
        .positions
        .iter()
        .zip(&normals)
        .map(|(p, n)| [p.x as f32, p.y as f32, p.z as f32, n.x as f32, n.y as f32, n.z as f32])
        .collect();
    drop(normals);
    let colors = vec![[0.5f32; 4]; mesh.positions.len()];
    let indices: Vec<u32> = mesh.triangles.iter().flatten().copied().collect();
    verts.len() + colors.len() + indices.len()
}

#[test]
#[ignore = "a measurement: run by hand under a peak-memory tool"]
fn peak_of_a_picked_import() {
    let Some(triangles) = std::env::var("SURFACECRAFT_IMPORT_TRIANGLES").ok().and_then(|v| v.parse().ok()) else {
        return;
    };
    let started = Instant::now();
    let bytes = grid_stl(triangles);
    eprintln!("file: {} bytes in {:.1} s", bytes.len(), started.elapsed().as_secs_f64());
    let mut app = SurfaceCraftApp::headless();
    app.background_jobs = true;
    // `SURFACECRAFT_IMPORT_BY_PATH` imports the file from disk instead (the desktop's path)
    if std::env::var_os("SURFACECRAFT_IMPORT_BY_PATH").is_some() {
        let path = std::env::temp_dir().join(format!("surfacecraft-import-memory-{triangles}.stl"));
        std::fs::write(&path, &bytes).expect("the scan is written");
        drop(bytes);
        app.import_picked_path(path.to_str().expect("a UTF-8 temporary path"));
    } else {
        app.import_picked_bytes("scan.stl", bytes);
    }
    if app.import_offer().is_some() {
        app.answer_import_offer(ImportChoice::Full);
    }
    while app.advance_job(started.elapsed().as_secs_f64()) {
        assert!(started.elapsed() < PATIENCE, "the import did not finish");
        std::thread::sleep(Duration::from_millis(5));
    }
    let entry = app.session.log.last().expect("the import ran");
    assert!(entry.ok, "the import failed: {}", entry.outcome);
    let mesh = app.session.doc.meshes.first().expect("a mesh came in").mesh.clone();
    eprintln!("imported: {} triangles in {:.1} s", mesh.triangles.len(), started.elapsed().as_secs_f64());
    let items = upload_buffers(&mesh);
    eprintln!("upload buffers: {items} items; log params: {} bytes", json!(entry.params).to_string().len());
}
