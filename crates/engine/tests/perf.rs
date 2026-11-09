//! P2: timings at scan scale (1 / 5 / 10 M triangles), design doc §12 target ~10 M triangles.
//!
//! The scans are generated (a filleted block, refined and scanified to the requested size), so the
//! measurement needs no third-party data. Ignored by default (minutes in debug, needs RAM):
//! `cargo test --release -p surfacecraft-engine --test perf -- --ignored --nocapture`
//! `SURFACECRAFT_PERF_SIZES=1,5,10` picks the sizes in millions of triangles.
use serde_json::json;
use std::time::Instant;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Rng, synth};

/// A scanned filleted block of about `millions` × 10⁶ triangles, as STL bytes.
fn scan_bytes(millions: f64) -> (Vec<u8>, usize) {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [120.0, 80.0, 40.0], "name": "block"})).unwrap();
    let b = s.doc.bodies.iter().find(|b| b.name == "block").unwrap();
    let edges: Vec<usize> = b
        .brep
        .edges
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            let (p, q) = (b.brep.vertices[e.vertices[0] as usize].point, b.brep.vertices[e.vertices[1] as usize].point);
            (p[0] - q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9
        })
        .map(|(i, _)| i)
        .collect();
    s.execute("body.fillet", json!({"object": "block", "edges": edges, "size": 10.0})).unwrap();
    s.execute("body.to_mesh", json!({"object": "block", "chord": 0.01, "name": "cad"})).unwrap();
    let id = s.doc.find_by_name("cad").unwrap().0;
    let mut cad = (*s.doc.mesh(id).unwrap().mesh).clone();
    cad.weld(1e-6);
    // ~2 A / h² triangles at spacing h; refinement halves edges, so the factor is measured
    let area = cad.area();
    let h = (2.0 * area / (millions * 1e6)).sqrt() * 1.78;
    let d = synth::ScanDefects { spacing: h, noise: 0.02, holes: 6, hole_radius: 3.0, ..synth::ScanDefects::default() };
    let m = synth::scanify(&cad, &d, &mut Rng::new(1));
    let n = m.triangle_count();
    (surfacecraft_io::write_mesh(&m, surfacecraft_io::MeshFormat::Stl), n)
}

#[test]
#[ignore = "minutes of CPU and several GB of RAM; run explicitly in release"]
fn perf_scan_scale() {
    let sizes: Vec<f64> = std::env::var("SURFACECRAFT_PERF_SIZES")
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| vec![1.0]);
    for m in sizes {
        let (bytes, n) = scan_bytes(m);
        let dir = std::env::temp_dir().join(format!("surfacecraft-perf-{m}.stl"));
        std::fs::write(&dir, &bytes).unwrap();
        drop(bytes);
        let mut s = Session::new();
        let t = Instant::now();
        s.execute("mesh.import", json!({"path": dir.to_string_lossy(), "name": "scan"})).unwrap();
        let import = t.elapsed().as_secs_f64();
        let t = Instant::now();
        s.execute("mesh.clean", json!({"object": "scan"})).unwrap();
        let clean = t.elapsed().as_secs_f64();
        surfacecraft_fit::stats::reset();
        let t = Instant::now();
        let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
        let segment = t.elapsed().as_secs_f64();
        let regions = seg["regions"].as_array().map_or(0, Vec::len);
        let work = surfacecraft_fit::stats::snapshot();
        s.execute("mesh.import", json!({"path": dir.to_string_lossy(), "name": "moved"})).unwrap();
        s.execute("mesh.transform", json!({"object": "moved", "rotate": {"axis": [0.2, 0.3, 1.0], "degrees": 2.0}, "translate": [1.0, 0.5, -0.4]}))
            .unwrap();
        let t = Instant::now();
        let icp = s.execute("align.icp", json!({"object": "moved", "target": "scan"})).unwrap();
        let icp_s = t.elapsed().as_secs_f64();
        let t = Instant::now();
        s.execute("deviation.compute", json!({"object": "moved", "against": "scan"})).unwrap();
        let dev = t.elapsed().as_secs_f64();
        let t = Instant::now();
        let items = s.display_items();
        let disp = t.elapsed().as_secs_f64();
        let drawn: usize = items.iter().map(|i| i.mesh.triangle_count()).sum();
        let _ = std::fs::remove_file(&dir);
        eprintln!(
            "perf {m} M: {n} tris, import {import:.2} s, clean {clean:.2} s, segment {segment:.2} s ({regions} regions), \
             icp {icp_s:.2} s ({} it), deviation {dev:.2} s, display prep {disp:.2} s ({drawn} tris drawn)",
            icp["iterations"]
        );
        eprintln!(
            "perf {m} M work: {} seed fits, {} grow refits, {} final fits, {} LM runs, {} LM iterations",
            work.seed_fits, work.grow_refits, work.final_fits, work.lm_runs, work.lm_iterations
        );
        let phases = surfacecraft_fit::stats::phase_seconds();
        let timed: f64 = phases.iter().map(|(_, s)| s).sum();
        let list: Vec<String> = phases.iter().map(|(n, s)| format!("{n} {s:.1}")).collect();
        eprintln!("perf {m} M segment phases (s): {}, patches+growth {:.1}", list.join(", "), segment - timed);
    }
}
