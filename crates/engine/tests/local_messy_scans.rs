//! Diagnosis of messy real scans (P9-O9): aggregate facts only, printed per scan NUMBER. The scans
//! stay outside the repository (`SURFACECRAFT_LOCAL_CORPUS`, third-party files); nothing derived from
//! a single file is stored here. `SURFACECRAFT_LOCAL_SCANS` lists the scan numbers (1-based, in file
//! order) to look at, default `13,3,2,9`.
//!
//! Run (release): `SURFACECRAFT_LOCAL_CORPUS=… cargo test --release -p surfacecraft-engine --test local_messy_scans -- --ignored --nocapture --test-threads=1`
use serde_json::json;
use std::sync::Arc;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, nesting, pieces};

fn corpus() -> Vec<std::path::PathBuf> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut files: Vec<_> =
        std::fs::read_dir(dir).map(|rd| rd.filter_map(Result::ok).map(|e| e.path()).collect()).unwrap_or_default();
    files.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    files.sort();
    files
}

fn numbers() -> Vec<usize> {
    let list = std::env::var("SURFACECRAFT_LOCAL_SCANS").unwrap_or_else(|_| "13,3,2,9".into());
    list.split(',').filter_map(|x| x.trim().parse().ok()).collect()
}

fn report(label: &str, mesh: &Mesh) {
    let total = mesh.triangle_count() as f64;
    let ps = pieces::pieces(mesh);
    let area: f64 = ps.iter().map(|p| p.area).sum();
    let size = mesh.bbox().size();
    eprintln!(
        "  [{label}] triangles {} vertices {} extent {:.1} x {:.1} x {:.1} area {:.0}; pieces {}; non-manifold edges {}; boundary edges {} in {} loops",
        mesh.triangle_count(),
        mesh.vertex_count(),
        size.x,
        size.y,
        size.z,
        area,
        ps.len(),
        ps.iter().map(|p| p.non_manifold_edges).sum::<usize>(),
        ps.iter().map(|p| p.boundary_edges).sum::<usize>(),
        ps.iter().map(|p| p.loops.len()).sum::<usize>(),
    );
    for (k, p) in ps.iter().take(8).enumerate() {
        let sizes: Vec<usize> = p.loops.iter().take(5).map(|l| l.edges).collect();
        eprintln!(
            "    piece {k}: {:.1} % of triangles, {:.1} % of area, closed {}, genus {:?} (euler {}), non-manifold {}, loops {} {:?}, extent {:.1}",
            100.0 * p.triangles.len() as f64 / total,
            100.0 * p.area / area.max(1e-12),
            p.closed(),
            p.genus(),
            p.euler,
            p.non_manifold_edges,
            p.loops.len(),
            sizes,
            p.bbox.diagonal(),
        );
    }
    let significant = ps.iter().filter(|p| p.triangles.len() as f64 >= 0.01 * total).count();
    eprintln!("    pieces with at least 1 % of the triangles: {significant}");
    nesting_facts(mesh, &ps);
    let bvh = Bvh::build(mesh);
    let s = nesting::sampled_self_intersections(mesh, &bvh, 3000);
    eprintln!("    self-intersections: {} of {} sampled triangles cross another", s.crossing, s.tested);
    let census = surfacecraft_geom::quality::edge_census(mesh);
    let l = census.lengths;
    eprintln!("    edge length min {:.3} median {:.3} mean {:.3} max {:.2}", l.min, l.median, l.mean, l.max);
    thickness(mesh, &bvh);
}

/// Distance along the vertex normal (either side) to the next surface crossing, over a sample of
/// vertices: a thin double wall or a lattice of tiny handles shows as many short distances.
fn thickness(mesh: &Mesh, bvh: &Bvh) {
    use surfacecraft_geom::Vec3;
    let normals = mesh.vertex_normals();
    let n = mesh.positions.len();
    let samples = 4000.min(n);
    let edges = [0.3, 0.6, 1.0, 2.0, 5.0, 20.0];
    let mut hist = [0usize; 7];
    for k in 0..samples {
        let i = k * n / samples;
        let (p, nv) = (mesh.positions[i], normals[i]);
        if nv.norm() < 0.5 {
            continue;
        }
        let eps = 1e-3;
        let hit = |d: Vec3| bvh.ray_cast(&(p + d * eps), &d).map(|(_, t)| t + eps);
        let best = [hit(nv), hit(-nv)].into_iter().flatten().fold(f64::INFINITY, f64::min);
        let bucket = edges.iter().position(|e| best < *e).unwrap_or(6);
        hist[bucket] += 1;
    }
    eprintln!("    normal-ray distance to the next surface (<0.3, <0.6, <1, <2, <5, <20, farther/none): {hist:?}");
}

fn nesting_facts(mesh: &Mesh, ps: &[pieces::Piece]) {
    let top: Vec<&pieces::Piece> = ps.iter().filter(|p| p.closed()).take(6).collect();
    for (i, outer) in top.iter().enumerate() {
        let bvh = Bvh::build(&mesh.submesh(&outer.triangles));
        for (j, inner) in top.iter().enumerate() {
            if i == j || !nesting::contains(&outer.bbox, &inner.bbox, 0.0) {
                continue;
            }
            let share = nesting::inside_share(&mesh.submesh(&inner.triangles), &bvh, 1e-6 * outer.bbox.diagonal());
            eprintln!(
                "    closed piece {j} lies in the box of closed piece {i}; {:.0} % of its probes are inside it",
                100.0 * share
            );
        }
    }
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn diagnose_messy_scans() {
    let files = corpus();
    assert!(!files.is_empty(), "set SURFACECRAFT_LOCAL_CORPUS");
    for n in numbers() {
        let Some(path) = files.get(n - 1) else { continue };
        let mut s = Session::new();
        let info = s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
        eprintln!("C{n:02}: imported {}", info["triangles"]);
        let before: Arc<Mesh> =
            s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).map(|m| m.mesh.clone()).unwrap();
        report("raw", &before);
        let noise = s.execute("deviation.noise", json!({"object": "scan"})).unwrap();
        eprintln!("    noise: {noise}");
        s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
        let cleaned = s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).map(|m| m.mesh.clone()).unwrap();
        report("cleaned", &cleaned);
    }
}

/// `scan.to_solid` on the listed scans: verdicts (one per part when the scan was split), bodies,
/// patches, share within the tolerance, time. Run one scan per process to read its peak memory
/// (`/usr/bin/time -l`). Numbers only.
#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn measure_to_solid_on_messy_scans() {
    let files = corpus();
    assert!(!files.is_empty(), "set SURFACECRAFT_LOCAL_CORPUS");
    for n in numbers() {
        let Some(path) = files.get(n - 1) else { continue };
        let mut s = Session::new();
        s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
        let started = std::time::Instant::now();
        let out = s.execute("scan.to_solid", json!({"scan": "scan"}));
        let seconds = started.elapsed().as_secs_f64();
        let Ok(out) = out else {
            eprintln!("C{n:02}: ERROR {out:?} in {seconds:.1} s");
            continue;
        };
        let parts: Vec<serde_json::Value> = out["components"].as_array().cloned().unwrap_or_default();
        let verdicts: Vec<&str> = parts.iter().filter_map(|c| c["verdict"].as_str()).collect();
        let bodies = if parts.is_empty() {
            usize::from(!out["body"].is_null())
        } else {
            parts.iter().filter(|c| !c["body"].is_null()).count()
        };
        let patches: Vec<String> = if parts.is_empty() {
            vec![out["patches"].to_string()]
        } else {
            parts.iter().map(|c| c["patches"].to_string()).collect()
        };
        let within: Vec<String> = if parts.is_empty() {
            vec![out["deviation"]["within"].to_string()]
        } else {
            parts.iter().map(|c| format!("{:.3}", c["deviation"]["within"].as_f64().unwrap_or(0.0))).collect()
        };
        let prep: Vec<&str> =
            out["preparation"].as_array().into_iter().flatten().filter_map(|p| p["step"].as_str()).collect();
        let codes: Vec<&str> =
            out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str()).collect();
        eprintln!(
            "C{n:02}: verdict {} parts {} part verdicts {verdicts:?} bodies {bodies} method {} patches {patches:?} within {within:?} time {seconds:.1} s preparation {prep:?} issues {codes:?}",
            out["verdict"],
            parts.len(),
            out["method"]
        );
        if let Some(p) = out["preparation"].as_array().and_then(|a| a.first()) {
            eprintln!("    preparation numbers: {}", p["numbers"]);
        }
    }
}
