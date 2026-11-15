//! The pieces of every corpus scan after the cleaning `scan.to_solid` does, with the facts that tell a
//! small real part from scan debris (P9-O9, scan with many parts). Numbers only, per scan NUMBER; the
//! scans stay outside the repository (`SURFACECRAFT_LOCAL_CORPUS`).
//!
//! Run: `SURFACECRAFT_LOCAL_CORPUS=… cargo test --profile ci -p surfacecraft-engine --test local_pieces -- --ignored --nocapture`
//! (`SURFACECRAFT_LOCAL_CORPUS_NUMBERS=13,2` limits the run).
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, pieces};

fn corpus() -> Vec<(usize, std::path::PathBuf)> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut files: Vec<_> =
        std::fs::read_dir(dir).map(|rd| rd.filter_map(Result::ok).map(|e| e.path()).collect()).unwrap_or_default();
    files.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    files.sort();
    let mut numbered: Vec<(usize, std::path::PathBuf)> =
        files.into_iter().enumerate().map(|(k, p)| (k + 1, p)).collect();
    if let Ok(list) = std::env::var("SURFACECRAFT_LOCAL_CORPUS_NUMBERS") {
        let wanted: Vec<usize> = list.split(',').filter_map(|x| x.trim().parse().ok()).collect();
        numbered.retain(|(n, _)| wanted.contains(n));
    }
    numbered
}

fn signed_volume(m: &Mesh) -> f64 {
    (0..m.triangle_count())
        .filter_map(|i| m.triangle(i))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum()
}

fn median_edge(m: &Mesh) -> f64 {
    surfacecraft_geom::quality::edge_census(m).lengths.median
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn list_pieces_of_every_scan() {
    let files = corpus();
    assert!(!files.is_empty(), "set SURFACECRAFT_LOCAL_CORPUS");
    for (n, path) in files {
        let mut s = Session::new();
        let mut import = json!({"path": path.to_string_lossy(), "name": "scan"});
        if n == 8 {
            import["units"] = json!("m");
        }
        if s.execute("mesh.import", import).is_err() {
            eprintln!("C{n:02}: import failed");
            continue;
        }
        s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
        let mesh = s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).map(|m| m.mesh.clone()).unwrap();
        let ps = pieces::pieces(&mesh);
        let total_area: f64 = ps.iter().map(|p| p.area).sum::<f64>().max(1e-12);
        let largest = ps.iter().map(|p| p.area).fold(0.0, f64::max);
        let whole_edge = median_edge(&mesh);
        eprintln!(
            "C{n:02}: triangles {} pieces {} median edge {whole_edge:.3} largest area share {:.4}",
            mesh.triangle_count(),
            ps.len(),
            largest / total_area
        );
        // histogram of every piece by triangle count and closedness
        let buckets = [10usize, 50, 100, 200, 500, 1000, 5000, usize::MAX];
        let mut closed_hist = [0usize; 8];
        let mut open_hist = [0usize; 8];
        for p in &ps {
            let b = buckets.iter().position(|x| p.triangles.len() < *x).unwrap_or(7);
            if p.closed() { closed_hist[b] += 1 } else { open_hist[b] += 1 }
        }
        eprintln!(
            "    pieces by triangles (<10,<50,<100,<200,<500,<1000,<5000,more): closed {closed_hist:?} open {open_hist:?}"
        );
        for (k, p) in ps.iter().enumerate().filter(|(_, p)| p.triangles.len() >= 20).take(120) {
            let sub = mesh.submesh(&p.triangles);
            let vol = signed_volume(&sub);
            let edge = median_edge(&sub);
            eprintln!(
                "    piece {k:3}: tris {:6} area% {:7.3} of-largest% {:7.2} closed {} nm {} unor {} pinch {} loops {} genus {:?} diag {:7.2} diag/edge {:6.1} vol {:+.1} edge {:.3}",
                p.triangles.len(),
                100.0 * p.area / total_area,
                100.0 * p.area / largest.max(1e-12),
                p.closed(),
                p.non_manifold_edges,
                p.unoriented_edges,
                p.pinched_vertices,
                p.loops.len(),
                p.genus(),
                p.bbox.diagonal(),
                p.bbox.diagonal() / edge.max(1e-12),
                vol,
                edge,
            );
        }
        // the small closed pieces (below 20 triangles) in aggregate
        let tiny: Vec<&pieces::Piece> = ps.iter().filter(|p| p.triangles.len() < 20).collect();
        let tiny_closed = tiny.iter().filter(|p| p.closed()).count();
        let tiny_area: f64 = tiny.iter().map(|p| p.area).sum();
        let tiny_diag = tiny.iter().map(|p| p.bbox.diagonal()).fold(0.0, f64::max);
        eprintln!(
            "    below 20 triangles: {} pieces ({tiny_closed} closed), {:.4} % of the area, largest diagonal {tiny_diag:.3}",
            tiny.len(),
            100.0 * tiny_area / total_area
        );
    }
}

/// The raw pieces (before cleaning) beyond the largest that a closed-solid rule would take, so that
/// debris the cleaning removes can be told from parts: closed, oriented, positive volume.
#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn raw_closed_pieces_of_every_scan() {
    for (n, path) in corpus() {
        let mut s = Session::new();
        let mut import = json!({"path": path.to_string_lossy(), "name": "scan"});
        if n == 8 {
            import["units"] = json!("m");
        }
        if s.execute("mesh.import", import).is_err() {
            continue;
        }
        let mesh = s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).map(|m| m.mesh.clone()).unwrap();
        let ps = pieces::pieces(&mesh);
        let largest = ps.iter().map(|p| p.triangles.len()).max().unwrap_or(0);
        let solid: Vec<&pieces::Piece> = ps
            .iter()
            .skip(1)
            .filter(|p| p.closed() && p.genus().is_some() && signed_volume(&mesh.submesh(&p.triangles)) > 0.0)
            .collect();
        let tris: Vec<usize> = solid.iter().map(|p| p.triangles.len()).collect();
        let diag: Vec<String> = solid.iter().map(|p| format!("{:.2}", p.bbox.diagonal())).collect();
        eprintln!(
            "C{n:02}: raw pieces {} (largest {largest} triangles; below 20 triangles {}), closed oriented outward beyond the largest {}: triangles {:?} diagonals {:?}",
            ps.len(),
            ps.iter().filter(|p| p.triangles.len() < 20).count(),
            solid.len(),
            &tris[..tris.len().min(20)],
            &diag[..diag.len().min(20)],
        );
        let small: Vec<String> = ps
            .iter()
            .filter(|p| p.triangles.len() * 100 < largest)
            .map(|p| {
                let v = signed_volume(&mesh.submesh(&p.triangles));
                format!("{}t/{:.2}mm/{}/{:+.0}", p.triangles.len(), p.bbox.diagonal(), p.closed(), v)
            })
            .collect();
        eprintln!("    removed by cleaning (below 1 % of the largest): {}: {small:?}", small.len());
    }
}

/// Every piece of the listed scans modelled on its own (as a split run does: cleaned once, the part
/// not cleaned again), with time, verdict, method, patches and the extra attempts it used.
#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn model_every_piece_alone() {
    for (n, path) in corpus() {
        let mut s = Session::new();
        if s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).is_err() {
            continue;
        }
        s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
        let split = s.execute("scan.split", json!({"object": "scan", "min_share": 1e-6})).unwrap();
        let only: Vec<usize> = std::env::var("SURFACECRAFT_PIECES")
            .map(|l| l.split(',').filter_map(|x| x.trim().parse().ok()).collect())
            .unwrap_or_default();
        let extra: serde_json::Value = std::env::var("SURFACECRAFT_PIECE_PARAMS")
            .ok()
            .and_then(|x| serde_json::from_str(&x).ok())
            .unwrap_or_else(|| json!({}));
        for (k, p) in split["parts"].as_array().unwrap().iter().enumerate() {
            if !only.is_empty() && !only.contains(&k) {
                continue;
            }
            let started = std::time::Instant::now();
            let mut params = json!({"scan": p["id"], "clean": false});
            params.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
            let out = s.execute("scan.to_solid", params);
            let secs = started.elapsed().as_secs_f64();
            let Ok(out) = out else {
                eprintln!("C{n:02} part {k}: ERROR in {secs:.1} s");
                continue;
            };
            let attempts: Vec<String> = out["attempts"]
                .as_array()
                .into_iter()
                .flatten()
                .map(|a| format!("{}:{}:{:.1}", a["method"], a["verdict"], a["seconds"].as_f64().unwrap_or(0.0)))
                .collect();
            let prep: Vec<&str> =
                out["preparation"].as_array().into_iter().flatten().filter_map(|x| x["step"].as_str()).collect();
            let codes: Vec<&str> =
                out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str()).collect();
            eprintln!(
                "C{n:02} part {k:2}: tris {:6} closed {} genus {} {:6.1} s verdict {} method {} patches {} within {:.4} retry {} prep {prep:?} attempts {attempts:?} issues {codes:?}",
                p["triangles"],
                p["closed"],
                p["genus"],
                secs,
                out["verdict"],
                out["method"],
                out["patches"],
                out["deviation"]["within"].as_f64().unwrap_or(0.0),
                out["retry"]["reason"],
            );
            for x in out["preparation"].as_array().into_iter().flatten() {
                eprintln!("    preparation {}: {}", x["step"], x["numbers"]);
            }
        }
    }
}
