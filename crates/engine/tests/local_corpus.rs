//! P1: the full pipeline on real scans that stay outside the repository.
//!
//! `SURFACECRAFT_LOCAL_CORPUS` names a directory of third-party scans (STL/OBJ/PLY, point clouds
//! as PLY/OBJ without faces or XYZ). They are never
//! committed and nothing derived from a single file (exported meshes, screenshots) goes into the
//! repository; `docs/STATUS.md` keeps only aggregate pass/fail and timing figures.
//!
//! Run (release, the files are millions of triangles):
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --release -p surfacecraft-engine --test local_corpus -- --ignored --nocapture`
use serde_json::json;
use std::time::Instant;
use surfacecraft_engine::Session;

fn corpus() -> Vec<std::path::PathBuf> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut v: Vec<_> =
        std::fs::read_dir(dir).map(|rd| rd.filter_map(Result::ok).map(|e| e.path()).collect()).unwrap_or_default();
    v.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply" | "xyz" | "asc"))
    });
    if let Ok(only) = std::env::var("SURFACECRAFT_LOCAL_CORPUS_ONLY") {
        v.retain(|p| p.to_string_lossy().contains(&only));
    }
    v.sort();
    v
}

/// Metrics of one file; every step must succeed.
#[derive(Debug, Default)]
struct Report {
    triangles: u64,
    /// Point cloud files: points, points left out of the mesh, boundary loops after reconstruction.
    cloud_points: u64,
    cloud_unused: u64,
    cloud_holes_left: u64,
    import_s: f64,
    sigma: f64,
    clean_removed: u64,
    holes_filled: u64,
    segment_s: f64,
    regions: usize,
    assigned: f64,
    primitives_ok: usize,
    primitives: usize,
    icp_s: f64,
    icp_err_mm: f64,
    surface_max: Option<f64>,
    surface_folds: bool,
}

fn run_one(path: &std::path::Path) -> Result<Report, String> {
    let mut s = Session::new();
    let mut r = Report::default();
    let e = |x: surfacecraft_engine::EngineError| x.to_string();
    let t = Instant::now();
    let info = s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).map_err(e)?;
    r.import_s = t.elapsed().as_secs_f64();
    r.triangles = info["triangles"].as_u64().unwrap_or(0);
    if let Some(c) = info.get("point_cloud") {
        // a point cloud: reconstructed into a mesh by the import
        r.cloud_points = c["points"].as_u64().unwrap_or(0);
        r.cloud_unused = c["unused_points"].as_u64().unwrap_or(0);
        r.cloud_holes_left = c["holes_left"].as_u64().unwrap_or(0);
    }

    // clean-up a real scan needs before segmentation
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 300})).map_err(e)?;
    r.clean_removed = c["removed_triangles"].as_u64().unwrap_or(0);
    r.holes_filled = c["holes_filled"].as_u64().unwrap_or(0);

    let n = s.execute("deviation.noise", json!({"object": "scan"})).map_err(e)?;
    r.sigma = n["sigma"].as_f64().unwrap_or(f64::NAN);

    let t = Instant::now();
    let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).map_err(e)?;
    r.segment_s = t.elapsed().as_secs_f64();
    let regions = seg["regions"].as_array().cloned().unwrap_or_default();
    r.regions = regions.len();
    let assigned: u64 = regions.iter().filter_map(|x| x["triangles"].as_u64()).sum();
    let total = s.execute("mesh.info", json!({"object": "scan"})).map_err(e)?["triangles"].as_u64().unwrap_or(1);
    r.assigned = assigned as f64 / total.max(1) as f64;

    // primitives: the largest regions; "ok" = RMS within max(0.1 mm, 2 σ) (design doc §10)
    let mut by_size = regions.clone();
    by_size.sort_by_key(|x| std::cmp::Reverse(x["triangles"].as_u64().unwrap_or(0)));
    let limit = 0.1f64.max(2.0 * r.sigma);
    for reg in by_size.iter().take(10) {
        r.primitives += 1;
        let f = s.execute("primitive.fit", json!({"region": reg["id"]})).map_err(e)?;
        if f["rms"].as_f64().is_some_and(|x| x <= limit) {
            r.primitives_ok += 1;
        }
    }

    // freeform: the largest connected piece of what no primitive took, as one patch when it is a
    // height field over its plane (wrapped or folded pieces need P3/P4 and are counted apart)
    let sid = s.doc.find_by_name("scan").map(|x| x.0).ok_or("scan")?;
    let mesh = s.doc.mesh(sid).ok_or("scan mesh")?.mesh.clone();
    let mut taken = vec![false; mesh.triangle_count()];
    for reg in s.doc.regions_of(sid) {
        for &t in reg.triangles.iter() {
            if let Some(x) = taken.get_mut(t as usize) {
                *x = true;
            }
        }
    }
    let free: Vec<u32> = (0..mesh.triangle_count() as u32).filter(|&t| !taken[t as usize]).collect();
    let pieces = surfacecraft_geom::edit::components(&mesh.submesh(&free));
    if let Some(p) = pieces.first().filter(|p| p.len() >= 500) {
        // `components` indexes the submesh; map back through `free`
        let tris: Vec<u32> = p.iter().filter_map(|&k| free.get(k as usize).copied()).collect();
        match s.execute("surface.fit", json!({"mesh": "scan", "triangles": tris})) {
            Ok(f) => r.surface_max = f["max"].as_f64(),
            Err(_) => r.surface_folds = true,
        }
    }

    // alignment: a copy moved by a known motion goes back by ICP
    s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "moved"})).map_err(e)?;
    // the same clean-up as `scan`, so both copies hold the same vertices
    s.execute("mesh.clean", json!({"object": "moved", "fill_holes_max_edges": 300})).map_err(e)?;
    s.execute(
        "mesh.transform",
        json!({"object": "moved", "rotate": {"axis": [0.3, 0.2, 1.0], "degrees": 3.0}, "translate": [1.5, -0.8, 0.6]}),
    )
    .map_err(e)?;
    let t = Instant::now();
    let icp = s.execute("align.icp", json!({"object": "moved", "target": "scan"})).map_err(e)?;
    eprintln!(
        "  icp: it {} conv {} last {} rms {}",
        icp["iterations"], icp["converged"], icp["last_delta"], icp["align"]["rms"]
    );
    r.icp_s = t.elapsed().as_secs_f64();
    let a = s.doc.find_by_name("moved").map(|x| x.0).ok_or("moved")?;
    let b = s.doc.find_by_name("scan").map(|x| x.0).ok_or("scan")?;
    let (ma, mb) = (s.doc.mesh(a).ok_or("a")?, s.doc.mesh(b).ok_or("b")?);
    // the clean-up changed `scan`'s vertices, so compare by position, not index
    let tree = surfacecraft_geom::KdTree::build(&mb.mesh.positions);
    let mut worst: f64 = 0.0;
    for p in ma.mesh.positions.iter().step_by(97) {
        if let Some((_, d2)) = tree.knn(p, 1).first() {
            worst = worst.max(d2.sqrt());
        }
    }
    r.icp_err_mm = worst;
    Ok(r)
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_full_pipeline() {
    let files = corpus();
    assert!(!files.is_empty(), "set SURFACECRAFT_LOCAL_CORPUS to a directory with STL/OBJ/PLY scans");
    let mut failures = Vec::new();
    for (k, f) in files.iter().enumerate() {
        // files are reported by number only
        match run_one(f) {
            Ok(r) => {
                eprintln!("C{:02} {r:?}", k + 1);
                // gates: every large region within the primitive threshold, ICP back to the truth
                if r.primitives_ok < r.primitives || r.icp_err_mm > 1e-3 {
                    eprintln!("C{:02} below the gate", k + 1);
                    failures.push(k + 1);
                }
            }
            Err(m) => {
                eprintln!("C{:02} FAILED: {m}", k + 1);
                failures.push(k + 1);
            }
        }
    }
    assert!(failures.is_empty(), "failed: {failures:?}");
}
