//! `scan.to_solid` with a `thickness` on the open real scans of the local corpus (numbers 4, 5, 6 and
//! 9 of the sorted files are one-sided). Third-party scans never enter the repository and are
//! reported by number only; see `local_to_solid.rs`.
//!
//! Run (release profile, 1 to 6 minutes per scan):
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --profile ci -p surfacecraft-engine --test local_shell -- --ignored --nocapture`
//! `SURFACECRAFT_LOCAL_SCANS` (default `4,5,6,9`) selects scans by number, `SURFACECRAFT_LOCAL_THICKNESS`
//! (default 2) is the wall in mm.
use serde_json::{Value, json};
use std::time::Instant;
use surfacecraft_engine::Session;

const DEFAULT_SCANS: &str = "4,5,6,9";
const DEFAULT_THICKNESS_MM: f64 = 2.0;

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

fn wanted() -> Vec<usize> {
    let list = std::env::var("SURFACECRAFT_LOCAL_SCANS").unwrap_or_else(|_| DEFAULT_SCANS.to_string());
    list.split(',').filter_map(|n| n.trim().parse().ok()).collect()
}

fn codes(out: &Value) -> Vec<String> {
    out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str().map(str::to_string)).collect()
}

fn run_one(number: usize, path: &std::path::Path, thickness: f64) -> Result<String, String> {
    let mut s = Session::new();
    let e = |x: surfacecraft_engine::EngineError| x.to_string();
    let info = s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).map_err(e)?;
    let qa = s.execute("scan.qa", json!({"object": "scan"})).map_err(e)?;
    eprintln!(
        "S{number:02} bbox {} {} edge {} sigma {} holes {}",
        qa["bbox_min"], qa["bbox_max"], qa["edge_length"], qa["noise_sigma"], qa["holes"]
    );
    let started = Instant::now();
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "thickness": thickness})).map_err(e)?;
    let seconds = started.elapsed().as_secs_f64();
    let verdict = out["verdict"].as_str().unwrap_or("?").to_string();
    eprintln!(
        "S{number:02} triangles {} thickness {thickness} verdict {verdict} in {seconds:.1} s; patches {} closed {} volume {}; \
         coverage {} within {} rms {} max {} not_facing {}; clamped {} (largest reduction {}) loops {} thickened triangles {}; \
         holes left {}; retry {}; issues {:?}",
        info["triangles"],
        out["patches"],
        out["body"]["closed"],
        out["body"]["volume"],
        out["deviation"]["coverage"],
        out["deviation"]["within"],
        out["deviation"]["rms"],
        out["deviation"]["max_abs"],
        out["deviation"]["not_facing"],
        out["thickened"]["clamped_vertices"],
        out["thickened"]["largest_reduction"],
        out["thickened"]["boundary_loops"],
        out["thickened"]["triangles"],
        out["scan_report"]["holes_left"],
        out["retry"]["reason"],
        codes(&out),
    );
    if verdict == "fail" {
        eprintln!("S{number:02} {}", out["issues"][0]["message"]);
    }
    Ok(verdict)
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_open_scans_become_solids() {
    let files = corpus();
    assert!(!files.is_empty(), "set SURFACECRAFT_LOCAL_CORPUS to a directory with STL/OBJ/PLY scans");
    let thickness =
        std::env::var("SURFACECRAFT_LOCAL_THICKNESS").ok().and_then(|t| t.parse().ok()).unwrap_or(DEFAULT_THICKNESS_MM);
    let mut errors = Vec::new();
    for number in wanted() {
        let Some(path) = number.checked_sub(1).and_then(|k| files.get(k)) else {
            eprintln!("S{number:02} not in the corpus");
            continue;
        };
        if let Err(e) = run_one(number, path, thickness) {
            eprintln!("S{number:02} ERROR {e}");
            errors.push(number);
        }
    }
    assert!(errors.is_empty(), "scans that ended in an error instead of a verdict: {errors:?}");
}

/// Percentiles 5, 25, 50, 75, 95 of `values` (sorted in place).
fn percentiles(values: &mut [f64]) -> Vec<f64> {
    values.sort_by(f64::total_cmp);
    [0.05, 0.25, 0.5, 0.75, 0.95]
        .iter()
        .map(|q| values.get(((values.len() as f64 - 1.0) * q) as usize).copied().unwrap_or(f64::NAN))
        .collect()
}

/// Diagnostic: the radius of curvature on the offset side at the reduced vertices of one scan.
#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_radius_of_the_reduced_vertices() {
    use surfacecraft_geom::shell::{ThickenParams, thicken};
    let files = corpus();
    let number = wanted().first().copied().unwrap_or(4);
    let thickness =
        std::env::var("SURFACECRAFT_LOCAL_THICKNESS").ok().and_then(|t| t.parse().ok()).unwrap_or(DEFAULT_THICKNESS_MM);
    let path = files.get(number - 1).expect("scan in the corpus");
    let mut s = Session::new();
    s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
    let qa = s.execute("scan.qa", json!({"object": "scan"})).unwrap();
    let rim = qa["hole_list"][0]["vertices"].as_u64().unwrap_or(1) as usize;
    s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400.min(rim - 1)})).unwrap();
    let id = s.doc.find_by_name("scan").unwrap().0;
    let mesh = (*s.doc.mesh(id).unwrap().mesh).clone();
    let shell = thicken(&mesh, &ThickenParams { thickness, symmetric: false }).unwrap();
    let face = shell.scan_face();
    let normals = face.vertex_normals();
    let mut ring: Vec<Vec<u32>> = vec![Vec::new(); face.vertex_count()];
    for t in &face.triangles {
        for k in 0..3 {
            ring[t[k] as usize].push(t[(k + 1) % 3]);
        }
    }
    // curvature towards the offset side (against the normals for a positive thickness)
    let side = if thickness > 0.0 { 1.0 } else { -1.0 };
    let mut reduced_r: Vec<f64> = Vec::new();
    for (i, &s) in shell.scale.iter().enumerate() {
        if s >= 1.0 {
            continue;
        }
        let c = ring[i]
            .iter()
            .map(|&j| {
                let e = face.positions[j as usize] - face.positions[i];
                side * (normals[j as usize] - normals[i]).dot(&e) / e.norm_squared().max(1e-18)
            })
            .fold(0.0, f64::max);
        reduced_r.push(if c > 1e-9 { 1.0 / c } else { f64::INFINITY });
    }
    eprintln!(
        "S{number:02} thickness {thickness}: {} of {} vertices reduced, {} folded; median reduction {:.3} mm; radius percentiles 5/25/50/75/95 at reduced vertices: {:?}",
        reduced_r.len(),
        face.vertex_count(),
        shell.report.folded_vertices,
        shell.report.median_reduction,
        percentiles(&mut reduced_r)
    );
}
