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
