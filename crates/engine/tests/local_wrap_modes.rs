//! `surface.wrap` on real scans that stay outside the repository, adaptive against uniform layout
//! (P9 measurement; no thresholds are asserted, the figures are the result).
//!
//! `SURFACECRAFT_LOCAL_CORPUS` names a directory of third-party scans (STL/OBJ/PLY). They are never
//! committed and nothing derived from a single file goes into the repository; files are reported by
//! number only (sorted order, from 1). Without the variable the test returns silently.
//!
//! - `SURFACECRAFT_WRAP_REFINE`: `adaptive` (default) or `uniform`.
//! - `SURFACECRAFT_WRAP_ONLY`: comma-separated file numbers (default all).
//! - `SURFACECRAFT_WRAP_TIMEOUT_S`: wall-clock limit per file in seconds (default 1200); a file that
//!   exceeds it is reported as `timeout` and the process ends, so run one number per process when
//!   a scan may be too large.
//! - `SURFACECRAFT_WRAP_SCRATCH`: directory for the temporary STEP export (default: the system
//!   temporary directory); the file is deleted right after it is measured.
//! - `SURFACECRAFT_WRAP_CLEAN=1` runs `mesh.clean` (holes up to 400 edges filled) before the wrap, as
//!   `scan.to_solid` does; the default wraps the scan as imported.
//! - `SURFACECRAFT_WRAP_EXTRAS=0` skips the `body.compare` and STEP export measurements.
//!
//! Run (release, the files are large):
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --release -p surfacecraft-engine --test local_wrap_modes -- --ignored --nocapture --test-threads=1`
use serde_json::{Value, json};
use std::path::{Path, PathBuf};
use std::time::{Duration, Instant};
use surfacecraft_engine::Session;

const TOLERANCE: f64 = 0.2;
const DEFAULT_TIMEOUT_S: u64 = 1200;

fn corpus() -> Vec<PathBuf> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut files: Vec<PathBuf> =
        std::fs::read_dir(dir).map(|rd| rd.filter_map(Result::ok).map(|e| e.path()).collect()).unwrap_or_default();
    files.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    files.sort();
    files
}

fn selected(numbers: &[usize], k: usize) -> bool {
    numbers.is_empty() || numbers.contains(&k)
}

fn env_numbers() -> Vec<usize> {
    std::env::var("SURFACECRAFT_WRAP_ONLY")
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_default()
}

/// The error text without anything that could name a file: the first words only.
fn error_class(e: &str) -> String {
    e.chars().filter(|c| !matches!(c, '/' | '\\')).take(120).collect()
}

/// Independent check and STEP size of the wrapped body; returns the printed fragment.
fn extras(s: &mut Session, wrapped: &Value, scratch: &Path, number: usize) -> String {
    let body = wrapped["body"]["id"].clone();
    let compare = match s.execute("body.compare", json!({"body": body, "scan": "scan", "tolerance": TOLERANCE})) {
        Ok(c) => format!("compare within {:.4}", c["totals"]["within"].as_f64().unwrap_or(f64::NAN)),
        Err(e) => format!("compare error {}", error_class(&e.to_string())),
    };
    let file = scratch.join(format!("wrap_modes_c{number:02}.step"));
    let started = Instant::now();
    let step = match s.execute("export.step", json!({"objects": [body], "path": file.to_string_lossy()})) {
        Ok(x) => {
            let bytes = x["bytes"].as_u64().unwrap_or(0);
            let patches = wrapped["patches"].as_u64().unwrap_or(1).max(1);
            format!(
                "step {bytes} bytes {} per patch ({}) in {:.1} s",
                bytes / patches,
                x["content"].as_str().unwrap_or("?"),
                started.elapsed().as_secs_f64()
            )
        }
        Err(e) => format!("step error {}", error_class(&e.to_string())),
    };
    // nothing derived from a scan stays on disk
    let _ = std::fs::remove_file(&file);
    format!("{compare}; {step}")
}

fn rounds(w: &Value) -> String {
    w["levels"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|l| format!("{}:{}p/{:.1}%", l["round"], l["patches"], 100.0 * l["within"].as_f64().unwrap_or(0.0)))
        .collect::<Vec<_>>()
        .join(" ")
}

fn run_one(number: usize, path: &Path, refine: &str, scratch: &Path, with_extras: bool) {
    let mut s = Session::new();
    let info = match s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})) {
        Ok(i) => i,
        Err(e) => {
            eprintln!("C{number:02} [{refine}] import error {}", error_class(&e.to_string()));
            return;
        }
    };
    let triangles = info["triangles"].clone();
    let clean = std::env::var("SURFACECRAFT_WRAP_CLEAN").is_ok_and(|v| v == "1");
    if clean && let Err(e) = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})) {
        eprintln!("C{number:02} [{refine}] clean error {}", error_class(&e.to_string()));
        return;
    }
    let started = Instant::now();
    let result = s.execute(
        "surface.wrap",
        json!({"mesh": "scan", "tolerance": TOLERANCE, "max_patches": 12000, "refine": refine}),
    );
    let seconds = started.elapsed().as_secs_f64();
    let w = match result {
        Ok(w) => w,
        Err(e) => {
            eprintln!(
                "C{number:02} [{refine}] triangles {triangles} ERROR after {seconds:.1} s: {}",
                error_class(&e.to_string())
            );
            return;
        }
    };
    let kind = if w["closed"] == true { "solid" } else { "sheet" };
    let tail = if with_extras { extras(&mut s, &w, scratch, number) } else { String::new() };
    eprintln!(
        "C{number:02} [{refine}] triangles {triangles} {kind} patches {} within {:.4} p99 {:.3} max {:.3} capped {} rounds {} \
         [{}] jump {:.1} deg in {seconds:.1} s; {tail}",
        w["patches"],
        w["deviation"]["within"].as_f64().unwrap_or(f64::NAN),
        w["deviation"]["p99"].as_f64().unwrap_or(f64::NAN),
        w["deviation"]["max"].as_f64().unwrap_or(f64::NAN),
        w["capped"],
        w["rounds"],
        rounds(&w),
        w["normal_jump_mean_deg"].as_f64().unwrap_or(f64::NAN),
    );
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_wrap_modes() {
    let files = corpus();
    if files.is_empty() {
        return;
    }
    let refine = std::env::var("SURFACECRAFT_WRAP_REFINE").unwrap_or_else(|_| "adaptive".into());
    let numbers = env_numbers();
    let timeout_s =
        std::env::var("SURFACECRAFT_WRAP_TIMEOUT_S").ok().and_then(|v| v.parse().ok()).unwrap_or(DEFAULT_TIMEOUT_S);
    let scratch =
        std::env::var("SURFACECRAFT_WRAP_SCRATCH").map(PathBuf::from).unwrap_or_else(|_| std::env::temp_dir());
    let with_extras = std::env::var("SURFACECRAFT_WRAP_EXTRAS").map_or(true, |v| v != "0");
    for (k, f) in files.iter().enumerate() {
        let number = k + 1;
        if !selected(&numbers, number) {
            continue;
        }
        let (path, worker_refine, worker_scratch) = (f.clone(), refine.clone(), scratch.clone());
        let (tx, rx) = std::sync::mpsc::channel();
        std::thread::Builder::new()
            .stack_size(256 << 20)
            .spawn(move || {
                run_one(number, &path, &worker_refine, &worker_scratch, with_extras);
                let _ = tx.send(());
            })
            .expect("worker thread");
        if rx.recv_timeout(Duration::from_secs(timeout_s)).is_err() {
            // the worker cannot be cancelled: report and end the process
            eprintln!("C{number:02} [{refine}] timeout/oom after {timeout_s} s");
            let _ = std::fs::remove_file(scratch.join(format!("wrap_modes_c{number:02}.step")));
            std::process::exit(0);
        }
    }
    eprintln!("done: {} files in the corpus", files.len());
}
