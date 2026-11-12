//! `scan.to_solid` on real scans that stay outside the repository: how often one call reaches a
//! pass, and what it says when it does not.
//!
//! `SURFACECRAFT_LOCAL_CORPUS` names a directory of third-party scans (STL/OBJ/PLY). They are
//! never committed and nothing derived from a single file goes into the repository;
//! `docs/STATUS.md` keeps only the aggregate figures. Files are reported by number only.
//!
//! Run (release, the files are large):
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --release -p surfacecraft-engine --test local_to_solid -- --ignored --nocapture`
use serde_json::{Value, json};
use std::time::Instant;
use surfacecraft_engine::Session;

fn corpus() -> Vec<std::path::PathBuf> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut files: Vec<_> =
        std::fs::read_dir(dir).map(|rd| rd.filter_map(Result::ok).map(|e| e.path()).collect()).unwrap_or_default();
    files.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    if let Ok(only) = std::env::var("SURFACECRAFT_LOCAL_CORPUS_ONLY") {
        files.retain(|p| p.to_string_lossy().contains(&only));
    }
    files.sort();
    files
}

fn codes(out: &Value) -> Vec<String> {
    out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str().map(str::to_string)).collect()
}

/// Runs the command on one file and prints one line of figures. Returns the verdict, or the error.
fn run_one(number: usize, path: &std::path::Path) -> Result<String, String> {
    let mut s = Session::new();
    let e = |x: surfacecraft_engine::EngineError| x.to_string();
    let info = s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).map_err(e)?;
    let started = Instant::now();
    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).map_err(e)?;
    let seconds = started.elapsed().as_secs_f64();
    let verdict = out["verdict"].as_str().unwrap_or("?").to_string();
    eprintln!(
        "C{number:02} triangles {} method {} patches {} verdict {verdict} in {seconds:.1} s; modelled {} left out {} ({} tri) unassigned {}; \
         faces {} volume {} coverage {} within {} rms {} max {}; sigma {}; holes left {}; issues {:?}; steps {:?}",
        info["triangles"],
        out["method"],
        out["patches"],
        out["modelled"].as_array().map_or(0, Vec::len),
        out["left_out_regions"],
        out["left_out_triangles"],
        out["unassigned_triangles"],
        out["body"]["faces"],
        out["body"]["volume"],
        out["deviation"]["coverage"],
        out["deviation"]["within"],
        out["deviation"]["rms"],
        out["deviation"]["max_abs"],
        out["scan_report"]["noise_sigma"],
        out["scan_report"]["holes_left"],
        codes(&out),
        out["next_steps"].as_array().into_iter().flatten().filter_map(|x| x["command"].as_str()).collect::<Vec<_>>(),
    );
    // whatever the verdict, the answer must be usable: a non-pass names its issues and next steps
    if verdict != "pass" && (codes(&out).is_empty() || out["next_steps"].as_array().is_none_or(Vec::is_empty)) {
        return Err(format!("verdict {verdict} without issues or next steps"));
    }
    Ok(verdict)
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_scan_to_solid() {
    let files = corpus();
    assert!(!files.is_empty(), "set SURFACECRAFT_LOCAL_CORPUS to a directory with STL/OBJ/PLY scans");
    let mut tally = std::collections::BTreeMap::<String, usize>::new();
    let mut errors = Vec::new();
    for (k, f) in files.iter().enumerate() {
        match run_one(k + 1, f) {
            Ok(verdict) => *tally.entry(verdict).or_default() += 1,
            Err(e) => {
                eprintln!("C{:02} ERROR {e}", k + 1);
                errors.push(k + 1);
            }
        }
    }
    eprintln!("verdicts over {} files: {tally:?}; errors: {errors:?}", files.len());
    assert!(errors.is_empty(), "files that ended in an error instead of a verdict: {errors:?}");
}
