//! P9-O6 on real scans that stay outside the repository: `scan.to_solid` with the freeform, the
//! hybrid and the auto method, side by side. Only aggregate figures are printed; files are reported
//! by number. Never run in CI.
//!
//! `SURFACECRAFT_LOCAL_CORPUS=… SURFACECRAFT_LOCAL_CORPUS_NUMBERS=1,4 cargo test --profile ci -p surfacecraft-engine --test local_hybrid -- --ignored --nocapture`
use serde_json::{Value, json};
use std::time::Instant;
use surfacecraft_engine::Session;

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

fn codes(out: &Value) -> Vec<String> {
    out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str().map(str::to_string)).collect()
}

fn line(number: usize, method: &str, out: &Value, seconds: f64) {
    let attempts: Vec<String> = out["attempts"]
        .as_array()
        .into_iter()
        .flatten()
        .map(|a| {
            format!(
                "{}:{}{}",
                a["method"].as_str().unwrap_or("?"),
                a["verdict"].as_str().unwrap_or("-"),
                if a["returned"] == true { "*" } else { "" }
            )
        })
        .collect();
    eprintln!(
        "C{number:02} {method}: method {} verdict {} in {seconds:.1} s; patches {} faces {} within {:.4} coverage {:.4} max {:.3}; analytic {} ; issues {:?}; attempts {attempts:?}",
        out["method"],
        out["verdict"],
        out["patches"],
        out["body"]["faces"],
        out["deviation"]["within"].as_f64().unwrap_or(f64::NAN),
        out["deviation"]["coverage"].as_f64().unwrap_or(f64::NAN),
        out["deviation"]["max_abs"].as_f64().unwrap_or(f64::NAN),
        out["analytic"],
        codes(out),
    );
    for i in out["issues"].as_array().into_iter().flatten() {
        if i["code"].as_str().is_some_and(|c| c.starts_with("hybrid_")) {
            eprintln!("C{number:02}   {method} issue {}: {}", i["code"], i["message"]);
        }
    }
    for m in out["modelled"].as_array().into_iter().flatten() {
        eprintln!(
            "C{number:02}   {method} primitive {} {} triangles, role {}, faces {}, skipped {}",
            m["kind"], m["triangles"], m["role"], m["faces"], m["skipped"]
        );
    }
}

fn run_method(s: &mut Session, number: usize, method: &str) -> Result<(), String> {
    let started = Instant::now();
    let out = s
        .execute("scan.to_solid", json!({"scan": "scan", "method": method, "name": method}))
        .map_err(|e| e.to_string())?;
    line(number, method, &out, started.elapsed().as_secs_f64());
    Ok(())
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_hybrid() {
    let files = corpus();
    assert!(!files.is_empty(), "set SURFACECRAFT_LOCAL_CORPUS to a directory with STL/OBJ/PLY scans");
    let methods: Vec<String> = std::env::var("SURFACECRAFT_LOCAL_METHODS")
        .unwrap_or_else(|_| "freeform,hybrid,auto".to_string())
        .split(',')
        .map(str::to_string)
        .collect();
    for (number, path) in &files {
        for method in &methods {
            // a fresh session per run: every method sees the scan as imported
            let mut s = Session::new();
            let info = s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"}));
            if let Err(e) = info {
                eprintln!("C{number:02} import ERROR {e}");
                continue;
            }
            if let Err(e) = run_method(&mut s, *number, method) {
                eprintln!("C{number:02} {method} ERROR {e}");
            }
        }
    }
}
