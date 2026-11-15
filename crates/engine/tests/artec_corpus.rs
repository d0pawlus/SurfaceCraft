//! The open test corpus: four scans published by Artec Group inc. (www.artec3d.com) under a
//! Creative Commons Attribution licence (ATTRIBUTION.md, "External test models"). The files are
//! 105 to 190 MB each, so they stay outside the repository: `xtask/fetch-artec-corpus.sh <dir>`
//! downloads them and checks their SHA-256, `SURFACECRAFT_ARTEC_CORPUS` names that directory.
//! Unlike the scans of `SURFACECRAFT_LOCAL_CORPUS`, these may be named and their figures committed.
//! The same directory also works as `SURFACECRAFT_LOCAL_CORPUS` for the `local_*` tests.
//!
//! Run: `SURFACECRAFT_ARTEC_CORPUS=… cargo test --profile ci -p surfacecraft-engine --test artec_corpus -- --ignored --nocapture`
use serde_json::json;
use std::path::{Path, PathBuf};
use std::time::Instant;
use surfacecraft_engine::Session;

/// One model of the corpus: the file name written by the fetch script and the number of
/// triangles `mesh.import` keeps (the faces stated in the file, less those that repeat a vertex).
struct Model {
    file: &'static str,
    triangles: u64,
}

const MODELS: &[Model] = &[
    Model { file: "car-rim.stl", triangles: 2_500_000 },
    Model { file: "four-cylinder-engine.ply", triangles: 9_994_566 },
    // 2 500 000 faces in the file, four of them with a repeated vertex index
    Model { file: "industrial-part.obj", triangles: 2_499_996 },
    Model { file: "water-pump.ply", triangles: 8_146_012 },
];

/// The corpus directory, or `None` when the variable is not set.
fn corpus_dir() -> Option<PathBuf> {
    std::env::var("SURFACECRAFT_ARTEC_CORPUS").ok().map(PathBuf::from)
}

fn import(path: &Path) -> Result<(u64, f64), String> {
    let mut session = Session::new();
    let started = Instant::now();
    let info = session
        .execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"}))
        .map_err(|e| e.to_string())?;
    let triangles = info["triangles"].as_u64().ok_or("mesh.import reported no triangle count")?;
    Ok((triangles, started.elapsed().as_secs_f64()))
}

#[test]
#[ignore = "needs SURFACECRAFT_ARTEC_CORPUS (xtask/fetch-artec-corpus.sh; large files, not in the repository)"]
fn artec_corpus_imports_every_model() {
    let dir = corpus_dir().expect("set SURFACECRAFT_ARTEC_CORPUS to the directory of xtask/fetch-artec-corpus.sh");
    let mut failures = Vec::new();
    for model in MODELS {
        let path = dir.join(model.file);
        match import(&path) {
            Ok((triangles, seconds)) => {
                eprintln!("{:<26} triangles {triangles:>8} import {seconds:.1} s", model.file);
                if triangles != model.triangles {
                    failures.push(format!("{}: {triangles} triangles, expected {}", model.file, model.triangles));
                }
            }
            Err(message) => failures.push(format!("{}: {message}", model.file)),
        }
    }
    assert!(failures.is_empty(), "{failures:#?}");
}
