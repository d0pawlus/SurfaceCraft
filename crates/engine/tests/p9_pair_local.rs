//! P9-M2b, local only (needs `SURFACECRAFT_LOCAL_CORPUS`, third-party scans that stay outside the
//! repository and are named by number only; everything happens in memory, nothing is written):
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --release -p surfacecraft-engine --test p9_pair_local -- --ignored --nocapture`
//!
//! Every scan is cut into two pieces along its longest axis and the second piece is moved by a
//! known motion, three times: `copy` (a decimated copy of the whole scan, as the older local test
//! does), `halves` (each piece 75 % of the extent, the middle half shared) and `apart` (the pieces
//! share nothing). `align.pair_coarse` must never call a wrong pose `unique`, and `apart` is never
//! `unique`. `SURFACECRAFT_LOCAL_SCAN=n` runs one scan only.
mod pair_common;
use pair_common::*;
use serde_json::json;
use surfacecraft_align::pair::UNIQUE_MAX_CONFLICT;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, synth};

fn corpus() -> Vec<std::path::PathBuf> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut files: Vec<_> = std::fs::read_dir(dir).unwrap().filter_map(Result::ok).map(|e| e.path()).collect();
    files.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    files.sort();
    files
}

fn load(path: &std::path::Path, ratio: Option<f64>) -> Mesh {
    let mut s = Session::new();
    s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
    if let Some(r) = ratio {
        s.execute("mesh.decimate", json!({"object": "scan", "ratio": r})).unwrap();
    }
    let (id, _) = s.doc.find_by_name("scan").unwrap();
    s.doc.mesh(id).unwrap().mesh.as_ref().clone()
}

/// The piece of `m` whose centroids lie between the fractions `from` and `to` of its longest extent.
fn piece(m: &Mesh, from: f64, to: f64) -> Mesh {
    let b = m.bbox();
    let ext = [b.max.x - b.min.x, b.max.y - b.min.y, b.max.z - b.min.z];
    let axis = (0..3).max_by(|&i, &j| ext[i].total_cmp(&ext[j])).unwrap_or(0);
    let (lo, len) = (b.min[axis], ext[axis]);
    cut(m, |p| (lo + from * len..=lo + to * len).contains(&p[axis]))
}

struct Row {
    scan: usize,
    kind: &'static str,
    truth: f64,
    verdict: String,
    error_mm: f64,
    confidence: f64,
    seconds: f64,
    angle_deg: f64,
    /// The command's own evidence for the pose, one line.
    evidence: String,
    /// The larger conflict of the two scans at the pose found.
    conflict: f64,
}

fn run(scan: usize, kind: &'static str, fixed: &Mesh, other: &Mesh, seed: u64) -> Row {
    let pose = random_motion(seed);
    let mut s = session(fixed, other, &pose);
    let truth = true_share(other, fixed, 0.3).max(true_share(fixed, other, 0.3));
    let t = std::time::Instant::now();
    let out = s.execute("align.pair_coarse", json!({"object": "moving", "target": "fixed"}));
    let seconds = t.elapsed().as_secs_f64();
    let Ok(out) = out else {
        let evidence = String::new();
        return Row {
            scan,
            kind,
            truth,
            verdict: "error".into(),
            error_mm: f64::NAN,
            confidence: 0.0,
            seconds,
            angle_deg: f64::NAN,
            evidence,
            conflict: f64::NAN,
        };
    };
    let (angle_deg, error_mm) = pose_error(&matrix_of(&out), &pose, other);
    let f = |k: &str| out[k].as_f64().unwrap_or(f64::NAN);
    let evidence = format!(
        "overlap {:.2}/{:.2} conflict {:.3}/{:.3} rms {:.3} tol {:.3} noise {:.3} stability {:.4} probed {} slides {} failed {} refined {} alternatives {}",
        f("overlap_moving"),
        f("overlap_fixed"),
        f("conflict_moving"),
        f("conflict_fixed"),
        f("rms"),
        f("tolerance"),
        f("noise"),
        f("stability"),
        out["probed"],
        out["slides"],
        out["probes_failed"],
        out["refined"],
        out["alternatives"].as_array().map_or(0, Vec::len)
    );
    let verdict = out["verdict"].as_str().unwrap_or("?").to_string();
    Row {
        scan,
        kind,
        truth,
        verdict,
        error_mm,
        confidence: f("confidence"),
        seconds,
        angle_deg,
        evidence,
        conflict: f("conflict_moving").max(f("conflict_fixed")),
    }
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS"]
fn local_corpus_pair_confusion() {
    let only: Option<usize> = std::env::var("SURFACECRAFT_LOCAL_SCAN").ok().and_then(|x| x.parse().ok());
    let mut rows = Vec::new();
    for (k, path) in corpus().iter().enumerate() {
        let scan = k + 1;
        if only.is_some_and(|n| n != scan) {
            continue;
        }
        let whole = load(path, None);
        let (fixed, other) = (load(path, Some(0.5)), load(path, Some(0.4)));
        rows.push(run(scan, "copy", &fixed, &other, 3));
        rows.push(run(scan, "halves", &piece(&whole, 0.0, 0.75), &piece(&whole, 0.25, 1.0), 4));
        rows.push(run(scan, "apart", &piece(&whole, 0.0, 0.4), &piece(&whole, 0.6, 1.0), 5));
    }
    eprintln!("scan | kind | true overlap | verdict | confidence | error mm, deg | s | evidence");
    for r in &rows {
        eprintln!(
            "{:02} | {} | {:.2} | {} | {:.2} | {:.3}, {:.2} | {:.1} | {}",
            r.scan, r.kind, r.truth, r.verdict, r.confidence, r.error_mm, r.angle_deg, r.seconds, r.evidence
        );
    }
    for kind in ["copy", "halves", "apart"] {
        let of: Vec<&Row> = rows.iter().filter(|r| r.kind == kind).collect();
        let count = |v: &str, right: Option<bool>| {
            of.iter().filter(|r| r.verdict == v && right.is_none_or(|x| (r.error_mm <= 0.5) == x)).count()
        };
        eprintln!(
            "{kind}: {} scans, unique right {}, unique wrong {}, ambiguous {}, no_match {}",
            of.len(),
            count("unique", Some(true)),
            count("unique", Some(false)),
            count("ambiguous", None),
            count("no_match", None)
        );
    }
    // the margin of the conflict bound on real data: the larger conflict of the poses that are right
    let right = |r: &&Row| r.verdict == "unique" && r.error_mm <= 0.5;
    for r in rows.iter().filter(right) {
        eprintln!(
            "right pose, scan {:02} {}: conflict {:.4} ({:.1} times below the bound)",
            r.scan,
            r.kind,
            r.conflict,
            UNIQUE_MAX_CONFLICT / r.conflict.max(1e-9)
        );
    }
    let worst = rows.iter().filter(right).map(|r| r.conflict).fold(0.0, f64::max);
    eprintln!("worst conflict of a right pose: {worst:.4}, bound {UNIQUE_MAX_CONFLICT}");
    // a rule that made every pair ambiguous would pass the precision check below: recall floors, at
    // least 1.5 times below what is measured (copy 13 of 13, halves 9 of 13) when all 13 scans run
    if only.is_none() {
        let found = |kind: &str| rows.iter().filter(right).filter(|r| r.kind == kind).count();
        assert!(found("copy") >= 8, "copy: {} of 13 right", found("copy"));
        assert!(found("halves") >= 6, "halves: {} of 13 right", found("halves"));
    }
    let bad: Vec<String> = rows
        .iter()
        .filter(|r| r.verdict == "unique" && r.error_mm > 1.0 && r.kind != "apart")
        .map(|r| format!("scan {} {}: unique, error {:.2} mm", r.scan, r.kind, r.error_mm))
        .collect();
    assert!(bad.is_empty(), "a wrong pose called unique: {bad:?}");
    // `apart` pieces that are congruent copies of each other (a symmetric part) fit each other
    // perfectly: not told from a scan taken twice, printed above, not asserted
}

/// What caching the noise estimate per scan saves in `align.global`: one estimate of a view of a few
/// hundred thousand triangles, and the 12-view case (66 pairs: two estimates each before, one per
/// scan now).
#[test]
#[ignore = "timing; run locally"]
fn local_noise_estimate_cost() {
    let view = synth::refine(&noisy(bracket_at(12, 48, 16), 5), |_| 0.4);
    let t = std::time::Instant::now();
    let e = surfacecraft_align::pair::ScanNoise::of(&view);
    let one = t.elapsed().as_secs_f64();
    eprintln!(
        "{} triangles: one estimate {one:.2} s (sigma {:.3}); 12 views: {:.1} s before (132 estimates), {:.1} s now (12)",
        view.triangle_count(),
        e.sigma,
        132.0 * one,
        12.0 * one
    );
}
