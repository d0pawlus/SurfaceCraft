//! P9-M3, local only (timing and real scans; both `#[ignore]`d, run in release):
//! `cargo test --release -p surfacecraft-engine --test p9_global_local -- --ignored --nocapture`
//! Scans stay outside the repository and are named by number only; nothing is written anywhere.
mod pair_common;
use pair_common::*;
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Transform, Vec3, synth};

/// The triangles on the side of `dir` of `centre`, or at most `reach` beyond it.
fn side(m: &Mesh, centre: &Point3, dir: &Vec3, reach: f64) -> Mesh {
    cut(m, |p| (p - centre).dot(dir) >= -reach)
}

fn directions() -> Vec<Vec3> {
    let mut d: Vec<Vec3> = [Vec3::x(), -Vec3::x(), Vec3::y(), -Vec3::y(), Vec3::z(), -Vec3::z()].to_vec();
    for (sx, sy) in [(1.0, 1.0), (-1.0, 1.0), (1.0, -1.0), (-1.0, -1.0)] {
        d.push(Vec3::new(sx, sy, 1.0).normalize());
    }
    d.push(Vec3::new(1.0, 0.0, -1.0).normalize());
    d.push(Vec3::new(-1.0, 0.0, -1.0).normalize());
    d
}

fn report(s: &mut Session, n: usize, views: &[Mesh], poses: &[Transform], started: std::time::Instant) {
    let names: Vec<String> = (0..n).map(|k| format!("v{k}")).collect();
    let out = s.execute("align.global", json!({"objects": names, "max_pairs": 66})).unwrap();
    eprintln!(
        "align.global on {n} scans: {:.1} s, verdict {}, issues {}, {} pairs tried, loop {}",
        started.elapsed().as_secs_f64(),
        out["verdict"],
        out["issues"],
        out["pairs_tried"],
        out["loop"]
    );
    for (k, sc) in out["scans"].as_array().unwrap().iter().enumerate() {
        if !sc["placed"].as_bool().unwrap() {
            eprintln!("  scan {k}: not placed ({})", sc["unplaced"]);
            continue;
        }
        let found = matrix_of(sc);
        let truth = poses[0].then_after(&poses[k].inverse());
        let on = views[k].transformed(&poses[k]);
        let err = found.max_displacement(&truth, &on.positions);
        eprintln!("  scan {k}: displacement {err:.4} mm");
    }
}

/// Twelve views of a few hundred thousand triangles each: the time, and whether it stays usable.
#[test]
#[ignore = "timing on 12 scans of a few hundred thousand triangles; run locally in release"]
fn local_large_global_cost() {
    let part = |k: usize| synth::refine(&noisy(bracket_at(12 + k, 48 + 4 * k, 16 + k), 50 + k as u64), |_| 0.4);
    let centre = Point3::new(30.0, 20.0, 15.0);
    let views: Vec<Mesh> = directions().iter().enumerate().map(|(k, d)| side(&part(k), &centre, d, 6.0)).collect();
    let sizes: Vec<usize> = views.iter().map(Mesh::triangle_count).collect();
    eprintln!("triangles per view: {sizes:?}");
    let poses: Vec<Transform> = (0..views.len() as u64).map(random_motion).collect();
    let mut s = Session::new();
    for (k, (v, p)) in views.iter().zip(&poses).enumerate() {
        s.doc.add_mesh(&format!("v{k}"), v.transformed(p));
    }
    report(&mut s, views.len(), &views, &poses, std::time::Instant::now());
}

/// One real scan, by number (`SURFACECRAFT_LOCAL_CORPUS`, `SURFACECRAFT_LOCAL_SCAN`, default 1):
/// cut in memory into six overlapping views, each moved by a known motion; prints the recovered
/// poses and the time.
#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS"]
fn local_corpus_global() {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return };
    let mut files: Vec<_> = std::fs::read_dir(dir).unwrap().filter_map(Result::ok).map(|e| e.path()).collect();
    files.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    files.sort();
    let number: usize = std::env::var("SURFACECRAFT_LOCAL_SCAN").ok().and_then(|x| x.parse().ok()).unwrap_or(1);
    let path = files.get(number.saturating_sub(1)).expect("scan number out of range");
    let mut s = Session::new();
    s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "whole"})).unwrap();
    s.execute("mesh.decimate", json!({"object": "whole", "ratio": 0.4})).unwrap();
    let (id, _) = s.doc.find_by_name("whole").unwrap();
    let whole = s.doc.mesh(id).unwrap().mesh.as_ref().clone();
    let bb = whole.bbox();
    let (centre, reach) = (bb.center(), 0.12 * bb.diagonal());
    eprintln!("scan {number}: {} triangles", whole.triangle_count());
    let views: Vec<Mesh> = directions().iter().take(6).map(|d| side(&whole, &centre, d, reach)).collect();
    let poses: Vec<Transform> = (0..views.len() as u64).map(random_motion).collect();
    let mut t = Session::new();
    for (k, (v, p)) in views.iter().zip(&poses).enumerate() {
        t.doc.add_mesh(&format!("v{k}"), v.transformed(p));
    }
    report(&mut t, views.len(), &views, &poses, std::time::Instant::now());
}
