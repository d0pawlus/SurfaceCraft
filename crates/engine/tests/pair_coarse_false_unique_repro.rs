//! REPRODUCTION OF A KNOWN DEFECT of `align.pair_coarse` (found while building `align.global`,
//! P9-M3): the command returns a pose that is wrong against the known truth with the verdict
//! `unique`, its design promise being never to do that. This test only measures and prints; it has
//! no assertion that could fail CI and is `#[ignore]`d. Run it with
//! `cargo test -p surfacecraft-engine --test pair_coarse_false_unique_repro -- --ignored --nocapture`
//! and read the table at the end.
//!
//! For every pair `(i, j)` of the views of a fixture (moving = j, fixed = i) it compares the pose
//! with the truth `P_i P_j^-1` (largest displacement of the vertices of view j) and lists every pair
//! that is `unique` and more than 1 mm off, with the command's own evidence. Fixtures (all views:
//! sigma 0.05, own tessellation, own random motion `random_motion(k)`):
//! - `bracket`: `bracket_views(6)` (half-space cuts, see `global_common`), noise seeds 100 + k;
//! - `blob`: `blob_views()`, seeds 200 + k, `blob_at(48 + 4k)`;
//! - `blob wedges`: six wedges of the blob, azimuth 60 k +- 70 degrees, seeds 900 + k;
//! - `ring`: `ring_arc(k, 45 k, 58)` for k in 0..8, a torus with twelve bumps, seeds 600 + k.
mod global_common;
mod pair_common;
use global_common::*;
use pair_common::*;
use serde_json::{Value, json};
use surfacecraft_geom::{Mesh, Transform};

fn wedge(k: usize) -> Mesh {
    let whole = noisy(blob_at(48 + 4 * k), 900 + k as u64);
    cut(&whole, |p| {
        let d = (p.y.atan2(p.x).to_degrees() - 60.0 * k as f64 + 540.0).rem_euclid(360.0) - 180.0;
        d.abs() <= 70.0
    })
}

struct Row {
    fixture: &'static str,
    i: usize,
    j: usize,
    truth: (f64, f64),
    error: (f64, f64),
    rms_per_diagonal: f64,
    out: Value,
}

fn f(v: &Value) -> f64 {
    v.as_f64().unwrap_or(f64::NAN)
}

fn scan(fixture: &'static str, views: &[Mesh]) -> (Vec<Row>, usize) {
    let poses = poses(views.len());
    let mut s = session_of(views, &poses);
    let (mut rows, mut unique) = (Vec::new(), 0);
    for i in 0..views.len() {
        for j in i + 1..views.len() {
            let params = json!({"object": format!("v{j}"), "target": format!("v{i}")});
            let Ok(out) = s.execute("align.pair_coarse", params) else { continue };
            if out["verdict"] != "unique" {
                continue;
            }
            unique += 1;
            let truth_pose = poses[i].then_after(&poses[j].inverse());
            let found = Transform::from_rows(&serde_json::from_value(out["matrix"].clone()).unwrap()).unwrap();
            let on = views[j].transformed(&poses[j]);
            let error = found.max_displacement(&truth_pose, &on.positions);
            if error > 1.0 {
                let q = truth_pose.inverse().then_after(&found).quaternion();
                let angle = (2.0 * q.vector().norm().atan2(q.scalar().abs())).to_degrees();
                let truth = (true_share(&views[j], &views[i], 0.3), true_share(&views[i], &views[j], 0.3));
                let rms_per_diagonal = f(&out["rms"]) / views[i].bbox().diagonal();
                rows.push(Row { fixture, i, j, truth, error: (error, angle), rms_per_diagonal, out });
            }
        }
    }
    (rows, unique)
}

#[test]
#[ignore = "reproduction of a known defect of align.pair_coarse (false `unique`): prints a table, asserts nothing"]
fn known_defect_pair_coarse_returns_wrong_poses_as_unique() {
    let fixtures: Vec<(&'static str, Vec<Mesh>)> = vec![
        ("bracket", bracket_views(6)),
        ("blob", blob_views()),
        ("blob wedges", (0..6).map(wedge).collect()),
        ("ring", (0..8).map(|k| ring_arc(k, 45.0 * k as f64, 58.0)).collect()),
    ];
    let mut rows = Vec::new();
    for (name, views) in &fixtures {
        let (found, unique) = scan(name, views);
        eprintln!(
            "{name}: {} views, {unique} unique pairs, {} of them wrong by more than 1 mm",
            views.len(),
            found.len()
        );
        rows.extend(found);
    }
    eprintln!(
        "\nTABLE: unique pairs more than 1 mm off the truth\nfixture | pair (fixed-moving) | true overlap m/f | error mm, deg | confidence | overlap m/f | rms | rms/diagonal | tol | stability | probed/failed/slides | probe_margin | return_gap | refined | alternatives | candidates | method | icp converged"
    );
    for r in &rows {
        let o = &r.out;
        eprintln!(
            "{} | {}-{} | {:.2}/{:.2} | {:.1}, {:.2} | {:.2} | {:.2}/{:.2} | {:.3} | {:.5} | {:.3} | {:.4} | {}/{}/{} | {} | {:.3} | {} | {} | {} | {} | {}",
            r.fixture,
            r.i,
            r.j,
            r.truth.0,
            r.truth.1,
            r.error.0,
            r.error.1,
            f(&o["confidence"]),
            f(&o["overlap_moving"]),
            f(&o["overlap_fixed"]),
            f(&o["rms"]),
            r.rms_per_diagonal,
            f(&o["tolerance"]),
            f(&o["stability"]),
            o["probed"],
            o["probes_failed"],
            o["slides"],
            o["probe_margin"],
            f(&o["probe_return_gap"]),
            o["refined"],
            o["alternatives"].as_array().map_or(0, Vec::len),
            o["candidates"],
            o["method"],
            o["icp_converged"]
        );
    }
}
