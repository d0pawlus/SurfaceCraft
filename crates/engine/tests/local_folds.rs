//! Fold repair on the real scans of the local corpus (numbers only, scans are named by their number
//! and never enter the repository): what `surfacecraft_geom::folds` finds and removes, and what the
//! shell makes of the scan before and after.
//!
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --profile ci -p surfacecraft-engine --test local_folds -- --ignored --nocapture`
//! `SURFACECRAFT_LOCAL_SCANS` (default `4,5,6,9`) selects scans by number, `SURFACECRAFT_LOCAL_THICKNESS` (default 1) is the wall (mm).
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_geom::shell::{ThickenError, ThickenParams, thicken};
use surfacecraft_geom::{Mesh, edit, folds};

fn files() -> Vec<std::path::PathBuf> {
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

fn wanted(default: &str) -> Vec<usize> {
    let list = std::env::var("SURFACECRAFT_LOCAL_SCANS").unwrap_or_else(|_| default.to_string());
    list.split(',').filter_map(|n| n.trim().parse().ok()).collect()
}

/// The scan as `scan.to_solid` hands it to the shell before the fold step existed: cleaned, holes up to 400 edges filled, rim kept.
fn prepared(number: usize) -> Option<(Mesh, f64, usize)> {
    let path = files().get(number.checked_sub(1)?)?.clone();
    let mut s = Session::new();
    s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).ok()?;
    let sigma = s.execute("scan.qa", json!({"object": "scan"})).ok()?["noise_sigma"].as_f64().unwrap_or(f64::NAN);
    s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 0, "remove_folds": false})).ok()?;
    let id = s.doc.find_by_name("scan")?.0;
    let scan_triangles = s.doc.mesh(id)?.mesh.triangle_count();
    let rim = edit::boundary_loops(&s.doc.mesh(id)?.mesh).iter().map(Vec::len).max().unwrap_or(0);
    if rim > 0 {
        s.execute("mesh.fill_holes", json!({"object": "scan", "max_edges": 400.min(rim.saturating_sub(1))})).ok()?;
    }
    Some(((*s.doc.mesh(id)?.mesh).clone(), sigma, scan_triangles))
}

fn shell_line(m: &Mesh, wall: f64) -> String {
    match thicken(m, &ThickenParams { thickness: wall, symmetric: false }) {
        Ok(t) => format!(
            "shell ok, unresolved {}, far crossings {}",
            t.report.unresolved_vertices, t.report.crossing_triangles
        ),
        Err(ThickenError::UnresolvedCrossings { vertices, share_percent, .. }) => {
            format!("shell refused, unresolved {vertices} ({share_percent:.2} %)")
        }
        Err(e) => format!("shell refused: {}", e.to_string().chars().take(70).collect::<String>()),
    }
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_fold_repair_makes_the_shell_resolve() {
    let wall: f64 = std::env::var("SURFACECRAFT_LOCAL_THICKNESS").ok().and_then(|t| t.parse().ok()).unwrap_or(1.0);
    for number in wanted("4,5,6,9") {
        let Some((mut m, sigma, scan_triangles)) = prepared(number) else { continue };
        let found = folds::find_folds(&m);
        eprintln!(
            "S{number:02} sigma {sigma:.4} triangles {} folds found {} (small {})",
            m.triangle_count(),
            found.len(),
            found.iter().filter(|f| f.small).count()
        );
        let mut sorted = found.clone();
        sorted.sort_by(|a, b| b.extent.total_cmp(&a.extent));
        for f in sorted.iter().take(12).chain(sorted.iter().rev().take(3)) {
            eprintln!(
                "S{number:02}   fold: {} triangles, area {:.3} mm2, extent {:.2} mm, height {:.3} mm, rim {}, crossing {}, small {}",
                f.triangles, f.area, f.extent, f.height, f.on_rim, f.crossing, f.small
            );
        }
        eprintln!("S{number:02} before: {}", shell_line(&m, wall));
        let t = std::time::Instant::now();
        let r = folds::remove_folds(&mut m, 0.2, &surfacecraft_geom::Cancel::new(), scan_triangles);
        eprintln!(
            "S{number:02} removed {} groups ({} triangles, {:.3} mm2, tallest {:.3} mm), added {}, refused {:?}, left {} in {:.1} s; largest left {:?}",
            r.removed,
            r.removed_triangles,
            r.removed_area,
            r.largest_removed_height,
            r.added_triangles,
            r.refused,
            r.left_total,
            t.elapsed().as_secs_f64(),
            r.left.first().map(|l| (l.triangles, (l.extent * 100.0).round() / 100.0))
        );
        eprintln!("S{number:02} after:  {}", shell_line(&m, wall));
        for f in folds::find_folds(&m) {
            eprintln!(
                "S{number:02}   remaining: {} triangles, area {:.3} mm2, extent {:.2} mm, height {:.3} mm, rim {}, crossing {}, small {}",
                f.triangles, f.area, f.extent, f.height, f.on_rim, f.crossing, f.small
            );
        }
    }
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_fold_census() {
    for number in wanted("1,2,3,4,5,6,7,8,9,10,11,12,13") {
        let Some((mut m, _, scan_triangles)) = prepared(number) else { continue };
        let (before, area) = (m.triangle_count(), m.area());
        let t = std::time::Instant::now();
        let r = folds::remove_folds(&mut m, 0.2, &surfacecraft_geom::Cancel::new(), scan_triangles);
        eprintln!(
            "S{number:02} triangles {before}: folds found {}, removed {} ({} triangles, {:.4} % of the area), refused {:?}, left {}, {:.1} s",
            r.found,
            r.removed,
            r.removed_triangles,
            100.0 * r.removed_area / area,
            r.refused,
            r.left_total,
            t.elapsed().as_secs_f64()
        );
    }
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_small_folds_in_numbers() {
    for number in wanted("3") {
        let Some((m, _, _)) = prepared(number) else { continue };
        let removable: Vec<_> = folds::find_folds(&m).into_iter().filter(|f| f.small).collect();
        let q = |mut v: Vec<f64>| {
            v.sort_by(f64::total_cmp);
            [0.0, 0.25, 0.5, 0.75, 1.0]
                .map(|p| v.get(((v.len() as f64 - 1.0) * p) as usize).copied().unwrap_or(f64::NAN))
        };
        eprintln!(
            "S{number:02} {} removable groups; triangles min/q1/med/q3/max {:?}",
            removable.len(),
            q(removable.iter().map(|f| f.triangles as f64).collect())
        );
        eprintln!(
            "S{number:02} extent mm {:?}; height mm {:?}; area mm2 {:?}; on rim {}",
            q(removable.iter().map(|f| f.extent).collect()),
            q(removable.iter().map(|f| f.height).collect()),
            q(removable.iter().map(|f| f.area).collect()),
            removable.iter().filter(|f| f.on_rim).count()
        );
        let qa = |p: &Mesh| {
            let mut l: Vec<f64> = p
                .triangles
                .iter()
                .take(20000)
                .map(|t| (p.positions[t[0] as usize] - p.positions[t[1] as usize]).norm())
                .collect();
            l.sort_by(f64::total_cmp);
            l[l.len() / 2]
        };
        eprintln!("S{number:02} median edge {:.3} mm, boundary loops {}", qa(&m), edit::boundary_loops(&m).len());
    }
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_clean_with_folds_then_fill() {
    for number in wanted("5") {
        let path = files()[number - 1].clone();
        let mut s = Session::new();
        s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
        let out = s
            .execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 0, "remove_folds": true}))
            .unwrap();
        eprintln!("S{number:02} clean: {}", out["folds"]);
        let id = s.doc.find_by_name("scan").unwrap().0;
        let rim = edit::boundary_loops(&s.doc.mesh(id).unwrap().mesh).iter().map(Vec::len).max().unwrap_or(0);
        s.execute("mesh.fill_holes", json!({"object": "scan", "max_edges": 400.min(rim - 1)})).unwrap();
        let m = (*s.doc.mesh(id).unwrap().mesh).clone();
        eprintln!("S{number:02} folds after: {:?}", folds::find_folds(&m).iter().filter(|f| f.crossing).count());
        eprintln!("S{number:02} {}", shell_line(&m, 1.0));
    }
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_which_fill_makes_the_fold() {
    for number in wanted("5") {
        let path = files()[number - 1].clone();
        for (label, fill) in [("no fill", None), ("flat fill", Some(false)), ("smooth fill", Some(true))] {
            let mut s = Session::new();
            s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
            s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 0})).unwrap();
            let id = s.doc.find_by_name("scan").unwrap().0;
            let rim = edit::boundary_loops(&s.doc.mesh(id).unwrap().mesh).iter().map(Vec::len).max().unwrap_or(0);
            if let Some(smooth) = fill {
                s.execute(
                    "mesh.fill_holes",
                    json!({"object": "scan", "max_edges": 400.min(rim - 1), "smooth": smooth}),
                )
                .unwrap();
            }
            let m = (*s.doc.mesh(id).unwrap().mesh).clone();
            let crossing = folds::find_folds(&m).iter().filter(|f| f.crossing).count();
            eprintln!("S{number:02} {label}: crossing groups {crossing}; {}", shell_line(&m, 1.0));
        }
    }
}

/// A: the scan cleaned and filled as the pipeline does, no fold removal; B: `scan.to_solid` as it is.
#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_fold_removal_a_b() {
    for number in wanted("5,6,4,9") {
        let path = files()[number - 1].clone();
        for (label, with_folds) in [("A absent ", false), ("B on     ", true)] {
            let mut s = Session::new();
            s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).unwrap();
            let mut p = json!({"scan": "scan", "thickness": 1.0});
            if with_folds {
                p["remove_folds"] = json!(true);
            }
            let out = s.execute("scan.to_solid", p).unwrap();
            let codes: Vec<&str> =
                out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str()).collect();
            let f = &out["thickened"]["folds"];
            let offer =
                out["next_steps"].as_array().into_iter().flatten().find(|n| n["params"]["remove_folds"] == json!(true));
            if let Some(o) = offer {
                eprintln!("S{number:02} {label}: offers {}", o["why"]);
            }
            eprintln!(
                "S{number:02} {label}: verdict {} within {:.4} coverage {:.4} max {:.3} unresolved {} issues {codes:?}; folds removed {} triangles {} share {:.5} height {:.3} fill {:.3} refused {:?} left {}; removed points {} (not facing {}) within {:.3} max {:.3}",
                out["verdict"],
                out["deviation"]["within"].as_f64().unwrap_or(f64::NAN),
                out["deviation"]["coverage"].as_f64().unwrap_or(f64::NAN),
                out["deviation"]["max_abs"].as_f64().unwrap_or(f64::NAN),
                out["thickened"]["unresolved_vertices"],
                f["removed"],
                f["removed_triangles"],
                f["removed_area_share"].as_f64().unwrap_or(0.0),
                f["largest_removed_height"].as_f64().unwrap_or(0.0),
                f["largest_fill_deviation"].as_f64().unwrap_or(0.0),
                f["refused"],
                f["left"],
                f["removed_points"],
                f["removed_points_not_facing"],
                f["removed_points_within"].as_f64().unwrap_or(0.0),
                f["removed_points_max_deviation"].as_f64().unwrap_or(0.0)
            );
        }
    }
}
