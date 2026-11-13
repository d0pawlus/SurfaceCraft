//! P9-CLEAN: topology of real scans (which stay outside the repository) after each step of
//! `mesh.clean` (measurement only, no thresholds). `SURFACECRAFT_LOCAL_CORPUS` names the
//! directory; files are reported by number only (sorted order, from 1); `SURFACECRAFT_CLEAN_ONLY`
//! is a comma-separated list of file numbers (default all). Without the variable the test returns
//! silently.
//!
//! `SURFACECRAFT_LOCAL_CORPUS=… SURFACECRAFT_CLEAN_ONLY=3 cargo test --profile ci -p surfacecraft-engine --test local_clean -- --ignored --nocapture`
use surfacecraft_geom::Mesh;
use surfacecraft_geom::edit::{
    boundary_loops, compact, fill_holes, remove_non_manifold, remove_outliers, remove_small_components,
};
use surfacecraft_geom::quality::edge_census;
use surfacecraft_io::{Limits, MeshFormat, read_mesh};

/// The fill limit of `scan.to_solid`'s clean.
const FILL_EDGES: usize = 400;

fn corpus() -> Vec<std::path::PathBuf> {
    let Ok(dir) = std::env::var("SURFACECRAFT_LOCAL_CORPUS") else { return Vec::new() };
    let mut v: Vec<_> =
        std::fs::read_dir(dir).map(|rd| rd.filter_map(Result::ok).map(|e| e.path()).collect()).unwrap_or_default();
    v.retain(|p| {
        p.extension()
            .and_then(|e| e.to_str())
            .is_some_and(|e| matches!(e.to_ascii_lowercase().as_str(), "stl" | "obj" | "ply"))
    });
    v.sort();
    v
}

fn wanted(number: usize) -> bool {
    std::env::var("SURFACECRAFT_CLEAN_ONLY")
        .map_or(true, |l| l.split(',').filter_map(|x| x.trim().parse::<usize>().ok()).any(|n| n == number))
}

fn report(number: usize, step: &str, m: &Mesh) {
    let c = edge_census(m);
    let loops = boundary_loops(m);
    let largest = loops.iter().map(Vec::len).max().unwrap_or(0);
    let closed = c.boundary == 0 && c.non_manifold == 0;
    eprintln!(
        "C{number:02} {step:<22} tri {:>8} non-manifold {:>5} boundary edges {:>6} loops {:>4} largest {:>5} closed {closed}",
        m.triangles.len(),
        c.non_manifold,
        c.boundary,
        loops.len(),
        largest,
    );
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_clean_steps() {
    for (k, path) in corpus().iter().enumerate() {
        let number = k + 1;
        if !wanted(number) {
            continue;
        }
        let Ok(bytes) = std::fs::read(path) else { continue };
        let Some(format) = path.extension().and_then(|e| e.to_str()).and_then(MeshFormat::from_extension) else {
            continue;
        };
        let Ok(mut m) = read_mesh(&bytes, format, &Limits::default()) else {
            eprintln!("C{number:02} not a mesh");
            continue;
        };
        drop(bytes);
        report(number, "raw", &m);
        m.remove_degenerate(0.0);
        report(number, "degenerate", &m);
        remove_outliers(&mut m, 1.5);
        report(number, "outliers", &m);
        remove_small_components(&mut m, 0.01, 20);
        report(number, "small components", &m);
        let mut flat = m.clone();
        fill_holes(&mut flat, FILL_EDGES, false);
        compact(&mut flat);
        report(number, "fill flat", &flat);
        drop(flat);
        let rep = fill_holes(&mut m, FILL_EDGES, true);
        compact(&mut m);
        eprintln!(
            "C{number:02} fill report: filled {} skipped {} added {}",
            rep.filled, rep.skipped, rep.added_triangles
        );
        report(number, "fill smooth", &m);
        remove_non_manifold(&mut m);
        report(number, "non-manifold removed", &m);
    }
}
