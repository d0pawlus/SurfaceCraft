//! P9-M4, local only (needs `SURFACECRAFT_LOCAL_CORPUS`, third-party scans that stay outside the
//! repository and are named by number only; everything happens in memory, nothing is written):
//! `SURFACECRAFT_LOCAL_CORPUS=… cargo test --profile ci -p surfacecraft-engine --test p9_merge_local -- --ignored --nocapture --test-threads 1`
//!
//! Every scan is cut into three overlapping pieces along its longest axis (each about 45 % of the
//! extent, neighbours sharing about 10 %) and merged back at their true poses with `mesh.merge`,
//! twice: `exact` (the pieces share their vertices and triangles where they overlap, as cut) and
//! `jitter` (every piece gets its own normal noise of a fifth of the scan's noise estimate, so no
//! vertex of one piece coincides with one of another and the seam must be built). The merge is
//! compared with the whole scan: the share of the scan's vertices within the tolerance of the merge,
//! the share found bit for bit (exact only), boundary loops and pieces against the scan's, open seam,
//! issues and time. `SURFACECRAFT_LOCAL_SCAN=n` runs one scan only. Nothing is asserted beyond "no
//! panic, an answer": the numbers are the result.
mod pair_common;
use pair_common::cut;
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Rng, synth};

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

fn load(path: &std::path::Path) -> Option<Mesh> {
    let mut s = Session::new();
    s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})).ok()?;
    let (id, _) = s.doc.find_by_name("scan")?;
    Some(s.doc.mesh(id)?.mesh.as_ref().clone())
}

/// The pieces of `m` whose centroids lie between the fractions of its longest extent.
fn pieces(m: &Mesh) -> Vec<Mesh> {
    let b = m.bbox();
    let ext = [b.max.x - b.min.x, b.max.y - b.min.y, b.max.z - b.min.z];
    let axis = (0..3).max_by(|&i, &j| ext[i].total_cmp(&ext[j])).unwrap_or(0);
    let (lo, len) = (b.min[axis], ext[axis]);
    [(0.0, 0.45), (0.35, 0.8), (0.7, 1.0)]
        .iter()
        .map(|&(from, to)| cut(m, |p| (lo + from * len..=lo + to * len).contains(&p[axis])))
        .collect()
}

/// Share of (at most 200 000, evenly taken) vertices of `whole` within `tol` of `merged`, and the
/// share found at exactly their positions.
fn compare(whole: &Mesh, merged: &Mesh, tol: f64) -> (f64, f64) {
    let bvh = Bvh::build(merged);
    let mut at = std::collections::HashSet::new();
    for p in &merged.positions {
        at.insert([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
    }
    let n = whole.positions.len();
    let step = n.div_ceil(200_000).max(1);
    let picked: Vec<_> = whole.positions.iter().step_by(step).collect();
    let near = picked.iter().filter(|p| bvh.closest(p, tol).is_some()).count();
    let same = picked.iter().filter(|p| at.contains(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])).count();
    (near as f64 / picked.len() as f64, same as f64 / picked.len() as f64)
}

/// Vertices of `m` (used by a triangle) with another vertex closer than `r` and no edge between them:
/// the near-coincident points the scan itself has (the baseline of `unwelded_vertices`).
fn near_pairs(m: &Mesh, r: f64) -> usize {
    let mut edges = std::collections::HashSet::new();
    let mut used = vec![false; m.positions.len()];
    for t in &m.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges.insert((a.min(b), a.max(b)));
            used[a as usize] = true;
        }
    }
    let ids: Vec<u32> = (0..m.positions.len() as u32).filter(|&v| used[v as usize]).collect();
    let pts: Vec<_> = ids.iter().map(|&v| m.positions[v as usize]).collect();
    let tree = surfacecraft_geom::KdTree::build(&pts);
    ids.iter()
        .enumerate()
        .filter(|&(i, &v)| {
            tree.within(&pts[i], r)
                .into_iter()
                .map(|j| ids[j])
                .any(|w| w != v && !edges.contains(&(v.min(w), v.max(w))))
        })
        .count()
}

/// The scan's own self-crossings (triangles crossing another they share no vertex with) and the share
/// of its area whose centre lies within `tol` of facing surface it shares no vertex with (its own
/// double layers): the baselines of `added_crossings` and `stacked_share`.
fn own_faults(m: &Mesh, tol: f64) -> (usize, f64) {
    let bvh = Bvh::build(m);
    let crossing = surfacecraft_geom::nesting::sampled_self_intersections(m, &bvh, m.triangle_count()).crossing;
    let (mut total, mut stacked) = (0.0, 0.0);
    for i in 0..m.triangle_count() {
        let (Some(c), Some(n)) = (m.face_centroid(i), m.face_normal(i)) else { continue };
        let t = m.triangles[i];
        let other = |o: usize| {
            !m.triangles[o].iter().any(|v| t.contains(v)) && m.face_normal(o).is_some_and(|k| k.dot(&n) >= 0.5)
        };
        total += m.face_area(i);
        if bvh.closest_filtered(&c, tol, &other).is_some() {
            stacked += m.face_area(i);
        }
    }
    (crossing, if total > 0.0 { stacked / total } else { 0.0 })
}

fn codes(out: &Value) -> Vec<String> {
    out["issues"]
        .as_array()
        .map_or_else(Vec::new, |v| v.iter().map(|i| i["code"].as_str().unwrap_or("").to_string()).collect())
}

fn run(number: usize, kind: &str, whole: &Mesh, parts: &[Mesh]) {
    let mut s = Session::new();
    for (k, p) in parts.iter().enumerate() {
        s.doc.add_mesh(&format!("p{k}"), p.clone());
    }
    let t0 = std::time::Instant::now();
    let out = s.execute("mesh.merge", json!({"objects": ["p0", "p1", "p2"]}));
    let secs = t0.elapsed().as_secs_f64();
    let out = match out {
        Ok(v) => v,
        Err(e) => {
            assert!(kind != "exact", "scan {number}: pieces of one scan must merge: {e}");
            eprintln!("| {number} | {kind} | error: {e} | | | | | | {secs:.1} |");
            return;
        }
    };
    if kind == "exact" {
        let pairs: Vec<String> = out["pairs"]
            .as_array()
            .into_iter()
            .flatten()
            .map(|p| {
                format!(
                    "{}-{} samples {} shared {} within {} over {}",
                    p["a"], p["b"], p["samples"], p["shared"], p["within"], p["over"]
                )
            })
            .collect();
        assert_eq!(out["refused"], json!(false), "scan {number}: {:?} {pairs:?}", codes(&out));
    }
    let tol = out["tolerance"].as_f64().unwrap_or(0.0);
    let topo = surfacecraft_geom::merge::topology(whole);
    let own = own_faults(whole, tol);
    let (near, same) = match out["id"].as_u64() {
        Some(id) => compare(whole, s.doc.mesh(surfacecraft_doc::ObjectId(id)).unwrap().mesh.as_ref(), tol),
        None => (0.0, 0.0),
    };
    if kind == "exact" {
        // measured 0.9996 to 1.0000 of the scan's vertices within the tolerance of the merge
        assert!(near >= 0.66, "scan {number}: {near}");
    }
    eprintln!(
        "| {number} | {kind} | {} | {tol:.4} | {near:.4} | {same:.4} | {}/{} | {}/{} | {} ({:.1} mm) | {:.4} | {} ({}) | {} ({}) | {:.4} ({:.4}) | {secs:.1} | {:?} |",
        whole.triangle_count(),
        out["after"]["pieces"],
        topo.pieces,
        out["after"]["loops"],
        topo.loops,
        out["seam"]["open_edges"],
        out["seam"]["open_length"].as_f64().unwrap_or(0.0),
        out["checks"]["removed_within_share"].as_f64().unwrap_or(0.0),
        out["checks"]["unwelded_vertices"],
        near_pairs(whole, 0.01 * surfacecraft_geom::quality::edge_census(whole).lengths.median),
        out["checks"]["added_crossings"],
        own.0,
        out["checks"]["stacked_share"].as_f64().unwrap_or(0.0),
        own.1,
        codes(&out)
    );
    let stages: Vec<String> = out["stages"].as_array().map_or_else(Vec::new, |v| {
        v.iter().map(|st| format!("{} {} ms", st["stage"].as_str().unwrap_or(""), st["ms"])).collect()
    });
    eprintln!("|   | stages: {} |", stages.join(", "));
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS"]
fn local_corpus_pieces_merge_back() {
    let only: Option<usize> = std::env::var("SURFACECRAFT_LOCAL_SCAN").ok().and_then(|v| v.parse().ok());
    eprintln!(
        "| scan | kind | triangles | tolerance | within tol | bit for bit | pieces merge/scan | loops merge/scan | open seam | removed within | unwelded (scan's own) | added crossings (scan's own) | stacked (scan's own) | s | issues |"
    );
    eprintln!("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|");
    for (i, path) in corpus().iter().enumerate() {
        let number = i + 1;
        if only.is_some_and(|n| n != number) {
            continue;
        }
        let Some(whole) = load(path) else {
            eprintln!("| {number} | load failed |");
            continue;
        };
        let parts = pieces(&whole);
        let census = surfacecraft_geom::quality::edge_census(&whole);
        eprintln!(
            "|   | scan {number}: diagonal {:.1} mm, median edge {:.3}, longest edge {:.3} |",
            whole.bbox().diagonal(),
            census.lengths.median,
            census.lengths.max
        );
        run(number, "exact", &whole, &parts);
        let sigma = surfacecraft_deviation::noise_estimate(&whole).sigma;
        let jittered: Vec<Mesh> = parts
            .iter()
            .enumerate()
            .map(|(k, p)| {
                let mut q = p.clone();
                synth::add_noise(&mut q, 0.2 * sigma, &mut Rng::new(70 + k as u64));
                q
            })
            .collect();
        run(number, "jitter", &whole, &jittered);
    }
}

/// Two views of a fine blob, about 0.6 million triangles each: the time of a large merge (printed,
/// not asserted).
#[test]
#[ignore = "timing run"]
fn large_merge_timing() {
    let views: Vec<Mesh> = [(1.0, 0u64), (-1.0, 1)]
        .iter()
        .map(|&(side, k)| {
            let mut m = pair_common::blob_at(1100 + 20 * k as usize);
            synth::add_noise(&mut m, 0.02, &mut Rng::new(300 + k));
            cut(&m, |p| side * p.x >= -8.0)
        })
        .collect();
    let whole = {
        let mut a = views[0].clone();
        a.append(&views[1]);
        a
    };
    run(0, "large", &whole, &[views[0].clone(), views[1].clone(), views[1].clone()]);
}
