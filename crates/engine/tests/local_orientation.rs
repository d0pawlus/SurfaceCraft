//! P9-O8: orientation of the bodies `surface.wrap` makes from real scans that stay outside the
//! repository (measurement only, no thresholds). `SURFACECRAFT_LOCAL_CORPUS` names the directory;
//! files are reported by number only (sorted order, from 1); `SURFACECRAFT_LOCAL_CORPUS_ONLY`
//! narrows by file-name text. Without the variable the test returns silently.
//!
//! `SURFACECRAFT_LOCAL_CORPUS=… SURFACECRAFT_ORIENTATION_ONLY=8 cargo test --release -p surfacecraft-engine --test local_orientation -- --ignored --nocapture`
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_fit::patch::{edge_normal_jumps, wrap_patches};
use surfacecraft_geom::edit::{DecimateOptions, remove_non_manifold};
use surfacecraft_geom::reduce::{ToleranceOptions, reduce_to_tolerance};
use surfacecraft_geom::{Bvh, Mesh, Point3};

const TOLERANCE: f64 = 0.2;
/// First budget of the adaptive layout, in tolerances.
const FIRST_BUDGET: f64 = 8.0;
const REACH_BUDGETS: f64 = 6.0;

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

/// The statistic as it was: normals compared as they come, whatever the parametrisation.
fn old_jumps(set: &surfacecraft_fit::patch::PatchSet) -> Vec<f64> {
    use surfacecraft_fit::patch::normal;
    let mut by_edge: std::collections::BTreeMap<(u32, u32), Vec<(usize, usize, bool)>> = Default::default();
    for (fi, c) in set.corners.iter().enumerate() {
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            by_edge.entry((a.min(b), a.max(b))).or_default().push((fi, s, a < b));
        }
    }
    let mut out = Vec::new();
    for uses in by_edge.values() {
        let [(f0, s0, fw0), (f1, s1, fw1)] = uses.as_slice() else { continue };
        let mut worst: f64 = 0.0;
        for k in 1..8 {
            let t = f64::from(k) / 8.0;
            let uv = |s: usize, fw: bool| {
                let t = if fw { t } else { 1.0 - t };
                match s {
                    0 => (t, 0.0),
                    1 => (1.0, t),
                    2 => (1.0 - t, 1.0),
                    _ => (0.0, 1.0 - t),
                }
            };
            let (u0, v0) = uv(*s0, *fw0);
            let (u1, v1) = uv(*s1, *fw1);
            if let (Some(a), Some(b)) = (normal(&set.nets[*f0], u0, v0), normal(&set.nets[*f1], u1, v1)) {
                worst = worst.max(a.dot(&b).clamp(-1.0, 1.0).acos());
            }
        }
        out.push(worst);
    }
    out
}

fn percentile_deg(sorted: &[f64], q: f64) -> f64 {
    sorted.get(((sorted.len().max(1) - 1) as f64 * q) as usize).copied().unwrap_or(0.0).to_degrees()
}

/// Triangles of `coarse` whose normal points away from the normal of the scan triangle nearest to their centroid.
fn disagreeing(coarse: &Mesh, scan: &Bvh) -> usize {
    (0..coarse.triangle_count())
        .filter(|&i| {
            let (Some([a, b, c]), Some(n)) = (coarse.triangle(i), coarse.face_normal(i)) else { return false };
            let centre = Point3::from((a.coords + b.coords + c.coords) / 3.0);
            scan.closest(&centre, f64::INFINITY)
                .and_then(|h| scan.triangle_normal(h.triangle))
                .is_some_and(|m| m.dot(&n) < 0.0)
        })
        .count()
}

fn signed_volume(m: &Mesh) -> f64 {
    (0..m.triangle_count())
        .filter_map(|i| m.triangle(i))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum()
}

fn scan_report(scan: &Mesh) -> String {
    let (mut lo, mut hi) = ([f64::INFINITY; 3], [f64::NEG_INFINITY; 3]);
    for p in &scan.positions {
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let area: f64 = (0..scan.triangle_count())
        .filter_map(|i| scan.triangle(i))
        .map(|[a, b, c]| (b - a).cross(&(c - a)).norm() / 2.0)
        .sum();
    let v = signed_volume(scan);
    format!(
        "extent {:.4} x {:.4} x {:.4}, area {:.4}, signed volume {:.5}, 2V/A {:.5}",
        hi[0] - lo[0],
        hi[1] - lo[1],
        hi[2] - lo[2],
        area,
        v,
        2.0 * v / area
    )
}

/// Dihedral angle (degrees, between the face normals) histogram over the interior edges.
fn dihedral_report(m: &Mesh) -> String {
    let mut edges: std::collections::BTreeMap<(u32, u32), Vec<usize>> = Default::default();
    for (i, t) in m.triangles.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edges.entry((a.min(b), a.max(b))).or_default().push(i);
        }
    }
    let mut bins = [0usize; 6];
    let mut same_dir = 0usize;
    for ((a, b), fs) in &edges {
        let [f0, f1] = fs.as_slice() else { continue };
        let dir = |f: usize| {
            let t = m.triangles[f];
            (0..3).any(|k| t[k] == *a && t[(k + 1) % 3] == *b)
        };
        if dir(*f0) == dir(*f1) {
            same_dir += 1;
        }
        if let (Some(n0), Some(n1)) = (m.face_normal(*f0), m.face_normal(*f1)) {
            let ang = n0.dot(&n1).clamp(-1.0, 1.0).acos().to_degrees();
            bins[((ang / 30.0) as usize).min(5)] += 1;
        }
    }
    format!("edges {} winding-inconsistent {same_dir}, dihedral per 30 deg {bins:?}", edges.len())
}

fn budget_sweep(scan: &Mesh) {
    let bvh = Bvh::build(scan);
    for budget in [1.6, 0.4, 0.1, 0.03, 0.01, 0.003, 0.001] {
        let mut coarse = scan.clone();
        let opts =
            DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
        let _ = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(budget));
        eprintln!(
            "sweep budget {budget}: {} triangles, {} disagree, {}",
            coarse.triangle_count(),
            disagreeing(&coarse, &bvh),
            dihedral_report(&coarse)
        );
    }
}

fn coarse_report(scan: &Mesh) -> String {
    if std::env::var("SURFACECRAFT_ORIENTATION_SWEEP").is_ok() {
        budget_sweep(scan);
    }
    let mut coarse = scan.clone();
    let opts =
        DecimateOptions { target_triangles: 0, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
    let _ = reduce_to_tolerance(&mut coarse, &opts, &ToleranceOptions::new(FIRST_BUDGET * TOLERANCE));
    remove_non_manifold(&mut coarse);
    let bvh = Bvh::build(scan);
    eprintln!("coarse dihedral: {}", dihedral_report(&coarse));
    eprintln!("scan: {}", scan_report(scan));
    let set = wrap_patches(&coarse, &bvh, REACH_BUDGETS * FIRST_BUDGET * TOLERANCE);
    let mut jumps = edge_normal_jumps(&set);
    jumps.sort_by(f64::total_cmp);
    let mut old = old_jumps(&set);
    old.sort_by(f64::total_cmp);
    eprintln!(
        "old statistic: p50 {:.1} p99 {:.1} max {:.1}; new: p50 {:.1} p99 {:.1} max {:.1}",
        percentile_deg(&old, 0.5),
        percentile_deg(&old, 0.99),
        percentile_deg(&old, 1.0),
        percentile_deg(&jumps, 0.5),
        percentile_deg(&jumps, 0.99),
        percentile_deg(&jumps, 1.0)
    );
    let mean = jumps.iter().sum::<f64>() / jumps.len().max(1) as f64;
    format!(
        "coarse {} triangles, {} disagree with the scan normal, volume sign {:+}, patches {}, jump mean {:.1} p50 {:.1} p99 {:.1} max {:.1}",
        coarse.triangle_count(),
        disagreeing(&coarse, &bvh),
        signed_volume(&coarse).signum(),
        set.nets.len(),
        mean.to_degrees(),
        percentile_deg(&jumps, 0.5),
        percentile_deg(&jumps, 0.99),
        percentile_deg(&jumps, 1.0)
    )
}

fn compare(s: &mut Session, w: &Value, facing: bool) -> String {
    match s.execute(
        "body.compare",
        json!({"body": w["body"]["id"], "scan": "scan", "tolerance": TOLERANCE, "facing": facing}),
    ) {
        Ok(c) => {
            format!("count {} within {:.4}", c["totals"]["count"], c["totals"]["within"].as_f64().unwrap_or(f64::NAN))
        }
        Err(e) => format!("error {}", e.to_string().chars().take(80).collect::<String>()),
    }
}

fn body_report(s: &Session, w: &Value, scan: &Mesh) -> String {
    let Some(body) = w["body"]["id"].as_u64().and_then(|id| s.doc.body(surfacecraft_doc::ObjectId(id))) else {
        return "no body".into();
    };
    let volume = match surfacecraft_kernel::solid::volume(&body.brep) {
        Ok(v) => format!("{v:.1}"),
        Err(e) => format!("ERROR {}", e.to_string().chars().take(100).collect::<String>()),
    };
    let tess = match surfacecraft_kernel::tess::tessellate(&body.brep, Some(0.05)) {
        Ok(t) => t,
        Err(e) => return format!("kernel volume {volume}; tessellation error {e}"),
    };
    let bvh = Bvh::build(scan);
    format!(
        "kernel volume {volume}; tessellation volume {:.1}, {} of {} triangles disagree with the scan normal",
        signed_volume(&tess),
        disagreeing(&tess, &bvh),
        tess.triangle_count()
    )
}

fn run_one(number: usize, path: &std::path::Path) {
    let mut s = Session::new();
    if let Err(e) = s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "scan"})) {
        eprintln!("C{number:02} import error {}", e.to_string().chars().take(80).collect::<String>());
        return;
    }
    // SURFACECRAFT_ORIENTATION_SCALE: the scan scaled by this factor (units), under the name "scan"
    if let Some(k) = std::env::var("SURFACECRAFT_ORIENTATION_SCALE").ok().and_then(|v| v.parse::<f64>().ok()) {
        let original = s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).map(|m| (*m.mesh).clone());
        let _ = s.execute("doc.delete", json!({"object": "scan"}));
        if let Some(mut m) = original {
            m.positions.iter_mut().for_each(|p| p.coords *= k);
            s.doc.add_mesh("scan", m);
        }
    }
    let _ = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400}));
    let scan = s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).map(|m| (*m.mesh).clone());
    let Some(scan) = scan else { return };
    let w = match s.execute("surface.wrap", json!({"mesh": "scan", "tolerance": TOLERANCE})) {
        Ok(w) => w,
        Err(e) => {
            eprintln!("C{number:02} wrap error {}", e.to_string().chars().take(100).collect::<String>());
            return;
        }
    };
    eprintln!(
        "C{number:02} wrap: patches {} closed {} within {:.4} reported volume {} jump mean {:.1}",
        w["patches"],
        w["closed"],
        w["deviation"]["within"].as_f64().unwrap_or(f64::NAN),
        w["body"]["volume"],
        w["normal_jump_mean_deg"].as_f64().unwrap_or(f64::NAN)
    );
    eprintln!("C{number:02} compare facing {}; unfiltered {}", compare(&mut s, &w, true), compare(&mut s, &w, false));
    eprintln!("C{number:02} {}", body_report(&s, &w, &scan));
    eprintln!("C{number:02} {}", coarse_report(&scan));
}

#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS (third-party scans, never committed)"]
fn local_corpus_wrap_orientation() {
    let only: Vec<usize> = std::env::var("SURFACECRAFT_ORIENTATION_ONLY")
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_default();
    for (k, path) in corpus().iter().enumerate() {
        if only.is_empty() || only.contains(&(k + 1)) {
            let (number, path) = (k + 1, path.clone());
            let worker = std::thread::Builder::new().stack_size(256 << 20).spawn(move || run_one(number, &path));
            if let Ok(h) = worker {
                let _ = h.join();
            }
        }
    }
}
