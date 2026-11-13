//! P9-M3: `align.global`, several scans of one part into one common pose at once, through the
//! command layer. Synthetic scans only: every view is its own tessellation of the analytic part with
//! independent noise (sigma 0.05), cut to the side it sees and moved by a random rigid motion; the
//! ground truth is that motion.
//!
//! Bounds, fixed before the first run: a pair gate of M2 is 0.1 degrees and 0.1 mm (partial overlap
//! measured 0.083 degrees / 0.073 mm). A view in the common frame is reached through up to five
//! pair poses (the chain along a ring of six views), and relaxation only averages them, so the
//! bound is 5 times the pair gate: 0.5 degrees and 0.5 mm. No assertion sits within 1.5 times of a
//! measured value; the measured values and their ratios are printed (run with `--nocapture`).
mod pair_common;
use pair_common::*;
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Transform, Vec3, synth};

const BOUND_DEGREES: f64 = 0.5;
const BOUND_MM: f64 = 0.5;

/// A view of the bracket: its own tessellation and noise, the triangles whose centroid lies on the
/// side of `dir` of the part centre or at most `reach` mm beyond it.
fn bracket_view(k: usize, dir: Vec3, reach: f64) -> Mesh {
    let whole = noisy(bracket_at(10 + 2 * k, 40 + 6 * k, 14 + k), 100 + k as u64);
    let c = Point3::new(30.0, 20.0, 15.0);
    cut(&whole, |p| (p - c).dot(&dir) >= -reach)
}

const SIDES: [([f64; 3], f64); 6] = [
    ([1.0, 0.0, 0.0], 6.0),
    ([-1.0, 0.0, 0.0], 6.0),
    ([0.0, 1.0, 0.0], 4.0),
    ([0.0, -1.0, 0.0], 4.0),
    ([0.0, 0.0, 1.0], 6.0),
    ([0.577_35, 0.577_35, 0.577_35], 7.0),
];

fn bracket_views(n: usize) -> Vec<Mesh> {
    SIDES.iter().take(n).enumerate().map(|(k, (d, r))| bracket_view(k, Vec3::new(d[0], d[1], d[2]), *r)).collect()
}

/// Six views of the blob, as the bracket's: the triangles on the side of a direction of the centre
/// or at most `reach` mm beyond it.
fn blob_views() -> Vec<Mesh> {
    let sides: [([f64; 3], f64); 6] = [
        ([1.0, 0.0, 0.0], 8.0),
        ([-1.0, 0.0, 0.0], 8.0),
        ([0.0, 1.0, 0.0], 5.0),
        ([0.0, -1.0, 0.0], 5.0),
        ([0.0, 0.0, 1.0], 3.5),
        ([0.0, 0.0, -1.0], 3.5),
    ];
    sides
        .iter()
        .enumerate()
        .map(|(k, (d, reach))| {
            let dir = Vec3::new(d[0], d[1], d[2]);
            cut(&noisy(blob_at(48 + 4 * k), 200 + k as u64), |p| p.coords.dot(&dir) >= -reach)
        })
        .collect()
}

/// A part with the rotational symmetry broken by twelve bumps of different sizes at irregular
/// azimuths: a ring (torus) with lumps, seen in arcs.
fn lumpy_ring(k: usize) -> Mesh {
    let mut m = synth::torus(Point3::origin(), Vec3::z(), 40.0, 9.0, 72 + 4 * k, 20 + k);
    let azimuths = [8.0, 31.0, 62.0, 80.0, 118.0, 139.0, 171.0, 203.0, 236.0, 262.0, 291.0, 330.0];
    for (i, a) in azimuths.iter().enumerate() {
        let (s, c) = f64::to_radians(*a).sin_cos();
        let radius = 3.5 + 0.9 * ((i * 7) % 5) as f64;
        let z = 1.8 * (((i * 5) % 7) as f64 - 3.0);
        let out = 44.0 + 0.7 * ((i * 3) % 4) as f64;
        m.append(&synth::sphere(Point3::new(out * c, out * s, z), radius, 20, 10));
    }
    noisy(m, 600 + k as u64)
}

/// A view of the lumpy ring: the arc of `half` degrees either side of azimuth `centre`.
fn ring_arc(k: usize, centre: f64, half: f64) -> Mesh {
    cut(&lumpy_ring(k), |p| {
        let d = (p.y.atan2(p.x).to_degrees() - centre + 540.0).rem_euclid(360.0) - 180.0;
        d.abs() <= half
    })
}

fn poses(n: usize) -> Vec<Transform> {
    (0..n as u64).map(random_motion).collect()
}

fn session_of(views: &[Mesh], poses: &[Transform]) -> Session {
    session_with_ids(views, poses).0
}

fn session_with_ids(views: &[Mesh], poses: &[Transform]) -> (Session, Vec<surfacecraft_doc::ObjectId>) {
    let mut s = Session::new();
    let ids = views
        .iter()
        .zip(poses)
        .enumerate()
        .map(|(k, (v, p))| s.doc.add_mesh(&format!("v{k}"), v.transformed(p)))
        .collect();
    (s, ids)
}

fn global(s: &mut Session, n: usize, extra: Value) -> Value {
    let names: Vec<String> = (0..n).map(|k| format!("v{k}")).collect();
    let mut p = json!({ "objects": names });
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("align.global", p).unwrap()
}

fn rows(v: &Value) -> Transform {
    Transform::from_rows(&serde_json::from_value(v.clone()).unwrap()).unwrap()
}

/// (angle in degrees, largest displacement in mm) of the motion found for view `k` against the
/// truth: the motion that carries it back into the frame of view 0.
fn error_of(found: &Transform, views: &[Mesh], poses: &[Transform], k: usize) -> (f64, f64) {
    let truth = poses[0].then_after(&poses[k].inverse());
    let on = views[k].transformed(&poses[k]);
    let q = truth.inverse().then_after(found).quaternion();
    let angle = 2.0 * q.vector().norm().atan2(q.scalar().abs());
    (angle.to_degrees(), found.max_displacement(&truth, &on.positions))
}

fn scan_error(out: &Value, views: &[Mesh], poses: &[Transform], k: usize, key: &str) -> (f64, f64) {
    error_of(&rows(&out["scans"][k][key]), views, poses, k)
}

fn count(out: &Value, status: &str) -> usize {
    out["edges"].as_array().unwrap().iter().filter(|e| e["status"] == status).count()
}

fn print_edges(out: &Value) {
    for e in out["edges"].as_array().unwrap() {
        eprintln!(
            "  {}-{}: {} {} confidence {:.2}, overlap {:.2}/{:.2}",
            e["fixed"],
            e["moving"],
            e["verdict"],
            e["status"],
            e["confidence"].as_f64().unwrap(),
            e["overlap_moving"].as_f64().unwrap(),
            e["overlap_fixed"].as_f64().unwrap()
        );
    }
}

fn loop_figures(out: &Value) -> (f64, f64, u64) {
    let l = &out["loop"];
    (l["before"]["worst_mm"].as_f64().unwrap(), l["after"]["worst_mm"].as_f64().unwrap(), l["cycles"].as_u64().unwrap())
}

/// Gate 1: six overlapping views of a part, each re-meshed and in a pose of its own: every view
/// within the bound of the truth relative to the fixed view, verdict `aligned`, and the loop
/// closure error smaller after relaxation than before.
fn gate_one(part: &str, views: Vec<Mesh>) {
    let n = views.len();
    let poses = poses(n);
    let mut s = session_of(&views, &poses);
    let started = std::time::Instant::now();
    let out = global(&mut s, n, json!({}));
    let (before, after, cycles) = loop_figures(&out);
    eprintln!(
        "{part}: {:?}, verdict {}, issues {}, {} of {} pairs used, loop worst {before:.4} -> {after:.4} mm over {cycles} cycles (ratio {:.2}), worst edge ratio {:.3}",
        started.elapsed(),
        out["verdict"],
        out["issues"],
        count(&out, "used"),
        out["pairs_tried"],
        before / after.max(1e-12),
        out["loop"]["worst_edge_ratio"].as_f64().unwrap()
    );
    print_edges(&out);
    for k in 0..n {
        let (angle, disp) = scan_error(&out, &views, &poses, k, "matrix");
        let (cangle, cdisp) = scan_error(&out, &views, &poses, k, "chained_matrix");
        eprintln!(
            "  view {k}: relaxed {angle:.4} deg {disp:.4} mm (bound ratio {:.1}), chained {cangle:.4} deg {cdisp:.4} mm",
            BOUND_MM / disp.max(1e-12)
        );
        assert!(angle <= BOUND_DEGREES && disp <= BOUND_MM, "{part} view {k}: {angle} deg {disp} mm");
    }
    assert_eq!(out["verdict"], "aligned", "{part}: {out}");
    assert!(cycles >= 1, "{part}: the pose graph has no cycle");
    assert!(after < before, "{part}: loop closure {before} -> {after}");
}

#[test]
fn gate_m3_six_bracket_views_are_aligned_within_the_bound() {
    gate_one("bracket", bracket_views(6));
}

#[test]
fn gate_m3_six_blob_views_are_aligned_within_the_bound() {
    gate_one("blob", blob_views());
}

/// Gate 2 through the command: a ring of eight arcs of a lumpy torus, each overlapping its two neighbours
/// only. The chain along the spanning tree and the relaxed poses are both reported; the loop closes
/// and the error around it is smaller after relaxation. (The decisive test that relaxation beats
/// the chain by a clear factor is `crates/align/tests/global_relax.rs`, where the error of every
/// pair pose is injected and known.)
#[test]
fn gate_m3_a_ring_of_views_closes_its_loop() {
    let views: Vec<Mesh> = (0..8).map(|k| ring_arc(k, 45.0 * k as f64, 58.0)).collect();
    let poses = poses(8);
    let mut s = session_of(&views, &poses);
    let out = global(&mut s, 8, json!({}));
    let (before, after, cycles) = loop_figures(&out);
    print_edges(&out);
    eprintln!("ring: verdict {}, loop {before:.4} -> {after:.4} mm, {cycles} cycles", out["verdict"]);
    for k in 0..8 {
        let (_, relaxed) = scan_error(&out, &views, &poses, k, "matrix");
        let (_, chained) = scan_error(&out, &views, &poses, k, "chained_matrix");
        eprintln!("  view {k}: relaxed {relaxed:.4} mm, chained {chained:.4} mm");
    }
    // the far view is the one the chain carries furthest off the truth
    let chained: Vec<f64> = (0..8).map(|k| scan_error(&out, &views, &poses, k, "chained_matrix").1).collect();
    let far = (0..8).max_by(|a, b| chained[*a].total_cmp(&chained[*b])).unwrap();
    let relaxed = scan_error(&out, &views, &poses, far, "matrix").1;
    eprintln!(
        "far view {far}: chained {:.4} mm, relaxed {relaxed:.4} mm, factor {:.1}; loop factor {:.1}",
        chained[far],
        chained[far] / relaxed,
        before / after.max(1e-12)
    );
    assert!(cycles >= 1);
    assert!(after * 3.0 <= before, "loop closure {before} -> {after}");
    assert!(relaxed * 2.0 <= chained[far], "far view {far}: chained {} relaxed {relaxed}", chained[far]);
}

/// Gate 3: a view of a different part among the views stays unplaced and is reported; the rest is
/// `partial`; with `apply` nothing of it moves and one undo restores everything.
#[test]
fn gate_m3_a_view_of_another_part_is_left_unplaced() {
    let mut views = bracket_views(5);
    views.push(noisy(blob_at(52), 300));
    let poses = poses(6);
    let (mut s, ids) = session_with_ids(&views, &poses);
    let blob_before = s.doc.mesh(ids[5]).unwrap().mesh.positions.clone();
    let out = global(&mut s, 6, json!({"apply": true}));
    print_edges(&out);
    eprintln!("verdict {}, issues {}", out["verdict"], out["issues"]);
    assert_eq!(out["verdict"], "partial", "{out}");
    let scan = &out["scans"][5];
    assert_eq!(scan["placed"], false);
    assert_eq!(scan["unplaced"], "no_unique_pair");
    assert_eq!(scan["applied"], false);
    assert!(out["issues"].as_array().unwrap().iter().any(|i| i == "unplaced_scans"));
    assert_eq!(s.doc.mesh(ids[5]).unwrap().mesh.positions, blob_before, "the unplaced scan moved");
    for k in 0..5 {
        let (angle, disp) = scan_error(&out, &views[..5], &poses[..5], k, "matrix");
        eprintln!("  view {k}: {angle:.4} deg {disp:.4} mm");
        assert!(angle <= BOUND_DEGREES && disp <= BOUND_MM, "view {k}");
    }
    assert_eq!(s.history.undo_labels().iter().filter(|l| **l == "align.global").count(), 1, "one undo step");
}

/// Gate 4: two groups of views with no overlap between the groups: the group without the fixed
/// scan is unplaced, the verdict is `partial`.
#[test]
fn gate_m3_two_groups_without_overlap_leave_one_unplaced() {
    let views =
        vec![ring_arc(0, 0.0, 58.0), ring_arc(1, 40.0, 58.0), ring_arc(2, 180.0, 58.0), ring_arc(3, 220.0, 58.0)];
    let poses = poses(4);
    let mut s = session_of(&views, &poses);
    let out = global(&mut s, 4, json!({}));
    print_edges(&out);
    eprintln!("verdict {}, issues {}", out["verdict"], out["issues"]);
    assert_eq!(out["verdict"], "partial", "{out}");
    for k in [2, 3] {
        assert_eq!(out["scans"][k]["placed"], false, "view {k}");
        assert!(out["scans"][k]["unplaced"].is_string());
    }
    assert_eq!(out["scans"][1]["placed"], true);
    let (angle, disp) = scan_error(&out, &views, &poses, 1, "matrix");
    eprintln!("  view 1: {angle:.4} deg {disp:.4} mm");
    assert!(angle <= BOUND_DEGREES && disp <= BOUND_MM);
}

/// Gate 5: copies of a box fit in several ways: every pair is ambiguous, none makes an edge, and
/// the verdict is not `aligned`.
#[test]
fn gate_m3_symmetric_views_make_no_edge_and_are_not_aligned() {
    let views: Vec<Mesh> = (0..3u64)
        .map(|k| noisy(synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 8 + 3 * k as usize), 500 + k))
        .collect();
    let poses = poses(3);
    let mut s = session_of(&views, &poses);
    let out = global(&mut s, 3, json!({}));
    print_edges(&out);
    assert_ne!(out["verdict"], "aligned");
    assert_eq!(count(&out, "used"), 0, "{out}");
    assert_eq!(count(&out, "ambiguous"), 3, "{out}");
}
