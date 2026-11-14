//! P9-M3: `align.global`, several scans of one part into one common pose at once, through the
//! command layer. Synthetic scans only: every view is its own tessellation of the analytic part with
//! independent noise (sigma 0.05), cut to the side it sees and moved by a random rigid motion; the
//! ground truth is that motion.
//!
//! Bounds, fixed before the first run: a pair gate of M2 is 0.1 degrees and 0.1 mm (partial overlap
//! measured 0.083 degrees / 0.073 mm). A view in the common frame is reached through up to five
//! pair poses (the chain along a ring of six views), and relaxation only averages them, so the
//! bound is 5 times the pair gate: 0.5 degrees and 0.5 mm. The measured values and their ratios to
//! the assertion bounds are printed (run with `--nocapture`); the smallest ratio of any assertion
//! is 1.8 (the ring: the relaxed far view must be at least 2 times better than the chain, measured
//! 3.6 times), every other one is 4 or more. Discrete outcomes (which pairs are rejected, whether a
//! pair of non-neighbours is `unique`) are never asserted exactly.
mod global_common;
mod pair_common;
use global_common::*;
use pair_common::*;
use serde_json::json;
use surfacecraft_geom::{Mesh, Point3, Transform, Vec3, synth};

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
    assert!(
        out["scans"].as_array().unwrap().iter().all(|s| s["held_by_loop"] == true),
        "{part}: a view hangs on one pair"
    );
    eprintln!(
        "  {} pairs used, {} rejected, consensus margin {}",
        out["used_edges"], out["rejected_edges"], out["consensus_margin"]
    );
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

/// Views of the ring gate: arcs of 116 degrees every 45. (Six arcs of 150 every 60 do not work: the
/// pairs of non-neighbours that come back `unique` outvote the right ones.)
const RING: usize = 8;

/// Gate 2 through the command: a ring of eight arcs of a lumpy torus, each overlapping its two neighbours
/// only. The chain along the spanning tree and the relaxed poses are both reported; the loop closes
/// and the error around it is smaller after relaxation. (The decisive test that relaxation beats
/// the chain by a clear factor is `crates/align/tests/global_relax.rs`, where the error of every
/// pair pose is injected and known.)
#[test]
fn gate_m3_a_ring_of_views_closes_its_loop() {
    let views: Vec<Mesh> = (0..RING).map(|k| ring_arc(k, 45.0 * k as f64, 58.0)).collect();
    let poses = poses(RING);
    let mut s = session_of(&views, &poses);
    let out = global(&mut s, RING, json!({}));
    let (before, after, cycles) = loop_figures(&out);
    print_edges(&out);
    eprintln!(
        "ring: verdict {}, issues {}, used {} rejected {} margin {} of {} trials, loop {before:.4} -> {after:.4} mm, {cycles} cycles",
        out["verdict"],
        out["issues"],
        out["used_edges"],
        out["rejected_edges"],
        out["consensus_margin"],
        out["consensus_trials"]
    );
    for k in 0..RING {
        let (_, relaxed) = scan_error(&out, &views, &poses, k, "matrix");
        let (_, chained) = scan_error(&out, &views, &poses, k, "chained_matrix");
        eprintln!("  view {k}: relaxed {relaxed:.4} mm, chained {chained:.4} mm");
    }
    // the far view is the one the chain carries furthest off the truth
    let chained: Vec<f64> = (0..RING).map(|k| scan_error(&out, &views, &poses, k, "chained_matrix").1).collect();
    let far = (0..RING).max_by(|a, b| chained[*a].total_cmp(&chained[*b])).unwrap();
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

/// Two whole copies of a part are one pair: a unique pose, but no loop confirms it. The verdict is
/// `partial` with `weakly_held` and the note quotes the pair's own evidence. `apply` alone moves
/// nothing and says why; with the scan listed in `accept_weak` it moves, in one undo step.
#[test]
fn gate_m3_two_scans_are_one_unconfirmed_pair_and_move_only_when_accepted() {
    let (fixed, moving) = pair_of("bracket");
    let pose = random_motion(4);
    let mut s = session(&fixed, &moving, &pose);
    let before = s.history.undo_labels().len();
    let out = s.execute("align.global", json!({"objects": ["fixed", "moving"], "apply": true})).unwrap();
    eprintln!("{} {}\n{}", out["verdict"], out["issues"], out["note"]);
    assert_eq!(out["verdict"], "partial");
    assert_eq!(out["issues"], json!(["weakly_held"]));
    assert_eq!(out["scans"][1]["held_by_loop"], false);
    assert_eq!(out["scans"][1]["weak_pair"], json!([0, 1]));
    assert_eq!((out["applied"].clone(), out["moved"].clone()), (json!(false), json!([])));
    assert_eq!(out["left_weakly_held"], json!([1]));
    assert_eq!(s.history.undo_labels().len(), before, "nothing moved, no undo step");
    let note = out["note"].as_str().unwrap();
    for part in
        ["nothing was moved", "accept_weak", "no loop of strong pairs confirms", "confidence", "of the diagonal"]
    {
        assert!(note.contains(part), "{part}: {note}");
    }
    let out = s
        .execute("align.global", json!({"objects": ["fixed", "moving"], "apply": true, "accept_weak": ["moving"]}))
        .unwrap();
    assert_eq!((out["applied"].clone(), out["moved"].clone()), (json!(true), json!([1])));
    assert_eq!(s.history.undo_labels().len(), before + 1, "one undo step");
    let err = s.execute("align.global", json!({"objects": ["fixed", "moving"], "accept_weak": ["nowhere"]}));
    assert!(err.is_err());
}

/// `start: current`: three whole scans placed by hand within about 2 degrees and 3 mm of each
/// other are brought together; the pairs start from the current poses.
#[test]
fn gate_m3_scans_placed_by_hand_are_refined_from_where_they_are() {
    let views: Vec<Mesh> = (0..3).map(|k| noisy(bracket_at(10 + 2 * k, 40 + 6 * k, 14 + k), 700 + k as u64)).collect();
    let axes = [Vec3::new(0.3, 0.8, 0.5), Vec3::new(-0.6, 0.1, 0.9)];
    let poses = vec![
        Transform::identity(),
        Transform::rotation_axis_angle(&axes[0], 2.0_f64.to_radians())
            .then_after(&Transform::translation(Vec3::new(2.0, -3.0, 1.0))),
        Transform::rotation_axis_angle(&axes[1], -1.5_f64.to_radians())
            .then_after(&Transform::translation(Vec3::new(-2.5, 1.0, 2.0))),
    ];
    let mut s = session_of(&views, &poses);
    let out = global(&mut s, 3, json!({"start": "current"}));
    print_edges(&out);
    eprintln!(
        "verdict {}, issues {}, methods {:?}",
        out["verdict"],
        out["issues"],
        out["edges"].as_array().unwrap().iter().map(|e| e["method"].clone()).collect::<Vec<_>>()
    );
    for k in 0..3 {
        let (angle, disp) = scan_error(&out, &views, &poses, k, "matrix");
        eprintln!("  view {k}: {angle:.4} deg {disp:.4} mm");
        assert!(angle <= BOUND_DEGREES && disp <= BOUND_MM, "view {k}");
    }
    assert_eq!(out["verdict"], "aligned", "{out}");
}

/// A part with a twofold symmetry (a box, which a half turn maps onto itself) and one small
/// off-centre bump: views of it fit the flipped way too, except for the bump. Whatever the verdict,
/// it is never `aligned` at a flipped pose: a result that claims `aligned` is within the bound.
#[test]
fn gate_m3_a_symmetric_part_is_never_aligned_at_the_flipped_pose() {
    let part = |k: usize| {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 8 + 2 * k);
        m.append(&synth::sphere(Point3::new(28.0, 10.0, 20.0), 3.0, 20 + 2 * k, 10 + k));
        noisy(m, 800 + k as u64)
    };
    let centre = Point3::new(20.0, 15.0, 10.0);
    let sides: [([f64; 3], f64); 4] =
        [([1.0, 0.0, 0.0], 5.0), ([-1.0, 0.0, 0.0], 5.0), ([0.0, 1.0, 0.0], 4.0), ([0.0, 0.0, 1.0], 3.0)];
    let views: Vec<Mesh> = sides
        .iter()
        .enumerate()
        .map(|(k, (d, r))| cut(&part(k), |p| (p - centre).dot(&Vec3::new(d[0], d[1], d[2])) >= -r))
        .collect();
    let poses = poses(4);
    let mut s = session_of(&views, &poses);
    let out = global(&mut s, 4, json!({}));
    print_edges(&out);
    eprintln!(
        "verdict {}, issues {}, used {}, rejected {}, margin {}",
        out["verdict"], out["issues"], out["used_edges"], out["rejected_edges"], out["consensus_margin"]
    );
    let errors: Vec<(f64, f64)> = (0..4)
        .filter(|k| out["scans"][*k]["placed"] == true)
        .map(|k| scan_error(&out, &views, &poses, k, "matrix"))
        .collect();
    eprintln!("errors of the placed views: {errors:?}");
    if out["verdict"] == "aligned" {
        assert!(
            errors.iter().all(|(a, d)| *a <= BOUND_DEGREES && *d <= BOUND_MM),
            "aligned at a wrong pose: {errors:?}"
        );
    }
}
