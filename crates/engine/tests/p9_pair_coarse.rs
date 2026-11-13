//! P9-M2: `align.pair_coarse` through the command layer: two scans of one part into one pose with
//! no picked points. Synthetic scans only; the ground truth is the random motion applied to the
//! moving copy, which is a different tessellation of the same analytic part with independent noise
//! (no vertex of one coincides with a vertex of the other).
//!
//! Tolerances, fixed before the first run: pose error at most 0.1 degrees and 0.1 mm (largest
//! displacement of any vertex of the moving scan against the true motion), the class of the noisy
//! ICP gate of M1 (0.02 mm at sigma 0.02) scaled to sigma 0.05. Only the small-patch gate has a
//! looser, derived bound (a patch of a tenth of the part constrains the pose less; see there).
mod pair_common;
use pair_common::*;
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Transform, Vec3, synth};

/// Checks trial `seed` and returns (angle in degrees, displacement in mm).
fn assert_recovers(part: &str, seed: u64) -> (f64, f64) {
    let (fixed, moving) = pair_of_trial(part, seed);
    let pose = random_motion(seed);
    let mut s = session(&fixed, &moving, &pose);
    let out = pair(&mut s, json!({}));
    let (angle, disp) = pose_error(&matrix_of(&out), &pose, &moving);
    eprintln!(
        "{part} seed {seed}: stability {:.5}, angle {angle:.4} deg, displacement {disp:.4} mm, verdict {}, confidence {:.2}, overlap {:.3}/{:.3}, method {}, tolerance {:.3}, {} candidates, icp {} iterations",
        stab(&out),
        out["verdict"],
        out["confidence"].as_f64().unwrap(),
        out["overlap_moving"].as_f64().unwrap(),
        out["overlap_fixed"].as_f64().unwrap(),
        out["method"],
        out["tolerance"].as_f64().unwrap(),
        out["candidates"],
        out["icp_iterations"],
    );
    assert!(angle <= MAX_ANGLE_DEG, "{part} seed {seed}: angle error {angle} deg, {out}");
    assert!(disp <= MAX_DISPLACEMENT_MM, "{part} seed {seed}: displacement {disp} mm, {out}");
    assert_eq!(out["verdict"], "unique", "{part} seed {seed}: {out}");
    assert_eq!(out["confident"], true);
    (angle, disp)
}

/// Runs the ten trials (each its own motion and its own noise) and prints the worst case.
fn ten_trials(part: &str) {
    let worst =
        (0..10).map(|seed| assert_recovers(part, seed)).fold((0.0_f64, 0.0_f64), |w, e| (w.0.max(e.0), w.1.max(e.1)));
    eprintln!("{part}: worst of ten trials: angle {:.4} deg, displacement {:.4} mm", worst.0, worst.1);
}

/// Gate 1: ten random motions, rotations up to 180 degrees, bracket then blob, every one a unique
/// and correct pose.
#[test]
fn gate_m2_pair_recovers_ten_random_motions_on_the_bracket() {
    ten_trials("bracket");
}

#[test]
fn gate_m2_pair_recovers_ten_random_motions_on_the_blob() {
    ten_trials("blob");
}

/// Two partial scans of a part, each from its own tessellation: (fixed, moving before its motion).
fn partial_pair(part: &str, trial: u64) -> (Mesh, Mesh) {
    let (full_f, full_m) = pair_of_trial(part, trial);
    if part == "bracket" {
        (cut(&full_f, |c| c.z > 1.0 && c.x <= 54.0), cut(&full_m, |c| c.z > 1.0 && c.x >= 6.0))
    } else {
        (cut(&full_f, |c| c.x <= 22.0 && c.z > -8.0), cut(&full_m, |c| c.x >= -22.0 && c.z > -8.0))
    }
}

fn assert_partial(part: &str, seed: u64) {
    let (fixed, moving) = partial_pair(part, seed);
    let whole = if part == "bracket" { synth::bracket() } else { blob_at(96) };
    let (cover_f, cover_m) = (fixed.area() / whole.area(), moving.area() / whole.area());
    let truth_m = true_share(&moving, &fixed, 0.3);
    let truth_f = true_share(&fixed, &moving, 0.3);
    let shared = truth_m * moving.area() / whole.area();
    eprintln!(
        "{part}: fixed covers {cover_f:.2}, moving {cover_m:.2}, shared {shared:.2}; true overlap {truth_m:.3}/{truth_f:.3}"
    );
    assert!((0.55..=0.75).contains(&cover_f) && (0.55..=0.75).contains(&cover_m), "construction: {cover_f} {cover_m}");
    assert!((0.4..=0.6).contains(&shared), "construction: shared {shared}");
    let pose = random_motion(seed);
    let mut s = session(&fixed, &moving, &pose);
    let out = pair(&mut s, json!({}));
    let (angle, disp) = pose_error(&matrix_of(&out), &pose, &moving);
    let (om, of) = (out["overlap_moving"].as_f64().unwrap(), out["overlap_fixed"].as_f64().unwrap());
    eprintln!(
        "STABILITY {part} partial seed {seed}: {:.5}, angle {angle:.4} deg, displacement {disp:.4} mm, verdict {}, overlap {om:.3}/{of:.3}, method {}",
        stab(&out),
        out["verdict"],
        out["method"]
    );
    assert!(angle <= MAX_ANGLE_DEG && disp <= MAX_DISPLACEMENT_MM, "{part} seed {seed}: {angle} deg {disp} mm {out}");
    assert!((om - truth_m).abs() <= 0.10, "overlap_moving {om} vs true {truth_m}");
    assert!((of - truth_f).abs() <= 0.10, "overlap_fixed {of} vs true {truth_f}");
    assert_eq!(out["verdict"], "unique", "{out}");
}

/// Gate 2: two scans of about 65 % of the part with about half of it shared; no principal frame
/// agrees, so the descriptors have to find the pose.
#[test]
fn gate_m2_partial_overlap_bracket() {
    (0..5).for_each(|seed| assert_partial("bracket", seed));
}

#[test]
fn gate_m2_partial_overlap_blob() {
    (0..5).for_each(|seed| assert_partial("blob", seed));
}

/// Gate 3: a box, a cube and a cylinder have several equally good poses. The command says so, lists
/// them, and every listed pose really is a symmetry of the part (best and alternatives alike, each
/// composed with the pose of the moving scan maps the part onto itself).
#[test]
fn gate_m2_symmetric_parts_are_ambiguous_with_real_alternatives() {
    for shape in ["box", "cube", "cylinder"] {
        let (fixed, moving) = symmetric_pair(shape);
        let pose = random_motion(3);
        let mut s = session(&fixed, &moving, &pose);
        let out = pair(&mut s, json!({}));
        eprintln!(
            "{shape}: verdict {}, confidence {}, {} alternatives",
            out["verdict"],
            out["confidence"],
            out["alternatives"].as_array().unwrap().len()
        );
        assert_eq!(out["verdict"], "ambiguous", "{shape}: {out}");
        assert_eq!(out["confident"], false);
        let alts = out["alternatives"].as_array().unwrap();
        assert!(!alts.is_empty(), "{shape}: no alternatives listed");
        let mut poses = vec![matrix_of(&out)];
        poses.extend(
            alts.iter()
                .map(|a| Transform::from_rows(&serde_json::from_value(a["transform"].clone()).unwrap()).unwrap()),
        );
        for (k, found) in poses.iter().enumerate() {
            let e = symmetry_error(&found.then_after(&pose), &fixed);
            eprintln!("{shape}: pose {k} maps the part onto itself within {e:.3} mm");
            // the best pose is exact up to noise; an alternative may also be a slide along the axis of
            // the open cylinder (its ends have no counterpart, so a slide of 1.5 mm costs under 5 %
            // of the area); a wrong pose is more than 10 mm off
            let limit = if k == 0 { 0.5 } else { 2.0 };
            assert!(e < limit, "{shape}: pose {k} is not a symmetry of the part");
        }
        // the best pose and the first alternative are different poses
        assert!(poses[0].max_displacement(&poses[1], &moving.positions) > 5.0, "{shape}: alternative equals the best");
    }
}

/// An ambiguous result moves nothing on `apply`; `choose` applies the named pose as one undo step.
#[test]
fn p9_pair_ambiguous_apply_needs_a_choice_and_is_one_undo_step() {
    let (fixed, moving) = symmetric_pair("box");
    let pose = random_motion(4);
    let mut s = session(&fixed, &moving, &pose);
    let before = s.doc.mesh(s.doc.find_by_name("moving").unwrap().0).unwrap().mesh.positions.clone();
    let depth = s.history.undo_labels().len();
    let out = pair(&mut s, json!({"apply": true}));
    assert_eq!(out["applied"], false, "{out}");
    assert_eq!(s.history.undo_labels().len(), depth, "a refusal is no history step");
    assert!(pair_err(&mut s, json!({"apply": true, "choose": 99})).contains("alternatives"));
    let out = pair(&mut s, json!({"apply": true, "choose": 1}));
    assert_eq!(out["applied"], true, "{out}");
    assert_eq!(s.history.undo_labels().len(), depth + 1);
    let (id, _) = s.doc.find_by_name("moving").unwrap();
    assert_ne!(s.doc.mesh(id).unwrap().mesh.positions, before);
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(s.doc.mesh(id).unwrap().mesh.positions, before, "one undo restores the scan");
}

/// A unique result applies as one undo step, `also` moves with it, a proposal changes nothing.
#[test]
fn p9_pair_unique_applies_once_and_proposals_change_nothing() {
    let (fixed, moving) = pair_of("bracket");
    let pose = random_motion(5);
    let mut s = session(&fixed, &moving, &pose);
    s.doc.add_mesh("buddy", moving.transformed(&pose));
    let (id, _) = s.doc.find_by_name("moving").unwrap();
    let before = s.doc.mesh(id).unwrap().mesh.positions.clone();
    let depth = s.history.undo_labels().len();
    let out = pair(&mut s, json!({}));
    assert_eq!(out["applied"], false);
    assert_eq!(s.doc.mesh(id).unwrap().mesh.positions, before);
    assert_eq!(s.history.undo_labels().len(), depth);
    let out = pair(&mut s, json!({"apply": true, "also": ["buddy"]}));
    assert_eq!(out["applied"], true, "{out}");
    assert_eq!(s.history.undo_labels().len(), depth + 1);
    let (bid, _) = s.doc.find_by_name("buddy").unwrap();
    let (a, b) = (&s.doc.mesh(id).unwrap().mesh.positions, &s.doc.mesh(bid).unwrap().mesh.positions);
    assert!(a.iter().zip(b).all(|(p, q)| (p - q).norm() < 1e-9), "`also` moved by the same motion");
    let bb = s.doc.mesh(id).unwrap().mesh.bbox();
    assert!(bb.min.coords.norm() < 1.0 && (bb.max - Point3::new(60.0, 40.0, 30.0)).norm() < 1.0, "{bb:?}");
}

/// Gate 4: different parts are flagged, nothing is applied, nothing panics.
#[test]
fn gate_m2_different_parts_are_a_no_match() {
    let (bracket, _) = pair_of("bracket");
    let blob = noisy(blob_at(60), 8);
    let torus = noisy(synth::torus(Point3::origin(), Vec3::z(), 25.0, 6.0, 40, 20), 9);
    for (name, other) in [("blob", blob), ("torus", torus)] {
        for (fixed, moving) in [(&bracket, &other), (&other, &bracket)] {
            let mut s = session(fixed, moving, &random_motion(6));
            let out = pair(&mut s, json!({"apply": true}));
            eprintln!(
                "bracket vs {name}: {} overlap {}/{} conflict-free rms {}",
                out["verdict"], out["overlap_moving"], out["overlap_fixed"], out["rms"]
            );
            assert_eq!(out["verdict"], "no_match", "{name}: {out}");
            assert_eq!(out["applied"], false);
            assert_eq!(out["confidence"], 0.0);
        }
    }
}

/// Almost no overlap is a no-match too, not a pose.
#[test]
fn p9_pair_scans_without_common_surface_are_a_no_match() {
    let (full_f, full_m) = pair_of("blob");
    let (fixed, moving) = (cut(&full_f, |c| c.x < -8.0), cut(&full_m, |c| c.x > 12.0));
    let mut s = session(&fixed, &moving, &random_motion(7));
    let out = pair(&mut s, json!({}));
    assert_eq!(out["verdict"], "no_match", "{out}");
}

/// Gate 5: hostile input is an error or a verdict, never a panic.
#[test]
fn gate_m2_hostile_input_never_panics() {
    let bracket = noisy(synth::bracket(), 1);
    let one =
        Mesh::new(vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)], vec![[0, 1, 2]])
            .unwrap();
    let mut nan = bracket.clone();
    nan.positions[7] = Point3::new(f64::NAN, 0.0, 0.0);
    let mut huge = bracket.clone();
    huge.scale(1e300);
    let flat = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 10.0, 10.0, 6, 6);
    let cases: Vec<(&str, Mesh, Mesh)> = vec![
        ("empty moving", Mesh::default(), bracket.clone()),
        ("empty fixed", bracket.clone(), Mesh::default()),
        ("single triangle", one.clone(), bracket.clone()),
        ("single triangle both", one.clone(), one),
        ("nan moving", nan.clone(), bracket.clone()),
        ("nan fixed", bracket.clone(), nan),
        ("huge", huge, bracket.clone()),
        ("plane on plane", flat.clone(), flat),
    ];
    for (name, moving, fixed) in cases {
        let mut s = Session::new();
        s.doc.add_mesh("fixed", fixed);
        s.doc.add_mesh("moving", moving);
        for extra in [
            json!({}),
            json!({"apply": true}),
            json!({"tolerance": 0.0}),
            json!({"tolerance": 1e308}),
            json!({"tolerance": 1e-300}),
            json!({"choose": 5, "apply": true}),
        ] {
            let mut p = json!({"object": "moving", "target": "fixed"});
            p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
            let r = s.execute("align.pair_coarse", p);
            assert!(!matches!(r, Err(surfacecraft_engine::EngineError::Internal(_))), "{name} {extra}: {r:?}");
        }
    }
    // a mesh with itself, and a copy of itself
    let mut s = Session::new();
    s.doc.add_mesh("a", bracket.clone());
    s.doc.add_mesh("b", bracket);
    for target in ["a", "b"] {
        let out = s.execute("align.pair_coarse", json!({"object": "a", "target": target})).unwrap();
        let t = matrix_of(&out);
        assert!(t.angle().to_degrees() < 0.01 && t.translation.norm() < 0.01, "identity expected: {out}");
        assert_eq!(out["verdict"], "unique", "{out}");
        assert!(out["rms"].as_f64().unwrap() < 1e-6, "{out}");
    }
}

/// An empty or single-triangle scan is an error that names the problem.
#[test]
fn p9_pair_degenerate_scans_say_what_is_wrong() {
    let mut s = Session::new();
    s.doc.add_mesh("fixed", synth::bracket());
    s.doc.add_mesh("moving", Mesh::default());
    assert!(pair_err(&mut s, json!({})).contains("has 0 triangles"));
    s.doc.add_mesh(
        "tri",
        Mesh::new(vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)], vec![[0, 1, 2]])
            .unwrap(),
    );
    assert!(pair_err(&mut s, json!({"object": "tri"})).contains("has 1 triangles"));
    assert!(pair_err(&mut s, json!({"tolerance": -1.0, "object": "fixed", "target": "fixed"})).contains("tolerance"));
}

/// Cost on a scan of about a million triangles (local; release build): the bracket refined to a
/// fine edge length, the moving copy another tessellation of it. Reports the time and the error.
/// `cargo test --release -p surfacecraft-engine --test p9_pair_coarse -- --ignored --nocapture large`
#[test]
#[ignore = "timing on 1 million triangles; run locally in release"]
fn local_large_pair_cost() {
    let fine = |m: &Mesh| synth::refine(&noisy(m.clone(), 1), |_| 0.28);
    let fixed = fine(&bracket_at(12, 48, 16));
    let moving = synth::refine(&noisy(bracket_at(17, 64, 20), 2), |_| 0.28);
    eprintln!("triangles: fixed {}, moving {}", fixed.triangle_count(), moving.triangle_count());
    let pose = random_motion(2);
    let mut s = session(&fixed, &moving, &pose);
    let t = std::time::Instant::now();
    let out = pair(&mut s, json!({}));
    let (angle, disp) = pose_error(&matrix_of(&out), &pose, &moving);
    eprintln!(
        "pair_coarse: {:.2} s, angle {angle:.4} deg, displacement {disp:.4} mm, verdict {}",
        t.elapsed().as_secs_f64(),
        out["verdict"]
    );
    assert!(angle <= MAX_ANGLE_DEG && disp <= MAX_DISPLACEMENT_MM);
}

/// One real scan, by number (`SURFACECRAFT_LOCAL_CORPUS`, `SURFACECRAFT_LOCAL_SCAN`, default 1): a
/// decimated copy is moved by a known motion in memory and brought back. Prints the pose error and
/// the time; nothing is written anywhere.
#[test]
#[ignore = "needs SURFACECRAFT_LOCAL_CORPUS"]
fn local_corpus_pair() {
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
    s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "fixed"})).unwrap();
    s.execute("mesh.import", json!({"path": path.to_string_lossy(), "name": "other"})).unwrap();
    // two different decimations of the scan (interior vertices differ; the boundary is kept in place)
    s.execute("mesh.decimate", json!({"object": "fixed", "ratio": 0.5})).unwrap();
    s.execute("mesh.decimate", json!({"object": "other", "ratio": 0.4})).unwrap();
    let (fid, _) = s.doc.find_by_name("fixed").unwrap();
    let fixed = s.doc.mesh(fid).unwrap().mesh.as_ref().clone();
    let (oid, _) = s.doc.find_by_name("other").unwrap();
    let other = s.doc.mesh(oid).unwrap().mesh.as_ref().clone();
    let pose = random_motion(3);
    s.doc.add_mesh("moving", other.transformed(&pose));
    eprintln!("scan {number}: {} triangles", fixed.triangle_count());
    let t = std::time::Instant::now();
    let out = pair(&mut s, json!({}));
    let (angle, disp) = pose_error(&matrix_of(&out), &pose, &other);
    eprintln!(
        "scan {number}: {:.2} s, angle {angle:.4} deg, displacement {disp:.4} mm, verdict {}, overlap {}, stability {:.5}, {} refined",
        t.elapsed().as_secs_f64(),
        out["verdict"],
        out["overlap_moving"],
        stab(&out),
        out["refined"]
    );
}

/// A cancelled run is the cancel error of the engine, not a data error, and leaves no trace.
#[test]
fn p9_pair_cancelled_before_it_starts_is_the_cancel_error() {
    let (fixed, moving) = pair_of("bracket");
    let mut s = session(&fixed, &moving, &random_motion(8));
    let depth = s.history.undo_labels().len();
    s.cancel.cancel();
    let r = s.execute("align.pair_coarse", json!({"object": "moving", "target": "fixed", "apply": true}));
    assert_eq!(r, Err(surfacecraft_engine::EngineError::Failed(surfacecraft_engine::cmd::CANCELLED.into())));
    assert_eq!(s.history.undo_labels().len(), depth);
}
