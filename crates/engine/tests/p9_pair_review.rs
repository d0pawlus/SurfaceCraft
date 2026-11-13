//! P9-M2, review rounds: a margin in shares, the chosen pose's own numbers, small patches, unwelded
//! meshes, stability on fine meshes and slender bars (see `p9_pair_coarse.rs` for the gates).
mod pair_common;
use pair_common::*;
use serde_json::json;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Transform, Vec3, synth};

// ---------------------------------------------------------------------------------------------
// Review round: a margin in shares, the chosen pose's own numbers, small patches, unwelded meshes.

/// Finding 1: the margin is a share of the moving scan's area. A symmetric plate whose only
/// asymmetry is a bump of 5 % of the area has a unique pose, and the half-turn poses (which lose
/// the bump) are never returned as unique.
#[test]
fn gate_m2_a_small_asymmetric_feature_makes_a_symmetric_plate_unique() {
    let mut worst = (0.0_f64, 0.0_f64);
    for seed in 0..10u64 {
        let fixed = noisy(plate_at(10, 16), 1 + 10 * seed);
        let moving = noisy(plate_at(14, 20), 2 + 10 * seed);
        let pose = random_motion(seed);
        let mut s = session(&fixed, &moving, &pose);
        let out = pair(&mut s, json!({}));
        let (angle, disp) = pose_error(&matrix_of(&out), &pose, &moving);
        eprintln!(
            "STABILITY plate seed {seed}: {:.5}, angle {angle:.4} deg, displacement {disp:.4} mm, verdict {}, confidence {:.2}",
            stab(&out),
            out["verdict"],
            out["confidence"].as_f64().unwrap()
        );
        assert!(
            out["verdict"] != "unique" || (angle <= MAX_ANGLE_DEG && disp <= MAX_DISPLACEMENT_MM),
            "plate seed {seed}: a wrong pose is unique: {angle} deg {disp} mm"
        );
        assert_eq!(out["verdict"], "unique", "plate seed {seed}: {out}");
        worst = (worst.0.max(angle), worst.1.max(disp));
    }
    eprintln!("plate: worst of ten trials: angle {:.4} deg, displacement {:.4} mm", worst.0, worst.1);
}

/// Finding 2: `choose` shows and applies the chosen pose with its own numbers.
#[test]
fn p9_pair_choose_reports_the_chosen_poses_own_numbers() {
    let (fixed, moving) = symmetric_pair("box");
    let pose = random_motion(4);
    let mut s = session(&fixed, &moving, &pose);
    let proposal = pair(&mut s, json!({}));
    let alt = &proposal["alternatives"].as_array().unwrap()[0];
    let out = pair(&mut s, json!({"apply": true, "choose": 1}));
    let applied = matrix_of(&out);
    let wanted = Transform::from_rows(&serde_json::from_value(alt["transform"].clone()).unwrap()).unwrap();
    assert!(applied.max_displacement(&wanted, &moving.positions) < 1e-9, "the matrix is the alternative's");
    assert_eq!(out["applied"], true);
    for key in ["rms", "overlap_moving", "overlap_fixed", "confidence", "method"] {
        let (a, b) =
            (if key == "rms" { &out["rms"] } else { &out[key] }, if key == "rms" { &alt["rms"] } else { &alt[key] });
        assert_eq!(a, b, "{key} of the chosen pose");
    }
    assert_eq!(out["confidence"], 0.0, "an alternative is not certain");
    assert_ne!(out["rms"], proposal["rms"], "not the best pose's rms");
}

/// Finding 7: `choose` without `apply` is a parameter error; a refused `apply` says so.
#[test]
fn p9_pair_choose_needs_apply_and_a_refusal_is_explicit() {
    let (fixed, moving) = symmetric_pair("box");
    let mut s = session(&fixed, &moving, &random_motion(4));
    assert!(pair_err(&mut s, json!({"choose": 1})).contains("apply"));
    let out = pair(&mut s, json!({"apply": true}));
    assert_eq!(
        (out["refused"].clone(), out["applied"].clone()),
        (json!(true), json!(false)),
        "ambiguous, no choose: {out}"
    );
    let out = pair(&mut s, json!({}));
    assert_eq!(out["refused"], false, "a proposal is not a refusal");
    let (bracket, _) = pair_of("bracket");
    let blob = noisy(blob_at(60), 8);
    let mut s = session(&bracket, &blob, &random_motion(6));
    let out = pair(&mut s, json!({"apply": true}));
    assert_eq!(
        (out["verdict"].clone(), out["refused"].clone(), out["applied"].clone()),
        (json!("no_match"), json!(true), json!(false))
    );
    assert!(out["note"].as_str().unwrap().contains("nothing was moved"));
}

/// Finding 6: a mesh that repeats its vertices per triangle gives the pose and verdict of the
/// welded one (borders are found by position).
#[test]
fn p9_pair_unwelded_scans_give_the_same_answer() {
    let unweld = |m: &Mesh| {
        let mut positions = Vec::new();
        let mut triangles = Vec::new();
        for t in &m.triangles {
            let base = positions.len() as u32;
            positions.extend(t.iter().map(|&i| m.positions[i as usize]));
            triangles.push([base, base + 1, base + 2]);
        }
        Mesh { positions, triangles }
    };
    let (full_f, full_m) = pair_of("bracket");
    let (fixed, moving) = (cut(&full_f, |c| c.z > 1.0 && c.x <= 54.0), cut(&full_m, |c| c.z > 1.0 && c.x >= 6.0));
    let pose = random_motion(2);
    let mut welded = session(&fixed, &moving, &pose);
    let mut loose = session(&unweld(&fixed), &unweld(&moving), &pose);
    let (a, b) = (pair(&mut welded, json!({})), pair(&mut loose, json!({})));
    let (ea, eb) = (pose_error(&matrix_of(&a), &pose, &moving), pose_error(&matrix_of(&b), &pose, &moving));
    eprintln!("welded {ea:?} {}, unwelded {eb:?} {}", a["verdict"], b["verdict"]);
    assert_eq!(a["verdict"], "unique");
    assert_eq!(b["verdict"], "unique", "{b}");
    assert!(eb.0 <= MAX_ANGLE_DEG && eb.1 <= MAX_DISPLACEMENT_MM, "{eb:?}");
}

/// Finding 4: a near-cubic part whose only asymmetry is a small bump has 24 near-equal principal
/// candidates; the true pose must survive the shortlist.
#[test]
fn gate_m2_near_cubic_part_with_a_small_feature_is_unique() {
    let cube = |n: usize| {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(30.0, 30.0, 30.0), n);
        m.append(&synth::sphere(Point3::new(8.0, 9.0, 30.0), 5.0, 32, 16));
        m
    };
    for seed in 0..5u64 {
        let (fixed, moving) = (noisy(cube(8), 1 + 10 * seed), noisy(cube(11), 2 + 10 * seed));
        let pose = random_motion(seed);
        let mut s = session(&fixed, &moving, &pose);
        let out = pair(&mut s, json!({}));
        let (angle, disp) = pose_error(&matrix_of(&out), &pose, &moving);
        eprintln!(
            "STABILITY cube seed {seed}: {:.5} {angle:.4} deg {disp:.4} mm {} ({} refined)",
            stab(&out),
            out["verdict"],
            out["refined"]
        );
        assert!(
            out["verdict"] != "unique" || (angle <= MAX_ANGLE_DEG && disp <= MAX_DISPLACEMENT_MM),
            "wrong unique pose"
        );
        assert_eq!(out["verdict"], "unique", "cube seed {seed}: {out}");
    }
}

/// A patch of about 15 % of the blob on the whole blob is found: `unique`, within the bound a patch
/// allows. A patch of lever arm L (about 12 mm here) turns by about delta / L for an error delta in
/// its points; delta is the noise (0.05) plus the chord error of the two tessellations (0.05), so
/// about 0.1 / 12 rad = 0.5 degrees and a displacement of a few tenths of a millimetre (measured
/// worst: 0.25 degrees, 0.18 mm). A wrong pose is centimetres off.
#[test]
fn gate_m2_a_small_blob_patch_is_unique_and_within_the_lever_arm_bound() {
    let mut outcomes = Vec::new();
    for seed in 0..5u64 {
        let (full_f, full_m) = pair_of_trial("blob", seed);
        let patch = cut(&full_m, |c| c.x > 15.0 && c.z > -4.0);
        let share = patch.area() / full_m.area();
        assert!((0.05..=0.2).contains(&share), "construction: the patch is {share} of the blob");
        let pose = random_motion(seed);
        let mut s = session(&full_f, &patch, &pose);
        let out = pair(&mut s, json!({}));
        let (angle, disp) = pose_error(&matrix_of(&out), &pose, &patch);
        eprintln!(
            "STABILITY blob patch {share:.2} seed {seed}: {:.5}, {angle:.3} deg {disp:.3} mm, {} overlap {:.2}/{:.2}",
            stab(&out),
            out["verdict"],
            out["overlap_moving"].as_f64().unwrap(),
            out["overlap_fixed"].as_f64().unwrap()
        );
        assert!(angle <= 0.5 && disp <= 0.3, "seed {seed}: {angle} deg, {disp} mm (bound 0.5 deg, 0.3 mm)");
        outcomes.push(out["verdict"].as_str().unwrap().to_string());
    }
    assert!(outcomes.iter().all(|v| v == "unique"), "the patch of an organic part has one place: {outcomes:?}");
}

fn ribbed_plate(block: usize, rings: usize) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(70.0, 30.0, 4.0), block);
    for x in [10.0, 25.0, 40.0, 55.0] {
        for y in [8.0, 22.0] {
            m.append(&synth::sphere(Point3::new(x, y, 4.0), 3.0, 2 * rings, rings));
        }
    }
    m
}

/// Finding 5b: a patch that fits in many places is never `unique`: a flat patch of the bracket, a
/// sector of a cylinder, two bumps of a plate with eight.
#[test]
fn gate_m2_a_patch_that_fits_in_many_places_is_never_unique() {
    let cyl = |nu: usize, seed: u64| noisy(synth::cylinder(Point3::origin(), Vec3::z(), 10.0, 30.0, nu, 12), seed);
    for seed in 0..3u64 {
        let cases: Vec<(&str, Mesh, Mesh)> = vec![
            (
                "flat patch",
                noisy(bracket_at(12, 48, 16), 1 + seed),
                cut(&noisy(bracket_at(17, 64, 20), 5 + seed), |c| {
                    (19.9..20.1).contains(&c.z) && c.x > 21.0 && c.y > 19.0
                }),
            ),
            (
                "cylinder sector",
                cyl(48, 1 + seed),
                cut(&cyl(64, 5 + seed), |c| c.y > 0.0 && c.x > 0.0 && c.x * c.x < 0.5 * (c.x * c.x + c.y * c.y)),
            ),
            (
                "repeated bumps",
                noisy(ribbed_plate(14, 8), 1 + seed),
                cut(&noisy(ribbed_plate(19, 10), 5 + seed), |c| c.z > 3.0 && c.x > 18.0 && c.x < 47.0),
            ),
        ];
        for (name, fixed, patch) in cases {
            let mut s = session(&fixed, &patch, &random_motion(seed));
            let out = pair(&mut s, json!({}));
            eprintln!(
                "{name} seed {seed}: {} overlap {:.2}, {} alternatives",
                out["verdict"],
                out["overlap_moving"].as_f64().unwrap(),
                out["alternatives"].as_array().unwrap().len()
            );
            assert_ne!(out["verdict"], "unique", "{name} seed {seed} fits in many places: {out}");
        }
    }
}

/// Finding 9: malformed meshes and parameter combinations give the specific error or verdict.
#[test]
fn p9_pair_malformed_meshes_and_choices_give_specific_answers() {
    let (fixed, moving) = pair_of("bracket");
    let pose = random_motion(1);
    // duplicate and degenerate triangles are harmless
    let mut messy = moving.clone();
    messy.triangles.extend([messy.triangles[0], messy.triangles[0], [0, 0, 1], [5, 5, 5]]);
    let mut s = session(&fixed, &messy, &pose);
    let out = pair(&mut s, json!({}));
    let (angle, disp) = pose_error(&matrix_of(&out), &pose, &moving);
    assert_eq!(out["verdict"], "unique", "{out}");
    assert!(angle <= MAX_ANGLE_DEG && disp <= MAX_DISPLACEMENT_MM, "{angle} {disp}");
    // an index outside the vertex list is an invalid-scan error naming the scan, in either role
    let mut broken = moving.clone();
    broken.triangles.push([0, 1, u32::MAX]);
    for (obj, tgt, who) in [("moving", "fixed", "moving"), ("moving", "fixed", "fixed")] {
        let mut s = Session::new();
        s.doc.add_mesh("fixed", if who == "fixed" { broken.clone() } else { fixed.clone() });
        s.doc.add_mesh("moving", if who == "moving" { broken.clone() } else { moving.clone() });
        let e = s.execute("align.pair_coarse", json!({"object": obj, "target": tgt})).unwrap_err();
        assert!(
            matches!(
                e,
                surfacecraft_engine::EngineError::Failed(_) | surfacecraft_engine::EngineError::InvalidParams(_)
            ),
            "{e:?}"
        );
        assert!(e.to_string().contains(&format!("the {who} scan is invalid")), "{e}");
    }
    // `choose` on a unique result: 0 applies the best pose, anything else is a parameter error
    let mut s = session(&fixed, &moving, &pose);
    assert!(pair_err(&mut s, json!({"apply": true, "choose": 1})).contains("unique (use 0)"));
    let out = pair(&mut s, json!({"apply": true, "choose": 0}));
    assert_eq!((out["applied"].clone(), out["verdict"].clone()), (json!(true), json!("unique")));
    // `choose` beyond the alternatives of an ambiguous result
    let (bf, bm) = symmetric_pair("box");
    let mut s = session(&bf, &bm, &pose);
    let n = pair(&mut s, json!({}))["alternatives"].as_array().unwrap().len();
    assert!(pair_err(&mut s, json!({"apply": true, "choose": n + 1})).contains(&format!("there are {n} alternatives")));
}

/// Gate (a) of review round 2: the stability does not depend on the density of the mesh (a raw face
/// normal of a fine noisy mesh tilts by noise over edge length; the smoothed normals do not), so
/// sliding patches stay `ambiguous` and pinned ones `unique` on meshes of 0.3 to 0.5 mm edges as on
/// coarse ones. Prints the table.
#[test]
fn gate_m2_stability_does_not_move_with_the_density_of_the_mesh() {
    let cyl =
        |nu: usize, nv: usize, seed: u64| noisy(synth::cylinder(Point3::origin(), Vec3::z(), 10.0, 30.0, nu, nv), seed);
    let sector = |c: &Point3| c.y > 0.0 && c.x > 0.0 && c.x * c.x < 0.5 * (c.x * c.x + c.y * c.y);
    let flat = |c: &Point3| (19.9..20.1).contains(&c.z) && c.x > 21.0 && c.y > 19.0;
    let blob_patch = |c: &Point3| c.x > 15.0 && c.z > -4.0;
    // (name, must be unique, coarse (fixed, patch), fine (fixed, patch))
    let cases: Vec<(&str, bool, (Mesh, Mesh), (Mesh, Mesh))> = vec![
        (
            "cylinder sector",
            false,
            (cyl(48, 12, 1), cut(&cyl(64, 14, 5), sector)),
            (cyl(210, 100, 1), cut(&cyl(250, 120, 5), sector)),
        ),
        (
            "flat patch",
            false,
            (noisy(bracket_at(12, 48, 16), 1), cut(&noisy(bracket_at(17, 64, 20), 5), flat)),
            (fine(&bracket_at(12, 48, 16), 1), cut(&fine(&bracket_at(17, 64, 20), 5), flat)),
        ),
        (
            "blob patch",
            true,
            (noisy(blob_at(48), 1), cut(&noisy(blob_at(62), 5), blob_patch)),
            (noisy(blob_at(420), 1), cut(&noisy(blob_at(480), 5), blob_patch)),
        ),
        (
            "plate, 5 % bump",
            true,
            (noisy(plate_at(10, 16), 1), noisy(plate_at(14, 20), 5)),
            (fine(&plate_at(10, 16), 1), fine(&plate_at(14, 20), 5)),
        ),
    ];
    for (name, pinned, (cf, cm), (ff, fm)) in cases {
        let (vc, sc) = patch_case(&cf, &cm, 1);
        let (vf, sf) = patch_case(&ff, &fm, 1);
        eprintln!(
            "STABILITY {name}: coarse {sc:.5} ({vc}, {} tris), fine {sf:.5} ({vf}, {} tris)",
            cf.triangle_count(),
            ff.triangle_count()
        );
        for (v, label) in [(&vc, "coarse"), (&vf, "fine")] {
            assert_eq!(v == "unique", pinned, "{name} on the {label} mesh: {v}");
        }
        let ratio = sf.max(1e-9) / sc.max(1e-9);
        assert!((0.33..=3.0).contains(&ratio), "{name}: the stability moved with the density: {sc} -> {sf}");
    }
}

fn bar_at(n: usize) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(400.0, 20.0, 20.0), n);
    m.append(&synth::cuboid(Point3::new(0.0, 0.0, 20.0), Vec3::new(60.0, 20.0, 10.0), 6));
    m
}

/// Finding 2: a bar of aspect 20:1 with a block at one end has a unique pose (four flat faces pin
/// its long axis however slender it is); a long plain cylinder is still ambiguous.
#[test]
fn gate_m2_a_slender_bar_is_unique_and_a_long_cylinder_is_not() {
    let (fixed, moving) = (noisy(bar_at(30), 1), noisy(bar_at(41), 2));
    let pose = random_motion(5);
    let mut s = session(&fixed, &moving, &pose);
    let out = pair(&mut s, json!({}));
    let (angle, disp) = pose_error(&matrix_of(&out), &pose, &moving);
    eprintln!("STABILITY bar: {:.5} {} {angle:.4} deg {disp:.4} mm", stab(&out), out["verdict"]);
    assert_eq!(out["verdict"], "unique", "{out}");
    assert!(angle <= MAX_ANGLE_DEG && disp <= MAX_DISPLACEMENT_MM, "{angle} deg {disp} mm");
    let long = |nu: usize, nv: usize, seed: u64| {
        noisy(synth::cylinder(Point3::origin(), Vec3::z(), 10.0, 400.0, nu, nv), seed)
    };
    let mut s = session(&long(48, 60, 1), &long(64, 80, 2), &pose);
    let out = pair(&mut s, json!({}));
    eprintln!("STABILITY long cylinder: {:.5} {}", stab(&out), out["verdict"]);
    assert_eq!(out["verdict"], "ambiguous", "{out}");
}
