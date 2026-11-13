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

/// Mirrors `STABILITY_FLOOR` of the library (below it a pose is called free without a test).
const FLOOR: f64 = 0.001;
/// Mirrors `AMBIGUITY_MARGIN`.
const MARGIN: f64 = 0.03;

/// How a case is built: (fixed, moving before its motion) on a coarse or a fine mesh.
type Build = Box<dyn Fn(bool) -> (Mesh, Mesh)>;

/// What a case must give: ambiguous, or unique within a pose bound (degrees, mm).
#[derive(Clone, Copy)]
enum Expect {
    /// Ambiguous, and the probe itself found slides (displaced starts that settle elsewhere).
    Slides,
    /// Ambiguous by whatever route (candidates that fit as well, or slides).
    Ambiguous,
    Unique(f64, f64),
}

fn cyl_at(h: f64, nu: usize, nv: usize, seed: u64) -> Mesh {
    noisy(synth::cylinder(Point3::origin(), Vec3::z(), 10.0, h, nu, nv), seed)
}

fn cubic_at(n: usize) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(30.0, 30.0, 30.0), n);
    m.append(&synth::sphere(Point3::new(8.0, 9.0, 30.0), 5.0, 32, 16));
    m
}

/// Refined to edges of `h` mm (the bar would have 7 million triangles at 0.35).
fn fine_at(m: &Mesh, h: f64, seed: u64) -> Mesh {
    noisy(synth::refine(m, |_| h), seed)
}

fn duo(fixed: Mesh, moving: Mesh) -> (Mesh, Mesh) {
    (fixed, moving)
}

fn cases() -> Vec<(&'static str, Expect, Build)> {
    let sector = |c: &Point3| c.y > 0.0 && c.x > 0.0 && c.x * c.x < 0.5 * (c.x * c.x + c.y * c.y);
    let sliver = |c: &Point3| (19.9..20.1).contains(&c.z) && c.x > 21.0 && c.y > 19.0;
    let interior = |c: &Point3| c.z > 3.9 && c.x > 15.0 && c.x < 40.0 && c.y > 10.0 && c.y < 30.0;
    let blob_patch = |c: &Point3| c.x > 15.0 && c.z > -4.0;
    let bumps = |c: &Point3| c.z > 3.0 && c.x > 18.0 && c.x < 47.0;
    let plain_plate = |n: usize| synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 4.0), n);
    vec![
        (
            "strictly flat patch",
            Expect::Slides,
            Box::new(move |f| {
                if f {
                    duo(fine(&plain_plate(10), 1), cut(&fine(&plain_plate(14), 5), interior))
                } else {
                    duo(noisy(plain_plate(10), 1), cut(&noisy(plain_plate(14), 5), interior))
                }
            }),
        ),
        (
            "patch held by slivers",
            Expect::Slides,
            Box::new(move |f| {
                if f {
                    duo(fine(&bracket_at(12, 48, 16), 1), cut(&fine(&bracket_at(17, 64, 20), 5), sliver))
                } else {
                    duo(noisy(bracket_at(12, 48, 16), 1), cut(&noisy(bracket_at(17, 64, 20), 5), sliver))
                }
            }),
        ),
        (
            "cylinder sector",
            Expect::Slides,
            Box::new(move |f| {
                if f {
                    duo(cyl_at(30.0, 210, 100, 1), cut(&cyl_at(30.0, 250, 120, 5), sector))
                } else {
                    duo(cyl_at(30.0, 48, 12, 1), cut(&cyl_at(30.0, 64, 14, 5), sector))
                }
            }),
        ),
        (
            "repeated bumps",
            Expect::Ambiguous,
            Box::new(move |f| {
                if f {
                    duo(fine(&ribbed_plate(14, 8), 1), cut(&fine(&ribbed_plate(19, 10), 5), bumps))
                } else {
                    duo(noisy(ribbed_plate(14, 8), 1), cut(&noisy(ribbed_plate(19, 10), 5), bumps))
                }
            }),
        ),
        (
            "blob patch",
            Expect::Unique(0.5, 0.3),
            Box::new(move |f| {
                if f {
                    duo(noisy(blob_at(420), 1), cut(&noisy(blob_at(480), 5), blob_patch))
                } else {
                    duo(noisy(blob_at(48), 1), cut(&noisy(blob_at(62), 5), blob_patch))
                }
            }),
        ),
        (
            "plate, 5 % bump",
            Expect::Unique(MAX_ANGLE_DEG, MAX_DISPLACEMENT_MM),
            Box::new(move |f| {
                if f {
                    duo(fine(&plate_at(10, 16), 1), fine(&plate_at(14, 20), 5))
                } else {
                    duo(noisy(plate_at(10, 16), 1), noisy(plate_at(14, 20), 5))
                }
            }),
        ),
        (
            "near-cubic with a bump",
            Expect::Unique(MAX_ANGLE_DEG, MAX_DISPLACEMENT_MM),
            Box::new(move |f| {
                if f {
                    duo(fine(&cubic_at(8), 1), fine(&cubic_at(11), 5))
                } else {
                    duo(noisy(cubic_at(8), 1), noisy(cubic_at(11), 5))
                }
            }),
        ),
        (
            "slender bar with a block",
            Expect::Unique(MAX_ANGLE_DEG, MAX_DISPLACEMENT_MM),
            Box::new(move |f| {
                if f {
                    duo(fine_at(&bar_at(30), 1.0, 1), fine_at(&bar_at(41), 1.0, 5))
                } else {
                    duo(noisy(bar_at(30), 1), noisy(bar_at(41), 5))
                }
            }),
        ),
        (
            "long cylinder",
            Expect::Slides,
            Box::new(move |f| {
                if f {
                    duo(cyl_at(400.0, 180, 1150, 1), cyl_at(400.0, 210, 1300, 5))
                } else {
                    duo(cyl_at(400.0, 48, 60, 1), cyl_at(400.0, 64, 80, 5))
                }
            }),
        ),
    ]
}

/// Review round 3: a pose is decided by pushing it, not by a threshold on how loose it is. Every
/// case on a coarse and a fine mesh (0.3 to 0.5 mm edges, 1 mm for the 400 mm bar; sigma 0.05). Prints the table: stability,
/// displaced starts tried, slides found, the smallest clearance of the pose over the displaced
/// starts that ended elsewhere (the bound is the margin 0.03), the largest distance a displaced
/// start came back from.
#[test]
fn gate_m2_loose_poses_are_decided_by_pushing_them() {
    for (name, expect, build) in cases() {
        for fine_mesh in [false, true] {
            let label = if fine_mesh { "fine" } else { "coarse" };
            let (fixed, moving) = build(fine_mesh);
            let pose = random_motion(1);
            let mut s = session(&fixed, &moving, &pose);
            let out = pair(&mut s, json!({}));
            let (angle, disp) = pose_error(&matrix_of(&out), &pose, &moving);
            let (stability, probed, slides) =
                (stab(&out), out["probed"].as_u64().unwrap(), out["slides"].as_u64().unwrap());
            let margin = out["probe_margin"].as_f64();
            let gap = out["probe_return_gap"].as_f64().unwrap();
            eprintln!(
                "PUSH {name} [{label}, {} tris]: {} stability {stability:.5}, probed {probed}, slides {slides}, margin {margin:?}, return gap {gap:.3} mm, pose {angle:.3} deg {disp:.3} mm, {} alternatives",
                fixed.triangle_count(),
                out["verdict"],
                out["alternatives"].as_array().unwrap().len()
            );
            match expect {
                Expect::Slides => {
                    assert_eq!(out["verdict"], "ambiguous", "{name} [{label}]: {out}");
                    assert!(slides >= 1, "{name} [{label}]: the probe found no slide ({probed} starts)");
                }
                Expect::Ambiguous => {
                    assert_eq!(out["verdict"], "ambiguous", "{name} [{label}]: {out}");
                    // free (below the floor) or found sliding by the probe, or ambiguous by the clearance
                    // of other candidates: never unique
                }
                Expect::Unique(max_angle, max_disp) => {
                    assert_eq!(out["verdict"], "unique", "{name} [{label}]: {out}");
                    assert!(angle <= max_angle && disp <= max_disp, "{name} [{label}]: {angle} deg {disp} mm");
                    assert!(stability > 3.0 * FLOOR, "{name} [{label}]: the floor must not catch it ({stability})");
                    assert_eq!(slides, 0);
                    // every displaced start that ended elsewhere fit clearly worse (twice the margin or more)
                    assert!(margin.is_none_or(|m| m >= 2.0 * MARGIN), "{name} [{label}]: margin {margin:?}");
                    // and those that came back did so within a quarter of the pose tolerance (2 % of the diagonal)
                    assert!(gap <= 0.25 * 0.02 * fixed.bbox().diagonal(), "{name} [{label}]: returned {gap} mm off");
                }
            }
        }
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

/// Restored (it was replaced by the table above in round 3 because the old "flat patch" case read
/// 0.00001 to 0.014 depending on where ICP left a barely held pose, which says nothing about the
/// density; that case was a garbage pose, see `gate_m2_loose_poses...`): the stability of a pose
/// does not move with the density of the mesh. Free parts (a patch strictly inside a flat face, a
/// cylinder sector) read below the floor 0.001 on both the coarse and the 0.35 mm mesh, with at
/// least a factor 4 to spare; pinned ones move by less than a factor 3 between the meshes.
#[test]
fn gate_m2_stability_does_not_move_with_the_density_of_the_mesh() {
    let wanted = ["strictly flat patch", "cylinder sector", "blob patch", "plate, 5 % bump", "near-cubic with a bump"];
    for (name, expect, build) in cases().into_iter().filter(|c| wanted.contains(&c.0)) {
        let mut read = Vec::new();
        for fine_mesh in [false, true] {
            let (fixed, moving) = build(fine_mesh);
            let mut s = session(&fixed, &moving, &random_motion(1));
            read.push(stab(&pair(&mut s, json!({}))));
        }
        eprintln!("DENSITY {name}: coarse {:.5}, fine {:.5}", read[0], read[1]);
        match expect {
            Expect::Slides | Expect::Ambiguous => {
                assert!(read.iter().all(|&v| v < FLOOR), "{name}: free parts read {read:?}, floor {FLOOR}");
                assert!(read.iter().all(|&v| 4.0 * v < FLOOR), "{name}: not 4 times below the floor: {read:?}");
            }
            Expect::Unique(..) => {
                let ratio = read[1] / read[0];
                assert!((1.0 / 3.0..=3.0).contains(&ratio), "{name}: moved with the density, {read:?}");
            }
        }
    }
}
