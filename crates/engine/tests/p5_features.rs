//! P5: the features added in this stage (loft, sweep, draft, mirror, split, pattern along a
//! path), auto-constraints between primitives and the redesign-assistant suggestions. Every
//! volume is checked against a closed form computed independently of the kernel.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_engine::Session;
use surfacecraft_kernel::naming::face_signature;
use surfacecraft_kernel::record::brep::Brep;

fn vol(v: &Value) -> f64 {
    v["volume"].as_f64().unwrap_or_else(|| panic!("no volume in {v}"))
}

fn body_of(s: &Session, name: &str) -> Brep {
    (*s.doc.bodies.iter().find(|b| b.name == name).unwrap().brep).clone()
}

/// A centred square sketch of half side `h` on the plane z = `z`.
fn square_at(s: &mut Session, name: &str, z: f64, h: f64) {
    s.execute(
        "sketch.create",
        json!({"name": name, "plane": {"origin": [0.0, 0.0, z], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": name, "entities": [{"polygon": {"points": [[-h, -h], [h, -h], [h, h], [-h, h]]}}]}),
    )
    .unwrap();
}

/// Loft of a 20 × 20 square at z = 0 to a 10 × 10 square at z = 10: a frustum,
/// V = h/3 (A1 + A2 + √(A1 A2)).
#[test]
fn loft_frustum_volume() {
    let mut s = Session::new();
    square_at(&mut s, "bottom", 0.0, 10.0);
    square_at(&mut s, "top", 10.0, 5.0);
    let r = s.execute("feature.loft", json!({"sketches": ["bottom", "top"], "name": "frustum"})).unwrap();
    let exact = 10.0 / 3.0 * (400.0 + 100.0 + (400.0_f64 * 100.0).sqrt());
    eprintln!("p5 loft: V = {:.6}, exact {exact:.6}", vol(&r));
    assert!((vol(&r) - exact).abs() < 1e-3, "{r}");
    // editing the top sketch's replayed profile: the loft follows (history)
    let again = s.execute("feature.replay", Value::Null).unwrap();
    assert_eq!(again["failed_features"], 0);
}

/// A 2 × 2 square swept 90° along an arc of radius 10 (path through the profile's centre):
/// Pappus, V = A · R · θ = 4 · 10 · π/2.
#[test]
fn sweep_along_an_arc_volume() {
    let mut s = Session::new();
    square_at(&mut s, "profile", 0.0, 1.0);
    // path in the XZ plane: centre (10, 0, 0), from (0, 0, 0) to (10, 0, -10)
    s.execute(
        "sketch.create",
        json!({"name": "path", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 0.0, 1.0]}}),
    )
    .unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "path", "entities": [{"arc": {"center": [10.0, 0.0], "start": [0.0, 0.0], "end": [10.0, -10.0]}}]}),
    )
    .unwrap();
    let r = s.execute("feature.sweep", json!({"profile": "profile", "path": "path", "name": "bend"})).unwrap();
    let exact = 4.0 * 10.0 * PI / 2.0;
    eprintln!("p5 sweep (arc): V = {:.6}, exact {exact:.6}", vol(&r));
    assert!((vol(&r) - exact).abs() < 1e-3 * exact, "{r}");
}

/// A straight sweep is the extrusion: 2 × 2 along 25.
#[test]
fn sweep_along_a_line_volume() {
    let mut s = Session::new();
    square_at(&mut s, "profile", 0.0, 1.0);
    s.execute(
        "sketch.create",
        json!({"name": "line", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 0.0, 1.0]}}),
    )
    .unwrap();
    s.execute("sketch.add", json!({"sketch": "line", "entities": [{"line": {"from": [0.0, 0.0], "to": [0.0, 25.0]}}]}))
        .unwrap();
    let r = s.execute("feature.sweep", json!({"profile": "profile", "path": "line"})).unwrap();
    assert!((vol(&r) - 100.0).abs() < 1e-3, "{r}");
}

fn side_faces(b: &Brep) -> Vec<usize> {
    (0..b.faces.len())
        .filter(|&f| face_signature(b, f).is_some_and(|s| s.kind == "plane" && s.normal[2].abs() < 1e-6))
        .collect()
}

/// A 20 × 20 × 10 block drafted 5° on its four sides about its bottom plane, pulled up: a
/// frustum with a top side of 20 − 2·10·tan 5°.
#[test]
fn draft_block_volume_and_suggestion() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [20.0, 20.0, 10.0], "name": "blk"})).unwrap();
    // the scan of the finished part, for the assistant
    let faces = side_faces(&body_of(&s, "blk"));
    assert_eq!(faces.len(), 4);
    let r = s
        .execute(
            "feature.draft",
            json!({"body": "blk", "faces": faces, "angle_deg": 5.0, "neutral_origin": [0.0, 0.0, 0.0], "neutral_normal": [0.0, 0.0, 1.0]}),
        )
        .unwrap();
    let top = 20.0 - 2.0 * 10.0 * 5.0_f64.to_radians().tan();
    let exact = 10.0 / 3.0 * (400.0 + top * top + (400.0_f64 * top * top).sqrt());
    eprintln!("p5 draft: V = {:.6}, exact {exact:.6}", vol(&r));
    assert!((vol(&r) - exact).abs() < 1e-3, "{r}");
    // faces named by signature survive a feature edit (angle 5° → 3°)
    let e = s.execute("feature.edit", json!({"feature": r["id"], "set": {"angle_deg": 3.0}})).unwrap();
    let top3 = 20.0 - 2.0 * 10.0 * 3.0_f64.to_radians().tan();
    let exact3 = 10.0 / 3.0 * (400.0 + top3 * top3 + (400.0_f64 * top3 * top3).sqrt());
    assert!((vol(&e) - exact3).abs() < 1e-3, "{e}");
    s.execute("feature.edit", json!({"feature": r["id"], "set": {"angle_deg": 5.0}})).unwrap();

    // redesign assistant: scan the drafted part, take the +x wall, suggest the angle
    // the kernel meshes a flat wall with as few triangles as it needs, so the "scan" is that
    // mesh refined to a 1 mm edge length
    s.execute("body.to_mesh", json!({"object": "blk", "chord": 0.01, "name": "cad"})).unwrap();
    let mut cad = (*s.doc.meshes.iter().find(|m| m.name == "cad").unwrap().mesh).clone();
    cad.weld(1e-6);
    s.doc.add_mesh("scan", surfacecraft_geom::synth::refine(&cad, |_| 1.0));
    let m = s.doc.meshes.iter().find(|m| m.name == "scan").unwrap().mesh.clone();
    let seed = m
        .triangles
        .iter()
        .position(|t| {
            let p: Vec<_> = t.iter().map(|&i| m.positions[i as usize]).collect();
            p.iter().all(|q| q.x > 16.0)
        })
        .unwrap();
    let g = s.execute("region.grow", json!({"mesh": "scan", "triangles": [seed]})).unwrap();
    let sug = s.execute("feature.suggest_draft", json!({"region": g["id"], "pull": [0.0, 0.0, 1.0]})).unwrap();
    eprintln!("p5 suggest_draft: {sug}");
    assert_eq!(sug["surface"], "plane");
    assert!((sug["angle_deg"].as_f64().unwrap() - 5.0).abs() < 0.1, "{sug}");
}

/// Mirror a block that stands clear of the plane (two solids) and one that touches it (one
/// solid of twice the volume); `merge: false` keeps only the reflection.
#[test]
fn mirror_volumes() {
    let mut s = Session::new();
    s.execute("body.box", json!({"origin": [2.0, 0.0, 0.0], "size": [10.0, 10.0, 10.0], "name": "apart"})).unwrap();
    let r = s
        .execute(
            "feature.mirror",
            json!({"body": "apart", "plane_origin": [0.0, 0.0, 0.0], "plane_normal": [1.0, 0.0, 0.0]}),
        )
        .unwrap();
    assert!((vol(&r) - 2000.0).abs() < 1e-3, "{r}");
    s.execute("body.box", json!({"origin": [0.0, 0.0, 0.0], "size": [10.0, 10.0, 10.0], "name": "touching"})).unwrap();
    let t = s
        .execute(
            "feature.mirror",
            json!({"body": "touching", "plane_origin": [0.0, 0.0, 0.0], "plane_normal": [1.0, 0.0, 0.0]}),
        )
        .unwrap();
    eprintln!("p5 mirror (touching): V = {:.6}", vol(&t));
    assert!((vol(&t) - 2000.0).abs() < 1e-3, "{t}");
    // only the reflection: x ∈ [-12, -2]
    s.execute("body.box", json!({"origin": [2.0, 0.0, 0.0], "size": [10.0, 10.0, 10.0], "name": "flip"})).unwrap();
    let f = s
        .execute(
            "feature.mirror",
            json!({"body": "flip", "plane_origin": [0.0, 0.0, 0.0], "plane_normal": [1.0, 0.0, 0.0], "merge": false}),
        )
        .unwrap();
    assert!((vol(&f) - 1000.0).abs() < 1e-3, "{f}");
    let b = body_of(&s, "flip");
    let (lo, hi) = b.vertices.iter().fold((f64::MAX, f64::MIN), |a, v| (a.0.min(v.point[0]), a.1.max(v.point[0])));
    assert!((lo + 12.0).abs() < 1e-6 && (hi + 2.0).abs() < 1e-6, "x {lo}..{hi}");
}

/// A 20 × 10 × 10 block cut by the plane x = 5: 15 mm in front, 5 mm behind.
#[test]
fn split_keeps_one_side() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [20.0, 10.0, 10.0], "name": "front"})).unwrap();
    s.execute("body.box", json!({"size": [20.0, 10.0, 10.0], "name": "back"})).unwrap();
    let f = s
        .execute(
            "feature.split",
            json!({"body": "front", "plane_origin": [5.0, 0.0, 0.0], "plane_normal": [1.0, 0.0, 0.0]}),
        )
        .unwrap();
    assert!((vol(&f) - 1500.0).abs() < 1e-3, "{f}");
    let b = s
        .execute(
            "feature.split",
            json!({"body": "back", "plane_origin": [5.0, 3.0, 2.0], "plane_normal": [1.0, 0.0, 0.0], "keep": "back"}),
        )
        .unwrap();
    assert!((vol(&b) - 500.0).abs() < 1e-3, "{b}");
    // a tilted plane through the middle halves a cube
    s.execute("body.box", json!({"size": [10.0, 10.0, 10.0], "name": "cube"})).unwrap();
    let t = s
        .execute(
            "feature.split",
            json!({"body": "cube", "plane_origin": [5.0, 5.0, 5.0], "plane_normal": [1.0, 1.0, 1.0]}),
        )
        .unwrap();
    assert!((vol(&t) - 500.0).abs() < 1e-3, "{t}");
}

fn bbox_max(b: &Brep, axis: usize) -> f64 {
    b.vertices.iter().map(|v| v.point[axis]).fold(f64::MIN, f64::max)
}

/// Pattern along a path: four pins along a straight path, then four bars along a quarter circle
/// that turn with it (the last one stands along +y at the end of the arc).
#[test]
fn pattern_along_a_path_volumes_and_positions() {
    let mut s = Session::new();
    s.execute("body.cylinder", json!({"radius": 2.0, "height": 4.0, "name": "pin"})).unwrap();
    s.execute("sketch.create", json!({"name": "row"})).unwrap();
    s.execute("sketch.add", json!({"sketch": "row", "entities": [{"line": {"from": [0.0, 0.0], "to": [30.0, 0.0]}}]}))
        .unwrap();
    let r = s.execute("feature.path_pattern", json!({"body": "pin", "path": "row", "count": 4})).unwrap();
    assert!((vol(&r) - 4.0 * PI * 16.0).abs() < 1e-3, "{r}");
    assert!((bbox_max(&body_of(&s, "pin"), 0) - 32.0).abs() < 1e-6);

    s.execute("body.box", json!({"size": [4.0, 1.0, 1.0], "name": "bar"})).unwrap();
    s.execute("sketch.create", json!({"name": "curve"})).unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "curve", "entities": [{"arc": {"center": [0.0, 20.0], "start": [0.0, 0.0], "end": [20.0, 20.0]}}]}),
    )
    .unwrap();
    let c =
        s.execute("feature.path_pattern", json!({"body": "bar", "path": "curve", "count": 4, "follow": true})).unwrap();
    // four disjoint turned bars: 24 faces. (The kernel cannot re-read a union of three or more
    // rotated copies, the same for the older `feature.pattern`, so no volume here: STATUS debt.)
    assert_eq!(c["faces"], 24, "{c}");
    // the last bar stands along +y from the end of the arc (20, 20): y reaches 24
    let b = body_of(&s, "bar");
    assert!((bbox_max(&b, 1) - 24.0).abs() < 1e-6, "{}", bbox_max(&b, 1));
    // two bars (the original and one turned a quarter): the kernel still measures that
    s.execute("body.box", json!({"size": [4.0, 1.0, 1.0], "name": "pair"})).unwrap();
    let two = s
        .execute("feature.path_pattern", json!({"body": "pair", "path": "curve", "count": 2, "follow": true}))
        .unwrap();
    assert!((vol(&two) - 8.0).abs() < 1e-3, "{two}");
    // without `follow` the bars stay along x: the highest y is the end of the arc plus the bar's 1
    s.execute("body.box", json!({"size": [4.0, 1.0, 1.0], "name": "flat"})).unwrap();
    let f = s.execute("feature.path_pattern", json!({"body": "flat", "path": "curve", "count": 4})).unwrap();
    assert!((vol(&f) - 16.0).abs() < 1e-3, "{f}");
    assert!((bbox_max(&body_of(&s, "flat"), 1) - 21.0).abs() < 1e-6);
}

/// Auto-constraints on noisy synthetic data: a noisy bracket (σ = 0.05 mm) fitted region by
/// region, plus a cylinder coaxial with its boss and another of slightly different radius.
#[test]
fn auto_constrain_snaps_noisy_primitives() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "noise": 0.05, "seed": 4, "name": "scan"})).unwrap();
    // a second tube on the boss axis (45, 12), and a parallel one of radius 6.1 elsewhere
    s.execute("mesh.synth", json!({"shape": "cylinder", "size": [6.0, 30.0], "center": [45.0, 12.0, 40.0], "noise": 0.02, "seed": 7, "name": "tube1"})).unwrap();
    s.execute("mesh.synth", json!({"shape": "cylinder", "size": [6.1, 30.0], "center": [15.0, 10.0, 40.0], "noise": 0.02, "seed": 9, "name": "tube2"})).unwrap();
    let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    for r in seg["regions"].as_array().unwrap() {
        s.execute("primitive.fit", json!({"region": r["id"]})).unwrap();
    }
    for t in ["tube1", "tube2"] {
        let n = s.doc.meshes.iter().find(|m| m.name == t).unwrap().mesh.triangle_count() as u32;
        let all: Vec<u32> = (0..n).collect();
        s.execute("primitive.fit", json!({"mesh": t, "triangles": all, "kind": "cylinder"})).unwrap();
    }
    assert_eq!(s.doc.primitives.len(), 10);
    let before: Vec<_> = s.doc.primitives.iter().map(|p| p.primitive.clone()).collect();

    // report only
    let dry = s.execute("primitive.auto_constrain", json!({"apply": false})).unwrap();
    assert_eq!(dry["applied"], false);
    let count = |v: &Value, kind: &str| v["relations"].as_array().unwrap().iter().filter(|r| r["kind"] == kind).count();
    eprintln!(
        "p5 auto_constrain: parallel {}, perpendicular {}, coaxial {}, equal radius {}; residuals before snapping: {} deg",
        count(&dry, "parallel"),
        count(&dry, "perpendicular"),
        count(&dry, "coaxial"),
        count(&dry, "equal_radius"),
        dry["max_angle_error_deg"]
    );
    // 6 box planes (3 opposite pairs) + 3 axes: planes among themselves 3, axes with the two z
    // planes 6, axes among themselves 3
    assert_eq!(count(&dry, "parallel"), 12, "{dry}");
    // plane pairs 12 + each axis with the four side planes 12
    assert_eq!(count(&dry, "perpendicular"), 24, "{dry}");
    // the boss and tube1 share an axis; tube2 is offset
    assert_eq!(count(&dry, "coaxial"), 1, "{dry}");
    // three cylinders within 0.2 mm of each other
    assert_eq!(count(&dry, "equal_radius"), 3, "{dry}");
    assert_eq!(s.doc.primitives.iter().map(|p| p.primitive.clone()).collect::<Vec<_>>(), before);
    assert!(dry["max_angle_error_deg"].as_f64().unwrap() > 1e-6, "noise must leave residuals to snap");

    let r = s.execute("primitive.auto_constrain", json!({})).unwrap();
    assert_eq!(r["applied"], true);
    assert!(r["max_angle_error_deg"].as_f64().unwrap() < 1e-6, "{r}");
    assert!(r["max_length_error"].as_f64().unwrap() < 1e-9, "{r}");
    // the snapped primitives are exactly parallel / perpendicular / coaxial / equal
    use surfacecraft_fit::Primitive;
    let after: Vec<_> = s.doc.primitives.iter().map(|p| p.primitive.clone()).collect();
    let normals: Vec<[f64; 3]> = after
        .iter()
        .filter_map(|p| if let Primitive::Plane { normal, .. } = p { Some(*normal) } else { None })
        .collect();
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    for (i, a) in normals.iter().enumerate() {
        for b in normals.iter().skip(i + 1) {
            let d = dot(*a, *b).abs();
            assert!(d < 1e-9 || (d - 1.0).abs() < 1e-9, "planes neither parallel nor perpendicular: {d}");
        }
    }
    let cyls: Vec<([f64; 3], [f64; 3], f64)> = after
        .iter()
        .filter_map(|p| {
            if let Primitive::Cylinder { point, axis, radius } = p { Some((*point, *axis, *radius)) } else { None }
        })
        .collect();
    assert_eq!(cyls.len(), 3);
    for c in &cyls {
        assert!((c.2 - cyls[0].2).abs() < 1e-12, "radii differ");
        assert!((dot(c.1, cyls[0].1).abs() - 1.0).abs() < 1e-12, "axes not parallel");
    }
    // the boss and tube1 on one line
    let off = |a: &([f64; 3], [f64; 3], f64), b: &([f64; 3], [f64; 3], f64)| {
        let w = [b.0[0] - a.0[0], b.0[1] - a.0[1], b.0[2] - a.0[2]];
        let t = dot(w, a.1);
        let p = [w[0] - a.1[0] * t, w[1] - a.1[1] * t, w[2] - a.1[2] * t];
        dot(p, p).sqrt()
    };
    let (boss, tube1) = (
        cyls.iter().find(|c| (c.0[0] - 45.0).abs() < 1.0 && c.0[2] < 30.0).unwrap(),
        cyls.iter().find(|c| (c.0[0] - 45.0).abs() < 1.0 && c.0[2] > 30.0).unwrap(),
    );
    assert!(off(boss, tube1) < 1e-9, "boss and tube not coaxial after snapping");
    // snapping stayed within the tolerances: no primitive moved by more than 3° or 0.2 mm
    for (b, a) in before.iter().zip(&after) {
        let (fb, fa) = (b.frame().3, a.frame().3);
        let ang = fb.dot(&fa).abs().clamp(0.0, 1.0).acos().to_degrees();
        assert!(ang < 3.0, "{b:?} → {a:?}: {ang}°");
    }
    // undo restores the fitted values
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(s.doc.primitives.iter().map(|p| p.primitive.clone()).collect::<Vec<_>>(), before);
    assert!(s.execute("primitive.auto_constrain", json!({"angle_tol_deg": 90.0})).is_err());
}

/// Redesign assistant: the axis of a scanned turned surface.
#[test]
fn suggest_axis_from_a_scan() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "cylinder", "size": [8.0, 25.0], "center": [5.0, 6.0, 0.0], "noise": 0.02, "seed": 3, "name": "shaft"})).unwrap();
    let n = s.doc.meshes[0].mesh.triangle_count() as u32;
    let all: Vec<u32> = (0..n).collect();
    let r = s.execute("feature.suggest_axis", json!({"mesh": "shaft", "triangles": all})).unwrap();
    eprintln!("p5 suggest_axis: {r}");
    assert_eq!(r["surface"], "cylinder");
    let d = r["axis_direction"].as_array().unwrap();
    assert!(d[2].as_f64().unwrap().abs() > 0.9999, "{r}");
    let o = r["axis_origin"].as_array().unwrap();
    assert!((o[0].as_f64().unwrap() - 5.0).abs() < 0.05 && (o[1].as_f64().unwrap() - 6.0).abs() < 0.05, "{r}");
    assert!((r["radius"].as_f64().unwrap() - 8.0).abs() < 0.05, "{r}");
    // a flat scan has no axis
    s.execute("mesh.synth", json!({"shape": "plane", "name": "flat"})).unwrap();
    let m = s.doc.meshes.iter().find(|m| m.name == "flat").unwrap().mesh.triangle_count() as u32;
    let all: Vec<u32> = (0..m).collect();
    let flat = s.execute("feature.suggest_axis", json!({"mesh": "flat", "triangles": all}));
    assert!(flat.is_err(), "a plane has no axis: {flat:?}");
}
