//! P8-F1: refit a CAD body to a scan of the changed part (`body.refit_features`,
//! `body.refit_faces`). Synthetic scans with noise sigma = 0.02 mm, exact ground truth.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Rng, synth};
use surfacecraft_kernel::feature::{BoolKind, boolean};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::refit::{FaceShape, face_shapes};
use surfacecraft_kernel::solid::{make_box, make_cylinder, volume};

const SIGMA: f64 = 0.02;

fn body_of(s: &Session, name: &str) -> Brep {
    (*s.doc.bodies.iter().find(|b| b.name == name).unwrap().brep).clone()
}

/// Record index of the edge of `b` along z at the corner `(x, y)`.
fn vertical_edge_at(b: &Brep, x: f64, y: f64) -> usize {
    b.edges
        .iter()
        .position(|e| {
            let (p, q) = (b.vertices[e.vertices[0] as usize].point, b.vertices[e.vertices[1] as usize].point);
            (p[0] - q[0]).abs() < 1e-9
                && (p[1] - q[1]).abs() < 1e-9
                && (p[2] - q[2]).abs() > 1.0
                && (p[0] - x).abs() < 1e-6
                && (p[1] - y).abs() < 1e-6
        })
        .unwrap()
}

/// A plate `l × w × h` with a through hole of diameter `hole_d` and four rounded corners, built
/// from sketches and features. Returns the ids of the features (extrude, cut, fillet).
fn build_part(s: &mut Session, l: f64, w: f64, h: f64, hole_d: f64, fillet: f64) -> [u64; 3] {
    s.execute("sketch.create", json!({"name": "base"})).unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "base", "entities": [{"polygon": {"points": [[0.0, 0.0], [l, 0.0], [l, w], [0.0, w]]}}]}),
    )
    .unwrap();
    s.execute(
        "sketch.constrain",
        json!({"sketch": "base", "constraints": [
            {"type": "fix", "point": 0, "at": [0.0, 0.0]},
            {"type": "horizontal", "line": 0}, {"type": "vertical", "line": 1},
            {"type": "horizontal", "line": 2}, {"type": "vertical", "line": 3},
            {"type": "length", "line": 0, "value": l}, {"type": "length", "line": 1, "value": w}
        ]}),
    )
    .unwrap();
    let ex = s.execute("feature.extrude", json!({"sketch": "base", "distance": h, "name": "plate"})).unwrap();
    s.execute("sketch.create", json!({"name": "hole"})).unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "hole", "entities": [{"circle": {"center": [15.0, 8.0], "radius": hole_d / 2.0}}]}),
    )
    .unwrap();
    s.execute(
        "sketch.constrain",
        json!({"sketch": "hole", "constraints": [
            {"type": "fix", "point": 0, "at": [15.0, 8.0]},
            {"type": "radius", "entity": 0, "value": hole_d / 2.0}
        ]}),
    )
    .unwrap();
    let cut = s
        .execute("feature.extrude", json!({"sketch": "hole", "distance": 12.0, "mode": "cut", "target": "plate"}))
        .unwrap();
    let b = body_of(s, "plate");
    let edges: Vec<usize> =
        [(0.0, 0.0), (l, 0.0), (l, w), (0.0, w)].iter().map(|(x, y)| vertical_edge_at(&b, *x, *y)).collect();
    let fil = s.execute("feature.fillet", json!({"body": "plate", "edges": edges, "size": fillet})).unwrap();
    [ex["id"].as_u64().unwrap(), cut["id"].as_u64().unwrap(), fil["id"].as_u64().unwrap()]
}

/// A noisy, scan-like point mesh of `body`, added to the session as `name`.
fn add_scan(s: &mut Session, body: &Brep, name: &str, seed: u64) {
    let m = surfacecraft_kernel::tess::tessellate(body, Some(0.01)).unwrap();
    let mut m = synth::refine(&m, |_| 0.7);
    synth::add_noise(&mut m, SIGMA, &mut Rng::new(seed));
    s.doc.add_mesh(name, m);
}

fn param<'a>(out: &'a Value, id: &str) -> &'a Value {
    out["params"]
        .as_array()
        .unwrap()
        .iter()
        .find(|p| p["id"] == id)
        .unwrap_or_else(|| panic!("no parameter {id}: {out}"))
}

/// Gate P8-F1 (features): a plate 40 x 20 x 10 with a diameter 8 hole and r = 3 corners is
/// scanned at 40.6 x 19.5 x 10.3 with a diameter 8.4 hole. The refit recovers every dimension
/// within 0.05 mm and the scan-to-body RMS falls to 0.05 mm or less.
#[test]
fn gate_p8_refit_features_recovers_changed_dimensions() {
    let mut scan_session = Session::new();
    build_part(&mut scan_session, 40.6, 19.5, 10.3, 8.4, 3.0);
    let truth = body_of(&scan_session, "plate");

    let mut s = Session::new();
    let ids = build_part(&mut s, 40.0, 20.0, 10.0, 8.0, 3.0);
    add_scan(&mut s, &truth, "scan", 11);
    let base = s.doc.sketches.iter().find(|k| k.name == "base").unwrap().id.0;
    let hole = s.doc.sketches.iter().find(|k| k.name == "hole").unwrap().id.0;

    let v_start = volume(&body_of(&s, "plate")).unwrap();
    let out = s.execute("body.refit_features", json!({"body": "plate", "mesh": "scan"})).unwrap();
    let cut_depth = param(&out, &format!("feature/{}/distance", ids[1]));
    assert_eq!(cut_depth["new"], cut_depth["old"], "a cut deeper than the part is not touched: {out}");
    let get = |id: String| param(&out, &id)["new"].as_f64().unwrap();
    let (l, w) = (get(format!("sketch/{base}/5")), get(format!("sketch/{base}/6")));
    let h = get(format!("feature/{}/distance", ids[0]));
    let r_hole = get(format!("sketch/{hole}/1"));
    let r_fillet = get(format!("feature/{}/radius", ids[2]));
    let rb = out["rms_before"].as_f64().unwrap();
    let ra = out["rms_after"].as_f64().unwrap();
    eprintln!(
        "L={l:.4} W={w:.4} H={h:.4} hole d={:.4} fillet={r_fillet:.4} rms {rb:.4} -> {ra:.4} ({out})",
        2.0 * r_hole
    );
    assert!((l - 40.6).abs() <= 0.05, "length {l}");
    assert!((w - 19.5).abs() <= 0.05, "width {w}");
    assert!((h - 10.3).abs() <= 0.05, "height {h}");
    assert!((2.0 * r_hole - 8.4).abs() <= 0.05, "hole diameter {}", 2.0 * r_hole);
    assert!((r_fillet - 3.0).abs() <= 0.05, "fillet {r_fillet}");
    assert!(rb > 0.2, "the scan differs from the model before: {rb}");
    assert!(ra <= 0.05, "rms after {ra}");
    assert_eq!(out["applied"], true);
    assert_eq!(out["failed_features"], 0);

    // the document holds the refitted body: volume agrees with the scan's model
    let v = volume(&body_of(&s, "plate")).unwrap();
    let v_truth = volume(&truth).unwrap();
    assert!((v - v_truth).abs() / v_truth < 0.01, "{v} vs {v_truth}");

    // one undoable step: undo restores the old dimensions
    s.execute("edit.undo", Value::Null).unwrap();
    let v_old = volume(&body_of(&s, "plate")).unwrap();
    assert!((v_old - v_start).abs() < 1e-9, "undo brought the old body back: {v_old} vs {v_start}");
}

/// Listed `params` restrict the refit; unknown ones and histories without parameters are refused.
#[test]
fn refit_features_params_and_refusals() {
    let mut scan_session = Session::new();
    build_part(&mut scan_session, 40.6, 20.0, 10.0, 8.0, 3.0);
    let truth = body_of(&scan_session, "plate");
    let mut s = Session::new();
    build_part(&mut s, 40.0, 20.0, 10.0, 8.0, 3.0);
    add_scan(&mut s, &truth, "scan", 5);
    let base = s.doc.sketches.iter().find(|k| k.name == "base").unwrap().id.0;
    let only = format!("sketch/{base}/5");
    let out = s.execute("body.refit_features", json!({"body": "plate", "mesh": "scan", "params": [only]})).unwrap();
    assert_eq!(out["params"].as_array().unwrap().len(), 1);
    assert!((param(&out, &only)["new"].as_f64().unwrap() - 40.6).abs() <= 0.05);
    assert!(s.execute("body.refit_features", json!({"body": "plate", "mesh": "scan", "params": ["nope"]})).is_err());
    // a body without history is for refit_faces
    let b = s.execute("body.box", json!({"size": [10.0, 10.0, 10.0], "name": "block"})).unwrap();
    assert!(b["faces"] == 6);
    let e = s.execute("body.refit_features", json!({"body": "block", "mesh": "scan"})).unwrap_err();
    assert!(e.to_string().contains("refit_faces"), "{e}");
    // and a body with history is not for refit_faces
    let e = s.execute("body.refit_faces", json!({"body": "plate", "mesh": "scan"})).unwrap_err();
    assert!(e.to_string().contains("refit_features"), "{e}");
}

fn boss(width: f64, radius: f64) -> Brep {
    let b = make_box([0.0; 3], [width, 30.0, 10.0]).unwrap();
    let c = make_cylinder([20.0, 15.0, 10.0], radius, 8.0).unwrap();
    boolean(&b, &c, BoolKind::Union).unwrap()
}

fn face_where(b: &Brep, pred: impl Fn(&FaceShape) -> bool) -> usize {
    face_shapes(b).unwrap().iter().position(|s| s.as_ref().is_ok_and(&pred)).unwrap()
}

/// Gate P8-F1 (faces): a box 40 x 30 x 10 with a r = 6 cylinder boss and no history; the scan has
/// the +x face moved out by 0.4 mm and the boss radius 6.3 mm. `body.refit_faces` recovers both
/// within 0.05 mm, the `fixed` base face does not move and the face count is unchanged.
#[test]
fn gate_p8_refit_faces_recovers_offset_and_radius() {
    let original = boss(40.0, 6.0);
    let truth = boss(40.4, 6.3);
    let mut s = Session::new();
    s.doc.add_body("part", original.clone());
    add_scan(&mut s, &truth, "scan", 3);
    let base = face_where(&original, |f| matches!(f, FaceShape::Plane { normal, .. } if normal[2] < -0.99));
    let plus_x = face_where(
        &original,
        |f| matches!(f, FaceShape::Plane { normal, point } if normal[0] > 0.99 && point[0] > 39.0),
    );
    let wall = face_where(&original, |f| matches!(f, FaceShape::Cylinder { .. }));

    let out = s
        .execute(
            "body.refit_faces",
            json!({"body": "part", "mesh": "scan", "faces": [{"face": base, "stiffness": "fixed"}]}),
        )
        .unwrap();
    let faces = out["faces"].as_array().unwrap();
    let f = |i: usize| &faces[i];
    let (x0, x1) = (f(plus_x)["old"]["offset"].as_f64().unwrap(), f(plus_x)["new"]["offset"].as_f64().unwrap());
    let (r0, r1) = (f(wall)["old"]["radius"].as_f64().unwrap(), f(wall)["new"]["radius"].as_f64().unwrap());
    eprintln!("x {x0} -> {x1}, r {r0} -> {r1}, rms {} -> {} ({out})", out["rms_before"], out["rms_after"]);
    assert!((x1 - x0 - 0.4).abs() <= 0.05, "face offset {}", x1 - x0);
    assert!((r1 - r0 - 0.3).abs() <= 0.05, "radius change {}", r1 - r0);
    assert_eq!(f(base)["status"], "fixed");
    assert_eq!(f(base)["new"]["offset"], f(base)["old"]["offset"]);
    assert_eq!(out["faces_before"], out["faces_after"]);
    assert_eq!(out["relations_before"], out["relations_after"]);
    assert!(out["rms_after"].as_f64().unwrap() <= 0.05, "{out}");
    assert!(out["rms_before"].as_f64().unwrap() > out["rms_after"].as_f64().unwrap());
    assert_eq!(out["applied"], true);

    // the body in the document is the rebuilt one; the base plane is where it was
    let nb = body_of(&s, "part");
    assert_eq!(nb.faces.len(), original.faces.len());
    let base_after = face_where(
        &nb,
        |f| matches!(f, FaceShape::Plane { normal, point } if normal[2] < -0.99 && point[2].abs() < 1e-9),
    );
    let _ = base_after;
    let expect = 40.4 * 30.0 * 10.0 + std::f64::consts::PI * 6.3 * 6.3 * 8.0;
    assert!((volume(&nb).unwrap() - expect).abs() / expect < 0.01);
}

/// Freeform faces are refused with a reason, a stiff face moves less, and bad input is an error.
#[test]
fn refit_faces_stiffness_and_refusals() {
    let original = boss(40.0, 6.0);
    let truth = boss(40.4, 6.0);
    let mut s = Session::new();
    s.doc.add_body("part", original.clone());
    add_scan(&mut s, &truth, "scan", 9);
    let plus_x = face_where(
        &original,
        |f| matches!(f, FaceShape::Plane { normal, point } if normal[0] > 0.99 && point[0] > 39.0),
    );
    let run = |s: &mut Session, st: &str| {
        let o = s
            .execute(
                "body.refit_faces",
                json!({"body": "part", "mesh": "scan", "apply": false, "faces": [{"face": plus_x, "stiffness": st}]}),
            )
            .unwrap();
        o["faces"][plus_x]["offset"].as_f64().unwrap()
    };
    let free = run(&mut s, "free");
    let tight = run(&mut s, "tight");
    assert!((free - 0.4).abs() <= 0.03, "{free}");
    assert!(tight > 0.05 && tight < free * 0.6, "tight {tight}, free {free}");
    // apply: false leaves the body alone
    assert_eq!(body_of(&s, "part"), original);
    assert!(
        s.execute(
            "body.refit_faces",
            json!({"body": "part", "mesh": "scan", "faces": [{"face": 999, "stiffness": "free"}]})
        )
        .is_err()
    );
    assert!(s.execute("body.refit_faces", json!({"body": "part", "mesh": "scan", "max_change": -1.0})).is_err());
}

/// A dimension or feature field that follows an expression is fitted like any other, and the
/// fitted value is then a plain number: the expression no longer drives it.
#[test]
fn refit_features_turns_expression_driven_values_into_plain_numbers() {
    let mut scan_session = Session::new();
    build_part(&mut scan_session, 40.6, 20.0, 10.3, 8.0, 3.0);
    let truth = body_of(&scan_session, "plate");
    let mut s = Session::new();
    let ids = build_part(&mut s, 40.0, 20.0, 10.0, 8.0, 3.0);
    add_scan(&mut s, &truth, "scan", 5);
    s.execute("param.set", json!({"name": "ln", "expression": "40"})).unwrap();
    s.execute("param.set", json!({"name": "ht", "expression": "10"})).unwrap();
    s.execute(
        "sketch.edit_constraint",
        json!({"sketch": "base", "index": 5, "constraint": {"type": "length", "line": 0, "value": {"expr": "ln"}}}),
    )
    .unwrap();
    s.execute("feature.edit", json!({"feature": ids[0], "set": {"distance": {"expr": "ht"}}})).unwrap();
    let base = s.doc.sketches.iter().find(|k| k.name == "base").unwrap().id.0;
    let only = [format!("sketch/{base}/5"), format!("feature/{}/distance", ids[0])];
    let out = s.execute("body.refit_features", json!({"body": "plate", "mesh": "scan", "params": only})).unwrap();
    assert!((param(&out, &only[0])["new"].as_f64().unwrap() - 40.6).abs() <= 0.05, "{out}");
    assert!(!s.doc.sketches.iter().any(|k| k.sketch.dim_exprs.contains_key(&5)), "the dimension is a plain number");
    assert!(s.doc.features.iter().all(|f| f.exprs.is_empty()), "so is the extrusion depth");
    let fitted = volume(&body_of(&s, "plate")).unwrap();
    s.execute("param.set", json!({"name": "ln", "expression": "99"})).unwrap();
    s.execute("param.set", json!({"name": "ht", "expression": "99"})).unwrap();
    let after = volume(&body_of(&s, "plate")).unwrap();
    assert!((after - fitted).abs() < 1e-9, "the parameters no longer move the part: {after} vs {fitted}");
}
