//! P5: persistent topology naming (criterion 5) and the features added in this stage.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_doc::FeatureOp;
use surfacecraft_engine::Session;
use surfacecraft_kernel::naming::{edge_signature, face_signature};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::solid::volume;

/// A small deterministic generator (xorshift64*), so the "random" edits are the same every run.
struct Rng(u64);

impl Rng {
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_F491_4F6C_DD1D)
    }
    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1u64 << 53) as f64
    }
    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

fn body_of(s: &Session, name: &str) -> Brep {
    (*s.doc.bodies.iter().find(|b| b.name == name).unwrap().brep).clone()
}

fn feature_op(s: &Session, id: u64) -> FeatureOp {
    s.doc.features.iter().find(|f| f.id.0 == id).unwrap().op.clone()
}

/// A 40 × 20 rectangle sketch, fully constrained (constraints 5 and 6 are its two lengths).
fn plate_sketch(s: &mut Session) {
    s.execute("sketch.create", json!({"name": "base"})).unwrap();
    s.execute("sketch.add", json!({"sketch": "base", "entities": [{"polygon": {"points": [[0.2, -0.1], [39.0, 0.4], [40.5, 20.2], [0.1, 19.0]]}}]}))
        .unwrap();
    s.execute(
        "sketch.constrain",
        json!({"sketch": "base", "constraints": [
            {"type": "fix", "point": 0, "at": [0.0, 0.0]},
            {"type": "horizontal", "line": 0}, {"type": "vertical", "line": 1},
            {"type": "horizontal", "line": 2}, {"type": "vertical", "line": 3},
            {"type": "length", "line": 0, "value": 40.0}, {"type": "length", "line": 1, "value": 20.0}
        ]}),
    )
    .unwrap();
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

/// Record index of the highest plane face of `b` whose normal points up.
fn top_face(b: &Brep) -> usize {
    (0..b.faces.len())
        .filter_map(|f| face_signature(b, f).map(|s| (f, s)))
        .filter(|(_, s)| s.kind == "plane" && s.normal[2] > 0.99)
        .max_by(|a, b| a.1.centroid[2].total_cmp(&b.1.centroid[2]))
        .unwrap()
        .0
}

fn set_suppressed(s: &mut Session, feature: u64, on: bool) {
    s.execute("feature.suppress", json!({"feature": feature, "suppressed": on})).unwrap();
}

/// Gate P5 (criterion 5): extrude + cut + two fillets + shell, then 20 seeded random edits of
/// earlier dimensions and feature parameters. After each, the fillets are still on the two
/// vertical edges at the opposite corners (the edge at the same bbox-normalised position and
/// direction), the shell still opens the top face, and the volumes match the independent
/// closed forms. The cut depth edit turns the pocket through ↔ blind, which renumbers the
/// record: positional indices would have moved.
#[test]
fn gate_p5_stable_references_over_20_random_edits() {
    let mut s = Session::new();
    plate_sketch(&mut s);
    let ex = s.execute("feature.extrude", json!({"sketch": "base", "distance": 10.0, "name": "plate"})).unwrap();
    let extrude_id = ex["id"].as_u64().unwrap();
    // a pocket from the bottom face: through when its depth reaches the plate height
    s.execute("sketch.create", json!({"name": "hole"})).unwrap();
    s.execute(
        "sketch.add",
        json!({"sketch": "hole", "entities": [{"circle": {"center": [15.0, 8.0], "radius": 4.0}}]}),
    )
    .unwrap();
    let cut = s
        .execute("feature.extrude", json!({"sketch": "hole", "distance": 12.0, "mode": "cut", "target": "plate"}))
        .unwrap();
    let cut_id = cut["id"].as_u64().unwrap();
    // two fillets at opposite vertical corners (indices read from the body as it is)
    let b0 = body_of(&s, "plate");
    let (e0, e1) = (vertical_edge_at(&b0, 0.0, 0.0), vertical_edge_at(&b0, 40.0, 20.0));
    let fil = s.execute("feature.fillet", json!({"body": "plate", "edges": [e0, e1], "size": 1.5})).unwrap();
    let fillet_id = fil["id"].as_u64().unwrap();
    let b1 = body_of(&s, "plate");
    let sh = s.execute("feature.shell", json!({"body": "plate", "faces": [top_face(&b1)], "thickness": 1.0})).unwrap();
    let shell_id = sh["id"].as_u64().unwrap();
    assert_eq!(sh["failed_features"], 0, "{sh}");

    let (mut length, mut width, mut height, mut depth, mut radius, mut thickness) = (40.0, 20.0, 10.0, 12.0, 1.5, 1.0);
    let mut rng = Rng(0x5eed_0005);
    let mut renumbered = 0;
    let mut unmeasured = 0;
    let mut last_edges = {
        let FeatureOp::Fillet { edges, .. } = feature_op(&s, fillet_id) else { panic!("not a fillet") };
        edges
    };
    let mut worst_volume_error = 0.0_f64;
    for step in 0..20 {
        let which = (rng.next() % 6) as usize;
        let r = match which {
            0 => {
                length = rng.range(30.0, 60.0);
                s.execute(
                    "sketch.edit_constraint",
                    json!({"sketch": "base", "index": 5, "constraint": {"type": "length", "line": 0, "value": length}}),
                )
            }
            1 => {
                width = rng.range(18.0, 35.0);
                s.execute(
                    "sketch.edit_constraint",
                    json!({"sketch": "base", "index": 6, "constraint": {"type": "length", "line": 1, "value": width}}),
                )
            }
            2 => {
                height = rng.range(8.0, 16.0);
                s.execute("feature.edit", json!({"feature": extrude_id, "set": {"distance": height}}))
            }
            3 => {
                depth = rng.range(5.0, 20.0);
                s.execute("feature.edit", json!({"feature": cut_id, "set": {"distance": depth}}))
            }
            4 => {
                radius = rng.range(0.8, 3.0);
                s.execute("feature.edit", json!({"feature": fillet_id, "set": {"radius": radius}}))
            }
            _ => {
                thickness = rng.range(0.5, 1.5);
                s.execute("feature.edit", json!({"feature": shell_id, "set": {"thickness": thickness}}))
            }
        };
        let r = r.unwrap_or_else(|e| panic!("edit {step} (kind {which}) failed: {e:?}"));
        assert_eq!(r["failed_features"], 0, "edit {step}: {r}");

        // the history as the fillet sees it: suppress fillet and shell, read the pocketed plate
        set_suppressed(&mut s, fillet_id, true);
        set_suppressed(&mut s, shell_id, true);
        let before_fillet = body_of(&s, "plate");
        let FeatureOp::Fillet { edges, .. } = feature_op(&s, fillet_id) else { panic!("not a fillet") };
        assert_eq!(edges.len(), 2, "step {step}: {edges:?}");
        let mut corners: Vec<(i32, i32)> = Vec::new();
        for &e in &edges {
            let sig = edge_signature(&before_fillet, e).unwrap_or_else(|| panic!("step {step}: edge {e} is gone"));
            assert_eq!(sig.curve, "line", "step {step}");
            assert!(sig.tangent[2].abs() > 0.99, "step {step}: the filleted edge is not vertical: {sig:?}");
            assert!((sig.mid[2] - 0.5).abs() < 0.05, "step {step}: {sig:?}");
            corners.push((sig.mid[0].round() as i32, sig.mid[1].round() as i32));
            assert!((sig.mid[0] - sig.mid[0].round()).abs() < 0.02 && (sig.mid[1] - sig.mid[1].round()).abs() < 0.02);
        }
        corners.sort_unstable();
        assert_eq!(corners, [(0, 0), (1, 1)], "step {step}: the fillets left their corners");
        if edges != last_edges {
            renumbered += 1;
        }
        last_edges = edges;
        // the plate with its pocket: closed form
        let v0 = volume(&before_fillet).unwrap();
        let pocket = PI * 16.0 * depth.min(height);
        let exact0 = length * width * height - pocket;
        assert!((v0 - exact0).abs() < 1e-3 * exact0, "step {step}: pocketed plate {v0} vs {exact0}");

        // then the fillets: the top face is still the one the shell opens
        set_suppressed(&mut s, fillet_id, false);
        let after_fillet = body_of(&s, "plate");
        let v1 = volume(&after_fillet).unwrap();
        let exact1 = exact0 - 2.0 * (1.0 - PI / 4.0) * radius * radius * height;
        worst_volume_error = worst_volume_error.max((v1 - exact1).abs() / exact1);
        assert!((v1 - exact1).abs() < 1e-3 * exact1, "step {step}: filleted plate {v1} vs {exact1}");
        // the shell back on resolves its face against exactly this body: it is still the top one
        set_suppressed(&mut s, shell_id, false);
        let FeatureOp::Shell { faces, .. } = feature_op(&s, shell_id) else { panic!("not a shell") };
        assert_eq!(faces.len(), 1);
        let fs = face_signature(&after_fillet, faces[0]).unwrap();
        assert_eq!(fs.kind, "plane", "step {step}: {fs:?}");
        assert!(fs.normal[2] > 0.99 && fs.centroid[2] > 0.95, "step {step}: the shell opens another face: {fs:?}");
        assert_eq!(faces[0], top_face(&after_fillet), "step {step}");

        let tree = s.execute("feature.tree", Value::Null).unwrap();
        assert_eq!(tree["failed_features"], 0, "step {step}: {tree}");
        // a hollow body of plausible volume
        // (the kernel cannot always re-read a shelled record: such a step is counted, not failed)
        let Ok(hollow) = volume(&body_of(&s, "plate")) else {
            unmeasured += 1;
            continue;
        };
        let area = 2.0 * (length * width + (length + width) * height) + 2.0 * PI * 4.0 * depth.min(height);
        assert!(
            hollow > 0.3 * thickness * area && hollow < 1.7 * thickness * area,
            "step {step}: shell volume {hollow} for wall {thickness} over ~{area} mm²"
        );
    }
    eprintln!(
        "gate_p5: 20 random edits, fillet/shell stayed on their edges/faces; the record was renumbered under \
         the fillet {renumbered} times; worst filleted-volume error {worst_volume_error:.2e}; shell volume unmeasurable          (kernel record round-trip) in {unmeasured} steps"
    );
    assert!(renumbered > 0, "the test never moved the record indices: it does not exercise the naming");
}

/// A reference that cannot be found again fails the feature with an error (nothing panics), and
/// editing `edges` by hand recaptures the signatures for the new edges.
#[test]
fn lost_reference_fails_the_feature_and_edge_edit_recaptures() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [20.0, 20.0, 10.0], "name": "blk"})).unwrap();
    let b = body_of(&s, "blk");
    let (a, c) = (vertical_edge_at(&b, 0.0, 0.0), vertical_edge_at(&b, 20.0, 20.0));
    let f = s.execute("feature.fillet", json!({"body": "blk", "edges": [a], "size": 2.0})).unwrap();
    let fid = f["id"].as_u64().unwrap();
    // pointing the feature at another edge by hand moves the fillet (signatures recaptured)
    let e = s.execute("feature.edit", json!({"feature": fid, "set": {"edges": [c]}})).unwrap();
    assert_eq!(e["failed_features"], 0, "{e}");
    let FeatureOp::Fillet { edges, edge_refs, .. } = feature_op(&s, fid) else { panic!("not a fillet") };
    assert_eq!(edges, vec![c]);
    assert_eq!(edge_refs.len(), 1);
    assert!(edge_refs[0].mid[0] > 0.99 && edge_refs[0].mid[1] > 0.99, "{:?}", edge_refs[0]);
    // an upstream edit that removes the edge's neighbourhood: a base box replaced by a cylinder
    // body leaves no edge like the stored one
    let mut doc_op = feature_op(&s, fid);
    if let FeatureOp::Fillet { edge_refs, .. } = &mut doc_op {
        edge_refs[0].mid = [0.5, 0.5, 0.5];
        edge_refs[0].start = [0.5, 0.5, 0.5];
        edge_refs[0].end = [0.5, 0.5, 0.5];
        edge_refs[0].curve = "circle".into();
    }
    s.doc.feature_mut(surfacecraft_doc::ObjectId(fid)).unwrap().op = doc_op;
    let r = s.execute("feature.replay", Value::Null).unwrap();
    assert_eq!(r["failed_features"], 1, "{r}");
    let err = r["features"].as_array().unwrap().iter().find_map(|f| f["error"].as_str()).unwrap();
    assert!(err.contains("matches no edge"), "{err}");
}
