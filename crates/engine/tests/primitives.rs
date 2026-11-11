//! M2 through the command layer: regions, primitive fits, symmetry, deviation.
use serde_json::{Value, json};
use surfacecraft_engine::Session;

fn noisy_bracket(s: &mut Session) {
    s.execute("mesh.synth", json!({"shape": "bracket", "noise": 0.05, "seed": 4, "name": "scan"})).unwrap();
}

/// Gate M2: auto-segment of a noisy synthetic part gives the right kinds, every region fit
/// within RMS 0.1 mm, and primitives fitted to the regions agree.
#[test]
fn gate_m2_segment_and_fit_through_commands() {
    let mut s = Session::new();
    noisy_bracket(&mut s);
    let out = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let regions = out["regions"].as_array().unwrap().clone();
    let mut kinds: Vec<String> = regions.iter().map(|r| r["kind"].as_str().unwrap().to_string()).collect();
    kinds.sort();
    assert_eq!(kinds, ["cylinder", "plane", "plane", "plane", "plane", "plane", "plane", "sphere"]);
    for r in &regions {
        let f = s.execute("primitive.fit", json!({"region": r["id"]})).unwrap();
        assert!(f["rms"].as_f64().unwrap() <= 0.1, "{f}");
        assert_eq!(f["primitive"]["type"], r["kind"]);
    }
    // the cylinder's radius is the boss's 6 mm
    let cyl = regions.iter().find(|r| r["kind"] == "cylinder").unwrap();
    let f = s.execute("primitive.fit", json!({"region": cyl["id"], "kind": "cylinder"})).unwrap();
    assert!((f["primitive"]["radius"].as_f64().unwrap() - 6.0).abs() < 0.02, "{f}");
}

#[test]
fn merge_split_and_deviation() {
    let mut s = Session::new();
    noisy_bracket(&mut s);
    let out = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let planes: Vec<Value> =
        out["regions"].as_array().unwrap().iter().filter(|r| r["kind"] == "plane").map(|r| r["id"].clone()).collect();
    // merging two perpendicular faces gives a region that is no longer a plane
    let m = s.execute("region.merge", json!({"regions": [planes[0], planes[1]]})).unwrap();
    assert_ne!(m["kind"], "plane");
    // splitting into connected pieces keeps every triangle (one piece if the faces touch, two if not)
    let sp = s.execute("region.split", json!({"region": m["id"]})).unwrap();
    let pieces = sp["regions"].as_array().unwrap();
    assert!((1..=2).contains(&pieces.len()));
    let total: u64 = pieces.iter().map(|r| r["triangles"].as_u64().unwrap()).sum();
    assert_eq!(total, m["triangles"].as_u64().unwrap());
    if pieces.len() == 2 {
        assert!(pieces.iter().all(|r| r["kind"] == "plane"));
    }
    s.execute("edit.undo", Value::Null).unwrap();
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!(s.doc.regions.len(), 8);
    // deviation of a planar region from its plane
    let f = s.execute("primitive.fit", json!({"region": planes[2]})).unwrap();
    let d = s.execute("deviation.compute", json!({"object": "scan", "against": f["id"], "region": planes[2]})).unwrap();
    let rms = d["stats"]["rms"].as_f64().unwrap();
    assert!((rms - f["rms"].as_f64().unwrap()).abs() < 0.01, "{d}");
    assert!(d["stats"]["within"].as_f64().unwrap() > 0.9);
    assert_eq!(s.maps.len(), 1);
    let items = s.display_items();
    assert!(items.iter().any(|i| i.colors.is_some()));
    s.execute("deviation.clear", json!({"object": "scan"})).unwrap();
    let n = s.execute("deviation.noise", json!({"object": "scan"})).unwrap();
    let sigma = n["sigma"].as_f64().unwrap();
    assert!((0.03..0.08).contains(&sigma), "{n}");
}

#[test]
fn grow_from_seed_and_symmetry() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "box", "size": [40, 30, 20], "resolution": 16})).unwrap();
    let g = s.execute("region.grow", json!({"mesh": "box", "triangles": [0]})).unwrap();
    assert_eq!(g["kind"], "plane");
    let sym = s.execute("primitive.symmetry", json!({"mesh": "box"})).unwrap();
    assert!(sym["rms"].as_f64().unwrap() < 1e-3, "{sym}");
    let deleted = s.execute("doc.delete", json!({"object": "box"})).unwrap();
    assert!(deleted["id"].is_number());
    assert!(s.doc.regions.is_empty());
}

#[test]
fn explicit_kind_and_bad_requests() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "cone", "noise": 0.05, "name": "c"})).unwrap();
    let n = s.doc.meshes[0].mesh.triangle_count() as u32;
    let all: Vec<u32> = (0..n).collect();
    let f = s.execute("primitive.fit", json!({"mesh": "c", "triangles": all, "kind": "cone"})).unwrap();
    assert!(f["rms"].as_f64().unwrap() <= 0.1);
    assert!(s.execute("primitive.fit", json!({"mesh": "c"})).is_err());
    assert!(s.execute("region.create", json!({"mesh": "c", "triangles": [n + 5]})).is_err());
    assert!(s.execute("region.merge", json!({"regions": [1]})).is_err());
}
