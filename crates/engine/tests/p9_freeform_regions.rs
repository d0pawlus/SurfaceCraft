//! Freeform regions through the command layer (P9-O7): the flag set by `region.auto_segment`,
//! cleared when a primitive is fitted to the region, and kept by the project file.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, synth};

/// A bumpy closed sphere of about 25 k triangles that segments into one freeform region.
fn organic_scan() -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 20.0, 158, 79);
    for p in &mut m.positions {
        let d = p.coords / 20.0;
        let bump = 1.2 * (26.0 * d.x + 0.3).sin() * (20.0 * d.y + 1.1).sin() * (16.0 * d.z + 2.0).sin();
        *p = Point3::from(d * (20.0 + bump));
    }
    synth::add_noise(&mut m, 0.02, &mut Rng::new(11));
    m
}

fn freeform_region(s: &mut Session) -> Value {
    let out = s.execute("region.list", json!({})).unwrap();
    out["regions"].as_array().unwrap().iter().find(|r| r["freeform"] == true).cloned().unwrap_or(Value::Null)
}

/// Writes the project, opens it in a fresh session and returns the region `id` there.
fn through_a_file(s: &mut Session, name: &str, id: &Value) -> Value {
    let dir = std::env::temp_dir().join(format!("sc-p9-{}-{name}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let path = dir.join("p.scproj").to_string_lossy().to_string();
    s.execute("doc.save", json!({"path": path})).unwrap();
    let mut t = Session::new();
    t.execute("doc.open", json!({"path": path})).unwrap();
    let list = t.execute("region.list", json!({})).unwrap();
    let _ = std::fs::remove_dir_all(dir);
    list["regions"].as_array().unwrap().iter().find(|r| r["id"] == *id).cloned().unwrap()
}

#[test]
fn fitting_a_primitive_to_a_freeform_region_clears_the_flag_and_the_file_agrees() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", organic_scan());
    s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let region = freeform_region(&mut s);
    assert!(!region.is_null(), "no freeform region");
    assert!(region["kind"].is_null() && region["rms"].is_null(), "{region}");
    assert_eq!(through_a_file(&mut s, "before", &region["id"])["freeform"], true, "the file keeps the flag");

    s.execute("primitive.fit", json!({"region": region["id"], "kind": "plane"})).unwrap();

    let list = s.execute("region.list", json!({})).unwrap();
    let after = list["regions"].as_array().unwrap().iter().find(|r| r["id"] == region["id"]).unwrap().clone();
    assert_eq!(after["kind"], "plane", "{after}");
    assert_eq!(after["freeform"], false, "a region with a primitive kind is no freeform region: {after}");
    let reopened = through_a_file(&mut s, "after", &region["id"]);
    assert_eq!((reopened["kind"].clone(), reopened["freeform"].clone()), (json!("plane"), json!(false)), "{reopened}");
}
