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
        let bump = 1.2 * (9.0 * d.x + 0.3).sin() * (7.0 * d.y + 1.1).sin() * (6.0 * d.z + 2.0).sin();
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

// --- the scanner noise the consolidation judges fits by -------------------------------------------

/// A grid over `[0, w] x [0, h]` with vertices `f(x, y)`.
fn sheet(w: f64, h: f64, n: usize, f: impl Fn(f64, f64) -> Point3) -> Mesh {
    let cols = n + 1;
    let mut positions = Vec::new();
    for j in 0..=n {
        for i in 0..=n {
            positions.push(f(w * i as f64 / n as f64, h * j as f64 / n as f64));
        }
    }
    let mut triangles = Vec::new();
    for j in 0..n {
        for i in 0..n {
            let (a, b) = ((j * cols + i) as u32, (j * cols + i + 1) as u32);
            let (c, d) = (((j + 1) * cols + i + 1) as u32, ((j + 1) * cols + i) as u32);
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh { positions, triangles }
}

fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A planar face (r < 25 mm) in a sculpted surface that runs into it tangentially, sigma 0.03.
fn mounting_face_scan() -> Mesh {
    let mut m = sheet(120.0, 120.0, 150, |x, y| {
        let r = (x - 60.0).hypot(y - 60.0);
        let bump = 2.5 * (0.25 * x + 0.3).sin() * (0.2 * y + 1.1).sin();
        Point3::new(x, y, bump * smoothstep((r - 25.0) / 10.0))
    });
    synth::add_noise(&mut m, 0.03, &mut Rng::new(5));
    m
}

fn organic_scan_with_noise(sigma: f64) -> Mesh {
    let mut m = organic_scan();
    // organic_scan() carries its own noise of 0.02; this adds the rest of `sigma`
    synth::add_noise(&mut m, (sigma * sigma - 0.02 * 0.02).max(0.0).sqrt(), &mut Rng::new(12));
    m
}

/// The noise is measured on the scan (sigma 0.03 here), reported with its source, and with it the
/// fragments of an organic scan are consolidated (distribution (b): misfits above the noise).
#[test]
fn the_noise_is_estimated_from_the_scan_and_fragments_are_consolidated() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", organic_scan_with_noise(0.03));
    let out = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    assert_eq!(out["noise_source"], "estimated", "{out}");
    let sigma = out["noise_used"].as_f64().unwrap();
    assert!((0.015..0.06).contains(&sigma), "estimated noise {sigma}");
    assert!(out["regions"].as_array().unwrap().iter().any(|r| r["freeform"] == true), "{out}");
}

/// The caller's noise wins over the estimate; a nonsense value is refused.
#[test]
fn a_given_noise_is_used_and_reported_and_nonsense_is_refused() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", organic_scan());
    let out = s.execute("region.auto_segment", json!({"mesh": "scan", "options": {"noise": 0.011}})).unwrap();
    assert_eq!((out["noise_source"].clone(), out["noise_used"].clone()), (json!("given"), json!(0.011)), "{out}");
    for bad in [json!(0.0), json!(-1.0)] {
        let r = s.execute("region.auto_segment", json!({"mesh": "scan", "options": {"noise": bad}}));
        assert!(r.is_err(), "{bad} was accepted");
    }
    let off = s.execute("region.auto_segment", json!({"mesh": "scan", "options": {"freeform": false}})).unwrap();
    assert!(off.get("noise_source").is_none(), "no noise is reported when the consolidation is off: {off}");
}

/// A mesh with no reliable estimate (too few points) gets the documented fallback, a quarter of
/// the tolerance, and says so.
#[test]
fn without_a_reliable_estimate_the_fallback_is_reported() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", synth::cuboid(Point3::origin(), surfacecraft_geom::Vec3::new(10.0, 10.0, 10.0), 1));
    let out = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    assert_eq!((out["noise_source"].clone(), out["noise_used"].clone()), (json!("fallback"), json!(0.05)), "{out}");
}

/// Distribution (a) through the estimator: a planar mounting face that a sculpted surface runs
/// into tangentially keeps its primitive; the surround is consolidated.
#[test]
fn a_mounting_face_in_a_sculpted_surface_stays_a_plane_with_the_estimated_noise() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", mounting_face_scan());
    let out = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    assert_eq!(out["noise_source"], "estimated", "{out}");
    let regions = out["regions"].as_array().unwrap();
    let plane = regions.iter().find(|r| r["kind"] == "plane" && r["triangles"].as_u64().unwrap() > 4000);
    assert!(plane.is_some(), "the mounting face is no longer a plane: {out}");
    assert!(regions.iter().any(|r| r["freeform"] == true), "the surround is not consolidated: {out}");
}

/// `region.resegment` reports the noise too (measured on the whole mesh, not on the selection).
#[test]
fn resegment_reports_the_noise() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", organic_scan_with_noise(0.03));
    let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let ids: Vec<Value> = seg["regions"].as_array().unwrap().iter().map(|r| r["id"].clone()).collect();
    let out = s.execute("region.resegment", json!({"regions": ids})).unwrap();
    assert_eq!(out["noise_source"], "estimated", "{out}");
}

// --- the noise is measured where the segmentation looks -------------------------------------------

/// A flat plate of `side` mm (150 cells a side) at x = 200, with noise `sigma`: geometry unrelated to
/// the scan it is appended to.
fn unrelated_plate(sigma: f64, seed: u64) -> Mesh {
    let mut m = sheet(100.0, 100.0, 150, |x, y| Point3::new(200.0 + x, y, 0.0));
    synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    m
}

/// The mounting-face scan (sigma 0.03), with `extra` appended; the first triangles are the scan's.
fn scan_with(extra: Option<Mesh>) -> (Mesh, usize) {
    let mut m = mounting_face_scan();
    let n = m.triangle_count();
    if let Some(e) = extra {
        m.append(&e);
    }
    (m, n)
}

/// Resegments the scan's own triangles (a region of them) and returns the source of the noise and
/// the sorted `(kind, freeform, triangles)` of the regions that come out.
fn resegment_the_scan(extra: Option<Mesh>) -> (Value, Vec<(String, bool, u64)>) {
    let (mesh, n) = scan_with(extra);
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    let made =
        s.execute("region.create", json!({"mesh": "scan", "triangles": (0..n as u32).collect::<Vec<_>>()})).unwrap();
    let out = s.execute("region.resegment", json!({"regions": [made["id"]]})).unwrap();
    let list = s.execute("region.list", json!({})).unwrap();
    let mut regions: Vec<(String, bool, u64)> = list["regions"]
        .as_array()
        .unwrap()
        .iter()
        .map(|r| (r["kind"].to_string(), r["freeform"] == true, r["triangles"].as_u64().unwrap()))
        .collect();
    regions.sort();
    (out["noise_source"].clone(), regions)
}

/// The same selected surface is classified the same with and without unrelated geometry in the
/// mesh, clean (a mesh dominated by clean geometry would give a low sigma and make the genuine
/// plane look unsound) or noisy (it would protect fragments): the noise is measured on the
/// selection.
#[test]
fn a_selection_is_classified_the_same_with_and_without_unrelated_geometry() {
    let (source, alone) = resegment_the_scan(None);
    let (clean_source, with_clean) = resegment_the_scan(Some(unrelated_plate(0.005, 1)));
    let (noisy_source, with_noisy) = resegment_the_scan(Some(unrelated_plate(0.12, 2)));
    eprintln!("alone {alone:?}\nclean {with_clean:?}\nnoisy {with_noisy:?}");
    assert_eq!((source, clean_source, noisy_source), (json!("estimated"), json!("estimated"), json!("estimated")));
    assert!(alone.iter().any(|r| r.0 == "\"plane\"" && r.2 > 4000), "the mounting face is a plane: {alone:?}");
    assert_eq!(with_clean, alone, "clean geometry appended changed the classification");
    assert_eq!(with_noisy, alone, "noisy geometry appended changed the classification");
}

/// A mesh whose parts differ in noise by far more than the estimator's own spread has no one
/// noise: `region.auto_segment` says so (fallback) instead of using a mixture.
#[test]
fn a_mesh_with_very_different_noise_in_its_parts_uses_the_fallback() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", scan_with(Some(unrelated_plate(0.005, 1))).0);
    let out = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    assert_eq!((out["noise_source"].clone(), out["noise_used"].clone()), (json!("fallback"), json!(0.05)), "{out}");
}
