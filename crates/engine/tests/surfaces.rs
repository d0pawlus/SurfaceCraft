//! M3 through the command layer: freeform surface fit, deviation against it, STEP round trip.
use serde_json::json;
use surfacecraft_engine::Session;

fn fit_wave(noise: f64) -> (Session, serde_json::Value) {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "wave", "resolution": 90, "noise": noise, "seed": 5, "name": "scan"}))
        .unwrap();
    let n = s.doc.meshes[0].mesh.triangle_count() as u32;
    s.execute("region.create", json!({"mesh": "scan", "triangles": (0..n).collect::<Vec<u32>>(), "name": "free"}))
        .unwrap();
    let out = s.execute("surface.fit", json!({"region": "free"})).unwrap();
    (s, out)
}

/// Gate M3: maximum surface deviation ≤ 0.2 mm (design doc §11.1), noise-free and with
/// σ = 0.02 mm scanner noise.
#[test]
fn gate_m3_fit_surface_max_deviation() {
    for noise in [0.0, 0.02] {
        let (_, out) = fit_wave(noise);
        let max = out["max"].as_f64().unwrap();
        eprintln!(
            "gate_m3 surface σ={noise}: control {:?} rms {:.4} max {max:.4} mm",
            out["control"],
            out["rms"].as_f64().unwrap()
        );
        assert!(max <= 0.2, "{out}");
        assert_eq!(out["within_tolerance"], true);
    }
}

/// Gate M3: the surface exported as STEP re-imports as a one-face sheet, and the scan's
/// deviation from the re-imported geometry equals the deviation from the original.
#[test]
fn gate_m3_surface_step_roundtrip_through_commands() {
    let (mut s, out) = fit_wave(0.02);
    let d0 = s.execute("deviation.compute", json!({"object": "scan", "against": out["id"], "tolerance": 0.2})).unwrap();
    let exp = s.execute("export.step", json!({"objects": [out["id"]]})).unwrap();
    assert_eq!(exp["content"], "sheets");
    let imp = s.execute("import.step", json!({"data": exp["data"]})).unwrap();
    assert_eq!(imp["faces"][0], 1);
    let body = imp["bodies"][0].clone();
    let d1 = s
        .execute("deviation.compute", json!({"object": "scan", "against": body, "tolerance": 0.2, "max_distance": 1.0}))
        .unwrap();
    let (m0, m1) = (d0["stats"]["max_abs"].as_f64().unwrap(), d1["stats"]["max_abs"].as_f64().unwrap());
    // body deviation goes through a display tessellation (chord 0.01 mm), so allow that much
    assert!((m0 - m1).abs() < 0.02, "surface {m0} vs re-imported {m1}");
    assert!(m1 <= 0.2);
}

#[test]
fn folded_region_and_bad_input_are_refused() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "sphere", "resolution": 24, "name": "ball"})).unwrap();
    let n = s.doc.meshes[0].mesh.triangle_count() as u32;
    let r = s.execute("surface.fit", json!({"mesh": "ball", "triangles": (0..n).collect::<Vec<u32>>()}));
    assert!(r.is_err(), "a closed sphere is not one patch: {r:?}");
    assert!(s.execute("surface.fit", json!({"mesh": "ball", "degree": 9})).is_err());
    assert!(s.execute("export.step", json!({"objects": []})).is_err());
    assert!(s.execute("import.step", json!({"data": "bm90IHN0ZXA="})).is_err());
}
