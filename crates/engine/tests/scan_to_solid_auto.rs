//! `scan.to_solid` with `method: auto`: the choice is by outcome, not by share alone.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Rng, synth};
use surfacecraft_kernel::{hybrid, nurbs, record::brep::Brep};

fn body_brep(s: &Session, name: &str) -> Brep {
    let id = s.doc.find_by_name(name).unwrap().0;
    (*s.doc.body(id).unwrap().brep).clone()
}

fn vertical_edges(brep: &Brep) -> Vec<usize> {
    brep.edges
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            let (p, q) = (brep.vertices[e.vertices[0] as usize].point, brep.vertices[e.vertices[1] as usize].point);
            (p[0] - q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9 && (p[2] - q[2]).abs() > 1.0
        })
        .map(|(i, _)| i)
        .collect()
}

fn wave(x: f64, y: f64) -> f64 {
    11.0 + 2.5 * (x / 9.0).sin() * (y / 7.0).cos()
}

fn wave_spline() -> surfacecraft_kernel::record::brep::Surface {
    let n = 40;
    let grid: Vec<Vec<[f64; 3]>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    let x = -4.0 + 48.0 * i as f64 / (n - 1) as f64;
                    let y = -4.0 + 38.0 * j as f64 / (n - 1) as f64;
                    [x, y, wave(x, y)]
                })
                .collect()
        })
        .collect();
    nurbs::fit_grid(&grid, 16, 16, 3).unwrap()
}

/// A scan of a 40 x 30 block with filleted vertical edges and a wavy freeform top (part A of the
/// hybrid gate): its sides segment into primitives, the top does not.
fn block_with_wavy_top(seed: u64) -> Session {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [40.0, 30.0, 20.0], "name": "block"})).unwrap();
    let edges = vertical_edges(&body_brep(&s, "block"));
    s.execute("body.fillet", json!({"object": "block", "edges": edges, "size": 5.0})).unwrap();
    let cut = hybrid::cut_by_spline(&body_brep(&s, "block"), &wave_spline(), [0.0, 0.0, 1.0]).unwrap();
    let part = s.doc.add_body("part", cut);
    s.execute("body.to_mesh", json!({"object": part.0, "chord": 0.01, "name": "cad"})).unwrap();
    let id = s.doc.find_by_name("cad").unwrap().0;
    let mut mesh = (*s.doc.mesh(id).unwrap().mesh).clone();
    mesh.weld(1e-6);
    let d = synth::ScanDefects { spacing: 0.7, noise: 0.03, ..synth::ScanDefects::default() };
    let mut out = Session::new();
    out.doc.add_mesh("scan", synth::scanify(&mesh, &d, &mut Rng::new(seed)));
    out
}

fn codes(list: &Value) -> Vec<String> {
    list.as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap().to_string()).collect()
}

fn methods_of(out: &Value) -> Vec<(String, String)> {
    out["attempts"]
        .as_array()
        .unwrap()
        .iter()
        .map(|a| (a["method"].as_str().unwrap().to_string(), a["verdict"].as_str().unwrap().to_string()))
        .collect()
}

/// The share rule picks the primitives (the sides are 70 % of the block), the planner cannot close
/// a solid over the wavy top and ends in `fail`; `auto` then runs the freeform wrap, which passes. The
/// primitives cover enough of the scan for a hybrid body, but a pass is not followed by a hybrid
/// attempt (it costs time and nobody asked for it): the result is the wrap, and a next step names the
/// `scan.to_solid` call with `method: hybrid`. The primitives attempt leaves nothing behind; one undo
/// removes the whole run.
#[test]
fn auto_falls_back_to_the_freeform_wrap_and_offers_hybrid_when_the_primitives_do_not_close() {
    let mut s = block_with_wavy_top(21);
    let before = (s.doc.meshes.len(), s.doc.meshes[0].mesh.triangle_count());

    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();

    eprintln!("verdict {} method {} attempts {}", out["verdict"], out["method"], out["attempts"]);
    assert_eq!(out["verdict"], "pass", "{out}");
    assert_eq!(out["method"], "freeform", "{out}");
    assert_eq!(
        methods_of(&out),
        vec![("primitives".to_string(), "fail".to_string()), ("freeform".to_string(), "pass".to_string())]
    );
    let returned: Vec<bool> = out["attempts"].as_array().unwrap().iter().map(|a| a["returned"] == true).collect();
    assert_eq!(returned, vec![false, true]);
    let offer =
        out["next_steps"].as_array().unwrap().iter().find(|n| n["params"]["method"] == "hybrid").expect("an offer");
    assert_eq!(offer["command"], "scan.to_solid", "{offer}");
    assert!(offer["why"].as_str().unwrap().contains("primitive regions"), "{offer}");
    assert!(!out["attempts"][0]["issues"].as_array().unwrap().is_empty() && out["attempts"][0]["seconds"].is_number());
    assert!(s.doc.primitives.is_empty(), "the losing attempt left primitives behind");
    assert_eq!(s.doc.bodies.len(), 1, "exactly the returned solid");
    s.execute("edit.undo", Value::Null).unwrap();
    assert!(s.doc.bodies.is_empty() && s.doc.primitives.is_empty() && s.doc.regions.is_empty());
    assert_eq!((s.doc.meshes.len(), s.doc.meshes[0].mesh.triangle_count()), before);
}

/// With `retry: false` the same scan is the primitives attempt alone: no second method, no hybrid.
#[test]
fn retry_false_stops_auto_after_the_first_attempt_whatever_the_primitives_cover() {
    let mut s = block_with_wavy_top(21);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "retry": false})).unwrap();
    let attempts = out["attempts"].as_array().unwrap();
    assert_eq!(attempts.len(), 3, "the primitives attempt, and the two that were skipped: {attempts:?}");
    assert!(attempts[0]["verdict"].is_string());
    for skipped in &attempts[1..] {
        assert!(skipped["skipped"]["reason"].as_str().unwrap().contains("`retry` is false"), "{skipped}");
    }
    let methods: Vec<&str> = attempts.iter().map(|a| a["method"].as_str().unwrap()).collect();
    assert_eq!(methods, ["primitives", "freeform", "hybrid"]);
}

/// With `method: primitives` the same scan is judged as before: one attempt, a fail.
#[test]
fn an_explicit_method_makes_one_attempt() {
    let mut s = block_with_wavy_top(21);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "primitives"})).unwrap();
    assert_eq!(out["verdict"], "fail", "{out}");
    assert_eq!(methods_of(&out), vec![("primitives".to_string(), "fail".to_string())]);
    assert!(codes(&out["issues"]).contains(&"low_coverage".to_string()));
}

/// A flat plate of 100 x 100 mm with `cells` cells a side and noise `sigma`.
fn plate_session(cells: usize, sigma: f64, seed: u64) -> Session {
    let mut m = surfacecraft_geom::synth::plane(
        surfacecraft_geom::Point3::origin(),
        surfacecraft_geom::Vec3::x(),
        surfacecraft_geom::Vec3::y(),
        100.0,
        100.0,
        cells,
        cells,
    );
    if sigma > 0.0 {
        synth::add_noise(&mut m, sigma, &mut Rng::new(seed));
    }
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    s
}

/// The tolerance of the call reaches the segmentation (it was the default 0.2 whatever the call
/// said): the noise floor of a clean scan is 5 % of it, and the report says which noise was used.
#[test]
fn the_callers_tolerance_reaches_the_segmentation_and_the_noise_is_reported() {
    let used = |tolerance: Option<f64>| {
        let mut s = plate_session(60, 0.0, 1);
        let params = match tolerance {
            Some(t) => json!({"scan": "scan", "tolerance": t}),
            None => json!({"scan": "scan"}),
        };
        let out = s.execute("scan.to_solid", params).unwrap();
        (out["scan_report"]["noise_used"].as_f64(), out["scan_report"]["noise_source"].clone())
    };
    let (default, source) = used(None);
    assert_eq!((default.map(|x| (x * 1e6).round()), source), (Some(10_000.0), json!("estimated")));
    let (fine, _) = used(Some(0.02));
    assert_eq!(fine.map(|x| (x * 1e6).round()), Some(1_000.0), "the floor is 5 % of the caller's tolerance");
}

/// A scan whose noise (sigma 0.4 mm) is above the tolerance cannot be judged: the result says so
/// and names a tolerance the scan can meet.
#[test]
fn a_scan_rougher_than_the_tolerance_gets_the_issue_and_a_workable_tolerance() {
    // a closed sphere of about 3 mm cells: no hole to fill, so no clean patch among the noisy points
    let mut m = synth::sphere(surfacecraft_geom::Point3::origin(), 30.0, 64, 32);
    synth::add_noise(&mut m, 0.4, &mut Rng::new(3));
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    // the primitives method: a freeform wrap can smooth the noise away and pass
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "primitives"})).unwrap();
    assert_ne!(out["verdict"], "pass", "{out}");
    assert_eq!(out["scan_report"]["noise_above_tolerance"], true, "{out}");
    assert_eq!(out["scan_report"]["noise_source"], "estimated", "{out}");
    assert!(codes(&out["issues"]).contains(&"noise_above_tolerance".to_string()), "{out}");
    let step = &out["next_steps"][0];
    assert_eq!(step["command"], "scan.to_solid", "{out}");
    let tolerance = step["params"]["tolerance"].as_f64().unwrap();
    let sigma = out["scan_report"]["noise_used"].as_f64().unwrap();
    assert!((tolerance - 3.0 * sigma).abs() < 0.002, "{tolerance} for sigma {sigma}");
    assert!(tolerance > 0.2);
}
