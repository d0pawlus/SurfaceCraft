//! `mesh.close_handles` and `scan.to_solid` with `close_handles` on a synthetic thin wall scanned from
//! both sides with hundreds of tunnels through it (the defect of the one real scan the pipeline failed).
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::synth_wall::{Wall, wall};
use surfacecraft_geom::{Mesh, Point3, Rng, synth};

/// An open curved wall 80 mm square, sheets `gap` apart, one rim, with one-cell tunnels every three
/// cells over its upper half.
fn tunnelled_wall(gap: f64) -> (Mesh, usize) {
    let mut tubes = Vec::new();
    for j in (41..77).step_by(3) {
        for i in (2..77).step_by(3) {
            tubes.push((i, j, 1));
        }
    }
    let w = Wall { size: 80.0, cells: 80, gap, bend: 5.0, tubes };
    let mut m = wall(&w);
    synth::add_noise(&mut m, 0.01, &mut Rng::new(11));
    (m, w.built_tubes().len())
}

fn codes(out: &Value) -> Vec<String> {
    out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str().map(str::to_string)).collect()
}

#[test]
fn close_handles_closes_the_tunnels_keeps_the_rim_and_makes_a_new_mesh() {
    // Arrange
    let (m, tunnels) = tunnelled_wall(0.6);
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);

    // Act
    let out = s.execute("mesh.close_handles", json!({"object": "scan", "max_size": 8.0})).unwrap();

    // Assert
    eprintln!("{out}");
    let closed = out["handles_closed"].as_u64().unwrap() as usize;
    assert_eq!(out["handles_before"], json!(tunnels));
    assert!(3 * closed >= 2 * tunnels, "at least two thirds: {closed} of {tunnels}");
    assert_eq!(out["handles_after"], json!(tunnels - closed), "none made");
    assert_eq!((out["loops_before"].clone(), out["loops_after"].clone()), (json!(1), json!(1)));
    assert_eq!((out["pieces_before"].clone(), out["pieces_after"].clone()), (json!(1), json!(1)));
    assert!(out["id"].is_u64() && out["changed"] == json!(true));
    assert!(s.doc.find_by_name("handles closed").is_some(), "a new mesh; the scan stays");
    let within = out["removed_within"].as_f64().unwrap() / out["removed_samples"].as_f64().unwrap().max(1.0);
    assert!(within > 0.5, "most of what was cut out lies on the result: {within}");
}

#[test]
fn a_mesh_without_small_handles_gets_no_new_mesh() {
    let mut s = Session::new();
    s.doc.add_mesh("plate", synth::wave(Point3::origin(), 40.0, 30.0, 2.0, 40, 30));
    let out = s.execute("mesh.close_handles", json!({"object": "plate"})).unwrap();
    assert!(out["id"].is_null() && out["changed"] == json!(false), "{out}");
    assert_eq!(s.doc.all_ids().len(), 1);
}

#[test]
fn close_handles_options_are_checked() {
    let (m, _) = tunnelled_wall(0.6);
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    for bad in [
        json!({"scan": "scan", "handle_size": 5.0}),
        json!({"scan": "scan", "close_handles": true, "handle_size": -1.0}),
        json!({"scan": "scan", "close_handles": true, "method": "primitives"}),
        json!({"scan": "scan", "close_handles": true, "thickness": 1.0}),
    ] {
        assert!(s.execute("scan.to_solid", bad.clone()).is_err(), "{bad}");
    }
    assert!(s.execute("mesh.close_handles", json!({"object": "scan", "max_size": -1.0})).is_err());
}

/// A wall that is all sponge: one-cell tunnels every three cells over the whole panel.
fn sponge_wall() -> (Mesh, usize) {
    let tubes: Vec<(usize, usize, usize)> =
        (2..77).step_by(3).flat_map(|j| (2..77).step_by(3).map(move |i| (i, j, 1))).collect();
    let w = Wall { size: 80.0, cells: 80, gap: 0.6, bend: 5.0, tubes };
    let mut m = wall(&w);
    synth::add_noise(&mut m, 0.01, &mut Rng::new(12));
    (m, w.built_tubes().len())
}

fn to_solid(m: &Mesh, extra: Value) -> Value {
    let mut s = Session::new();
    s.doc.add_mesh("scan", m.clone());
    let mut params = json!({"scan": "scan", "method": "freeform"});
    if let (Some(p), Some(e)) = (params.as_object_mut(), extra.as_object()) {
        p.extend(e.clone());
    }
    let t0 = std::time::Instant::now();
    let out = s.execute("scan.to_solid", params).unwrap();
    eprintln!(
        "{extra}: verdict {} patches {} within {} coverage {} in {:.1} s; issues {:?}; retry {}",
        out["verdict"],
        out["patches"],
        out["deviation"]["within"],
        out["deviation"]["coverage"],
        t0.elapsed().as_secs_f64(),
        codes(&out),
        out["retry"]
    );
    out
}

/// The gate: the wrap of a wall that is all tunnels spends its patch budget on them (the first network
/// stops at the default cap); with `close_handles` it is `pass` or `review` within the default cap and
/// with far fewer patches, the rim stays, and the plain run offers the step.
#[test]
fn gate_m4_a_tunnelled_wall_wraps_after_its_handles_are_closed() {
    let (m, tunnels) = sponge_wall();
    let plain = to_solid(&m, json!({}));
    let closed = to_solid(&m, json!({"close_handles": true, "handle_size": 8.0}));
    let step = closed["preparation"].as_array().unwrap().iter().find(|p| p["step"] == "close_handles").unwrap();
    eprintln!("tunnels {tunnels}; close_handles {}", step["numbers"]);
    let offered = plain["next_steps"]
        .as_array()
        .into_iter()
        .flatten()
        .any(|st| st["command"] == "scan.to_solid" && st["params"]["close_handles"] == json!(true));
    assert!(offered, "the plain run offers close_handles: {}", plain["next_steps"]);
    let first_patches = plain["retry"]["patches_from"].as_u64().or_else(|| plain["patches"].as_u64()).unwrap();
    // measured 11 952 (macOS, Linux and Windows CI) against the default cap of 12 000: the bound is
    // 1.5 times below the measured value
    assert!(first_patches >= 7_900, "the first plain network stops near the default cap: {first_patches}");
    assert!(matches!(closed["verdict"].as_str(), Some("pass" | "review")), "{}", closed["verdict"]);
    let patches = closed["patches"].as_u64().unwrap();
    // measured 66 patches against 11 952: far below two thirds
    assert!(patches * 3 <= first_patches * 2, "{patches} patches against {first_patches}");
    assert!(closed["retry"].is_null(), "no larger cap was needed");
    assert!(3 * step["numbers"]["handles_closed"].as_u64().unwrap() as usize >= 2 * tunnels);
    assert_eq!((step["numbers"]["loops_before"].clone(), step["numbers"]["loops_after"].clone()), (json!(1), json!(1)));
    assert_eq!(step["numbers"]["pieces_after"], json!(1));
    // measured 0.994; the bound is 1.5 times below it
    assert!(closed["deviation"]["within"].as_f64().unwrap() >= 0.66, "{}", closed["deviation"]);
    assert!(codes(&closed).contains(&"scan_handles_closed".to_string()), "anything cut is reported");
    assert_ne!(closed["verdict"], json!("pass"));
}

/// With `close_handles` and nothing to close the result is that of the plain call, without the issue;
/// with one tunnel closed the issue is raised and the verdict is at most `review`.
#[test]
fn scan_handles_closed_is_raised_exactly_when_something_was_cut() {
    let plate = synth::wave(Point3::origin(), 40.0, 30.0, 2.0, 40, 30);
    let plain = to_solid(&plate, json!({}));
    let asked = to_solid(&plate, json!({"close_handles": true}));
    assert_eq!(plain["verdict"], asked["verdict"]);
    assert!(!codes(&asked).contains(&"scan_handles_closed".to_string()), "{:?}", codes(&asked));
    let one = wall(&Wall { size: 20.0, cells: 20, gap: 0.6, bend: 1.0, tubes: vec![(10, 10, 1)] });
    let cut = to_solid(&one, json!({"close_handles": true, "handle_size": 8.0}));
    assert!(codes(&cut).contains(&"scan_handles_closed".to_string()), "{:?}", codes(&cut));
    assert_ne!(cut["verdict"], json!("pass"));
}

/// A smooth panel with one rough corner, wrapped under a cap its first round does not fit: the coarse
/// start (`coarse_start`) begins with larger patches where it is smooth and spends the rest of the cap
/// on the corner, so more of the scan is within the tolerance than when the first round is cut to the
/// cap and the wrap stops there.
#[test]
fn gate_m4_the_coarse_start_spends_the_patch_cap_where_the_patches_miss() {
    let mut m = synth::wave(Point3::origin(), 120.0, 120.0, 3.0, 240, 240);
    for p in &mut m.positions {
        if p.x < 30.0 && p.y < 30.0 {
            p.z += 0.8 * (p.x * 1.3).sin() * (p.y * 1.1).cos();
        }
    }
    let run = |coarse: bool| {
        let mut s = Session::new();
        s.doc.add_mesh("scan", m.clone());
        let out =
            s.execute("surface.wrap", json!({"mesh": "scan", "max_patches": 300, "coarse_start": coarse})).unwrap();
        eprintln!(
            "coarse {coarse}: patches {} within {} levels {}",
            out["patches"], out["deviation"]["within"], out["levels"]
        );
        out
    };
    let (plain, coarse) = (run(false), run(true));
    let within = |o: &Value| o["deviation"]["within"].as_f64().unwrap();
    assert!(plain["levels"][0]["partial"] == json!("first_round_over_cap"), "{}", plain["levels"]);
    assert!(coarse["levels"].as_array().unwrap().len() > 1, "the coarse start refines: {}", coarse["levels"]);
    // measured: 1.78 % of the scan outside the tolerance with the first round cut to the cap, 0.74 %
    // with the coarse start (2.4 times fewer); the bound asks for 1.5 times fewer
    let (miss_plain, miss_coarse) = (1.0 - within(&plain), 1.0 - within(&coarse));
    assert!(1.5 * miss_coarse < miss_plain, "{miss_coarse} vs {miss_plain}");
}
