//! P9 scan 2 gates: on a closed organic scan whose patch network is held at its cap, the patches
//! are refitted to the scan by least squares (not only through a few points under the chords of
//! the coarse mesh) and the adaptive rounds go on at the cap while they gain. The tolerance and
//! the share of design doc §11.1 are unchanged.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

const TOLERANCE: f64 = 0.2;
const RADIUS: f64 = 20.0;
/// Noise of the scan (mm), that of the real scan this reproduces.
const NOISE: f64 = 0.05;

/// Small sharp bumps on the body: how many, their height and width (mm).
const BUMPS: usize = 90;
const BUMP_HEIGHT: f64 = 1.4;
const BUMP_WIDTH: f64 = 1.3;

/// Directions of the bumps, spread over the sphere (a Fibonacci lattice).
fn bump_directions() -> Vec<Vec3> {
    let golden = std::f64::consts::PI * (3.0 - 5f64.sqrt());
    (0..BUMPS)
        .map(|k| {
            let z = 1.0 - 2.0 * (k as f64 + 0.5) / BUMPS as f64;
            let r = (1.0 - z * z).sqrt();
            let a = golden * k as f64;
            Vec3::new(r * a.cos(), r * a.sin(), z)
        })
        .collect()
}

/// A closed organic body: a sphere of radius 20 mm with broad lobes and small sharp bumps, scanned
/// with noise [`NOISE`].
fn organic_scan(seed: u64) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, 280, 140);
    let dirs = bump_directions();
    for p in &mut m.positions {
        let d: Vec3 = p.coords;
        let lobes = 2.0 * (2.0 * d.x).sin() * (1.5 * d.y + 0.4).cos() + 1.2 * (3.0 * d.z).sin();
        let bumps: f64 = dirs
            .iter()
            .map(|c| {
                let gap = RADIUS * d.angle(c);
                BUMP_HEIGHT * (-(gap * gap) / (BUMP_WIDTH * BUMP_WIDTH)).exp()
            })
            .sum();
        p.coords = d * (RADIUS + lobes + bumps);
    }
    let defects = synth::ScanDefects {
        spacing: 0.5,
        density_ramp: 0.0,
        noise: NOISE,
        holes: 0,
        hole_radius: 0.0,
        spikes: 0.0,
        debris: 0,
    };
    synth::scanify(&m, &defects, &mut Rng::new(seed))
}

fn wrap(scan: Mesh, cap: usize) -> Value {
    let mut s = Session::new();
    s.doc.add_mesh("scan", scan);
    let params = json!({"mesh": "scan", "tolerance": TOLERANCE, "max_patches": cap, "name": "w"});
    s.execute("surface.wrap", params).unwrap()
}

fn within(w: &Value) -> f64 {
    w["deviation"]["within"].as_f64().unwrap()
}

fn patches(w: &Value) -> usize {
    w["patches"].as_u64().unwrap() as usize
}

/// Rounds of the wrap cut to the cap.
fn rounds_at_cap(w: &Value) -> usize {
    w["levels"].as_array().unwrap().iter().filter(|l| l["partial"] == "next_round_over_cap").count()
}

/// Cap of the gate: the body needs more patches than this (main stopped at 99.03 % of the samples
/// within the tolerance, 0.97 % out; with the refit and the rounds at the cap 0.19 % are out).
const GATE_CAP: usize = 1200;
/// Largest share of the samples out of the tolerance the gate allows.
const GATE_MISSED: f64 = 0.005;

/// The wrap of [`organic_scan`] at [`GATE_CAP`], made once for the tests that read it.
fn gate_wrap() -> &'static Value {
    static WRAP: std::sync::OnceLock<Value> = std::sync::OnceLock::new();
    WRAP.get_or_init(|| wrap(organic_scan(7), GATE_CAP))
}

#[test]
fn gate_m4_an_organic_scan_held_at_its_cap_is_within_the_tolerance() {
    // Arrange / Act
    let w = gate_wrap();

    // Assert
    eprintln!("cap {GATE_CAP}: patches {} within {:.5} levels {}", w["patches"], within(w), w["levels"]);
    assert!(patches(w) <= GATE_CAP, "{} patches over the cap", patches(w));
    assert!(1.0 - within(w) <= GATE_MISSED, "{:.3} % out of {TOLERANCE} mm", 100.0 * (1.0 - within(w)));
    let levels = w["levels"].as_array().unwrap();
    assert!(levels.iter().all(|l| l["patches"].as_u64().unwrap() <= GATE_CAP as u64), "{levels:?}");
}

#[test]
fn rounds_at_the_cap_go_on_while_they_gain_and_never_lose_the_best() {
    // Arrange / Act
    let w = gate_wrap();

    // Assert: the rounds went on after the first round cut to the cap (measured: three rounds cut
    // to it, the first gaining 1.6 points; the rule needs only that one more round followed), and
    // the result is the best round
    let levels = w["levels"].as_array().unwrap();
    let best = levels.iter().map(|l| l["within"].as_f64().unwrap()).fold(0.0, f64::max);
    assert!(rounds_at_cap(w) >= 2, "{levels:?}");
    assert_eq!(within(w), best, "{levels:?}");
}

/// Share of the samples out of the tolerance on main for the body at the default cap (1614 patches,
/// 99.66 % within).
const MAIN_DEFAULT_MISSED: f64 = 0.0034;

#[test]
fn an_organic_scan_that_already_passes_passes_closer_than_before() {
    // Arrange: the default cap is far above what the body needs
    let scan = organic_scan(7);

    // Act
    let mut s = Session::new();
    s.doc.add_mesh("scan", scan);
    let w = s.execute("surface.wrap", json!({"mesh": "scan", "name": "w"})).unwrap();

    // Assert
    eprintln!("default cap: patches {} within {:.5} capped {}", w["patches"], within(&w), w["capped"]);
    assert_eq!(w["within_tolerance"], true, "{w}");
    assert_eq!(w["capped"], false, "{w}");
    assert!(1.0 - within(&w) <= MAIN_DEFAULT_MISSED / 1.5, "{w}");
}
