//! P9-O1b gates: `max_patches` of `surface.wrap` is a hard bound for every round, the part of the
//! cap a next round would overshoot is spent on the worst regions instead of being left unused, the
//! share within the tolerance never falls when the cap rises, and `scan.to_solid` retries once with
//! a larger cap. The tolerance of design doc §11.1 is unchanged.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

const TOLERANCE: f64 = 0.2;
/// Cap of the first gate (the shape needs several times as many patches).
const SMALL_CAP: usize = 600;
/// Share within the tolerance the code before this change reached on [`rippled_scan`] with
/// [`SMALL_CAP`] patches (`adaptive`).
const BASELINE_WITHIN: f64 = 0.8357124649502932;
/// Caps of the monotonicity gate.
const CAPS: [usize; 4] = [300, 600, 1200, 2400];
const RIPPLE_PERIOD: f64 = 4.0;
const RADIUS: f64 = 20.0;

/// A closed shape rippled all over: radius 20 mm plus a `amplitude` mm wave of the given period.
fn rippled_scan(amplitude: f64, period: f64, seed: u64) -> Mesh {
    let k = std::f64::consts::TAU * RADIUS / period;
    let mut m = synth::sphere(Point3::origin(), 1.0, 180, 90);
    for p in &mut m.positions {
        let d: Vec3 = p.coords;
        let wave = (k * d.x + 0.3).sin() * (k * d.y + 0.7).sin() * (0.5 * k * d.z).cos();
        p.coords = d * (RADIUS + amplitude * wave);
    }
    let defects = synth::ScanDefects {
        spacing: 0.6,
        density_ramp: 0.0,
        noise: 0.02,
        holes: 0,
        hole_radius: 0.0,
        spikes: 0.0,
        debris: 0,
    };
    synth::scanify(&m, &defects, &mut Rng::new(seed))
}

fn session_with(scan: Mesh) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("scan", scan);
    s
}

fn wrap(s: &mut Session, cap: usize, refine: &str, tolerance: f64) -> Value {
    let params = json!({"mesh": "scan", "tolerance": tolerance, "max_patches": cap, "refine": refine, "name": "w"});
    s.execute("surface.wrap", params).unwrap()
}

fn within(w: &Value) -> f64 {
    w["deviation"]["within"].as_f64().unwrap()
}

fn patches(w: &Value) -> usize {
    w["patches"].as_u64().unwrap() as usize
}

#[test]
fn a_scan_that_needs_more_patches_than_the_cap_stays_under_it_and_beats_the_old_share() {
    // Arrange
    let mut s = session_with(rippled_scan(0.8, RIPPLE_PERIOD, 3));

    // Act
    let w = wrap(&mut s, SMALL_CAP, "adaptive", TOLERANCE);

    // Assert
    eprintln!("cap {SMALL_CAP}: patches {} within {:.4} capped {}", w["patches"], within(&w), w["capped"]);
    assert!(patches(&w) <= SMALL_CAP, "{} patches over the cap {SMALL_CAP}", patches(&w));
    assert_eq!(w["capped"], true, "{w}");
    assert!(within(&w) > BASELINE_WITHIN, "within {} is not above the old {BASELINE_WITHIN}", within(&w));
}

#[test]
fn the_first_round_never_overshoots_a_tiny_cap() {
    // Arrange: fine detail everywhere and a tolerance far below it, so even the coarsest mesh the
    // budget allows is bigger than the cap.
    for refine in ["adaptive", "uniform"] {
        let mut s = session_with(rippled_scan(0.8, 2.5, 5));
        for cap in [60, 150, 600] {
            // Act
            let w = wrap(&mut s, cap, refine, 0.02);

            // Assert
            eprintln!("{refine} cap {cap}: patches {} within {:.4} capped {}", w["patches"], within(&w), w["capped"]);
            assert!(patches(&w) <= cap, "{refine}: {} patches over the cap {cap}", patches(&w));
            assert_eq!(w["capped"], true, "{refine} cap {cap}: {w}");
            assert!(w["cap_used"].as_u64() == Some(cap as u64), "{w}");
        }
    }
}

#[test]
fn a_larger_cap_never_gives_a_worse_share_and_never_exceeds_itself() {
    for refine in ["adaptive", "uniform"] {
        // Arrange
        let mut s = session_with(rippled_scan(0.8, RIPPLE_PERIOD, 3));
        let mut previous = 0.0;
        for cap in CAPS {
            // Act
            let w = wrap(&mut s, cap, refine, TOLERANCE);

            // Assert
            eprintln!("{refine} cap {cap}: patches {} within {:.4} capped {}", w["patches"], within(&w), w["capped"]);
            eprintln!("  levels {}", w["levels"]);
            assert!(patches(&w) <= cap, "{refine}: {} patches over the cap {cap}", patches(&w));
            assert!(within(&w) >= previous, "{refine}: cap {cap} gives {} after {previous}", within(&w));
            previous = within(&w);
        }
    }
}

#[test]
fn a_cap_that_a_next_round_would_overshoot_is_spent_not_left_unused() {
    // Arrange / Act
    let mut s = session_with(rippled_scan(0.8, RIPPLE_PERIOD, 3));
    let w = wrap(&mut s, 1500, "adaptive", TOLERANCE);

    // Assert: a capped result lands close under the cap (the search resolution is 10 %)
    eprintln!("patches {} of 1500, within {:.4}", w["patches"], within(&w));
    if w["capped"] == true {
        assert!(patches(&w) * 100 >= 1500 * 85, "{} of 1500 patches used: {w}", patches(&w));
    }
    let levels = w["levels"].as_array().unwrap();
    assert!(levels.iter().all(|l| l["patches"].as_u64().unwrap() <= 1500), "{levels:?}");
}

#[test]
fn a_later_uniform_round_is_never_smaller_and_worse_than_an_earlier_one() {
    // Arrange / Act
    let mut s = session_with(rippled_scan(0.8, RIPPLE_PERIOD, 3));
    let w = wrap(&mut s, 2400, "uniform", TOLERANCE);

    // Assert
    let levels = w["levels"].as_array().unwrap();
    eprintln!("levels {levels:?}");
    for (j, later) in levels.iter().enumerate() {
        for earlier in &levels[..j] {
            let (pl, pe) = (later["patches"].as_u64().unwrap(), earlier["patches"].as_u64().unwrap());
            let (wl, we) = (later["within"].as_f64().unwrap(), earlier["within"].as_f64().unwrap());
            assert!(!(pl < pe && wl < we), "round {} has fewer patches and a lower share: {levels:?}", later["round"]);
        }
    }
}

#[test]
fn the_wrap_is_deterministic_under_a_cap() {
    // Arrange / Act
    let mut a = session_with(rippled_scan(0.8, RIPPLE_PERIOD, 3));
    let mut b = session_with(rippled_scan(0.8, RIPPLE_PERIOD, 3));
    let (wa, wb) = (wrap(&mut a, 700, "adaptive", TOLERANCE), wrap(&mut b, 700, "adaptive", TOLERANCE));

    // Assert
    assert_eq!(wa["patches"], wb["patches"]);
    assert_eq!(wa["deviation"], wb["deviation"]);
    assert_eq!(wa["levels"], wb["levels"]);
}

#[test]
fn to_solid_retries_once_with_a_larger_cap_and_says_so() {
    // Arrange: the default cap is lowered to 300 patches so that the first attempt cannot pass
    let mut s = session_with(rippled_scan(0.5, 6.0, 9));

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "max_patches": 300})).unwrap();

    // Assert
    eprintln!("verdict {} retry {} patches {}", out["verdict"], out["retry"], out["patches"]);
    assert_eq!(out["retry"]["cap_from"], 300, "{out}");
    let to = out["retry"]["cap_to"].as_u64().unwrap();
    assert!(to > 300 && to <= 60_000, "{out}");
    assert!(out["retry"]["within_to"].as_f64().unwrap() > out["retry"]["within_from"].as_f64().unwrap(), "{out}");
    assert_eq!(out["verdict"], "pass", "{out}");
    let bodies = s.doc.bodies.len();
    assert_eq!(bodies, 1, "the body of the first attempt is gone");
}

#[test]
fn to_solid_wraps_once_when_the_first_attempt_passes() {
    // Arrange
    let mut s = session_with(rippled_scan(0.3, 8.0, 9));

    // Act
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();

    // Assert
    assert_eq!(out["verdict"], "pass", "{out}");
    assert!(out.get("retry").is_none_or(Value::is_null), "{out}");
    assert_eq!(s.doc.bodies.len(), 1);
}
