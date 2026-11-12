//! P9-O8 gates: the bodies of `surface.wrap` are oriented and the verdict of `scan.to_solid` tells
//! what is wrong. On real scans the failure was a part small against the tolerance (a scan in
//! metres, a tolerance in millimetres): the coarse mesh was reduced to a handful of crumpled
//! triangles, the body had no volume and a third of the scan faced away from it.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

const DEFAULT_TOLERANCE: f64 = 0.2;
/// A closed solid made of the scan must be this close to the scan's own volume.
const VOLUME_TOLERANCE: f64 = 0.01;
const MIN_FACING_COVERAGE: f64 = 0.999;
/// Smooth parts must not fold back on themselves.
const MAX_SMOOTH_JUMP_P99_DEG: f64 = 30.0;

fn mesh_volume(m: &Mesh) -> f64 {
    (0..m.triangle_count())
        .filter_map(|i| m.triangle(i))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum()
}

fn scaled(mut m: Mesh, k: f64) -> Mesh {
    m.positions.iter_mut().for_each(|p| p.coords *= k);
    m
}

fn blob(nu: usize) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, nu, nu / 2);
    for p in &mut m.positions {
        let d = p.coords;
        let bumps =
            0.09 * (3.0 * d.x).sin() * (2.0 * d.y).cos() + 0.07 * (2.0 * d.z + 1.2 * d.x).sin() * (3.0 * d.y).cos();
        p.coords = d * (20.0 * (1.0 + bumps));
    }
    synth::add_noise(&mut m, 0.03, &mut Rng::new(9));
    m
}

fn wrap(scan: Mesh, params: Value) -> (Session, Value) {
    let mut s = Session::new();
    s.doc.add_mesh("scan", scan);
    let mut p = json!({"mesh": "scan"});
    p.as_object_mut().unwrap().extend(params.as_object().unwrap().clone());
    let w = s.execute("surface.wrap", p).unwrap();
    (s, w)
}

fn facing_coverage(s: &mut Session, w: &Value, tolerance: f64) -> f64 {
    let c = s
        .execute(
            "body.compare",
            json!({"body": w["body"]["id"], "scan": "scan", "tolerance": tolerance, "facing": true}),
        )
        .unwrap();
    let vertices = s.doc.find_by_name("scan").and_then(|(id, _)| s.doc.mesh(id)).map_or(1, |m| m.mesh.vertex_count());
    c["totals"]["count"].as_f64().unwrap() / vertices as f64
}

#[test]
fn a_smooth_blob_wraps_into_an_outward_solid_without_folds() {
    // Arrange
    let scan = blob(160);
    let exact = mesh_volume(&scan);

    // Act
    let (mut s, w) = wrap(scan, json!({"tolerance": DEFAULT_TOLERANCE}));
    let facing = facing_coverage(&mut s, &w, DEFAULT_TOLERANCE);

    // Assert
    assert_eq!(w["closed"], true, "{w}");
    let volume = w["body"]["volume"].as_f64().expect("a closed network has a volume");
    assert!((volume - exact).abs() <= VOLUME_TOLERANCE * exact, "volume {volume} vs {exact}");
    assert!(facing >= MIN_FACING_COVERAGE, "facing coverage {facing}");
    let p99 = w["smooth_jump"]["p99_deg"].as_f64().unwrap();
    eprintln!("blob: volume {volume} exact {exact}, facing {facing}, smooth jump {}", w["smooth_jump"]);
    assert!(p99 < MAX_SMOOTH_JUMP_P99_DEG, "smooth jump p99 {p99} deg: {}", w["smooth_jump"]);
    assert!(w["smooth_jump"]["max_deg"].as_f64().unwrap() < 179.0, "{}", w["smooth_jump"]);
    assert!(w["warning"].is_null(), "{w}");
}

fn tiny_parts() -> Vec<(&'static str, Mesh)> {
    vec![
        ("torus", scaled(synth::torus(Point3::origin(), Vec3::z(), 20.0, 4.0, 200, 80), 0.001)),
        ("bracket", scaled(synth::bracket(), 0.001)),
        ("blob", scaled(blob(160), 0.001)),
    ]
}

#[test]
fn a_part_much_smaller_than_the_tolerance_still_gives_a_solid_with_a_positive_volume_and_a_warning() {
    let mut closed = 0;
    for (name, scan) in tiny_parts() {
        // Act
        let (_s, w) = wrap(scan, json!({}));

        // Assert
        if w["closed"] == true {
            closed += 1;
            let v = w["body"]["volume"].as_f64();
            assert!(v.is_some_and(|v| v.is_finite() && v > 0.0), "{name}: volume {:?}: {w}", w["body"]["volume"]);
        }
        let warning = w["warning"].as_str().unwrap_or_default();
        assert!(warning.contains("units"), "{name}: no warning about the units: {w}");
    }
    assert!(closed > 0, "no part closed, so no volume was checked");
}

#[test]
fn scan_to_solid_names_the_tolerance_when_the_part_is_smaller_than_it() {
    // Arrange
    let mut s = Session::new();
    s.doc.add_mesh("scan", scaled(synth::torus(Point3::origin(), Vec3::z(), 20.0, 4.0, 200, 80), 0.001));

    // Act
    let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();

    // Assert
    let codes: Vec<&str> = r["issues"].as_array().unwrap().iter().filter_map(|i| i["code"].as_str()).collect();
    assert!(codes.contains(&"tolerance_exceeds_part"), "{codes:?}: {r}");
    assert_ne!(r["verdict"], "pass");
}

#[test]
fn a_decimation_that_stops_above_its_target_says_so_and_why() {
    // Arrange: a thin plate cannot be brought to 4 triangles without folding it over
    let mut s = Session::new();
    let mut plate = synth::rounded_cuboid(Point3::origin(), Vec3::new(40.0, 40.0, 1.0), 0.4, 30);
    synth::add_noise(&mut plate, 0.05, &mut Rng::new(3));
    s.doc.add_mesh("plate", plate);
    s.doc.add_mesh("ball", synth::sphere(Point3::origin(), 10.0, 40, 20));

    // Act
    let thin = s.execute("mesh.decimate", json!({"object": "plate", "target": 4})).unwrap();
    let ball = s.execute("mesh.decimate", json!({"object": "ball", "target": 100})).unwrap();

    // Assert
    assert!(thin["triangles"].as_u64().unwrap() > 4, "{thin}");
    let why = thin["target_not_reached"].as_str().unwrap_or_default();
    assert!(why.starts_with("stopped at"), "{thin}");
    assert!(ball["target_not_reached"].is_null(), "{ball}");
}
