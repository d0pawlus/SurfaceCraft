//! P9-O1 gate: `surface.wrap` lays its patches adaptively. A closed organic scan that is smooth
//! almost everywhere and carries one small region of fine detail (ripples of 0.7 mm over a cap of
//! about 12 mm) is wrapped twice: with the adaptive layout (local refinement where the patches miss
//! the tolerance) and with the uniform one (the whole budget halved). Both must bring the scan into
//! the 0.2 mm tolerance; the adaptive layout may use at most 40 % of the patches of the uniform one.
//! The tolerance of design doc §11.1 is unchanged.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Vec3, synth};

const TOLERANCE: f64 = 0.2;
const MIN_WITHIN: f64 = 0.99;
const MAX_PATCH_SHARE: f64 = 0.4;
/// Angular radius of the detail cap (rad) and the ripple period along the surface (mm).
const DETAIL_ANGLE: f64 = 0.5;
const RIPPLE_PERIOD: f64 = 2.0;
const RIPPLE_AMPLITUDE: f64 = 1.0;

fn detail_radius(d: &Vec3) -> f64 {
    let axis = Vec3::new(1.0, 0.3, 0.2).normalize();
    let angle = d.dot(&axis).clamp(-1.0, 1.0).acos();
    let smooth = 20.0 * (1.0 + 0.08 * (2.0 * d.x).sin() * (1.5 * d.y).cos() + 0.06 * (1.5 * d.z + d.x).sin());
    if angle >= DETAIL_ANGLE {
        return smooth;
    }
    let window = (0.5 * std::f64::consts::PI * angle / DETAIL_ANGLE).cos().powi(2);
    smooth + RIPPLE_AMPLITUDE * window * (std::f64::consts::TAU * 20.0 * angle / RIPPLE_PERIOD).sin()
}

fn detail_scan(seed: u64) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, 260, 130);
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = d * detail_radius(&d);
    }
    let defects = synth::ScanDefects {
        spacing: 0.5,
        density_ramp: 0.0,
        noise: 0.03,
        holes: 0,
        hole_radius: 0.0,
        spikes: 0.0,
        debris: 0,
    };
    synth::scanify(&m, &defects, &mut Rng::new(seed))
}

fn wrap(refine: &str) -> (Session, Value) {
    let mut s = Session::new();
    s.doc.add_mesh("scan", detail_scan(21));
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    assert_eq!(c["holes_left"], 0, "{c}");
    let w = s
        .execute(
            "surface.wrap",
            json!({"mesh": "scan", "tolerance": TOLERANCE, "min_within": MIN_WITHIN, "max_patches": 60000, "refine": refine}),
        )
        .unwrap();
    (s, w)
}

/// Share of the scan vertices within the tolerance of a dense tessellation of the wrapped solid.
fn independent_within(s: &Session, w: &Value) -> f64 {
    let id = w["body"]["id"].as_u64().unwrap();
    let body = s.doc.body(surfacecraft_doc::ObjectId(id)).unwrap();
    let tess = surfacecraft_kernel::tess::tessellate(&body.brep, Some(0.01)).unwrap();
    let bvh = Bvh::build(&tess);
    let scan = s.doc.find_by_name("scan").map(|(id, _)| s.doc.mesh(id).unwrap().mesh.clone()).unwrap();
    let n = scan.positions.len();
    let ok =
        scan.positions.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).filter(|h| h.distance <= TOLERANCE).count();
    ok as f64 / n as f64
}

#[test]
fn gate_p9_adaptive_wrap_needs_far_fewer_patches() {
    // Arrange / Act
    let (_su, uniform) = wrap("uniform");
    let (sa, adaptive) = wrap("adaptive");
    let (pu, pa) = (uniform["patches"].as_u64().unwrap(), adaptive["patches"].as_u64().unwrap());
    eprintln!("uniform: {uniform}");
    eprintln!("adaptive: {adaptive}");
    let within = independent_within(&sa, &adaptive);
    eprintln!(
        "patches uniform {pu}, adaptive {pa} ({:.1} %); independent within {:.4}",
        100.0 * pa as f64 / pu as f64,
        within
    );

    // Assert
    assert_eq!(uniform["within_tolerance"], true, "the uniform layout must reach the tolerance: {uniform}");
    assert_eq!(adaptive["within_tolerance"], true, "{adaptive}");
    assert_eq!(adaptive["closed"], true, "{adaptive}");
    let body = sa.doc.body(surfacecraft_doc::ObjectId(adaptive["body"]["id"].as_u64().unwrap())).unwrap();
    assert_eq!(body.brep.edges.iter().filter(|e| e.uses.len() != 2).count(), 0, "watertight: every edge has two faces");
    assert!(adaptive["deviation"]["within"].as_f64().unwrap() >= MIN_WITHIN, "{adaptive}");
    assert!(within >= MIN_WITHIN, "independent share within the tolerance {within}");
    assert!(
        (pa as f64) <= MAX_PATCH_SHARE * pu as f64,
        "adaptive {pa} patches vs uniform {pu}: more than {} %",
        100.0 * MAX_PATCH_SHARE
    );
}

#[test]
fn adaptive_wrap_is_deterministic_and_reports_its_levels() {
    // Arrange / Act
    let (_s1, a) = wrap("adaptive");
    let (_s2, b) = wrap("adaptive");

    // Assert
    assert_eq!(a["patches"], b["patches"]);
    assert_eq!(a["deviation"], b["deviation"]);
    let levels = a["levels"].as_array().unwrap();
    assert!(!levels.is_empty(), "{a}");
    assert_eq!(levels.len() as u64, a["rounds"].as_u64().unwrap());
    assert_eq!(a["refine"], "adaptive");
}

#[test]
fn unknown_refine_mode_is_refused() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", synth::sphere(Point3::origin(), 10.0, 24, 12));
    let r = s.execute("surface.wrap", json!({"mesh": "scan", "refine": "nonsense"}));
    assert!(r.is_err(), "{r:?}");
}
