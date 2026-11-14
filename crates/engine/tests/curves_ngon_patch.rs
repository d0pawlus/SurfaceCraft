//! `surface.from_curves` with `ngon: "patch"`: regions that are not four-sided become single
//! trimmed patches while the four-sided ones stay Bézier patches. A cube with one corner cut off
//! by the plane x + y + z = CUT has three quadrilateral, three pentagonal and one triangular
//! face; every face bulges outward by a smooth, analytic dome that vanishes (with its slope) at
//! the face edges, so the faces are curved and meet along the sharp edges of the polyhedron.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

/// Half edge of the cube (mm) and the position of the cutting plane x + y + z = CUT (it removes
/// only the corner (HALF, HALF, HALF)).
const HALF: f64 = 20.0;
const CUT: f64 = 40.0;
/// Height of the domes (mm).
const BULGE: f64 = 2.0;
/// Subdivision levels of a face for the scan.
const MESH_LEVELS: usize = 5;

type Polygon = Vec<Vec3>;

/// Clips `poly` to the half-space `n · p <= d` (Sutherland–Hodgman).
fn clip(poly: &Polygon, n: &Vec3, d: f64) -> Polygon {
    let mut out = Vec::new();
    for k in 0..poly.len() {
        let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
        let (sa, sb) = (n.dot(&a) - d, n.dot(&b) - d);
        if sa <= 0.0 {
            out.push(a);
        }
        if (sa < 0.0 && sb > 0.0) || (sa > 0.0 && sb < 0.0) {
            out.push(a + (b - a) * (sa / (sa - sb)));
        }
    }
    out
}

/// The faces of the cut cube as convex polygons, counter-clockwise seen from outside, with
/// their outward normals.
fn faces() -> Vec<(Polygon, Vec3)> {
    let planes: Vec<(Vec3, f64)> = vec![
        (Vec3::x(), HALF),
        (-Vec3::x(), HALF),
        (Vec3::y(), HALF),
        (-Vec3::y(), HALF),
        (Vec3::z(), HALF),
        (-Vec3::z(), HALF),
        (Vec3::new(1.0, 1.0, 1.0).normalize(), CUT / 3f64.sqrt()),
    ];
    let mut out = Vec::new();
    for (i, (n, d)) in planes.iter().enumerate() {
        let u = if n.x.abs() < 0.9 { Vec3::x() } else { Vec3::y() };
        let e1 = (u - n * u.dot(n)).normalize();
        let e2 = n.cross(&e1);
        let c = n * *d;
        let big = 200.0;
        let mut poly: Polygon =
            vec![c - e1 * big - e2 * big, c + e1 * big - e2 * big, c + e1 * big + e2 * big, c - e1 * big + e2 * big];
        for (j, (m, dm)) in planes.iter().enumerate() {
            if i != j {
                poly = clip(&poly, m, *dm);
            }
        }
        if poly.len() >= 3 {
            out.push((poly, *n));
        }
    }
    out
}

/// Distances of `p` from the edge lines of the convex polygon `poly` (inside: all positive).
fn edge_distances(poly: &Polygon, n: &Vec3, p: &Vec3) -> Vec<f64> {
    (0..poly.len())
        .map(|k| {
            let (a, b) = (poly[k], poly[(k + 1) % poly.len()]);
            let inward = n.cross(&(b - a)).normalize();
            (p - a).dot(&inward)
        })
        .collect()
}

/// Height of the dome over the face at `p`: a product of C¹ ramps of the edge distances.
fn dome(poly: &Polygon, n: &Vec3, scale: f64, p: &Vec3) -> f64 {
    BULGE
        * edge_distances(poly, n, p)
            .iter()
            .map(|d| (std::f64::consts::FRAC_PI_2 * (d / scale).clamp(0.0, 1.0)).sin().powi(2))
            .product::<f64>()
}

/// Midpoint subdivision of the fan of `poly` around its centroid: triangles of about 1 mm.
fn fan_triangles(poly: &Polygon, levels: usize) -> Vec<[Vec3; 3]> {
    let centre = poly.iter().sum::<Vec3>() / poly.len() as f64;
    let mut tris: Vec<[Vec3; 3]> = (0..poly.len()).map(|k| [centre, poly[k], poly[(k + 1) % poly.len()]]).collect();
    for _ in 0..levels {
        tris = tris
            .iter()
            .flat_map(|[a, b, c]| {
                let (ab, bc, ca) = ((a + b) * 0.5, (b + c) * 0.5, (c + a) * 0.5);
                [[*a, ab, ca], [ab, *b, bc], [ca, bc, *c], [ab, bc, ca]]
            })
            .collect();
    }
    tris
}

fn inradius(poly: &Polygon, n: &Vec3) -> f64 {
    let c = poly.iter().sum::<Vec3>() / poly.len() as f64;
    edge_distances(poly, n, &c).into_iter().fold(f64::INFINITY, f64::min)
}

/// The part as a triangle mesh.
fn part() -> Mesh {
    let mut mesh = Mesh { positions: Vec::new(), triangles: Vec::new() };
    for (poly, n) in faces() {
        let scale = 0.95 * inradius(&poly, &n);
        let mut face = Mesh { positions: Vec::new(), triangles: Vec::new() };
        for [a, b, c] in fan_triangles(&poly, MESH_LEVELS) {
            let base = face.positions.len() as u32;
            for p in [a, b, c] {
                face.positions.push(Point3::from(p + n * dome(&poly, &n, scale, &p)));
            }
            face.triangles.push([base, base + 1, base + 2]);
        }
        // wind outward
        for t in &mut face.triangles {
            let (a, b, c) =
                (face.positions[t[0] as usize], face.positions[t[1] as usize], face.positions[t[2] as usize]);
            if (b - a).cross(&(c - a)).dot(&n) < 0.0 {
                t.swap(1, 2);
            }
        }
        mesh.append(&face);
    }
    mesh.weld(1e-9);
    mesh
}

/// A session holding the noisy, cleaned scan of the part as the mesh `scan`.
fn scan_session() -> Session {
    let mut s = Session::new();
    let defects = synth::ScanDefects {
        spacing: 0.9,
        density_ramp: 0.6,
        noise: 0.03,
        holes: 2,
        hole_radius: 1.5,
        spikes: 0.0003,
        debris: 3,
    };
    s.doc.add_mesh("scan", synth::scanify(&part(), &defects, &mut Rng::new(21)));
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    assert_eq!(c["holes_left"], 0, "{c}");
    s
}

/// A small cube (six four-sided regions) as the mesh `cube`.
fn cube_session() -> Session {
    let mut s = Session::new();
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(10.0, 12.0, 8.0), 6);
    m.weld(1e-9);
    s.doc.add_mesh("cube", m);
    s
}

/// `out` without the object ids (they depend on what else the session holds).
fn without_ids(mut out: Value) -> Value {
    if let Some(o) = out.as_object_mut() {
        o.remove("surface");
        o.remove("solid");
        if let Some(Value::Array(patches)) = o.get_mut("trimmed_patches") {
            for p in patches {
                if let Some(po) = p.as_object_mut() {
                    po.remove("surface");
                }
            }
        }
    }
    out
}

/// The creases of 54.7° between the cut face and its neighbours are blurred by the resampling of
/// the scan, so the sharp-edge angle is lowered to 20°.
const ANGLE: f64 = 20.0;

#[test]
fn from_curves_ngon_patch_fills_triangle_and_pentagon() {
    let quads = faces().iter().filter(|f| f.0.len() == 4).count();
    let mut s = scan_session();
    let r = s.execute("surface.from_curves", json!({"mesh": "scan", "angle": ANGLE, "ngon": "patch"})).unwrap();
    eprintln!("from_curves ngon patch: {r}");
    assert_eq!(r["regions"], faces().len(), "{r}");
    assert!(r["skipped"].as_array().unwrap().is_empty(), "{r}");
    let patches = r["trimmed_patches"].as_array().unwrap();
    assert!(patches.len() >= 2, "{r}");
    let corners: Vec<u64> = patches.iter().map(|p| p["corners"].as_u64().unwrap()).collect();
    assert!(corners.contains(&3) && corners.contains(&5), "corners {corners:?}");
    for p in patches {
        assert!(p["max"].as_f64().unwrap() <= 0.2, "{p}");
        assert!(p["side_gap_max_mm"].as_f64().unwrap().is_finite(), "{p}");
        // the fit measured the region's vertices (a patch that measured none is not within tolerance)
        assert!(p["points"].as_u64().unwrap() > 50, "{p}");
        assert!(p["outside"].as_u64().is_some(), "{p}");
        assert_eq!(p["within_tolerance"], true, "{p}");
    }
    assert!(r["side_gap_max_mm"].as_f64().is_some(), "{r}");
    assert!(r["max"].as_f64().unwrap() <= 0.2, "{r}");
    assert_eq!(r["within_tolerance"], true, "{r}");
    // no global split: one Bézier patch per four-cornered region
    assert_eq!(r["patches"].as_u64().unwrap() as usize, quads, "{r}");
    // separate sheets: no solid
    assert_eq!(r["closed"], false, "{r}");
    assert!(r["solid"].is_null(), "{r}");
    assert!(r["warning"].as_str().unwrap().contains("single trimmed patches"), "{r}");
}

/// The default of `ngon` is the old behaviour, pinned against what `surface.from_curves` gave before
/// the option existed (and still gives under `split`): one Bézier patch per four-sided region, a
/// closed network with a solid, no trimmed patches. (`gate_p8_h9_from_curves_ngon_regions` in
/// `curves_ngon.rs` and `gate_p4` in `organic.rs` pin the same default on the n-sided cube and on a
/// box: closed, a solid, volume within 0.1 % / 1 %.)
#[test]
fn from_curves_default_keeps_the_old_closed_network() {
    let mut s = cube_session();
    let r = s.execute("surface.from_curves", json!({"mesh": "cube", "min_triangles": 4, "degree": 3})).unwrap();
    assert_eq!(r["patches"], 6, "{r}");
    assert_eq!(r["curves"], 12, "{r}");
    assert_eq!(r["closed"], true, "{r}");
    assert!(r["solid"].as_u64().is_some(), "{r}");
    assert!(r["trimmed_patches"].is_null() && r["side_gap_max_mm"].is_null(), "{r}");
    assert!(r["volume"].as_f64().unwrap() > 0.0, "{r}");
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn a_cancelled_from_curves_with_patches_adds_nothing() {
    use std::sync::Arc;
    use std::time::{Duration, Instant};
    use surfacecraft_engine::{EngineError, Job, JobPoll, cmd};
    let s = scan_session();
    let before = s.doc.surfaces.len();
    let params = json!({"mesh": "scan", "angle": ANGLE, "ngon": "patch"});
    let job = Job::start(&s, "surface.from_curves", Arc::new(params)).unwrap();
    job.cancel();
    let started = Instant::now();
    let done = loop {
        match job.poll() {
            JobPoll::Finished(d) => break *d,
            JobPoll::Lost => panic!("the job was lost"),
            JobPoll::Running => {
                assert!(started.elapsed() < Duration::from_secs(300));
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    };
    assert_eq!(done.result, Err(EngineError::Failed(cmd::CANCELLED.into())));
    assert_eq!(done.session.doc.surfaces.len(), before, "a cancelled command is rolled back");
}

#[test]
fn from_curves_ngon_patch_on_a_four_sided_network_is_the_same_network() {
    let mut s = cube_session();
    let args = |ngon: &str| json!({"mesh": "cube", "min_triangles": 4, "degree": 3, "ngon": ngon, "name": ngon});
    let split = s.execute("surface.from_curves", args("split")).unwrap();
    let patch = s.execute("surface.from_curves", args("patch")).unwrap();
    assert_eq!(split["patches"], 6, "{split}");
    assert!(patch["trimmed_patches"].is_null(), "{patch}");
    assert_eq!(without_ids(split), without_ids(patch));
}

#[test]
fn from_curves_ngon_rejects_unknown_value() {
    let mut s = cube_session();
    let e = s.execute("surface.from_curves", json!({"mesh": "cube", "ngon": "banana"}));
    assert!(e.is_err(), "{e:?}");
}

#[test]
fn from_curves_ngon_hostile_values_are_errors() {
    let mut s = cube_session();
    for bad in [
        json!(null),
        json!(1),
        json!(-1.5),
        json!(1e300),
        json!(true),
        json!([]),
        json!(["patch"]),
        json!({}),
        json!({"ngon": "patch"}),
        json!(""),
        json!("PATCH"),
        json!("patch "),
        json!("split\u{0}"),
        json!("\u{1F600}"),
    ] {
        let e = s.execute("surface.from_curves", json!({"mesh": "cube", "ngon": bad.clone()}));
        assert!(e.is_err(), "ngon {bad}: {e:?}");
    }
    // the session is still usable afterwards
    let ok = s.execute("surface.from_curves", json!({"mesh": "cube", "min_triangles": 4, "degree": 3}));
    assert!(ok.is_ok(), "{ok:?}");
}
