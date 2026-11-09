//! P8: the deviation toolkit (facing-aware signed deviation, statistics, hotspots, volume error and
//! the deviation from one primitive) on synthetic scans whose deviation is known exactly.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_engine::Session;
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Mesh, Point3, Vec3, synth};

/// A square grid of `n` × `n` cells over `[0, size]²` whose vertices sit at height `z(x, y)`. The
/// triangles are counter-clockwise seen from +Z, so the normals point up.
fn height_grid(n: usize, size: f64, z: impl Fn(f64, f64) -> f64) -> Mesh {
    let h = size / n as f64;
    let mut positions = Vec::with_capacity((n + 1) * (n + 1));
    for j in 0..=n {
        for i in 0..=n {
            let (x, y) = (i as f64 * h, j as f64 * h);
            positions.push(Point3::new(x, y, z(x, y)));
        }
    }
    let idx = |i: usize, j: usize| (j * (n + 1) + i) as u32;
    let mut triangles = Vec::with_capacity(2 * n * n);
    for j in 0..n {
        for i in 0..n {
            let (a, b, c, d) = (idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1));
            triangles.push([a, b, c]);
            triangles.push([a, c, d]);
        }
    }
    Mesh::new(positions, triangles).unwrap()
}

/// Gate: a sphere scan of radius 10.2 against a sphere body of radius 10 is 0.2 mm outside the
/// material everywhere, so the mean is +0.2 mm and the volume error is area × 0.2 mm. The area is
/// the scan's own (its radius is 10.2), because the volume error sums over scan triangles.
#[test]
fn gate_p8_sphere_scan_offset_mean_and_volume_error() {
    let mut s = Session::new();
    let scan_mesh = synth::sphere(Point3::origin(), 10.2, 128, 64);
    let scan_area = scan_mesh.area();
    s.doc.add_mesh("scan", scan_mesh);
    s.execute("body.sphere", json!({"radius": 10.0, "name": "ball"})).unwrap();
    let st = s.execute("deviation.stats", json!({"object": "scan", "against": "ball"})).unwrap();
    let mean = st["stats"]["mean"].as_f64().unwrap();
    eprintln!("p8 sphere body: mean {mean:.5} mm, not facing {}", st["stats"]["not_facing"]);
    assert!((mean - 0.2).abs() < 0.01, "{st}");
    assert_eq!(st["stats"]["not_facing"].as_u64(), Some(0), "{st}");
    let hs = s.execute("deviation.hotspots", json!({"object": "scan", "against": "ball", "tolerance": 0.1})).unwrap();
    let excess = hs["volume"]["excess_mm3"].as_f64().unwrap();
    let exact = 4.0 * PI * 10.2 * 10.2 * 0.2;
    eprintln!("p8 sphere body: excess {excess:.3} mm3, 4 pi r^2 0.2 = {exact:.3}, scan area {scan_area:.3}");
    assert!((excess - exact).abs() < 0.02 * exact, "{hs}");
    assert!(hs["volume"]["missing_mm3"].as_f64().unwrap().abs() < 1e-6, "{hs}");

    // the same against the exact primitive: the deviation is exactly 0.2 mm
    s.doc.add_primitive(
        "ball_exact",
        Primitive::Sphere { center: [0.0, 0.0, 0.0], radius: 10.0 },
        None,
        0.0,
        0.0,
        Mesh::default(),
    );
    let st = s.execute("deviation.stats", json!({"object": "scan", "against": "ball_exact"})).unwrap();
    assert!((st["stats"]["mean"].as_f64().unwrap() - 0.2).abs() < 1e-9, "{st}");
    assert!((st["stats"]["min"].as_f64().unwrap() - 0.2).abs() < 1e-9, "{st}");
    let hs =
        s.execute("deviation.hotspots", json!({"object": "scan", "against": "ball_exact", "tolerance": 0.1})).unwrap();
    let excess = hs["volume"]["excess_mm3"].as_f64().unwrap();
    // every triangle is 0.2 mm, so the volume is 0.2 × the footprint area (the scan area projected
    // on the sphere's radial normal, a little under the surface area)
    let footprint = hs["volume"]["area_mm2"].as_f64().unwrap();
    assert!(footprint <= scan_area && footprint > 0.999 * scan_area, "{hs}");
    assert!((excess - 0.2 * footprint).abs() < 1e-6 * excess, "{hs}");
    assert!((excess - exact).abs() < 0.02 * exact, "{hs}");
}

/// Gate: a 5 × 5 mm bump 0.5 mm high on a box top is one hotspot of 25 mm² at tolerance 0.2 mm.
/// The area is the footprint on the reference (the triangles projected along the box-top normal):
/// the bump's top, plus the border triangles whose corners average above the tolerance, about
/// 2.5 mm² at this 0.25 mm grid. The surface area is larger: the 0.5 mm step is sloped over one cell.
#[test]
fn gate_p8_box_bump_is_one_hotspot_of_25_mm2() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [30.0, 30.0, 10.0], "name": "block"})).unwrap();
    let inside = |v: f64| (10.0..=15.0).contains(&v);
    s.doc.add_mesh("scan", height_grid(120, 30.0, |x, y| 10.0 + if inside(x) && inside(y) { 0.5 } else { 0.0 }));
    let hs = s.execute("deviation.hotspots", json!({"object": "scan", "against": "block", "tolerance": 0.2})).unwrap();
    eprintln!(
        "p8 bump: {} group(s), footprint {}, surface {}",
        hs["total"], hs["hotspots"][0]["area"], hs["hotspots"][0]["surface_area"]
    );
    assert_eq!(hs["total"].as_u64(), Some(1), "{hs}");
    let g = &hs["hotspots"][0];
    let area = g["area"].as_f64().unwrap();
    assert!((area - 25.0).abs() <= 0.15 * 25.0, "{hs}");
    assert!(g["surface_area"].as_f64().unwrap() > area, "{hs}");
    assert_eq!(g["sign"], "positive", "{hs}");
    assert!((g["max"].as_f64().unwrap() - 0.5).abs() < 1e-9, "{hs}");
    assert!((g["mean"].as_f64().unwrap() - 0.5).abs() < 0.05, "{hs}");
    let c = g["centroid"].as_array().unwrap();
    assert!((c[0].as_f64().unwrap() - 12.5).abs() < 0.1 && (c[1].as_f64().unwrap() - 12.5).abs() < 0.1, "{hs}");
    // the bump is excess material: 25 mm² × 0.5 mm, up to the border triangles
    let excess = hs["volume"]["excess_mm3"].as_f64().unwrap();
    assert!((excess - 12.5).abs() < 0.15 * 12.5, "{hs}");
}

/// Gate: a 1 mm plate scanned on both faces, measured against its top face, never counts the
/// bottom side: its normals point away from the top normal, so those points are not facing.
#[test]
fn gate_p8_plate_bottom_is_never_measured_against_top() {
    let mut s = Session::new();
    let top = synth::plane(Point3::new(0.0, 0.0, 1.0), Vec3::x(), Vec3::y(), 30.0, 30.0, 20, 20);
    let bottom = synth::plane(Point3::origin(), Vec3::y(), Vec3::x(), 30.0, 30.0, 20, 20);
    let (top_tris, nt, nb) = (top.triangle_count(), top.vertex_count(), bottom.vertex_count());
    let mut plate = top.clone();
    plate.append(&bottom);
    let plate_id = s.doc.add_mesh("plate", plate);
    s.doc.add_mesh("top", top);
    let st = s.execute("deviation.stats", json!({"object": "plate", "against": "top"})).unwrap();
    eprintln!("p8 plate vs top mesh: {st}");
    assert_eq!(st["stats"]["count"].as_u64(), Some(nt as u64), "{st}");
    assert_eq!(st["stats"]["not_facing"].as_u64(), Some(nb as u64), "{st}");
    assert!(st["stats"]["max"].as_f64().unwrap().abs() < 1e-9, "{st}");
    // switched off, the bottom side is measured: 1 mm below the top face
    let off = s.execute("deviation.stats", json!({"object": "plate", "against": "top", "facing": false})).unwrap();
    assert_eq!(off["stats"]["count"].as_u64(), Some((nt + nb) as u64), "{off}");
    assert!((off["stats"]["min"].as_f64().unwrap() + 1.0).abs() < 1e-9, "{off}");

    // a fitted plane judged by its region: its normal is given the wrong way round on purpose and
    // is turned to the region's scan normals, so the top side still counts and the bottom does not
    let region = s.doc.add_region("top_region", plate_id, (0..top_tris as u32).collect());
    let plane = s.doc.add_primitive(
        "top_plane",
        Primitive::Plane { point: [0.0, 0.0, 1.0], normal: [0.0, 0.0, -1.0] },
        Some(region),
        0.0,
        0.0,
        Mesh::default(),
    );
    let st = s.execute("deviation.stats", json!({"object": "plate", "against": plane})).unwrap();
    assert_eq!(st["stats"]["count"].as_u64(), Some(nt as u64), "{st}");
    assert_eq!(st["stats"]["not_facing"].as_u64(), Some(nb as u64), "{st}");
    let tp = s.execute("deviation.to_primitive", json!({"region": "top_region", "primitive": "top_plane"})).unwrap();
    assert_eq!(tp["kind"], "plane", "{tp}");
    assert_eq!(tp["stats"]["count"].as_u64(), Some(nt as u64), "{tp}");
    assert!(tp["stats"]["mean"].as_f64().unwrap().abs() < 1e-9, "{tp}");
}

/// Histogram counts sum to the counted points (values outside the range fall in the end bins),
/// percentiles are ordered, and the bands are nested shares.
#[test]
fn gate_p8_histogram_counts_sum_to_the_counted_points() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [30.0, 30.0, 10.0], "name": "block"})).unwrap();
    let inside = |v: f64| (10.0..=15.0).contains(&v);
    s.doc.add_mesh("scan", height_grid(60, 30.0, |x, y| 10.0 + if inside(x) && inside(y) { 0.5 } else { 0.0 }));
    let st = s.execute("deviation.stats", json!({"object": "scan", "against": "block"})).unwrap();
    let count = st["stats"]["count"].as_u64().unwrap();
    let counts: Vec<u64> =
        st["stats"]["histogram"]["counts"].as_array().unwrap().iter().filter_map(Value::as_u64).collect();
    assert_eq!(counts.len(), 20);
    assert_eq!(counts.iter().sum::<u64>(), count, "{st}");
    let p = &st["stats"]["percentiles"];
    let ps: Vec<f64> = ["p1", "p5", "p50", "p95", "p99"].iter().filter_map(|k| p[*k].as_f64()).collect();
    assert_eq!(ps.len(), 5);
    assert!(ps.windows(2).all(|w| w[0] <= w[1]), "{st}");
    let shares: Vec<f64> =
        st["stats"]["bands"].as_array().unwrap().iter().filter_map(|b| b["share"].as_f64()).collect();
    assert!(shares.windows(2).all(|w| w[0] <= w[1]) && shares.iter().all(|x| (0.0..=1.0).contains(x)), "{st}");
    // a narrow explicit range puts the outliers in the end bins
    let narrow = s
        .execute("deviation.stats", json!({"object": "scan", "against": "block", "bins": 4, "range": [-0.1, 0.1]}))
        .unwrap();
    let nc: Vec<u64> =
        narrow["stats"]["histogram"]["counts"].as_array().unwrap().iter().filter_map(Value::as_u64).collect();
    assert_eq!(nc.len(), 4);
    assert_eq!(nc.iter().sum::<u64>(), count, "{narrow}");
    assert!(nc[3] > 0, "{narrow}");
}

/// A scan whose normals are reversed (the scan side faces the air) is not facing anywhere: nothing
/// is counted, and the colour map leaves every point grey.
#[test]
fn p8_scan_with_inward_normals_counts_nothing() {
    let mut s = Session::new();
    let mut sphere = synth::sphere(Point3::origin(), 10.2, 48, 24);
    for t in &mut sphere.triangles {
        t.swap(1, 2);
    }
    let n = sphere.vertex_count() as u64;
    s.doc.add_mesh("inside_out", sphere);
    s.doc.add_primitive(
        "ball",
        Primitive::Sphere { center: [0.0, 0.0, 0.0], radius: 10.0 },
        None,
        0.0,
        0.0,
        Mesh::default(),
    );
    let st = s.execute("deviation.compute", json!({"object": "inside_out", "against": "ball"})).unwrap();
    assert_eq!(st["stats"]["count"].as_u64(), Some(0), "{st}");
    assert_eq!(st["stats"]["not_facing"].as_u64(), Some(n), "{st}");
}

/// Hostile parameters are refused with a message, never a panic or an internal error.
#[test]
fn p8_hostile_parameters_are_refused() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [30.0, 30.0, 10.0], "name": "block"})).unwrap();
    s.doc.add_mesh("scan", height_grid(4, 30.0, |_, _| 10.0));
    let base = json!({"object": "scan", "against": "block"});
    let with = |extra: Value| {
        let mut v = base.clone();
        for (k, x) in extra.as_object().unwrap() {
            v[k] = x.clone();
        }
        v
    };
    let bad = [
        ("deviation.stats", with(json!({"bins": 0}))),
        ("deviation.stats", with(json!({"bins": 5000}))),
        ("deviation.stats", with(json!({"range": [1.0, 0.0]}))),
        ("deviation.stats", with(json!({"bands": [-1.0]}))),
        ("deviation.stats", with(json!({"bands": vec![0.1; 100]}))),
        ("deviation.stats", with(json!({"max_distance": 0.0}))),
        ("deviation.hotspots", with(json!({"tolerance": 0.0}))),
        ("deviation.hotspots", with(json!({"min_area": -1.0}))),
        ("deviation.hotspots", with(json!({"limit": 20000}))),
        ("deviation.compute", with(json!({"tolerance": -0.1}))),
        ("deviation.to_primitive", json!({"region": "nope", "primitive": "nope"})),
        ("deviation.to_primitive", json!({"region": "scan", "primitive": "block"})),
    ];
    for (id, params) in bad {
        let r = s.execute(id, params.clone());
        assert!(r.is_err(), "{id} accepted {params}");
        assert!(!matches!(r, Err(surfacecraft_engine::EngineError::Internal(_))), "{id} panicked on {params}");
    }
}
