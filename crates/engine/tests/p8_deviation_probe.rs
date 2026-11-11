//! P8-A3: the deviation probe behind the viewport readout (`deviation.at`), the query path that
//! keeps per-frame reads out of the command log, and the histogram and bands that a deviation map
//! reports, on synthetic scans whose deviation is known exactly.
use serde_json::{Value, json};
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_fit::Primitive;
use surfacecraft_geom::Mesh;

/// A square grid of `n` × `n` cells over `[0, size]²` whose vertices sit at height `z(x, y)`. The
/// triangles are counter-clockwise seen from +Z, so the normals point up; `flip` turns them down.
fn height_grid(n: usize, size: f64, z: impl Fn(f64, f64) -> f64, flip: bool) -> Mesh {
    let h = size / n as f64;
    let mut positions = Vec::with_capacity((n + 1) * (n + 1));
    for j in 0..=n {
        for i in 0..=n {
            let (x, y) = (i as f64 * h, j as f64 * h);
            positions.push(surfacecraft_geom::Point3::new(x, y, z(x, y)));
        }
    }
    let idx = |i: usize, j: usize| (j * (n + 1) + i) as u32;
    let mut triangles = Vec::with_capacity(2 * n * n);
    for j in 0..n {
        for i in 0..n {
            let (a, b, c, d) = (idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1));
            if flip {
                triangles.push([a, c, b]);
                triangles.push([a, d, c]);
            } else {
                triangles.push([a, b, c]);
                triangles.push([a, c, d]);
            }
        }
    }
    Mesh::new(positions, triangles).expect("a valid grid")
}

/// A 40 mm scan plane 0.05 mm above the reference plane z = 0 (a plane primitive named `ref`).
fn offset_plane(flip: bool) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("scan", height_grid(20, 40.0, |_, _| 0.05, flip));
    s.doc.add_primitive(
        "ref",
        Primitive::Plane { point: [0.0, 0.0, 0.0], normal: [0.0, 0.0, 1.0] },
        None,
        0.0,
        0.0,
        Mesh::default(),
    );
    s
}

fn map(s: &mut Session, tolerance: f64) -> Value {
    s.execute("deviation.compute", json!({"object": "scan", "against": "ref", "tolerance": tolerance}))
        .expect("the deviation map")
}

/// Gate: every corner of the scan lies 0.05 mm outside the reference, so triangle 7 reads +0.05 mm
/// (the mean of its three corners) and agrees with the map's own mean.
#[test]
fn gate_deviation_at_reads_the_mean_of_the_picked_triangle() {
    let mut s = offset_plane(false);
    let out = map(&mut s, 0.02);
    let at = s.query("deviation.at", json!({"object": "scan", "triangle": 7})).expect("a reading");
    assert_eq!(at["counted_corners"], 3, "{at}");
    let value = at["deviation"].as_f64().expect("a deviation");
    assert!((value - 0.05).abs() < 1e-6, "{at}");
    assert!((value - out["stats"]["mean"].as_f64().unwrap()).abs() < 1e-6, "{at} against {out}");
}

#[test]
fn deviation_at_names_the_missing_map_and_forgets_it_when_cleared() {
    let mut s = offset_plane(false);
    let before = s.query("deviation.at", json!({"object": "scan", "triangle": 0}));
    assert!(matches!(before, Err(EngineError::Failed(ref m)) if m.contains("deviation.compute")), "{before:?}");
    map(&mut s, 0.02);
    assert!(s.query("deviation.at", json!({"object": "scan", "triangle": 0})).is_ok());
    s.execute("deviation.clear", json!({"object": "scan"})).expect("the map is cleared");
    assert!(s.query("deviation.at", json!({"object": "scan", "triangle": 0})).is_err());
}

#[test]
fn deviation_at_rejects_triangles_outside_the_mesh_and_unknown_objects() {
    let mut s = offset_plane(false);
    map(&mut s, 0.02);
    for triangle in [800_u64, u64::from(u32::MAX)] {
        let r = s.query("deviation.at", json!({"object": "scan", "triangle": triangle}));
        assert!(matches!(r, Err(EngineError::InvalidParams(_)) | Err(EngineError::Failed(_))), "{triangle}: {r:?}");
    }
    assert!(s.query("deviation.at", json!({"object": "nowhere", "triangle": 0})).is_err());
    assert!(s.query("deviation.at", json!({"object": "scan", "triangle": 0, "extra": 1})).is_err());
}

/// Gate: normals pointing down are not facing the reference, so no corner is counted: the reading
/// is null, the same grey the colour map paints there. (A plane primitive takes the scan's side, so
/// the reference is a mesh with normals pointing up.)
#[test]
fn gate_deviation_at_is_null_where_no_corner_is_counted() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", height_grid(20, 40.0, |_, _| 0.05, true));
    s.doc.add_mesh("ref", height_grid(20, 40.0, |_, _| 0.0, false));
    s.execute("deviation.compute", json!({"object": "scan", "against": "ref", "tolerance": 0.02})).expect("the map");
    let at = s.query("deviation.at", json!({"object": "scan", "triangle": 3})).expect("a reading");
    assert_eq!(at["counted_corners"], 0, "{at}");
    assert!(at["deviation"].is_null(), "{at}");
}

#[test]
fn query_runs_reads_without_a_log_entry_and_refuses_commands_that_change_things() {
    let mut s = offset_plane(false);
    map(&mut s, 0.02);
    let logged = s.log.len();
    s.query("deviation.at", json!({"object": "scan", "triangle": 1})).expect("a reading");
    assert_eq!(s.log.len(), logged, "a query leaves the log alone");
    s.execute("deviation.at", json!({"object": "scan", "triangle": 1})).expect("the same reading as a command");
    assert_eq!(s.log.len(), logged + 1, "a command is logged");
    assert!(matches!(
        s.query("deviation.compute", json!({"object": "scan", "against": "ref"})),
        Err(EngineError::Unavailable(_))
    ));
    assert!(matches!(s.query("mesh.synth", json!({"shape": "plane"})), Err(EngineError::Unavailable(_))));
    assert_eq!(s.log.len(), logged + 1, "refused queries are not logged either");
}

/// Gate: the report of a map holds every counted point in its histogram, and its bands are the
/// cumulative shares of |deviation|. With every point at +0.05 mm and tolerance 0.02 mm the bands
/// at 0.01, 0.02, 0.04 and 0.06 mm hold 0, 0, 0 and all of the points.
#[test]
fn gate_the_map_reports_its_histogram_and_cumulative_bands() {
    let mut s = offset_plane(false);
    let out = map(&mut s, 0.02);
    let report = &out["report"];
    let counted = report["count"].as_u64().expect("the counted points");
    assert_eq!(counted, out["stats"]["count"].as_u64().unwrap(), "{out}");
    let counts = report["histogram"]["counts"].as_array().expect("bins");
    assert_eq!(counts.len(), 20, "{out}");
    assert_eq!(counts.iter().filter_map(Value::as_u64).sum::<u64>(), counted, "{out}");
    let bands = report["bands"].as_array().expect("bands");
    let tolerances: Vec<f64> = bands.iter().map(|b| b["tolerance"].as_f64().unwrap()).collect();
    for (got, want) in tolerances.iter().zip([0.01, 0.02, 0.04, 0.06]) {
        assert!((got - want).abs() < 1e-12, "{tolerances:?}");
    }
    let shares: Vec<f64> = bands.iter().map(|b| b["share"].as_f64().unwrap()).collect();
    assert_eq!(shares, vec![0.0, 0.0, 0.0, 1.0], "{out}");
}
