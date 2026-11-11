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
    assert_eq!(counts.len(), 10, "{out}");
    assert_eq!(counts.iter().filter_map(Value::as_u64).sum::<u64>(), counted, "{out}");
    let bands = report["bands"].as_array().expect("bands");
    let tolerances: Vec<f64> = bands.iter().map(|b| b["tolerance"].as_f64().unwrap()).collect();
    for (got, want) in tolerances.iter().zip([0.01, 0.02, 0.04, 0.06]) {
        assert!((got - want).abs() < 1e-12, "{tolerances:?}");
    }
    let shares: Vec<f64> = bands.iter().map(|b| b["share"].as_f64().unwrap()).collect();
    assert_eq!(shares, vec![0.0, 0.0, 0.0, 1.0], "{out}");
}

/// Gate: the report's histogram spans ±3 tolerances (the legend's range) in the stats' own ten bins,
/// so it is the same histogram, computed in the same pass without sorting.
#[test]
fn gate_the_report_histogram_is_the_stats_histogram_over_three_tolerances() {
    let mut s = offset_plane(false);
    let out = map(&mut s, 0.02);
    assert_eq!(out["report"]["histogram"]["counts"], out["stats"]["histogram"], "{out}");
    let lo = out["report"]["histogram"]["lo"].as_f64().unwrap();
    let hi = out["report"]["histogram"]["hi"].as_f64().unwrap();
    assert!((lo + 0.06).abs() < 1e-12 && (hi - 0.06).abs() < 1e-12, "{out}");
}

/// Gate: a map is tied to the geometry it was computed on. Moving the scan drops it, so the reading
/// is an error rather than the old number, and undoing the move does not bring the map back.
#[test]
fn a_map_is_dropped_when_its_scan_moves_and_undo_does_not_restore_it() {
    let mut s = offset_plane(false);
    map(&mut s, 0.02);
    s.execute("mesh.transform", json!({"object": "scan", "translate": [0.0, 0.0, 1.0]})).expect("the move");
    assert!(s.maps.is_empty(), "the moved scan has no map");
    assert!(s.query("deviation.at", json!({"object": "scan", "triangle": 7})).is_err());
    s.execute("edit.undo", json!({})).expect("the move is undone");
    let back = s.query("deviation.at", json!({"object": "scan", "triangle": 7}));
    assert!(back.is_err(), "undo gives the old geometry, but no stale reading: {back:?}");
}

#[test]
fn doc_new_clears_the_maps() {
    let mut s = offset_plane(false);
    map(&mut s, 0.02);
    s.execute("doc.new", json!({})).expect("a new document");
    assert!(s.maps.is_empty());
    assert!(s.query("deviation.at", json!({"object": "scan", "triangle": 0})).is_err());
}

#[test]
fn doc_open_clears_the_maps() {
    let mut s = offset_plane(false);
    map(&mut s, 0.02);
    let path = std::env::temp_dir().join(format!("p8-a3-probe-{}.json", std::process::id()));
    let path_text = path.to_string_lossy().to_string();
    s.execute("doc.save", json!({"path": path_text})).expect("saved");
    map(&mut s, 0.02);
    s.execute("doc.open", json!({"path": path_text})).expect("opened");
    std::fs::remove_file(&path).ok();
    assert!(s.maps.is_empty(), "opening a document clears its maps");
}

#[test]
fn deleting_the_scan_drops_its_map() {
    let mut s = offset_plane(false);
    map(&mut s, 0.02);
    s.execute("doc.delete", json!({"object": "scan"})).expect("deleted");
    assert!(s.maps.is_empty());
}

/// Gate: the read path takes a shared session and runs only the listed reads: a command that
/// changes the document is refused, and the document is left alone.
#[test]
fn query_takes_a_shared_session_and_refuses_anything_but_the_listed_reads() {
    let mut s = offset_plane(false);
    map(&mut s, 0.02);
    let shared: &Session = &s;
    assert!(shared.query("deviation.at", json!({"object": "scan", "triangle": 0})).is_ok());
    assert!(matches!(shared.query("doc.delete", json!({"object": "scan"})), Err(EngineError::Unavailable(_))));
    assert!(matches!(
        shared.query("mesh.transform", json!({"object": "scan", "translate": [0.0, 0.0, 1.0]})),
        Err(EngineError::Unavailable(_))
    ));
    assert_eq!(s.doc.summaries().len(), 2, "the refused commands changed nothing");
    assert!(!s.maps.is_empty(), "the map is still there");
}

/// Gate: an output logged before the report existed still parses; its report is empty.
#[test]
fn a_compute_output_without_a_report_still_parses() {
    let mut s = offset_plane(false);
    let mut out = map(&mut s, 0.02);
    out.as_object_mut().expect("an object").remove("report");
    let old: surfacecraft_engine::cmd::deviation::ComputeOut =
        serde_json::from_value(out).expect("an output without a report parses");
    assert!(old.report.histogram.counts.is_empty() && old.report.bands.is_empty());
    assert!(old.stats.count > 0, "the statistics are kept");
}
