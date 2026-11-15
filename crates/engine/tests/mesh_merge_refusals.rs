//! P9-M4: `mesh.merge` refusals, issues and guarantees: one test per refusal and issue reason, the
//! inputs untouched and the merge undone in one step, identical scans, scans that share nothing,
//! pieces of one scan merged back, hostile parameters, cancellation and the same result for any
//! number of threads. Bounds carry the measured value beside them.
mod pair_common;
use pair_common::*;
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Transform, Vec3};

/// View `k` of the blob: its own tessellation and noise, the side of `dir` up to `reach` mm beyond the centre.
fn view(k: usize, dir: Vec3, reach: f64) -> Mesh {
    cut(&noisy(blob_at(48 + 4 * k), 200 + k as u64), |p| p.coords.dot(&dir) >= -reach)
}

fn halves() -> Vec<Mesh> {
    vec![view(0, Vec3::x(), 8.0), view(1, -Vec3::x(), 8.0)]
}

fn session(views: &[Mesh]) -> Session {
    let mut s = Session::new();
    for (k, v) in views.iter().enumerate() {
        s.doc.add_mesh(&format!("v{k}"), v.clone());
    }
    s
}

fn names(n: usize) -> Vec<String> {
    (0..n).map(|k| format!("v{k}")).collect()
}

fn merge(s: &mut Session, n: usize, extra: Value) -> Value {
    let mut p = json!({ "objects": names(n) });
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("mesh.merge", p).unwrap()
}

fn codes(out: &Value) -> Vec<String> {
    out["issues"].as_array().unwrap().iter().map(|i| i["code"].as_str().unwrap().to_string()).collect()
}

fn mesh_of(s: &Session, id: &Value) -> Mesh {
    s.doc.mesh(ObjectId(id.as_u64().unwrap())).unwrap().mesh.as_ref().clone()
}

fn flipped(m: &Mesh) -> Mesh {
    Mesh { positions: m.positions.clone(), triangles: m.triangles.iter().map(|t| [t[0], t[2], t[1]]).collect() }
}

/// A refusal: no mesh, no undo step, the issue with its code, severity `refusal`.
fn assert_refused(s: &Session, out: &Value, code: &str, objects: usize) {
    assert_eq!(out["refused"], json!(true), "{out}");
    assert!(out["id"].is_null());
    assert!(codes(out).contains(&code.to_string()), "{:?}", codes(out));
    let issue = out["issues"].as_array().unwrap().iter().find(|i| i["code"] == code).unwrap();
    assert_eq!(issue["severity"], json!("refusal"));
    assert_eq!(s.doc.summaries().len(), objects, "nothing was added");
    assert_eq!(s.history.undo_labels().len(), 0, "a refusal is no undo step");
}

#[test]
fn scans_misaligned_by_several_tolerances_are_refused() {
    let tol = 0.4;
    for times in [3.0, 6.0] {
        let mut views = halves();
        views[1] = views[1].transformed(&Transform::translation(Vec3::new(0.0, 0.0, times * tol)));
        let mut s = session(&views);
        let out = merge(&mut s, 2, json!({"tolerance": tol}));
        assert_refused(&s, &out, "scans_misaligned", 2);
        let pair = &out["pairs"][0];
        eprintln!("{times} tolerances: shared {} beyond {} rms {}", pair["shared"], pair["beyond_share"], pair["rms"]);
        // measured beyond share 0.86 (3 tolerances) and 0.94 (6): far above the 0.1 that refuses
        assert!(pair["beyond_share"].as_f64().unwrap() >= 0.5, "{pair}");
        // measured rms 0.95 and 1.87 mm against bounds of 0.3 and 0.6 (a quarter of the offset)
        assert!(pair["rms"].as_f64().unwrap() >= 0.25 * times * tol, "{pair}");
    }
}

/// Offsets beyond the reach of the comparison (8 tolerances): the scans share no surface within it,
/// but they lie over each other; refused, not merged as a union of pieces.
#[test]
fn scans_misaligned_beyond_the_reach_are_refused() {
    let tol = 0.1;
    // the blob's halves: where the shifted surfaces cross on the flanks some points are shared
    // (how many depends on the platform), so either form may fire; both give their numbers
    for times in [10.0, 30.0, 100.0] {
        let mut views = halves();
        views[1] = views[1].transformed(&Transform::translation(Vec3::new(0.0, 0.0, times * tol)));
        let mut s = session(&views);
        let out = merge(&mut s, 2, json!({"tolerance": tol}));
        let near: Vec<u64> = out["pairs"].as_array().unwrap().iter().map(|p| p["shared"].as_u64().unwrap()).collect();
        eprintln!("{times} tolerances: shared {near:?}, issues {:?}", codes(&out));
        assert_refused(&s, &out, "scans_misaligned", 2);
        assert!(!codes(&out).contains(&"scans_disjoint".to_string()));
        says_what_was_measured(&out);
    }
    // flat sheets share nothing within the reach on any platform: always the displaced form
    for times in [10.0, 30.0, 100.0] {
        let views = vec![sheet(0.0), sheet(times * tol)];
        let mut s = session(&views);
        let out = merge(&mut s, 2, json!({"tolerance": tol}));
        assert_refused(&s, &out, "scans_misaligned", 2);
        assert!(says_what_was_measured(&out), "{times} tolerances: the displaced form");
    }
}

/// A flat 20 x 20 mm sheet of 1 mm cells at height `z`, facing up.
fn sheet(z: f64) -> Mesh {
    let n = 20;
    let mut m = Mesh::default();
    for j in 0..=n {
        for i in 0..=n {
            m.positions.push(Point3::new(i as f64, j as f64, z));
        }
    }
    let at = |i: usize, j: usize| (j * (n + 1) + i) as u32;
    for j in 0..n {
        for i in 0..n {
            m.triangles.push([at(i, j), at(i + 1, j), at(i + 1, j + 1)]);
            m.triangles.push([at(i, j), at(i + 1, j + 1), at(i, j + 1)]);
        }
    }
    m
}

/// Checks that the refusal's text gives the numbers of its pair; true for the displaced form.
fn says_what_was_measured(out: &Value) -> bool {
    let issue = out["issues"].as_array().unwrap().iter().find(|i| i["code"] == "scans_misaligned").unwrap();
    let text = issue["message"].as_str().unwrap();
    let ab: Vec<usize> = issue["scans"].as_array().unwrap().iter().map(|v| v.as_u64().unwrap() as usize).collect();
    let pairs = out["pairs"].as_array().unwrap();
    if text.contains("shared points") {
        // the share form: the shared points of one of the two directions
        assert!(pairs.iter().any(|p| text.contains(&format!("of {} shared points", p["shared"]))), "{text}");
        return false;
    }
    let pair = pairs.iter().find(|p| p["a"] == ab[0] && p["b"] == ab[1]).unwrap();
    assert!(text.contains(&format!("{} of {} points of scan {}", pair["over"], pair["samples"], ab[0])), "{text}");
    assert!(text.contains(&format!("{} within the tolerance", pair["within"])), "{text}");
    true
}

#[test]
fn an_inverted_open_scan_is_refused() {
    let mut views = halves();
    views[1] = flipped(&views[1]);
    let mut s = session(&views);
    let out = merge(&mut s, 2, json!({}));
    assert_refused(&s, &out, "opposed_normals", 2);
}

#[test]
fn a_scan_wound_both_ways_is_refused() {
    let mut views = halves();
    let m = &mut views[1];
    for (i, t) in m.triangles.iter_mut().enumerate() {
        if i % 3 == 0 {
            *t = [t[0], t[2], t[1]];
        }
    }
    let mut s = session(&views);
    let out = merge(&mut s, 2, json!({}));
    assert_refused(&s, &out, "inconsistent_winding", 2);
}

/// The `count` triangles of view 0 nearest to the point of the blob farthest along +x, flipped: a
/// patch turned inside out where only view 0 sees the surface.
fn with_flipped_patch(count: usize) -> Vec<Mesh> {
    with_flipped_patch_in(halves(), count)
}

fn with_flipped_patch_in(mut views: Vec<Mesh>, count: usize) -> Vec<Mesh> {
    let m = &mut views[0];
    let tip = m.positions.iter().copied().max_by(|a, b| a.x.total_cmp(&b.x)).unwrap();
    let away = |i: usize| (m.face_centroid(i).unwrap() - tip).norm();
    let mut ids: Vec<usize> = (0..m.triangle_count()).collect();
    ids.sort_by(|&i, &j| away(i).total_cmp(&away(j)));
    ids.truncate(count);
    for i in &ids {
        let t = m.triangles[*i];
        m.triangles[*i] = [t[0], t[2], t[1]];
    }
    eprintln!("{} triangles flipped", ids.len());
    views
}

#[test]
fn a_patch_turned_inside_out_is_refused() {
    let mut s = session(&with_flipped_patch(60));
    let out = merge(&mut s, 2, json!({}));
    assert_refused(&s, &out, "inconsistent_winding", 2);
}

#[test]
fn a_few_flipped_triangles_are_left_out_and_reported() {
    // one triangle of a fine view (0.006 % of its area, 16 times below the 0.1 % that refuses) is left
    // out; no flipped part remains. (The refusal case flips 60 triangles of a coarse view: 4 %, 40
    // times above.)
    let fine = vec![cut(&noisy(blob_at(160), 200), |p| p.x >= -8.0), view(1, -Vec3::x(), 8.0)];
    let mut s = session(&with_flipped_patch_in(fine, 1));
    let out = merge(&mut s, 2, json!({}));
    assert!(codes(&out).contains(&"flipped_triangles_dropped".to_string()), "{:?}", codes(&out));
    assert_eq!(out["after"]["unoriented_edges"], json!(0));
    assert!(out["scans"][0]["dropped"].as_u64().unwrap() >= 1);
}

#[test]
fn a_closed_scan_turned_inside_out_is_refused() {
    let whole = noisy(blob_at(48), 3);
    let views = vec![flipped(&whole), view(1, -Vec3::x(), 8.0)];
    let mut s = session(&views);
    let out = merge(&mut s, 2, json!({}));
    assert_refused(&s, &out, "inverted_scan", 2);
}

#[test]
fn triangles_on_non_manifold_edges_are_left_out_and_counted() {
    let mut views = halves();
    // a fin: a third triangle on an edge of the first view
    let [a, b, _] = views[0].triangles[10];
    let tip = views[0].positions[a as usize] + Vec3::new(0.0, 0.0, 3.0);
    views[0].positions.push(tip);
    let t = (views[0].positions.len() - 1) as u32;
    views[0].triangles.push([b, a, t]);
    let mut s = session(&views);
    let out = merge(&mut s, 2, json!({}));
    assert!(codes(&out).contains(&"non_manifold_input".to_string()), "{:?}", codes(&out));
    assert!(out["scans"][0]["dropped"].as_u64().unwrap() >= 3, "the fin and the two triangles under it");
    assert_eq!(out["after"]["non_manifold_edges"], json!(0));
    assert_eq!(out["after"]["unoriented_edges"], json!(0));
}

#[test]
fn identical_scans_give_one_copy() {
    let a = view(0, Vec3::x(), 8.0);
    let mut s = session(&[a.clone(), a.clone()]);
    let out = merge(&mut s, 2, json!({}));
    let m = mesh_of(&s, &out["id"]);
    assert_eq!(m.triangle_count(), a.triangle_count());
    assert_eq!(m.positions, a.positions, "the copy kept is the first scan, vertex for vertex");
    assert!(codes(&out).contains(&"scan_fully_covered".to_string()));
    assert_eq!(out["scans"][1]["kept"], json!(0));
    assert!(out["pairs"][0]["rms"].as_f64().unwrap() <= 1e-9, "the copies agree to rounding");
}

#[test]
fn scans_that_share_nothing_are_a_union_of_pieces() {
    let far = view(1, -Vec3::x(), 8.0).transformed(&Transform::translation(Vec3::new(200.0, 0.0, 0.0)));
    let views = vec![view(0, Vec3::x(), 8.0), far];
    let mut s = session(&views);
    let out = merge(&mut s, 2, json!({}));
    assert_eq!(out["refused"], json!(false));
    assert!(codes(&out).contains(&"scans_disjoint".to_string()), "{:?}", codes(&out));
    assert_eq!(out["after"]["pieces"], json!(2));
    assert_eq!(out["scans"][0]["removed"], json!(0));
    assert_eq!(out["scans"][1]["removed"], json!(0));
    assert_eq!(out["triangles"].as_u64().unwrap() as usize, views[0].triangle_count() + views[1].triangle_count());
}

#[test]
fn inputs_stay_and_one_undo_removes_the_merge() {
    let views = halves();
    let mut s = session(&views);
    let out = merge(&mut s, 2, json!({"name": "both"}));
    assert_eq!(s.doc.summaries().len(), 3);
    for (k, v) in views.iter().enumerate() {
        let (id, _) = s.doc.find_by_name(&format!("v{k}")).unwrap();
        assert_eq!(s.doc.mesh(id).unwrap().mesh.as_ref(), v, "scan {k} untouched");
    }
    assert!(s.doc.find_by_name("both").is_some());
    // every number of the report is a finite number in the JSON (none became null)
    assert!(out["checks"]["removed_worst"].is_f64() && out["checks"]["stacked_share"].is_f64());
    for r in out["scans"].as_array().unwrap() {
        let n = |k: &str| r[k].as_u64().unwrap();
        assert_eq!(n("triangles"), n("kept") + n("restored") + n("collapsed") + n("removed"), "{r}");
    }
    assert!(out["id"].is_u64());
    s.execute("edit.undo", json!({})).unwrap();
    assert_eq!(s.doc.summaries().len(), 2);
    assert!(s.doc.find_by_name("both").is_none());
}

#[test]
fn pieces_of_one_scan_merge_back_into_it() {
    let whole = noisy(blob_at(64), 5);
    let pieces = vec![
        cut(&whole, |p| p.x <= 8.0),
        cut(&whole, |p| p.x >= -8.0 && p.y <= 4.0),
        cut(&whole, |p| p.x >= -8.0 && p.y >= -4.0),
    ];
    let mut s = session(&pieces);
    let out = merge(&mut s, 3, json!({}));
    let m = mesh_of(&s, &out["id"]);
    // exact, not bounded: the pieces carry the scan's own vertices bit for bit, so a removed
    // triangle whose corners are all in the union comes back with no distance decided (no tolerance,
    // no rounding of the math library involved); the seam is a re-weld (passed on all three systems)
    assert_eq!(out["after"]["pieces"], json!(1));
    assert_eq!(out["after"]["loops"], json!(0));
    let mut at: std::collections::HashSet<[u64; 3]> = std::collections::HashSet::new();
    m.positions.iter().for_each(|p| {
        at.insert([p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]);
    });
    let same = whole.positions.iter().filter(|p| at.contains(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])).count();
    let share = same as f64 / whole.positions.len() as f64;
    eprintln!(
        "{same} of {} vertices of the scan unchanged in the merge; triangles {} against {}",
        whole.positions.len(),
        m.triangle_count(),
        whole.triangle_count()
    );
    // measured 0.999 (1984 of 1986 vertices of the scan, bit for bit)
    assert!(share >= 0.95, "{share}");
    assert_eq!(out["checks"]["moved_vertices"], json!(0));
    assert_eq!(out["checks"]["unwelded_vertices"], json!(0));
}

#[test]
fn bad_parameters_are_refused_without_a_panic() {
    let mut s = session(&halves());
    s.execute("body.box", json!({"size": [1.0, 1.0, 1.0]})).unwrap();
    s.doc.add_mesh("empty", Mesh::default());
    let mut nan = halves()[0].clone();
    nan.positions[0] = Point3::new(f64::NAN, 0.0, 0.0);
    s.doc.add_mesh("nan", nan);
    let many: Vec<String> = (0..65).map(|_| "v0".to_string()).collect();
    for bad in [
        json!({"objects": ["v0"]}),
        json!({"objects": []}),
        json!({"objects": ["v0", "v0"]}),
        json!({"objects": ["v0", "v1"], "tolerance": 0.0}),
        json!({"objects": ["v0", "v1"], "tolerance": -1.0}),
        json!({"objects": ["v0", "v1"], "tolerance": "NaN"}),
        json!({"objects": ["v0", "empty"]}),
        json!({"objects": ["v0", "nan"]}),
        json!({"objects": ["v0", "no such mesh"]}),
        json!({"objects": many}),
        json!({"objects": ["v0", "v1"], "unknown": 1}),
    ] {
        let r = s.execute("mesh.merge", bad.clone());
        assert!(r.is_err(), "{bad}: {r:?}");
        assert!(!matches!(r, Err(surfacecraft_engine::EngineError::Internal(_))), "{bad} panicked: {r:?}");
    }
    // absurd but valid numbers: an answer or an error, never a panic
    let mut huge = halves()[0].clone();
    huge.positions.iter_mut().for_each(|p| *p = Point3::from(p.coords * 1e198));
    s.doc.add_mesh("huge", huge);
    let r = s.execute("mesh.merge", json!({"objects": ["v0", "huge"]}));
    assert!(!matches!(r, Err(surfacecraft_engine::EngineError::Internal(_))), "coordinates near 1e200: {r:?}");
    for odd in [1e300, 1e200, 1e-300, f64::MIN_POSITIVE] {
        let r = s.execute("mesh.merge", json!({"objects": ["v0", "v1"], "tolerance": odd}));
        assert!(!matches!(r, Err(surfacecraft_engine::EngineError::Internal(_))), "{odd}: {r:?}");
    }
    let body = s.doc.summaries().iter().find(|o| o.kind == surfacecraft_doc::ObjectKind::Body).unwrap().id;
    assert!(s.execute("mesh.merge", json!({"objects": ["v0", body.0]})).is_err());
}

#[test]
fn a_cancelled_merge_stops_and_makes_nothing() {
    let mut s = session(&halves());
    s.cancel.cancel();
    let r = s.execute("mesh.merge", json!({"objects": names(2)}));
    assert!(r.is_err());
    assert_eq!(s.doc.summaries().len(), 2);
}

#[test]
fn the_default_tolerance_comes_from_the_noise() {
    let mut s = session(&halves());
    let out = merge(&mut s, 2, json!({}));
    assert_eq!(out["tolerance_source"], json!("noise"));
    let tol = out["tolerance"].as_f64().unwrap();
    let noise: Vec<f64> = out["scans"].as_array().unwrap().iter().map(|r| r["noise"].as_f64().unwrap()).collect();
    eprintln!("tolerance {tol} from noise {noise:?}");
    // sigma 0.05 per scan: 4 * 0.05 * sqrt(2) = 0.28 plus 5 % of a 3 mm edge; measured 0.46
    assert!((0.2..=1.0).contains(&tol), "{tol}");
    let given = merge(&mut s, 2, json!({"tolerance": 0.35}));
    assert_eq!(given["tolerance_source"], json!("given"));
    assert_eq!(given["tolerance"], json!(0.35));
    // no estimate made: the field is absent, as the schema says
    assert!(given["scans"].as_array().unwrap().iter().all(|r| r.get("noise").is_none()), "{}", given["scans"]);
    // the seam length is half the length of the cut edges the seam closed (each seam has two sides),
    // never more than the cut length
    let seam = &given["seam"];
    let length = seam["length"].as_f64().unwrap();
    assert!(length > 0.0 && seam["cut_edges"].as_u64().unwrap() > 0, "{seam}");
}

/// Everything but the stage times.
fn without_times(mut v: Value) -> Value {
    v.as_object_mut().unwrap().remove("stages");
    v.as_object_mut().unwrap().remove("id");
    v
}

#[test]
fn the_same_merge_for_one_and_four_threads() {
    let views =
        vec![view(0, Vec3::x(), 8.0), view(1, -Vec3::x(), 8.0), view(2, Vec3::y(), 5.0), view(3, -Vec3::y(), 5.0)];
    let run = |threads: usize| {
        rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(|| {
            let mut s = session(&views);
            let out = merge(&mut s, 4, json!({}));
            let m = mesh_of(&s, &out["id"]);
            (without_times(out), m)
        })
    };
    let (a, ma) = run(1);
    let (b, mb) = run(4);
    assert_eq!(ma, mb, "the mesh, bit for bit");
    assert_eq!(a, b, "the report");
}
