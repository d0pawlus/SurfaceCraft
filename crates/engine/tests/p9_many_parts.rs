//! P9-O9 gates for scans of many parts: a piece is modelled by what it is, not only by its share of
//! the largest. Co-equal parts (at least `part_share` of the largest) keep their verdict; small parts
//! (closed, consistently oriented solids that are no debris) are modelled too, each with its own
//! verdict; every other piece is left out with its reason, and surface not modelled to a pass above
//! 2 % of the scan keeps the call from a pass. Tolerances are those of design doc §11.1, unchanged.
use serde_json::{Value, json};
mod scan_common;
use scan_common::{session_with, wavy, weld};
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

/// A small closed part: a ball of radius `r` (well above the debris floor of 10 mm across).
fn small_part(at: Point3, r: f64) -> Mesh {
    weld(synth::sphere(at, r, 24, 12))
}

/// A speck of scan debris: a ball 3 mm across of 224 triangles.
fn speck(at: Point3) -> Mesh {
    weld(synth::sphere(at, 1.5, 16, 8))
}

fn codes(out: &Value) -> Vec<String> {
    out["issues"].as_array().into_iter().flatten().filter_map(|i| i["code"].as_str().map(str::to_string)).collect()
}

fn parts(out: &Value) -> Vec<Value> {
    out["components"].as_array().cloned().unwrap_or_default()
}

/// One large part and six small closed parts (each 3 to 4 % of its area, 17 to 21 mm across): all seven are modelled
/// and pass, so the call passes; the small parts say so, and every solid is exported.
#[test]
fn gate_a_large_part_and_small_closed_parts_are_all_modelled_and_pass() {
    let mut scan = wavy(Point3::origin(), 30.0, 3.0, 160);
    for k in 0..6 {
        scan.append(&small_part(Point3::new(60.0 + 25.0 * f64::from(k), 0.0, 0.0), 5.0 + 0.2 * f64::from(k)));
    }
    let mut s = session_with("scan", scan);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    let comps = parts(&out);
    assert_eq!(comps.len(), 7, "{out}");
    assert_eq!(comps.iter().filter(|c| c["co_equal"] == true).count(), 1, "{out}");
    for c in &comps {
        assert_eq!(c["verdict"], "pass", "{c}");
        assert!(c["body"]["volume"].as_f64().unwrap() > 0.0, "{c}");
    }
    assert_eq!(out["verdict"], "pass", "{out}");
    assert_eq!(out["issues"], json!([]), "{out}");
    let numbers = &out["preparation"][0]["numbers"];
    assert_eq!((numbers["co_equal"].as_u64(), numbers["small_parts"].as_u64()), (Some(1), Some(6)), "{numbers}");
    assert_eq!(out["next_steps"][0]["command"], "export.step");
    assert_eq!(out["next_steps"][0]["params"]["objects"].as_array().map(Vec::len), Some(7));
}

/// A two-sided open sheet beside the parts is left out and named with its reason (`open`); a piece
/// with two triangles turned has no consistent inside and is left out too (`not_oriented`). The rest is
/// modelled; together the two hold more than 2 % of the surface, so the verdict is `review`, not `fail`.
#[test]
fn gate_an_open_and_an_unoriented_piece_are_left_out_with_reasons_and_cap_at_review() {
    let mut scan = wavy(Point3::origin(), 30.0, 3.0, 160);
    // each piece is about 4 % of the largest: far from the 10 % of a co-equal part
    scan.append(&small_part(Point3::new(60.0, 0.0, 0.0), 6.0));
    let mut open = small_part(Point3::new(90.0, 0.0, 0.0), 6.0);
    open.triangles.truncate(open.triangles.len() - 120);
    scan.append(&open);
    let mut turned = small_part(Point3::new(120.0, 0.0, 0.0), 6.0);
    turned.triangles.iter_mut().skip(100).take(2).for_each(|t| t.swap(1, 2));
    scan.append(&turned);
    let mut s = session_with("scan", scan);
    // the opening of the sheet (about 24 edges) stays open: holes up to 10 edges are filled
    let out =
        s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "fill_holes_max_edges": 10})).unwrap();
    assert_eq!(parts(&out).len(), 2, "{out}");
    assert!(parts(&out).iter().all(|c| c["verdict"] == "pass"), "{out}");
    let left: Vec<&str> =
        out["pieces_left_out"].as_array().unwrap().iter().filter_map(|p| p["reason"].as_str()).collect();
    assert_eq!(left.len(), 2, "{out}");
    assert!(left.contains(&"open") && left.contains(&"not_oriented"), "{left:?}");
    assert_eq!(out["verdict"], "review", "{out}");
    assert!(codes(&out).contains(&"unmodelled_pieces".to_string()), "{out}");
    let message = out["issues"].as_array().unwrap().iter().find(|i| i["code"] == "unmodelled_pieces").unwrap();
    assert!(message["message"].as_str().unwrap().contains("open 1,"), "{message}");
    assert!(out["next_steps"].as_array().unwrap().iter().any(|st| st["command"] == "scan.split"), "{out}");
}

/// Two small parts made to fail (method `primitives` on wavy balls that no primitive fits) beside a
/// co-equal box that passes: they are modelled, reported with their own `fail`, and the call is
/// `review` (more than 2 % of the surface without a pass), never `fail`.
#[test]
fn gate_small_parts_that_fail_do_not_fail_the_call() {
    let mut scan = synth::cuboid(Point3::new(0.0, 0.0, 0.0), Vec3::new(60.0, 50.0, 40.0), 24);
    // each wavy ball is about 4 % of the area of the box
    scan.append(&wavy(Point3::new(120.0, 0.0, 0.0), 7.0, 5.0, 60));
    scan.append(&wavy(Point3::new(160.0, 0.0, 0.0), 7.0, 4.0, 60));
    let mut s = session_with("scan", scan);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "primitives"})).unwrap();
    let comps = parts(&out);
    assert_eq!(comps.len(), 3, "{out}");
    let box_part = comps.iter().find(|c| c["co_equal"] == true).unwrap();
    assert_eq!(box_part["verdict"], "pass", "{out}");
    let small: Vec<&Value> = comps.iter().filter(|c| c["co_equal"] == false).collect();
    assert_eq!(small.len(), 2);
    assert!(small.iter().all(|c| c["verdict"] != "pass"), "{out}");
    assert_eq!(out["verdict"], "review", "{out}");
    assert!(codes(&out).contains(&"parts_not_passed".to_string()), "{out}");
    let numbers = &out["preparation"][0]["numbers"];
    assert_eq!(numbers["small_parts_not_passed"], 2, "{numbers}");
    // the steps stay bounded and the box is offered for export
    let steps = out["next_steps"].as_array().unwrap();
    assert!(steps.len() <= 3 + 1 + 1, "{steps:?}");
    assert!(steps.iter().any(|st| st["command"] == "export.step"), "{steps:?}");
}

/// Real debris (forty specks 3 mm across) beside two parts is not modelled: the specks are reported
/// as debris and only the parts get solids.
#[test]
fn gate_debris_specks_are_not_modelled() {
    let mut scan = wavy(Point3::origin(), 14.0, 3.0, 120);
    scan.append(&wavy(Point3::new(50.0, 0.0, 0.0), 12.0, 4.0, 110));
    let mut rng = Rng::new(5);
    for _ in 0..40 {
        scan.append(&speck(Point3::new(rng.range(-20.0, 80.0), rng.range(40.0, 120.0), rng.range(-10.0, 30.0))));
    }
    let mut s = session_with("scan", scan);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert_eq!(parts(&out).len(), 2, "{out}");
    let numbers = &out["preparation"][0]["numbers"];
    assert_eq!(numbers["debris_pieces"], 40, "{numbers}");
    assert_eq!(numbers["left_out_by_reason"]["debris"]["pieces"], 40, "{numbers}");
    assert!(out["pieces_left_out"].as_array().unwrap().len() <= 16, "the listing is bounded");
    assert!(out["pieces_left_out"].as_array().unwrap().iter().all(|p| p["reason"] == "debris"));
}

/// A part dropped by the default cleaning (under 1 % of the largest piece's triangles) is kept when it
/// is a small part, and modelled; a single part with debris is cleaned as before.
#[test]
fn gate_cleaning_keeps_a_small_part_below_its_default() {
    let mut scan = wavy(Point3::origin(), 30.0, 3.0, 240);
    let small = small_part(Point3::new(60.0, 0.0, 0.0), 6.0);
    assert!(small.triangle_count() * 100 < scan.triangle_count(), "below the default of the cleaning");
    scan.append(&small);
    let mut s = session_with("scan", scan);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert_eq!(parts(&out).len(), 2, "{out}");
    assert_eq!(out["verdict"], "pass", "{out}");
}

/// Hostile pieces beside two parts never panic and never become parts: degenerate triangles, a lone
/// triangle, and pieces of 100 triangles and more (so that the volume test runs) with a NaN vertex, an
/// infinite vertex, and no volume (flattened). A sound ball 1e12 mm from the origin is a part like any
/// other: its volume, taken about the piece, keeps its digits.
#[test]
fn hostile_pieces_never_panic_and_never_become_parts() {
    let mut scan = wavy(Point3::origin(), 14.0, 3.0, 80);
    scan.append(&wavy(Point3::new(50.0, 0.0, 0.0), 12.0, 4.0, 80));
    let parts_tris = scan.triangle_count();
    let n = scan.positions.len() as u32;
    scan.positions.extend([Point3::new(200.0, 0.0, 0.0), Point3::new(200.0, 0.0, 0.0), Point3::new(201.0, 0.0, 0.0)]);
    scan.triangles.extend([[n, n + 1, n + 2], [n, n, n + 2], [n + 2, n + 1, n]]);
    let ball = |at: f64| small_part(Point3::new(at, 200.0, 0.0), 6.0);
    let mut nan = ball(0.0);
    nan.positions[5].x = f64::NAN;
    let mut inf = ball(40.0);
    inf.positions[5].y = f64::INFINITY;
    let mut flat = ball(80.0);
    flat.positions.iter_mut().for_each(|p| p.z = 0.0);
    let far = weld(synth::sphere(Point3::new(1e12, 0.0, 0.0), 6.0, 30, 15));
    let far_tris = far.triangle_count();
    for m in [&nan, &inf, &flat, &far] {
        assert!(m.triangle_count() >= 100);
        scan.append(m);
    }
    let hostile_tris = nan.triangle_count();
    assert_ne!(hostile_tris, far_tris);
    let mut s = session_with("scan", scan);
    let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"}));
    match r {
        Ok(out) => {
            let tris: Vec<usize> = parts(&out).iter().map(|c| c["triangles"].as_u64().unwrap() as usize).collect();
            assert!(!tris.contains(&hostile_tris), "a hostile piece became a part: {out}");
            assert!(tris.iter().sum::<usize>() <= parts_tris + far_tris, "{out}");
        }
        Err(e) => assert!(!matches!(e, surfacecraft_engine::EngineError::Internal(_)), "a panic: {e}"),
    }
}

/// A block and four bosses that no primitive closes: under `auto` each boss is tried with
/// primitives first and wrapped as freeform second. The first two bosses spend the call's two extra
/// attempts; the others must still get their one wrap (it is not an extra attempt) and pass.
#[test]
fn gate_a_part_after_the_spent_budget_still_gets_its_freeform_wrap() {
    let mut scan = synth::cuboid(Point3::new(0.0, 0.0, 0.0), Vec3::new(40.0, 30.0, 20.0), 16);
    for k in 0..4 {
        let mut boss = weld(synth::cylinder(Point3::origin(), Vec3::z(), 8.0, 31.0, 96, 24));
        boss.positions.iter_mut().for_each(|p| p.x += 80.0 + 40.0 * f64::from(k));
        scan.append(&boss);
    }
    let mut s = session_with("scan", scan);
    let out = s.execute("scan.to_solid", json!({"scan": "scan"})).unwrap();
    let comps = parts(&out);
    assert_eq!(comps.len(), 5, "{out}");
    for c in &comps {
        assert_eq!(c["verdict"], "pass", "{c}");
    }
    let skipped = comps.iter().filter(|c| {
        c["attempts"].as_array().unwrap().iter().any(|a| a["method"] == "freeform" && a["skipped"].is_object())
    });
    assert!(
        skipped.clone().all(|c| c["preparation"].as_array().unwrap().iter().any(|p| p["step"] == "freeform_fallback")),
        "a part refused the second method gets its wrap: {out}"
    );
    assert!(skipped.count() >= 1, "the budget was spent before the last boss: {out}");
    assert_eq!(out["verdict"], "pass", "{out}");
    // every returned solid has the part's name, and a part that took its own wrap keeps nothing of
    // the primitives attempt that lost (no region on its mesh, no primitive fitted to one)
    let mut names: Vec<String> = s.doc.bodies.iter().map(|b| b.name.clone()).collect();
    names.sort();
    assert_eq!(names, (1..=5).map(|n| format!("solid-{n}")).collect::<Vec<_>>(), "{names:?}");
    for c in
        comps.iter().filter(|c| c["preparation"].as_array().unwrap().iter().any(|p| p["step"] == "freeform_fallback"))
    {
        let mesh = c["mesh"].as_u64().unwrap();
        assert!(s.doc.regions.iter().all(|r| r.mesh.0 != mesh), "regions of a losing attempt: {c}");
    }
    let regions: Vec<u64> = s.doc.regions.iter().map(|r| r.id.0).collect();
    assert!(s.doc.primitives.iter().all(|p| p.region.is_none_or(|r| regions.contains(&r.0))));
    // still one undo step
    s.execute("edit.undo", json!({})).unwrap();
    assert!(s.doc.bodies.is_empty() && s.doc.regions.is_empty() && s.doc.primitives.is_empty());
    assert_eq!(s.doc.meshes.len(), 1);
}

/// One small part that fails (method `primitives` on a wavy ball no primitive fits) at about 1 % of
/// the surface beside a box that passes: under the 2 % bound the call passes with no issue, but the
/// part keeps its own `fail`, is counted, and a next step names it. Two such parts at about 4 % each:
/// `review` with `parts_not_passed`.
#[test]
fn gate_a_small_part_that_fails_under_two_percent_is_counted_and_named_not_hidden() {
    let block = || synth::cuboid(Point3::new(0.0, 0.0, 0.0), Vec3::new(60.0, 50.0, 40.0), 24);
    let mut scan = block();
    // area about 154 mm2 against 14 800 of the box: 1 %
    scan.append(&wavy(Point3::new(120.0, 0.0, 0.0), 3.5, 5.0, 60));
    let mut s = session_with("scan", scan);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "primitives"})).unwrap();
    let comps = parts(&out);
    assert_eq!(comps.len(), 2, "{out}");
    let small = comps.iter().find(|c| c["co_equal"] == false).unwrap();
    assert_ne!(small["verdict"], "pass", "{out}");
    assert_eq!(out["verdict"], "pass", "{out}");
    assert_eq!(out["issues"], json!([]), "{out}");
    assert_eq!(out["preparation"][0]["numbers"]["small_parts_not_passed"], 1, "{out}");
    let named = out["next_steps"].as_array().unwrap().iter().any(|st| {
        st["command"] == "scan.to_solid" && st["why"].as_str().unwrap().contains(small["name"].as_str().unwrap())
    });
    assert!(named, "{}", out["next_steps"]);
    let mut over = block();
    over.append(&wavy(Point3::new(120.0, 0.0, 0.0), 7.0, 5.0, 60));
    over.append(&wavy(Point3::new(160.0, 0.0, 0.0), 7.0, 4.0, 60));
    let mut s = session_with("scan", over);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "primitives"})).unwrap();
    assert_eq!(out["verdict"], "review", "{out}");
    assert!(codes(&out).contains(&"parts_not_passed".to_string()), "{out}");
}

/// A closed piece turned inside out beside a part is left out as `inside_out` through a call.
#[test]
fn gate_an_inside_out_piece_is_left_out_with_its_reason() {
    let mut scan = wavy(Point3::origin(), 30.0, 3.0, 160);
    let mut turned = small_part(Point3::new(60.0, 0.0, 0.0), 6.0);
    turned.triangles.iter_mut().for_each(|t| t.swap(1, 2));
    scan.append(&turned);
    let mut s = session_with("scan", scan);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert_eq!(parts(&out).len(), 1, "{out}");
    let left: Vec<&str> =
        out["pieces_left_out"].as_array().unwrap().iter().filter_map(|p| p["reason"].as_str()).collect();
    assert_eq!(left, ["inside_out"], "{out}");
}

/// The cleaning protects exactly the small parts: an open fragment below its default is dropped with
/// or without an unrelated small part beside it (the verdict does not change because of the part);
/// a small part with a spike survives the cleaning; a single part with debris is cleaned exactly as
/// `mesh.clean` cleans it.
#[test]
fn gate_the_cleaning_protects_small_parts_by_identity() {
    let main = wavy(Point3::origin(), 30.0, 3.0, 240);
    let mut fragment = small_part(Point3::new(0.0, 80.0, 0.0), 8.0);
    fragment.triangles.truncate(fragment.triangles.len() - 120);
    assert!(fragment.triangle_count() * 100 < main.triangle_count(), "below the default of the cleaning");
    let mut without = main.clone();
    without.append(&fragment);
    let mut with = without.clone();
    let mut spiked = small_part(Point3::new(80.0, 0.0, 0.0), 6.0);
    if let Some(p) = spiked.positions.get_mut(40) {
        *p += Vec3::new(0.0, 0.0, 4.0);
    }
    with.append(&spiked);
    let params = json!({"scan": "scan", "method": "freeform", "fill_holes_max_edges": 10});
    let alone = session_with("scan", without).execute("scan.to_solid", params.clone()).unwrap();
    let both = session_with("scan", with).execute("scan.to_solid", params).unwrap();
    assert!(alone["components"].is_null(), "the fragment is dropped: {alone}");
    assert_eq!(parts(&both).len(), 2, "the spiked part survives, the fragment does not: {both}");
    assert!(both["pieces_left_out"].is_null(), "{both}");
    assert_eq!(alone["verdict"], both["components"][0]["verdict"], "{both}");
    // a single part with debris: the same mesh as `mesh.clean`
    let mut scan = main;
    scan.append(&weld(synth::sphere(Point3::new(0.0, 0.0, 80.0), 1.0, 8, 4)));
    let mut a = session_with("scan", scan.clone());
    a.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    let mut b = session_with("scan", scan);
    b.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    let (ma, mb) = (a.doc.meshes[0].mesh.clone(), b.doc.meshes[0].mesh.clone());
    assert_eq!((&ma.positions, &ma.triangles), (&mb.positions, &mb.triangles));
}
