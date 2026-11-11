//! P8-H4 gates: `surface.fit` trimmed to the outline of its region, and split into height-field
//! pieces when the region wraps. Both on synthetic scans with a known truth; the deviation
//! threshold is that of design doc §11.1 (0.2 mm), unchanged.
//!
//! - Trim: a wavy patch (σ = 0.02 mm noise) cut to an elliptical outline with a round hole and a
//!   notch. The trimmed face must cover the region (area within 2 % of the sum of the region's
//!   triangle areas, while the untrimmed rectangle is about 1.5 times as large), keep the scan
//!   within 0.2 mm of the surface and survive a STEP round trip through solidcore's importer.
//! - Split: the 240° pipe wall of `gate_p4_open_wrapped_patch` (not a height field) is refused
//!   without `split`; with it, every piece is fitted within 0.2 mm and the gaps between
//!   neighbouring pieces are reported and bounded.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, synth};
use surfacecraft_kernel::tess;

fn centroid(m: &Mesh, t: usize) -> Point3 {
    let [a, b, c] = m.triangle(t).unwrap();
    Point3::from((a.coords + b.coords + c.coords) / 3.0)
}

/// Is the point inside the test outline: an ellipse with a round hole and a rectangular notch?
fn in_outline(p: &Point3) -> bool {
    let ellipse = ((p.x - 30.0) / 27.0).powi(2) + ((p.y - 20.0) / 17.0).powi(2) <= 1.0;
    let hole = (p.x - 22.0).hypot(p.y - 18.0) <= 5.0;
    let notch = p.x > 42.0 && p.y > 24.0;
    ellipse && !hole && !notch
}

fn trim_scene() -> (Session, Vec<u32>, f64) {
    let mut m = synth::wave(Point3::origin(), 60.0, 40.0, 3.0, 90, 60);
    synth::add_noise(&mut m, 0.02, &mut Rng::new(7));
    let ids: Vec<u32> = (0..m.triangle_count()).filter(|&t| in_outline(&centroid(&m, t))).map(|t| t as u32).collect();
    let area: f64 = ids.iter().map(|&t| m.face_area(t as usize)).sum();
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    s.execute("region.create", json!({"mesh": "scan", "triangles": ids, "name": "free"})).unwrap();
    (s, ids, area)
}

fn sheet_area(s: &Session, id: &Value) -> f64 {
    let o = s.doc.surfaces.iter().find(|o| o.id.0 == id.as_u64().unwrap()).unwrap();
    tess::tessellate(&o.sheet, Some(0.01)).unwrap().area()
}

/// Distances from the vertices of `from` to the surface of `to`.
fn distances(from: &[Point3], to: &Mesh) -> Vec<f64> {
    let bvh = Bvh::build(to);
    let mut d: Vec<f64> = from.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).map(|h| h.distance).collect();
    d.sort_by(f64::total_cmp);
    d
}

/// Gate P8-H4 (a): trimming to the region outline.
#[test]
fn gate_h4_trim_to_region_outline() {
    let (mut s, ids, region_area) = trim_scene();
    let plain = s.execute("surface.fit", json!({"region": "free", "name": "plain"})).unwrap();
    assert_eq!(plain["trimmed"], false, "trimming is off by default: {plain}");
    let t = s.execute("surface.fit", json!({"region": "free", "trim": true, "name": "trimmed"})).unwrap();
    eprintln!("gate_h4 trim: {t}");
    assert_eq!(t["trimmed"], true, "{t}");
    assert_eq!(t["outline"]["holes"], 1, "{t}");
    let max = t["max"].as_f64().unwrap();
    assert!(max <= 0.2 && t["within_tolerance"] == true, "scan to surface max {max}: {t}");
    // the trimmed face covers the region, the rectangle does not
    let trimmed = sheet_area(&s, &t["id"]);
    let untrimmed = sheet_area(&s, &plain["id"]);
    let off = (trimmed - region_area).abs() / region_area;
    eprintln!(
        "gate_h4 trim: region {region_area:.1} mm2, trimmed face {trimmed:.1} mm2 ({:.2} % off), rectangle {untrimmed:.1} mm2",
        100.0 * off
    );
    assert!(off < 0.02, "trimmed area {trimmed} vs region {region_area}");
    assert!(untrimmed > 1.4 * region_area, "the untrimmed sheet is the whole rectangle: {untrimmed}");
    // the outline's own area (measured in the fit plane) agrees with the tessellated face
    let flat = t["outline"]["area"].as_f64().unwrap();
    assert!((flat - region_area).abs() / region_area < 0.03, "outline area {flat}");
    // independent deviation: the region's vertices against the tessellated trimmed face
    let scan = s.doc.mesh(s.doc.find_by_name("scan").unwrap().0).unwrap().mesh.clone();
    let tris: Vec<[Point3; 3]> = ids.iter().filter_map(|&i| scan.triangle(i as usize)).collect();
    let pts: Vec<Point3> = tris.iter().flatten().copied().collect();
    let face = s.doc.surfaces.iter().find(|o| o.id.0 == t["id"].as_u64().unwrap()).unwrap();
    let d = distances(&pts, &tess::tessellate(&face.sheet, Some(0.005)).unwrap());
    let p99 = d[d.len() * 99 / 100];
    eprintln!("gate_h4 trim: region vertices -> trimmed face: p99 {p99:.4} mm, max {:.4} mm", d[d.len() - 1]);
    assert!(p99 <= 0.2, "99th percentile {p99}");
}

/// Gate P8-H4 (a): the trimmed sheet exports as STEP and reads back with both loops and its area.
#[test]
fn gate_h4_trimmed_step_roundtrip() {
    let (mut s, _, _) = trim_scene();
    let t = s.execute("surface.fit", json!({"region": "free", "trim": true})).unwrap();
    let before = sheet_area(&s, &t["id"]);
    let exp = s.execute("export.step", json!({"objects": [t["id"]]})).unwrap();
    assert_eq!(exp["content"], "sheets");
    let imp = s.execute("import.step", json!({"data": exp["data"]})).unwrap();
    assert_eq!(imp["faces"][0], 1);
    let body = s.doc.body(ObjectId(imp["bodies"][0].as_u64().unwrap())).unwrap();
    assert_eq!(body.brep.faces[0].loops.len(), 2, "outer loop and hole survive");
    let back = tess::tessellate(&body.brep, Some(0.01)).unwrap();
    let rel = (back.area() - before).abs() / before;
    // both tessellations, against each other
    let orig = s.doc.surfaces.iter().find(|o| o.id.0 == t["id"].as_u64().unwrap()).unwrap();
    let orig_mesh = tess::tessellate(&orig.sheet, Some(0.005)).unwrap();
    let d = distances(&back.positions, &orig_mesh);
    eprintln!("gate_h4 STEP: area {before:.2} -> {:.2} mm2 ({:.3} %), reimported vertices -> original max {:.5} mm", back.area(), 100.0 * rel, d[d.len() - 1]);
    assert!(rel < 0.005, "area changed by {rel}");
    assert!(d[d.len() - 1] < 0.02, "max {}", d[d.len() - 1]);
}

/// Parametric open surface that wraps `sweep` radians around the z axis: a bumpy half pipe (the
/// shape of `gate_p4_open_wrapped_patch`).
fn pipe_point(theta: f64, z: f64, sweep: f64) -> Point3 {
    let u = theta / sweep;
    let r = 15.0 * (1.0 + 0.10 * (2.0 * PI * u * 1.5).sin() * (z / 9.0).cos() + 0.04 * (z / 5.0).sin());
    Point3::new(r * theta.cos(), r * theta.sin(), z)
}

fn pipe_scan(noise: f64, seed: u64, sweep: f64) -> Mesh {
    let (nu, nv) = (220usize, 160usize);
    let mut positions = Vec::new();
    for j in 0..=nv {
        for i in 0..=nu {
            positions.push(pipe_point(sweep * i as f64 / nu as f64, 40.0 * j as f64 / nv as f64, sweep));
        }
    }
    let mut triangles = Vec::new();
    let w = (nu + 1) as u32;
    for j in 0..nv as u32 {
        for i in 0..nu as u32 {
            let a = j * w + i;
            triangles.push([a, a + 1, a + w + 1]);
            triangles.push([a, a + w + 1, a + w]);
        }
    }
    let mut m = Mesh { positions, triangles };
    synth::add_noise(&mut m, noise, &mut Rng::new(seed));
    m
}

/// Gate P8-H4 (b): a surface wrapped by 240° is refused as one patch and split into pieces.
#[test]
fn gate_h4_wrapped_region_is_split() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", pipe_scan(0.025, 4, 240f64.to_radians()));
    let e = s.execute("surface.fit", json!({"mesh": "scan"})).unwrap_err().to_string();
    assert!(e.contains("split"), "the refusal must point at `split`: {e}");
    let r = s.execute("surface.fit", json!({"mesh": "scan", "split": true, "name": "wall"})).unwrap();
    eprintln!("gate_h4 split: {r}");
    let pieces = r["pieces"].as_array().unwrap();
    assert!(pieces.len() >= 3 && pieces.len() <= 6, "{} pieces", pieces.len());
    assert!(r["note"].as_str().unwrap().contains("not a G1 network"), "{r}");
    for p in pieces {
        let max = p["max"].as_f64().unwrap();
        eprintln!("gate_h4 split: piece {} control {} max {max:.4} mm, {} triangles", p["id"], p["control"], p["triangles"]);
        assert!(max <= 0.2 && p["within_tolerance"] == true, "{p}");
        assert_eq!(p["trimmed"], true, "pieces tile only when trimmed: {p}");
    }
    assert!(r["max"].as_f64().unwrap() <= 0.2 && r["within_tolerance"] == true, "{r}");
    assert_eq!(r["skipped"], Value::Null, "nothing left out: {r}");
    // the pieces are separate surfaces in the document
    assert_eq!(s.doc.surfaces.len(), pieces.len());
    // neighbouring sheets meet: the gap is measured at the shared boundary vertices
    let gap = r["gap_max"].as_f64().unwrap();
    eprintln!("gate_h4 split: {} neighbouring pairs, largest gap {gap:.4} mm", r["gaps"].as_array().unwrap().len());
    assert!(r["gaps"].as_array().unwrap().len() >= pieces.len() - 1);
    assert!(gap <= 0.4, "two sheets each within 0.2 mm of a shared vertex are within 0.4 mm of each other: {gap}");
    // independent deviation of the whole scan against the pieces' tessellations together
    let mut union = Mesh::default();
    for o in &s.doc.surfaces {
        union.append(&tess::tessellate(&o.sheet, Some(0.005)).unwrap());
    }
    let scan = s.doc.mesh(s.doc.find_by_name("scan").unwrap().0).unwrap().mesh.clone();
    let d = distances(&scan.positions, &union);
    let p99 = d[d.len() * 99 / 100];
    eprintln!("gate_h4 split: scan -> pieces p99 {p99:.4} mm, max {:.4} mm over {} points", d[d.len() - 1], d.len());
    assert!(p99 <= 0.2, "99th percentile {p99}");
}

/// A height field is one piece whatever `split` says, and the new parameters are validated.
#[test]
fn split_leaves_a_height_field_alone_and_validates_parameters() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "wave", "resolution": 60, "noise": 0.02, "seed": 5, "name": "scan"})).unwrap();
    let r = s.execute("surface.fit", json!({"mesh": "scan", "split": true})).unwrap();
    assert!(r.get("pieces").is_none() && r.get("note").is_none(), "one piece: {r}");
    assert_eq!(r["within_tolerance"], true);
    for bad in [
        json!({"mesh": "scan", "outline_tolerance": -1.0}),
        json!({"mesh": "scan", "outline_tolerance": 0.0}),
        json!({"mesh": "scan", "outline_tolerance": 100.0}),
        json!({"mesh": "scan", "max_piece_angle": 5.0}),
        json!({"mesh": "scan", "max_piece_angle": 120.0}),
        json!({"mesh": "scan", "max_pieces": 0}),
        json!({"mesh": "scan", "max_pieces": 1000}),
    ] {
        assert!(s.execute("surface.fit", bad.clone()).is_err(), "{bad}");
    }
}

/// Regression: the control net was chosen on an evenly strided subset of the points (at most
/// 1500) and the reported maximum was then measured on all of them, so a net that met 0.15 mm on
/// the subset could be returned with 0.19 mm on the full set. The search now measures a net that
/// passes the subset on every point and goes on to the next net when that fails: on this scan
/// the 6 x 6 net reaches 0.122 mm on the subset and 0.186 mm on all points.
#[test]
fn net_is_judged_on_every_point() {
    let mut m = synth::wave(Point3::origin(), 60.0, 40.0, 3.0, 90, 60);
    synth::add_noise(&mut m, 0.03, &mut Rng::new(4));
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    let r = s.execute("surface.fit", json!({"mesh": "scan", "tolerance": 0.15})).unwrap();
    eprintln!("net regression: {r}");
    let max = r["max"].as_f64().unwrap();
    assert!(max <= 0.15 && r["within_tolerance"] == true, "max {max} with net {}", r["control"]);
    assert_ne!(r["control"], json!([6, 6]), "the 6 x 6 net misses on the full point set");
}
