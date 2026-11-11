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

/// Points of every edge of a sheet (a trimmed face's edges are degree-1 splines).
fn edge_points(sheet: &surfacecraft_kernel::record::brep::Brep) -> Vec<Point3> {
    use surfacecraft_kernel::record::brep::Curve;
    sheet
        .edges
        .iter()
        .filter_map(|e| match &e.curve {
            Curve::Spline { points, .. } => {
                Some(points.iter().map(|p| Point3::new(p[0], p[1], p[2])).collect::<Vec<_>>())
            }
            _ => None,
        })
        .flatten()
        .collect()
}

/// The analytic boundary of the test outline in the xy plane, sampled every ~0.02 mm: the
/// ellipse outside the notch, the notch's two sides inside the ellipse, and the hole.
fn truth_boundary() -> Vec<Point3> {
    let mut pts = Vec::new();
    let on_ellipse = |t: f64| Point3::new(30.0 + 27.0 * t.cos(), 20.0 + 17.0 * t.sin(), 0.0);
    for k in 0..20_000 {
        let p = on_ellipse(2.0 * PI * f64::from(k) / 20_000.0);
        if !(p.x > 42.0 && p.y > 24.0) {
            pts.push(p);
        }
    }
    for k in 0..2_000 {
        let t = 2.0 * PI * f64::from(k) / 2_000.0;
        pts.push(Point3::new(22.0 + 5.0 * t.cos(), 18.0 + 5.0 * t.sin(), 0.0));
    }
    for k in 0..4_000 {
        let s = f64::from(k) * 0.01;
        for p in [Point3::new(42.0, 24.0 + s, 0.0), Point3::new(42.0 + s, 24.0, 0.0)] {
            if ((p.x - 30.0) / 27.0).powi(2) + ((p.y - 20.0) / 17.0).powi(2) <= 1.0 {
                pts.push(p);
            }
        }
    }
    pts
}

/// Area of the analytic outline in the xy plane (counting cells of 0.05 mm).
fn truth_area() -> f64 {
    let h = 0.05;
    let mut n = 0u64;
    for i in 0..1_200 {
        for j in 0..800 {
            if in_outline(&Point3::new((f64::from(i) + 0.5) * h, (f64::from(j) + 0.5) * h, 0.0)) {
                n += 1;
            }
        }
    }
    n as f64 * h * h
}

/// Area of a mesh projected on the xy plane (the wave is a height field over it).
fn projected_area(m: &Mesh) -> f64 {
    (0..m.triangle_count())
        .filter_map(|t| m.triangle(t))
        .map(|[a, b, c]| ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs() / 2.0)
        .sum()
}

/// Gate P8-H4 (a): trimming to the region outline, checked on the geometry delivered.
///
/// - the 0.2 mm requirement: the MAX over every vertex of the region against the tessellated
///   trimmed face (not a percentile, not the untrimmed spline);
/// - the trim edge against the analytic outline (ellipse, hole, notch): the region is selected
///   by triangle centroids, so its boundary vertices lie within half a cell diagonal (0.47 mm for
///   the 0.667 x 0.667 mm cells) of the truth, and the trim loop within the outline tolerance
///   (0.1 mm) of them: 0.57 mm at most, and on average much closer;
/// - the area of the delivered face projected on the plane against the analytic outline's area.
#[test]
fn gate_h4_trim_to_region_outline() {
    let (mut s, ids, region_area) = trim_scene();
    let plain = s.execute("surface.fit", json!({"region": "free", "name": "plain"})).unwrap();
    assert_eq!(plain["trimmed"], false, "trimming is off by default: {plain}");
    let t = s.execute("surface.fit", json!({"region": "free", "trim": true, "name": "trimmed"})).unwrap();
    eprintln!("gate_h4 trim: {t}");
    assert_eq!(t["trimmed"], true, "{t}");
    assert_eq!(t["outline"]["holes"], 1, "{t}");
    // three separate numbers, each within its bound, and the summary says so
    let (surf, outl, deliv) = (
        t["surface_deviation"].as_f64().unwrap(),
        t["outline_deviation"].as_f64().unwrap(),
        t["delivered_deviation"].as_f64().unwrap(),
    );
    eprintln!("gate_h4 trim: surface {surf:.4}, outline {outl:.4}, delivered {deliv:.4} mm");
    assert!(surf <= 0.2 && outl <= 0.1 && deliv <= 0.2 && t["within_tolerance"] == true, "{t}");
    // independent: MAX over all region vertices against the delivered, tessellated trimmed face
    let scan = s.doc.mesh(s.doc.find_by_name("scan").unwrap().0).unwrap().mesh.clone();
    let mut verts: Vec<usize> =
        ids.iter().filter_map(|&i| scan.triangles.get(i as usize)).flatten().map(|&v| v as usize).collect();
    verts.sort_unstable();
    verts.dedup();
    let pts: Vec<Point3> = verts.iter().map(|&v| scan.positions[v]).collect();
    let face = s.doc.surfaces.iter().find(|o| o.id.0 == t["id"].as_u64().unwrap()).unwrap();
    let tess_face = tess::tessellate(&face.sheet, Some(0.005)).unwrap();
    let d = distances(&pts, &tess_face);
    let max = d[d.len() - 1];
    eprintln!(
        "gate_h4 trim: region vertices -> trimmed face: max {max:.4} mm, p99 {:.4} mm over {} vertices",
        d[d.len() * 99 / 100],
        d.len()
    );
    assert!(max <= 0.2, "max over all region vertices {max}");
    // the trim edges against the analytic outline
    let truth = surfacecraft_geom::KdTree::build(&truth_boundary());
    let off: Vec<f64> = edge_points(&face.sheet)
        .iter()
        .filter_map(|p| truth.knn(&Point3::new(p.x, p.y, 0.0), 1).first().map(|n| n.1.sqrt()))
        .collect();
    let (edge_max, edge_mean) = (off.iter().copied().fold(0.0, f64::max), off.iter().sum::<f64>() / off.len() as f64);
    eprintln!(
        "gate_h4 trim: trim edge vs analytic outline: max {edge_max:.3} mm, mean {edge_mean:.3} mm over {} points",
        off.len()
    );
    assert!(edge_max <= 0.57, "trim edge {edge_max} mm from the analytic outline");
    assert!(edge_mean <= 0.3, "mean {edge_mean}");
    // area of the delivered face against the analytic outline and against the region's triangles
    let (flat, exact) = (projected_area(&tess_face), truth_area());
    let off_area = (flat - exact).abs() / exact;
    let delivered_area = tess_face.area();
    let untrimmed = sheet_area(&s, &plain["id"]);
    eprintln!(
        "gate_h4 trim: projected area {flat:.1} vs analytic {exact:.1} mm2 ({:.2} %); face {delivered_area:.1} vs region triangles {region_area:.1} mm2; rectangle {untrimmed:.1} mm2",
        100.0 * off_area
    );
    assert!(off_area < 0.015, "projected area {flat} vs analytic {exact}");
    assert!((delivered_area - region_area).abs() / region_area < 0.02, "face {delivered_area} vs region {region_area}");
    assert!(untrimmed > 1.4 * region_area, "the untrimmed sheet is the whole rectangle: {untrimmed}");
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
    eprintln!(
        "gate_h4 STEP: area {before:.2} -> {:.2} mm2 ({:.3} %), reimported vertices -> original max {:.5} mm",
        back.area(),
        100.0 * rel,
        d[d.len() - 1]
    );
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
        eprintln!(
            "gate_h4 split: piece {} control {} max {max:.4} mm, {} triangles",
            p["id"], p["control"], p["triangles"]
        );
        assert!(max <= 0.2, "{p}");
        assert_eq!(p["trimmed"], true, "pieces tile only when trimmed: {p}");
    }
    assert!(r["max"].as_f64().unwrap() <= 0.2, "{r}");
    assert_eq!(r["skipped"], Value::Null, "nothing left out: {r}");
    // the pieces are separate surfaces in the document
    assert_eq!(s.doc.surfaces.len(), pieces.len());
    // the delivered geometry: the whole scan against the trimmed faces of all pieces together
    let (delivered, surface) = (r["delivered_deviation"].as_f64().unwrap(), r["surface_deviation"].as_f64().unwrap());
    let tess_of: Vec<Mesh> = pieces
        .iter()
        .map(|p| {
            let o = s.doc.surfaces.iter().find(|o| o.id.0 == p["id"].as_u64().unwrap()).unwrap();
            tess::tessellate(&o.sheet, Some(0.005)).unwrap()
        })
        .collect();
    let mut union = Mesh::default();
    for m in &tess_of {
        union.append(m);
    }
    let scan = s.doc.mesh(s.doc.find_by_name("scan").unwrap().0).unwrap().mesh.clone();
    let d = distances(&scan.positions, &union);
    let (max, p99) = (d[d.len() - 1], d[d.len() * 99 / 100]);
    eprintln!(
        "gate_h4 split: surface {surface:.4}, delivered {delivered:.4} (command), scan -> trimmed pieces max {max:.4} p99 {p99:.4} mm over {} points",
        d.len()
    );
    // the command's number is the one measured here (up to the chord of the tessellations)
    assert!((delivered - max).abs() < 0.01, "command {delivered} vs independent {max}");
    // 0.2 mm on the delivered geometry; within_tolerance says so either way
    assert_eq!(r["within_tolerance"] == true, max <= 0.2 && surface <= 0.2, "{r}");
    assert!(max <= 0.2, "max over all scan points against the trimmed pieces: {max}");
    // neighbouring faces: the gap at the scan vertices both faces reach (each within the delivered
    // deviation of the vertex), measured between the faces' closest points. At such a vertex the
    // gap cannot exceed the sum of the two distances, hence 2 x the delivered deviation: the bound
    // follows from the measured deviation, it is not a tolerance chosen to pass. The command
    // reports the same measure at the mesh vertices the two pieces share.
    let bvhs: Vec<Bvh> = tess_of.iter().map(Bvh::build).collect();
    let mut seam: f64 = 0.0;
    let mut samples = 0;
    for p in &scan.positions {
        let near: Vec<Point3> = bvhs.iter().filter_map(|b| b.closest(p, delivered)).map(|h| h.point).collect();
        for (i, a) in near.iter().enumerate() {
            for b in near.iter().skip(i + 1) {
                seam = seam.max((a - b).norm());
                samples += 1;
            }
        }
    }
    let gaps = r["gaps"].as_array().unwrap();
    let gap = r["gap_max"].as_f64().unwrap();
    eprintln!(
        "gate_h4 split: {} neighbouring pairs, gap_max {gap:.4} mm (command, delivered faces); seam gap {seam:.4} mm over {samples} vertex pairs",
        gaps.len()
    );
    assert!(gaps.len() >= pieces.len() - 1);
    assert!(seam <= 2.0 * delivered, "seam gap {seam} exceeds the two faces' deviations");
    assert!(gap <= 2.0 * delivered, "gap {gap} exceeds the two faces' deviations");
}

/// A height field is one piece whatever `split` says, and the new parameters are validated.
#[test]
fn split_leaves_a_height_field_alone_and_validates_parameters() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "wave", "resolution": 60, "noise": 0.02, "seed": 5, "name": "scan"}))
        .unwrap();
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

/// `max_pieces` is a hard bound, also against the finer re-split of a piece that misses.
#[test]
fn max_pieces_is_a_hard_bound() {
    let mut s = Session::new();
    s.doc.add_mesh("scan", pipe_scan(0.025, 4, 240f64.to_radians()));
    // the 240 degree wall needs 4 pieces: with 2, whatever is fitted is within the limit, and
    // what is left out is reported and keeps `within_tolerance` false
    let r = s.execute("surface.fit", json!({"mesh": "scan", "split": true, "max_pieces": 2})).unwrap();
    let n = r["pieces"].as_array().map_or(1, Vec::len);
    assert!(n <= 2, "{n} pieces with max_pieces 2: {r}");
    assert_eq!(r["within_tolerance"], false, "{r}");
    assert!(r["skipped"].as_array().is_some_and(|v| !v.is_empty()), "what is left out is listed: {r}");
    assert!(s.doc.surfaces.len() <= 2, "{} surfaces in the document", s.doc.surfaces.len());
    // with 1 the wrapped wall cannot be split at all
    assert!(s.execute("surface.fit", json!({"mesh": "scan", "split": true, "max_pieces": 1})).is_err());
    // a generous limit is not reached
    let free = s.execute("surface.fit", json!({"mesh": "scan", "split": true, "max_pieces": 16})).unwrap();
    assert!(free["pieces"].as_array().unwrap().len() <= 16);
}

/// What the trimmed face does not cover, or could not be trimmed to, is in the structured output
/// and makes `within_tolerance` false.
#[test]
fn lost_parts_and_failed_trims_are_visible() {
    let mut m = synth::wave(Point3::origin(), 60.0, 40.0, 3.0, 90, 60);
    synth::add_noise(&mut m, 0.02, &mut Rng::new(3));
    // two separate squares of the same wave: only the larger one can be trimmed to
    let big = |p: &Point3| p.x > 5.0 && p.x < 30.0 && p.y > 5.0 && p.y < 30.0;
    let small = |p: &Point3| p.x > 40.0 && p.x < 52.0 && p.y > 10.0 && p.y < 22.0;
    let ids: Vec<u32> = (0..m.triangle_count())
        .filter(|&t| big(&centroid(&m, t)) || small(&centroid(&m, t)))
        .map(|t| t as u32)
        .collect();
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    s.execute("region.create", json!({"mesh": "scan", "triangles": ids, "name": "two"})).unwrap();
    let r = s.execute("surface.fit", json!({"region": "two", "trim": true})).unwrap();
    eprintln!("lost parts: {r}");
    assert_eq!(r["trimmed"], true);
    assert_eq!(r["islands_ignored"], 1, "{r}");
    assert_eq!(r["within_tolerance"], false, "a part of the region is not covered: {r}");
    // an outline tolerance far too loose for the square: the outline is a 4-corner box that
    // cuts the region's boundary vertices; the delivered deviation says so and the fit is not "within"
    let scan = s.doc.mesh(s.doc.find_by_name("scan").unwrap().0).unwrap().mesh.clone();
    let only: Vec<u32> = (0..scan.triangle_count()).filter(|&t| small(&centroid(&scan, t))).map(|t| t as u32).collect();
    s.execute("region.create", json!({"mesh": "scan", "triangles": only, "name": "one"})).unwrap();
    let f = s.execute("surface.fit", json!({"region": "one", "trim": true, "outline_tolerance": 10.0})).unwrap();
    eprintln!("loose outline: {f}");
    assert!(f["trim_error"].is_null() && f["trimmed"] == true, "{f}");
    assert!(f["surface_deviation"].as_f64().unwrap() <= 0.2, "the spline itself is fine: {f}");
    assert!(f["delivered_deviation"].as_f64().unwrap() > 0.2, "the box cuts the boundary vertices: {f}");
    assert_eq!(f["within_tolerance"], false, "{f}");
}

/// The JSON Schema carries the limits of the new parameters.
#[test]
fn schema_has_the_parameter_limits() {
    let spec = surfacecraft_engine::registry().iter().find(|c| c.id == "surface.fit").unwrap();
    let schema = (spec.input)().to_string();
    for (name, lo, hi) in
        [("outline_tolerance", "0.001", "10"), ("max_piece_angle", "10", "80"), ("max_pieces", "1", "64")]
    {
        let at = schema.find(&format!("\"{name}\"")).unwrap_or_else(|| panic!("{name} missing: {schema}"));
        let part = &schema[at..(at + 400).min(schema.len())];
        assert!(part.contains("\"minimum\"") && part.contains("\"maximum\""), "{name} has no range: {part}");
        assert!(part.contains(lo) && part.contains(hi), "{name}: {part}");
    }
}
