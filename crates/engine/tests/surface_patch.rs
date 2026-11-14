//! `surface.patch`: one trimmed bicubic patch over a closed outline of sides (design doc
//! `2026-11-13-surface-patch-design.md`). The sides are polylines or edges of a body; the data are
//! synthetic scans with a known truth. The thresholds are those of the kernel's driver: gap 1e-3 mm,
//! angle 0.05 degrees, scan deviation 0.2 mm (design doc §11.1), never loosened here.
use serde_json::{Value, json};
use std::f64::consts::{FRAC_PI_2, TAU};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, synth};
use surfacecraft_kernel::nurbs;
use surfacecraft_kernel::record::brep::Loop;
use surfacecraft_kernel::tess;

/// The corners of a regular pentagon of radius `r` in the plane `z = 0`.
fn pentagon(r: f64) -> Vec<[f64; 3]> {
    (0..5)
        .map(|k| {
            let a = FRAC_PI_2 + TAU * f64::from(k) / 5.0;
            [r * a.cos(), r * a.sin(), 0.0]
        })
        .collect()
}

/// `n` points from `a` to `b` on the wave `z = 3 sin(x / 12) cos(y / 9)`.
fn wave_line(a: [f64; 2], b: [f64; 2], n: usize) -> Vec<[f64; 3]> {
    (0..n)
        .map(|i| {
            let t = i as f64 / (n - 1) as f64;
            let (x, y) = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]));
            [x, y, synth::wave_height(3.0, x, y)]
        })
        .collect()
}

fn centroid(m: &Mesh, t: usize) -> Point3 {
    let [a, b, c] = m.triangle(t).unwrap();
    Point3::from((a.coords + b.coords + c.coords) / 3.0)
}

fn sheet_of<'a>(s: &'a Session, r: &Value) -> &'a surfacecraft_doc::SurfaceObject {
    s.doc.surface(ObjectId(r["surface"].as_u64().unwrap())).unwrap()
}

#[test]
fn patch_flat_pentagon_from_polylines() {
    let c = pentagon(15.0);
    // out of order, and two of them reversed: the command chains them into one loop
    let order = [(0, false), (3, false), (2, true), (1, false), (4, true)];
    let sides: Vec<Value> = order
        .iter()
        .map(|&(k, reversed)| {
            let (a, b) = (c[k], c[(k + 1) % 5]);
            if reversed { json!({"points": [b, a]}) } else { json!({"points": [a, b]}) }
        })
        .collect();
    let mut s = Session::new();
    let r = s.execute("surface.patch", json!({"sides": sides, "name": "cap"})).unwrap();
    assert_eq!(s.doc.surfaces.len(), 1);
    assert_eq!(r["degree"], 3);
    assert_eq!(r["max"].as_f64().unwrap(), 0.0, "no data, nothing to deviate from: {r}");
    assert_eq!(r["within_targets"], true, "{r}");
    assert_eq!(r["parameters"], "plane");
    let report = r["sides"].as_array().unwrap();
    assert_eq!(report.len(), 5, "{r}");
    let mut which: Vec<u64> = report.iter().map(|e| e["side"].as_u64().unwrap()).collect();
    which.sort_unstable();
    assert_eq!(which, [0, 1, 2, 3, 4]);
    for e in report {
        assert!(e["gap_max_mm"].as_f64().unwrap() <= 1e-3, "{e}");
        assert_eq!(e["continuity"], "g0");
    }
    // a flat boundary gives a flat patch: the delivered sheet has no height
    let o = sheet_of(&s, &r);
    assert_eq!(o.name, "cap");
    assert_eq!(o.sheet.faces.len(), 1);
    assert!(o.parent.is_none());
    let mesh = tess::tessellate(&o.sheet, None).unwrap();
    assert!(!mesh.positions.is_empty());
    assert!(mesh.positions.iter().all(|p| p.z.abs() <= 1e-9));
}

#[test]
fn weighted_interior_point_is_followed_and_reported() {
    let c = pentagon(15.0);
    let sides: Vec<Value> = (0..5).map(|k| json!({"points": [c[k], c[(k + 1) % 5]]})).collect();
    let mut s = Session::new();
    // a heavy point 1 mm above the centre of a flat pentagon: the patch comes close to it
    let points = json!([{"at": [0.0, 0.0, 1.0], "weight": 100.0}]);
    let r = s.execute("surface.patch", json!({"sides": sides, "points": points, "smoothing": 1e-5})).unwrap();
    let miss = r["soft_points_max_mm"].as_f64().unwrap();
    assert!(miss < 0.2, "{r}");
    // no weighted point, nothing to report
    let r = s.execute("surface.patch", json!({"sides": sides})).unwrap();
    assert!(r.get("soft_points_max_mm").is_none(), "{r}");
    // a weighted point outside the outline is an error that names it
    let points = json!([{"at": [40.0, 0.0, 0.0]}]);
    let e = s.execute("surface.patch", json!({"sides": sides, "points": points})).unwrap_err().to_string();
    assert!(e.contains("soft point 0"), "{e}");
}

/// A wavy sheet (fitted by `surface.fit`) with a five-sided hole, added as a body. Returns the body
/// and the indices of the edges of the hole.
fn wavy_sheet_with_hole(s: &mut Session) -> (ObjectId, Vec<usize>) {
    s.doc.add_mesh("scan", synth::wave(Point3::origin(), 60.0, 40.0, 3.0, 90, 60));
    let fit = s.execute("surface.fit", json!({"mesh": "scan", "name": "sheet"})).unwrap();
    let spline = s.doc.surface(ObjectId(fit["id"].as_u64().unwrap())).unwrap().sheet.faces[0].surface.clone();
    let [[u0, u1], [v0, v1]] = nurbs::domain(&spline).unwrap();
    let (du, dv) = (u1 - u0, v1 - v0);
    let outer = [
        [u0 + 0.03 * du, v0 + 0.03 * dv],
        [u1 - 0.03 * du, v0 + 0.03 * dv],
        [u1 - 0.03 * du, v1 - 0.03 * dv],
        [u0 + 0.03 * du, v1 - 0.03 * dv],
    ];
    let hole: Vec<[f64; 2]> = (0..5)
        .map(|k| {
            let a = FRAC_PI_2 + TAU * f64::from(k) / 5.0;
            [u0 + 0.5 * du + 0.1 * du * a.cos(), v0 + 0.5 * dv + 0.1 * dv * a.sin()]
        })
        .collect();
    let brep = nurbs::trimmed_sheet(&spline, &outer, &[hole]).unwrap();
    let id = s.doc.add_body("holed", brep);
    let body = s.doc.body(id).unwrap();
    let Loop::Edges { edges } = &body.brep.faces[0].loops[1] else { panic!("the hole is a loop of edges") };
    (id, edges.iter().map(|e| e.edge as usize).collect())
}

#[test]
fn patch_fills_hole_in_wavy_sheet_g1() {
    let mut s = Session::new();
    let (body, hole) = wavy_sheet_with_hole(&mut s);
    assert_eq!(hole.len(), 5);
    // every edge of the hole is used by the one face of the sheet: no `face` needed
    let sides: Vec<Value> = hole.iter().map(|e| json!({"body": body.0, "edge": e, "continuity": "g1"})).collect();
    let r = s.execute("surface.patch", json!({"sides": sides, "name": "filler"})).unwrap();
    eprintln!("hole filled: {r}");
    let report = r["sides"].as_array().unwrap();
    assert_eq!(report.len(), 5);
    for e in report {
        assert_eq!(e["continuity"], "g1");
        assert_eq!(e["met"], true, "{e}");
        assert!(e["angle_max_deg"].as_f64().unwrap() <= 0.05, "{e}");
        assert_eq!(e["folded"], false, "{e}");
    }
    assert!(r["corners"].as_array().unwrap().is_empty(), "the sides share one reference face: {r}");
    // an object of its own, a sheet of one trimmed face
    assert_eq!(s.doc.surfaces.len(), 2);
    let o = sheet_of(&s, &r);
    assert_eq!(o.sheet.faces.len(), 1);
    assert!(o.parent.is_none());
}

#[test]
fn g1_side_needs_reference() {
    let mut s = Session::new();
    let c = pentagon(10.0);
    let sides = json!([
        {"points": [c[0], c[1]]},
        {"points": [c[1], c[2]], "continuity": "g1"},
        {"points": [c[2], c[0]]},
    ]);
    let e = s.execute("surface.patch", json!({"sides": sides})).unwrap_err().to_string();
    assert!(e.contains("side 1") && e.contains("face"), "{e}");
    // a body edge with two faces is ambiguous: the error names the side and the count
    s.execute("body.box", json!({"size": [10.0, 10.0, 10.0], "name": "block"})).unwrap();
    let e = s
        .execute("surface.patch", json!({"sides": [{"body": "block", "edge": 0, "continuity": "g1"}]}))
        .unwrap_err()
        .to_string();
    assert!(e.contains("side 0") && e.contains("2 faces"), "{e}");
    assert!(s.doc.surfaces.is_empty());
}

#[test]
fn open_loop_is_refused() {
    let mut s = Session::new();
    let c = pentagon(15.0);
    let mut sides: Vec<Value> = (0..5).map(|k| json!({"points": [c[k], c[(k + 1) % 5]]})).collect();
    // the last side ends 1 mm short of the first corner
    sides[4] = json!({"points": [c[4], [c[0][0] + 1.0, c[0][1], 0.0]]});
    let e = s.execute("surface.patch", json!({"sides": sides})).unwrap_err().to_string();
    assert!(e.contains("do not close") && e.contains("1.00"), "{e}");
    assert!(s.doc.surfaces.is_empty(), "nothing is left behind");
    // the same gap within `gap_tolerance` is closed, and the note says so
    let mut sides: Vec<Value> = (0..5).map(|k| json!({"points": [c[k], c[(k + 1) % 5]]})).collect();
    sides[4] = json!({"points": [c[4], [c[0][0] + 0.005, c[0][1], 0.0]]});
    let r = s.execute("surface.patch", json!({"sides": sides})).unwrap();
    assert!(r["warnings"].as_array().unwrap().iter().any(|w| w.as_str().unwrap().contains("was closed")), "{r}");
}

#[test]
fn patch_with_region_data() {
    let mut m = synth::wave(Point3::origin(), 60.0, 40.0, 3.0, 90, 60);
    synth::add_noise(&mut m, 0.03, &mut Rng::new(7));
    let inside = |p: &Point3| p.x > 10.0 && p.x < 50.0 && p.y > 8.0 && p.y < 32.0;
    let ids: Vec<u32> = (0..m.triangle_count()).filter(|&t| inside(&centroid(&m, t))).map(|t| t as u32).collect();
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    s.execute("region.create", json!({"mesh": "scan", "triangles": ids, "name": "free"})).unwrap();
    // the outline of the region, on the exact wave (the scan is that wave plus noise)
    let (a, b, c, d) = ([10.0, 8.0], [50.0, 8.0], [50.0, 32.0], [10.0, 32.0]);
    let sides: Vec<Value> =
        [(a, b), (b, c), (c, d), (d, a)].iter().map(|&(p, q)| json!({"points": wave_line(p, q, 41)})).collect();
    let r = s.execute("surface.patch", json!({"sides": sides, "region": "free"})).unwrap();
    eprintln!("region patch: {r}");
    assert!(r["max"].as_f64().unwrap() <= 0.2, "{r}");
    assert_eq!(r["within_tolerance"], true, "{r}");
    assert!(r["points"].as_u64().unwrap() > 1000, "{r}");
    // the sides do not run through boundary vertices of the region: the plane of the outline is used
    assert_eq!(r["parameters"], "plane");
    // the patch belongs to the region it was fitted to
    assert!(sheet_of(&s, &r).region.is_some());
}

#[test]
fn patch_uses_the_flattening_when_the_sides_are_the_boundary_of_the_scan() {
    let (nu, nv) = (40usize, 30usize);
    let mut m = synth::wave(Point3::origin(), 60.0, 40.0, 3.0, nu, nv);
    synth::add_noise(&mut m, 0.02, &mut Rng::new(3));
    let at = |i: usize, j: usize| {
        let p = m.positions[j * (nu + 1) + i];
        [p.x, p.y, p.z]
    };
    // the four rows and columns of the grid's border, in loop order
    let bottom: Vec<[f64; 3]> = (0..=nu).map(|i| at(i, 0)).collect();
    let right: Vec<[f64; 3]> = (0..=nv).map(|j| at(nu, j)).collect();
    let top: Vec<[f64; 3]> = (0..=nu).rev().map(|i| at(i, nv)).collect();
    let left: Vec<[f64; 3]> = (0..=nv).rev().map(|j| at(0, j)).collect();
    // the sides run through noisy boundary vertices of the scan: they are soft, the patch follows the data
    let sides: Vec<Value> = [bottom, right, top, left].iter().map(|p| json!({"points": p, "soft": true})).collect();
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    let r = s.execute("surface.patch", json!({"sides": sides, "mesh": "scan"})).unwrap();
    eprintln!("flattened patch: {r}");
    assert_eq!(r["parameters"], "flattened", "{r}");
    assert!(r["max"].as_f64().unwrap() <= 0.2, "{r}");
    // noise sigma 0.02 per axis; measured rms 0.0273 (macOS aarch64): bounds 0.44x and 1.65x
    assert!((0.012..=0.045).contains(&r["rms"].as_f64().unwrap()), "{r}");
    assert_eq!(r["within_tolerance"], true, "{r}");
    assert!(r["sides"].as_array().unwrap().iter().all(|e| e["soft"] == true), "{r}");
    assert!(r["points"].as_u64().unwrap() > 1000, "every vertex of the scan is data: {r}");
}

#[test]
fn schema_names_the_fields_and_the_command_is_heavy() {
    let spec = surfacecraft_engine::registry().iter().find(|c| c.id == "surface.patch").unwrap();
    assert!(spec.undoable);
    assert!(surfacecraft_engine::cmd::is_heavy("surface.patch"));
    let schema = (spec.input)();
    let props = schema["properties"].as_object().unwrap();
    for name in [
        "sides",
        "points",
        "curves",
        "region",
        "mesh",
        "triangles",
        "tolerance",
        "smoothing",
        "control",
        "max_control",
        "gap_tolerance",
        "conflicts",
        "name",
    ] {
        assert!(props.contains_key(name), "{name} is missing from the input schema");
    }
    assert_eq!(schema["required"], json!(["sides"]));
    // unknown fields are refused
    let mut s = Session::new();
    assert!(s.execute("surface.patch", json!({"sides": [], "bogus": 1})).is_err());
}
