//! P8-E2: `scan.qa` (quality report of a scan before modelling) and `body.compare` (body-versus-scan
//! report per B-rep face), on synthetic data whose defects and deviations are known exactly.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

/// Closed box `[0, 20] × [0, 10] × [0, 5]` mm with `n × n` cells per face.
fn box_scan(n: usize) -> Mesh {
    synth::cuboid(Point3::origin(), Vec3::new(20.0, 10.0, 5.0), n)
}

/// Removes the triangles whose centroid lies inside the box `[lo, hi]`. Cut on grid lines this
/// removes a block of whole cells of one face, whose boundary is a single loop.
fn punch_block(mesh: &mut Mesh, lo: Point3, hi: Point3) {
    let inside =
        |p: &Point3| (lo.x..=hi.x).contains(&p.x) && (lo.y..=hi.y).contains(&p.y) && (lo.z..=hi.z).contains(&p.z);
    let kept: Vec<[u32; 3]> = mesh
        .triangles
        .iter()
        .copied()
        .filter(|t| {
            let sum = t.iter().filter_map(|&i| mesh.positions.get(i as usize)).fold(Vec3::zeros(), |s, p| s + p.coords);
            !inside(&Point3::from(sum / 3.0))
        })
        .collect();
    mesh.triangles = kept;
}

/// A closed tetrahedron of edge `size` (outward orientation) as a separate piece.
fn tetrahedron(mesh: &mut Mesh, origin: Point3, size: f64) {
    let b = mesh.positions.len() as u32;
    let p = |x: f64, y: f64, z: f64| origin + Vec3::new(x, y, z) * size;
    mesh.positions.extend([p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(0.0, 0.0, 1.0)]);
    mesh.triangles.extend([[b, b + 2, b + 1], [b, b + 1, b + 3], [b, b + 3, b + 2], [b + 1, b + 2, b + 3]]);
}

/// Two triangles on the same three collinear points, one per orientation: both have zero area,
/// they repeat each other's corners, and their edges pair up so the piece is closed.
fn degenerate_pair(mesh: &mut Mesh, at: Point3) {
    let b = mesh.positions.len() as u32;
    mesh.positions.extend([at, at + Vec3::x(), at + Vec3::new(2.0, 0.0, 0.0)]);
    mesh.triangles.extend([[b, b + 1, b + 2], [b, b + 2, b + 1]]);
}

fn finding_codes(report: &Value) -> Vec<String> {
    report["findings"]
        .as_array()
        .map_or_else(Vec::new, |v| v.iter().filter_map(|f| f["code"].as_str().map(str::to_string)).collect())
}

/// Gate: a closed box with 16 × 16 cells per face is a clean scan. Its counts follow from the grid:
/// the closed triangulated sphere has E = 3F/2 edges and V − E + F = 2; its area is
/// 2(20·10 + 20·5 + 10·5) = 700 mm²; its shortest edge is the z-step of the side faces (5/n mm) and
/// its longest the diagonal of an x–y cell, √(20² + 10²)/n mm.
#[test]
fn gate_p8_clean_box_scan_reports_no_findings() {
    let n = 16;
    let mesh = box_scan(n);
    let (v, f) = (mesh.vertex_count(), mesh.triangle_count());
    let mut s = Session::new();
    s.doc.add_mesh("box", mesh);
    let r = s.execute("scan.qa", json!({"object": "box"})).unwrap();
    eprintln!("p8 clean box: {r}");
    assert_eq!(r["vertices"].as_u64(), Some(v as u64), "{r}");
    assert_eq!(r["triangles"].as_u64(), Some(f as u64), "{r}");
    let e = 3 * f / 2;
    assert_eq!(r["edges"].as_u64(), Some(e as u64), "{r}");
    assert_eq!(v + f, e + 2, "the grid is a sphere: V - E + F = 2");
    assert_eq!(r["boundary_edges"].as_u64(), Some(0), "{r}");
    assert_eq!(r["non_manifold_edges"].as_u64(), Some(0), "{r}");
    assert_eq!(r["components"].as_u64(), Some(1), "{r}");
    assert_eq!(r["holes"].as_u64(), Some(0), "{r}");
    assert_eq!(r["debris_triangles"].as_u64(), Some(0), "{r}");
    assert_eq!(r["degenerate_triangles"].as_u64(), Some(0), "{r}");
    assert_eq!(r["duplicate_triangles"].as_u64(), Some(0), "{r}");
    assert!((r["area"].as_f64().unwrap() - 700.0).abs() < 1e-9, "{r}");
    assert_eq!(r["bbox_min"], json!([0.0, 0.0, 0.0]), "{r}");
    assert_eq!(r["bbox_max"], json!([20.0, 10.0, 5.0]), "{r}");
    let nf = n as f64;
    assert!((r["edge_length"]["min"].as_f64().unwrap() - 5.0 / nf).abs() < 1e-9, "{r}");
    assert!((r["edge_length"]["max"].as_f64().unwrap() - 500f64.sqrt() / nf).abs() < 1e-9, "{r}");
    assert!(r["noise_sigma"].as_f64().unwrap() < 1e-6, "{r}");
    assert!(finding_codes(&r).is_empty(), "{r}");
}

/// Gate: a box scan with three holes (one block of 2 × 2 cells on the top and on the bottom, and one
/// of 2 × 4 cells on the front), three separate tetrahedra and one degenerate pair. Ground truth:
/// 3 open boundary loops of 8, 8 and 12 vertices, each 5 × 2.5 mm (diagonal √31.25 mm); 5 pieces (the
/// box plus 4 debris pieces); 4 × 3 + 2 = 14 debris triangles; 2 degenerate and 1 duplicate triangle;
/// no non-manifold edge.
#[test]
fn gate_p8_punched_box_with_debris_counts_holes_and_pieces() {
    let mut mesh = box_scan(8);
    punch_block(&mut mesh, Point3::new(7.5, 3.75, 4.9), Point3::new(12.5, 6.25, 5.1));
    punch_block(&mut mesh, Point3::new(7.5, 3.75, -0.1), Point3::new(12.5, 6.25, 0.1));
    punch_block(&mut mesh, Point3::new(7.5, -0.1, 1.25), Point3::new(12.5, 0.1, 3.75));
    for i in 0..3 {
        tetrahedron(&mut mesh, Point3::new(30.0 + 4.0 * i as f64, -15.0, 0.0), 1.0);
    }
    degenerate_pair(&mut mesh, Point3::new(30.0, 20.0, 0.0));
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    let r = s.execute("scan.qa", json!({"object": "scan"})).unwrap();
    eprintln!("p8 punched box: {r}");
    assert_eq!(r["holes"].as_u64(), Some(3), "{r}");
    assert_eq!(r["components"].as_u64(), Some(5), "{r}");
    assert_eq!(r["debris_triangles"].as_u64(), Some(14), "{r}");
    assert_eq!(r["degenerate_triangles"].as_u64(), Some(2), "{r}");
    assert_eq!(r["duplicate_triangles"].as_u64(), Some(1), "{r}");
    assert_eq!(r["non_manifold_edges"].as_u64(), Some(0), "{r}");
    let holes = r["hole_list"].as_array().unwrap();
    let sizes: Vec<u64> = holes.iter().filter_map(|h| h["vertices"].as_u64()).collect();
    assert_eq!(sizes, vec![12, 8, 8], "{r}");
    for h in holes {
        let d = h["diameter"].as_f64().unwrap();
        assert!((d - 31.25f64.sqrt()).abs() < 0.01, "5 × 2.5 mm block has diagonal {d}: {r}");
    }
    let codes = finding_codes(&r);
    for code in ["holes", "debris", "degenerate_triangles", "duplicate_triangles"] {
        assert!(codes.iter().any(|c| c == code), "missing finding {code}: {codes:?}");
    }
    assert!(!codes.iter().any(|c| c == "non_manifold_edges"), "{codes:?}");
    assert_eq!(r["findings"][0]["severity"], "warning", "{r}");
}

/// Gate: a scanner noise of σ = 0.03 mm is estimated within the range of the deviation gate.
#[test]
fn gate_p8_scan_qa_noise_estimate_follows_scanner_sigma() {
    let base = synth::cuboid(Point3::origin(), Vec3::new(30.0, 20.0, 10.0), 1);
    let defects = synth::ScanDefects { noise: 0.03, holes: 0, spikes: 0.0, debris: 0, ..synth::ScanDefects::default() };
    let scan = synth::scanify(&base, &defects, &mut Rng::new(2));
    let mut s = Session::new();
    s.doc.add_mesh("scan", scan);
    let r = s.execute("scan.qa", json!({"object": "scan"})).unwrap();
    let sigma = r["noise_sigma"].as_f64().unwrap_or(f64::NAN);
    eprintln!("p8 scan.qa noise σ=0.03 → {sigma:.4}");
    assert!((0.022..0.04).contains(&sigma), "{r}");
}

/// Gate: a coarse box (4 × 4 cells per face) has sharp edges and too few flat samples to measure
/// its noise: the report says so (`noise_unknown`) and gives no number, rather than an inflated one.
#[test]
fn gate_p8_coarse_box_has_no_noise_number() {
    let mut s = Session::new();
    s.doc.add_mesh("coarse", box_scan(4));
    let r = s.execute("scan.qa", json!({"object": "coarse"})).unwrap();
    eprintln!("p8 coarse box: {r}");
    assert!(r["noise_sigma"].is_null(), "{r}");
    assert_eq!(finding_codes(&r), vec!["noise_unknown".to_string()], "{r}");
    assert_eq!(r["findings"][0]["severity"], "info", "{r}");
}

/// Gate: an empty mesh is reported as an error finding instead of failing.
#[test]
fn gate_p8_empty_mesh_is_an_error_finding() {
    let mut s = Session::new();
    s.doc.add_mesh("empty", Mesh::default());
    let r = s.execute("scan.qa", json!({"object": "empty"})).unwrap();
    assert_eq!(r["triangles"].as_u64(), Some(0), "{r}");
    assert_eq!(r["findings"][0]["code"], "empty_mesh", "{r}");
    assert_eq!(r["findings"][0]["severity"], "error", "{r}");
    assert_eq!(finding_codes(&r), vec!["empty_mesh".to_string()], "no noise judgement without triangles: {r}");
}

/// A grid of 1 mm cells over the face spanned by `u` and `v` from `origin`, pushed out by `offset`.
/// The triangles are counter-clockwise seen along `u × v`, so the normals point out of the box.
/// Returns the number of points added.
fn push_face(mesh: &mut Mesh, origin: Point3, u: Vec3, v: Vec3, cells: (usize, usize), offset: Vec3) -> usize {
    let (nu, nv) = cells;
    let base = mesh.positions.len() as u32;
    let stride = (nv + 1) as u32;
    for i in 0..=nu {
        for j in 0..=nv {
            let p = origin + u * (i as f64 / nu as f64) + v * (j as f64 / nv as f64) + offset;
            mesh.positions.push(p);
        }
    }
    let idx = |i: usize, j: usize| base + i as u32 * stride + j as u32;
    for i in 0..nu {
        for j in 0..nv {
            let (a, b, c, d) = (idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1));
            mesh.triangles.extend([[a, b, c], [a, c, d]]);
        }
    }
    (nu + 1) * (nv + 1)
}

/// A scan of the box `[0, 20] × [0, 10] × [0, 5]` mm inset 1 mm from its edges, with the top face
/// lifted by `lift` mm. Returns the scan and the number of points on each face: top, bottom, front,
/// back, left, right.
fn box_face_scan(lift: f64) -> (Mesh, [usize; 6]) {
    let mut m = Mesh::default();
    let up = Vec3::new(0.0, 0.0, lift);
    let none = Vec3::zeros();
    let counts = [
        push_face(&mut m, Point3::new(1.0, 1.0, 5.0), Vec3::new(18.0, 0.0, 0.0), Vec3::new(0.0, 8.0, 0.0), (18, 8), up),
        push_face(
            &mut m,
            Point3::new(1.0, 1.0, 0.0),
            Vec3::new(0.0, 8.0, 0.0),
            Vec3::new(18.0, 0.0, 0.0),
            (8, 18),
            none,
        ),
        push_face(
            &mut m,
            Point3::new(1.0, 0.0, 1.0),
            Vec3::new(18.0, 0.0, 0.0),
            Vec3::new(0.0, 0.0, 3.0),
            (18, 3),
            none,
        ),
        push_face(
            &mut m,
            Point3::new(1.0, 10.0, 1.0),
            Vec3::new(0.0, 0.0, 3.0),
            Vec3::new(18.0, 0.0, 0.0),
            (3, 18),
            none,
        ),
        push_face(&mut m, Point3::new(0.0, 1.0, 1.0), Vec3::new(0.0, 0.0, 3.0), Vec3::new(0.0, 8.0, 0.0), (3, 8), none),
        push_face(
            &mut m,
            Point3::new(20.0, 1.0, 1.0),
            Vec3::new(0.0, 8.0, 0.0),
            Vec3::new(0.0, 0.0, 3.0),
            (8, 3),
            none,
        ),
    ];
    (m, counts)
}

/// Gate: a body whose top face is 0.2 mm outside the scan. Every scan point is nearest to its own
/// face (the points are at least 1 mm from any other), so the top face has 171 points at +0.2 mm
/// (mean, maximum and RMS), outside the 0.1 mm tolerance; every other face is exact.
#[test]
fn gate_p8_body_compare_offset_face_is_the_worst() {
    let (scan, counts) = box_face_scan(0.2);
    assert_eq!(counts, [171, 171, 76, 76, 36, 36]);
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [20.0, 10.0, 5.0], "name": "block"})).unwrap();
    s.doc.add_mesh("scan", scan);
    let r = s.execute("body.compare", json!({"body": "block", "scan": "scan"})).unwrap();
    eprintln!("p8 body.compare offset top: totals {} faces {}", r["totals"], r["per_face"]);
    assert_eq!(r["totals"]["count"].as_u64(), Some(566), "{r}");
    assert_eq!(r["totals"]["not_facing"].as_u64(), Some(0), "{r}");
    let expected_mean = 0.2 * 171.0 / 566.0;
    assert!((r["totals"]["mean"].as_f64().unwrap() - expected_mean).abs() < 1e-6, "{r}");
    let rows = r["per_face"].as_array().unwrap();
    assert_eq!(rows.len(), 6, "{r}");
    assert_eq!(rows[0]["count"].as_u64(), Some(171), "the worst face comes first: {r}");
    assert!((rows[0]["mean"].as_f64().unwrap() - 0.2).abs() < 1e-6, "{r}");
    assert!((rows[0]["max_abs"].as_f64().unwrap() - 0.2).abs() < 1e-6, "{r}");
    assert!((rows[0]["rms"].as_f64().unwrap() - 0.2).abs() < 1e-6, "{r}");
    assert!(rows[0]["within"].as_f64().unwrap() < 1e-12, "{r}");
    for row in &rows[1..] {
        assert!(row["max_abs"].as_f64().unwrap() < 1e-9, "exact face: {row}");
        assert!((row["within"].as_f64().unwrap() - 1.0).abs() < 1e-12, "{row}");
    }
    let rms: Vec<f64> = rows.iter().filter_map(|x| x["rms"].as_f64()).collect();
    assert!(rms.windows(2).all(|w| w[0] >= w[1]), "worst first: {rms:?}");
}

/// Gate: a scan of the body itself deviates by nothing.
#[test]
fn gate_p8_body_compare_of_its_own_tessellation_is_zero() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [20.0, 10.0, 5.0], "name": "block"})).unwrap();
    s.execute("body.to_mesh", json!({"object": "block", "name": "scan"})).unwrap();
    let r = s.execute("body.compare", json!({"body": "block", "scan": "scan"})).unwrap();
    assert!(r["totals"]["max_abs"].as_f64().unwrap() < 1e-9, "{r}");
    assert_eq!(r["unmeasured"].as_u64(), Some(0), "{r}");
}

/// Errors: a missing object, a wrong kind, a non-positive tolerance or chord, an unknown field.
#[test]
fn p8_qa_commands_refuse_bad_parameters() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [20.0, 10.0, 5.0], "name": "block"})).unwrap();
    s.doc.add_mesh("scan", box_scan(2));
    assert!(s.execute("scan.qa", json!({"object": "nope"})).is_err());
    assert!(s.execute("scan.qa", json!({"object": "scan", "extra": 1})).is_err());
    assert!(s.execute("body.compare", json!({"body": "scan", "scan": "scan"})).is_err());
    assert!(s.execute("body.compare", json!({"body": "block", "scan": "block"})).is_err());
    assert!(s.execute("body.compare", json!({"body": "block", "scan": "scan", "tolerance": -1})).is_err());
    assert!(s.execute("body.compare", json!({"body": "block", "scan": "scan", "chord": 0})).is_err());
}
