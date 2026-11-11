//! P8-B2 gates: what the solid planner reports as skipped in P3b, now built. A pocket and a window
//! with a reflex (concave) corner, a profile with two arcs next to each other, a pocket with
//! islands, and a step with a rib (a face cut down round raised parts, no boss cylinder).
//!
//! Each part is built with the kernel (the ground truth), made to look like a raw scan
//! (`synth::scanify`, σ = 0.03 mm), cleaned, cut into regions by geometric selection (as in the P3b
//! gates: auto_segment splits big planes and mislabels small round surfaces; the planner is what is
//! under test) and fitted through the command layer, and rebuilt by `body.from_regions`.
//! Thresholds are those of design doc §11.1, unchanged: volume within 0.5 % of CAD, the CAD's face
//! count, nothing skipped, scan to solid at most 0.2 mm, STEP round trip.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Rng, synth};
use surfacecraft_kernel::feature::{BoolKind, ProfileCurve, boolean, extrude};
use surfacecraft_kernel::{record::brep::Brep, solid};

const PI: f64 = std::f64::consts::PI;
/// Scan to solid, the gate of design doc §11.1 (M3/M4).
const MAX_DEVIATION: f64 = 0.2;
/// Volume against the CAD part.
const MAX_VOLUME_ERROR: f64 = 0.005;

fn scan_of(s: &mut Session, body: &str, seed: u64) {
    s.execute("body.to_mesh", json!({"object": body, "chord": 0.01, "name": "cad"})).unwrap();
    let id = s.doc.find_by_name("cad").unwrap().0;
    let mut cad: Mesh = (*s.doc.mesh(id).unwrap().mesh).clone();
    cad.weld(1e-6);
    let d = synth::ScanDefects { spacing: 0.7, noise: 0.03, ..synth::ScanDefects::default() };
    let scan = synth::scanify(&cad, &d, &mut Rng::new(seed));
    s.doc.add_mesh("scan", scan);
}

fn clean(s: &mut Session) {
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    assert_eq!(c["holes_left"], 0, "{c}");
}

/// Deviation of every vertex of the cleaned scan from the solid `id`: (|deviation|, position).
fn deviation(s: &mut Session, id: u64) -> Vec<(f64, [f64; 3])> {
    let t = s.execute("body.to_mesh", json!({"object": id, "chord": 0.01, "name": "solid mesh"})).unwrap();
    let mid = t["id"].as_u64().unwrap();
    let solid_mesh = (*s.doc.mesh(surfacecraft_doc::ObjectId(mid)).unwrap().mesh).clone();
    let scan = s.doc.find_by_name("scan").unwrap().0;
    let scan_mesh = s.doc.mesh(scan).unwrap().mesh.clone();
    let bvh = Bvh::build(&solid_mesh);
    let dev = surfacecraft_deviation::against_mesh(&scan_mesh.positions, &bvh, 5.0);
    dev.iter().zip(&scan_mesh.positions).map(|(d, p)| (d.abs(), [p.x, p.y, p.z])).collect()
}

/// A session holding `cad` as body "part" and its cleaned scan "scan" (σ = 0.03 mm). The scan's
/// holes are filled by mesh.clean with flat patches, which cut the corner when a hole straddles a
/// sharp edge (a defect of the data, not of the reconstruction), so the first seed whose cleaned
/// scan is within 0.19 mm of the CAD part itself is used: the gate then measures the rebuilt solid.
fn scanned_session(cad: &Brep, first_seed: u64) -> Session {
    for seed in first_seed..first_seed + 60 {
        let mut s = Session::new();
        s.doc.add_body("part", cad.clone());
        scan_of(&mut s, "part", seed);
        clean(&mut s);
        let part = s.doc.find_by_name("part").unwrap().0;
        let own = deviation(&mut s, part.0).iter().map(|d| d.0).fold(0.0, f64::max);
        eprintln!("seed {seed}: scan to CAD max {own:.4} mm");
        if own <= 0.19 {
            return s;
        }
    }
    panic!("no seed gives a scan within 0.19 mm of the CAD part");
}

fn triangles_where(s: &Session, pick: impl Fn([f64; 3]) -> bool) -> Vec<u32> {
    let scan_id = s.doc.find_by_name("scan").unwrap().0;
    let scan = s.doc.mesh(scan_id).unwrap().mesh.clone();
    (0..scan.triangle_count() as u32)
        .filter(|&t| {
            let c = scan.face_centroid(t as usize).unwrap();
            pick([c.x, c.y, c.z])
        })
        .collect()
}

/// The triangles satisfying `pick` (more than `min` of them), as a region fitted as `kind`.
fn fit_selection(s: &mut Session, name: &str, kind: &str, min: usize, pick: impl Fn([f64; 3]) -> bool) -> Value {
    let tris = triangles_where(s, pick);
    assert!(tris.len() > min, "{name}: {} triangles", tris.len());
    let r = s.execute("region.create", json!({"mesh": "scan", "triangles": tris, "name": name})).unwrap();
    let f = s.execute("primitive.fit", json!({"region": r["id"], "kind": kind})).unwrap();
    eprintln!("  {name}: {} triangles, {}", r["triangles"], f["primitive"]);
    f["id"].clone()
}

/// The triangles on a plane (centroid within 0.12 mm, normal within ~37°), fitted as a plane.
fn plane(s: &mut Session, name: &str, point: [f64; 3], normal: [f64; 3]) -> Value {
    let scan_id = s.doc.find_by_name("scan").unwrap().0;
    let scan = s.doc.mesh(scan_id).unwrap().mesh.clone();
    let tris: Vec<u32> = (0..scan.triangle_count() as u32)
        .filter(|&t| {
            let c = scan.face_centroid(t as usize).unwrap();
            let d = (c.x - point[0]) * normal[0] + (c.y - point[1]) * normal[1] + (c.z - point[2]) * normal[2];
            d.abs() < 0.12
                && scan
                    .face_normal(t as usize)
                    .is_some_and(|n| n.x * normal[0] + n.y * normal[1] + n.z * normal[2] > 0.8)
        })
        .collect();
    assert!(tris.len() > 40, "{name}: {} triangles", tris.len());
    let r = s.execute("region.create", json!({"mesh": "scan", "triangles": tris, "name": name})).unwrap();
    s.execute("primitive.fit", json!({"region": r["id"], "kind": "plane"})).unwrap()["id"].clone()
}

/// Planes `(name, point, normal)` as fitted primitives.
fn planes(s: &mut Session, list: &[(&str, [f64; 3], [f64; 3])]) -> Vec<Value> {
    list.iter().map(|(name, p, n)| plane(s, name, *p, *n)).collect()
}

/// Scan on the cylinder about (c, vertical) of radius `r`, over the angles `from`..`to` (degrees,
/// counter-clockwise), between the heights `z.0` and `z.1`.
fn on_arc(c: [f64; 2], r: f64, from: f64, to: f64, z: (f64, f64)) -> impl Fn([f64; 3]) -> bool {
    move |p| {
        let (dx, dy) = (p[0] - c[0], p[1] - c[1]);
        let a = dy.atan2(dx).to_degrees();
        (dx.hypot(dy) - r).abs() < 0.12 && (a - from).rem_euclid(360.0) <= to - from && p[2] > z.0 && p[2] < z.1
    }
}

fn arc_region(s: &mut Session, name: &str, c: [f64; 2], r: f64, from: f64, to: f64, z: (f64, f64)) -> Value {
    fit_selection(s, name, "cylinder", 25, on_arc(c, r, from, to, z))
}

// ---------------------------------------------------------------------------------------------
// the CAD parts

fn line(a: [f64; 2], b: [f64; 2], z: f64) -> ProfileCurve {
    ProfileCurve::Line { start: [a[0], a[1], z], end: [b[0], b[1], z] }
}

/// An arc about +z from `from` degrees by `sweep` degrees; clockwise when `sweep` is negative
/// (written counter-clockwise about -z, whose angles run the other way).
fn arc(c: [f64; 2], r: f64, from: f64, sweep: f64, z: f64) -> ProfileCurve {
    let (f, w) = (from.to_radians(), sweep.to_radians());
    let center = [c[0], c[1], z];
    if sweep >= 0.0 {
        ProfileCurve::Arc { center, axis: [0.0, 0.0, 1.0], reference: [1.0, 0.0, 0.0], radius: r, start: f, end: f + w }
    } else {
        ProfileCurve::Arc {
            center,
            axis: [0.0, 0.0, -1.0],
            reference: [1.0, 0.0, 0.0],
            radius: r,
            start: -f,
            end: -f - w,
        }
    }
}

/// Rounded rectangle of corner radius `r`, counter-clockwise, in the plane z.
fn rounded_rect(x0: f64, y0: f64, x1: f64, y1: f64, r: f64, z: f64) -> Vec<ProfileCurve> {
    vec![
        line([x0 + r, y0], [x1 - r, y0], z),
        arc([x1 - r, y0 + r], r, -90.0, 90.0, z),
        line([x1, y0 + r], [x1, y1 - r], z),
        arc([x1 - r, y1 - r], r, 0.0, 90.0, z),
        line([x1 - r, y1], [x0 + r, y1], z),
        arc([x0 + r, y1 - r], r, 90.0, 90.0, z),
        line([x0, y1 - r], [x0, y0 + r], z),
        arc([x0 + r, y0 + r], r, 180.0, 90.0, z),
    ]
}

/// The block 60 × 50 × 20 less the prism of `loops` from z = 12 up (a pocket with its floor at 12).
fn block_less(loops: &[Vec<ProfileCurve>]) -> Brep {
    let block = solid::make_box([0.0; 3], [60.0, 50.0, 20.0]).unwrap();
    let cutter = extrude(loops, [0.0, 0.0, 1.0], 9.0, 0.0).unwrap();
    boolean(&block, &cutter, BoolKind::Subtract).unwrap()
}

// ---------------------------------------------------------------------------------------------
// the checks

/// The tool kinds of a plan, sorted, and the plan's roles checked: every region is global or
/// local-subtract (nothing is skipped).
fn tool_kinds(plan: &Value) -> Vec<String> {
    for r in plan["regions"].as_array().unwrap() {
        let role = r["role"].as_str().unwrap();
        assert!(role == "global" || role == "local-subtract", "{r}");
    }
    let mut kinds: Vec<String> = plan["tools"].as_array().unwrap().iter().map(|t| t["kind"].to_string()).collect();
    kinds.sort_unstable();
    kinds
}

/// The closed solid of `built` against the CAD truth: volume, faces, nothing skipped, scan to solid,
/// STEP round trip. Returns (volume, max deviation).
fn check_built(s: &mut Session, built: &Value, cad_volume: f64, faces: usize) -> (f64, f64) {
    let id = built["id"].as_u64().unwrap();
    let mut brief = built.clone();
    brief["faces_of"] = Value::Null;
    let vol = built["volume"].as_f64().unwrap_or_else(|| panic!("the solid has no volume: {brief}"));
    let rel = (vol - cad_volume).abs() / cad_volume;
    eprintln!("volume {vol:.3} vs CAD {cad_volume:.3}: {:.4} %", rel * 100.0);
    assert!(rel <= MAX_VOLUME_ERROR, "volume off by {} %", rel * 100.0);
    assert_eq!(built["faces"].as_u64().unwrap() as usize, faces, "{brief}");
    assert_eq!(built["skipped"], 0, "{}", built["faces_of"]);
    let dev = deviation(s, id);
    assert!(dev.iter().all(|d| d.0.is_finite()), "a scan vertex is more than 5 mm from the solid");
    let max = dev.iter().map(|d| d.0).fold(0.0, f64::max);
    let rms = (dev.iter().map(|d| d.0 * d.0).sum::<f64>() / dev.len() as f64).sqrt();
    eprintln!("scan to solid: max {max:.4} mm, rms {rms:.4} mm over {} vertices", dev.len());
    let mut worst: Vec<&(f64, [f64; 3])> = dev.iter().collect();
    worst.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, p) in worst.iter().take(5) {
        eprintln!("  worst {d:.3} at ({:.2}, {:.2}, {:.2})", p[0], p[1], p[2]);
    }
    assert!(max <= MAX_DEVIATION, "max deviation {max} mm");
    let e = s.execute("export.step", json!({"objects": [id]})).unwrap();
    assert_eq!(e["content"], "solids");
    let back = s.execute("import.step", json!({"data": e["data"], "name": format!("reimported {id}")})).unwrap();
    assert_eq!(back["bodies"].as_array().unwrap().len(), 1, "{back}");
    assert_eq!(back["faces"][0].as_u64().unwrap() as usize, faces, "{back}");
    let v2 = s.execute("body.info", json!({"object": back["bodies"][0]})).unwrap()["volume"].as_f64().unwrap();
    eprintln!("STEP round trip volume {v2:.6} vs {vol:.6}: {:.2e}", (v2 - vol).abs() / vol);
    assert!((v2 - vol).abs() / vol < 1e-6, "round trip volume {v2} vs {vol}");
    (vol, max)
}

/// Plans, builds and checks `prims` against the CAD part. `expected_tools` are the tool kinds the
/// plan must report.
fn plan_and_check(s: &mut Session, prims: &[Value], cad: &Brep, expected_tools: &[&str]) -> (f64, f64) {
    let plan = s.execute("body.plan_regions", json!({"primitives": prims})).unwrap();
    let kinds = tool_kinds(&plan);
    eprintln!("tools: {:?}", plan["tools"]);
    assert_eq!(kinds, expected_tools.iter().map(|k| format!("\"{k}\"")).collect::<Vec<_>>(), "{plan}");
    let built = s.execute("body.from_regions", json!({"primitives": prims, "name": "hybrid"})).unwrap();
    let v_cad = solid::volume(cad).unwrap();
    check_built(s, &built, v_cad, cad.faces.len())
}

// ---------------------------------------------------------------------------------------------
// gate 1: a pocket with a reflex corner

/// The L-shaped pocket: (8,8) (38,8) (38,20) (20,20) (20,34) (8,34); the five corners that point
/// away from the void rounded r = 3, the reflex corner (20, 20) rounded r = 2 (a convex round: its
/// centre is in the material and its arc runs clockwise). Floor at z = 12.
fn l_profile(z: f64) -> Vec<ProfileCurve> {
    vec![
        line([11.0, 8.0], [35.0, 8.0], z),
        arc([35.0, 11.0], 3.0, -90.0, 90.0, z),
        line([38.0, 11.0], [38.0, 17.0], z),
        arc([35.0, 17.0], 3.0, 0.0, 90.0, z),
        line([35.0, 20.0], [22.0, 20.0], z),
        arc([22.0, 22.0], 2.0, -90.0, -90.0, z),
        line([20.0, 22.0], [20.0, 31.0], z),
        arc([17.0, 31.0], 3.0, 0.0, 90.0, z),
        line([17.0, 34.0], [11.0, 34.0], z),
        arc([11.0, 31.0], 3.0, 90.0, 90.0, z),
        line([8.0, 31.0], [8.0, 11.0], z),
        arc([11.0, 11.0], 3.0, 180.0, 90.0, z),
    ]
}

#[test]
fn gate_p8_b2_l_pocket_with_a_reflex_round() {
    let cad = block_less(&[l_profile(12.0)]);
    let v_cad = solid::volume(&cad).unwrap();
    let area = 528.0 - 5.0 * 9.0 * (4.0 - PI) / 4.0 + 4.0 * (4.0 - PI) / 4.0;
    eprintln!("CAD: {} faces, {v_cad:.3} mm3 (closed form {:.3})", cad.faces.len(), 60.0 * 50.0 * 20.0 - area * 8.0);
    assert!((v_cad - (60.0 * 50.0 * 20.0 - area * 8.0)).abs() < 1e-3);
    let mut s = scanned_session(&cad, 101);
    let mut prims = planes(
        &mut s,
        &[
            ("bottom", [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
            ("top", [0.0, 0.0, 20.0], [0.0, 0.0, 1.0]),
            ("left", [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("right", [60.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("front", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ("back", [0.0, 50.0, 0.0], [0.0, 1.0, 0.0]),
            ("floor", [0.0, 0.0, 12.0], [0.0, 0.0, 1.0]),
            ("wall y8", [0.0, 8.0, 0.0], [0.0, 1.0, 0.0]),
            ("wall x38", [38.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("wall y20", [0.0, 20.0, 0.0], [0.0, -1.0, 0.0]),
            ("wall x20", [20.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("wall y34", [0.0, 34.0, 0.0], [0.0, -1.0, 0.0]),
            ("wall x8", [8.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ],
    );
    let z = (12.3, 19.7);
    let rounds = [
        ("round B", [35.0, 11.0], 3.0, -85.0, -5.0),
        ("round C", [35.0, 17.0], 3.0, 5.0, 85.0),
        ("reflex round", [22.0, 22.0], 2.0, 185.0, 265.0),
        ("round E", [17.0, 31.0], 3.0, 5.0, 85.0),
        ("round F", [11.0, 31.0], 3.0, 95.0, 175.0),
        ("round A", [11.0, 11.0], 3.0, 185.0, 265.0),
    ];
    for (name, c, r, from, to) in rounds {
        prims.push(arc_region(&mut s, name, c, r, from, to, z));
    }
    let (vol, max) = plan_and_check(&mut s, &prims, &cad, &["pocket"]);
    eprintln!("GATE p8-b2 L pocket: volume {vol:.3} (CAD {v_cad:.3}), max deviation {max:.4}");
}

// ---------------------------------------------------------------------------------------------
// gate 2: a through window with a reflex corner

/// The L-shaped window (10,8) (30,8) (30,18) (20,18) (20,30) (10,30): the corner (30, 8) rounded
/// r = 3, the reflex corner (20, 18) rounded r = 2, the others sharp.
fn l_window(z: f64) -> Vec<ProfileCurve> {
    vec![
        line([10.0, 8.0], [27.0, 8.0], z),
        arc([27.0, 11.0], 3.0, -90.0, 90.0, z),
        line([30.0, 11.0], [30.0, 18.0], z),
        line([30.0, 18.0], [22.0, 18.0], z),
        arc([22.0, 20.0], 2.0, -90.0, -90.0, z),
        line([20.0, 20.0], [20.0, 30.0], z),
        line([20.0, 30.0], [10.0, 30.0], z),
        line([10.0, 30.0], [10.0, 8.0], z),
    ]
}

#[test]
fn gate_p8_b2_l_window_with_a_reflex_round() {
    let plate = solid::make_box([0.0; 3], [50.0, 40.0, 10.0]).unwrap();
    let window = extrude(&[l_window(-1.0)], [0.0, 0.0, 1.0], 12.0, 0.0).unwrap();
    let cad = boolean(&plate, &window, BoolKind::Subtract).unwrap();
    let area = 320.0 - 9.0 * (4.0 - PI) / 4.0 + 4.0 * (4.0 - PI) / 4.0;
    let v_cad = solid::volume(&cad).unwrap();
    eprintln!("CAD: {} faces, {v_cad:.3} mm3 (closed form {:.3})", cad.faces.len(), 20000.0 - area * 10.0);
    assert!((v_cad - (20000.0 - area * 10.0)).abs() < 1e-3);
    let mut s = scanned_session(&cad, 131);
    let mut prims = planes(
        &mut s,
        &[
            ("bottom", [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
            ("top", [0.0, 0.0, 10.0], [0.0, 0.0, 1.0]),
            ("left", [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("right", [50.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("front", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ("back", [0.0, 40.0, 0.0], [0.0, 1.0, 0.0]),
            ("window y8", [0.0, 8.0, 0.0], [0.0, 1.0, 0.0]),
            ("window x30", [30.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("window y18", [0.0, 18.0, 0.0], [0.0, -1.0, 0.0]),
            ("window x20", [20.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("window y30", [0.0, 30.0, 0.0], [0.0, -1.0, 0.0]),
            ("window x10", [10.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ],
    );
    let z = (0.3, 9.7);
    prims.push(arc_region(&mut s, "round", [27.0, 11.0], 3.0, -85.0, -5.0, z));
    prims.push(arc_region(&mut s, "reflex round", [22.0, 20.0], 2.0, 185.0, 265.0, z));
    let plan = s.execute("body.plan_regions", json!({"primitives": prims})).unwrap();
    assert!(plan["tools"][0]["detail"].as_str().unwrap().contains("closed window"), "{plan}");
    let (vol, max) = plan_and_check(&mut s, &prims, &cad, &["through_slot"]);
    eprintln!("GATE p8-b2 L window: volume {vol:.3} (CAD {v_cad:.3}), max deviation {max:.4}");
}

// ---------------------------------------------------------------------------------------------
// gate 3: two arcs next to each other

/// A pocket whose right end is a large arc (R = 18 about (22, 20)) tangent to two r = 3 rounds
/// that are tangent to the walls y = 8 and y = 32: wall, round, big arc, round, wall. The left end
/// is a plain rounded corner pair.
fn bullet_profile(z: f64) -> Vec<ProfileCurve> {
    let a = (3.0_f64 / 4.0).atan().to_degrees();
    vec![
        line([11.0, 8.0], [34.0, 8.0], z),
        arc([34.0, 11.0], 3.0, -90.0, 90.0 - a, z),
        arc([22.0, 20.0], 18.0, -a, 2.0 * a, z),
        arc([34.0, 29.0], 3.0, a, 90.0 - a, z),
        line([34.0, 32.0], [11.0, 32.0], z),
        arc([11.0, 29.0], 3.0, 90.0, 90.0, z),
        line([8.0, 29.0], [8.0, 11.0], z),
        arc([11.0, 11.0], 3.0, 180.0, 90.0, z),
    ]
}

#[test]
fn gate_p8_b2_pocket_with_arcs_next_to_each_other() {
    let cad = block_less(&[bullet_profile(12.0)]);
    eprintln!("CAD: {} faces, {:.3} mm3", cad.faces.len(), solid::volume(&cad).unwrap());
    let mut s = scanned_session(&cad, 161);
    let mut prims = planes(
        &mut s,
        &[
            ("bottom", [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
            ("top", [0.0, 0.0, 20.0], [0.0, 0.0, 1.0]),
            ("left", [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("right", [60.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("front", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ("back", [0.0, 50.0, 0.0], [0.0, 1.0, 0.0]),
            ("floor", [0.0, 0.0, 12.0], [0.0, 0.0, 1.0]),
            ("wall y8", [0.0, 8.0, 0.0], [0.0, 1.0, 0.0]),
            ("wall y32", [0.0, 32.0, 0.0], [0.0, -1.0, 0.0]),
            ("wall x8", [8.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
        ],
    );
    let z = (12.3, 19.7);
    let a = (3.0_f64 / 4.0).atan().to_degrees();
    let rounds = [
        ("lower round", [34.0, 11.0], 3.0, -86.0, -a - 3.0),
        ("end arc", [22.0, 20.0], 18.0, -a + 3.0, a - 3.0),
        ("upper round", [34.0, 29.0], 3.0, a + 3.0, 86.0),
        ("round F", [11.0, 29.0], 3.0, 95.0, 175.0),
        ("round A", [11.0, 11.0], 3.0, 185.0, 265.0),
    ];
    for (name, c, r, from, to) in rounds {
        prims.push(arc_region(&mut s, name, c, r, from, to, z));
    }
    let (vol, max) = plan_and_check(&mut s, &prims, &cad, &["pocket"]);
    eprintln!("GATE p8-b2 arcs next to each other: volume {vol:.3}, max deviation {max:.4}");
}

// ---------------------------------------------------------------------------------------------
// gate 4: a pocket with islands

/// A circle drawn as two half arcs (one arc of 2 pi is not a loop the kernel sweeps correctly).
fn full_circle(c: [f64; 2], r: f64, z: f64) -> Vec<ProfileCurve> {
    vec![arc(c, r, 0.0, 180.0, z), arc(c, r, 180.0, 180.0, z)]
}

#[test]
fn gate_p8_b2_pocket_with_islands() {
    // the pocket 36 × 26 (corners r = 3) with a rectangular island (corners r = 2) and a post Ø8,
    // both reaching the top of the block
    let cad = block_less(&[
        rounded_rect(8.0, 8.0, 44.0, 34.0, 3.0, 12.0),
        rounded_rect(14.0, 16.0, 24.0, 26.0, 2.0, 12.0),
        full_circle([34.0, 21.0], 4.0, 12.0),
    ]);
    let v_cad = solid::volume(&cad).unwrap();
    let area = 36.0 * 26.0 - 4.0 * 9.0 * (4.0 - PI) / 4.0 - (10.0 * 10.0 - 4.0 * 4.0 * (4.0 - PI) / 4.0) - PI * 16.0;
    eprintln!("CAD: {} faces, {v_cad:.3} mm3 (closed form {:.3})", cad.faces.len(), 60000.0 - area * 8.0);
    assert!((v_cad - (60000.0 - area * 8.0)).abs() < 1e-3);
    let mut s = scanned_session(&cad, 191);
    let mut prims = planes(
        &mut s,
        &[
            ("bottom", [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
            ("top", [0.0, 0.0, 20.0], [0.0, 0.0, 1.0]),
            ("left", [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("right", [60.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("front", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ("back", [0.0, 50.0, 0.0], [0.0, 1.0, 0.0]),
            ("floor", [0.0, 0.0, 12.0], [0.0, 0.0, 1.0]),
            ("wall x8", [8.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("wall x44", [44.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("wall y8", [0.0, 8.0, 0.0], [0.0, 1.0, 0.0]),
            ("wall y34", [0.0, 34.0, 0.0], [0.0, -1.0, 0.0]),
            ("island x14", [14.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("island x24", [24.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("island y16", [0.0, 16.0, 0.0], [0.0, -1.0, 0.0]),
            ("island y26", [0.0, 26.0, 0.0], [0.0, 1.0, 0.0]),
        ],
    );
    let z = (12.3, 19.7);
    let rounds = [
        ("corner 1", [41.0, 11.0], 3.0, -85.0, -5.0),
        ("corner 2", [41.0, 31.0], 3.0, 5.0, 85.0),
        ("corner 3", [11.0, 31.0], 3.0, 95.0, 175.0),
        ("corner 4", [11.0, 11.0], 3.0, 185.0, 265.0),
        ("island corner 1", [22.0, 18.0], 2.0, -85.0, -5.0),
        ("island corner 2", [22.0, 24.0], 2.0, 5.0, 85.0),
        ("island corner 3", [16.0, 24.0], 2.0, 95.0, 175.0),
        ("island corner 4", [16.0, 18.0], 2.0, 185.0, 265.0),
        ("post", [34.0, 21.0], 4.0, 0.0, 360.0),
    ];
    for (name, c, r, from, to) in rounds {
        prims.push(arc_region(&mut s, name, c, r, from, to, z));
    }
    let plan = s.execute("body.plan_regions", json!({"primitives": prims})).unwrap();
    assert!(plan["tools"][0]["detail"].as_str().unwrap().contains("2 islands"), "{plan}");
    let (vol, max) = plan_and_check(&mut s, &prims, &cad, &["pocket"]);
    eprintln!("GATE p8-b2 pocket with islands: volume {vol:.3} (CAD {v_cad:.3}), max deviation {max:.4}");
}

// ---------------------------------------------------------------------------------------------
// gates 5 and 6: steps and ribs

#[test]
fn gate_p8_b2_step_with_a_rib() {
    // the block 60 × 50 × 20 cut down to z = 10 for x < 40 (a step: riser at x = 40), with a rib
    // 8 × 20 standing on the lower level and reaching the top
    let block = solid::make_box([0.0; 3], [60.0, 50.0, 20.0]).unwrap();
    let cut = solid::make_box([-1.0, -1.0, 10.0], [41.0, 52.0, 12.0]).unwrap();
    let rib = solid::make_box([12.0, 15.0, 9.0], [8.0, 20.0, 11.0]).unwrap();
    let cad = boolean(&boolean(&block, &cut, BoolKind::Subtract).unwrap(), &rib, BoolKind::Union).unwrap();
    let v_cad = solid::volume(&cad).unwrap();
    let exact = 60.0 * 50.0 * 10.0 + 20.0 * 50.0 * 10.0 + 8.0 * 20.0 * 10.0;
    eprintln!("CAD: {} faces, {v_cad:.3} mm3 (closed form {exact:.3})", cad.faces.len());
    assert!((v_cad - exact).abs() < 1e-3);
    let mut s = scanned_session(&cad, 221);
    let prims = planes(
        &mut s,
        &[
            ("bottom", [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
            ("top", [0.0, 0.0, 20.0], [0.0, 0.0, 1.0]),
            ("left", [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("right", [60.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("front", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ("back", [0.0, 50.0, 0.0], [0.0, 1.0, 0.0]),
            ("lower level", [0.0, 0.0, 10.0], [0.0, 0.0, 1.0]),
            ("riser", [40.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("rib x12", [12.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("rib x20", [20.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("rib y15", [0.0, 15.0, 0.0], [0.0, -1.0, 0.0]),
            ("rib y35", [0.0, 35.0, 0.0], [0.0, 1.0, 0.0]),
        ],
    );
    let plan = s.execute("body.plan_regions", json!({"primitives": prims})).unwrap();
    assert!(plan["tools"][0]["detail"].as_str().unwrap().contains("1 islands"), "{plan}");
    let (vol, max) = plan_and_check(&mut s, &prims, &cad, &["step"]);
    eprintln!("GATE p8-b2 step with a rib: volume {vol:.3} (CAD {v_cad:.3}), max deviation {max:.4}");
}

#[test]
fn gate_p8_b2_post_on_a_plate() {
    // a plate 50 × 40 × 8 with a rectangular post 10 × 10 up to z = 20: the plate's face is cut
    // down everywhere but under the post (no cylinder, so no boss)
    let plate = solid::make_box([0.0; 3], [50.0, 40.0, 8.0]).unwrap();
    let post = solid::make_box([20.0, 15.0, 7.0], [10.0, 10.0, 13.0]).unwrap();
    let cad = boolean(&plate, &post, BoolKind::Union).unwrap();
    let v_cad = solid::volume(&cad).unwrap();
    eprintln!("CAD: {} faces, {v_cad:.3} mm3 (closed form {:.3})", cad.faces.len(), 50.0 * 40.0 * 8.0 + 1200.0);
    assert!((v_cad - (50.0 * 40.0 * 8.0 + 10.0 * 10.0 * 12.0)).abs() < 1e-3);
    let mut s = scanned_session(&cad, 251);
    let prims = planes(
        &mut s,
        &[
            ("bottom", [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
            ("post top", [0.0, 0.0, 20.0], [0.0, 0.0, 1.0]),
            ("left", [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("right", [50.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("front", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ("back", [0.0, 40.0, 0.0], [0.0, 1.0, 0.0]),
            ("plate top", [0.0, 0.0, 8.0], [0.0, 0.0, 1.0]),
            ("post x20", [20.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("post x30", [30.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("post y15", [0.0, 15.0, 0.0], [0.0, -1.0, 0.0]),
            ("post y25", [0.0, 25.0, 0.0], [0.0, 1.0, 0.0]),
        ],
    );
    let (vol, max) = plan_and_check(&mut s, &prims, &cad, &["rib"]);
    eprintln!("GATE p8-b2 post on a plate: volume {vol:.3} (CAD {v_cad:.3}), max deviation {max:.4}");
}

/// A wall that is not the floor of anything is still refused, with a reason: a lone pocket wall
/// whose floor is missing from the regions must not become a step.
#[test]
fn p8_b2_a_pocket_wall_is_not_taken_for_a_floor() {
    let cad = block_less(&[rounded_rect(8.0, 8.0, 38.0, 28.0, 4.0, 12.0)]);
    let mut s = scanned_session(&cad, 281);
    let prims = planes(
        &mut s,
        &[
            ("bottom", [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
            ("top", [0.0, 0.0, 20.0], [0.0, 0.0, 1.0]),
            ("left", [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
            ("right", [60.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("front", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
            ("back", [0.0, 50.0, 0.0], [0.0, 1.0, 0.0]),
            ("wall x8", [8.0, 0.0, 0.0], [1.0, 0.0, 0.0]),
            ("wall y8", [0.0, 8.0, 0.0], [0.0, 1.0, 0.0]),
        ],
    );
    let plan = s.execute("body.plan_regions", json!({"primitives": prims})).unwrap();
    assert!(plan["tools"].as_array().unwrap().is_empty(), "{plan}");
    let skipped = plan["regions"].as_array().unwrap().iter().filter(|r| r["role"] == "skipped").count();
    assert_eq!(skipped, 2, "{plan}");
}
