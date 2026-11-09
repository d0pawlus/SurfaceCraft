//! P3b gate 2 (plan Z3): partial shapes as rounds, chamfers and local solids. A plate with filleted
//! vertical edges that meet rounded top edges at the corners (sphere corner patches), and a
//! cylindrical boss with a concave fillet at its base (a torus) and a chamfer on its top (a cone),
//! is built with the kernel (the ground truth), made to look like a raw scan (`synth::scanify`,
//! σ = 0.03 mm), cleaned, cut into regions by geometric selection (as in P3's part B and the P3b
//! pocket gate: auto_segment splits big planes and mislabels the small round surfaces) and fitted
//! through the command layer, and rebuilt by `body.from_regions`, whose planner has to tell the
//! plate's planes (global) from the boss (a local addition) and the rounds (blends the kernel makes
//! at the edges). Thresholds are those of design doc §11.1.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Rng, synth};
use surfacecraft_kernel::feature::{BoolKind, boolean};
use surfacecraft_kernel::record::brep::{Brep, Curve};
use surfacecraft_kernel::solid;

fn scan_of(s: &mut Session, body: &str, seed: u64) -> Mesh {
    s.execute("body.to_mesh", json!({"object": body, "chord": 0.01, "name": "cad"})).unwrap();
    let id = s.doc.find_by_name("cad").unwrap().0;
    let mut cad = (*s.doc.mesh(id).unwrap().mesh).clone();
    cad.weld(1e-6);
    let d = synth::ScanDefects { spacing: 0.7, noise: 0.03, ..synth::ScanDefects::default() };
    let scan = synth::scanify(&cad, &d, &mut Rng::new(seed));
    s.doc.add_mesh("scan", scan);
    cad
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

/// The triangles of the scan whose centroid satisfies `pick` (at least `min` of them), as a
/// region fitted as `kind`.
fn fit_selection(s: &mut Session, name: &str, kind: &str, min: usize, pick: impl Fn([f64; 3]) -> bool) -> Value {
    let scan_id = s.doc.find_by_name("scan").unwrap().0;
    let scan = s.doc.mesh(scan_id).unwrap().mesh.clone();
    let tris: Vec<u32> = (0..scan.triangle_count() as u32)
        .filter(|&t| {
            let c = scan.face_centroid(t as usize).unwrap();
            pick([c.x, c.y, c.z])
        })
        .collect();
    assert!(tris.len() > min, "{name}: {} triangles", tris.len());
    let r = s.execute("region.create", json!({"mesh": "scan", "triangles": tris, "name": name})).unwrap();
    let f = s.execute("primitive.fit", json!({"region": r["id"], "kind": kind})).unwrap();
    eprintln!("  {name}: {} triangles, {}", r["triangles"], f["primitive"]);
    f["id"].clone()
}

/// Region of the scan triangles lying on a plane (centroid within 0.12 mm, normal within ~37°).
fn plane_primitive(s: &mut Session, name: &str, point: [f64; 3], normal: [f64; 3]) -> Value {
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
    assert!(tris.len() > 150, "{name}: {} triangles", tris.len());
    let r = s.execute("region.create", json!({"mesh": "scan", "triangles": tris, "name": name})).unwrap();
    s.execute("primitive.fit", json!({"region": r["id"], "kind": "plane"})).unwrap()["id"].clone()
}

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

/// The edges (record indices) of `part` whose two vertices satisfy `pick`.
fn edges_where(part: &Brep, pick: impl Fn([f64; 3], [f64; 3]) -> bool, circle: Option<bool>) -> Vec<usize> {
    part.edges
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            let a = part.vertices[e.vertices[0] as usize].point;
            let b = part.vertices[e.vertices[1] as usize].point;
            circle.is_none_or(|c| matches!(e.curve, Curve::Circle { .. }) == c) && pick(a, b)
        })
        .map(|(i, _)| i)
        .collect()
}

const PLATE: [f64; 3] = [60.0, 40.0, 8.0];
const ROUND: f64 = 5.0;
const BOSS_AT: [f64; 2] = [30.0, 20.0];
const BOSS_R: f64 = 8.0;
const BOSS_TOP: f64 = 20.0;
const FOOT: f64 = 2.0;
const CHAMFER: f64 = 1.5;

/// The plate 60 × 40 × 8 with its four vertical edges and four top edges rounded (r = 5, one blend
/// call: sphere corner patches), a boss Ø16 up to z = 20 with a concave r = 2 fillet at its foot and
/// a 1.5 mm chamfer on its top edge. `with_boss_features` false leaves the boss's edges sharp.
fn plate_with_boss(with_boss_features: bool) -> Brep {
    let plate = solid::make_box([0.0; 3], PLATE).unwrap();
    let rounded_edges = edges_where(
        &plate,
        |a, b| {
            let vertical = (a[2] - b[2]).abs() > 1.0;
            let top = (a[2] - PLATE[2]).abs() < 1e-9 && (b[2] - PLATE[2]).abs() < 1e-9;
            vertical || top
        },
        None,
    );
    assert_eq!(rounded_edges.len(), 8);
    let plate = solid::fillet(&plate, &rounded_edges, ROUND).unwrap();
    let boss =
        solid::make_cylinder([BOSS_AT[0], BOSS_AT[1], PLATE[2] - 1.0], BOSS_R, BOSS_TOP - PLATE[2] + 1.0).unwrap();
    let mut part = boolean(&plate, &boss, BoolKind::Union).unwrap();
    if with_boss_features {
        let foot =
            edges_where(&part, |a, b| (a[2] - PLATE[2]).abs() < 1e-6 && (b[2] - PLATE[2]).abs() < 1e-6, Some(true));
        // the boss's foot is the circle of radius 8 about (30, 20)
        let foot: Vec<usize> = foot
            .into_iter()
            .filter(|&e| {
                let v = part.vertices[part.edges[e].vertices[0] as usize].point;
                ((v[0] - BOSS_AT[0]).hypot(v[1] - BOSS_AT[1]) - BOSS_R).abs() < 1e-6
            })
            .collect();
        assert_eq!(foot.len(), 1, "the boss's foot circle");
        part = solid::fillet(&part, &foot, FOOT).unwrap();
        let top =
            edges_where(&part, |a, b| (a[2] - BOSS_TOP).abs() < 1e-6 && (b[2] - BOSS_TOP).abs() < 1e-6, Some(true));
        assert_eq!(top.len(), 1, "the boss's top circle");
        part = solid::chamfer(&part, &top, CHAMFER).unwrap();
    }
    part
}

#[test]
fn gate_p3b_plate_with_boss() {
    let cad = plate_with_boss(true);
    let v_cad = solid::volume(&cad).unwrap();
    eprintln!("CAD: {} faces, {v_cad:.3} mm3", cad.faces.len());
    // plate 6 + 4 vertical rounds + 4 top rounds + 4 corner patches; boss wall, top, foot torus, chamfer cone
    let faces = cad.faces.len();
    assert_eq!(faces, 6 + 4 + 4 + 4 + 4, "faces of the CAD part");
    let mut s = scanned_session(&cad, 11);
    let mut prims: Vec<Value> = Vec::new();
    for (name, p, n) in [
        ("bottom", [0.0, 0.0, 0.0], [0.0, 0.0, -1.0]),
        ("top", [0.0, 0.0, PLATE[2]], [0.0, 0.0, 1.0]),
        ("left", [0.0, 0.0, 0.0], [-1.0, 0.0, 0.0]),
        ("right", [PLATE[0], 0.0, 0.0], [1.0, 0.0, 0.0]),
        ("front", [0.0, 0.0, 0.0], [0.0, -1.0, 0.0]),
        ("back", [0.0, PLATE[1], 0.0], [0.0, 1.0, 0.0]),
        ("boss top", [0.0, 0.0, BOSS_TOP], [0.0, 0.0, 1.0]),
    ] {
        prims.push(plane_primitive(&mut s, name, p, n));
    }
    let zc = PLATE[2] - ROUND; // height of the round's centre line: 3
    let corners =
        [[ROUND, ROUND], [PLATE[0] - ROUND, ROUND], [ROUND, PLATE[1] - ROUND], [PLATE[0] - ROUND, PLATE[1] - ROUND]];
    let toward = |c: [f64; 2]| (if c[0] < 30.0 { -1.0 } else { 1.0 }, if c[1] < 20.0 { -1.0 } else { 1.0 });
    for (k, c) in corners.iter().enumerate() {
        let (sx, sy) = toward(*c);
        let c = *c;
        prims.push(fit_selection(&mut s, &format!("vertical round {k}"), "cylinder", 40, |p| {
            ((p[0] - c[0]).hypot(p[1] - c[1]) - ROUND).abs() < 0.12
                && (p[0] - c[0]) * sx > 0.1
                && (p[1] - c[1]) * sy > 0.1
                && p[2] < zc - 0.2
        }));
        prims.push(fit_selection(&mut s, &format!("corner sphere {k}"), "sphere", 40, |p| {
            ((p[0] - c[0]).hypot(p[1] - c[1]).hypot(p[2] - zc) - ROUND).abs() < 0.12
                && (p[0] - c[0]) * sx > 0.15
                && (p[1] - c[1]) * sy > 0.15
                && p[2] > zc + 0.2
        }));
    }
    // the rounded top edges: along x at the front and back, along y at left and right
    for (k, y0) in [ROUND, PLATE[1] - ROUND].into_iter().enumerate() {
        let sy = if y0 < 20.0 { -1.0 } else { 1.0 };
        prims.push(fit_selection(&mut s, &format!("top round x {k}"), "cylinder", 100, |p| {
            ((p[1] - y0).hypot(p[2] - zc) - ROUND).abs() < 0.12
                && (p[1] - y0) * sy > 0.1
                && p[2] > zc + 0.1
                && p[0] > ROUND + 0.3
                && p[0] < PLATE[0] - ROUND - 0.3
        }));
    }
    for (k, x0) in [ROUND, PLATE[0] - ROUND].into_iter().enumerate() {
        let sx = if x0 < 30.0 { -1.0 } else { 1.0 };
        prims.push(fit_selection(&mut s, &format!("top round y {k}"), "cylinder", 60, |p| {
            ((p[0] - x0).hypot(p[2] - zc) - ROUND).abs() < 0.12
                && (p[0] - x0) * sx > 0.1
                && p[2] > zc + 0.1
                && p[1] > ROUND + 0.3
                && p[1] < PLATE[1] - ROUND - 0.3
        }));
    }
    // the boss: wall, foot fillet (torus) and chamfer (cone)
    let rho = |p: [f64; 3]| (p[0] - BOSS_AT[0]).hypot(p[1] - BOSS_AT[1]);
    let z_foot = PLATE[2] + FOOT;
    let z_chamfer = BOSS_TOP - CHAMFER;
    prims.push(fit_selection(&mut s, "boss wall", "cylinder", 100, |p| {
        (rho(p) - BOSS_R).abs() < 0.12 && p[2] > z_foot + 0.2 && p[2] < z_chamfer - 0.1
    }));
    prims.push(fit_selection(&mut s, "boss foot", "torus", 100, |p| {
        // the quarter round about (rho = 10, z = 10) of radius 2, inside the corner
        ((rho(p) - (BOSS_R + FOOT)).hypot(p[2] - z_foot) - FOOT).abs() < 0.1
            && rho(p) < BOSS_R + FOOT - 0.05
            && p[2] < z_foot - 0.05
            && p[2] > PLATE[2] + 0.05
    }));
    prims.push(fit_selection(&mut s, "boss chamfer", "cone", 100, |p| {
        // the 45° line from (rho = 8, z = 18.5) to (rho = 6.5, z = 20)
        ((rho(p) + p[2]) - (BOSS_R + z_chamfer)).abs() / 2f64.sqrt() < 0.12
            && p[2] > z_chamfer + 0.1
            && p[2] < BOSS_TOP - 0.1
    }));

    // --- the plan: every region has a role, none is skipped
    let plan = s.execute("body.plan_regions", json!({"primitives": prims})).unwrap();
    eprintln!("plan: {}", serde_json::to_string_pretty(&plan["regions"]).unwrap());
    let roles: Vec<&str> = plan["regions"].as_array().unwrap().iter().map(|r| r["role"].as_str().unwrap()).collect();
    assert!(!roles.contains(&"skipped"), "{plan}");
    assert_eq!(plan["adds"].as_array().unwrap().len(), 1, "one boss: {plan}");
    assert_eq!(plan["adds"][0]["kind"], "boss", "{plan}");
    assert_eq!(roles.iter().filter(|r| **r == "local-add").count(), 2, "boss wall and top: {plan}");
    // 4 + 4 rounds, 4 corner patches, the foot torus and the chamfer cone are blends
    assert_eq!(roles.iter().filter(|r| **r == "blend").count(), 4 + 4 + 4 + 2, "{plan}");
    assert_eq!(roles.iter().filter(|r| **r == "global").count(), 6, "{plan}");

    let built = s.execute("body.from_regions", json!({"primitives": prims, "name": "hybrid"})).unwrap();
    eprintln!("from_regions: {}", serde_json::to_string_pretty(&built["faces_of"]).unwrap());
    let id = built["id"].as_u64().unwrap();
    let vol = built["volume"].as_f64().unwrap_or_else(|| panic!("the solid has no volume: {}", built["faces_of"]));
    let rel = (vol - v_cad).abs() / v_cad;
    eprintln!("volume {vol:.3} vs CAD {v_cad:.3}: {:.4} %", rel * 100.0);
    assert!(rel <= 0.005, "volume off by {} %", rel * 100.0);
    assert_eq!(built["faces"].as_u64().unwrap() as usize, faces, "{}", built["faces_of"]);
    assert_eq!(built["skipped"], 0, "{}", built["faces_of"]);
    let dev = deviation(&mut s, id);
    assert!(dev.iter().all(|d| d.0.is_finite()), "a scan vertex is more than 5 mm from the solid");
    let max = dev.iter().map(|d| d.0).fold(0.0, f64::max);
    let rms = (dev.iter().map(|d| d.0 * d.0).sum::<f64>() / dev.len() as f64).sqrt();
    eprintln!("scan to solid: max {max:.4} mm, rms {rms:.4} mm over {} vertices", dev.len());
    let mut worst: Vec<&(f64, [f64; 3])> = dev.iter().collect();
    worst.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, p) in worst.iter().take(6) {
        eprintln!("  worst {d:.3} at ({:.2}, {:.2}, {:.2})", p[0], p[1], p[2]);
    }
    assert!(max <= 0.2, "max deviation {max} mm");
    // STEP round trip: same volume to 1e-6 relative, same face count
    let e = s.execute("export.step", json!({"objects": [id]})).unwrap();
    assert_eq!(e["content"], "solids");
    let back = s.execute("import.step", json!({"data": e["data"], "name": "reimported"})).unwrap();
    assert_eq!(back["bodies"].as_array().unwrap().len(), 1, "{back}");
    assert_eq!(back["faces"][0].as_u64().unwrap() as usize, faces, "{back}");
    let bid = back["bodies"][0].as_u64().unwrap();
    let v2 = s.execute("body.info", json!({"object": bid})).unwrap()["volume"].as_f64().unwrap();
    eprintln!("STEP round trip volume {v2:.6} vs {vol:.6}: {:.2e}", (v2 - vol).abs() / vol);
    assert!((v2 - vol).abs() / vol < 1e-6, "round trip volume {v2} vs {vol}");
    eprintln!("GATE p3b plate with boss: volume {vol:.3} (CAD {v_cad:.3}), faces {faces}, max deviation {max:.4}");
}
