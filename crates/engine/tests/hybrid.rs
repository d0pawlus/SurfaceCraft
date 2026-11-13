//! P3 gate: scanned primitives and freeform patches become ONE closed solid exported to STEP.
//!
//! The parts are built with the kernel (so their volume and surfaces are the ground truth),
//! tessellated, made to look like raw scans (`synth::scanify`, σ = 0.03 mm), cleaned,
//! segmented and fitted through the command layer, and then `body.from_regions` trims a stock
//! into the solid. Thresholds are those of design doc §11.1, unchanged.
//!
//! `tangency-labs/models` holds no models yet (checked 2026-11-08), so the second reference
//! part is a generated CAD part too; nothing binary is committed.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Rng, synth};
use surfacecraft_kernel::feature::{BoolKind, boolean};
use surfacecraft_kernel::{hybrid, nurbs, record::brep::Brep, solid};

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

fn vertical_edges(brep: &Brep) -> Vec<usize> {
    brep.edges
        .iter()
        .enumerate()
        .filter(|(_, e)| {
            let (p, q) = (brep.vertices[e.vertices[0] as usize].point, brep.vertices[e.vertices[1] as usize].point);
            (p[0] - q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9 && (p[2] - q[2]).abs() > 1.0
        })
        .map(|(i, _)| i)
        .collect()
}

/// z of the wavy top of part A.
fn wave(x: f64, y: f64) -> f64 {
    11.0 + 2.5 * (x / 9.0).sin() * (y / 7.0).cos()
}

fn wave_spline() -> surfacecraft_kernel::record::brep::Surface {
    let n = 40;
    let grid: Vec<Vec<[f64; 3]>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    let x = -4.0 + 48.0 * i as f64 / (n - 1) as f64;
                    let y = -4.0 + 38.0 * j as f64 / (n - 1) as f64;
                    [x, y, wave(x, y)]
                })
                .collect()
        })
        .collect();
    nurbs::fit_grid(&grid, 16, 16, 3).unwrap()
}

/// Volume of part A by quadrature (independent of the kernel): the footprint 40 × 30 with
/// corner radius 5, times the wavy height.
fn analytic_volume_a() -> f64 {
    let (nx, ny) = (1600, 1200);
    let (dx, dy) = (40.0 / nx as f64, 30.0 / ny as f64);
    let r = 5.0;
    let mut v = 0.0;
    for i in 0..nx {
        let x = (i as f64 + 0.5) * dx;
        for j in 0..ny {
            let y = (j as f64 + 0.5) * dy;
            let cx = x.clamp(r, 40.0 - r);
            let cy = y.clamp(r, 30.0 - r);
            if (x - cx).hypot(y - cy) <= r {
                v += wave(x, y) * dx * dy;
            }
        }
    }
    v
}

fn body_brep(s: &Session, name: &str) -> Brep {
    let id = s.doc.find_by_name(name).unwrap().0;
    (*s.doc.body(id).unwrap().brep).clone()
}

/// The checks shared by both parts: closed solid, volume against the CAD truth, scan-to-solid
/// deviation, STEP round trip. Returns (volume, max deviation).
fn check_solid(s: &mut Session, built: &Value, cad_volume: f64, faces: usize) -> (f64, f64) {
    let id = built["id"].as_u64().unwrap();
    let vol = built["volume"].as_f64().unwrap();
    let rel = (vol - cad_volume).abs() / cad_volume;
    eprintln!("hybrid volume {vol:.3} vs CAD {cad_volume:.3}: {:.4} %", rel * 100.0);
    assert!(rel < 0.01, "volume off by {} %", rel * 100.0);
    assert_eq!(built["faces"].as_u64().unwrap() as usize, faces, "{built}");
    assert_eq!(built["skipped"], 0, "{built}");
    // closed manifold solid, one solid, valid
    let brep = body_brep(s, &s.doc.body(surfacecraft_doc::ObjectId(id)).unwrap().name.clone());
    assert!(!surfacecraft_io::step::is_sheet(&brep));
    // deviation of the scan from the solid, over every vertex of the cleaned scan
    let t = s.execute("body.to_mesh", json!({"object": id, "chord": 0.01, "name": "solid mesh"})).unwrap();
    let mid = t["id"].as_u64().unwrap();
    let solid_mesh = (*s.doc.mesh(surfacecraft_doc::ObjectId(mid)).unwrap().mesh).clone();
    let scan = s.doc.find_by_name("scan").unwrap().0;
    let scan_mesh = s.doc.mesh(scan).unwrap().mesh.clone();
    let bvh = Bvh::build(&solid_mesh);
    let dev: Vec<f64> =
        surfacecraft_deviation::against_mesh(&scan_mesh.positions, &bvh, 5.0).iter().map(|d| d.abs()).collect();
    assert!(dev.iter().all(|d| d.is_finite()), "a scan vertex is more than 5 mm from the solid");
    let max = dev.iter().copied().fold(0.0, f64::max);
    let rms = (dev.iter().map(|d| d * d).sum::<f64>() / dev.len() as f64).sqrt();
    eprintln!("scan to solid: max {max:.4} mm, rms {rms:.4} mm over {} vertices", dev.len());
    let mut worst: Vec<(f64, usize)> = dev.iter().copied().zip(0..).collect();
    worst.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (d, i) in worst.iter().take(8) {
        let p = scan_mesh.positions[*i];
        eprintln!("  worst {d:.3} at ({:.2}, {:.2}, {:.2})", p.x, p.y, p.z);
    }
    let over = dev.iter().filter(|d| **d > 0.2).count();
    eprintln!("  {over} vertices over 0.2 mm");
    assert!(max <= 0.2, "max deviation {max} mm");
    // STEP round trip: same volume to 1e-6 relative, same face count
    let e = s.execute("export.step", json!({"objects": [id]})).unwrap();
    assert_eq!(e["content"], "solids");
    let back = s.execute("import.step", json!({"data": e["data"], "name": "reimported"})).unwrap();
    let bid = back["bodies"][0].as_u64().unwrap();
    eprintln!(
        "record: {} regions {} shells {} faces; reimported {back}",
        brep.regions.len(),
        brep.shells.len(),
        brep.faces.len()
    );
    assert_eq!(back["bodies"].as_array().unwrap().len(), 1);
    assert_eq!(back["faces"][0].as_u64().unwrap() as usize, faces);
    let info = s.execute("body.info", json!({"object": bid})).unwrap();
    let v2 = info["volume"].as_f64().unwrap();
    eprintln!("STEP round trip volume {v2:.6} vs {vol:.6}: {:.2e}", (v2 - vol).abs() / vol);
    assert!((v2 - vol).abs() / vol < 1e-6, "round trip volume {v2} vs {vol}");
    (vol, max)
}

/// Noise seeds of part A: the first is the one the gate has always run; with the other three the
/// gate failed before spikes next to each other were removed and holes over sharp edges were
/// filled with the edge (a top region that leaked down the walls, a filled bottom edge 0.27 mm
/// off, a top patch fitted with no margin).
const PART_A_SEEDS: [u64; 4] = [21, 8, 11, 12];

/// Gate P3, part A, for every seed of [`PART_A_SEEDS`].
#[test]
fn gate_p3_block_with_freeform_top() {
    for seed in PART_A_SEEDS {
        eprintln!("part A, seed {seed}");
        part_a(seed);
    }
}

/// Part A: a 40 × 30 block, vertical edges filleted r = 5, with a wavy freeform top
/// (CAD = box + fillet + split by a spline), scanned with noise `seed`, segmented and rebuilt.
/// Part A as a CAD body `block` in `s`: the filleted block cut by the wavy spline.
fn part_a_cad(s: &mut Session) -> Brep {
    s.execute("body.box", json!({"size": [40.0, 30.0, 20.0], "name": "block"})).unwrap();
    let edges = vertical_edges(&body_brep(s, "block"));
    assert_eq!(edges.len(), 4);
    s.execute("body.fillet", json!({"object": "block", "edges": edges, "size": 5.0})).unwrap();
    hybrid::cut_by_spline(&body_brep(s, "block"), &wave_spline(), [0.0, 0.0, 1.0]).unwrap()
}

fn part_a(seed: u64) {
    let mut s = Session::new();
    let cut = part_a_cad(&mut s);
    let v_cad = solid::volume(&cut).unwrap();
    let v_exact = analytic_volume_a();
    eprintln!("part A: CAD volume {v_cad:.3}, quadrature {v_exact:.3}, faces {}", cut.faces.len());
    assert!((v_cad - v_exact).abs() / v_exact < 1e-3, "CAD volume {v_cad} vs quadrature {v_exact}");
    let cad_faces = cut.faces.len();
    s.doc.add_body("part", cut);
    scan_of(&mut s, "part", seed);
    clean(&mut s);
    let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let regions = seg["regions"].as_array().unwrap().clone();
    // the CAD primitives: big planes aligned with the axes (the bottom and the four walls) and
    // vertical cylinders of the fillet radius; the wavy top falls into small unrelated pieces
    let mut prims: Vec<Value> = Vec::new();
    for r in &regions {
        if r["triangles"].as_u64().unwrap() < 200 {
            continue;
        }
        let kind = r["kind"].as_str().unwrap_or("none");
        if kind != "plane" && kind != "cylinder" {
            continue;
        }
        let f = s.execute("primitive.fit", json!({"region": r["id"], "kind": kind})).unwrap();
        let p = &f["primitive"];
        let keep = if kind == "plane" {
            let n = p["normal"].as_array().unwrap();
            let n: Vec<f64> = n.iter().map(|x| x.as_f64().unwrap()).collect();
            let axis_aligned = n.iter().any(|c| c.abs() > 0.999);
            let z0 = p["point"][2].as_f64().unwrap();
            // a (nearly) horizontal plane is the bottom only when it sits at z = 0
            axis_aligned && (n[2].abs() < 0.5 || z0.abs() < 0.5)
        } else {
            let a = p["axis"][2].as_f64().unwrap().abs();
            a > 0.99 && (p["radius"].as_f64().unwrap() - 5.0).abs() < 0.3
        };
        if keep {
            prims.push(f["id"].clone());
        }
    }
    eprintln!("part A: {} primitives chosen of {} regions", prims.len(), regions.len());
    // freeform top: grow from the highest triangle of the scan
    let scan_id = s.doc.find_by_name("scan").unwrap().0;
    let scan = s.doc.mesh(scan_id).unwrap().mesh.clone();
    let seed = (0..scan.triangle_count())
        .max_by(|&a, &b| scan.face_centroid(a).unwrap().z.total_cmp(&scan.face_centroid(b).unwrap().z))
        .unwrap();
    let top = s
        .execute("region.grow", json!({"mesh": "scan", "triangles": [seed], "max_angle_deg": 25.0, "name": "top"}))
        .unwrap();
    eprintln!("top region: {} triangles", top["triangles"]);
    // fitted tighter than the 0.2 mm the solid is held to, so the gate has a margin
    let fit = s.execute("surface.fit", json!({"region": top["id"], "tolerance": 0.15})).unwrap();
    eprintln!("top fit: control {} max {} rms {}", fit["control"], fit["max"], fit["rms"]);
    assert!(fit["max"].as_f64().unwrap() <= 0.2, "{fit}");
    let built = s
        .execute("body.from_regions", json!({"primitives": prims, "surfaces": [fit["id"]], "name": "hybrid"}))
        .unwrap();
    eprintln!("from_regions: {}", built["faces_of"]);
    let (vol, max) = check_solid(&mut s, &built, v_cad, cad_faces);
    eprintln!("GATE part A: volume {vol:.3} (CAD {v_cad:.3}), max deviation {max:.4}");
    // the trimmed faces: every input has exactly one face; sheets can be extracted
    let faces_of = built["faces_of"].as_array().unwrap();
    assert!(faces_of.iter().all(|f| f["faces"].as_array().unwrap().len() == 1), "{faces_of:?}");
    let hybrid_id = built["id"].clone();
    let t = s
        .execute("surface.trim_to_region", json!({"object": fit["id"], "body": hybrid_id, "name": "top sheet"}))
        .unwrap();
    assert_eq!(t["faces"], 1);
    assert!(t["volume"].as_f64().is_none_or(|v| v.abs() < 1e-9), "{t}");
    let p =
        s.execute("primitive.trim_to_region", json!({"object": prims[0], "body": hybrid_id, "name": "face"})).unwrap();
    assert_eq!(p["faces"], 1);
    // replace_face: the flat top of a plain block becomes the fitted patch
    s.execute("body.box", json!({"size": [40.0, 30.0, 20.0], "name": "flat"})).unwrap();
    let flat = body_brep(&s, "flat");
    let top_face =
        hybrid::faces_on_shape(&flat, &hybrid::Shape::Plane { point: [0.0, 0.0, 20.0], normal: [0.0, 0.0, 1.0] }, 1e-6);
    let r =
        s.execute("body.replace_face", json!({"object": "flat", "face": top_face[0], "surface": fit["id"]})).unwrap();
    // a box (the sharp corners) with the wavy top: 40 × 30 × mean height
    let vr = r["volume"].as_f64().unwrap();
    eprintln!("replace_face volume {vr:.2}");
    assert!(vr > 40.0 * 30.0 * 10.0 && vr < 40.0 * 30.0 * 12.0, "{r}");
    assert_eq!(r["faces"], 6);
}

/// Part B: the chamfered cylinder with a through hole: plane, plane, cylinder, cone and a
/// concave cylinder, all full revolutions.
#[test]
fn gate_p3_chamfered_cylinder_with_hole() {
    let mut s = Session::new();
    s.execute("body.cylinder", json!({"radius": 12.0, "height": 30.0, "name": "pin"})).unwrap();
    let b = body_brep(&s, "pin");
    let top = b
        .edges
        .iter()
        .position(|e| e.vertices.iter().all(|&v| (b.vertices[v as usize].point[2] - 30.0).abs() < 1e-9))
        .unwrap();
    s.execute("body.chamfer", json!({"object": "pin", "edges": [top], "size": 3.0})).unwrap();
    let hole = solid::make_cylinder([0.0, 0.0, -1.0], 4.0, 32.0).unwrap();
    let part = boolean(&body_brep(&s, "pin"), &hole, BoolKind::Subtract).unwrap();
    let v_cad = solid::volume(&part).unwrap();
    let pi = std::f64::consts::PI;
    let exact = pi * (144.0 - 16.0) * 30.0 - 2.0 * pi * 11.0 * 4.5;
    eprintln!("part B: CAD volume {v_cad:.3}, closed form {exact:.3}, faces {}", part.faces.len());
    assert!((v_cad - exact).abs() / exact < 1e-6);
    let cad_faces = part.faces.len();
    s.doc.add_body("part", part);
    scan_of(&mut s, "part", 8);
    clean(&mut s);
    let seg = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    for r in seg["regions"].as_array().unwrap() {
        eprintln!("  region {} tris {} kind {} rms {}", r["id"], r["triangles"], r["kind"], r["rms"]);
    }
    let big: Vec<Value> =
        seg["regions"].as_array().unwrap().iter().filter(|r| r["triangles"].as_u64().unwrap() > 400).cloned().collect();
    let mut kinds: Vec<&str> = big.iter().map(|r| r["kind"].as_str().unwrap_or("none")).collect();
    kinds.sort_unstable();
    eprintln!("part B regions: {kinds:?}");
    // the annular planes come out whole now (merging runs to a fixed point), no manual selection
    assert_eq!(kinds, ["cone", "cylinder", "cylinder", "plane", "plane"]);
    let prims: Vec<Value> =
        big.iter().map(|r| s.execute("primitive.fit", json!({"region": r["id"]})).unwrap()["id"].clone()).collect();
    let built = s.execute("body.from_regions", json!({"primitives": prims, "name": "hybrid"})).unwrap();
    eprintln!("from_regions: {}", built["faces_of"]);
    let (vol, max) = check_solid(&mut s, &built, v_cad, cad_faces);
    eprintln!("GATE part B: volume {vol:.3} (CAD {v_cad:.3}), max deviation {max:.4}");
}

/// Bad input is refused, not panicked on: empty, wrong kinds, regionless primitives.
#[test]
fn p3_commands_refuse_bad_input() {
    let mut s = Session::new();
    assert!(s.execute("body.from_regions", json!({})).is_err());
    assert!(s.execute("body.from_regions", json!({"primitives": [999]})).is_err());
    assert!(s.execute("body.from_regions", json!({"primitives": [], "surfaces": [], "margin": -1.0})).is_err());
    s.execute("body.box", json!({"size": [10.0, 10.0, 10.0], "name": "b"})).unwrap();
    assert!(s.execute("body.extract_face", json!({"object": "b", "face": 99})).is_err());
    assert!(s.execute("body.replace_face", json!({"object": "b", "face": 0, "surface": 1})).is_err());
    let f = s.execute("body.extract_face", json!({"object": "b", "face": 0, "name": "f"})).unwrap();
    assert_eq!(f["faces"], 1);
    assert!(f["volume"].as_f64().is_none_or(|v| v.abs() < 1e-9), "{f}");
}

fn patch_below() -> surfacecraft_kernel::record::brep::Surface {
    let n = 12;
    let grid: Vec<Vec<[f64; 3]>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    let x = -4.0 + 48.0 * i as f64 / (n - 1) as f64;
                    let y = -4.0 + 38.0 * j as f64 / (n - 1) as f64;
                    [x, y, 8.0 + (x / 9.0).sin() * (y / 7.0).cos()]
                })
                .collect()
        })
        .collect();
    nurbs::fit_grid(&grid, 8, 8, 3).unwrap()
}

/// P3b gate 3: `replace_face` with a patch that crosses the face (partly above, partly below)
/// adds and removes material. Volume against quadrature of the patch over the footprint, a
/// closed manifold solid, STEP round trip; a face that cannot be raised is refused.
#[test]
fn gate_p3b_replace_face_up_and_down() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [40.0, 30.0, 11.0], "name": "slab"})).unwrap();
    let slab = body_brep(&s, "slab");
    let plane = hybrid::Shape::Plane { point: [0.0, 0.0, 11.0], normal: [0.0, 0.0, 1.0] };
    let top = hybrid::faces_on_shape(&slab, &plane, 1e-6);
    assert_eq!(top.len(), 1);
    let patch = wave_spline();
    let rise = hybrid::patch_rise(&slab, top[0], &patch, [0.0, 0.0, 1.0]).unwrap();
    assert!(rise > 2.0, "the patch must rise above the face, rise {rise}");
    let r = hybrid::replace_face(&slab, top[0], &patch).unwrap();
    // quadrature of the patch height over the footprint (the wave is the patch's own formula)
    let (nx, ny) = (2000, 1500);
    let (dx, dy) = (40.0 / nx as f64, 30.0 / ny as f64);
    let mut exact = 0.0;
    for i in 0..nx {
        for j in 0..ny {
            exact += wave((i as f64 + 0.5) * dx, (j as f64 + 0.5) * dy) * dx * dy;
        }
    }
    let v = solid::volume(&r).unwrap();
    let rel = (v - exact).abs() / exact;
    eprintln!(
        "GATE p3b replace_face up+down: volume {v:.4} vs quadrature {exact:.4}: {:.4} % (rise {rise:.3}, faces {})",
        rel * 100.0,
        r.faces.len()
    );
    assert!(rel < 1e-3, "volume {v} vs quadrature {exact}");
    assert!(!surfacecraft_io::step::is_sheet(&r));
    // STEP round trip through the engine
    let id = s.doc.add_body("up and down", r);
    let e = s.execute("export.step", json!({"objects": [id.0]})).unwrap();
    assert_eq!(e["content"], "solids");
    let back = s.execute("import.step", json!({"data": e["data"], "name": "again"})).unwrap();
    assert_eq!(back["bodies"].as_array().unwrap().len(), 1);
    let v2 = s.execute("body.info", json!({"object": back["bodies"][0]})).unwrap()["volume"].as_f64().unwrap();
    assert!((v2 - v).abs() / v < 1e-6, "round trip volume {v2} vs {v}");
    // a patch entirely below keeps the old behaviour (material only removed)
    let low = hybrid::replace_face(&slab, top[0], &patch_below()).unwrap();
    assert!(solid::volume(&low).unwrap() < 40.0 * 30.0 * 11.0);
    // collision: an overhang above the face blocks the extension
    let base = solid::make_box([0.0, 0.0, 0.0], [40.0, 30.0, 11.0]).unwrap();
    let post = solid::make_box([-10.0, 0.0, 0.0], [10.0, 30.0, 14.0]).unwrap();
    let arm = solid::make_box([-10.0, 0.0, 12.0], [40.0, 30.0, 7.0]).unwrap();
    let tee = boolean(&boolean(&base, &post, BoolKind::Union).unwrap(), &arm, BoolKind::Union).unwrap();
    let t_top = hybrid::faces_on_shape(&tee, &plane, 1e-6);
    assert_eq!(t_top.len(), 1);
    let err = hybrid::replace_face(&tee, t_top[0], &patch).unwrap_err();
    eprintln!("collision refused: {err}");
    assert!(err.to_string().contains("cannot be raised"), "{err}");
}

/// The CAD primitives of part A among segmentation `regions`, chosen as the gate of part A does
/// (the axis-aligned planes and the vertical r = 5 cylinders of at least 200 triangles): sorted
/// `(kind, triangles)`.
fn cad_primitives_of_part_a(s: &mut Session, regions: &[Value]) -> Vec<(String, u64)> {
    let mut out = Vec::new();
    for r in regions.iter().filter(|r| r["triangles"].as_u64().unwrap() >= 200) {
        let Some(kind) = r["kind"].as_str().filter(|k| *k == "plane" || *k == "cylinder") else { continue };
        let f = s.execute("primitive.fit", json!({"region": r["id"], "kind": kind})).unwrap();
        let p = &f["primitive"];
        let keep = if kind == "plane" {
            let n: Vec<f64> = p["normal"].as_array().unwrap().iter().map(|x| x.as_f64().unwrap()).collect();
            let z0 = p["point"][2].as_f64().unwrap();
            n.iter().any(|c| c.abs() > 0.999) && (n[2].abs() < 0.5 || z0.abs() < 0.5)
        } else {
            p["axis"][2].as_f64().unwrap().abs() > 0.99 && (p["radius"].as_f64().unwrap() - 5.0).abs() < 0.3
        };
        if keep {
            out.push((kind.to_string(), r["triangles"].as_u64().unwrap()));
        }
    }
    out.sort();
    out
}

/// What segmenting part A with noise seed `seed` gave: structural `problems` (CAD primitives
/// that differ from the plain segmentation, not exactly one freeform region, one that runs down
/// the walls), the share of the top's triangles in the freeform region, and what else holds the
/// top (`(kind, triangles, top triangles)` of every other region with some).
struct TopCase {
    problems: Vec<String>,
    share: f64,
    others: Vec<(String, u64, usize)>,
}

fn wavy_top_case(seed: u64) -> TopCase {
    let mut s = Session::new();
    let cut = part_a_cad(&mut s);
    s.doc.add_body("part", cut);
    scan_of(&mut s, "part", seed);
    clean(&mut s);
    let old = s.execute("region.auto_segment", json!({"mesh": "scan", "options": {"freeform": false}})).unwrap();
    let old_regions = old["regions"].as_array().unwrap().clone();
    let cad_old = cad_primitives_of_part_a(&mut s, &old_regions);
    let new = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let regions = new["regions"].as_array().unwrap().clone();
    let cad_new = cad_primitives_of_part_a(&mut s, &regions);
    let mut problems = Vec::new();
    if cad_new != cad_old {
        problems.push(format!("CAD primitives before {cad_old:?}, after {cad_new:?}"));
    }
    let freeform: Vec<&Value> = regions.iter().filter(|r| r["freeform"] == true).collect();
    if freeform.len() != 1 {
        problems.push(format!("{} freeform regions", freeform.len()));
    }
    let scan_id = s.doc.find_by_name("scan").unwrap().0;
    let scan = s.doc.mesh(scan_id).unwrap().mesh.clone();
    // the top: triangles facing up, in the upper half of the part
    let is_top =
        |t: usize| matches!((scan.face_normal(t), scan.face_centroid(t)), (Some(n), Some(c)) if n.z > 0.5 && c.z > 8.0);
    let top = (0..scan.triangle_count()).filter(|&t| is_top(t)).count();
    let mut in_freeform = 0;
    let mut others = Vec::new();
    for r in &regions {
        let tris = s.doc.region(surfacecraft_doc::ObjectId(r["id"].as_u64().unwrap())).unwrap().triangles.clone();
        let on_top = tris.iter().filter(|&&t| is_top(t as usize)).count();
        if r["freeform"] == true {
            in_freeform += on_top;
            let walls = tris.iter().filter(|&&t| scan.face_centroid(t as usize).is_some_and(|c| c.z < 6.0)).count();
            if walls * 50 > tris.len() {
                problems.push(format!("the freeform region runs down the walls ({walls})"));
            }
        } else if on_top > 0 {
            others.push((r["kind"].to_string(), r["triangles"].as_u64().unwrap(), on_top));
        }
    }
    TopCase { problems, share: in_freeform as f64 / top.max(1) as f64, others }
}

/// Noise seeds of the wavy top gate: those of part A, the worst ones of a sweep of 1..=40 (before the
/// islands were joined again after each absorption, the freeform region held only 86 to 93 % of the
/// top for these, and on Linux's different tessellation under half) and seed 77, the worst of the
/// sweep of 1..=80 (0.861). The gate asks for 85 % of the top on every one of them.
const WAVY_TOP_SEEDS: [u64; 12] = [21, 8, 11, 12, 18, 17, 22, 26, 19, 16, 14, 77];

/// P9-O7, part A: the walls and the bottom come out as the same primitives as without the
/// consolidation, and the wavy top (it fell into small unrelated pieces) is ONE freeform region
/// that holds nearly all of it and nothing of the walls.
#[test]
fn p9_block_with_wavy_top_has_one_freeform_top() {
    for seed in WAVY_TOP_SEEDS {
        let c = wavy_top_case(seed);
        eprintln!("seed {seed}: top share {:.3}, other regions on the top {:?}", c.share, c.others);
        assert!(c.problems.is_empty(), "seed {seed}: {:?}", c.problems);
        assert!(
            c.share >= 0.85,
            "seed {seed}: {:.1} % of the top in the freeform region, rest in {:?}",
            100.0 * c.share,
            c.others
        );
    }
}

/// Sweep of noise seeds (`--ignored --nocapture`): the share per seed.
#[test]
#[ignore = "sweep"]
fn p9_wavy_top_seed_sweep() {
    let (lo, hi): (u64, u64) = (
        std::env::var("SWEEP_LO").ok().and_then(|v| v.parse().ok()).unwrap_or(1),
        std::env::var("SWEEP_HI").ok().and_then(|v| v.parse().ok()).unwrap_or(40),
    );
    for seed in lo..=hi {
        let c = std::panic::catch_unwind(|| wavy_top_case(seed));
        match c {
            Ok(c) => eprintln!("SWEEP {seed} share {:.3} problems {:?} others {:?}", c.share, c.problems, c.others),
            Err(_) => eprintln!("SWEEP {seed} panicked"),
        }
    }
}

/// P9-O7: a prismatic bracket has no freeform region and the primitives of before.
#[test]
fn p9_bracket_has_no_freeform_region() {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "bracket", "noise": 0.05, "seed": 4, "name": "scan"})).unwrap();
    let out = s.execute("region.auto_segment", json!({"mesh": "scan"})).unwrap();
    let regions = out["regions"].as_array().unwrap();
    assert!(regions.iter().all(|r| r["freeform"] == false), "{out}");
    let mut kinds: Vec<&str> = regions.iter().map(|r| r["kind"].as_str().unwrap()).collect();
    kinds.sort_unstable();
    assert_eq!(kinds, ["cylinder", "plane", "plane", "plane", "plane", "plane", "plane", "sphere"]);
}
