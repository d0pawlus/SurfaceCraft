//! P9-O6 gates: `scan.to_solid` with the hybrid method on a synthetic organic part with a flat cut and
//! a cylindrical boss, scanned with noise. The part is an organic blob (a freeform patch network, the
//! ground truth made with the kernel), cut flat and given a boss; its scan goes through the command.
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};
use surfacecraft_kernel::feature::{self, BoolKind};
use surfacecraft_kernel::record::brep::{Brep, Surface};
use surfacecraft_kernel::solid;

/// The flat cut: the blob is cut at this height.
const FLAT_Z: f64 = 15.0;
const BOSS_R: f64 = 5.0;
const BOSS_TOP: f64 = 24.0;
const NOISE: f64 = 0.03;

fn blob_radius(d: &Vec3) -> f64 {
    20.0 * (1.0
        + 0.09 * (3.0 * d.x).sin() * (2.0 * d.y).cos()
        + 0.07 * (2.0 * d.z + 1.2 * d.x).sin() * (3.0 * d.y).cos()
        + 0.05 * (4.0 * d.z).cos() * d.x)
}

fn blob_mesh() -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, 200, 100);
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = d * blob_radius(&d);
    }
    m
}

/// Faces of the record by kind of surface: (planes, cylinders, splines, others).
fn kinds(b: &Brep) -> (usize, usize, usize, usize) {
    let mut k = (0, 0, 0, 0);
    for f in &b.faces {
        match f.surface {
            Surface::Plane { .. } | Surface::PlaneThrough { .. } => k.0 += 1,
            Surface::Cylinder { .. } => k.1 += 1,
            Surface::Spline { .. } | Surface::RationalSpline { .. } => k.2 += 1,
            _ => k.3 += 1,
        }
    }
    k
}

/// The ground truth: the organic blob as a patch network, cut flat at `FLAT_Z` and with a cylindrical
/// boss standing on the flat.
fn cad_part(s: &mut Session) -> Brep {
    s.doc.add_mesh("blob", blob_mesh());
    let w = s.execute("surface.wrap", json!({"mesh": "blob", "tolerance": 0.05, "max_patches": 3000})).unwrap();
    let network = (*s.doc.body(ObjectId(w["body"]["id"].as_u64().unwrap())).unwrap().brep).clone();
    let cut = feature::split(&network, [0.0, 0.0, FLAT_Z], [0.0, 0.0, 1.0], false).unwrap();
    let boss = solid::make_cylinder([0.0, 0.0, FLAT_Z - 5.0], BOSS_R, BOSS_TOP - FLAT_Z + 5.0).unwrap();
    feature::boolean(&cut, &boss, BoolKind::Union).unwrap()
}

/// A session holding the cleaned scan "scan" of the CAD part.
fn scanned_session(seed: u64) -> (Session, Brep) {
    let mut s = Session::new();
    let part = cad_part(&mut s);
    s.doc.add_body("part", part.clone());
    s.execute("body.to_mesh", json!({"object": "part", "chord": 0.01, "name": "cad"})).unwrap();
    let id = s.doc.find_by_name("cad").unwrap().0;
    let mut cad = (*s.doc.mesh(id).unwrap().mesh).clone();
    cad.weld(1e-6);
    let d = synth::ScanDefects { spacing: 0.7, noise: NOISE, ..synth::ScanDefects::default() };
    s.doc.add_mesh("scan", synth::scanify(&cad, &d, &mut Rng::new(seed)));
    (s, part)
}

fn to_solid(s: &mut Session, method: &str) -> Value {
    s.execute("scan.to_solid", json!({"scan": "scan", "method": method, "tolerance": 0.2, "name": method})).unwrap()
}

fn body_of(s: &Session, r: &Value) -> Brep {
    let id = ObjectId(r["body"]["id"].as_u64().expect("a body"));
    (*s.doc.body(id).expect("the body is in the document").brep).clone()
}

/// Share of the scan's vertices within `tol` of the body, measured on a fine tessellation (not the
/// command's own comparison).
fn share_within(s: &mut Session, body: u64, tol: f64) -> f64 {
    let t = s.execute("body.to_mesh", json!({"object": body, "chord": 0.01, "name": "result mesh"})).unwrap();
    let mesh = (*s.doc.mesh(ObjectId(t["id"].as_u64().unwrap())).unwrap().mesh).clone();
    let scan = s.doc.mesh(s.doc.find_by_name("scan").unwrap().0).unwrap().mesh.clone();
    let bvh = surfacecraft_geom::Bvh::build(&mesh);
    let dev = surfacecraft_deviation::against_mesh(&scan.positions, &bvh, 5.0);
    dev.iter().filter(|d| d.abs() <= tol).count() as f64 / dev.len() as f64
}

/// The gate: the flat cut and the boss of an organic part become analytic faces of one closed body,
/// the rest stays patches, the body matches the scan and the CAD volume, survives STEP, and is one
/// undo step. The same scan modelled as freeform has no analytic face (so the test fails without the
/// feature).
#[test]
fn the_hybrid_method_puts_the_flat_cut_and_the_boss_on_analytic_faces() {
    let (mut s, part) = scanned_session(5);
    let cad_volume = solid::volume(&part).unwrap();
    let docs_before = (s.doc.bodies.len(), s.doc.primitives.len());
    let r = to_solid(&mut s, "hybrid");
    assert_eq!(r["method"], "hybrid", "{r}");
    assert_eq!(r["verdict"], "pass", "{r}");
    // one undo step takes the whole run back (the clean-up of the scan, the primitives, the body); redo
    // brings it back
    assert_eq!(s.doc.bodies.len(), docs_before.0 + 1, "one body was added");
    assert!(s.doc.primitives.len() > docs_before.1, "the fitted primitives stay");
    s.execute("edit.undo", Value::Null).unwrap();
    assert_eq!((s.doc.bodies.len(), s.doc.primitives.len()), docs_before);
    s.execute("edit.redo", Value::Null).unwrap();
    let brep = body_of(&s, &r);
    let (planes, cylinders, splines, others) = kinds(&brep);
    eprintln!("hybrid: {planes} planes, {cylinders} cylinders, {splines} splines, {others} others; {}", r["analytic"]);
    assert!(planes >= 1 && cylinders == 1, "the flat cut is a plane and the boss wall a cylinder");
    assert!(splines > 50, "the organic rest is still patches: {splines}");
    assert_eq!(r["analytic"]["faces"].as_u64().unwrap() as usize, planes + cylinders + others);
    assert_eq!(r["analytic"]["total_faces"].as_u64().unwrap() as usize, brep.faces.len());
    let share = r["analytic"]["area_share"].as_f64().unwrap();
    assert!((0.08..0.5).contains(&share), "analytic area share {share}");
    // every planned primitive that was applied reports its faces
    let with_faces = r["modelled"].as_array().unwrap().iter().filter(|m| !m["faces"].as_array().unwrap().is_empty());
    assert!(with_faces.count() >= 2, "{r}");
    // closed, the CAD's volume
    let id = r["body"]["id"].as_u64().unwrap();
    let v = r["body"]["volume"].as_f64().expect("a closed solid has a volume");
    assert!((v - cad_volume).abs() / cad_volume < 0.01, "volume {v} against the CAD's {cad_volume}");
    // independent deviation from the scan
    let within = share_within(&mut s, id, 0.2);
    assert!(within >= 0.985, "{within} of the scan within 0.2 mm of the body");
    assert!(r["deviation"]["within"].as_f64().unwrap() >= 0.99, "{r}");
    // STEP keeps the face count and the volume
    let e = s.execute("export.step", json!({"objects": [id]})).unwrap();
    let back = s.execute("import.step", json!({"data": e["data"], "name": "reimported"})).unwrap();
    assert_eq!(back["faces"][0].as_u64().unwrap() as usize, brep.faces.len());
    let bid = back["bodies"][0].as_u64().unwrap();
    let v2 = s.execute("body.info", json!({"object": bid})).unwrap()["volume"].as_f64().unwrap();
    assert!((v2 - v).abs() / v < 1e-6, "volume {v2} after the STEP round trip, {v} before");
    // the freeform method on the same scan has no analytic face
    let f = to_solid(&mut s, "freeform");
    let (fp, fc, fs, fo) = kinds(&body_of(&s, &f));
    assert_eq!((fp, fc, fo), (0, 0, 0), "a patch network has no analytic face");
    assert!(fs > 50);
    assert!(f["analytic"].is_null());
}

/// `auto` tries the hybrid method as its third attempt on a scan with enough primitives and returns it
/// when it is not worse: here the freeform wrap passes as well, and the analytic faces decide. The
/// two extra attempts of the call's budget are the wrap and the hybrid body.
#[test]
fn auto_returns_the_hybrid_body_when_the_patch_network_is_no_better() {
    let (mut s, _) = scanned_session(5);
    let bodies_before = s.doc.bodies.len();
    let r = to_solid(&mut s, "auto");
    let attempts = r["attempts"].as_array().unwrap();
    let methods: Vec<&str> = attempts.iter().map(|a| a["method"].as_str().unwrap()).collect();
    eprintln!("auto: {methods:?}, returned {}, analytic {}", r["method"], r["analytic"]);
    assert_eq!(methods.last(), Some(&"hybrid"), "{r}");
    assert!(methods.contains(&"freeform"), "the patch network was tried too: {methods:?}");
    let returned: Vec<&str> =
        attempts.iter().filter(|a| a["returned"] == true).map(|a| a["method"].as_str().unwrap()).collect();
    assert_eq!(returned, ["hybrid"], "{r}");
    assert_eq!(r["verdict"], "pass");
    assert!(r["analytic"]["faces"].as_u64().unwrap() >= 2, "{r}");
    // the losers left nothing: one more body than before, the result's
    assert_eq!(s.doc.bodies.len(), bodies_before + 1, "a losing attempt left a body");
    assert_eq!(s.doc.bodies.iter().filter(|b| b.name == "auto").count(), 1);
}

fn open_scan(mesh: Mesh) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    s
}

/// A hybrid body that cannot be made fails with an issue code, no body and nothing added to the
/// document but what the segmentation made (the regions stay, for manual work, as with any method).
#[test]
fn a_hybrid_body_that_cannot_be_made_fails_cleanly_and_leaves_nothing() {
    let plate = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 60.0, 40.0, 60, 40);
    let wave = synth::wave(Point3::origin(), 60.0, 40.0, 4.0, 60, 40);
    for (label, mesh) in [("an open plate", plate), ("an open wave", wave)] {
        let mut s = open_scan(mesh);
        let before = (s.doc.bodies.len(), s.doc.primitives.len(), s.doc.meshes.len());
        let r = to_solid(&mut s, "hybrid");
        assert_eq!(r["verdict"], "fail", "{label}: {r}");
        assert_eq!(r["method"], "hybrid");
        assert!(r["body"].is_null() && r["analytic"].is_null(), "{label}");
        let code = r["issues"][0]["code"].as_str().unwrap();
        assert!(code.starts_with("hybrid_"), "{label}: {r}");
        assert_eq!(before, (s.doc.bodies.len(), s.doc.primitives.len(), s.doc.meshes.len()), "{label}");
        assert!(r["next_steps"].as_array().unwrap().iter().any(|n| n["params"]["method"] == "freeform"));
    }
}
