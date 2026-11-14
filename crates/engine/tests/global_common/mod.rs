//! Shared fixtures of the `align.global` tests: the views of the bracket, the blob and the lumpy ring.
#![allow(dead_code)]
use crate::pair_common::*;
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Transform, Vec3, synth};

pub const BOUND_DEGREES: f64 = 0.5;
pub const BOUND_MM: f64 = 0.5;

/// A view of the bracket: its own tessellation and noise, the triangles whose centroid lies on the
/// side of `dir` of the part centre or at most `reach` mm beyond it.
pub fn bracket_view(k: usize, dir: Vec3, reach: f64) -> Mesh {
    let whole = noisy(bracket_at(10 + 2 * k, 40 + 6 * k, 14 + k), 100 + k as u64);
    let c = Point3::new(30.0, 20.0, 15.0);
    cut(&whole, |p| (p - c).dot(&dir) >= -reach)
}

pub const SIDES: [([f64; 3], f64); 6] = [
    ([1.0, 0.0, 0.0], 6.0),
    ([-1.0, 0.0, 0.0], 6.0),
    ([0.0, 1.0, 0.0], 4.0),
    ([0.0, -1.0, 0.0], 4.0),
    ([0.0, 0.0, 1.0], 6.0),
    ([0.577_35, 0.577_35, 0.577_35], 7.0),
];

pub fn bracket_views(n: usize) -> Vec<Mesh> {
    SIDES.iter().take(n).enumerate().map(|(k, (d, r))| bracket_view(k, Vec3::new(d[0], d[1], d[2]), *r)).collect()
}

/// Six views of the blob, as the bracket's: the triangles on the side of a direction of the centre
/// or at most `reach` mm beyond it.
pub fn blob_views() -> Vec<Mesh> {
    let sides: [([f64; 3], f64); 6] = [
        ([1.0, 0.0, 0.0], 8.0),
        ([-1.0, 0.0, 0.0], 8.0),
        ([0.0, 1.0, 0.0], 5.0),
        ([0.0, -1.0, 0.0], 5.0),
        ([0.0, 0.0, 1.0], 3.5),
        ([0.0, 0.0, -1.0], 3.5),
    ];
    sides
        .iter()
        .enumerate()
        .map(|(k, (d, reach))| {
            let dir = Vec3::new(d[0], d[1], d[2]);
            cut(&noisy(blob_at(48 + 4 * k), 200 + k as u64), |p| p.coords.dot(&dir) >= -reach)
        })
        .collect()
}

/// A part with the rotational symmetry broken by twelve bumps of different sizes at irregular
/// azimuths: a ring (torus) with lumps, seen in arcs.
pub fn lumpy_ring(k: usize) -> Mesh {
    let mut m = synth::torus(Point3::origin(), Vec3::z(), 40.0, 9.0, 72 + 4 * k, 20 + k);
    let azimuths = [8.0, 31.0, 62.0, 80.0, 118.0, 139.0, 171.0, 203.0, 236.0, 262.0, 291.0, 330.0];
    for (i, a) in azimuths.iter().enumerate() {
        let (s, c) = f64::to_radians(*a).sin_cos();
        let radius = 3.5 + 0.9 * ((i * 7) % 5) as f64;
        let z = 1.8 * (((i * 5) % 7) as f64 - 3.0);
        let out = 44.0 + 0.7 * ((i * 3) % 4) as f64;
        m.append(&synth::sphere(Point3::new(out * c, out * s, z), radius, 20, 10));
    }
    noisy(m, 600 + k as u64)
}

/// A view of the lumpy ring: the arc of `half` degrees either side of azimuth `centre`.
pub fn ring_arc(k: usize, centre: f64, half: f64) -> Mesh {
    cut(&lumpy_ring(k), |p| {
        let d = (p.y.atan2(p.x).to_degrees() - centre + 540.0).rem_euclid(360.0) - 180.0;
        d.abs() <= half
    })
}

pub fn poses(n: usize) -> Vec<Transform> {
    (0..n as u64).map(random_motion).collect()
}

pub fn session_of(views: &[Mesh], poses: &[Transform]) -> Session {
    session_with_ids(views, poses).0
}

pub fn session_with_ids(views: &[Mesh], poses: &[Transform]) -> (Session, Vec<surfacecraft_doc::ObjectId>) {
    let mut s = Session::new();
    let ids = views
        .iter()
        .zip(poses)
        .enumerate()
        .map(|(k, (v, p))| s.doc.add_mesh(&format!("v{k}"), v.transformed(p)))
        .collect();
    (s, ids)
}

pub fn global(s: &mut Session, n: usize, extra: Value) -> Value {
    let names: Vec<String> = (0..n).map(|k| format!("v{k}")).collect();
    let mut p = json!({ "objects": names });
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("align.global", p).unwrap()
}

pub fn rows(v: &Value) -> Transform {
    Transform::from_rows(&serde_json::from_value(v.clone()).unwrap()).unwrap()
}

/// (angle in degrees, largest displacement in mm) of the motion found for view `k` against the
/// truth: the motion that carries it back into the frame of view 0.
pub fn error_of(found: &Transform, views: &[Mesh], poses: &[Transform], k: usize) -> (f64, f64) {
    let truth = poses[0].then_after(&poses[k].inverse());
    let on = views[k].transformed(&poses[k]);
    let q = truth.inverse().then_after(found).quaternion();
    let angle = 2.0 * q.vector().norm().atan2(q.scalar().abs());
    (angle.to_degrees(), found.max_displacement(&truth, &on.positions))
}

pub fn scan_error(out: &Value, views: &[Mesh], poses: &[Transform], k: usize, key: &str) -> (f64, f64) {
    error_of(&rows(&out["scans"][k][key]), views, poses, k)
}

pub fn count(out: &Value, status: &str) -> usize {
    out["edges"].as_array().unwrap().iter().filter(|e| e["status"] == status).count()
}

pub fn print_edges(out: &Value) {
    for e in out["edges"].as_array().unwrap() {
        eprintln!(
            "  {}-{}: {} {} confidence {:.2}, overlap {:.2}/{:.2}, rms {:.5} of the diagonal (strong {})",
            e["fixed"],
            e["moving"],
            e["verdict"],
            e["status"],
            e["confidence"].as_f64().unwrap(),
            e["overlap_moving"].as_f64().unwrap(),
            e["overlap_fixed"].as_f64().unwrap(),
            e["rms_per_diagonal"].as_f64().unwrap(),
            e["strong_evidence"]
        );
    }
}

pub fn loop_figures(out: &Value) -> (f64, f64, u64) {
    let l = &out["loop"];
    (l["before"]["worst_mm"].as_f64().unwrap(), l["after"]["worst_mm"].as_f64().unwrap(), l["cycles"].as_u64().unwrap())
}
