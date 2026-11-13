//! Shared synthetic scans and checks of the `align.pair_coarse` tests.
#![allow(dead_code)]
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Transform, Vec3, synth};

pub const SIGMA: f64 = 0.05;
pub const MAX_ANGLE_DEG: f64 = 0.1;
pub const MAX_DISPLACEMENT_MM: f64 = 0.1;

/// The bracket at a given tessellation: block resolution, cylinder segments, sphere rings.
pub fn bracket_at(block: usize, segments: usize, rings: usize) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 20.0), block);
    m.append(&synth::cylinder(Point3::new(45.0, 12.0, 20.0), Vec3::z(), 6.0, 10.0, segments, 6));
    m.append(&synth::sphere(Point3::new(15.0, 28.0, 20.0), 5.0, 2 * rings, rings));
    m
}

/// The organic blob (a lumpy ellipsoid) at a given resolution.
pub fn blob_at(nu: usize) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 20.0, nu, nu / 2);
    for p in &mut m.positions {
        let lump = 1.0 + 0.12 * (0.21 * p.x).sin() * (0.17 * p.y).cos() + 0.08 * (0.3 * p.z).sin();
        *p = Point3::new(p.x * 1.6 * lump, p.y * lump, p.z * 0.7 * lump);
    }
    m
}

pub fn noisy(mut m: Mesh, seed: u64) -> Mesh {
    synth::add_noise(&mut m, SIGMA, &mut Rng::new(seed));
    m
}

/// The two tessellations of a part: (fixed scan, moving scan before its motion), with the noise of
/// trial `k` (independent for every trial).
pub fn pair_of_trial(part: &str, k: u64) -> (Mesh, Mesh) {
    match part {
        "bracket" => (noisy(bracket_at(12, 48, 16), 1 + 10 * k), noisy(bracket_at(17, 64, 20), 2 + 10 * k)),
        _ => (noisy(blob_at(48), 3 + 10 * k), noisy(blob_at(62), 4 + 10 * k)),
    }
}

pub fn pair_of(part: &str) -> (Mesh, Mesh) {
    pair_of_trial(part, 0)
}

/// A random rotation up to 180 degrees with a translation up to twice the part size (75 mm).
pub fn random_motion(seed: u64) -> Transform {
    let mut rng = Rng::new(1000 + seed);
    let angle = rng.range(0.2, 180.0_f64.to_radians());
    Transform::rotation_axis_angle(&rng.unit_vector(), angle)
        .then_after(&Transform::translation(rng.unit_vector() * rng.range(20.0, 150.0)))
}

pub fn matrix_of(v: &Value) -> Transform {
    Transform::from_rows(&serde_json::from_value(v["matrix"].clone()).unwrap()).unwrap()
}

/// Session with the fixed scan `fixed` and the moving scan `moving` put at `pose`.
pub fn session(fixed: &Mesh, moving: &Mesh, pose: &Transform) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh("fixed", fixed.clone());
    s.doc.add_mesh("moving", moving.transformed(pose));
    s
}

pub fn pair(s: &mut Session, extra: Value) -> Value {
    let mut p = json!({"object": "moving", "target": "fixed"});
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("align.pair_coarse", p).unwrap()
}

/// (angle in degrees, largest displacement in mm) of `found` against the true motion `pose⁻¹`.
pub fn pose_error(found: &Transform, pose: &Transform, moving: &Mesh) -> (f64, f64) {
    let net = found.then_after(pose); // identity when `found` undoes `pose`
    let on_scan = moving.transformed(pose);
    let q = net.quaternion();
    let angle = 2.0 * q.vector().norm().atan2(q.scalar().abs());
    (angle.to_degrees(), found.max_displacement(&pose.inverse(), &on_scan.positions))
}

/// The triangles of `m` whose centroid satisfies `keep`.
pub fn cut(m: &Mesh, keep: impl Fn(&Point3) -> bool) -> Mesh {
    let ids: Vec<u32> =
        (0..m.triangle_count()).filter(|&i| m.face_centroid(i).is_some_and(|c| keep(&c))).map(|i| i as u32).collect();
    m.submesh(&ids)
}

/// Ground truth of the overlap: the share of the area of `of` (placed at the true pose) that lies
/// within `tau` of `on`, by dense area-uniform samples and the BVH at the *true* motion. It does
/// not use the method under test.
pub fn true_share(of: &Mesh, on: &Mesh, tau: f64) -> f64 {
    let bvh = Bvh::build(on);
    let pts = of.sample_points(20_000, &mut Rng::new(77));
    let hit = pts.iter().filter(|(p, _)| bvh.closest(p, tau).is_some()).count();
    hit as f64 / pts.len() as f64
}

/// Does `net` (a motion of the part onto itself) map the surface of `part` onto its own surface?
pub fn symmetry_error(net: &Transform, part: &Mesh) -> f64 {
    let bvh = Bvh::build(part);
    part.positions
        .iter()
        .map(|p| bvh.closest(&net.apply(p), f64::INFINITY).map_or(f64::INFINITY, |h| h.distance))
        .fold(0.0, f64::max)
}

pub fn symmetric_pair(shape: &str) -> (Mesh, Mesh) {
    match shape {
        "box" => (
            noisy(synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 8), 5),
            noisy(synth::cuboid(Point3::origin(), Vec3::new(40.0, 30.0, 20.0), 11), 6),
        ),
        "cube" => (
            noisy(synth::cuboid(Point3::origin(), Vec3::new(30.0, 30.0, 30.0), 8), 5),
            noisy(synth::cuboid(Point3::origin(), Vec3::new(30.0, 30.0, 30.0), 11), 6),
        ),
        _ => (
            noisy(synth::cylinder(Point3::origin(), Vec3::z(), 10.0, 30.0, 48, 10), 5),
            noisy(synth::cylinder(Point3::origin(), Vec3::z(), 10.0, 30.0, 64, 14), 6),
        ),
    }
}

pub fn pair_err(s: &mut Session, extra: Value) -> String {
    let mut p = json!({"object": "moving", "target": "fixed"});
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("align.pair_coarse", p).unwrap_err().to_string()
}

/// A 60 x 40 x 4 plate (symmetric under half turns) with one off-centre bump of about 5 % of its
/// area: a half sphere of radius 7 at (15, 12).
pub fn plate_at(block: usize, rings: usize) -> Mesh {
    let mut m = synth::cuboid(Point3::origin(), Vec3::new(60.0, 40.0, 4.0), block);
    m.append(&synth::sphere(Point3::new(15.0, 12.0, 4.0), 7.0, 2 * rings, rings));
    m
}

/// Stability of the result, 0 when absent.
pub fn stab(out: &Value) -> f64 {
    out["stability"].as_f64().unwrap()
}

pub fn fine(m: &Mesh, seed: u64) -> Mesh {
    noisy(synth::refine(m, |_| 0.35), seed)
}

/// Runs a patch case; returns (verdict, stability).
pub fn patch_case(fixed: &Mesh, patch: &Mesh, seed: u64) -> (String, f64) {
    let mut s = session(fixed, patch, &random_motion(seed));
    let out = pair(&mut s, json!({}));
    (out["verdict"].as_str().unwrap().to_string(), stab(&out))
}
