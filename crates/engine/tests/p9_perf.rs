//! P9 performance of the large-network path. `body.compare` (the verdict step of `scan.to_solid`)
//! of a noisy scan against a closed network of thousands of bicubic patches: tests that the
//! faster implementation answers exactly as the straightforward one did and does not depend on the
//! thread count; ignored perf tests that time the stages with the growth exponent in the number
//! of patches and in the number of scan triangles (minutes: run with `--profile ci` or
//! `--release`),
//! `cargo test --profile ci -p surfacecraft-engine --test p9_perf -- --ignored --nocapture`.
//! `SURFACECRAFT_PERF_PATCHES=1000,10000` and `SURFACECRAFT_PERF_TRIS=100000,1000000` pick the
//! sizes (patches × triangles), `SURFACECRAFT_PERF_JITTER` (mm) roughens the patches.
use serde_json::json;
use std::collections::BTreeMap;
use std::time::Instant;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Vec3, synth};
use surfacecraft_kernel::nurbs::sheet_from_bezier_network_as_is;
use surfacecraft_kernel::{solid, tess};

const RADIUS: f64 = 50.0;
const AMPLITUDE: f64 = 0.3;
const RIPPLE_PERIOD: f64 = 6.0;
const NOISE: f64 = 0.02;
/// Default tolerance of `body.compare`, and the chord it derives from it.
const TOLERANCE: f64 = 0.1;
const CHORD: f64 = 0.005;
const REACH: f64 = 5.0;

/// The radius of the rippled sphere in the direction `u` (unit).
fn radius_at(u: Vec3) -> f64 {
    let k = std::f64::consts::TAU * RADIUS / RIPPLE_PERIOD;
    RADIUS + AMPLITUDE * (k * u.x + 0.3).sin() * (k * u.y + 0.7).sin() * (0.5 * k * u.z).cos()
}

/// Radial noise (mm) of every control point: patches fitted to a noisy scan follow it
/// (`SURFACECRAFT_PERF_JITTER`, default none).
fn jitter() -> f64 {
    std::env::var("SURFACECRAFT_PERF_JITTER").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0)
}

/// A deterministic value in [-1, 1] of a lattice point (a hash, so neighbouring patches agree).
fn lattice_noise(l: [usize; 3]) -> f64 {
    let mut h = 0x9E37_79B9_7F4A_7C15_u64;
    for a in l {
        h = (h ^ a as u64).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        h ^= h >> 31;
    }
    (h >> 11) as f64 / (1u64 << 52) as f64 - 1.0
}

/// A closed cube-sphere network of at least `count` patches on the rippled sphere. Boundary control
/// points come from the same lattice point, so neighbouring patches agree exactly.
fn network(count: usize) -> (Vec<Vec<Vec<[f64; 3]>>>, Vec<[u32; 4]>) {
    let n = (1..).find(|n| 6 * n * n >= count).unwrap_or(1);
    let steps = 3 * n;
    let point = |lat: [usize; 3]| -> [f64; 3] {
        let c = lat.map(|a| 2.0 * a as f64 / steps as f64 - 1.0);
        let len = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        let u = Vec3::new(c[0] / len, c[1] / len, c[2] / len);
        let r = radius_at(u) + jitter() * lattice_noise(lat);
        [u.x * r, u.y * r, u.z * r]
    };
    let mut ids: BTreeMap<[usize; 3], u32> = BTreeMap::new();
    let mut patches = Vec::with_capacity(6 * n * n);
    let mut corners = Vec::with_capacity(6 * n * n);
    for axis in 0..3 {
        for side in 0..2 {
            let (p, q) = ((axis + 1) % 3, (axis + 2) % 3);
            let (au, av) = if side == 1 { (p, q) } else { (q, p) };
            for i in 0..n {
                for j in 0..n {
                    let lattice = |m: usize, k: usize| -> [usize; 3] {
                        let mut l = [0; 3];
                        l[axis] = side * steps;
                        l[au] = 3 * i + m;
                        l[av] = 3 * j + k;
                        l
                    };
                    let net = (0..4).map(|m| (0..4).map(|k| point(lattice(m, k))).collect()).collect();
                    let mut id = |l: [usize; 3]| {
                        let next = ids.len() as u32;
                        *ids.entry(l).or_insert(next)
                    };
                    corners.push([id(lattice(0, 0)), id(lattice(3, 0)), id(lattice(3, 3)), id(lattice(0, 3))]);
                    patches.push(net);
                }
            }
        }
    }
    (patches, corners)
}

/// A noisy UV-sphere scan of about `tris` triangles on the same rippled sphere.
fn scan(tris: usize) -> Mesh {
    let nv = ((tris as f64 / 4.0).sqrt() as usize).max(4);
    let mut m = synth::sphere(Point3::origin(), 1.0, 2 * nv, nv);
    let mut rng = Rng::new(7);
    for p in &mut m.positions {
        let u = p.coords;
        p.coords = u * (radius_at(u) + NOISE * rng.gauss());
    }
    m
}

/// The record of the network of about `count` patches. With `SURFACECRAFT_PERF_CACHE` (a
/// directory) it is read from there, or built and written there: the build of a large network
/// leaves a peak of memory behind, which would hide that of the command measured after it.
fn body_record(count: usize) -> surfacecraft_kernel::record::brep::Brep {
    let path = std::env::var("SURFACECRAFT_PERF_CACHE")
        .ok()
        .map(|d| std::path::Path::new(&d).join(format!("net-{count}-{}.json", jitter())));
    if let Some(bytes) = path.as_ref().and_then(|p| std::fs::read(p).ok()) {
        return serde_json::from_slice(&bytes).unwrap();
    }
    let (nets, corners) = network(count);
    let record = sheet_from_bezier_network_as_is(&nets, &corners).unwrap();
    if let Some(p) = path {
        std::fs::write(p, serde_json::to_vec(&record).unwrap()).unwrap();
    }
    record
}

/// A session with the rippled network of about `patches` patches and a noisy scan of about `tris`
/// triangles of it.
fn session(patches: usize, tris: usize) -> Session {
    let mut s = Session::new();
    s.doc.add_body("body", body_record(patches));
    s.doc.add_mesh("scan", scan(tris));
    s
}

fn compare(s: &mut Session) -> serde_json::Value {
    s.execute("body.compare", json!({"body": "body", "scan": "scan", "tolerance": TOLERANCE})).unwrap()
}

/// `body.compare` as it was before the tessellation took the BVH over: the tessellation copied
/// into the tree, the samples, the per-face rows and the statistics from the public pieces.
fn reference_compare(s: &Session) -> (serde_json::Value, serde_json::Value) {
    use surfacecraft_deviation::faces::{per_face, worst_first};
    use surfacecraft_deviation::{samples_mesh_tagged, stats_of};
    let body = s.doc.bodies.iter().find(|b| b.name == "body").unwrap();
    let scan = s.doc.meshes.iter().find(|m| m.name == "scan").unwrap();
    let topo = tess::tessellate_topo(&body.brep, Some(CHORD)).unwrap();
    let bvh = Bvh::build(&topo.mesh);
    let normals = scan.mesh.vertex_normals();
    let tagged: Vec<_> = samples_mesh_tagged(&scan.mesh.positions, &normals, &bvh, REACH)
        .into_iter()
        .map(|(sample, tri)| (sample, tri.and_then(|t| topo.tri_face.get(t)).map(|&f| f as usize)))
        .collect();
    let samples: Vec<_> = tagged.iter().map(|(x, _)| *x).collect();
    let mut rows = per_face(&tagged, solid::counts(&body.brep).faces, TOLERANCE);
    worst_first(&mut rows);
    (json!(stats_of(&samples, TOLERANCE)), json!(rows))
}

/// `SURFACECRAFT_PERF_DIRECT=0` skips the separate tessellation measurement (its mesh would stay
/// in memory next to the one `body.compare` makes: for reading the peak memory of the command).
fn direct() -> bool {
    std::env::var("SURFACECRAFT_PERF_DIRECT").is_ok_and(|v| v != "0")
        || std::env::var("SURFACECRAFT_PERF_DIRECT").is_err()
}

fn env_list(name: &str, default: &[usize]) -> Vec<usize> {
    std::env::var(name)
        .ok()
        .map(|v| v.split(',').filter_map(|x| x.trim().parse().ok()).collect())
        .unwrap_or_else(|| default.to_vec())
}

fn timed<T>(f: impl FnOnce() -> T) -> (T, f64) {
    let t = Instant::now();
    let r = f();
    (r, t.elapsed().as_secs_f64())
}

/// Slope of ln(time) against ln(size), least squares.
fn exponent(points: &[(f64, f64)]) -> f64 {
    let pts: Vec<(f64, f64)> = points.iter().filter(|p| p.1 > 0.0).map(|&(n, t)| (n.ln(), t.ln())).collect();
    let k = pts.len() as f64;
    let (sx, sy) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0, a.1 + p.1));
    let (sxx, sxy) = pts.iter().fold((0.0, 0.0), |a, p| (a.0 + p.0 * p.0, a.1 + p.0 * p.1));
    (k * sxy - sx * sy) / (k * sxx - sx * sx)
}

#[test]
#[ignore = "minutes of CPU and GB of RAM; run explicitly with --profile ci or --release"]
fn perf_body_compare_stages() {
    let patch_sizes = env_list("SURFACECRAFT_PERF_PATCHES", &[1_000, 10_000, 60_000]);
    let tri_sizes = env_list("SURFACECRAFT_PERF_TRIS", &[100_000, 1_000_000]);
    let mut by_patches: BTreeMap<usize, Vec<(f64, f64)>> = BTreeMap::new();
    let mut by_tris: BTreeMap<usize, Vec<(f64, f64)>> = BTreeMap::new();
    for &count in &patch_sizes {
        let brep = body_record(count);
        let faces = solid::counts(&brep).faces;
        let t_tess = if direct() {
            let (topo, t) = timed(|| tess::tessellate_topo(&brep, Some(CHORD)).unwrap());
            eprintln!("PERF patches {faces}: tessellate {t:.2} s ({} tris)", topo.mesh.triangle_count());
            t
        } else {
            0.0
        };
        for &tris in &tri_sizes {
            let m = scan(tris);
            let n = m.triangle_count();
            let mut s = Session::new();
            s.doc.add_body("body", brep.clone());
            s.doc.add_mesh("scan", m);
            let (r, t_total) = timed(|| compare(&mut s));
            eprintln!(
                "PERF patches {faces} x scan {n} tris: body.compare total {t_total:.2} s (tessellation alone {t_tess:.2} s), \
                 {} samples, within {:.6}, rms {:.6}",
                r["totals"]["count"],
                r["totals"]["within"].as_f64().unwrap_or(f64::NAN),
                r["totals"]["rms"].as_f64().unwrap_or(f64::NAN)
            );
            by_patches.entry(tris).or_default().push((faces as f64, t_total));
            by_tris.entry(faces).or_default().push((n as f64, t_total));
        }
    }
    for (tris, pts) in &by_patches {
        eprintln!("PERF exponent body.compare in patches (scan ~{tris} tris): {:.2}", exponent(pts));
    }
    for (faces, pts) in &by_tris {
        eprintln!("PERF exponent body.compare in scan tris ({faces} patches): {:.2}", exponent(pts));
    }
}

/// STEP export and import of an open sheet network and of a closed solid network through the
/// commands (`export.step`, `import.step`), by size. `SURFACECRAFT_PERF_STEP_IMPORT_MAX` is the
/// largest network whose file is read back (the import is the slowest stage).
#[test]
#[ignore = "minutes of CPU; run explicitly with --profile ci or --release"]
fn perf_step_stages() {
    let import_max: usize =
        std::env::var("SURFACECRAFT_PERF_STEP_IMPORT_MAX").ok().and_then(|v| v.parse().ok()).unwrap_or(usize::MAX);
    for count in env_list("SURFACECRAFT_PERF_PATCHES", &[1_000, 4_000, 16_000]) {
        let (patches, corners) = network(count);
        let solid_rec = sheet_from_bezier_network_as_is(&patches, &corners).unwrap();
        let sheet_rec = sheet_from_bezier_network_as_is(&patches[1..], &corners[1..]).unwrap();
        let faces = solid_rec.faces.len();
        for (kind, rec) in [("sheet", sheet_rec), ("solid", solid_rec)] {
            let mut s = Session::new();
            s.doc.add_body("b", rec);
            let (out, t_export) = timed(|| s.execute("export.step", json!({"objects": ["b"]})).unwrap());
            let bytes = out["bytes"].as_u64().unwrap_or(0);
            let data = out["data"].as_str().unwrap_or_default().to_string();
            let t_import = if faces <= import_max {
                timed(|| s.execute("import.step", json!({"data": data, "name": "back"})).unwrap()).1
            } else {
                0.0
            };
            eprintln!(
                "PERF step {kind} patches {faces}: export {t_export:.2} s ({:.1} KB/patch), import {t_import:.2} s",
                bytes as f64 / 1e3 / faces as f64
            );
        }
    }
}

const MID_PATCHES: usize = 1_500;
const MID_TRIS: usize = 60_000;

#[test]
fn body_compare_equals_the_copying_reference_on_a_mid_size_network() {
    let mut s = session(MID_PATCHES, MID_TRIS);
    let got = compare(&mut s);
    let (totals, rows) = reference_compare(&s);
    assert_eq!(got["totals"], totals);
    assert_eq!(got["per_face"], rows);
    assert!(got["totals"]["count"].as_u64().unwrap_or(0) > 10_000, "{}", got["totals"]);
}

/// Runs `f` on a pool of `threads` threads.
fn on<T: Send>(threads: usize, f: impl FnOnce() -> T + Send) -> T {
    rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap().install(f)
}

#[test]
fn body_compare_is_identical_on_one_and_eight_threads_and_between_runs() {
    let mut s = session(MID_PATCHES, MID_TRIS);
    let one = on(1, || compare(&mut s));
    let eight = on(8, || compare(&mut s));
    let again = on(8, || compare(&mut s));
    assert_eq!(one, eight);
    assert_eq!(eight, again);
}

/// The STEP text of the open sheet (the network without its first patch) of about `patches` patches.
fn sheet_step(patches: usize) -> (String, usize) {
    let (nets, corners) = network(patches);
    let sheet = sheet_from_bezier_network_as_is(&nets[1..], &corners[1..]).unwrap();
    let faces = sheet.faces.len();
    let mut s = Session::new();
    s.doc.add_body("b", sheet);
    let out = s.execute("export.step", json!({"objects": ["b"]})).unwrap();
    (out["data"].as_str().unwrap_or_default().to_string(), faces)
}

fn decode(data: &str) -> Vec<u8> {
    surfacecraft_doc::base64::decode(data).unwrap()
}

#[test]
fn an_own_open_sheet_imports_as_the_stitched_import_does() {
    let (data, faces) = sheet_step(900);
    assert!(faces >= 300, "the topology build is used from 300 faces");
    let bytes = decode(&data);
    let fast = surfacecraft_kernel::step::import(&bytes).unwrap();
    let slow = surfacecraft_kernel::step::import_stitched(&bytes).unwrap();
    assert_eq!(fast.len(), 1);
    assert_eq!(slow.len(), 1);
    let (a, b) = (&fast[0], &slow[0]);
    assert_eq!((a.faces.len(), a.edges.len(), a.vertices.len()), (b.faces.len(), b.edges.len(), b.vertices.len()));
    assert_eq!(a.faces.len(), faces);
    assert_eq!(a.shells.len(), b.shells.len());
    assert!(!surfacecraft_kernel::nurbs::is_closed(a));
    // the same geometry: the same display area
    let area = |r: &surfacecraft_kernel::record::brep::Brep| tess::tessellate(r, Some(0.05)).unwrap().area();
    assert!((area(a) - area(b)).abs() <= 1e-9 * area(b), "{} vs {}", area(a), area(b));
}
