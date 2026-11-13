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

/// FNV-1a over the bits of `values`.
fn fnv(values: impl IntoIterator<Item = f64>) -> u64 {
    values.into_iter().fold(0xcbf2_9ce4_8422_2325, |h, v| (h ^ v.to_bits()).wrapping_mul(0x0000_0100_0000_01b3))
}

/// What an import is, orientation included: sorted hashes of the surface of every face, of the
/// curve of every edge and of every vertex; the number of edges used once, twice and more often;
/// the sorted (face, edge, direction) incidences, where the direction joins the sense of the face
/// and the direction of the loop edge; the sorted face senses; the number of loops.
#[derive(Debug, PartialEq)]
struct Signature {
    faces: Vec<u64>,
    edges: Vec<u64>,
    vertices: Vec<u64>,
    uses: [usize; 3],
    incidences: Vec<(u64, u64, bool)>,
    senses: Vec<(u64, bool)>,
    loops: usize,
}

fn signature(r: &surfacecraft_kernel::record::brep::Brep) -> Signature {
    use surfacecraft_kernel::record::brep::{Curve, Loop, Shell, Surface};
    let face_hash: Vec<u64> = r
        .faces
        .iter()
        .map(|f| match &f.surface {
            Surface::Spline { points, .. } => fnv(points.iter().flatten().flatten().copied()),
            _ => 0,
        })
        .collect();
    let edge_hash: Vec<u64> = r
        .edges
        .iter()
        .map(|e| match &e.curve {
            Curve::Spline { points, .. } => fnv(points.iter().flatten().copied()),
            _ => 0,
        })
        .collect();
    let (mut incidences, mut senses, mut loops) = (Vec::new(), Vec::new(), 0);
    for shell in &r.shells {
        let Shell::Faces { faces, .. } = shell else { continue };
        for of in faces {
            let fh = face_hash.get(of.face as usize).copied().unwrap_or(0);
            senses.push((fh, of.forward));
            for lp in r.faces.get(of.face as usize).map(|f| f.loops.as_slice()).unwrap_or_default() {
                loops += 1;
                if let Loop::Edges { edges } = lp {
                    for oe in edges {
                        let eh = edge_hash.get(oe.edge as usize).copied().unwrap_or(0);
                        incidences.push((fh, eh, oe.forward == of.forward));
                    }
                }
            }
        }
    }
    let mut uses = [0usize; 3];
    for e in &r.edges {
        uses[e.uses.len().clamp(1, 3) - 1] += 1;
    }
    let mut faces = face_hash;
    let mut edges = edge_hash;
    let mut vertices: Vec<u64> = r.vertices.iter().map(|v| fnv(v.point)).collect();
    faces.sort_unstable();
    edges.sort_unstable();
    vertices.sort_unstable();
    incidences.sort_unstable();
    senses.sort_unstable();
    Signature { faces, edges, vertices, uses, incidences, senses, loops }
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
    assert!(!surfacecraft_kernel::nurbs::is_closed(a));
    // the same display area, and the same file when each is written out again
    let area = |r: &surfacecraft_kernel::record::brep::Brep| tess::tessellate(r, Some(0.05)).unwrap().area();
    assert!((area(a) - area(b)).abs() <= 1e-9 * area(b), "{} vs {}", area(a), area(b));
    // the same entities, in the order of the file instead of the stitcher's: the multisets of the
    // face, edge and vertex geometry (bit for bit) and of how many faces use an edge
    assert_eq!(signature(a), signature(b));
}

/// Writes the STEP text of an open sheet of about `SURFACECRAFT_PERF_PATCHES` patches to
/// `SURFACECRAFT_PERF_DUMP` (for measuring the importer's memory in its own process).
#[test]
#[ignore = "writes a file"]
fn dump_sheet_step() {
    let n: usize = std::env::var("SURFACECRAFT_PERF_PATCHES").ok().and_then(|v| v.parse().ok()).unwrap_or(16_000);
    let (data, _) = sheet_step(n);
    std::fs::write(std::env::var("SURFACECRAFT_PERF_DUMP").unwrap(), decode(&data)).unwrap();
}

/// Edges both of whose uses run the same way: a sheet that is not consistently oriented.
fn same_direction_edges(sig: &Signature) -> usize {
    let mut by_edge: std::collections::BTreeMap<u64, Vec<bool>> = Default::default();
    for (_, e, d) in &sig.incidences {
        by_edge.entry(*e).or_default().push(*d);
    }
    by_edge.values().filter(|d| d.len() == 2 && d[0] == d[1]).count()
}

/// The own sheet's text with one change; the change must hit (the text differs).
fn mutated(text: &str, from: &str, to: &str) -> Vec<u8> {
    assert!(text.contains(from), "nothing to change: {from}");
    text.replacen(from, to, 1).into_bytes()
}

/// A file the fast path may try (the mark, 300 faces or more) with a defect: the answer must be
/// `Ok` or `Err` without a panic, and the same sheet as the stitched import gives (or an error
/// from both).
fn assert_agrees_with_stitched(file: &[u8], what: &str) {
    let fast = surfacecraft_kernel::step::import(file);
    let slow = surfacecraft_kernel::step::import_stitched(file);
    match (fast, slow) {
        (Ok(a), Ok(b)) => {
            assert_eq!(a.len(), b.len(), "{what}");
            for (x, y) in a.iter().zip(&b) {
                assert_eq!(x.faces.len(), y.faces.len(), "{what}: face count");
                assert_eq!(x.edges.len(), y.edges.len(), "{what}: edge count");
                let (sx, sy) = (signature(x), signature(y));
                assert_eq!(sx.uses, sy.uses, "{what}: edge-use histogram");
                assert_eq!(same_direction_edges(&sx), same_direction_edges(&sy), "{what}: orientation of shared edges");
                assert_eq!(sx, sy, "{what}: the sheet");
            }
        }
        (Err(_), Err(_)) => {}
        (a, b) => panic!("{what}: fast {:?}, stitched {:?}", a.map(|v| v.len()), b.map(|v| v.len())),
    }
}

#[test]
fn defective_own_sheets_of_300_faces_import_as_the_stitched_import_does() {
    let (data, faces) = sheet_step(400);
    assert!(faces >= 300);
    let text = String::from_utf8(decode(&data)).unwrap();
    // a face of reversed sense
    let face_line = text.lines().find(|l| l.contains("ADVANCED_FACE(")).unwrap().to_string();
    assert_agrees_with_stitched(
        &mutated(&text, &face_line, &face_line.replace(".T.);", ".F.);")),
        "face of reversed sense",
    );
    // a face without its loop
    let bound = face_line.split("(#").nth(1).and_then(|r| r.split(')').next()).unwrap().to_string();
    assert_agrees_with_stitched(
        &mutated(&text, &face_line, &face_line.replace(&format!("(#{bound})"), "()")),
        "face without a loop",
    );
    // an edge used by three faces: the first loop edge points at the edge of the second one
    let oriented: Vec<&str> = text.lines().filter(|l| l.contains("ORIENTED_EDGE(")).collect();
    let edge_of = |l: &str| l.split(",*,*,").nth(1).and_then(|r| r.split(',').next()).unwrap().to_string();
    let (e0, e1) = (edge_of(oriented[0]), edge_of(oriented[7]));
    assert_ne!(e0, e1);
    assert_agrees_with_stitched(
        &mutated(&text, oriented[0], &oriented[0].replace(&format!(",*,*,{e0},"), &format!(",*,*,{e1},"))),
        "edge of three faces",
    );
    // a dangling vertex reference
    let curve = text.lines().find(|l| l.contains("EDGE_CURVE(")).unwrap().to_string();
    let v = curve.split(',').nth(1).unwrap().to_string();
    assert_agrees_with_stitched(&mutated(&text, &curve, &curve.replacen(&v, "#99999999", 1)), "dangling vertex");
    // the mark in the description of a file that names another writer
    let foreign = text.replacen("'SurfaceCraft','SurfaceCraft'", "'Other','Other'", 1).replacen(
        "FILE_DESCRIPTION((''),",
        "FILE_DESCRIPTION(('SurfaceCraft','SurfaceCraft'),",
        1,
    );
    assert!(foreign.contains("FILE_DESCRIPTION(('SurfaceCraft','SurfaceCraft')"));
    assert_agrees_with_stitched(foreign.as_bytes(), "mark in a description");
}

#[test]
fn a_marked_file_over_the_entity_limit_is_refused_not_parsed() {
    let (data, _) = sheet_step(400);
    let mut text = String::from_utf8(decode(&data)).unwrap();
    // pad the data section with 4.1 million tiny entities
    let at = text.find("ENDSEC;\nEND-ISO").unwrap();
    let pad: String = (1_000_000..5_100_000).map(|i| format!("#{i}=B();\n")).collect();
    text.insert_str(at, &pad);
    assert!(surfacecraft_kernel::step::import(text.as_bytes()).is_err());
}
