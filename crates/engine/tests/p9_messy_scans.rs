//! P9-O9 gates: messy scans are prepared before they are modelled. A file with several parts is split
//! (`scan.split`) and every part modelled on its own; a closed organic scan with topological noise
//! (tiny handles, non-manifold fins, self-intersecting slivers) is rebuilt (`mesh.remesh`) and then
//! modelled; thin walls are kept, open scans are not closed, and a scan that already works takes the
//! path it always took. Tolerances are those of design doc §11.1, unchanged.
use serde_json::{Value, json};
use std::f64::consts::{PI, TAU};
use surfacecraft_engine::Session;
use surfacecraft_geom::pieces::pieces;
use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

const TOLERANCE: f64 = 0.2;
const MIN_WITHIN: f64 = 0.99;

fn weld(mut m: Mesh) -> Mesh {
    m.weld(1e-9);
    m
}

fn flip(mut m: Mesh) -> Mesh {
    m.triangles.iter_mut().for_each(|t| t.swap(1, 2));
    m
}

/// A closed speck of debris: a tetrahedron of `size` mm.
fn speck(at: Point3, size: f64) -> Mesh {
    let p = |x: f64, y: f64, z: f64| at + Vec3::new(x, y, z) * size;
    Mesh {
        positions: vec![p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0), p(0.0, 0.0, 1.0)],
        triangles: vec![[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]],
    }
}

fn three_parts_and_debris() -> (Mesh, [Mesh; 3]) {
    let wavy = |c: Point3, r: f64, k: f64| {
        let mut m = weld(synth::sphere(Point3::origin(), 1.0, 120, 60));
        for p in &mut m.positions {
            let d = p.coords;
            p.coords = c.coords + d * r * (1.0 + 0.08 * (k * d.x).sin() * (k * 0.7 * d.y).cos());
        }
        synth::add_noise(&mut m, 0.02, &mut Rng::new(c.x as u64 + 3));
        m
    };
    let a = wavy(Point3::new(0.0, 0.0, 0.0), 14.0, 3.0);
    let b = wavy(Point3::new(45.0, 0.0, 0.0), 10.0, 4.0);
    let mut c = weld(synth::torus(Point3::origin(), Vec3::z(), 9.0, 3.5, 120, 60));
    c.positions.iter_mut().for_each(|p| *p += Vec3::new(0.0, 45.0, 0.0));
    synth::add_noise(&mut c, 0.02, &mut Rng::new(8));
    let mut all = Mesh::default();
    for m in [&a, &b, &c] {
        all.append(m);
    }
    let mut rng = Rng::new(9);
    for _ in 0..20 {
        all.append(&speck(
            Point3::new(rng.range(-20.0, 70.0), rng.range(-20.0, 20.0), rng.range(-10.0, 40.0))
                + Vec3::new(0.0, 80.0, 0.0),
            0.8,
        ));
    }
    (all, [a, b, c])
}

fn session_with(name: &str, m: Mesh) -> Session {
    let mut s = Session::new();
    s.doc.add_mesh(name, m);
    s
}

#[test]
fn gate_split_finds_three_parts_and_reports_the_debris() {
    let (all, parts) = three_parts_and_debris();
    let mut s = session_with("scan", all);
    let r = s.execute("scan.split", json!({"object": "scan", "keep_debris": true})).unwrap();
    eprintln!("split: {r}");
    let listed = r["parts"].as_array().unwrap();
    assert_eq!(listed.len(), 3, "{r}");
    let mut expected: Vec<(usize, f64)> = parts.iter().map(|m| (m.triangle_count(), m.area())).collect();
    expected.sort_by_key(|e| std::cmp::Reverse(e.0));
    for (k, p) in listed.iter().enumerate() {
        assert!((p["area"].as_f64().unwrap() - expected[k].1).abs() / expected[k].1 < 0.01, "part {k}: {p}");
        assert_eq!(p["triangles"].as_u64().unwrap() as usize, expected[k].0, "part {k}");
        assert!(p["closed"].as_bool().unwrap());
        assert!(p["nested_in"].is_null());
    }
    assert_eq!(r["debris"]["pieces"], 20, "{r}");
    assert_eq!(r["debris"]["triangles"], 80);
    assert!(r["debris"]["id"].as_u64().is_some());
    assert_eq!(r["distinct_parts"], 3);
    s.execute("edit.undo", json!({})).unwrap();
    assert!(s.doc.find_by_name("scan-part-1").is_none(), "one undo step removes the parts");
}

#[test]
fn gate_split_reports_a_hollow_part_as_nested_not_as_two_parts() {
    let outer = weld(synth::sphere(Point3::origin(), 15.0, 80, 40));
    let inner = flip(weld(synth::sphere(Point3::origin(), 9.0, 64, 32)));
    let mut hollow = outer.clone();
    hollow.append(&inner);
    let mut s = session_with("scan", hollow.clone());
    let r = s.execute("scan.split", json!({"object": "scan"})).unwrap();
    assert_eq!(r["parts"].as_array().unwrap().len(), 2, "{r}");
    assert_eq!(r["parts"][1]["nested_in"], 0, "{r}");
    assert_eq!(r["parts"][1]["relation"], "cavity", "{r}");
    assert_eq!(r["distinct_parts"], 1);
    // a ball inside a ball (not turned inside out) is a separate part inside another
    let mut included = outer;
    included.append(&weld(synth::sphere(Point3::origin(), 6.0, 48, 24)));
    let mut s2 = session_with("scan", included);
    let r2 = s2.execute("scan.split", json!({"object": "scan"})).unwrap();
    assert_eq!(r2["parts"][1]["relation"], "inclusion", "{r2}");
    // a hollow scan is one part for scan.to_solid: nothing is split, no preparation
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert!(out["components"].is_null() || out["components"].as_array().is_some_and(Vec::is_empty), "{out}");
    assert_eq!(out["preparation"], json!([]));
}

#[test]
fn gate_to_solid_models_every_part_and_components_largest_models_one() {
    let (all, _) = three_parts_and_debris();
    let mut s = session_with("scan", all.clone());
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    eprintln!("to_solid parts: verdict {} preparation {}", out["verdict"], out["preparation"]);
    let comps = out["components"].as_array().unwrap();
    assert_eq!(comps.len(), 3, "{out}");
    for c in comps {
        eprintln!(
            "  part {} {} verdict {} within {} issues {}",
            c["name"], c["method"], c["verdict"], c["deviation"]["within"], c["issues"]
        );
        assert_eq!(c["verdict"], "pass", "{c}");
        assert!(c["deviation"]["within"].as_f64().unwrap() >= MIN_WITHIN, "{c}");
        assert!(c["body"]["volume"].as_f64().unwrap() > 0.0);
    }
    assert_eq!(out["verdict"], "pass");
    assert_eq!(out["preparation"][0]["step"], "split");
    let mut s1 = session_with("scan", all.clone());
    let one =
        s1.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "components": "largest"})).unwrap();
    assert_eq!(one["components"].as_array().unwrap().len(), 1, "{one}");
    let mut s2 = session_with("scan", all);
    let two = s2.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "components": 2})).unwrap();
    assert_eq!(two["components"].as_array().unwrap().len(), 2);
    let bad = s2.execute("scan.to_solid", json!({"scan": "scan", "components": "none"}));
    assert!(bad.is_err());
}

// ---- topological noise -------------------------------------------------------------------------

fn blob_radius(d: &Vec3) -> f64 {
    20.0 * (1.0
        + 0.09 * (3.0 * d.x).sin() * (2.0 * d.y).cos()
        + 0.07 * (2.0 * d.z + 1.2 * d.x).sin() * (3.0 * d.y).cos()
        + 0.05 * (4.0 * d.z).cos() * d.x)
}

fn blob(nu: usize, nv: usize) -> Mesh {
    let mut m = weld(synth::sphere(Point3::origin(), 1.0, nu, nv));
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = d * blob_radius(&d);
    }
    m
}

fn blob_volume() -> f64 {
    let (nt, np) = (720, 360);
    let mut v = 0.0;
    for i in 0..nt {
        let th = TAU * (i as f64 + 0.5) / nt as f64;
        for j in 0..np {
            let ph = PI * (j as f64 + 0.5) / np as f64;
            let d = Vec3::new(ph.sin() * th.cos(), ph.sin() * th.sin(), ph.cos());
            v += blob_radius(&d).powi(3) * ph.sin();
        }
    }
    v * TAU / nt as f64 * PI / np as f64 / 3.0
}

/// The prism that replaces triangles `ta` and `tb`: a tiny handle (genus + 1) that adds no vertex, so
/// every vertex stays on the shape. The corners of `tb` are matched to those of `ta` in reversed
/// order (the two ends face away from each other), rotated to the shortest struts.
fn prism(m: &Mesh, ta: [u32; 3], tb: [u32; 3]) -> Vec<[u32; 3]> {
    let strut = |c: &[u32; 3]| -> f64 {
        (0..3).map(|i| (m.positions[ta[i] as usize] - m.positions[c[i] as usize]).norm()).sum()
    };
    let rots: Vec<[u32; 3]> = (0..3).map(|r| [tb[r % 3], tb[(r + 2) % 3], tb[(r + 1) % 3]]).collect();
    let c = *rots.iter().min_by(|x, y| strut(x).total_cmp(&strut(y))).unwrap_or(&tb);
    let mut out = Vec::new();
    for i in 0..3 {
        let j = (i + 1) % 3;
        out.push([ta[i], ta[j], c[j]]);
        out.push([ta[i], c[j], c[i]]);
    }
    out
}

/// Picks `count` pairs of triangles about `gap` mm apart that share no vertex with each other or
/// with another pair, and bridges each pair with a prism. Returns how many were made.
fn inject_handles(m: &mut Mesh, count: usize, gap: f64, rng: &mut Rng) -> usize {
    let centres: Vec<Point3> =
        (0..m.triangle_count()).map(|i| m.face_centroid(i).unwrap_or_else(Point3::origin)).collect();
    let mut taken = vec![false; m.positions.len()];
    let mut pairs: Vec<(usize, usize)> = Vec::new();
    let tree = surfacecraft_geom::KdTree::build(&centres);
    for _ in 0..count * 40 {
        if pairs.len() >= count {
            break;
        }
        let a = rng.index(m.triangles.len());
        let near = tree.within(&centres[a], 1.2 * gap);
        if near.is_empty() {
            continue;
        }
        let b = near[rng.index(near.len())];
        let d = (centres[a] - centres[b]).norm();
        let free = |t: [u32; 3]| t.iter().all(|v| !taken[*v as usize]);
        if a == b
            || d < 0.8 * gap
            || d > 1.2 * gap
            || !free(m.triangles[a])
            || !free(m.triangles[b])
            || m.triangles[a].iter().any(|v| m.triangles[b].contains(v))
        {
            continue;
        }
        for v in m.triangles[a].iter().chain(&m.triangles[b]) {
            taken[*v as usize] = true;
        }
        pairs.push((a, b));
    }
    let chosen: std::collections::HashSet<usize> = pairs.iter().flat_map(|&(a, b)| [a, b]).collect();
    let bridges: Vec<[u32; 3]> = pairs.iter().flat_map(|&(a, b)| prism(m, m.triangles[a], m.triangles[b])).collect();
    let mut keep: Vec<[u32; 3]> =
        m.triangles.iter().enumerate().filter(|(i, _)| !chosen.contains(i)).map(|(_, t)| *t).collect();
    keep.extend(bridges);
    m.triangles = keep;
    pairs.len()
}

/// Fins: a triangle on an existing edge, its tip 0.15 mm off the surface (an edge of three triangles).
fn inject_fins(m: &mut Mesh, count: usize, rng: &mut Rng) {
    let normals = m.vertex_normals();
    for _ in 0..count {
        let t = m.triangles[rng.index(m.triangles.len())];
        let (a, b) = (t[0] as usize, t[1] as usize);
        let tip = Point3::from(
            (m.positions[a].coords + m.positions[b].coords) / 2.0 + (normals[a] + normals[b]).normalize() * 0.15,
        );
        m.positions.push(tip);
        let n = (m.positions.len() - 1) as u32;
        m.triangles.push([t[1], t[0], n]);
    }
}

/// Slivers: small triangles that pass through the shell.
fn inject_slivers(m: &mut Mesh, count: usize, rng: &mut Rng) {
    let normals = m.vertex_normals();
    for _ in 0..count {
        let v = rng.index(m.positions.len());
        let (p, n) = (m.positions[v], normals[v]);
        let t = synth_tangent(&n);
        let base = m.positions.len() as u32;
        m.positions.extend([p + n * 0.15, p - n * 0.15 + t * 0.4, p - n * 0.15 - t * 0.4]);
        m.triangles.push([base, base + 1, base + 2]);
    }
}

fn synth_tangent(n: &Vec3) -> Vec3 {
    let a = if n.x.abs() < 0.9 { Vec3::x() } else { Vec3::y() };
    n.cross(&a).normalize()
}

fn noisy_blob(handles: usize, seed: u64) -> (Mesh, usize) {
    let mut m = blob(300, 150);
    synth::add_noise(&mut m, 0.03, &mut Rng::new(seed));
    let mut rng = Rng::new(seed + 1);
    let made = inject_handles(&mut m, handles, 1.5, &mut rng);
    if handles <= 200 {
        assert_eq!(pieces(&m)[0].genus(), Some(made), "every bridge adds one handle to the closed blob");
    }
    inject_fins(&mut m, 50, &mut rng);
    inject_slivers(&mut m, 4, &mut rng);
    (m, made)
}

fn wrap_summary(m: &Mesh) -> Value {
    let mut s = session_with("scan", m.clone());
    s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    s.execute("surface.wrap", json!({"mesh": "scan", "tolerance": TOLERANCE, "min_within": MIN_WITHIN})).map_or_else(|e| json!({"error": e.to_string()}), |w| json!({"within": w["deviation"]["within"], "patches": w["patches"], "capped": w["capped"], "closed": w["closed"]}))
}

#[test]
fn gate_topological_noise_is_removed_and_the_blob_is_modelled() {
    let (scan, made) = noisy_blob(200, 31);
    let ps = pieces(&scan);
    let big = &ps[0];
    eprintln!(
        "injected {made} handles; main piece genus {:?}, non-manifold edges {}",
        big.genus(),
        ps.iter().map(|p| p.non_manifold_edges).sum::<usize>()
    );
    assert!(made >= 190, "handles injected: {made}");
    let before = wrap_summary(&scan);
    eprintln!("surface.wrap on the messy scan, as it is (200 handles): {before}");
    let mut s = session_with("scan", scan.clone());
    let r = s.execute("mesh.remesh", json!({"object": "scan"})).unwrap();
    eprintln!("remesh: {r}");
    let out = s.doc.mesh(surfacecraft_doc::ObjectId(r["id"].as_u64().unwrap())).unwrap().mesh.clone();
    let ps = pieces(&out);
    assert_eq!(ps.len(), 1);
    assert!(ps[0].closed(), "manifold and closed");
    assert_eq!(ps[0].genus(), Some(0), "the true genus of a blob");
    let worst =
        out.positions.iter().map(|p| (p.coords.norm() - blob_radius(&p.coords.normalize())).abs()).fold(0.0, f64::max);
    assert!(worst <= 0.1, "deviation from the clean shape {worst}");
    let t = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    eprintln!(
        "to_solid: verdict {} within {} patches {} volume {} preparation {}",
        t["verdict"], t["deviation"]["within"], t["patches"], t["body"]["volume"], t["preparation"]
    );
    assert_eq!(t["verdict"], "pass", "{t}");
    assert!(t["deviation"]["within"].as_f64().unwrap() >= MIN_WITHIN);
    assert_eq!(t["preparation"][0]["step"], "remesh");
    let v = t["body"]["volume"].as_f64().unwrap();
    assert!((v - blob_volume()).abs() / blob_volume() < 0.005, "volume {v} vs {}", blob_volume());
}

#[test]
fn gate_a_torus_keeps_its_genuine_handle_and_loses_the_injected_ones() {
    let mut m = weld(synth::torus(Point3::origin(), Vec3::z(), 14.0, 5.0, 300, 120));
    synth::add_noise(&mut m, 0.03, &mut Rng::new(41));
    let mut rng = Rng::new(42);
    let made = inject_handles(&mut m, 200, 1.5, &mut rng);
    inject_fins(&mut m, 50, &mut rng);
    assert_eq!(pieces(&m)[0].genus(), None, "fins make the scan non-manifold");
    let mut s = session_with("scan", m);
    let r = s.execute("mesh.remesh", json!({"object": "scan"})).unwrap();
    eprintln!("torus remesh ({made} handles injected): genus {} -> {}", r["genus_before"], r["genus_after"]);
    assert_eq!(r["genus_after"], json!([1]), "{r}");
}

#[test]
fn gate_a_one_millimetre_plate_keeps_two_faces() {
    let mut plate = synth::cuboid(Point3::origin(), Vec3::new(30.0, 30.0, 1.0), 60);
    synth::add_noise(&mut plate, 0.01, &mut Rng::new(3));
    let mut s = session_with("plate", plate);
    match s.execute("mesh.remesh", json!({"object": "plate"})) {
        Ok(r) => {
            let out = s.doc.mesh(surfacecraft_doc::ObjectId(r["id"].as_u64().unwrap())).unwrap().mesh.clone();
            let n = out.vertex_normals();
            let mean_z = |up: bool| {
                let z: Vec<f64> = out
                    .positions
                    .iter()
                    .zip(&n)
                    .filter(|(p, v)| {
                        (if up { v.z > 0.95 } else { v.z < -0.95 })
                            && p.x > 5.0
                            && p.x < 25.0
                            && p.y > 5.0
                            && p.y < 25.0
                    })
                    .map(|(p, _)| p.z)
                    .collect();
                z.iter().sum::<f64>() / z.len().max(1) as f64
            };
            let thickness = mean_z(true) - mean_z(false);
            eprintln!("plate remeshed at voxel {}: thickness {thickness}", r["voxel"]);
            assert!((thickness - 1.0).abs() <= 0.05, "{thickness}");
        }
        Err(e) => assert!(e.to_string().contains("thin"), "a refusal must say why: {e}"),
    }
}

#[test]
fn gate_an_open_scan_is_refused_not_closed_over() {
    let mut half = weld(synth::sphere(Point3::origin(), 20.0, 80, 40));
    half.triangles.retain(|t| t.iter().all(|&v| half.positions[v as usize].z > -5.0));
    let mut s = session_with("scan", half);
    let e = s.execute("mesh.remesh", json!({"object": "scan"})).unwrap_err().to_string();
    assert!(e.contains("open"), "{e}");
}

#[test]
fn gate_a_resolution_beyond_the_limits_is_refused_with_a_clear_error() {
    let mut s = session_with("scan", weld(synth::sphere(Point3::origin(), 20.0, 80, 40)));
    let e = s.execute("mesh.remesh", json!({"object": "scan", "voxel": 0.0005})).unwrap_err().to_string();
    assert!(e.contains("limit") && e.contains("voxel"), "{e}");
}

#[test]
fn gate_a_scan_that_works_takes_the_unchanged_path() {
    let mut m = blob(160, 80);
    synth::add_noise(&mut m, 0.03, &mut Rng::new(5));
    let mut s = session_with("scan", m);
    let t = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert_eq!(t["preparation"], json!([]), "{t}");
    assert!(t["components"].is_null());
    assert_eq!(t["verdict"], "pass", "{t}");
}

#[test]
#[ignore = "exploration: where does today's wrap stall on injected handles"]
fn explore_handle_density() {
    for handles in [200usize, 1000, 3000] {
        let mut m = blob(300, 150);
        synth::add_noise(&mut m, 0.03, &mut Rng::new(31));
        let mut rng = Rng::new(32);
        let made = inject_handles(&mut m, handles, 1.5, &mut rng);
        let started = std::time::Instant::now();
        let r = wrap_summary(&m);
        eprintln!(
            "handles {made}: genus {:?}: wrap {r} in {:.1} s",
            pieces(&m)[0].genus(),
            started.elapsed().as_secs_f64()
        );
    }
}

/// RED: with a thousand handles today's wrap does not close its network and runs past the patch cap
/// (the reduction stalls on the topology); after the rebuild `scan.to_solid` passes.
#[test]
fn gate_a_thousand_handles_stall_the_wrap_and_the_rebuild_cures_it() {
    let (scan, made) = noisy_blob(1000, 77);
    let before = wrap_summary(&scan);
    eprintln!("RED ({made} handles): surface.wrap as it is: {before}");
    assert!(
        before["closed"] == json!(false)
            || before["within"].as_f64().unwrap_or(0.0) < MIN_WITHIN
            || before["capped"] == json!(true),
        "{before}"
    );
    let mut s = session_with("scan", scan);
    let t = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    eprintln!(
        "GREEN: verdict {} within {} patches {} volume {} preparation {}",
        t["verdict"], t["deviation"]["within"], t["patches"], t["body"]["volume"], t["preparation"]
    );
    assert_eq!(t["verdict"], "pass", "{t}");
    assert_eq!(t["preparation"][0]["step"], "remesh");
    let v = t["body"]["volume"].as_f64().unwrap();
    assert!((v - blob_volume()).abs() / blob_volume() < 0.005, "volume {v}");
}
