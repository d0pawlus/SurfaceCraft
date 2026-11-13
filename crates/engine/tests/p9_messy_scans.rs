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

/// A closed organic part: a sphere of radius `r` at `c` with a wavy skin, noise 0.02.
fn wavy(c: Point3, r: f64, k: f64, res: usize) -> Mesh {
    let mut m = weld(synth::sphere(Point3::origin(), 1.0, res, res / 2));
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = c.coords + d * r * (1.0 + 0.08 * (k * d.x).sin() * (k * 0.7 * d.y).cos());
    }
    synth::add_noise(&mut m, 0.02, &mut Rng::new(c.x as u64 + 3));
    m
}

/// `parts` placed 60 mm apart along x, plus `specks` closed specks of debris far away.
fn assembly(parts: &[Mesh], specks: usize) -> Mesh {
    let mut all = Mesh::default();
    for (n, p) in parts.iter().enumerate() {
        let mut q = p.clone();
        q.positions.iter_mut().for_each(|v| v.x += 60.0 * n as f64);
        all.append(&q);
    }
    let mut rng = Rng::new(9);
    for _ in 0..specks {
        let at =
            Point3::new(rng.range(-20.0, 60.0 * parts.len() as f64), rng.range(60.0, 140.0), rng.range(-10.0, 30.0));
        all.append(&weld(synth::sphere(at, 1.5, 16, 8)));
    }
    all
}

fn three_parts_and_debris() -> (Mesh, [Mesh; 3]) {
    let a = wavy(Point3::new(0.0, 0.0, 0.0), 14.0, 3.0, 120);
    let b = wavy(Point3::new(45.0, 0.0, 0.0), 10.0, 4.0, 120);
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
}

/// A hollow casting: the outer skin is modelled, the cavity shell is reported, and the verdict says so.
#[test]
fn gate_a_hollow_casting_models_the_outer_shell_and_says_what_it_left_out() {
    let outer = wavy(Point3::origin(), 15.0, 3.0, 120);
    let mut inner = wavy(Point3::origin(), 9.0, 3.0, 96);
    inner.triangles.iter_mut().for_each(|t| t.swap(1, 2));
    let mut hollow = outer;
    hollow.append(&inner);
    let mut s = session_with("scan", hollow);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert_eq!(out["components"].as_array().map(Vec::len), Some(1), "{out}");
    assert_eq!(out["bodies"].as_array().map(Vec::len), Some(1));
    let codes: Vec<&str> = out["issues"].as_array().unwrap().iter().filter_map(|i| i["code"].as_str()).collect();
    assert!(codes.contains(&"nested_shell_skipped"), "{codes:?}");
    assert_eq!(out["verdict"], "review", "{out}");
    assert_eq!(out["preparation"][0]["numbers"]["cavity_shells"], 1);
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
    // every part passed: one export of all the bodies, and the bodies are listed
    let ids: Vec<u64> = comps.iter().map(|c| c["body"]["id"].as_u64().unwrap()).collect();
    assert_eq!(out["bodies"].as_array().map(Vec::len), Some(3));
    assert_eq!(out["next_steps"].as_array().map(Vec::len), Some(1), "{}", out["next_steps"]);
    assert_eq!(out["next_steps"][0]["command"], "export.step");
    assert_eq!(out["next_steps"][0]["params"]["objects"], json!(ids));
    assert!(out["deviation"]["within"].as_f64().unwrap() >= MIN_WITHIN, "the aggregate over the parts");
    let mut s1 = session_with("scan", all.clone());
    let one =
        s1.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "components": "largest"})).unwrap();
    assert_eq!(one["components"].as_array().unwrap().len(), 1, "{one}");
    let mut s2 = session_with("scan", all);
    let two = s2.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "components": 2})).unwrap();
    assert_eq!(two["components"].as_array().unwrap().len(), 2);
    // parts that were not asked for are not an issue and do not lower a pass; the numbers say how many
    assert_eq!(one["verdict"], "pass", "{one}");
    assert_eq!(one["issues"], json!([]), "{one}");
    assert_eq!(one["preparation"][0]["numbers"]["not_requested"], 2);
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
    // recorded, not hoped for: with 200 handles today's wrap still reaches the tolerance (so this gate
    // is not RED at 200; the thousand-handle gate is)
    assert!(before["error"].is_null(), "{before}");
    assert_eq!(before["closed"], json!(true), "{before}");
    assert!(before["within"].as_f64().unwrap() >= MIN_WITHIN, "{before}");
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
    assert!(t["preparation"][0]["numbers"]["ms"].is_u64(), "{t}");
    assert_eq!(s.doc.bodies.len(), 1, "the other attempt leaves nothing behind");
    assert_eq!(s.doc.bodies[0].name, "solid", "the winner has the requested name");
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
    assert!(before["error"].is_null(), "{before}");
    assert!(
        before["closed"] == json!(false)
            || before["within"].as_f64().unwrap_or(0.0) < MIN_WITHIN
            || before["capped"] == json!(true),
        "{before}"
    );
    let mut s = session_with("scan", scan.clone());
    let plain = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "rebuild": false})).unwrap();
    eprintln!("rebuild off: verdict {} preparation {}", plain["verdict"], plain["preparation"]);
    assert_ne!(plain["verdict"], "pass", "{plain}");
    assert_eq!(plain["preparation"], json!([]));
    let mut s = session_with("scan", scan);
    let t = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    eprintln!(
        "GREEN: verdict {} within {} patches {} volume {} preparation {}",
        t["verdict"], t["deviation"]["within"], t["patches"], t["body"]["volume"], t["preparation"]
    );
    assert_eq!(t["verdict"], "pass", "{t}");
    assert_eq!(t["preparation"][0]["step"], "remesh");
    assert_eq!(s.doc.bodies.len(), 1);
    assert_eq!(s.doc.bodies[0].name, "solid");
    let v = t["body"]["volume"].as_f64().unwrap();
    assert!((v - blob_volume()).abs() / blob_volume() < 0.005, "volume {v}");
}

// ---- which parts count ---------------------------------------------------------------------------

/// One part with 98 % of the surface and a small open fragment (above the share of a part, below that
/// of a co-equal one): one body, the verdict of the main part. The fragment is shown to fail on its own
/// first, so that the pass proves that it was left out.
#[test]
fn gate_a_small_fragment_is_not_a_part_and_does_not_hide_the_main_pass() {
    let main = wavy(Point3::origin(), 20.0, 3.0, 160);
    let mut fragment = wavy(Point3::origin(), 3.2, 5.0, 60);
    fragment.triangles.retain(|t| t.iter().all(|&v| fragment.positions[v as usize].z > -0.3));
    let scan = assembly(&[main, fragment], 0);
    // the hole of the fragment (about 30 edges) is left open
    let params = |extra: Value| {
        let mut p = json!({"scan": "scan", "method": "freeform", "fill_holes_max_edges": 10});
        p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
        p
    };
    let mut s = session_with("scan", scan.clone());
    let both = s.execute("scan.to_solid", params(json!({"part_share": 0.01}))).unwrap();
    assert_eq!(both["components"].as_array().map(Vec::len), Some(2), "{both}");
    assert_ne!(both["verdict"], "pass", "the fragment on its own is not a pass: {both}");
    let mut s = session_with("scan", scan);
    let out = s.execute("scan.to_solid", params(json!({}))).unwrap();
    assert_eq!(out["components"].as_array().map(Vec::len), Some(1), "{out}");
    assert_eq!(out["bodies"].as_array().map(Vec::len), Some(1));
    assert_eq!(out["verdict"], "pass", "{out}");
    assert_eq!(out["issues"], json!([]), "{out}");
    assert_eq!(out["preparation"][0]["numbers"]["minor_and_debris_pieces"], 1);
    assert_eq!(out["body"]["id"], out["components"][0]["body"]["id"]);
}

#[test]
fn gate_two_parts_of_70_and_30_percent_are_two_bodies() {
    let big = wavy(Point3::origin(), 15.0, 3.0, 120);
    let small = wavy(Point3::origin(), 9.8, 4.0, 100);
    let mut s = session_with("scan", assembly(&[big, small], 0));
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert_eq!(out["components"].as_array().map(Vec::len), Some(2), "{out}");
    assert_eq!(out["bodies"].as_array().map(Vec::len), Some(2));
    assert_eq!(out["verdict"], "pass", "{out}");
}

/// Two parts and fifty small pieces that are no part: the pieces are reported and the verdict is not a pass.
#[test]
fn gate_unmodelled_pieces_are_an_issue_and_cap_the_verdict() {
    let a = wavy(Point3::origin(), 14.0, 3.0, 120);
    let b = wavy(Point3::origin(), 12.0, 4.0, 110);
    let mut s = session_with("scan", assembly(&[a, b], 50));
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert_eq!(out["components"].as_array().map(Vec::len), Some(2), "{out}");
    let codes: Vec<&str> = out["issues"].as_array().unwrap().iter().filter_map(|i| i["code"].as_str()).collect();
    assert!(codes.contains(&"unmodelled_pieces"), "{codes:?}");
    assert_ne!(out["verdict"], "pass", "{out}");
    assert!(
        out["issues"]
            .as_array()
            .unwrap()
            .iter()
            .any(|i| i["message"].as_str().is_some_and(|m| m.contains("50 pieces")))
    );
    // what to do about it: the message says, and the next steps split the scan; no claim that every part passed
    let says = |what: &[&str]| {
        out["issues"]
            .as_array()
            .unwrap()
            .iter()
            .any(|i| i["message"].as_str().is_some_and(|m| what.iter().all(|w| m.contains(w))))
    };
    assert!(says(&["part_share", "scan.split"]));
    let steps = out["next_steps"].as_array().unwrap();
    assert!(steps.iter().any(|st| st["command"] == "scan.split"), "{steps:?}");
    assert!(steps.iter().all(|st| !st["why"].as_str().unwrap().contains("every part passed")), "{steps:?}");
}

#[test]
fn a_cancelled_run_is_a_cancelled_error_not_a_failed_part() {
    let (all, _) = three_parts_and_debris();
    let mut s = session_with("scan", all);
    s.cancel.cancel();
    let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"}));
    let e = r.expect_err("a cancelled run is an error").to_string();
    assert!(e.contains("cancelled"), "{e}");
}

#[test]
fn scan_split_says_when_no_piece_reaches_the_share_and_wants_a_positive_share() {
    let (all, _) = three_parts_and_debris();
    let mut s = session_with("scan", all);
    let e = s.execute("scan.split", json!({"object": "scan", "min_share": 1.0})).unwrap_err().to_string();
    assert!(e.contains("no piece reaches"), "{e}");
    assert!(s.execute("scan.split", json!({"object": "scan", "min_share": 0.0})).is_err());
}

/// A single shell that does not close (the old meaning of the three-shell test of `scan_to_solid`).
#[test]
fn gate_a_single_open_shell_gets_a_verdict_and_next_steps_not_an_error() {
    let tube = weld(synth::cylinder(Point3::origin(), Vec3::z(), 8.0, 30.0, 96, 24));
    let mut s = session_with("scan", tube);
    // the ends of the tube are two holes of 96 edges: left open, they keep the scan from closing
    let out =
        s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "fill_holes_max_edges": 10})).unwrap();
    assert_ne!(out["verdict"], "pass", "{out}");
    assert!(out["components"].is_null(), "one shell is not split: {out}");
    assert_eq!(out["preparation"], json!([]));
    assert!(!out["issues"].as_array().unwrap().is_empty());
    assert!(!out["next_steps"].as_array().unwrap().is_empty());
}

/// Timing of the piece analysis `scan.to_solid` runs on every cleaned scan, on about a million triangles.
#[test]
#[ignore = "measurement"]
fn measure_piece_analysis_of_a_million_triangles() {
    let m = weld(synth::sphere(Point3::origin(), 150.0, 1000, 500));
    let started = std::time::Instant::now();
    let ps = pieces(&m);
    eprintln!("pieces of {} triangles: {} in {:.2} s", m.triangle_count(), ps.len(), started.elapsed().as_secs_f64());
    let started = std::time::Instant::now();
    let mut s = session_with("scan", m);
    let r = s.execute("scan.split", json!({"object": "scan"})).unwrap();
    eprintln!(
        "scan.split (analysis with nesting, parts made): {} parts in {:.2} s",
        r["parts"].as_array().unwrap().len(),
        started.elapsed().as_secs_f64()
    );
}

/// Footprint of `mesh.remesh` on a scan of about a million triangles and 300 mm at the default voxel:
/// run it alone under `/usr/bin/time -l` to read the peak memory of the process.
#[test]
#[ignore = "measurement: run alone under /usr/bin/time -l"]
fn measure_remesh_footprint_of_a_million_triangles() {
    let mut m = weld(synth::sphere(Point3::origin(), 1.0, 1000, 500));
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = d * 150.0 * (1.0 + 0.05 * (3.0 * d.x).sin() * (2.0 * d.y).cos());
    }
    synth::add_noise(&mut m, 0.03, &mut Rng::new(2));
    eprintln!("scan: {} triangles, extent {:?}, area {:.0} mm2", m.triangle_count(), m.bbox().size(), m.area());
    let mut s = session_with("scan", m);
    let started = std::time::Instant::now();
    let r = s.execute("mesh.remesh", json!({"object": "scan"}));
    match r {
        Ok(r) => eprintln!(
            "remesh in {:.1} s: voxel {} grid {} crossings {} triangles {} estimated peak {} MB deviation to scan p99 {} max {}",
            started.elapsed().as_secs_f64(),
            r["voxel"],
            r["grid"],
            r["crossings"],
            r["triangles"],
            r["peak_bytes"].as_u64().unwrap_or(0) / 1_000_000,
            r["original_to_result"]["p99"],
            r["original_to_result"]["max"]
        ),
        Err(e) => eprintln!("refused: {e}"),
    }
}

#[test]
fn part_share_is_validated() {
    let mut s = session_with("scan", wavy(Point3::origin(), 10.0, 3.0, 60));
    for bad in [json!(-0.1), json!(1.5), json!(2)] {
        let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "part_share": bad}));
        assert!(r.is_err(), "{bad}");
    }
    for ok in [json!(0), json!(1), json!(0.1)] {
        let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform", "part_share": ok}));
        assert!(r.is_ok(), "{ok}: {r:?}");
    }
}

/// A cancellation in the middle of a split run leaves nothing of the parts that were already modelled.
#[test]
fn a_cancel_in_the_middle_of_a_split_run_leaves_no_body_and_no_part_mesh() {
    let (all, _) = three_parts_and_debris();
    let mut s = session_with("scan", all);
    let flag = s.cancel.clone();
    let stopper = std::thread::spawn(move || {
        std::thread::sleep(std::time::Duration::from_millis(1500));
        flag.cancel();
    });
    let r = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"}));
    stopper.join().unwrap();
    let e = r.expect_err("the run takes several seconds").to_string();
    assert!(e.contains("cancelled"), "{e}");
    assert!(s.doc.bodies.is_empty(), "no body of an earlier part is left");
    assert_eq!(s.doc.meshes.len(), 1, "no part mesh is left: only the scan");
}

/// An opening that cleaning cannot fill (above 400 edges) at the default parameters: a verdict with
/// reasons and next steps, never an error or a pass.
#[test]
fn gate_an_open_scan_that_cannot_be_closed_gets_reasons_and_next_steps_at_the_default_parameters() {
    let mut half = weld(synth::sphere(Point3::origin(), 20.0, 500, 250));
    half.triangles.retain(|t| t.iter().all(|&v| half.positions[v as usize].z > -1e-9));
    let loops = pieces(&half)[0].loops.clone();
    assert!(loops.len() == 1 && loops[0].edges > 400, "{loops:?}");
    let mut s = session_with("scan", half);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    assert_ne!(out["verdict"], "pass", "{out}");
    assert!(!out["issues"].as_array().unwrap().is_empty());
    assert!(!out["next_steps"].as_array().unwrap().is_empty());
    assert!(out["components"].is_null());
}

/// Strong evidence (many fins) but a remesh that is refused: the refused step is in `preparation`.
#[test]
fn gate_a_refused_remesh_is_recorded_in_the_preparation() {
    let mut m = wavy(Point3::origin(), 8.0, 3.0, 80);
    // every other triangle turned around: a closed scan whose orientation cannot be read, so that the
    // remesh refuses; thirty fins make the evidence strong
    m.triangles.iter_mut().step_by(2).for_each(|t| t.swap(1, 2));
    let mut rng = Rng::new(4);
    inject_fins(&mut m, 30, &mut rng);
    let mut s = session_with("scan", m);
    let out = s.execute("scan.to_solid", json!({"scan": "scan", "method": "freeform"})).unwrap();
    let steps = out["preparation"].as_array().unwrap();
    assert!(steps.iter().any(|p| p["step"] == "remesh" && p["numbers"]["refused"].is_string()), "{out}");
    assert!(steps.iter().all(|p| p["numbers"]["ms"].is_u64()));
}

/// Eighty equal pieces: the debris report of the split counts all of them.
#[test]
fn scan_split_counts_pieces_beyond_its_limit_of_listed_parts() {
    let ball = weld(synth::sphere(Point3::origin(), 3.0, 24, 12));
    let mut all = Mesh::default();
    for k in 0..80 {
        let mut b = ball.clone();
        b.positions.iter_mut().for_each(|p| p.x += 10.0 * f64::from(k));
        all.append(&b);
    }
    let mut s = session_with("scan", all);
    let r = s.execute("scan.split", json!({"object": "scan"})).unwrap();
    assert_eq!(r["parts"].as_array().map(Vec::len), Some(64), "{r}");
    assert_eq!(r["parts_beyond_limit"], 16);
    assert!(r["beyond_limit_triangles"].as_u64().unwrap() > 0 && r["beyond_limit_area"].as_f64().unwrap() > 0.0);
}
