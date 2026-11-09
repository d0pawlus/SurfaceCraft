//! P4 gates: unstructured retopology and patch networks for organic shapes, through the command
//! layer. A closed organic blob and an open surface that wraps more than 180° (not a height
//! field) are made to look like raw scans (`synth::scanify`, σ = 0.025 mm), cleaned,
//! retopologised and fitted; the deviation threshold is that of design doc §11.1, unchanged.
//!
//! `tangency-labs/models` holds no models yet, so both shapes are generated.
use serde_json::json;
use std::f64::consts::{PI, TAU};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Bvh, Mesh, Point3, Rng, Vec3, synth};
use surfacecraft_kernel::{nurbs, solid};

/// Radius of the blob in the unit direction `d`: a sphere with smooth, analytic bumps of about
/// ±4 mm (no singularity at the poles of the parametrisation).
fn blob_direction_radius(d: &Vec3) -> f64 {
    20.0 * (1.0
        + 0.09 * (3.0 * d.x).sin() * (2.0 * d.y).cos()
        + 0.07 * (2.0 * d.z + 1.2 * d.x).sin() * (3.0 * d.y).cos()
        + 0.05 * (4.0 * d.z).cos() * d.x)
}

fn blob_radius(theta: f64, phi: f64) -> f64 {
    blob_direction_radius(&Vec3::new(phi.sin() * theta.cos(), phi.sin() * theta.sin(), phi.cos()))
}

/// Exact volume of the blob: (1/3) ∬ r³ sin φ dφ dθ by the midpoint rule.
fn blob_volume() -> f64 {
    let (nt, np) = (720, 360);
    let mut v = 0.0;
    for i in 0..nt {
        let th = TAU * (i as f64 + 0.5) / nt as f64;
        for j in 0..np {
            let ph = PI * (j as f64 + 0.5) / np as f64;
            v += blob_radius(th, ph).powi(3) * ph.sin();
        }
    }
    v * TAU / nt as f64 * PI / np as f64 / 3.0
}

fn blob_scan(noise: f64, seed: u64) -> Mesh {
    let mut m = synth::sphere(Point3::origin(), 1.0, 200, 100);
    for p in &mut m.positions {
        let d = p.coords;
        p.coords = d * blob_direction_radius(&d);
    }
    let d = synth::ScanDefects {
        spacing: 0.9,
        density_ramp: 0.6,
        noise,
        holes: 2,
        hole_radius: 1.5,
        spikes: 0.0003,
        debris: 3,
    };
    synth::scanify(&m, &d, &mut Rng::new(seed))
}

fn mesh_volume(m: &Mesh) -> f64 {
    (0..m.triangle_count())
        .filter_map(|i| m.triangle(i))
        .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
        .sum()
}

/// Gate P4(a): a closed organic blob, scanned (σ = 0.025 mm), becomes a field-aligned quad layout,
/// a watertight network of bicubic patches within 0.2 mm of the scan, and a solid whose volume
/// matches the exact one.
#[test]
fn gate_p4_closed_organic_blob() {
    let mut s = Session::new();
    let scan = blob_scan(0.025, 11);
    s.doc.add_mesh("scan", scan);
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    assert_eq!(c["holes_left"], 0, "{c}");
    let r = s.execute("quad.retopo", json!({"mesh": "scan", "target_quads": 60})).unwrap();
    eprintln!("gate_p4(a) retopo: {r}");
    assert_eq!(r["layout"], "field", "{r}");
    assert_eq!(r["closed"], true);
    assert_eq!(r["euler"], 2, "a sphere-like layout has Euler characteristic 2: {r}");
    assert!(r["snap_max"].as_f64().unwrap() < 0.5, "{r}");
    let a = s.execute("surface.auto", json!({"object": r["id"], "tolerance": 0.2})).unwrap();
    eprintln!("gate_p4(a) auto: {a}");
    let max = a["max"].as_f64().unwrap();
    assert!(max <= 0.2, "max deviation {max}: {a}");
    assert_eq!(a["closed"], true, "{a}");
    let jump_max = a["normal_jump_max_deg"].as_f64().unwrap();
    let jump_mean = a["normal_jump_mean_deg"].as_f64().unwrap();
    assert!(jump_max < 4.0 && jump_mean < 0.6, "normal jump max {jump_max} mean {jump_mean}");

    // independent deviation: scan vertices against a dense tessellation of the kernel sheet
    let sid = a["blocks"][0]["id"].as_u64().unwrap();
    let surf = s.doc.surfaces.iter().find(|o| o.id.0 == sid).unwrap().clone();
    let tess = surfacecraft_kernel::tess::tessellate(&surf.sheet, Some(0.005)).unwrap();
    let bvh = Bvh::build(&tess);
    let scan = s.doc.find_by_name("scan").map(|(id, _)| s.doc.mesh(id).unwrap().mesh.clone()).unwrap();
    let dev: f64 =
        scan.positions.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).map(|h| h.distance).fold(0.0, f64::max);
    // deviation of the patches from the exact (noise-free) blob, radially
    let truth: f64 = tess
        .positions
        .iter()
        .map(|p| (p.coords.norm() - blob_direction_radius(&p.coords.normalize())).abs())
        .fold(0.0, f64::max);
    eprintln!("gate_p4(a): scan -> patches max {dev:.4} mm; patches -> exact blob max {truth:.4} mm");
    assert!(dev <= 0.2, "independent scan deviation {dev}");
    assert!(truth <= 0.2, "deviation from the exact blob {truth}");

    // the closed network is a solid: watertight, one body, volume of the blob
    assert!(nurbs::is_closed(&surf.sheet));
    let body = s.doc.body(surfacecraft_doc::ObjectId(a["solid"].as_u64().unwrap())).unwrap();
    let v = solid::volume(&body.brep).unwrap();
    let exact = blob_volume();
    let cad_mesh = {
        let mut m = synth::sphere(Point3::origin(), 1.0, 400, 200);
        for p in &mut m.positions {
            let d = p.coords;
            p.coords = d * blob_direction_radius(&d);
        }
        mesh_volume(&m)
    };
    eprintln!(
        "gate_p4(a): solid {} faces, volume {v:.2} mm3 vs exact {exact:.2} ({:.3} %), dense CAD mesh {cad_mesh:.2}",
        body.brep.faces.len(),
        100.0 * (v - exact).abs() / exact
    );
    assert!((v - exact).abs() / exact < 0.01, "volume {v} vs exact {exact}");
    assert_eq!(body.brep.edges.iter().filter(|e| e.uses.len() != 2).count(), 0, "every edge is shared by two faces");
    assert_eq!(a["volume"].as_f64().map(|x| (x - v).abs() < 1e-6), Some(true));
}

/// Parametric open surface that wraps `sweep` radians around the z axis: a bumpy half pipe.
fn pipe_point(theta: f64, z: f64, sweep: f64) -> Point3 {
    let u = theta / sweep;
    let r = 15.0 * (1.0 + 0.10 * (2.0 * PI * u * 1.5).sin() * (z / 9.0).cos() + 0.04 * (z / 5.0).sin());
    Point3::new(r * theta.cos(), r * theta.sin(), z)
}

fn pipe_scan(noise: f64, seed: u64, sweep: f64) -> Mesh {
    let (nu, nv) = (220usize, 160usize);
    let height = 40.0;
    let mut positions = Vec::new();
    for j in 0..=nv {
        for i in 0..=nu {
            positions.push(pipe_point(sweep * i as f64 / nu as f64, height * j as f64 / nv as f64, sweep));
        }
    }
    let mut triangles = Vec::new();
    let w = (nu + 1) as u32;
    for j in 0..nv as u32 {
        for i in 0..nu as u32 {
            let a = j * w + i;
            // counter-clockwise seen from outside (the +radial side)
            triangles.push([a, a + 1, a + w + 1]);
            triangles.push([a, a + w + 1, a + w]);
        }
    }
    let mut m = Mesh { positions, triangles };
    synth::add_noise(&mut m, noise, &mut Rng::new(seed));
    m
}

/// Gate P4(b): an open organic surface wrapping 240° (far from a height field) gets a layout
/// whose patches follow it within 0.2 mm; the structured Fit Surface refuses the same region.
#[test]
fn gate_p4_open_wrapped_patch() {
    let mut s = Session::new();
    let sweep = 240f64.to_radians();
    s.doc.add_mesh("scan", pipe_scan(0.025, 4, sweep));
    // not a height field: the single-patch fit refuses it
    assert!(s.execute("surface.fit", json!({"mesh": "scan"})).is_err());
    assert!(s.execute("quad.create", json!({"mesh": "scan", "target_quads": 40})).is_err());
    let r = s.execute("quad.retopo", json!({"mesh": "scan", "target_quads": 80})).unwrap();
    eprintln!("gate_p4(b) retopo: {r}");
    assert_eq!(r["layout"], "field", "{r}");
    assert_eq!(r["closed"], false);
    let a = s.execute("surface.auto", json!({"object": r["id"], "tolerance": 0.2})).unwrap();
    eprintln!("gate_p4(b) auto: {a}");
    assert!(a["max"].as_f64().unwrap() <= 0.2, "{a}");
    assert_eq!(a["closed"], false);
    assert!(a["uncovered"].as_f64().unwrap() < 0.1, "the layout must reach most of the scan: {a}");
    assert!(a["normal_jump_max_deg"].as_f64().unwrap() < 4.0, "{a}");
    // independent deviation of the covered scan against the kernel tessellation
    let sid = a["blocks"][0]["id"].as_u64().unwrap();
    let surf = s.doc.surfaces.iter().find(|o| o.id.0 == sid).unwrap().clone();
    let tess = surfacecraft_kernel::tess::tessellate(&surf.sheet, Some(0.005)).unwrap();
    let bvh = Bvh::build(&tess);
    let scan = s.doc.find_by_name("scan").map(|(id, _)| s.doc.mesh(id).unwrap().mesh.clone()).unwrap();
    let mut dists: Vec<f64> =
        scan.positions.iter().filter_map(|p| bvh.closest(p, f64::INFINITY)).map(|h| h.distance).collect();
    dists.sort_by(f64::total_cmp);
    let p99 = dists[dists.len() * 99 / 100];
    eprintln!("gate_p4(b): 99th percentile of scan -> patches {p99:.4} mm over {} points", dists.len());
    assert!(p99 <= 0.2, "99th percentile {p99}");
}

/// A housing: a 60 × 40 × 24 box whose six faces bulge outward by up to 2.5 mm (cosine domes that
/// vanish at the edges), so the faces are smooth and curved and meet along sharp edges.
fn housing_mesh() -> (Mesh, f64) {
    let (w, d, h, a) = (60.0, 40.0, 24.0, 2.5);
    let n = 36usize;
    let bulge = |s: f64, t: f64, ls: f64, lt: f64| a * (PI * s / ls).cos() * (PI * t / lt).cos();
    let mut all = Mesh { positions: Vec::new(), triangles: Vec::new() };
    // the face perpendicular to `axis`, displaced outward by `sign`
    for axis in 0..3usize {
        for sign in [-1.0, 1.0] {
            let (la, lb, lc) = match axis {
                0 => (w, d, h),
                1 => (d, w, h),
                _ => (h, w, d),
            };
            let mut positions = Vec::new();
            for j in 0..=n {
                for i in 0..=n {
                    let s = (i as f64 / n as f64 - 0.5) * lb;
                    let t = (j as f64 / n as f64 - 0.5) * lc;
                    let off = sign * (la / 2.0 + bulge(s, t, lb, lc));
                    positions.push(match axis {
                        0 => Point3::new(off, s, t),
                        1 => Point3::new(s, off, t),
                        _ => Point3::new(s, t, off),
                    });
                }
            }
            let mut triangles = Vec::new();
            let wv = (n + 1) as u32;
            for j in 0..n as u32 {
                for i in 0..n as u32 {
                    let q = j * wv + i;
                    triangles.push([q, q + 1, q + wv + 1]);
                    triangles.push([q, q + wv + 1, q + wv]);
                }
            }
            let mut face = Mesh { positions, triangles };
            // wind outward
            let inward = (0..face.triangle_count())
                .filter(|&k| {
                    let (nn, c) = (face.face_normal(k).unwrap(), face.face_centroid(k).unwrap());
                    nn.dot(&c.coords) < 0.0
                })
                .count();
            if inward * 2 > face.triangle_count() {
                for t in &mut face.triangles {
                    t.swap(1, 2);
                }
            }
            all.append(&face);
        }
    }
    all.weld(1e-9);
    let cap = |l1: f64, l2: f64| 2.0 * a * (2.0 * l1 / PI) * (2.0 * l2 / PI);
    (all, w * d * h + cap(w, d) + cap(d, h) + cap(w, h))
}

/// Gate P4(c): a housing with clear edges, scanned (σ = 0.025 mm), gets a patch network from its
/// sharp-edge curves: six degree-5 patches meeting along shared feature curves, within 0.2 mm,
/// closed into a solid of the exact volume.
#[test]
fn gate_p4_feature_curve_housing() {
    let (cad, exact) = housing_mesh();
    let mut s = Session::new();
    let d = synth::ScanDefects {
        spacing: 0.9,
        density_ramp: 0.6,
        noise: 0.025,
        holes: 2,
        hole_radius: 1.5,
        spikes: 0.0003,
        debris: 3,
    };
    s.doc.add_mesh("scan", synth::scanify(&cad, &d, &mut Rng::new(21)));
    let c = s.execute("mesh.clean", json!({"object": "scan", "fill_holes_max_edges": 400})).unwrap();
    assert_eq!(c["holes_left"], 0, "{c}");
    let r = s.execute("surface.from_curves", json!({"mesh": "scan"})).unwrap();
    eprintln!("gate_p4(c): {r}");
    assert_eq!(r["patches"], 6, "{r}");
    assert_eq!(r["closed"], true, "{r}");
    assert!(r["skipped"].as_array().unwrap().is_empty(), "{r}");
    assert_eq!(r["curves"], 12, "a box has 12 feature curves: {r}");
    let max = r["max"].as_f64().unwrap();
    assert!(max <= 0.2, "max deviation {max}");
    let v = r["volume"].as_f64().unwrap();
    eprintln!("gate_p4(c): volume {v:.2} vs exact {exact:.2} ({:.3} %)", 100.0 * (v - exact).abs() / exact);
    assert!((v - exact).abs() / exact < 0.01, "volume {v} vs {exact}");
    assert!(r["solid"].as_u64().is_some());
}

/// Retopology, patches from curves and AutoSurface return errors, never panics, on tiny,
/// flat and otherwise unsuitable meshes and on a layout corrupted in the document.
#[test]
fn retopo_and_curves_never_panic_on_unsuitable_input() {
    use surfacecraft_engine::EngineError;
    for shape in ["plane", "box", "cylinder", "sphere", "torus", "wave", "cone"] {
        for res in [4usize, 8, 16] {
            let mut s = Session::new();
            s.execute("mesh.synth", json!({"shape": shape, "resolution": res, "name": "m"})).unwrap();
            for (cmd, p) in [
                ("quad.retopo", json!({"mesh": "m", "target_quads": 8})),
                ("quad.retopo", json!({"mesh": "m", "target_quads": 40})),
                ("surface.from_curves", json!({"mesh": "m"})),
                ("surface.from_curves", json!({"mesh": "m", "angle": 10.0, "min_triangles": 4, "degree": 2})),
            ] {
                let r = s.execute(cmd, p);
                assert!(!matches!(r, Err(EngineError::Internal(_))), "{cmd} on {shape} {res}: {r:?}");
            }
        }
    }
    let mut s = Session::new();
    s.doc.add_mesh("scan", blob_scan(0.0, 3));
    let r = s.execute("quad.retopo", json!({"mesh": "scan", "target_quads": 30})).unwrap();
    let id = surfacecraft_doc::ObjectId(r["id"].as_u64().unwrap());
    s.doc.quad_mut(id).unwrap().faces[0][0] = 9_999_999;
    assert!(s.execute("surface.auto", json!({"object": id.0})).is_err());
    assert!(s.execute("quad.split", json!({"object": id.0})).is_err());
    s.doc.quad_mut(id).unwrap().normals.clear();
    assert!(s.execute("quad.snap", json!({"object": id.0})).is_err());
    // structured-only operations refuse an unstructured layout
    let r = s.execute("quad.retopo", json!({"mesh": "scan", "target_quads": 30})).unwrap();
    assert!(s.execute("quad.merge", json!({"object": r["id"]})).is_err());
    assert!(s.execute("quad.crease", json!({"object": r["id"], "u_lines": [1]})).is_err());
    assert!(s.execute("quad.retopo", json!({"mesh": "scan", "target_quads": 3})).is_err());
}
