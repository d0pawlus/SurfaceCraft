//! M4 through the command layer: quad layout, AutoSurface, creases, thicken.
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_kernel::nurbs::SurfaceProbe;

fn wave(noise: f64) -> Session {
    let mut s = Session::new();
    s.execute("mesh.synth", json!({"shape": "wave", "resolution": 90, "noise": noise, "seed": 8, "name": "scan"}))
        .unwrap();
    s
}

/// Gate M4: AutoSurface over a quad layout keeps the maximum deviation ≤ 0.2 mm, and its
/// patches meet G1 (normals agree across every shared edge).
#[test]
fn gate_m4_autosurface_deviation_and_continuity() {
    for noise in [0.0, 0.02] {
        let mut s = wave(noise);
        let q = s.execute("quad.create", json!({"mesh": "scan", "target_quads": 12})).unwrap();
        assert_eq!(q["nu"].as_u64().unwrap() * q["nv"].as_u64().unwrap(), 12, "{q}");
        let a = s.execute("surface.auto", json!({"object": q["id"]})).unwrap();
        let max = a["max"].as_f64().unwrap();
        eprintln!("gate_m4 autosurface σ={noise}: {} patches, max {max:.4} mm", a["blocks"][0]["patches"]);
        assert!(max <= 0.2, "{a}");
        assert_eq!(a["blocks"][0]["patches"], 12);
        // G1: across every edge two faces share, the surface normals agree
        let sid = a["blocks"][0]["id"].as_u64().unwrap();
        let surf = s.doc.surfaces.iter().find(|o| o.id.0 == sid).unwrap();
        let sheet = &surf.sheet;
        let mut worst: f64 = 0.0;
        for e in sheet.edges.iter().filter(|e| e.uses.len() == 2) {
            let normals: Vec<[f64; 3]> = e
                .uses
                .iter()
                .map(|u| {
                    let f = &sheet.faces[u.face as usize];
                    let probe = SurfaceProbe::new(&f.surface).unwrap();
                    let [[u0, u1], [v0, v1]] = f.domain;
                    // the shared edge point in this face's parameters: midpoint of the use's pcurve
                    let surfacecraft_kernel::record::brep::Loop::Edges { edges } = &f.loops[u.r#loop as usize] else {
                        panic!()
                    };
                    let surfacecraft_kernel::record::brep::Curve2::Spline { points, .. } =
                        &edges[u.position as usize].pcurve
                    else {
                        panic!()
                    };
                    let (a, b) = (points[0], points[points.len() - 1]);
                    let (pu, pv) = ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0);
                    let h = 1e-4 * ((u1 - u0) + (v1 - v0));
                    let du = sub(probe.eval((pu + h).min(u1), pv).unwrap(), probe.eval((pu - h).max(u0), pv).unwrap());
                    let dv = sub(probe.eval(pu, (pv + h).min(v1)).unwrap(), probe.eval(pu, (pv - h).max(v0)).unwrap());
                    normalize(cross(du, dv))
                })
                .collect();
            let c = dot(normals[0], normals[1]).clamp(-1.0, 1.0);
            worst = worst.max(c.acos());
        }
        eprintln!("gate_m4 G1: largest normal jump across shared edges {worst:e} rad");
        assert!(worst < 1e-3, "normals differ by {worst} rad across a shared edge");
    }
}

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn normalize(a: [f64; 3]) -> [f64; 3] {
    let n = dot(a, a).sqrt();
    [a[0] / n, a[1] / n, a[2] / n]
}

#[test]
fn split_merge_crease_and_thicken() {
    let mut s = wave(0.02);
    let q = s.execute("quad.create", json!({"mesh": "scan", "target_quads": 6})).unwrap();
    let id = q["id"].clone();
    let sp = s.execute("quad.split", json!({"object": id})).unwrap();
    assert_eq!(sp["nu"].as_u64().unwrap(), 2 * q["nu"].as_u64().unwrap());
    assert!(sp["snap_max"].as_f64().unwrap() < 0.2);
    let mg = s.execute("quad.merge", json!({"object": id})).unwrap();
    assert_eq!(mg["nu"], q["nu"]);
    let nu = q["nu"].as_u64().unwrap();
    if nu >= 2 {
        s.execute("quad.crease", json!({"object": id, "u_lines": [1]})).unwrap();
        let a = s.execute("surface.auto", json!({"object": id})).unwrap();
        assert_eq!(a["blocks"].as_array().unwrap().len(), 2, "a crease splits the layout in two blocks: {a}");
        assert!(a["max"].as_f64().unwrap() <= 0.2);
        s.execute("edit.undo", Value::Null).unwrap();
    }
    s.execute("quad.crease", json!({"object": id, "u_lines": [1], "remove": true})).unwrap();
    let a = s.execute("surface.auto", json!({"object": id})).unwrap();
    let t = s.execute("surface.thicken", json!({"object": a["blocks"][0]["id"], "thickness": 2.0})).unwrap();
    let v = t["volume"].as_f64().unwrap();
    assert!(v > 60.0 * 40.0 * 2.0 * 0.99 && v < 60.0 * 40.0 * 2.0 * 1.2, "{t}");
    // the thickened solid exports as STEP through solidcore and reads back
    let e = s.execute("export.step", json!({"objects": [t["id"]]})).unwrap();
    assert_eq!(e["content"], "solids");
    let back = s.execute("import.step", json!({"data": e["data"]})).unwrap();
    assert_eq!(back["bodies"].as_array().unwrap().len(), 1);
}

#[test]
fn bad_quad_requests() {
    let mut s = wave(0.0);
    assert!(s.execute("quad.create", json!({"mesh": "scan", "target_quads": 0})).is_err());
    let q = s.execute("quad.create", json!({"mesh": "scan", "target_quads": 1})).unwrap();
    assert!(s.execute("quad.merge", json!({"object": q["id"]})).is_err());
    assert!(s.execute("quad.crease", json!({"object": q["id"], "u_lines": [0]})).is_err());
    assert!(s.execute("surface.thicken", json!({"object": "scan", "thickness": 1.0})).is_err());
}
