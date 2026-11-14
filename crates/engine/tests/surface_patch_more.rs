//! `surface.patch`, the rest of the command's contract: G2 through the command, `conflicts`,
//! interior `curves`, `triangles`, undo, the cost bound, cancellation, and a failure that leaves the
//! document as it was. Fixtures as in `surface_patch.rs`.
use serde_json::{Value, json};
use std::f64::consts::{FRAC_PI_2, TAU};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng, synth};
use surfacecraft_kernel::nurbs;
use surfacecraft_kernel::record::brep::{Loop, Surface};
use surfacecraft_kernel::tess;

fn pentagon(r: f64) -> Vec<[f64; 3]> {
    (0..5)
        .map(|k| {
            let a = FRAC_PI_2 + TAU * f64::from(k) / 5.0;
            [r * a.cos(), r * a.sin(), 0.0]
        })
        .collect()
}

fn flat_sides(r: f64) -> Vec<Value> {
    let c = pentagon(r);
    (0..5).map(|k| json!({"points": [c[k], c[(k + 1) % 5]]})).collect()
}

/// A wavy sheet with a five-sided hole as a body; the body and the edges of the hole.
fn wavy_sheet_with_hole(s: &mut Session) -> (ObjectId, Vec<usize>) {
    s.doc.add_mesh("scan", synth::wave(Point3::origin(), 60.0, 40.0, 3.0, 90, 60));
    let fit = s.execute("surface.fit", json!({"mesh": "scan", "name": "sheet"})).unwrap();
    let spline = s.doc.surface(ObjectId(fit["id"].as_u64().unwrap())).unwrap().sheet.faces[0].surface.clone();
    let [[u0, u1], [v0, v1]] = nurbs::domain(&spline).unwrap();
    let (du, dv) = (u1 - u0, v1 - v0);
    let outer = [
        [u0 + 0.03 * du, v0 + 0.03 * dv],
        [u1 - 0.03 * du, v0 + 0.03 * dv],
        [u1 - 0.03 * du, v1 - 0.03 * dv],
        [u0 + 0.03 * du, v1 - 0.03 * dv],
    ];
    let hole: Vec<[f64; 2]> = (0..5)
        .map(|k| {
            let a = FRAC_PI_2 + TAU * f64::from(k) / 5.0;
            [u0 + 0.5 * du + 0.1 * du * a.cos(), v0 + 0.5 * dv + 0.1 * dv * a.sin()]
        })
        .collect();
    let id = s.doc.add_body("holed", nurbs::trimmed_sheet(&spline, &outer, &[hole]).unwrap());
    let body = s.doc.body(id).unwrap();
    let Loop::Edges { edges } = &body.brep.faces[0].loops[1] else { panic!("the hole is a loop of edges") };
    (id, edges.iter().map(|e| e.edge as usize).collect())
}

fn g_sides(body: ObjectId, hole: &[usize], continuity: &str) -> Vec<Value> {
    hole.iter().map(|e| json!({"body": body.0, "edge": e, "continuity": continuity})).collect()
}

#[test]
fn g2_through_the_command_reports_curvature_and_what_was_met() {
    let mut s = Session::new();
    let (body, hole) = wavy_sheet_with_hole(&mut s);
    let r = s.execute("surface.patch", json!({"sides": g_sides(body, &hole, "g2"), "name": "g2"})).unwrap();
    for e in r["sides"].as_array().unwrap() {
        assert_eq!(e["continuity"], "g2", "{e}");
        let (angle, curv) = (e["angle_max_deg"].as_f64().unwrap(), e["curvature_error_max"].as_f64().unwrap());
        assert!(angle.is_finite() && curv.is_finite(), "{e}");
        // measured on the wavy hole (aarch64): gap 0.02 to 0.33, angle 0.03 to 0.25, curvature 0.11 to
        // 0.57 of their targets (at least 1.75x to spare): every target is met, not merely reported
        assert_eq!(e["met"], true, "{e}");
        assert!(e["gap_max_mm"].as_f64().unwrap() <= 1e-3 && angle <= 0.05 && curv <= 0.05, "{e}");
    }
    assert_eq!(r["within_targets"], true, "{r}");
    assert_eq!(r["folded"], false, "{r}");
    assert!(r["hard_residual_mm"].as_f64().unwrap() < 1e-6, "{r}");
}

/// A box, and a triangle across one of its edges: two sides on the two faces that meet there (G1,
/// the planes 90 degrees apart at the shared corner) and a G0 side back.
fn corner_request(s: &mut Session, conflicts: Option<&str>) -> Value {
    s.execute("body.box", json!({"size": [10.0, 10.0, 10.0], "name": "block"})).unwrap();
    let body = s.doc.body(ObjectId(1)).unwrap().brep.clone();
    let planes: Vec<(usize, [f64; 3])> = body
        .faces
        .iter()
        .enumerate()
        .filter_map(|(k, f)| match &f.surface {
            Surface::Plane { u_axis, v_axis, .. } => Some((k, cross(*u_axis, *v_axis))),
            _ => None,
        })
        .collect();
    let (f0, n0) = planes[0];
    let (f1, n1) = *planes.iter().find(|(_, n)| dot(*n, n0).abs() < 1e-9).unwrap();
    let topo = tess::tessellate_topo(&body, None).unwrap();
    let pts: Vec<[f64; 3]> = topo.edges.iter().flatten().map(|p| [p.x, p.y, p.z]).collect();
    let centre = [0, 1, 2].map(|c| pts.iter().map(|p| p[c]).sum::<f64>() / pts.len() as f64);
    // a vertex of the box on both faces; `e` runs along the edge they share, `a` and `b` lie in the
    // first and in the second face, all three towards the box's centre
    let vertex = pts
        .iter()
        .copied()
        .find(|p| (dot(*p, n0) - dot(centre, n0)).abs() > 4.0 && (dot(*p, n1) - dot(centre, n1)).abs() > 4.0)
        .unwrap();
    let towards = |d: [f64; 3]| if dot(sub(centre, vertex), d) > 0.0 { scale(d, 1.0) } else { scale(d, -1.0) };
    let (a, b, e) = (towards(n1), towards(n0), towards(cross(n0, n1)));
    let unit = |d: [f64; 3]| scale(d, 1.0 / dot(d, d).sqrt());
    let (a, b, e) = (unit(a), unit(b), unit(e));
    let (q, r) = (add(vertex, scale(a, 3.0)), add(vertex, scale(b, 3.0)));
    let (q6, r6) = (add(q, scale(e, 6.0)), add(r, scale(e, 6.0)));
    let mut args = json!({"sides": [
        {"points": [q, vertex], "continuity": "g1", "face": {"body": 1, "face": f0}},
        {"points": [vertex, r], "continuity": "g1", "face": {"body": 1, "face": f1}},
        {"points": [r, r6, q6, q]},
    ]});
    if let Some(c) = conflicts {
        args["conflicts"] = json!(c);
    }
    args
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: [f64; 3], k: f64) -> [f64; 3] {
    [a[0] * k, a[1] * k, a[2] * k]
}

#[test]
fn conflicts_fail_refuses_and_relax_reports_the_corner() {
    let mut s = Session::new();
    let args = corner_request(&mut s, Some("fail"));
    let e = s.execute("surface.patch", args).unwrap_err().to_string();
    assert!(e.contains("corner"), "{e}");
    assert!(s.doc.surfaces.is_empty(), "a refused call leaves nothing");
    let mut s = Session::new();
    let args = corner_request(&mut s, None);
    let r = s.execute("surface.patch", args).unwrap();
    assert_eq!(r["corners"].as_array().unwrap().len(), 1, "{r}");
    assert_eq!(r["within_targets"], false, "{r}");
}

#[test]
fn interior_curves_lie_on_the_patch() {
    let mut s = Session::new();
    let curve: Vec<[f64; 3]> = (0..=10).map(|i| [-5.0 + f64::from(i), 0.0, 0.5]).collect();
    let r = s.execute("surface.patch", json!({"sides": flat_sides(15.0), "curves": [curve]})).unwrap();
    assert_eq!(r["within_targets"], true, "{r}");
    let o = s.doc.surface(ObjectId(r["surface"].as_u64().unwrap())).unwrap();
    let mesh = tess::tessellate(&o.sheet, None).unwrap();
    let top = mesh.positions.iter().map(|p| p.z).fold(f64::MIN, f64::max);
    assert!(top >= 0.4, "the patch rises to the curve: {top}");
    // a curve outside the outline's parameter domain is refused
    let far: Vec<[f64; 3]> = vec![[100.0, 0.0, 0.0], [101.0, 0.0, 0.0]];
    assert!(s.execute("surface.patch", json!({"sides": flat_sides(15.0), "curves": [far]})).is_err());
}

fn centroid(m: &Mesh, t: usize) -> Point3 {
    let [a, b, c] = m.triangle(t).unwrap();
    Point3::from((a.coords + b.coords + c.coords) / 3.0)
}

#[test]
fn triangles_of_a_mesh_are_the_data() {
    let mut m = synth::wave(Point3::origin(), 60.0, 40.0, 3.0, 90, 60);
    synth::add_noise(&mut m, 0.03, &mut Rng::new(7));
    let inside = |p: &Point3| p.x > 10.0 && p.x < 50.0 && p.y > 8.0 && p.y < 32.0;
    let ids: Vec<u32> = (0..m.triangle_count()).filter(|&t| inside(&centroid(&m, t))).map(|t| t as u32).collect();
    let line = |a: [f64; 2], b: [f64; 2]| -> Vec<[f64; 3]> {
        (0..=40)
            .map(|i| {
                let t = f64::from(i) / 40.0;
                let (x, y) = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]));
                [x, y, synth::wave_height(3.0, x, y)]
            })
            .collect()
    };
    let (a, b, c, d) = ([10.0, 8.0], [50.0, 8.0], [50.0, 32.0], [10.0, 32.0]);
    let sides: Vec<Value> =
        [(a, b), (b, c), (c, d), (d, a)].iter().map(|&(p, q)| json!({"points": line(p, q)})).collect();
    let mut s = Session::new();
    s.doc.add_mesh("scan", m);
    let r = s.execute("surface.patch", json!({"sides": sides, "mesh": "scan", "triangles": ids})).unwrap();
    assert!(r["points"].as_u64().unwrap() > 1000, "{r}");
    assert!(r["max"].as_f64().unwrap() <= 0.2, "{r}");
    // an index past the mesh is an error, and nothing is added
    let before = s.doc.surfaces.len();
    let bad = s.execute("surface.patch", json!({"sides": [], "mesh": "scan", "triangles": [u32::MAX]}));
    assert!(bad.is_err());
    assert_eq!(s.doc.surfaces.len(), before);
}

#[test]
fn one_undo_step_removes_the_patch() {
    let mut s = Session::new();
    s.execute("surface.patch", json!({"sides": flat_sides(15.0)})).unwrap();
    assert_eq!(s.doc.surfaces.len(), 1);
    s.execute("edit.undo", Value::Null).unwrap();
    assert!(s.doc.surfaces.is_empty());
    s.execute("edit.redo", Value::Null).unwrap();
    assert_eq!(s.doc.surfaces.len(), 1);
}

#[test]
fn a_call_over_the_cost_bound_is_refused_and_leaves_the_document() {
    let mut s = Session::new();
    let (body, hole) = wavy_sheet_with_hole(&mut s);
    let labels = |s: &Session| s.history.undo_labels().iter().map(ToString::to_string).collect::<Vec<_>>();
    let before = (s.doc.surfaces.len(), labels(&s));
    // 24 x 24 with G1 sides and 1500 hard points: far over the bound
    let points: Vec<Value> = (0..1500)
        .map(|k| {
            let (a, b) = (f64::from(k % 40) / 40.0, f64::from(k / 40) / 40.0);
            json!({"at": [-1.0 + 2.0 * a, -1.0 + 2.0 * b, 0.0], "hard": true})
        })
        .collect();
    let args = json!({"sides": g_sides(body, &hole, "g1"), "points": points, "control": [24, 24], "max_control": 24});
    let e = s.execute("surface.patch", args).unwrap_err().to_string();
    assert!(e.contains("per solve") || e.contains("outside"), "{e}");
    assert_eq!((s.doc.surfaces.len(), labels(&s)), before, "a failed call changes nothing");
}

#[cfg(not(target_arch = "wasm32"))]
#[test]
fn a_cancelled_patch_returns_the_cancellation_and_adds_nothing() {
    use std::sync::Arc;
    use std::time::{Duration, Instant};
    use surfacecraft_engine::{EngineError, Job, JobPoll, cmd};
    let mut s = Session::new();
    let (body, hole) = wavy_sheet_with_hole(&mut s);
    let before = s.doc.surfaces.len();
    let job = Job::start(&s, "surface.patch", Arc::new(json!({"sides": g_sides(body, &hole, "g1")}))).unwrap();
    job.cancel();
    let started = Instant::now();
    let done = loop {
        match job.poll() {
            JobPoll::Finished(d) => break *d,
            JobPoll::Lost => panic!("the job was lost"),
            JobPoll::Running => {
                assert!(started.elapsed() < Duration::from_secs(300));
                std::thread::sleep(Duration::from_millis(5));
            }
        }
    };
    assert_eq!(done.result, Err(EngineError::Failed(cmd::CANCELLED.into())));
    assert_eq!(done.session.doc.surfaces.len(), before);
}

#[test]
fn face_on_a_g0_side_is_ignored_with_a_warning() {
    let mut s = Session::new();
    s.execute("body.box", json!({"size": [10.0, 10.0, 10.0], "name": "block"})).unwrap();
    let mut sides = flat_sides(15.0);
    sides[2] = json!({"points": sides[2]["points"], "face": {"body": "block", "face": 0}});
    let r = s.execute("surface.patch", json!({"sides": sides})).unwrap();
    let w: Vec<&str> = r["warnings"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert!(w.iter().any(|w| w.contains("side 2") && w.contains("ignored")), "{w:?}");
}

/// A grid mesh on the surface `p(i / nu, j / nv)` and the four sides of its border, in loop order.
fn grid_scan(nu: usize, nv: usize, p: &dyn Fn(f64, f64) -> [f64; 3]) -> (Mesh, Vec<Value>) {
    let at = |i: usize, j: usize| p(i as f64 / nu as f64, j as f64 / nv as f64);
    let mut positions = Vec::new();
    for j in 0..=nv {
        for i in 0..=nu {
            let q = at(i, j);
            positions.push(Point3::new(q[0], q[1], q[2]));
        }
    }
    let w = (nu + 1) as u32;
    let mut triangles = Vec::new();
    for j in 0..nv as u32 {
        for i in 0..nu as u32 {
            let a = j * w + i;
            triangles.push([a, a + 1, a + w + 1]);
            triangles.push([a, a + w + 1, a + w]);
        }
    }
    let bottom: Vec<[f64; 3]> = (0..=nu).map(|i| at(i, 0)).collect();
    let right: Vec<[f64; 3]> = (0..=nv).map(|j| at(nu, j)).collect();
    let top: Vec<[f64; 3]> = (0..=nu).rev().map(|i| at(i, nv)).collect();
    let left: Vec<[f64; 3]> = (0..=nv).rev().map(|j| at(0, j)).collect();
    let sides = [bottom, right, top, left].iter().map(|p| json!({"points": p, "soft": true})).collect();
    (Mesh { positions, triangles }, sides)
}

#[test]
fn a_region_over_the_flatten_limit_is_fitted_on_the_plane_of_the_outline() {
    let wave = |u: f64, v: f64| {
        let (x, y) = (60.0 * u, 40.0 * v);
        [x, y, synth::wave_height(3.0, x, y)]
    };
    let (mesh, sides) = grid_scan(40, 30, &wave);
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    let flattened = s.execute("surface.patch", json!({"sides": sides, "mesh": "scan"})).unwrap();
    assert_eq!(flattened["parameters"], "flattened", "{flattened}");
    let r = s.execute("surface.patch", json!({"sides": sides, "mesh": "scan", "max_flatten_triangles": 100})).unwrap();
    assert_eq!(r["parameters"], "plane", "{r}");
    let w: Vec<&str> = r["warnings"].as_array().unwrap().iter().filter_map(Value::as_str).collect();
    assert!(w.iter().any(|w| w.contains("2400 triangles") && w.contains("at most 100")), "{w:?}");
    assert!(r["max"].as_f64().unwrap() <= 0.2, "{r}");
    assert!(
        s.execute("surface.patch", json!({"sides": sides, "mesh": "scan", "max_flatten_triangles": 400001})).is_err()
    );
}

#[test]
fn when_the_plane_fallback_fails_the_error_says_the_region_was_not_flattened() {
    // 300 degrees of a cylinder: it flattens by unrolling, but its outline projected on a plane crosses itself
    let wrap = |u: f64, v: f64| {
        let t = 300.0_f64.to_radians() * u;
        [20.0 * t.cos(), 20.0 * t.sin(), 10.0 * v]
    };
    let (mesh, sides) = grid_scan(60, 6, &wrap);
    let mut s = Session::new();
    s.doc.add_mesh("scan", mesh);
    let ok = s.execute("surface.patch", json!({"sides": sides, "mesh": "scan"})).unwrap();
    assert_eq!(ok["parameters"], "flattened", "{ok}");
    let before = s.doc.surfaces.len();
    let e = s
        .execute("surface.patch", json!({"sides": sides, "mesh": "scan", "max_flatten_triangles": 100}))
        .unwrap_err()
        .to_string();
    assert!(e.contains("height field") && e.contains("at most 100 are flattened"), "{e}");
    assert_eq!(s.doc.surfaces.len(), before);
}
