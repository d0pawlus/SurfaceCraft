//! P9-M3: `align.global`, several scans of one part into one common pose at once.
mod pair_common;
use pair_common::*;
use serde_json::{Value, json};
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Transform, Vec3};

/// A view of the bracket: its own tessellation and noise, the triangles on the side of `dir`
/// (centroid within `reach` mm of the part centre against it), in a random pose of its own.
fn bracket_view(k: usize, dir: Vec3, reach: f64) -> Mesh {
    let whole = noisy(bracket_at(10 + 2 * k, 40 + 6 * k, 14 + k), 100 + k as u64);
    let c = Point3::new(30.0, 20.0, 15.0);
    cut(&whole, |p| (p - c).dot(&dir) >= -reach)
}

fn bracket_views() -> Vec<Mesh> {
    let dirs = [
        (Vec3::new(1.0, 0.0, 0.0), 6.0),
        (Vec3::new(-1.0, 0.0, 0.0), 6.0),
        (Vec3::new(0.0, 1.0, 0.0), 4.0),
        (Vec3::new(0.0, -1.0, 0.0), 4.0),
        (Vec3::new(0.0, 0.0, 1.0), 6.0),
        (Vec3::new(1.0, 1.0, 1.0).normalize(), 7.0),
    ];
    dirs.iter().enumerate().map(|(k, (d, r))| bracket_view(k, *d, *r)).collect()
}

fn session_of(views: &[Mesh], poses: &[Transform]) -> Session {
    let mut s = Session::new();
    for (k, (v, p)) in views.iter().zip(poses).enumerate() {
        s.doc.add_mesh(&format!("v{k}"), v.transformed(p));
    }
    s
}

fn global(s: &mut Session, n: usize, extra: Value) -> Value {
    let names: Vec<String> = (0..n).map(|k| format!("v{k}")).collect();
    let mut p = json!({ "objects": names });
    p.as_object_mut().unwrap().extend(extra.as_object().unwrap().clone());
    s.execute("align.global", p).unwrap()
}

#[test]
fn explore_bracket_views() {
    let views = bracket_views();
    let poses: Vec<Transform> = (0..views.len() as u64).map(random_motion).collect();
    let mut s = session_of(&views, &poses);
    let t = std::time::Instant::now();
    let out = global(&mut s, views.len(), json!({}));
    eprintln!("time {:?} verdict {} issues {} loop {}", t.elapsed(), out["verdict"], out["issues"], out["loop"]);
    for e in out["edges"].as_array().unwrap() {
        let (a, b) = (e["fixed"].as_u64().unwrap() as usize, e["moving"].as_u64().unwrap() as usize);
        // b -> a truth: P_a * P_b^-1
        let truth = poses[a].then_after(&poses[b].inverse());
        let on = views[b].transformed(&poses[b]);
        let found = Transform::from_rows(&serde_json::from_value(e["transform"].clone()).unwrap()).unwrap();
        let err = found.max_displacement(&truth, &on.positions);
        eprintln!("   true overlap {:.2}/{:.2} pair error {err:.3} mm", true_share(&views[b], &views[a], 0.3), true_share(&views[a], &views[b], 0.3));
        eprintln!(
            "{}-{}: {} {} conf {:.2} ov {:.2}/{:.2} rms {:.3} tol {:.3} after {}",
            e["fixed"], e["moving"], e["verdict"], e["status"], e["confidence"].as_f64().unwrap(),
            e["overlap_moving"].as_f64().unwrap(), e["overlap_fixed"].as_f64().unwrap(),
            e["rms"].as_f64().unwrap(), e["tolerance"].as_f64().unwrap(), e["misfit_after"]
        );
    }
    for (k, sc) in out["scans"].as_array().unwrap().iter().enumerate() {
        let found = matrix_of(sc);
        let truth = poses[0].then_after(&poses[k].inverse());
        let on = views[k].transformed(&poses[k]);
        let err = found.max_displacement(&truth, &on.positions);
        eprintln!("scan {k}: placed {} err {err:.4} mm", sc["placed"]);
    }
}
