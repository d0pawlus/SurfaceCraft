//! `mesh.import` with `reduce_to`: a scan with more triangles than wanted is reduced while it is
//! read, and the result says what the reduction did.
use serde_json::{Value, json};
use surfacecraft_doc::base64;
use surfacecraft_engine::{EngineError, Session};
use surfacecraft_geom::{Mesh, Point3, Vec3, synth};
use surfacecraft_io::{MeshFormat, write_mesh};

fn torus() -> Mesh {
    let mut m = synth::torus(Point3::origin(), Vec3::z(), 30.0, 8.0, 240, 120);
    m.weld(1e-9);
    m
}

/// Extent of the mesh `scan` along x.
fn width(s: &mut Session) -> f64 {
    let info = s.execute("mesh.info", json!({"object": "scan"})).unwrap();
    info["bbox_max"][0].as_f64().unwrap() - info["bbox_min"][0].as_f64().unwrap()
}

fn import(s: &mut Session, m: &Mesh, format: MeshFormat, extra: Value) -> Result<Value, EngineError> {
    let mut params = json!({"data": base64::encode(&write_mesh(m, format)), "format": format, "name": "scan"});
    for (k, v) in extra.as_object().unwrap() {
        params[k] = v.clone();
    }
    s.execute("mesh.import", params)
}

#[test]
fn a_scan_above_reduce_to_comes_in_reduced_and_says_so() {
    let m = torus();
    let n = m.triangle_count();
    for format in [MeshFormat::Stl, MeshFormat::Ply, MeshFormat::Obj] {
        let mut s = Session::new();
        let target = n / 6;
        let r = import(&mut s, &m, format, json!({"reduce_to": target})).unwrap();
        let got = r["triangles"].as_u64().unwrap() as usize;
        assert!(got <= target && got * 4 >= target, "{format:?}: {got} of {target}");
        assert_eq!(r["reduced"]["source_triangles"].as_u64().unwrap(), n as u64, "{format:?}: {r}");
        let max_error = r["reduced"]["max_error"].as_f64().unwrap();
        assert!(max_error > 0.0 && max_error < 8.0, "{format:?}: {max_error}");
        // the mesh in the document is the reduced one, at the size of the file
        let info = s.execute("mesh.info", json!({"object": "scan"})).unwrap();
        assert_eq!(info["triangles"].as_u64().unwrap() as usize, got);
        assert!((width(&mut s) - 76.0).abs() < 2.0 * max_error + 0.1);
    }
}

#[test]
fn a_scan_within_reduce_to_is_imported_whole() {
    let m = torus();
    let mut s = Session::new();
    let r = import(&mut s, &m, MeshFormat::Stl, json!({"reduce_to": 10_000_000})).unwrap();
    assert_eq!(r["triangles"].as_u64().unwrap() as usize, m.triangle_count());
    assert!(r.get("reduced").is_none(), "{r}");
}

#[test]
fn units_apply_to_a_reduced_import() {
    let m = torus();
    let mut s = Session::new();
    let r = import(&mut s, &m, MeshFormat::Stl, json!({"reduce_to": 5000, "units": "in"})).unwrap();
    assert_eq!(r["scale_factor"].as_f64().unwrap(), 25.4);
    assert!((width(&mut s) - 76.0 * 25.4).abs() < 150.0);
}

#[test]
fn a_reduce_to_out_of_range_is_refused_as_a_parameter_error() {
    let m = synth::sphere(Point3::origin(), 10.0, 16, 8);
    for bad in [json!(0), json!(999), json!(4_000_000_000u64)] {
        let mut s = Session::new();
        let e = import(&mut s, &m, MeshFormat::Stl, json!({"reduce_to": bad})).unwrap_err();
        assert!(matches!(e, EngineError::InvalidParams(_)), "{e:?}");
        assert!(s.doc.summaries().is_empty());
    }
}
