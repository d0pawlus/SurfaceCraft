//! P9-O5: `export.step` of a closed multi-patch solid is compact and round-trips through
//! `import.step`.
use serde_json::json;
use std::collections::HashMap;
use surfacecraft_doc::base64;
use surfacecraft_engine::Session;
use surfacecraft_kernel::nurbs::sheet_from_bezier_network_as_is;
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::solid;

/// Bytes allowed per bicubic patch.
const MAX_BYTES_PER_PATCH: usize = 6 * 1024;
const DEGREE: usize = 3;

fn shape_point(k: [usize; 3], steps: usize) -> [f64; 3] {
    let p = k.map(|c| -1.0 + 2.0 * c as f64 / steps as f64);
    let len = p.iter().map(|c| c * c).sum::<f64>().sqrt();
    let d = p.map(|c| c / len);
    let radius = 10.0 * (1.0 + 0.09 * (3.0 * d[0]).sin() * (2.0 * d[1]).cos() + 0.06 * (2.0 * d[2] + 1.2 * d[0]).sin());
    d.map(|c| c * radius)
}

/// A closed cube-sphere of `6 n²` bicubic patches with outward normals.
fn closed_network(n: usize) -> Brep {
    let steps = DEGREE * n;
    let mut ids: HashMap<[usize; 3], u32> = HashMap::new();
    let (mut patches, mut corners) = (Vec::new(), Vec::new());
    for axis in 0..3 {
        for side in [0, 1] {
            let (b, c) = ((axis + 1) % 3, (axis + 2) % 3);
            let (ua, va) = if side == 1 { (b, c) } else { (c, b) };
            for pi in 0..n {
                for pj in 0..n {
                    let at = |i: usize, j: usize, scale: usize| {
                        let mut k = [0; 3];
                        k[axis] = side * scale * n;
                        k[ua] = scale * pi + i;
                        k[va] = scale * pj + j;
                        k
                    };
                    let net = (0..=DEGREE)
                        .map(|i| (0..=DEGREE).map(|j| shape_point(at(i, j, DEGREE), steps)).collect())
                        .collect();
                    corners.push([(0, 0), (1, 0), (1, 1), (0, 1)].map(|(i, j)| {
                        let next = ids.len() as u32;
                        *ids.entry(at(i, j, 1)).or_insert(next)
                    }));
                    patches.push(net);
                }
            }
        }
    }
    sheet_from_bezier_network_as_is(&patches, &corners).unwrap()
}

#[test]
fn export_step_of_a_closed_patch_network_is_compact_and_round_trips() {
    let net = closed_network(7);
    let patches = net.faces.len();
    let volume = solid::volume(&net).unwrap();
    let mut s = Session::new();
    let id = s.doc.add_body("blob", net);
    let exp = s.execute("export.step", json!({"objects": [id]})).unwrap();
    assert_eq!(exp["content"], "solids");
    let bytes = exp["bytes"].as_u64().unwrap() as usize;
    eprintln!("export.step: {patches} patches, {bytes} bytes, {} B/patch", bytes / patches);
    assert!(bytes / patches <= MAX_BYTES_PER_PATCH, "{} bytes per patch", bytes / patches);
    let text = String::from_utf8(base64::decode(exp["data"].as_str().unwrap()).unwrap()).unwrap();
    assert!(text.contains("MANIFOLD_SOLID_BREP") && text.contains("CLOSED_SHELL"));

    let imp = s.execute("import.step", json!({"data": exp["data"]})).unwrap();
    assert_eq!(imp["faces"], json!([patches]));
    let back = s.doc.body(serde_json::from_value(imp["bodies"][0].clone()).unwrap()).unwrap();
    let err = (solid::volume(&back.brep).unwrap() - volume).abs() / volume;
    eprintln!("export.step round trip: relative volume error {err:e}");
    assert!(err <= 1e-6, "{err}");
}

#[test]
fn export_step_of_primitive_solids_still_round_trips() {
    let mut s = Session::new();
    let id = s.doc.add_body("box", solid::make_box([0.0; 3], [2.0, 3.0, 4.0]).unwrap());
    let exp = s.execute("export.step", json!({"objects": [id]})).unwrap();
    let imp = s.execute("import.step", json!({"data": exp["data"]})).unwrap();
    assert_eq!(imp["faces"], json!([6]));
}
