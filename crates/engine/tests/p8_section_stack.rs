//! P8 gate: section stacks (`section.stack`) and section sheets (`export.section_sheet`).
//! A cone frustum has the draft of atan(4/20) on every side and a linear radius trend; a box is
//! prismatic with zero draft; the DXF of a square section parses back to four segments of the
//! right length; the SVG of it holds one closed path.
use serde_json::{Value, json};
use std::f64::consts::PI;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Vec3, synth};

/// A session holding `mesh` under the name `part`.
fn session_with(mesh: Mesh) -> Session {
    let mut s = Session::new();
    let _ = s.doc.add_mesh("part", mesh);
    s
}

/// Lengths of the segments of a DXF text, for the subset the writer emits: one per LINE, and one
/// per edge of an LWPOLYLINE, the closing edge included when group 70 has bit 1 set.
fn dxf_segment_lengths(text: &str) -> Vec<f64> {
    let lines: Vec<&str> = text.lines().collect();
    let mut segs = Vec::new();
    let mut in_entities = false;
    let mut kind = String::new();
    let mut xs: Vec<f64> = Vec::new();
    let mut ys: Vec<f64> = Vec::new();
    let mut flag = 0u32;
    for pair in lines.chunks(2) {
        let [code, value] = pair else { continue };
        let Ok(code) = code.trim().parse::<u32>() else { continue };
        let value = value.trim();
        let num = value.parse::<f64>().ok();
        match (code, num) {
            (2, _) if value == "ENTITIES" => in_entities = true,
            (0, _) if in_entities => {
                // a new entity, or the end of the section, closes the previous one
                segs.extend(segment_lengths(&kind, &xs, &ys, flag));
                kind = value.to_string();
                xs.clear();
                ys.clear();
                flag = 0;
                if value == "ENDSEC" {
                    in_entities = false;
                }
            }
            (10 | 11, Some(v)) if in_entities => xs.push(v),
            (20 | 21, Some(v)) if in_entities => ys.push(v),
            (70, Some(v)) if in_entities => flag = v as u32,
            _ => {}
        }
    }
    segs
}

/// The edge lengths of one entity: `xs` and `ys` are its points in order.
fn segment_lengths(kind: &str, xs: &[f64], ys: &[f64], flag: u32) -> Vec<f64> {
    let pts: Vec<(f64, f64)> = xs.iter().copied().zip(ys.iter().copied()).collect();
    let dist = |a: (f64, f64), b: (f64, f64)| (b.0 - a.0).hypot(b.1 - a.1);
    let mut out: Vec<f64> = pts.windows(2).map(|w| dist(w[0], w[1])).collect();
    if let (true, Some(&first), Some(&last)) = (kind == "LWPOLYLINE" && (flag & 1) == 1, pts.first(), pts.last()) {
        out.push(dist(last, first));
    }
    out
}

/// The draft of a cone frustum (radius 10 at z = 0, radius 6 at z = 20) is atan(4 / 20).
#[test]
fn gate_p8_cone_draft_and_radius_taper() {
    let mut s = session_with(synth::cone(Point3::origin(), Vec3::z(), 10.0, 6.0, 20.0, 96, 20));
    let r = s.execute("section.stack", json!({"mesh": "part", "start": 0.5, "step": 1.0, "count": 18})).unwrap();
    let want = 0.2_f64.atan().to_degrees();
    let sides = r["trend"]["sides"].as_array().unwrap();
    assert_eq!(sides.len(), 4, "{r}");
    for side in sides {
        let draft = side["draft_deg"].as_f64().unwrap();
        assert!((draft - want).abs() < 0.2, "{side}");
    }
    assert_eq!(r["trend"]["profile"], "tapered", "{r}");
    assert_eq!(r["trend"]["prismatic"], false);
    let radius = &r["trend"]["radius"];
    assert!((radius["taper_deg"].as_f64().unwrap() - want).abs() < 0.2, "{radius}");
    assert!((radius["slope"].as_f64().unwrap() + 0.2).abs() < 0.002, "{radius}");
    assert!((radius["radius_at_zero"].as_f64().unwrap() - 10.0).abs() < 0.05, "{radius}");
    assert!(radius["rms"].as_f64().unwrap() < 0.01, "{radius}");
    let levels = r["levels"].as_array().unwrap();
    assert_eq!(levels.len(), 18);
    for level in levels {
        let z = level["offset"].as_f64().unwrap();
        let rad = 10.0 - 0.2 * z;
        assert_eq!(level["closed"].as_u64(), Some(1), "{level}");
        assert_eq!(level["open"].as_u64(), Some(0), "{level}");
        let circle = &level["circles"][0];
        assert!((circle["radius"].as_f64().unwrap() - rad).abs() < 0.02, "{level}");
        let area = level["area"].as_f64().unwrap();
        assert!((area - PI * rad * rad).abs() / (PI * rad * rad) < 0.005, "{level}");
    }
}

/// A 10 × 10 × 20 box cut every 2.1 mm: the same chain, the same area and no draft, so prismatic.
#[test]
fn gate_p8_box_stack_is_prismatic() {
    let mut s = session_with(synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 20.0), 4));
    let r = s.execute("section.stack", json!({"mesh": "part", "start": 1.3, "step": 2.1, "count": 8})).unwrap();
    for side in r["trend"]["sides"].as_array().unwrap() {
        assert!(side["draft_deg"].as_f64().unwrap().abs() < 0.05, "{side}");
    }
    assert_eq!(r["trend"]["profile"], "prismatic", "{r}");
    assert_eq!(r["trend"]["prismatic"], true);
    for level in r["levels"].as_array().unwrap() {
        let z = level["offset"].as_f64().unwrap();
        assert_eq!(level["closed"].as_u64(), Some(1), "{level}");
        assert!((level["area"].as_f64().unwrap() - 100.0).abs() < 1e-6, "{level}");
        assert!((level["size"][0].as_f64().unwrap() - 10.0).abs() < 1e-6, "{level}");
        assert!((level["size"][1].as_f64().unwrap() - 10.0).abs() < 1e-6, "{level}");
        let c = &level["centroid"];
        assert!((c[0].as_f64().unwrap() - 5.0).abs() < 1e-6, "{level}");
        assert!((c[1].as_f64().unwrap() - 5.0).abs() < 1e-6, "{level}");
        assert!((c[2].as_f64().unwrap() - z).abs() < 1e-6, "{level}");
    }
}

/// A 10 mm square section (a 10 mm box cut at mid-height) written as DXF parses back to four
/// segments of 10 mm each.
#[test]
fn gate_p8_square_section_dxf_parses_back_to_four_segments() {
    let mut s = session_with(synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 10.0), 1));
    let r =
        s.execute("export.section_sheet", json!({"mesh": "part", "origin": [0.0, 0.0, 5.0], "format": "dxf"})).unwrap();
    assert_eq!(r["chains"].as_u64(), Some(1), "{r}");
    assert_eq!(r["levels"].as_u64(), Some(1), "{r}");
    assert!(r.get("path").is_none(), "{r}");
    let text = r["text"].as_str().unwrap();
    let lengths = dxf_segment_lengths(text);
    assert_eq!(lengths.len(), 4, "{text}");
    for len in lengths {
        assert!((len - 10.0).abs() < 1e-6, "{len}");
    }
}

/// The same square as SVG: one path, closed, in a viewBox in millimetres.
#[test]
fn gate_p8_square_section_svg_has_one_closed_path() {
    let mut s = session_with(synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 10.0), 1));
    let r =
        s.execute("export.section_sheet", json!({"mesh": "part", "origin": [0.0, 0.0, 5.0], "format": "svg"})).unwrap();
    let text = r["text"].as_str().unwrap();
    assert_eq!(text.matches("<path").count(), 1, "{text}");
    assert_eq!(text.matches(" Z").count(), 1, "{text}");
    assert!(text.contains("viewBox="), "{text}");
}

/// Hostile parameters are refused with an error, never a panic.
#[test]
fn gate_p8_bad_parameters_are_refused() {
    let mut s = session_with(synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 10.0), 1));
    let bad_stacks: [Value; 4] = [
        json!({"mesh": "part", "count": 0}),
        json!({"mesh": "part", "count": 3, "step": 0.0}),
        json!({"mesh": "part", "normal": [0.0, 0.0, 0.0]}),
        json!({"mesh": "part", "count": 100000, "step": 1.0}),
    ];
    for bad in bad_stacks {
        assert!(s.execute("section.stack", bad.clone()).is_err(), "{bad}");
    }
    assert!(s.execute("section.stack", json!({"mesh": "missing"})).is_err());
    assert!(s.execute("export.section_sheet", json!({"mesh": "part", "format": "pdf"})).is_err());
    assert!(s.execute("export.section_sheet", json!({"mesh": "part", "tolerance": -1.0})).is_err());
}
