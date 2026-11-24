//! `surface.sew` (P9-O8): separate sheets sewn into one body. Gate: six `surface.patch` patches over
//! the faces of a block close into a solid of the block's volume (tolerance from the M3 bound of
//! design doc §11.1, 0.2 mm over the whole surface, never loosened); exact Bézier sheets, the top
//! domed, give the exact volume; hostile parameters are errors, never panics.
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;
use surfacecraft_engine::Session;
use surfacecraft_kernel::nurbs::sheet_from_spline;
use surfacecraft_kernel::record::brep::{Brep, Surface};

const SIZE: [f64; 3] = [40.0, 30.0, 20.0];
/// Rise of the four inner control points of the domed top (mm).
const DOME: f64 = 4.0;
/// The M3 bound of design doc §11.1 (mm).
const M3_MAX_DEVIATION_MM: f64 = 0.2;

fn corner(i: usize) -> [f64; 3] {
    [
        if i & 1 == 1 { SIZE[0] } else { 0.0 },
        if i & 2 == 2 { SIZE[1] } else { 0.0 },
        if i & 4 == 4 { SIZE[2] } else { 0.0 },
    ]
}

/// The block's faces as loops of corner indices.
const FACES: [[usize; 4]; 6] = [[0, 2, 3, 1], [4, 5, 7, 6], [0, 1, 5, 4], [2, 6, 7, 3], [0, 4, 6, 2], [1, 3, 7, 5]];

fn area() -> f64 {
    2.0 * (SIZE[0] * SIZE[1] + SIZE[0] * SIZE[2] + SIZE[1] * SIZE[2])
}

/// One `surface.patch` per face of the block, each over four polyline sides; their ids.
fn patched_block(s: &mut Session) -> Vec<u64> {
    FACES
        .iter()
        .enumerate()
        .map(|(f, quad)| {
            let sides: Vec<Value> =
                (0..4).map(|k| json!({"points": [corner(quad[k]), corner(quad[(k + 1) % 4])]})).collect();
            let r = s.execute("surface.patch", json!({"sides": sides, "name": format!("face {f}")})).unwrap();
            r["surface"].as_u64().unwrap()
        })
        .collect()
}

#[test]
fn gate_o8_six_patches_of_a_block_sew_into_its_solid() {
    let mut s = Session::new();
    let ids = patched_block(&mut s);
    let r = s.execute("surface.sew", json!({"objects": ids, "name": "block"})).unwrap();
    let exact = SIZE[0] * SIZE[1] * SIZE[2];
    eprintln!("o8 patched block: {r}");
    assert_eq!(r["closed"], true, "{r}");
    assert_eq!(r["open_edges"], 0, "{r}");
    let volume = r["volume"].as_f64().unwrap();
    // the patches may lie anywhere within the M3 bound of the true faces
    let allowed = M3_MAX_DEVIATION_MM * area();
    eprintln!(
        "o8 patched block: volume {volume}, exact {exact}, error {:e} (allowed {allowed})",
        (volume - exact).abs()
    );
    assert!((volume - exact).abs() <= allowed, "{volume} vs {exact}");
    let body = s.doc.body(ObjectId(r["id"].as_u64().unwrap())).unwrap();
    assert_eq!(body.name, "block");
    assert!(surfacecraft_kernel::nurbs::is_closed(&body.brep));
    // one undo step takes the body away again
    assert_eq!(s.doc.bodies.len(), 1);
    s.execute("edit.undo", Value::Null).unwrap();
    assert!(s.doc.bodies.is_empty());
    assert_eq!(s.doc.surfaces.len(), 6, "the patches stay");
}

#[test]
fn five_patches_sew_into_an_open_sheet() {
    let mut s = Session::new();
    let mut ids = patched_block(&mut s);
    ids.remove(1);
    let r = s.execute("surface.sew", json!({"objects": ids})).unwrap();
    assert_eq!(r["closed"], false, "{r}");
    assert!(r["open_edges"].as_u64().unwrap() > 0, "{r}");
    assert!(r["volume"].is_null(), "{r}");
    // a sheet body can be sewn again, with the missing patch
    let all = s.doc.surfaces.iter().map(|o| o.id.0).collect::<Vec<_>>();
    let r2 = s.execute("surface.sew", json!({"objects": [r["id"], all[1]]})).unwrap();
    assert_eq!(r2["closed"], true, "{r2}");
}

/// A bicubic Bézier sheet on the parallelogram from `a` along `b - a` and `d - a`, its four inner
/// control points raised by `rise` along +z.
fn bezier(a: [f64; 3], b: [f64; 3], d: [f64; 3], rise: f64) -> Brep {
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let points = (0..4)
        .map(|i| {
            (0..4)
                .map(|j| {
                    let (u, v) = (f64::from(i) / 3.0, f64::from(j) / 3.0);
                    let mut p: [f64; 3] = std::array::from_fn(|k| a[k] + u * (b[k] - a[k]) + v * (d[k] - a[k]));
                    if (1..=2).contains(&i) && (1..=2).contains(&j) {
                        p[2] += rise;
                    }
                    p
                })
                .collect()
        })
        .collect();
    sheet_from_spline(&Surface::Spline { u_knots: knots.clone(), v_knots: knots, points }).unwrap()
}

#[test]
fn exact_sheet_bodies_of_a_domed_block_sew_into_the_exact_volume() {
    let mut s = Session::new();
    let ids: Vec<u64> = FACES
        .iter()
        .enumerate()
        .map(|(f, q)| {
            let rise = if f == 1 { DOME } else { 0.0 };
            s.doc.add_body(&format!("sheet {f}"), bezier(corner(q[0]), corner(q[1]), corner(q[3]), rise)).0
        })
        .collect();
    let r = s.execute("surface.sew", json!({"objects": ids, "tolerance": 1e-4})).unwrap();
    // the block and the dome: four of the sixteen Bernstein products rise, each 1/16 of the top
    let exact = SIZE[0] * SIZE[1] * (SIZE[2] + DOME / 4.0);
    let volume = r["volume"].as_f64().unwrap();
    eprintln!("o8 domed sheets: {r}; exact {exact}");
    assert_eq!(r["closed"], true, "{r}");
    assert!((volume - exact).abs() <= 1e-6 * exact, "{volume} vs {exact}");
}

#[test]
fn hostile_parameters_are_errors_not_panics() {
    let mut s = Session::new();
    let ids = patched_block(&mut s);
    s.doc.add_body("solid", surfacecraft_kernel::solid::make_box([0.0; 3], [1.0; 3]).unwrap());
    s.doc.add_mesh("scan", surfacecraft_geom::synth::wave(surfacecraft_geom::Point3::origin(), 10.0, 10.0, 1.0, 5, 5));
    for bad in [
        json!({}),
        json!({"objects": []}),
        json!({"objects": ["solid"]}),
        json!({"objects": ["scan"]}),
        json!({"objects": [99_999]}),
        json!({"objects": ["no such object"]}),
        json!({"objects": ids, "tolerance": 0.0}),
        json!({"objects": ids, "tolerance": -1.0}),
        json!({"objects": ids, "tolerance": 2.0}),
        json!({"objects": ids, "tolerance": 1e308}),
        json!({"objects": ids, "unknown": 1}),
        json!({"objects": "face 0"}),
    ] {
        assert!(s.execute("surface.sew", bad.clone()).is_err(), "{bad}");
    }
    assert!(s.doc.bodies.len() == 1, "nothing made by a refused call");
    // the same object twice is sewn once
    let r = s.execute("surface.sew", json!({"objects": [ids[0], ids[0]]})).unwrap();
    assert_eq!(r["faces"], 1, "{r}");
}
