//! P8: whole section chains fitted into constrained sketches (`sketch.fit_chains`).
//!
//! Synthetic sections of extruded profiles: the profile ring carries Gaussian noise of
//! σ = 0.02 mm in the plane and is turned by 0.7 degrees; the section plane cuts the walls at
//! mid height. Gate: 4 lines + 4 arcs for a rounded rectangle, horizontal and vertical lines,
//! radii 5 within 0.05 and equal, tangency present, never over-defined; a slot; a D shape; an
//! open chain stays open.
use serde_json::{Value, json};
use std::f64::consts::TAU;
use surfacecraft_engine::Session;
use surfacecraft_geom::{Mesh, Point3, Rng};

enum Seg {
    Line([f64; 2], [f64; 2]),
    /// Centre, radius, start and end angle (degrees).
    Arc([f64; 2], f64, f64, f64),
}

/// Points of a path of segments about `step` mm apart.
fn sample(segs: &[Seg], step: f64) -> Vec<[f64; 2]> {
    let mut out = Vec::new();
    for s in segs {
        match s {
            Seg::Line(a, b) => {
                let n = ((b[0] - a[0]).hypot(b[1] - a[1]) / step).ceil().max(1.0) as usize;
                for k in 0..n {
                    let f = k as f64 / n as f64;
                    out.push([a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]);
                }
            }
            Seg::Arc(c, r, a0, a1) => {
                let n = ((a1 - a0).abs().to_radians() * r / step).ceil().max(1.0) as usize;
                for k in 0..n {
                    let a = (a0 + (a1 - a0) * k as f64 / n as f64).to_radians();
                    out.push([c[0] + r * a.cos(), c[1] + r * a.sin()]);
                }
            }
        }
    }
    out
}

/// A wall of height 10 over the profile ring (closed or open), the ring noisy and turned by
/// `deg` about the z axis, moved by `t`.
fn wall(ring: &[[f64; 2]], closed: bool, deg: f64, t: [f64; 2], sigma: f64, seed: u64) -> Mesh {
    let mut rng = Rng::new(seed);
    let (c, s) = (deg.to_radians().cos(), deg.to_radians().sin());
    let xy: Vec<[f64; 2]> = ring
        .iter()
        .map(|p| [c * p[0] - s * p[1] + t[0] + sigma * rng.gauss(), s * p[0] + c * p[1] + t[1] + sigma * rng.gauss()])
        .collect();
    let n = xy.len();
    let mut positions: Vec<Point3> = xy.iter().map(|p| Point3::new(p[0], p[1], 0.0)).collect();
    positions.extend(xy.iter().map(|p| Point3::new(p[0], p[1], 10.0)));
    let mut triangles = Vec::new();
    let quads = if closed { n } else { n - 1 };
    for i in 0..quads {
        let j = (i + 1) % n;
        let (a, b, c2, d) = (i as u32, j as u32, (n + j) as u32, (n + i) as u32);
        triangles.push([a, b, c2]);
        triangles.push([a, c2, d]);
    }
    Mesh { positions, triangles }
}

fn rounded_rect(w: f64, h: f64, r: f64) -> Vec<Seg> {
    vec![
        Seg::Line([r, 0.0], [w - r, 0.0]),
        Seg::Arc([w - r, r], r, -90.0, 0.0),
        Seg::Line([w, r], [w, h - r]),
        Seg::Arc([w - r, h - r], r, 0.0, 90.0),
        Seg::Line([w - r, h], [r, h]),
        Seg::Arc([r, h - r], r, 90.0, 180.0),
        Seg::Line([0.0, h - r], [0.0, r]),
        Seg::Arc([r, r], r, 180.0, 270.0),
    ]
}

/// Adds the wall to a session and fits the section at mid height.
fn fit(ring: &[[f64; 2]], closed: bool, seed: u64, extra: Value) -> (Session, Value) {
    let mut s = Session::new();
    s.doc.add_mesh("scan", wall(ring, closed, 0.7, [12.3, -4.5], 0.02, seed));
    let mut params =
        json!({"mesh": "scan", "origin": [0, 0, 5], "normal": [0, 0, 1], "noise_sigma": 0.02, "name": "sec"});
    if let (Some(p), Some(e)) = (params.as_object_mut(), extra.as_object()) {
        p.extend(e.clone());
    }
    let r = s.execute("sketch.fit_chains", params).unwrap();
    (s, r)
}

fn kept(chain: &Value, kind: &str) -> usize {
    chain["relations_kept"].as_array().unwrap().iter().filter(|r| r["kind"] == kind).count()
}

/// Radii of the arcs of the sketch (centre to start point).
fn radii(s: &Session, name: &str) -> Vec<f64> {
    let sk = &s.doc.sketches.iter().find(|o| o.name == name).unwrap().sketch;
    sk.entities
        .iter()
        .filter_map(|e| match e {
            surfacecraft_sketch::Entity::Arc { center, a, .. } => {
                let (c, p) = (sk.points[*center], sk.points[*a]);
                Some((p[0] - c[0]).hypot(p[1] - c[1]))
            }
            _ => None,
        })
        .collect()
}

#[test]
fn gate_p8_rounded_rectangle_section() {
    let ring = sample(&rounded_rect(40.0, 20.0, 5.0), 0.5);
    for seed in [1, 2, 3] {
        let (s, r) = fit(&ring, true, seed, json!({}));
        let c = &r["chains"][0];
        assert_eq!(
            (c["lines"].as_u64(), c["arcs"].as_u64(), c["circles"].as_u64()),
            (Some(4), Some(4), Some(0)),
            "{r}"
        );
        assert_eq!(c["closed"], true);
        assert_eq!(kept(c, "horizontal"), 2, "{c}");
        assert_eq!(kept(c, "vertical"), 2, "{c}");
        assert_eq!(kept(c, "tangent"), 8, "{c}");
        assert_eq!(kept(c, "equal_radius"), 3, "{c}");
        assert_ne!(c["status"], "over_defined");
        assert_ne!(r["sketch"]["report"]["status"], "over_defined", "{r}");
        assert_eq!(r["sketch"]["profiles"], 1);
        let rad = radii(&s, "sec");
        assert_eq!(rad.len(), 4);
        for x in &rad {
            assert!((x - 5.0).abs() < 0.05, "{rad:?}");
            assert!((x - rad[0]).abs() < 1e-6, "radii equal: {rad:?}");
        }
        // the frame was turned by the scan's 0.7 degrees (any multiple of 90 is the same frame)
        let f = r["frame_rotation_degrees"].as_f64().unwrap();
        assert!((f.rem_euclid(90.0) - 0.7).abs() < 0.15 || (f.rem_euclid(90.0) - 0.7 - 90.0).abs() < 0.15, "{f}");
        // deviation within the tolerance, before and after
        for k in ["before", "after"] {
            assert!(c[k]["rms"].as_f64().unwrap() <= 0.05 && c[k]["max"].as_f64().unwrap() <= 0.15, "{k}: {c}");
        }
        // lines are exactly horizontal or vertical in the (turned) sketch frame
        let sk = &s.doc.sketches.iter().find(|o| o.name == "sec").unwrap().sketch;
        for e in &sk.entities {
            if let surfacecraft_sketch::Entity::Line { a, b } = e {
                let d = [sk.points[*b][0] - sk.points[*a][0], sk.points[*b][1] - sk.points[*a][1]];
                assert!(d[0].abs().min(d[1].abs()) < 1e-6, "{d:?}");
            }
        }
    }
}

#[test]
fn gate_p8_slot_section() {
    // two lines and two half circles
    let slot = [
        Seg::Line([-15.0, -5.0], [15.0, -5.0]),
        Seg::Arc([15.0, 0.0], 5.0, -90.0, 90.0),
        Seg::Line([15.0, 5.0], [-15.0, 5.0]),
        Seg::Arc([-15.0, 0.0], 5.0, 90.0, 270.0),
    ];
    let (s, r) = fit(&sample(&slot, 0.5), true, 4, json!({}));
    let c = &r["chains"][0];
    assert_eq!((c["lines"].as_u64(), c["arcs"].as_u64()), (Some(2), Some(2)), "{r}");
    assert_eq!(kept(c, "tangent"), 4, "{c}");
    assert_eq!(kept(c, "horizontal") + kept(c, "vertical"), 2);
    assert_ne!(c["status"], "over_defined");
    assert_eq!(r["sketch"]["profiles"], 1);
    for x in radii(&s, "sec") {
        assert!((x - 5.0).abs() < 0.05, "{x}");
    }
}

#[test]
fn gate_p8_d_shape_section() {
    // an arc of 253.7 degrees and its vertical chord
    let a0: f64 = 90.0 + 53.13;
    let a1: f64 = a0 + 253.74;
    let (x1, y1) = (10.0 * a1.to_radians().cos(), 10.0 * a1.to_radians().sin());
    let (x0, y0) = (10.0 * a0.to_radians().cos(), 10.0 * a0.to_radians().sin());
    let d = [Seg::Arc([0.0, 0.0], 10.0, a0, a1), Seg::Line([x1, y1], [x0, y0])];
    let (s, r) = fit(&sample(&d, 0.5), true, 5, json!({}));
    let c = &r["chains"][0];
    assert_eq!((c["lines"].as_u64(), c["arcs"].as_u64()), (Some(1), Some(1)), "{r}");
    assert_eq!(kept(c, "horizontal") + kept(c, "vertical"), 1, "{c}");
    assert_ne!(c["status"], "over_defined");
    assert_eq!(r["sketch"]["profiles"], 1);
    assert!((radii(&s, "sec")[0] - 10.0).abs() < 0.05);
}

#[test]
fn gate_p8_open_chain_stays_open() {
    // a line, a quarter arc and a line, as an open wall
    let l = [
        Seg::Line([0.0, 0.0], [20.0, 0.0]),
        Seg::Arc([20.0, 5.0], 5.0, -90.0, 0.0),
        Seg::Line([25.0, 5.0], [25.0, 25.0]),
    ];
    let (s, r) = fit(&sample(&l, 0.5), false, 6, json!({}));
    let c = &r["chains"][0];
    assert_eq!(c["closed"], false, "{r}");
    assert_eq!((c["lines"].as_u64(), c["arcs"].as_u64()), (Some(2), Some(1)), "{r}");
    assert_eq!(kept(c, "tangent"), 2, "{c}");
    assert_eq!(r["sketch"]["profiles"], 0, "an open chain closes no profile");
    let sk = &s.doc.sketches.iter().find(|o| o.name == "sec").unwrap().sketch;
    assert_eq!(sk.points.len(), 5, "two lines and an arc share their junctions; one centre");
    assert_ne!(c["status"], "over_defined");
}

#[test]
fn p8_two_chains_and_parameters() {
    // a plate with a round hole: an outer rounded rectangle and an inner circle
    let mut s = Session::new();
    let outer = sample(&rounded_rect(40.0, 20.0, 5.0), 0.5);
    let hole: Vec<[f64; 2]> = (0..80)
        .map(|k| {
            let t = f64::from(k) / 80.0 * TAU;
            [20.0 + 4.0 * t.cos(), 10.0 + 4.0 * t.sin()]
        })
        .collect();
    // two walls in one mesh
    let mut m = wall(&outer, true, 0.0, [0.0, 0.0], 0.01, 7);
    let h = wall(&hole, true, 0.0, [0.0, 0.0], 0.01, 8);
    let off = m.positions.len() as u32;
    m.positions.extend(h.positions.iter().copied());
    m.triangles.extend(h.triangles.iter().map(|t| [t[0] + off, t[1] + off, t[2] + off]));
    s.doc.add_mesh("plate", m);
    let r = s.execute("sketch.fit_chains", json!({"mesh": "plate", "origin": [0, 0, 5], "normal": [0, 0, 1]})).unwrap();
    assert_eq!(r["chains"].as_array().unwrap().len(), 2, "{r}");
    assert_eq!(r["chains"][0]["entities"], 8);
    assert_eq!((r["chains"][1]["circles"].as_u64(), r["chains"][1]["entities"].as_u64()), (Some(1), Some(1)));
    assert_eq!(r["sketch"]["profiles"], 2);
    assert_ne!(r["sketch"]["report"]["status"], "over_defined");
    // a larger tolerance and no frame alignment are accepted; hostile values are errors
    s.execute(
        "sketch.fit_chains",
        json!({"mesh": "plate", "origin": [0, 0, 5], "normal": [0, 0, 1], "tolerance": 0.2, "align_frame": false}),
    )
    .unwrap();
    for bad in [
        json!({"tolerance": -1.0}),
        json!({"tolerance": 0.0}),
        json!({"noise_sigma": -0.1}),
        json!({"angle_tolerance_degrees": 0.0}),
        json!({"angle_tolerance_degrees": 90.0}),
        json!({"normal": [0, 0, 0]}),
        json!({"origin": [0, 0, 500]}),
    ] {
        let mut p = json!({"mesh": "plate", "origin": [0, 0, 5], "normal": [0, 0, 1]});
        p.as_object_mut().unwrap().extend(bad.as_object().unwrap().clone());
        assert!(s.execute("sketch.fit_chains", p).is_err(), "{bad}");
    }
}
