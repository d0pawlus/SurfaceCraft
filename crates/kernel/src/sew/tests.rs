//! Gate of O8 at the kernel: a block and a domed block, each made of six separate one-face sheets,
//! sew into closed solids of the exact volume; a box with a face missing stays open; hostile input
//! is refused.
use super::*;
use crate::nurbs::sheet_from_spline;
use crate::record::brep::Surface;

const SIZE: [f64; 3] = [40.0, 30.0, 20.0];
/// Rise of the four inner control points of the domed top (mm).
const DOME: f64 = 4.0;

/// A bicubic Bézier sheet on the parallelogram from `a` along `b - a` (u) and `d - a` (v), its four
/// inner control points raised by `rise` along +z.
fn patch(a: [f64; 3], b: [f64; 3], d: [f64; 3], rise: f64) -> Brep {
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

/// The six faces of the block `SIZE` as separate sheets, the top domed by `dome`.
fn six_sheets(dome: f64) -> Vec<Brep> {
    let [x, y, z] = SIZE;
    let c = |i: usize| -> [f64; 3] {
        [if i & 1 == 1 { x } else { 0.0 }, if i & 2 == 2 { y } else { 0.0 }, if i & 4 == 4 { z } else { 0.0 }]
    };
    vec![
        patch(c(0), c(2), c(1), 0.0),  // bottom
        patch(c(4), c(5), c(6), dome), // top
        patch(c(0), c(1), c(4), 0.0),  // front (y = 0)
        patch(c(2), c(6), c(3), 0.0),  // back (y = y)
        patch(c(0), c(4), c(2), 0.0),  // left (x = 0)
        patch(c(1), c(3), c(5), 0.0),  // right (x = x)
    ]
}

fn refs(v: &[Brep]) -> Vec<&Brep> {
    v.iter().collect()
}

/// The exact volume: the block, and over its top a Bézier dome whose inner control points rise by
/// `dome` (each of the 16 Bernstein products integrates to 1/16 of the square; four of them rise).
fn exact_volume(dome: f64) -> f64 {
    SIZE[0] * SIZE[1] * (SIZE[2] + 4.0 * dome / 16.0)
}

#[test]
fn gate_o8_six_separate_sheets_sew_into_a_closed_block() {
    for (dome, label) in [(0.0, "flat"), (DOME, "domed")] {
        for tolerance in [1e-6, 1e-2] {
            let sheets = six_sheets(dome);
            let sewn = sew(&refs(&sheets), tolerance).unwrap();
            let volume = crate::solid::volume(&sewn.brep).unwrap();
            let exact = exact_volume(dome);
            eprintln!(
                "o8 {label} block at {tolerance:e} mm: closed {}, open edges {}, faces {}, volume {volume} (exact {exact}, relative error {:e})",
                sewn.closed,
                sewn.open_edges,
                sewn.brep.faces.len(),
                (volume - exact).abs() / exact
            );
            assert!(sewn.closed && sewn.open_edges == 0, "{label}");
            assert_eq!(sewn.brep.faces.len(), 6, "{label}");
            assert!(crate::nurbs::is_closed(&sewn.brep), "{label}");
            // the M3 bound (0.2 mm) over the whole surface is far looser than this
            assert!((volume - exact).abs() <= 1e-6 * exact, "{label}: {volume} vs {exact}");
        }
    }
}

#[test]
fn five_sheets_sew_into_an_open_body() {
    let mut sheets = six_sheets(DOME);
    sheets.remove(1);
    let sewn = sew(&refs(&sheets), 1e-3).unwrap();
    assert!(!sewn.closed);
    assert_eq!(sewn.open_edges, 4, "the rim of the missing top");
    assert_eq!(sewn.brep.faces.len(), 5);
}

#[test]
fn hostile_input_is_refused_not_a_panic() {
    let sheets = six_sheets(0.0);
    let all = refs(&sheets);
    assert!(sew(&[], 1e-3).is_err());
    for t in [0.0, -1.0, f64::NAN, f64::INFINITY, MAX_TOLERANCE_MM * 2.0] {
        assert!(matches!(sew(&all, t), Err(KernelError::Argument(_))), "{t}");
    }
    let mut broken = sheets[0].clone();
    broken.faces.clear();
    let _ = sew(&[&broken, all[1]], 1e-3);
    let mut nan = sheets[0].clone();
    nan.vertices[0].point[0] = f64::NAN;
    let _ = sew(&[&nan, all[1]], 1e-3);
}
