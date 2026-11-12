//! A closed bicubic patch network for the size and round-trip gates: a cube-sphere of `6 n²`
//! patches whose radius varies smoothly, so no face is planar.

use std::collections::HashMap;
use surfacecraft_kernel::nurbs::sheet_from_bezier_network_as_is;
use surfacecraft_kernel::record::brep::Brep;

const DEGREE: usize = 3;

fn radius(d: [f64; 3]) -> f64 {
    10.0 * (1.0 + 0.09 * (3.0 * d[0]).sin() * (2.0 * d[1]).cos() + 0.06 * (2.0 * d[2] + 1.2 * d[0]).sin())
}

/// The shape's point for integer cube-surface coordinates `k` (each in `0..=3 n`).
fn shape_point(k: [usize; 3], steps: usize) -> [f64; 3] {
    let p = k.map(|c| -1.0 + 2.0 * c as f64 / steps as f64);
    let len = p.iter().map(|c| c * c).sum::<f64>().sqrt();
    let d = p.map(|c| c / len);
    d.map(|c| c * radius(d))
}

/// A closed network of `6 n²` bicubic patches (`n >= 1`).
pub(super) fn closed_network(n: usize) -> Brep {
    let steps = DEGREE * n;
    let mut ids: HashMap<[usize; 3], u32> = HashMap::new();
    let (mut patches, mut corners) = (Vec::new(), Vec::new());
    for axis in 0..3 {
        for side in [0, 1] {
            let (b, c) = ((axis + 1) % 3, (axis + 2) % 3);
            // u × v must point outward: swap the directions on the negative side
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
                    let cell = [(0, 0), (1, 0), (1, 1), (0, 1)].map(|(i, j)| at(i, j, 1));
                    let corner = cell.map(|k| {
                        let next = ids.len() as u32;
                        *ids.entry(k).or_insert(next)
                    });
                    patches.push(net);
                    corners.push(corner);
                }
            }
        }
    }
    sheet_from_bezier_network_as_is(&patches, &corners).expect("fixture network is consistent")
}
