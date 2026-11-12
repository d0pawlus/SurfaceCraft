//! A synthetic closed network of bicubic Bézier patches for the large-network tests: a cube
//! sphere (the six faces of a cube, each cut into `n × n` quads, every quad one patch whose
//! control points lie on the sphere). Shared boundary control points are computed from the same
//! lattice point, so neighbouring patches agree exactly.
#![allow(dead_code)]

use solidcore_record::brep::{
    Brep, Curve, Curve2, Edge, EdgeUseRef, Face, Loop, OrientedEdge, OrientedFace, Region, Shell, Surface, VERSION,
    Vertex,
};
use std::collections::BTreeMap;

/// Control nets and corner ids of a closed network, as `sheet_from_bezier_network` takes them.
pub struct Network {
    pub patches: Vec<Vec<Vec<[f64; 3]>>>,
    pub corners: Vec<[u32; 4]>,
    pub radius: f64,
}

/// Patches of a cube sphere with `n` quads along each cube edge: `6 n²` patches.
pub fn cube_sphere(n: usize, radius: f64) -> Network {
    let steps = 3 * n;
    let point = |lat: [usize; 3]| -> [f64; 3] {
        let c = lat.map(|a| 2.0 * a as f64 / steps as f64 - 1.0);
        let len = (c[0] * c[0] + c[1] * c[1] + c[2] * c[2]).sqrt();
        c.map(|x| x / len * radius)
    };
    let mut ids: BTreeMap<[usize; 3], u32> = BTreeMap::new();
    let mut patches = Vec::with_capacity(6 * n * n);
    let mut corners = Vec::with_capacity(6 * n * n);
    for axis in 0..3 {
        for side in 0..2 {
            let (p, q) = ((axis + 1) % 3, (axis + 2) % 3);
            // u x v must point out of the cube: swap the tangent axes on the low side
            let (au, av) = if side == 1 { (p, q) } else { (q, p) };
            for i in 0..n {
                for j in 0..n {
                    let lattice = |m: usize, k: usize| -> [usize; 3] {
                        let mut l = [0; 3];
                        l[axis] = side * steps;
                        l[au] = 3 * i + m;
                        l[av] = 3 * j + k;
                        l
                    };
                    let net: Vec<Vec<[f64; 3]>> =
                        (0..4).map(|m| (0..4).map(|k| point(lattice(m, k))).collect()).collect();
                    let mut id = |l: [usize; 3]| {
                        let next = ids.len() as u32;
                        *ids.entry(l).or_insert(next)
                    };
                    corners.push([id(lattice(0, 0)), id(lattice(3, 0)), id(lattice(3, 3)), id(lattice(0, 3))]);
                    patches.push(net);
                }
            }
        }
    }
    Network { patches, corners, radius }
}

/// The `n` whose cube sphere has at least `count` patches.
pub fn n_for(count: usize) -> usize {
    (1..).find(|n| 6 * n * n >= count).unwrap_or(1)
}

/// Volume of a sphere.
pub fn sphere_volume(r: f64) -> f64 {
    4.0 / 3.0 * std::f64::consts::PI * r * r * r
}

/// The network as a solidcore record, assembled with `solidcore-record` types only (no
/// SurfaceCraft code), edges and vertices numbered in the canonical order of first use.
pub fn record_of(net: &Network) -> Brep {
    let knots = vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0];
    let uv = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]];
    let mut vertex_of: BTreeMap<u32, u32> = BTreeMap::new();
    let mut vertices: Vec<Vertex> = Vec::new();
    let mut edge_of: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    let mut edges: Vec<Edge> = Vec::new();
    let mut faces: Vec<Face> = Vec::new();
    for (f, (net3, c)) in net.patches.iter().zip(&net.corners).enumerate() {
        let side = |s: usize| -> Vec<[f64; 3]> {
            (0..4)
                .map(|k| match s {
                    0 => net3[k][0],
                    1 => net3[3][k],
                    2 => net3[3 - k][3],
                    _ => net3[0][3 - k],
                })
                .collect()
        };
        let mut loop_edges = Vec::new();
        for s in 0..4 {
            let (a, b) = (c[s], c[(s + 1) % 4]);
            let (lo, hi, forward) = (a.min(b), a.max(b), a < b);
            let eid = *edge_of.entry((lo, hi)).or_insert_with(|| {
                let mut pts = side(s);
                if !forward {
                    pts.reverse();
                }
                let mut ids = [0u32; 2];
                for (slot, v) in ids.iter_mut().zip([lo, hi]) {
                    let next = vertices.len() as u32;
                    *slot = *vertex_of.entry(v).or_insert_with(|| {
                        vertices.push(Vertex { point: if v == lo { pts[0] } else { pts[3] }, tolerance: 1e-6 });
                        next
                    });
                }
                edges.push(Edge {
                    curve: Curve::Spline { knots: knots.clone(), points: pts },
                    nurbs: None,
                    interval: [0.0, 1.0],
                    vertices: ids,
                    tolerance: 1e-6,
                    uses: Vec::new(),
                });
                (edges.len() - 1) as u32
            });
            edges[eid as usize].uses.push(EdgeUseRef {
                face: f as u32,
                r#loop: 0,
                position: s as u32,
                upper: !forward,
            });
            let (p0, p1) = (uv[s], uv[(s + 1) % 4]);
            let (start, end) = if forward { (p0, p1) } else { (p1, p0) };
            let pcurve = Curve2::Spline { knots: vec![0.0, 0.0, 1.0, 1.0], points: vec![start, end] };
            loop_edges.push(OrientedEdge { edge: eid, forward, pcurve });
        }
        faces.push(Face {
            surface: Surface::Spline { u_knots: knots.clone(), v_knots: knots.clone(), points: net3.clone() },
            nurbs: None,
            domain: [[0.0, 1.0], [0.0, 1.0]],
            loops: vec![Loop::Edges { edges: loop_edges }],
        });
    }
    let oriented = (0..faces.len() as u32).map(|face| OrientedFace { face, forward: true }).collect();
    Brep {
        version: VERSION,
        tolerance: 1e-6,
        regions: vec![Region { shells: vec![0], void: false }, Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: oriented, wires: vec![] }],
        faces,
        edges,
        vertices,
    }
}

/// Patch `f` turned around: its net transposed and its corners `[c0, c3, c2, c1]` (its normal
/// reversed, boundary curves unchanged).
pub fn flip_patch(net: &mut Network, f: usize) {
    let old = net.patches[f].clone();
    net.patches[f] = (0..old.len()).map(|i| (0..old.len()).map(|j| old[j][i]).collect()).collect();
    let c = net.corners[f];
    net.corners[f] = [c[0], c[3], c[2], c[1]];
}

/// Every edge of the record is traversed in opposite directions by its two faces.
pub fn edges_are_consistent(record: &Brep) -> bool {
    record.edges.iter().all(|e| e.uses.len() == 2 && e.uses[0].upper != e.uses[1].upper)
}

/// The network of patches given as functions `(u, v) -> point` that are polynomials of degree
/// `degree` (1 or 2) in each parameter; corner ids follow the corner positions.
pub fn network_from_fns(fns: &[&dyn Fn(f64, f64) -> [f64; 3]], degree: usize) -> Network {
    let mut ids: BTreeMap<[i64; 3], u32> = BTreeMap::new();
    let mut patches = Vec::new();
    let mut corners = Vec::new();
    for f in fns {
        let grid: Vec<Vec<[f64; 3]>> = (0..=degree)
            .map(|i| (0..=degree).map(|j| f(i as f64 / degree as f64, j as f64 / degree as f64)).collect())
            .collect();
        let net = if degree == 1 { grid } else { bezier_of_samples(&grid) };
        let mut id = |p: [f64; 3]| {
            let next = ids.len() as u32;
            *ids.entry(p.map(|x| (x * 1e6).round() as i64)).or_insert(next)
        };
        corners.push([id(f(0.0, 0.0)), id(f(1.0, 0.0)), id(f(1.0, 1.0)), id(f(0.0, 1.0))]);
        patches.push(net);
    }
    Network { patches, corners, radius: 0.0 }
}

/// Bézier control net of degree 2 from the samples at 0, 1/2 and 1 in each parameter.
fn bezier_of_samples(g: &[Vec<[f64; 3]>]) -> Vec<Vec<[f64; 3]>> {
    let mid = |a: [f64; 3], m: [f64; 3], b: [f64; 3]| -> [f64; 3] {
        std::array::from_fn(|k| 2.0 * m[k] - 0.5 * a[k] - 0.5 * b[k])
    };
    let rows: Vec<Vec<[f64; 3]>> = g.iter().map(|r| vec![r[0], mid(r[0], r[1], r[2]), r[2]]).collect();
    let middle: Vec<[f64; 3]> = (0..3).map(|j| mid(rows[0][j], rows[1][j], rows[2][j])).collect();
    vec![rows[0].clone(), middle, rows[2].clone()]
}

/// The faces of the record have the control nets of the network, in the order and orientation given.
pub fn faces_are_as_given(record: &Brep, net: &Network) -> bool {
    record.faces.len() == net.patches.len()
        && record
            .faces
            .iter()
            .zip(&net.patches)
            .all(|(f, p)| matches!(&f.surface, Surface::Spline { points, .. } if points == p))
}

/// 64 bit FNV-1a of the record's JSON: a fingerprint of the whole record.
pub fn record_fingerprint(record: &Brep) -> u64 {
    let text = serde_json::to_string(record).unwrap_or_default();
    text.bytes().fold(0xcbf2_9ce4_8422_2325, |h, b| (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3))
}

/// The thin enclosure of the review: the region between z = 0 and z = 0.01 + (u - v)^2 over the unit
/// square, six biquadratic patches (top, bottom, four ruled walls), all oriented outward. Its
/// volume is 0.01 + 1/6.
pub fn thin_enclosure() -> Network {
    let z = |x: f64, y: f64| 0.01 + (x - y) * (x - y);
    let top = |u: f64, v: f64| [u, v, z(u, v)];
    let bottom = |u: f64, v: f64| [v, u, 0.0];
    let w0 = |u: f64, v: f64| [0.0, v, u * z(0.0, v)];
    let w1 = |u: f64, v: f64| [1.0, u, v * z(1.0, u)];
    let w2 = |u: f64, v: f64| [u, 0.0, v * z(u, 0.0)];
    let w3 = |u: f64, v: f64| [v, 1.0, u * z(v, 1.0)];
    network_from_fns(&[&top, &bottom, &w0, &w1, &w2, &w3], 2)
}

/// A box `1 x 1 x 0.001` at `origin`, six bilinear patches oriented outward.
pub fn thin_box(origin: [f64; 3]) -> Network {
    let p = |x: f64, y: f64, z: f64| [origin[0] + x, origin[1] + y, origin[2] + z * 0.001];
    let bottom = |u: f64, v: f64| p(v, u, 0.0);
    let top = |u: f64, v: f64| p(u, v, 1.0);
    let front = |u: f64, v: f64| p(u, 0.0, v);
    let back = |u: f64, v: f64| p(v, 1.0, u);
    let left = |u: f64, v: f64| p(0.0, v, u);
    let right = |u: f64, v: f64| p(1.0, u, v);
    network_from_fns(&[&bottom, &top, &front, &back, &left, &right], 1)
}

/// Two coincident sheets with opposite normals: closed, consistent, and no volume at all.
pub fn pillow() -> Network {
    let sheet = |u: f64, v: f64| [u, v, 0.0];
    let mut net = network_from_fns(&[&sheet], 1);
    let c = net.corners[0];
    net.patches.push(net.patches[0].clone());
    net.corners.push(c);
    flip_patch(&mut net, 1);
    net
}
