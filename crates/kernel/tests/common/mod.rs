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
