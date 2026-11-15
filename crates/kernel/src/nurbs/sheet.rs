//! The pieces of a sheet record over one clamped spline surface (see [`super::sheet_from_spline`]):
//! the four boundary curves, the corner vertices, the boundary edges with their outer loop, and the
//! record that holds them.
use super::{SurfaceProbe, line2};
use crate::Result;
use crate::record::brep::{
    Brep, Curve, Edge, EdgeUseRef, Face, Loop, OrientedEdge, OrientedFace, Region, Shell, Surface, Vertex,
};

/// The four boundary iso-curves of a clamped spline: bottom v=v0 (along u), right u=u1 (along v),
/// top v=v1 (along u), left u=u0 (along v). They are the outer control rows and columns.
pub(super) fn boundary_curves(u: &[f64], v: &[f64], points: &[Vec<[f64; 3]>]) -> [Curve; 4] {
    let nu = points.len();
    let nv = points.first().map_or(0, Vec::len);
    let row = |j: usize| -> Vec<[f64; 3]> { points.iter().filter_map(|r| r.get(j).copied()).collect() };
    let col = |i: usize| -> Vec<[f64; 3]> { points.get(i).cloned().unwrap_or_default() };
    [
        Curve::Spline { knots: u.to_vec(), points: row(0) },
        Curve::Spline { knots: v.to_vec(), points: col(nu - 1) },
        Curve::Spline { knots: u.to_vec(), points: row(nv - 1) },
        Curve::Spline { knots: v.to_vec(), points: col(0) },
    ]
}

/// The four corners of the spline over the parameter box `dom` (as kernel-evaluated vertices).
pub(super) fn corner_vertices(s: &Surface, dom: [[f64; 2]; 2]) -> Result<Vec<Vertex>> {
    let [[u0, u1], [v0, v1]] = dom;
    let probe = SurfaceProbe::new(s)?;
    let corners = [probe.eval(u0, v0)?, probe.eval(u1, v0)?, probe.eval(u1, v1)?, probe.eval(u0, v1)?];
    Ok(corners.iter().map(|c| Vertex { point: *c, tolerance: 1e-7 }).collect())
}

/// The four boundary edges over `dom` and the oriented edges of the face's outer loop.
pub(super) fn boundary_edges(curves: &[Curve; 4], dom: [[f64; 2]; 2]) -> (Vec<Edge>, Vec<OrientedEdge>) {
    let [[u0, u1], [v0, v1]] = dom;
    // vertex indices [start, end] of each edge on its own parameter
    let ends: [[u32; 2]; 4] = [[0, 1], [1, 2], [3, 2], [0, 3]];
    let intervals = [[u0, u1], [v0, v1], [u0, u1], [v0, v1]];
    // the outer loop runs counter-clockwise in (u, v): bottom →, right ↑, top ←, left ↓
    let forward = [true, true, false, false];
    let pcurves = [
        line2([u0, v0], [u1, v0], u0, u1),
        line2([u1, v0], [u1, v1], v0, v1),
        line2([u0, v1], [u1, v1], u0, u1),
        line2([u0, v0], [u0, v1], v0, v1),
    ];
    let mut edges = Vec::with_capacity(4);
    let mut loop_edges = Vec::with_capacity(4);
    for k in 0..4 {
        edges.push(Edge {
            curve: curves[k].clone(),
            nurbs: None,
            interval: intervals[k],
            vertices: ends[k],
            tolerance: 1e-7,
            uses: vec![EdgeUseRef { face: 0, r#loop: 0, position: k as u32, upper: !forward[k] }],
        });
        loop_edges.push(OrientedEdge { edge: k as u32, forward: forward[k], pcurve: pcurves[k].clone() });
    }
    (edges, loop_edges)
}

/// The record of a sheet: one region, one shell, one face on `s` bounded by the given edges.
pub(super) fn sheet_record(
    s: &Surface,
    dom: [[f64; 2]; 2],
    edges: Vec<Edge>,
    loop_edges: Vec<OrientedEdge>,
    vertices: Vec<Vertex>,
) -> Brep {
    Brep {
        version: crate::record::brep::VERSION,
        tolerance: 1e-6,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces { faces: vec![OrientedFace { face: 0, forward: true }], wires: vec![] }],
        faces: vec![Face {
            surface: s.clone(),
            nurbs: None,
            domain: dom,
            loops: vec![Loop::Edges { edges: loop_edges }],
        }],
        edges,
        vertices,
    }
}
