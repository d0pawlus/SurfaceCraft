//! The new face as the kernel builds it from the boundary of a group (P9-O6b): the surface's NURBS
//! form and parameter box, and the pcurves (the face's trims in its own parameters) of every edge of
//! its loops, are the kernel's, not computed here.
//!
//! The boundary edges of the group, as the record has them (their curves and vertices), and the
//! primitive go to the kernel's assembly of loose faces, edges and vertices as a one-face sheet (the
//! stage the STEP reader builds its bodies with, `solidcore_step::assembly`). The assembly makes the
//! pcurves, chooses the parameter box and the seam, and checks the loops; its record of the face is
//! read back, and its loops are mapped to the edges of the body by their end points.
use super::boundary::{Boundary, LoopEdge};
use crate::record::brep::{Brep, Curve, Curve2, Face, Loop, OrientedEdge, Surface};
use crate::step::KernelTrims;
use crate::{KernelError, Result, op_err};
use solidcore_step::assembly::{Assembly, Bound, Kind};
use std::collections::BTreeMap;

/// End points closer than this (mm) are the same point when the kernel's edges are mapped back.
const SAME_POINT: f64 = 1e-6;

/// The least tolerance the face is assembled at (mm).
const MIN_TOLERANCE: f64 = 1e-7;

fn refuse(what: impl Into<String>) -> KernelError {
    KernelError::Argument(what.into())
}

/// The record surface the kernel's assembly is given for the face: a plane, cylinder, cone (opening
/// along its axis) or sphere, as the hybrid merge places groups on.
fn checked_surface(s: &Surface) -> Result<Surface> {
    match s {
        Surface::Plane { .. } | Surface::Cylinder { .. } | Surface::Sphere { .. } => Ok(s.clone()),
        Surface::Cone { half_angle, radius_at_origin, .. } if *half_angle > 0.0 && *radius_at_origin >= 0.0 => {
            Ok(s.clone())
        }
        _ => Err(refuse("the kernel's assembly is given a plane, cylinder, cone (opening along its axis) or sphere")),
    }
}

/// The extent of a loop: the diagonal of the box of its edges' control points (mm).
fn extent(body: &Brep, l: &[LoopEdge]) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for le in l {
        if let Some(Curve::Spline { points, .. }) = body.edges.get(le.edge as usize).map(|e| &e.curve) {
            for p in points {
                for k in 0..3 {
                    lo[k] = lo[k].min(p[k]);
                    hi[k] = hi[k].max(p[k]);
                }
            }
        }
    }
    crate::hybrid::norm(crate::hybrid::sub(hi, lo))
}

/// The loops of a boundary, the largest first: it is the outer one, a hole is inside it and smaller (the
/// boundary lists its loops by edge number, which says nothing of which is outer).
fn by_size<'a>(body: &Brep, loops: &'a [Vec<LoopEdge>]) -> Vec<&'a Vec<LoopEdge>> {
    let mut sized: Vec<(f64, &Vec<LoopEdge>)> = loops.iter().map(|l| (extent(body, l), l)).collect();
    sized.sort_by(|a, b| b.0.total_cmp(&a.0));
    sized.into_iter().map(|(_, l)| l).collect()
}

/// The vertices and edges handed to the assembly as they are first used, and the body's edges in
/// that order.
struct Parts<'t> {
    assembly: Assembly<'t>,
    vertex_of: BTreeMap<u32, u32>,
    edge_of: BTreeMap<u32, (u32, bool)>,
    source: Vec<u32>,
}

impl Parts<'_> {
    fn vertex(&mut self, body: &Brep, v: u32) -> Result<u32> {
        if let Some(&i) = self.vertex_of.get(&v) {
            return Ok(i);
        }
        let point = body.vertices.get(v as usize).ok_or_else(|| refuse("vertex index"))?.point;
        let i = self.assembly.add_vertex(point).map_err(|e| op_err("assembly vertex", e))?;
        self.vertex_of.insert(v, i);
        Ok(i)
    }

    /// The assembly edge of the body's edge `edge` (made when it is first used), and whether it runs
    /// the way the body's edge does.
    fn edge(&mut self, body: &Brep, edge: u32) -> Result<(u32, bool)> {
        if let Some(&i) = self.edge_of.get(&edge) {
            return Ok(i);
        }
        let e = body.edges.get(edge as usize).ok_or_else(|| refuse("edge index"))?;
        if !matches!(e.curve, Curve::Spline { .. }) {
            return Err(refuse("a boundary edge is not a spline"));
        }
        let ends = [self.vertex(body, e.vertices[0])?, self.vertex(body, e.vertices[1])?];
        let added = self
            .assembly
            .add_edge(u64::from(edge) + 1, e.curve.clone(), e.interval, ends)
            .map_err(|m| op_err("assembly edge", m))?;
        self.source.push(edge);
        self.edge_of.insert(edge, added);
        Ok(added)
    }
}

/// The group's boundary on `surface` as one face the kernel assembles into a one-face sheet record,
/// and the body's edges that went into it. Each loop is given relative to the surface's natural
/// normal: reversed where the face's side is not that normal.
fn assembled(body: &Brep, b: &Boundary, surface: &Surface, same_sense: bool) -> Result<(Brep, Vec<u32>)> {
    let trims = KernelTrims;
    let assembly = Assembly::new(body.tolerance.max(MIN_TOLERANCE), Some(&trims));
    let mut parts = Parts { assembly, vertex_of: BTreeMap::new(), edge_of: BTreeMap::new(), source: Vec::new() };
    let mut bounds = Vec::new();
    for l in by_size(body, &b.loops) {
        let mut uses = Vec::with_capacity(l.len());
        for le in l {
            let (e, along) = parts.edge(body, le.edge)?;
            uses.push((e, le.forward == along));
        }
        if !same_sense {
            uses.reverse();
            for u in &mut uses {
                u.1 = !u.1;
            }
        }
        bounds.push(Bound::Edges(uses));
    }
    let face = parts
        .assembly
        .add_face(1, checked_surface(surface)?, bounds, Some(0), same_sense)
        .map_err(|m| op_err("assembly face", m))?;
    let solid = parts
        .assembly
        .finish(1, String::new(), vec![vec![(face, true)]], Kind::Model)
        .map_err(|m| op_err("assembly", m))?;
    Ok((solid.brep, parts.source))
}

/// A pcurve of the kernel's edge as it runs along the body's edge: reversed where the two run opposite
/// ways, and on the body's interval.
fn along(p: &Curve2, reverse: bool, from: [f64; 2], to: [f64; 2]) -> Result<Curve2> {
    let Curve2::Spline { knots, points } = p else { return Err(refuse("the kernel gave a rational pcurve")) };
    let [lo, hi] = from;
    let span = hi - lo;
    if !(span.abs() > 0.0) {
        return Err(refuse("a pcurve with no parameter span"));
    }
    let map = |k: f64| {
        let t = (k - lo) / span;
        to[0] + (if reverse { 1.0 - t } else { t }) * (to[1] - to[0])
    };
    let mut knots: Vec<f64> = knots.iter().map(|k| map(*k)).collect();
    let mut points = points.clone();
    if reverse {
        knots.reverse();
        points.reverse();
    }
    Ok(Curve2::Spline { knots, points })
}

fn point_of(rec: &Brep, v: u32) -> Option<[f64; 3]> {
    rec.vertices.get(v as usize).map(|x| x.point)
}

fn close(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|k| (a[k] - b[k]).abs() <= SAME_POINT)
}

/// The face of the kernel's record `rec` with its loops over the body's edges.
fn face_over(body: &Brep, rec: &Brep, source: &[u32]) -> Result<Face> {
    let [face] = rec.faces.as_slice() else { return Err(refuse("the kernel built no single face")) };
    let mut loops = Vec::with_capacity(face.loops.len());
    for l in &face.loops {
        let Loop::Edges { edges } = l else { return Err(refuse("the kernel gave a vertex loop")) };
        let mut out = Vec::with_capacity(edges.len());
        for oe in edges {
            let re = rec.edges.get(oe.edge as usize).ok_or_else(|| refuse("kernel edge index"))?;
            let ends = [point_of(rec, re.vertices[0]), point_of(rec, re.vertices[1])];
            let (Some(r0), Some(r1)) = (ends[0], ends[1]) else { return Err(refuse("kernel vertex index")) };
            let found = source.iter().find_map(|&e| {
                let ours = body.edges.get(e as usize)?;
                let (o0, o1) = (point_of(body, ours.vertices[0])?, point_of(body, ours.vertices[1])?);
                if close(o0, r0) && close(o1, r1) {
                    Some((e, ours, false))
                } else if close(o0, r1) && close(o1, r0) {
                    Some((e, ours, true))
                } else {
                    None
                }
            });
            let (e, ours, opposite) =
                found.ok_or_else(|| refuse("the kernel's face has an edge the body does not (a seam)"))?;
            let pcurve = along(&oe.pcurve, opposite, re.interval, ours.interval)?;
            out.push(OrientedEdge { edge: e, forward: oe.forward != opposite, pcurve });
        }
        loops.push(Loop::Edges { edges: out });
    }
    Ok(Face { surface: face.surface.clone(), nurbs: face.nurbs.clone(), domain: face.domain, loops })
}

/// The new face of the group whose boundary is `b`, on `surface` (a plane, cylinder, cone or sphere
/// of the record), built by the kernel; its loops are over the edges of `body`.
///
/// # Errors
/// A boundary the kernel's assembly does not take, or whose face needs an edge the body has not (a
/// seam).
pub(super) fn built_face(body: &Brep, b: &Boundary, surface: &Surface, same_sense: bool) -> Result<Face> {
    let (rec, source) = assembled(body, b, surface, same_sense)?;
    face_over(body, &rec, &source)
}
