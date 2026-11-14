//! The new face as the kernel builds it from the boundary of a group (P9-O6b): the surface's NURBS
//! form and parameter box, and the pcurves (the face's trims in its own parameters) of every edge of
//! its loops, are the kernel's, not computed here.
//!
//! The boundary edges of the group, as the record has them (their curves and vertices), and the
//! primitive go to the kernel's topology builder as a one-face sheet of the neutral model the STEP
//! importer builds from (`step_brep::build_from_topology`). The builder projects the edges onto the
//! surface, chooses the parameter box and the seam, and orients the loops; its record of the face is
//! read back, and its loops are mapped to the edges of the body by their end points.
use super::boundary::Boundary;
use crate::record::brep::{Brep, Curve, Curve2, Edge, Face, Loop, OrientedEdge, Surface};
use crate::{KernelError, Result, op_err};
use std::collections::BTreeMap;
use step_import::model as m;

/// End points closer than this (mm) are the same point when the kernel's edges are mapped back.
const SAME_POINT: f64 = 1e-6;

fn refuse(what: impl Into<String>) -> KernelError {
    KernelError::Argument(what.into())
}

/// The distinct knots of a clamped knot vector and their multiplicities.
fn distinct(knots: &[f64]) -> (Vec<f64>, Vec<u32>) {
    let mut values: Vec<f64> = Vec::new();
    let mut mults: Vec<u32> = Vec::new();
    for &k in knots {
        if values.last() == Some(&k) {
            if let Some(last) = mults.last_mut() {
                *last += 1;
            }
        } else {
            values.push(k);
            mults.push(1);
        }
    }
    (values, mults)
}

fn model_curve(e: &Edge) -> Result<m::Curve> {
    let Curve::Spline { knots, points } = &e.curve else { return Err(refuse("a boundary edge is not a spline")) };
    let degree =
        knots.len().checked_sub(points.len() + 1).ok_or_else(|| refuse("a boundary edge has a bad knot vector"))?;
    let (knots, mults) = distinct(knots);
    Ok(m::Curve::BSpline(m::BSplineCurve {
        degree: u32::try_from(degree).map_err(|_| refuse("degree"))?,
        points: points.clone(),
        weights: None,
        knots,
        mults,
    }))
}

fn frame(f: &crate::record::brep::Frame) -> m::Frame {
    let z = crate::hybrid::cross(f.x_axis, f.y_axis);
    m::Frame { origin: f.origin, x: f.x_axis, y: f.y_axis, z }
}

fn model_surface(s: &Surface) -> Result<m::Surface> {
    Ok(match s {
        Surface::Plane { origin, u_axis, v_axis } => m::Surface::Plane {
            frame: frame(&crate::record::brep::Frame { origin: *origin, x_axis: *u_axis, y_axis: *v_axis }),
        },
        Surface::Cylinder { frame: f, radius } => m::Surface::Cylinder { frame: frame(f), radius: *radius },
        Surface::Cone { frame: f, radius_at_origin, half_angle } if *half_angle > 0.0 && *radius_at_origin >= 0.0 => {
            m::Surface::Cone { frame: frame(f), radius: *radius_at_origin, semi_angle_deg: half_angle.to_degrees() }
        }
        Surface::Sphere { frame: f, radius } => m::Surface::Sphere { frame: frame(f), radius: *radius },
        _ => {
            return Err(refuse(
                "the kernel's builder takes a plane, cylinder, cone (opening along its axis) or sphere",
            ));
        }
    })
}

/// The extent of a loop: the diagonal of the box of its edges' control points (mm).
fn extent(body: &Brep, l: &[super::boundary::LoopEdge]) -> f64 {
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
fn by_size<'a>(body: &Brep, loops: &'a [Vec<super::boundary::LoopEdge>]) -> Vec<&'a Vec<super::boundary::LoopEdge>> {
    let mut sized: Vec<(f64, &Vec<super::boundary::LoopEdge>)> = loops.iter().map(|l| (extent(body, l), l)).collect();
    sized.sort_by(|a, b| b.0.total_cmp(&a.0));
    sized.into_iter().map(|(_, l)| l).collect()
}

/// The vertices and edges of the model as they are first used, and the body's edge each model edge is.
#[derive(Default)]
struct Parts {
    vertex_of: BTreeMap<u32, usize>,
    edge_of: BTreeMap<u32, usize>,
    vertices: Vec<m::Vertex>,
    edges: Vec<m::Edge>,
    source: Vec<u32>,
}

impl Parts {
    fn vertex(&mut self, body: &Brep, v: u32) -> Result<usize> {
        if let Some(&i) = self.vertex_of.get(&v) {
            return Ok(i);
        }
        let point = body.vertices.get(v as usize).ok_or_else(|| refuse("vertex index"))?.point;
        self.vertices.push(m::Vertex { step_id: self.vertices.len() as u64 + 1, point });
        self.vertex_of.insert(v, self.vertices.len() - 1);
        Ok(self.vertices.len() - 1)
    }

    /// The model edge of the body's edge `edge` (made when it is first used).
    fn edge(&mut self, body: &Brep, edge: u32) -> Result<usize> {
        if let Some(&i) = self.edge_of.get(&edge) {
            return Ok(i);
        }
        let e = body.edges.get(edge as usize).ok_or_else(|| refuse("edge index"))?;
        let (start, end) = (self.vertex(body, e.vertices[0])?, self.vertex(body, e.vertices[1])?);
        self.edges.push(m::Edge {
            step_id: self.edges.len() as u64 + 1,
            start,
            end,
            curve: model_curve(e)?,
            same_sense: true,
            t_start: e.interval[0],
            t_end: e.interval[1],
        });
        self.source.push(edge);
        self.edge_of.insert(edge, self.edges.len() - 1);
        Ok(self.edges.len() - 1)
    }
}

/// The neutral model of the group's boundary on `surface`: one face, its edges and vertices numbered as
/// they first appear, and the body's edge each model edge is.
fn model_of(body: &Brep, b: &Boundary, surface: &Surface, same_sense: bool) -> Result<(m::Body, Vec<u32>)> {
    let mut parts = Parts::default();
    let mut loops: Vec<m::Loop> = Vec::new();
    for l in by_size(body, &b.loops) {
        let mut uses = Vec::with_capacity(l.len());
        for le in l {
            uses.push(m::OrientedEdge { edge: parts.edge(body, le.edge)?, forward: le.forward });
        }
        loops.push(m::Loop { outer: loops.is_empty(), orientation: true, bound: m::Bound::Edges(uses) });
    }
    let face = m::Face { step_id: 1, surface: model_surface(surface)?, same_sense, loops };
    let shell = m::Shell { step_id: 1, closed: false, orientation: true, faces: vec![0] };
    let model = m::Body {
        step_id: 1,
        name: None,
        kind: m::BodyKind::ShellModel,
        unit_mm: 1.0,
        uncertainty_mm: None,
        shells: vec![shell],
        faces: vec![face],
        edges: parts.edges,
        vertices: parts.vertices,
    };
    Ok((model, parts.source))
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
/// A boundary the kernel's builder does not take, or whose face needs an edge the body has not (a
/// seam).
pub(super) fn built_face(body: &Brep, b: &Boundary, surface: &Surface, same_sense: bool) -> Result<Face> {
    let (model, source) = model_of(body, b, surface, same_sense)?;
    let built = step_brep::build_from_topology(&model, step_brep::BuildOptions::default())
        .map_err(|e| op_err("step_brep::build_from_topology", format!("{e:?}")))?;
    if let Some(f) = built.face_failures.first() {
        return Err(refuse(format!("the kernel could not build the face: {f:?}")));
    }
    let mut brep = built.brep;
    let mut k = solidcore::kernel::Kernel::new(&solidcore::kernel::KernelOptions::default());
    let (rec, _) =
        solidcore_convert::brep::to_record(&mut k, &mut brep).map_err(|e| op_err("to_record", format!("{e:?}")))?;
    face_over(body, &rec, &source)
}
