// Adapted from SolidCraft (https://github.com/tangency-labs/solidcraft) at fcd3186,
// crates/step/src/write/topology.rs (open shells as SHELL_BASED_SURFACE_MODEL / OPEN_SHELL of
// ADVANCED_FACEs bounded by EDGE_LOOPs of ORIENTED_EDGEs, side handling by face use).
// Copyright (c) 2026 SolidCraft contributors. Licensed under MIT OR Apache-2.0.
// Modified for SurfaceCraft (Apache-2.0 §4): works directly on solidcore-record without
// solidcraft-geom; edges are SURFACE_CURVEs (SEAM_CURVEs where a face uses one twice) carrying a
// PCURVE for each face that uses them; faces on a cylinder, cone, sphere or torus are written on
// the surface STEP parameterises (kernel crate).

//! Topology of a sheet: vertices, edges with their pcurves, faces and the shell's sides.

use super::geom::surface_name;
use super::p21::{Param, Writer};
use super::pcurve::FaceGeometry;
use super::{bad, e, r, s};
use crate::IoError;
use surfacecraft_kernel::pcurve as kp;
use surfacecraft_kernel::record::brep::{Brep, Curve2, Loop, Surface};

/// A SURFACE_CURVE holds one pcurve per face using the edge: at most two.
const MAX_EDGE_USES: usize = 2;

pub(super) struct ShellWriter<'a> {
    pub(super) w: Writer,
    pub(super) b: &'a Brep,
    vertices: Vec<Option<u64>>,
    edges: Vec<Option<u64>>,
    faces: Vec<Option<FaceGeometry>>,
}

fn record_pcurves(loops: &[Loop]) -> Vec<Vec<Curve2>> {
    loops
        .iter()
        .map(|l| match l {
            Loop::Edges { edges } => edges.iter().map(|u| u.pcurve.clone()).collect(),
            Loop::Vertex { .. } => Vec::new(),
        })
        .collect()
}

impl<'a> ShellWriter<'a> {
    pub(super) fn new(w: Writer, b: &'a Brep) -> Self {
        Self {
            w,
            b,
            vertices: vec![None; b.vertices.len()],
            edges: vec![None; b.edges.len()],
            faces: vec![None; b.faces.len()],
        }
    }

    /// The face's surface entity, side and pcurves in STEP's parameters, made once.
    pub(super) fn geometry(&mut self, f: usize) -> Result<&FaceGeometry, IoError> {
        if self.faces.get(f).is_some_and(Option::is_none) {
            let made = self.make_geometry(f)?;
            if let Some(slot) = self.faces.get_mut(f) {
                *slot = Some(made);
            }
        }
        self.faces.get(f).and_then(Option::as_ref).ok_or_else(|| bad(format!("face {f} missing")))
    }

    fn make_geometry(&mut self, f: usize) -> Result<FaceGeometry, IoError> {
        let face = self.b.faces.get(f).ok_or_else(|| bad(format!("face {f} missing")))?;
        let kernel = |e: surfacecraft_kernel::KernelError| bad(format!("face {f}: {e}"));
        if kp::is_analytic(&face.surface) {
            let a = kp::analytic_face(face, &self.b.edges).map_err(kernel)?;
            let surface = self.step_surface(&a.surface)?;
            return Ok(FaceGeometry {
                surface,
                flip: a.flip_sense,
                pcurves: a.pcurves,
                #[cfg(test)]
                written: a.surface,
            });
        }
        if face.nurbs.is_some() {
            return Err(bad(format!("not supported yet: STEP export of face {f} on a NURBS form in a sheet")));
        }
        match &face.surface {
            Surface::Plane { origin, u_axis, v_axis } => {
                let plane = kp::step_plane(*origin, *u_axis, *v_axis).map_err(kernel)?;
                let surface = self.step_surface(&plane.surface)?;
                let pcurves = record_pcurves(&face.loops)
                    .iter()
                    .map(|row| row.iter().map(|c| plane.pcurve(c)).collect())
                    .collect();
                Ok(FaceGeometry {
                    surface,
                    flip: false,
                    pcurves,
                    #[cfg(test)]
                    written: plane.surface,
                })
            }
            Surface::Spline { .. } | Surface::RationalSpline { .. } => {
                let surface = self.step_surface(&face.surface)?;
                Ok(FaceGeometry {
                    surface,
                    flip: false,
                    pcurves: record_pcurves(&face.loops),
                    #[cfg(test)]
                    written: face.surface.clone(),
                })
            }
            other => Err(bad(format!(
                "not supported yet: STEP export of face {f}'s {} surface in a sheet",
                surface_name(other)
            ))),
        }
    }

    fn vertex(&mut self, v: u32) -> Result<u64, IoError> {
        if let Some(Some(id)) = self.vertices.get(v as usize) {
            return Ok(*id);
        }
        let p = self.b.vertices.get(v as usize).ok_or_else(|| bad(format!("vertex {v} missing")))?.point;
        let pt = self.point(p)?;
        let id = self.w.add_distinct("VERTEX_POINT", &[s(), r(pt)]);
        if let Some(slot) = self.vertices.get_mut(v as usize) {
            *slot = Some(id);
        }
        Ok(id)
    }

    /// The edge's 3-D curve, wrapped in a SURFACE_CURVE (a SEAM_CURVE where one face uses the edge
    /// twice) with a PCURVE for each use. The 3-D curve is the master: on a cylinder, cone, sphere
    /// or torus the pcurve runs over the edge's interval at its own speed.
    fn surface_curve(&mut self, e_idx: u32, curve_3d: u64) -> Result<u64, IoError> {
        let edge = self.b.edges.get(e_idx as usize).ok_or_else(|| bad(format!("edge {e_idx} missing")))?;
        if edge.uses.len() > MAX_EDGE_USES {
            // a non-manifold edge: SURFACE_CURVE holds at most two pcurves, the bare curve is valid
            return Ok(curve_3d);
        }
        let mut pcurves = Vec::with_capacity(edge.uses.len());
        for u in &edge.uses {
            let fg = self.geometry(u.face as usize)?;
            let surface = fg.surface;
            let pc = fg
                .pcurves
                .get(u.r#loop as usize)
                .and_then(|row| row.get(u.position as usize))
                .cloned()
                .ok_or_else(|| bad(format!("edge {e_idx}: no pcurve for its use on face {}", u.face)))?;
            pcurves.push(r(self.pcurve(surface, &pc)?));
        }
        if pcurves.is_empty() {
            return Ok(curve_3d);
        }
        let seam = matches!(edge.uses.as_slice(), [a, c] if a.face == c.face);
        let kind = if seam { "SEAM_CURVE" } else { "SURFACE_CURVE" };
        Ok(self.w.add_distinct(kind, &[s(), r(curve_3d), Param::List(pcurves), e("CURVE_3D")]))
    }

    fn edge(&mut self, e_idx: u32) -> Result<u64, IoError> {
        if let Some(Some(id)) = self.edges.get(e_idx as usize) {
            return Ok(*id);
        }
        let edge = self.b.edges.get(e_idx as usize).ok_or_else(|| bad(format!("edge {e_idx} missing")))?;
        let curve = edge.nurbs.as_ref().unwrap_or(&edge.curve).clone();
        let (a, b) = (self.vertex(edge.vertices[0])?, self.vertex(edge.vertices[1])?);
        let g = self.curve(&curve)?;
        let geometry = self.surface_curve(e_idx, g)?;
        let id = self.w.add_distinct("EDGE_CURVE", &[s(), r(a), r(b), r(geometry), e("T")]);
        if let Some(slot) = self.edges.get_mut(e_idx as usize) {
            *slot = Some(id);
        }
        Ok(id)
    }

    /// The face as a shell uses it; `side` true where its normal is the form's natural one. The
    /// loops keep the form's direction (reversed for the other side); the ADVANCED_FACE's sense
    /// also accounts for a written surface whose natural normal opposes the form's.
    pub(super) fn face(&mut self, f: u32, side: bool) -> Result<u64, IoError> {
        let face = self.b.faces.get(f as usize).ok_or_else(|| bad(format!("face {f} missing")))?;
        let loops = face.loops.clone();
        let (geometry, flip) = {
            let fg = self.geometry(f as usize)?;
            (fg.surface, fg.flip)
        };
        let mut bounds = Vec::with_capacity(loops.len());
        for (li, l) in loops.iter().enumerate() {
            let lp = match l {
                Loop::Edges { edges } => {
                    let mut oriented = Vec::with_capacity(edges.len());
                    let order: Box<dyn Iterator<Item = _>> =
                        if side { Box::new(edges.iter()) } else { Box::new(edges.iter().rev()) };
                    for u in order {
                        let ec = self.edge(u.edge)?;
                        let along = if u.forward == side { "T" } else { "F" };
                        oriented.push(r(self
                            .w
                            .add_distinct("ORIENTED_EDGE", &[s(), Param::Derived, Param::Derived, r(ec), e(along)])));
                    }
                    self.w.add_distinct("EDGE_LOOP", &[s(), Param::List(oriented)])
                }
                Loop::Vertex { vertex } => {
                    let v = self.vertex(*vertex)?;
                    self.w.add_distinct("VERTEX_LOOP", &[s(), r(v)])
                }
            };
            let kind = if li == 0 { "FACE_OUTER_BOUND" } else { "FACE_BOUND" };
            bounds.push(r(self.w.add_distinct(kind, &[s(), r(lp), e("T")])));
        }
        let same_sense = side != flip;
        Ok(self.w.add_distinct(
            "ADVANCED_FACE",
            &[s(), Param::List(bounds), r(geometry), e(if same_sense { "T" } else { "F" })],
        ))
    }
}

/// The gap (mm) of every edge use of every face of `b`, as the writer would write them: the
/// pcurve on the surface in STEP's parameters against the edge's curve, both evaluated by the
/// kernel.
#[cfg(test)]
pub(super) fn pcurve_gaps(b: &Brep) -> Result<Vec<f64>, IoError> {
    let mut sw = ShellWriter::new(Writer::new(), b);
    let mut out = Vec::new();
    for (f, face) in b.faces.iter().enumerate() {
        let fg = sw.geometry(f)?.clone();
        for (li, l) in face.loops.iter().enumerate() {
            let Loop::Edges { edges } = l else { continue };
            for (pi, u) in edges.iter().enumerate() {
                let edge = b.edges.get(u.edge as usize).ok_or_else(|| bad("edge"))?;
                let pc = fg.pcurves.get(li).and_then(|row| row.get(pi)).ok_or_else(|| bad("pcurve"))?;
                let curve = edge.nurbs.as_ref().unwrap_or(&edge.curve);
                out.push(kp::pcurve_gap(&fg.written, pc, curve, edge.interval).map_err(|e| bad(e.to_string()))?);
            }
        }
    }
    Ok(out)
}
