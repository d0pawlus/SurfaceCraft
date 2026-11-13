//! Merging the patches of a freeform body that lie on one primitive into a single analytic face
//! (P9-O6b), by editing the topology of the record: nothing is cut, no sharp edge is needed.
//!
//! The faces of a group go; their interior edges and vertices go with them; the group's
//! boundary loop (the edges it shares with the patches around it, or the open border of a
//! sheet) becomes the outer loop of one new face on the primitive's surface, so the boundary
//! edges stay shared with the neighbours. The boundary edges are splines that lie on the
//! primitive only within the fit, so each one (and its end vertices) is given the measured gap as
//! its tolerance, which the kernel takes as a tolerant edge. The result is rebuilt by the kernel
//! (it builds the trims of the analytic face from the edges) and refused when the kernel does not
//! take it or when its volume or topology class changed.
use crate::hybrid::{Shape, V, add, cross, norm, scale, sub, unit};
use crate::nurbs::{SurfaceProbe, canonicalize, domain, is_closed};
use crate::record::brep::{Brep, Curve2, Face, Loop, OrientedEdge, OrientedFace, Shell, Surface};
use crate::solid::{decode, encode};
use crate::{KernelError, Result, finite_all};

mod boundary;
mod place;
mod revolve;

use boundary::{Boundary, LoopEdge, boundary, edge_points};
use place::{Placement, distance, place};

/// The largest edge-to-surface gap a merge accepts (mm); the caller passes its own, smaller one.
pub const MAX_GAP: f64 = 0.5;

/// What one merge did.
#[derive(Clone, Debug)]
pub struct Merged {
    /// The body with the group replaced by one analytic face.
    pub brep: Brep,
    /// For each face of the input body, its face in `brep`; `None` for the removed ones. The
    /// new face is the image of the lowest face of the group.
    pub face_map: Vec<Option<usize>>,
    /// The new face in `brep`.
    pub face: usize,
    /// The largest distance of a boundary edge of the new face from its surface (mm).
    pub gap: f64,
    /// Faces merged away (the group's size), edges and vertices that went.
    pub removed: [usize; 3],
}

/// The outward direction of a patch face at the middle of its domain (its natural normal turned
/// by the shell's use of it).
fn patch_outward(body: &Brep, face: usize) -> Result<V> {
    let f = body.faces.get(face).ok_or_else(|| KernelError::Argument("face index out of range".into()))?;
    if !matches!(f.surface, Surface::Spline { .. } | Surface::RationalSpline { .. }) {
        return Err(KernelError::Argument("only spline patches are merged".into()));
    }
    let [[u0, u1], [v0, v1]] = domain(&f.surface)?;
    let probe = SurfaceProbe::new(&f.surface)?;
    let (um, vm, du, dv) = (0.5 * (u0 + u1), 0.5 * (v0 + v1), 1e-3 * (u1 - u0), 1e-3 * (v1 - v0));
    let a = sub(probe.eval(um + du, vm)?, probe.eval(um - du, vm)?);
    let b = sub(probe.eval(um, vm + dv)?, probe.eval(um, vm - dv)?);
    let n = unit(cross(a, b)).ok_or_else(|| KernelError::Argument("a degenerate patch".into()))?;
    let forward = body.shells.iter().find_map(|s| match s {
        Shell::Faces { faces, .. } => faces.iter().find(|o| o.face as usize == face).map(|o| o.forward),
        _ => None,
    });
    Ok(if forward == Some(false) { scale(n, -1.0) } else { n })
}

/// The pcurve of a boundary edge on the new face, along the edge's own interval.
fn pcurve(body: &Brep, edge: u32, placed: &Placement) -> Result<Curve2> {
    let e = body.edges.get(edge as usize).ok_or_else(|| KernelError::Argument("edge index".into()))?;
    let vertex = |i: u32| body.vertices.get(i as usize).map(|v| v.point);
    let [lo, hi] = e.interval;
    if let (true, crate::record::brep::Curve::Spline { knots, points }) = (placed.is_plane(), &e.curve) {
        let pts = points.iter().map(|p| placed.param(*p)).collect();
        return Ok(Curve2::Spline { knots: knots.clone(), points: pts });
    }
    if placed.is_plane() {
        let (a, b) =
            vertex(e.vertices[0]).zip(vertex(e.vertices[1])).ok_or_else(|| KernelError::Argument("vertex".into()))?;
        return Ok(Curve2::Spline { knots: vec![lo, lo, hi, hi], points: vec![placed.param(a), placed.param(b)] });
    }
    Ok(through(&edge_points(body, edge)?, lo, hi, placed))
}

/// A C0 chain of cubic Béziers through the surface parameters of the samples `points` (equally
/// spaced over `[lo, hi]`), with Catmull-Rom tangents: the pcurve of an edge on a curved surface.
fn through(points: &[V], lo: f64, hi: f64, placed: &Placement) -> Curve2 {
    let q: Vec<[f64; 2]> = points.iter().map(|p| placed.param(*p)).collect();
    let n = q.len().saturating_sub(1).max(1);
    let at = |i: usize| q.get(i.min(n)).copied().unwrap_or([0.0; 2]);
    let tangent = |i: usize| -> [f64; 2] {
        let (a, b) = (at(i.saturating_sub(1)), at(i + 1));
        let k = if i == 0 || i == n { 1.0 } else { 0.5 };
        [k * (b[0] - a[0]) / 3.0, k * (b[1] - a[1]) / 3.0]
    };
    let mut pts = vec![at(0)];
    for i in 0..n {
        let (p0, p1, t0, t1) = (at(i), at(i + 1), tangent(i), tangent(i + 1));
        pts.push([p0[0] + t0[0], p0[1] + t0[1]]);
        pts.push([p1[0] - t1[0], p1[1] - t1[1]]);
        pts.push(p1);
    }
    let mut knots = vec![lo; 4];
    for i in 1..n {
        knots.extend([lo + (hi - lo) * i as f64 / n as f64; 3]);
    }
    knots.extend([hi; 4]);
    Curve2::Spline { knots, points: pts }
}

/// The new face and its use in the shell, and the gap of the boundary from the surface.
fn new_face(body: &Brep, b: &Boundary, shape: &Shape, outward: V) -> Result<(Face, bool, f64)> {
    let mut samples: Vec<V> = Vec::new();
    for le in &b.loop_edges {
        samples.extend(edge_points(body, le.edge)?);
    }
    let gap = samples
        .iter()
        .map(|p| distance(shape, *p).ok_or_else(|| KernelError::Argument("the shape cannot be merged".into())))
        .try_fold(0.0_f64, |m, d| d.map(|d| m.max(d)))?;
    let placed = place(shape, &samples, outward)?;
    let mut edges: Vec<OrientedEdge> = Vec::with_capacity(b.loop_edges.len());
    for le in &b.loop_edges {
        edges.push(OrientedEdge { edge: le.edge, forward: le.forward, pcurve: pcurve(body, le.edge, &placed)? });
    }
    if placed.flip {
        edges.reverse();
        for e in &mut edges {
            e.forward = !e.forward;
        }
    }
    let face = Face {
        surface: placed.surface,
        nurbs: placed.nurbs,
        domain: placed.domain,
        loops: vec![Loop::Edges { edges }],
    };
    Ok((face, !placed.flip, gap))
}

/// Gives the boundary edges and their end vertices the tolerance of the gap.
fn loosen(rec: &mut Brep, loop_edges: &[LoopEdge], gap: f64) {
    let tol = 1.5 * gap + 1e-6;
    for le in loop_edges {
        let Some(e) = rec.edges.get_mut(le.edge as usize) else { continue };
        e.tolerance = e.tolerance.max(tol);
        for v in e.vertices {
            if let Some(v) = rec.vertices.get_mut(v as usize) {
                v.tolerance = v.tolerance.max(tol);
            }
        }
    }
}

/// The record with `group` replaced by the new face (not yet rebuilt by the kernel).
fn edited(body: &Brep, group: &[usize], b: &Boundary, face: Face, forward: bool) -> (Brep, Vec<Option<usize>>) {
    let first = group.iter().copied().min().unwrap_or(0);
    let mut map = vec![None; body.faces.len()];
    let mut faces = Vec::with_capacity(body.faces.len());
    for (i, f) in body.faces.iter().enumerate() {
        if i == first {
            faces.push(face.clone());
        } else if group.contains(&i) {
            continue;
        } else {
            faces.push(f.clone());
        }
        if let Some(slot) = map.get_mut(i) {
            *slot = Some(faces.len() - 1);
        }
    }
    let mut out = body.clone();
    out.faces = faces;
    let new_index = |old: u32| map.get(old as usize).copied().flatten().map(|n| n as u32);
    for (si, s) in out.shells.iter_mut().enumerate() {
        if let Shell::Faces { faces, .. } = s {
            *faces = faces
                .iter()
                .filter_map(|o| {
                    let forward = if o.face as usize == first && si == b.shell { forward } else { o.forward };
                    new_index(o.face).map(|face| OrientedFace { face, forward })
                })
                .collect();
        }
    }
    (out, map)
}

/// Merges the faces `group` of `body` into one face on `shape`, within `max_gap` of it.
///
/// # Errors
/// A group that is not one connected patch of a face shell with one boundary loop, a shape the
/// merge does not place or whose surface the boundary leaves by more than `max_gap`, a face that
/// would wrap round its surface's axis, and a result the kernel does not take or whose volume
/// differs from the body's by more than the gap allows.
pub fn merge_faces_onto_shape(body: &Brep, group: &[usize], shape: &Shape, max_gap: f64) -> Result<Merged> {
    if !(max_gap.is_finite() && max_gap > 0.0 && max_gap <= MAX_GAP) {
        return Err(KernelError::Argument(format!("the gap must be in (0, {MAX_GAP}]")));
    }
    let b = boundary(body, group)?;
    let mut outward = [0.0; 3];
    for &f in group {
        outward = add(outward, patch_outward(body, f)?);
    }
    if !finite_all(&outward) || norm(outward) < 1e-9 {
        return Err(KernelError::Argument("the patches of the group do not face one way".into()));
    }
    let (face, forward, gap) = new_face(body, &b, shape, outward)?;
    if gap > max_gap {
        return Err(KernelError::Argument(format!(
            "the group's boundary is {gap:.4} mm from the surface (limit {max_gap})"
        )));
    }
    let (mut rec, face_map) = edited(body, group, &b, face, forward);
    loosen(&mut rec, &b.loop_edges, gap);
    let rec = canonicalize(rec);
    let rebuilt = encode(&decode(&rec)?)?;
    let first = group.iter().copied().min().unwrap_or(0);
    let new = face_map.get(first).copied().flatten().ok_or_else(|| KernelError::Argument("group".into()))?;
    verify(body, &rebuilt, group.len())?;
    let removed = [
        group.len().saturating_sub(1),
        body.edges.len().saturating_sub(rebuilt.edges.len()),
        body.vertices.len().saturating_sub(rebuilt.vertices.len()),
    ];
    Ok(Merged { brep: rebuilt, face_map, face: new, gap, removed })
}

/// The rebuilt body must have the faces the edit made and the class (closed or open) of the input.
fn verify(before: &Brep, after: &Brep, group: usize) -> Result<()> {
    if after.faces.len() + group - 1 != before.faces.len() {
        return Err(KernelError::Argument("the kernel changed the faces of the merged body".into()));
    }
    if is_closed(before) != is_closed(after) {
        return Err(KernelError::Argument("the merge changed the body from closed to open or back".into()));
    }
    Ok(())
}
