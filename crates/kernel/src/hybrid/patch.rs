//! Freeform patches of the hybrid model: cutting the stock by a spline and replacing or measuring faces.

use super::cutters::*;
use super::faces::*;
use super::vector::*;
use super::*;

/// The normal of a spline at the middle of its domain, as finite differences of the surface.
fn spline_normal(surface: &Surface) -> Result<V> {
    let [[u0, u1], [v0, v1]] = crate::nurbs::domain(surface)?;
    let probe = SurfaceProbe::new(surface)?;
    let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
    let (du, dv) = (1e-3 * (u1 - u0), 1e-3 * (v1 - v0));
    let a = sub(probe.eval(um + du, vm)?, probe.eval(um - du, vm)?);
    let b = sub(probe.eval(um, vm + dv)?, probe.eval(um, vm - dv)?);
    unit(cross(a, b)).ok_or_else(|| KernelError::Argument("degenerate spline".into()))
}

pub(super) fn cut_by_spline_body(body: &ops::Body, surface: &Surface, outward: V) -> Result<ops::Body> {
    if !finite_all(&outward) {
        return Err(KernelError::Argument("outward direction must be finite".into()));
    }
    let domain = crate::nurbs::domain(surface)?;
    let n = spline_normal(surface)?;
    let tool = ops::split::Tool { surface: surface.clone(), domain };
    let parts = ops::split::split_body(body, &tool).map_err(|e| op_err("split_body", e))?;
    pick(parts, n, outward)
}

/// Cuts `body` with a spline tool and keeps the side `outward` points away from (the tool's
/// own side that holds the material). The spline must cross the body completely.
pub fn cut_by_spline(body: &Brep, surface: &Surface, outward: [f64; 3]) -> Result<Brep> {
    let b = decode(body)?;
    let r = cut_by_spline_body(&b, surface, outward)?;
    encode(&r)
}

/// Replaces `face` (a record index of a planar face) of a solid by the spline tool `surface`:
/// the material beyond the patch (on the face's outward side) is cut away. The patch must run
/// across the whole footprint of the face and not above it everywhere (it only removes material).
pub fn replace_face(body: &Brep, face: usize, surface: &Surface) -> Result<Brep> {
    let outward = planar_face_outward(body, face)?;
    let rise = patch_rise(body, face, surface, outward)?;
    if rise <= 1e-6 {
        return cut_by_spline(body, surface, outward);
    }
    replace_face_up_and_down(body, face, surface, rise)
}

/// How far the patch reaches beyond the plane of `face` on its outward side (mm, sampled on a
/// 48 x 48 grid of the patch; negative when the patch lies entirely below the face).
pub fn patch_rise(body: &Brep, face: usize, surface: &Surface, outward: V) -> Result<f64> {
    let verts = face_vertices(body, face);
    let p0 = *verts.first().ok_or_else(|| KernelError::Argument("the face has no vertices".into()))?;
    let level = dot(outward, p0);
    let [[u0, u1], [v0, v1]] = crate::nurbs::domain(surface)?;
    let probe = SurfaceProbe::new(surface)?;
    let n = 48;
    let mut top = f64::NEG_INFINITY;
    for i in 0..=n {
        for j in 0..=n {
            let u = u0 + (u1 - u0) * i as f64 / n as f64;
            let v = v0 + (v1 - v0) * j as f64 / n as f64;
            let h = dot(outward, probe.eval(u, v)?) - level;
            if h.is_finite() {
                top = top.max(h);
            }
        }
    }
    if top.is_finite() { Ok(top) } else { Err(KernelError::Argument("the patch could not be sampled".into())) }
}

/// The outer loop of a planar face as profile curves (lines and circular arcs only).
fn face_profile(body: &Brep, face: usize) -> Result<Vec<crate::feature::ProfileCurve>> {
    use crate::feature::ProfileCurve;
    use crate::record::brep::Curve;
    let f = body
        .faces
        .get(face)
        .ok_or_else(|| KernelError::Argument(format!("face indices must be in 0..{}", body.faces.len())))?;
    let Some(Loop::Edges { edges }) = f.loops.first() else {
        return Err(KernelError::Argument("the face has no edge loop".into()));
    };
    let vertex = |i: u32| {
        body.vertices
            .get(i as usize)
            .map(|v| v.point)
            .ok_or_else(|| KernelError::Argument("vertex index out of range".into()))
    };
    let mut out = Vec::with_capacity(edges.len());
    for oe in edges {
        let e =
            body.edges.get(oe.edge as usize).ok_or_else(|| KernelError::Argument("edge index out of range".into()))?;
        let (a, b) = (vertex(e.vertices[0])?, vertex(e.vertices[1])?);
        match &e.curve {
            Curve::Line { .. } => out.push(ProfileCurve::Line { start: a, end: b }),
            Curve::Circle { frame, radius } => {
                let axis =
                    unit(frame_axis(frame)).ok_or_else(|| KernelError::Argument("degenerate circle frame".into()))?;
                let (x, y) = (frame.x_axis, frame.y_axis);
                let ang = |p: V| {
                    let d = sub(p, frame.origin);
                    dot(d, y).atan2(dot(d, x))
                };
                let start = ang(a);
                let mut end = ang(b);
                if e.vertices[0] == e.vertices[1] || end <= start + 1e-12 {
                    end += std::f64::consts::TAU;
                }
                out.push(ProfileCurve::Arc {
                    center: frame.origin,
                    axis,
                    reference: unit(x).unwrap_or(x),
                    radius: *radius,
                    start,
                    end,
                });
            }
            _ => {
                return Err(KernelError::Argument(
                    "the face boundary has an edge that is neither a line nor a circular arc".into(),
                ));
            }
        }
    }
    Ok(out)
}

/// The prism standing on a planar face: its outer loop (and inner loops as holes) extruded
/// outward by `height`.
fn face_prism(body: &Brep, face: usize, outward: V, height: f64) -> Result<Brep> {
    let f = body
        .faces
        .get(face)
        .ok_or_else(|| KernelError::Argument(format!("face indices must be in 0..{}", body.faces.len())))?;
    if f.loops.len() > 1 {
        return Err(KernelError::Argument("a face with holes cannot be raised yet".into()));
    }
    let profile = face_profile(body, face)?;
    crate::feature::extrude(&[profile], outward, height, 0.0)
}

/// Replaces a planar face by a patch that may lie above the face in places and below it in
/// others: the face is first extruded outward by `rise` (the patch's highest point over the
/// plane) and united with the body, then the material beyond the patch is cut away.
///
/// # Errors
/// When the extrusion would meet other geometry of the body (the union does not add the prism's
/// whole volume), when the face boundary is not made of lines and arcs, or when a kernel
/// operation is refused.
pub fn replace_face_up_and_down(body: &Brep, face: usize, surface: &Surface, rise: f64) -> Result<Brep> {
    let outward = planar_face_outward(body, face)?;
    if !rise.is_finite() || rise <= 0.0 {
        return Err(KernelError::Argument("the rise must be positive and finite".into()));
    }
    let pad = 0.05 * rise + 1e-3;
    let prism = face_prism(body, face, outward, rise + pad)?;
    let raise = || -> Result<(Brep, f64, f64, f64)> {
        let (vb, vp) = (solid::volume(body)?, solid::volume(&prism)?);
        let united = crate::feature::boolean(body, &prism, crate::feature::BoolKind::Union)?;
        let vu = solid::volume(&united)?;
        Ok((united, vb, vp, vu))
    };
    let (united, vb, vp, vu) = raise().map_err(|e| {
        KernelError::Argument(format!("the face cannot be raised: the extension meets other geometry ({e})"))
    })?;
    if (vu - vb - vp).abs() > 1e-6 * (vb + vp) {
        return Err(KernelError::Argument(
            "the face cannot be raised: the extension would collide with other geometry of the body".into(),
        ));
    }
    let b = decode(&united)?;
    let r = cut_by_spline_body(&b, surface, outward)?;
    encode(&r)
}
