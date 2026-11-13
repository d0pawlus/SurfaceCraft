//! The hybrid model on a given stock (P9-O6): the stock is a solid that already follows the part
//! (the freeform wrap of a scan prepared so that it lies a little beyond every primitive), and the
//! primitives trim it, so that the faces of the result are the primitives where they fit and the
//! stock's own patches elsewhere.
//!
//! Unlike [`solid_from_plan`](super::solid_from_plan) this takes no box and makes no blends (a blend
//! needs the sharp edge two planes make, which a patch network does not have). Every cut is made on
//! its own and checked: one the kernel refuses, or that leaves anything but one closed manifold
//! solid, is reported as skipped and the body before it stands, so one refusal never costs the other
//! analytic faces.

use super::cutters::*;
use super::vector::*;
use super::*;
use crate::local::{Applied, LocalAdd, LocalTool, subtract_tool, unite_add};

/// Room around the stock's vertices for the tools, as a share of its diagonal (plus 1 mm).
const TOOL_ROOM: f64 = 0.1;

fn bounds(stock: &Brep) -> Result<(V, V)> {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for v in &stock.vertices {
        for i in 0..3 {
            lo[i] = lo[i].min(v.point[i]);
            hi[i] = hi[i].max(v.point[i]);
        }
    }
    if !finite_all(&lo) || !finite_all(&hi) {
        return Err(KernelError::Argument("the stock has no finite vertices".into()));
    }
    let room = TOOL_ROOM * norm(sub(hi, lo)) + 1.0;
    Ok((sub(lo, [room; 3]), add(hi, [room; 3])))
}

/// Is `body` one closed manifold solid?
fn is_one_solid(body: &ops::Body) -> std::result::Result<(), String> {
    let manifold = body.is_manifold_solid().map_err(|e| e.to_string())?;
    let count = body.solid_count().map_err(|e| e.to_string())?;
    if manifold && count == 1 { Ok(()) } else { Err(format!("the result was not one closed solid ({count} solids)")) }
}

/// Takes `candidate` as the new body when it is one closed solid; else says why not.
fn accept(body: &mut ops::Body, candidate: Result<ops::Body>) -> std::result::Result<(), String> {
    let next = candidate.map_err(|e| e.to_string())?;
    is_one_solid(&next)?;
    *body = next;
    Ok(())
}

fn status_of(r: std::result::Result<&'static str, String>) -> CutStatus {
    match r {
        Ok(how) => CutStatus::Applied(how),
        Err(why) => CutStatus::Skipped(why),
    }
}

fn plane_step(body: &mut ops::Body, point: V, normal: V, half: f64) -> CutStatus {
    let Some(n) = unit(normal) else { return CutStatus::Skipped("plane normal is zero".into()) };
    status_of(accept(body, half_space(body, point, n, half)).map(|()| "halfspace"))
}

fn revolution_step(body: &mut ops::Body, cut: &Cut, lo: V, hi: V) -> CutStatus {
    let op = if cut.convex { ops::BooleanOp::Intersect } else { ops::BooleanOp::Subtract };
    let how = if cut.convex { "intersect" } else { "subtract" };
    let made = (|| -> Result<ops::Body> {
        let tool = shape_solid(&cut.shape, lo, hi)?;
        let copy = body.copy().map_err(|e| op_err("copy", e))?;
        ops::boolean::boolean(copy, tool, op)
            .map_err(|e| op_err("boolean", e))?
            .ok_or_else(|| KernelError::Op { op: "boolean", message: "the cut removed everything".into() })
    })();
    status_of(accept(body, made).map(|()| how))
}

fn cut_step(body: &mut ops::Body, cut: &Cut, half: f64, lo: V, hi: V) -> CutStatus {
    if let Err(e) = check_shape(&cut.shape) {
        return CutStatus::Skipped(e.to_string());
    }
    match &cut.shape {
        Shape::Plane { point, normal } => plane_step(body, *point, *normal, half),
        _ if !cut.full => CutStatus::Skipped(
            "a partial cylinder, cone, sphere or torus is a blend, which needs the sharp edge of two planes; \
             a patch stock has none"
                .into(),
        ),
        _ => revolution_step(body, cut, lo, hi),
    }
}

/// Applies a tool or an addition (`apply` consumes the body and gives it back) and keeps the
/// result only when it is one closed solid.
fn local_step(body: &mut ops::Body, apply: impl FnOnce(ops::Body) -> (ops::Body, Result<Applied>)) -> CutStatus {
    let backup = match body.copy() {
        Ok(b) => b,
        Err(e) => return CutStatus::Skipped(format!("copy: {e}")),
    };
    let (done, result) = apply(std::mem::replace(body, backup));
    match result.map_err(|e| e.to_string()).and_then(|a| is_one_solid(&done).map(|()| a)) {
        Ok(Applied::Done(how)) => {
            *body = done;
            CutStatus::Applied(how)
        }
        Ok(Applied::Partly(why)) => CutStatus::Skipped(why),
        Err(why) => CutStatus::Skipped(why),
    }
}

/// Trims `stock` with the primitives of the plan: planes and full-revolution shapes first, then the
/// local subtractions (`tools`), then the additions (`adds`). The result is verified to be one closed
/// manifold solid; a cut that cannot be made is skipped (see [`Built`]), and `freeform` is empty.
///
/// # Errors
/// When the stock cannot be read by the kernel or nothing could be applied.
pub fn cut_stock(stock: &Brep, cuts: &[Cut], tools: &[LocalTool], adds: &[LocalAdd]) -> Result<Built> {
    if cuts.len() > 256 || tools.len() > 256 || adds.len() > 256 {
        return Err(KernelError::Argument("at most 256 cuts, tools or additions".into()));
    }
    let (lo, hi) = bounds(stock)?;
    let half = 4.0 * norm(sub(hi, lo)).max(1.0);
    let mut body = decode(stock)?;
    is_one_solid(&body).map_err(|e| KernelError::Argument(format!("the stock is not usable: {e}")))?;
    let mut primitives: Vec<CutStatus> = vec![CutStatus::Skipped("not applied".into()); cuts.len()];
    // planes before revolutions, as in the box stock: a revolution is cut from what the planes left
    for pass_planes in [true, false] {
        for (cut, st) in cuts.iter().zip(primitives.iter_mut()) {
            if matches!(cut.shape, Shape::Plane { .. }) == pass_planes {
                *st = cut_step(&mut body, cut, half, lo, hi);
            }
        }
    }
    let tool_status: Vec<CutStatus> =
        tools.iter().map(|t| local_step(&mut body, |b| subtract_tool(b, t, lo, hi))).collect();
    let add_status: Vec<CutStatus> = adds.iter().map(|a| local_step(&mut body, |b| unite_add(b, a))).collect();
    let applied = primitives.iter().chain(&tool_status).chain(&add_status).any(|s| matches!(s, CutStatus::Applied(_)));
    if !applied {
        return Err(KernelError::Op { op: "cut_stock", message: "no cut could be applied to the stock".into() });
    }
    Ok(Built { brep: encode(&body)?, primitives, freeform: Vec::new(), tools: tool_status, adds: add_status })
}

/// The record faces that lie on an analytic surface (plane, cylinder, cone, sphere, torus), as
/// opposed to a spline.
#[must_use]
pub fn analytic_faces(body: &Brep) -> Vec<usize> {
    body.faces
        .iter()
        .enumerate()
        .filter(|(_, f)| {
            matches!(
                f.surface,
                Surface::Plane { .. }
                    | Surface::PlaneThrough { .. }
                    | Surface::Cylinder { .. }
                    | Surface::Cone { .. }
                    | Surface::Sphere { .. }
                    | Surface::Torus { .. }
                    | Surface::BallPatch { .. }
            )
        })
        .map(|(i, _)| i)
        .collect()
}
