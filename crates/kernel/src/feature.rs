//! Feature operations through `solidcore-ops` (design doc §6.4): extrude (with draft), revolve,
//! booleans, shell, pattern (copies + transform + `union_all`, the application recipe the
//! spike found, §1.3).
use crate::record::brep::Brep;
use crate::solid::{decode, encode};
use crate::{KernelError, Result, finite_all, op_err};
use serde::{Deserialize, Serialize};
use solidcore_ops as ops;

/// A profile curve in model space.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub enum ProfileCurve {
    /// Straight segment.
    Line {
        /// Start.
        start: [f64; 3],
        /// End.
        end: [f64; 3],
    },
    /// Arc counter-clockwise about `axis` from `start` to `end` radians, angle 0 along `reference`.
    Arc {
        /// Centre.
        center: [f64; 3],
        /// Unit axis.
        axis: [f64; 3],
        /// Unit reference direction.
        reference: [f64; 3],
        /// Radius.
        radius: f64,
        /// Start angle.
        start: f64,
        /// End angle.
        end: f64,
    },
}

fn to_ops(c: &ProfileCurve) -> Result<ops::sweep::Curve> {
    match c {
        ProfileCurve::Line { start, end } => {
            if !finite_all(start) || !finite_all(end) {
                return Err(KernelError::Argument("profile point not finite".into()));
            }
            Ok(ops::sweep::Curve::Line { start: *start, end: *end })
        }
        ProfileCurve::Arc { center, axis, reference, radius, start, end } => {
            if !finite_all(center)
                || !finite_all(axis)
                || !finite_all(reference)
                || !finite_all(&[*radius, *start, *end])
            {
                return Err(KernelError::Argument("profile arc not finite".into()));
            }
            Ok(ops::sweep::Curve::Arc {
                center: *center,
                axis: *axis,
                reference: *reference,
                radius: *radius,
                start: *start,
                end: *end,
            })
        }
    }
}

fn loop_ops(l: &[ProfileCurve]) -> Result<Vec<ops::sweep::Curve>> {
    if l.is_empty() {
        return Err(KernelError::Argument("empty profile loop".into()));
    }
    l.iter().map(to_ops).collect()
}

/// Extrudes closed loops along `direction` by `distance` with a draft angle (radians). The
/// first loop is the outer boundary; further loops are holes (subtracted).
pub fn extrude(loops: &[Vec<ProfileCurve>], direction: [f64; 3], distance: f64, draft: f64) -> Result<Brep> {
    let (outer, holes) = loops.split_first().ok_or_else(|| KernelError::Argument("no profile".into()))?;
    if !finite_all(&direction) || !distance.is_finite() || distance == 0.0 || !draft.is_finite() {
        return Err(KernelError::Argument(
            "extrude needs a finite direction, a non-zero distance and a finite draft".into(),
        ));
    }
    let (dir, dist) =
        if distance < 0.0 { ([-direction[0], -direction[1], -direction[2]], -distance) } else { (direction, distance) };
    let mut body = ops::sweep::extrude(&loop_ops(outer)?, dir, dist, draft, Default::default(), true)
        .map_err(|e| op_err("sweep::extrude", e))?;
    for h in holes {
        let cut = ops::sweep::extrude(&loop_ops(h)?, dir, dist, draft, Default::default(), true)
            .map_err(|e| op_err("sweep::extrude (hole)", e))?;
        body = ops::boolean::boolean(body, cut, ops::BooleanOp::Subtract)
            .map_err(|e| op_err("boolean (hole)", e))?
            .ok_or_else(|| KernelError::Op { op: "boolean (hole)", message: "the hole removed everything".into() })?;
    }
    encode(&body)
}

/// Revolves a closed loop about the axis through `origin` along `axis` by `angle` radians.
pub fn revolve(profile: &[ProfileCurve], origin: [f64; 3], axis: [f64; 3], angle: f64) -> Result<Brep> {
    if !finite_all(&origin) || !finite_all(&axis) || !angle.is_finite() {
        return Err(KernelError::Argument("revolve needs a finite axis and angle".into()));
    }
    let b =
        ops::sweep::revolve(&loop_ops(profile)?, origin, axis, angle, true).map_err(|e| op_err("sweep::revolve", e))?;
    encode(&b)
}

/// Boolean kinds.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BoolKind {
    /// a ∪ b.
    Union,
    /// a − b.
    Subtract,
    /// a ∩ b.
    Intersect,
}

/// A boolean of two solids (`None` result → error: empty).
pub fn boolean(a: &Brep, b: &Brep, kind: BoolKind) -> Result<Brep> {
    let op = match kind {
        BoolKind::Union => ops::BooleanOp::Union,
        BoolKind::Subtract => ops::BooleanOp::Subtract,
        BoolKind::Intersect => ops::BooleanOp::Intersect,
    };
    let r = ops::boolean::boolean(decode(a)?, decode(b)?, op)
        .map_err(|e| op_err("boolean", e))?
        .ok_or_else(|| KernelError::Op { op: "boolean", message: "the result is empty".into() })?;
    encode(&r)
}

/// Hollows a solid: the listed faces (record indices) are removed, walls of `thickness`
/// (positive outward, negative inward).
pub fn shell(body: &Brep, open_faces: &[usize], thickness: f64) -> Result<Brep> {
    if !thickness.is_finite() || thickness == 0.0 {
        return Err(KernelError::Argument("shell thickness must be finite and non-zero".into()));
    }
    if open_faces.iter().any(|&f| f >= body.faces.len()) {
        return Err(KernelError::Argument(format!("face indices must be in 0..{}", body.faces.len())));
    }
    let b = decode(body)?;
    let (_, order) = b.to_record().map_err(|e| op_err("to_record", e))?;
    ops::shell(&b, &order, open_faces, thickness).map_err(|e| op_err("shell", e))?;
    encode(&b)
}

/// `count` copies of a solid, each moved by `step` (row-major rigid 4×4) from the previous,
/// united (count includes the original).
pub fn pattern(body: &Brep, count: usize, step: &[[f64; 4]; 4]) -> Result<Brep> {
    if count == 0 || count > 1000 {
        return Err(KernelError::Argument("pattern count must be 1..=1000".into()));
    }
    if step.iter().flatten().any(|v| !v.is_finite()) {
        return Err(KernelError::Argument("pattern step must be finite".into()));
    }
    let base = decode(body)?;
    let mut bodies = vec![base.copy().map_err(|e| op_err("copy", e))?];
    let mut prev = base;
    for _ in 1..count {
        let c = prev.copy().map_err(|e| op_err("copy", e))?;
        c.transform(&crate::to_kernel_matrix(step)).map_err(|e| op_err("transform", e))?;
        bodies.push(c.copy().map_err(|e| op_err("copy", e))?);
        prev = c;
    }
    let u = ops::boolean::union_all(bodies).map_err(|e| op_err("union_all", e))?;
    encode(&u)
}

/// A skin through closed `sections` (each a loop of profile curves), `degree` across them
/// (1 = ruled), capped into a solid.
pub fn loft(sections: &[Vec<ProfileCurve>], degree: u32, cap: bool) -> Result<Brep> {
    if sections.len() < 2 {
        return Err(KernelError::Argument("a loft needs at least two sections".into()));
    }
    if !(1..=3).contains(&degree) {
        return Err(KernelError::Argument("loft degree must be 1..=3".into()));
    }
    let secs: Vec<Vec<ops::sweep::Curve>> = sections.iter().map(|s| loop_ops(s)).collect::<Result<_>>()?;
    let b = ops::sweep::loft(&secs, degree, cap).map_err(|e| op_err("sweep::loft", e))?;
    encode(&b)
}

/// `profile` (a closed loop at the start of the path) swept along `path` (an open chain). With
/// `follow` the profile turns with the path, otherwise it only moves along it.
pub fn sweep(profile: &[ProfileCurve], path: &[ProfileCurve], follow: bool) -> Result<Brep> {
    let frame = if follow { ops::sweep::SweepFrame::Follow } else { ops::sweep::SweepFrame::Translate };
    let (b, _, _) = ops::sweep::sweep_placed(&loop_ops(profile)?, &loop_ops(path)?, frame, true)
        .map_err(|e| op_err("sweep::sweep_placed", e))?;
    encode(&b)
}

/// Drafts the listed faces (record indices) by `angle` radians about the neutral plane through
/// `neutral_origin` with normal `neutral_normal`, leaning in going along `pull`.
pub fn draft(
    body: &Brep,
    faces: &[usize],
    neutral_origin: [f64; 3],
    neutral_normal: [f64; 3],
    pull: [f64; 3],
    angle: f64,
) -> Result<Brep> {
    if !finite_all(&neutral_origin) || !finite_all(&neutral_normal) || !finite_all(&pull) || !angle.is_finite() {
        return Err(KernelError::Argument("draft needs a finite plane, pull and angle".into()));
    }
    if faces.is_empty() || faces.iter().any(|&f| f >= body.faces.len()) {
        return Err(KernelError::Argument(format!("face indices must be in 0..{}", body.faces.len())));
    }
    let b = decode(body)?;
    let (_, order) = b.to_record().map_err(|e| op_err("to_record", e))?;
    ops::offset::draft(&b, &order, faces, neutral_origin, neutral_normal, pull, angle)
        .map_err(|e| op_err("draft", e))?;
    encode(&b)
}

fn plane_frame(normal: [f64; 3]) -> Result<([f64; 3], [f64; 3], [f64; 3])> {
    let len = (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
    if !finite_all(&normal) || !len.is_finite() || len < 1e-9 {
        return Err(KernelError::Argument("a plane needs a finite non-zero normal".into()));
    }
    let n = [normal[0] / len, normal[1] / len, normal[2] / len];
    // the axis least aligned with n, crossed with n, gives the first in-plane direction
    let a = if n[0].abs() <= n[1].abs() && n[0].abs() <= n[2].abs() {
        [1.0, 0.0, 0.0]
    } else if n[1].abs() <= n[2].abs() {
        [0.0, 1.0, 0.0]
    } else {
        [0.0, 0.0, 1.0]
    };
    let cross =
        |p: [f64; 3], q: [f64; 3]| [p[1] * q[2] - p[2] * q[1], p[2] * q[0] - p[0] * q[2], p[0] * q[1] - p[1] * q[0]];
    let u0 = cross(n, a);
    let ul = (u0[0] * u0[0] + u0[1] * u0[1] + u0[2] * u0[2]).sqrt();
    let u = [u0[0] / ul, u0[1] / ul, u0[2] / ul];
    let v = cross(n, u);
    Ok((n, u, v))
}

/// The body reflected in the plane through `origin` with `normal`; with `merge` united with the
/// original, else the reflection alone.
pub fn mirror(body: &Brep, origin: [f64; 3], normal: [f64; 3], merge: bool) -> Result<Brep> {
    if !finite_all(&origin) {
        return Err(KernelError::Argument("mirror origin must be finite".into()));
    }
    let (n, _, _) = plane_frame(normal)?;
    let d = n[0] * origin[0] + n[1] * origin[1] + n[2] * origin[2];
    let m = [
        [1.0 - 2.0 * n[0] * n[0], -2.0 * n[0] * n[1], -2.0 * n[0] * n[2], 2.0 * d * n[0]],
        [-2.0 * n[1] * n[0], 1.0 - 2.0 * n[1] * n[1], -2.0 * n[1] * n[2], 2.0 * d * n[1]],
        [-2.0 * n[2] * n[0], -2.0 * n[2] * n[1], 1.0 - 2.0 * n[2] * n[2], 2.0 * d * n[2]],
        [0.0, 0.0, 0.0, 1.0],
    ];
    let reflected = crate::solid::transform(body, &m)?;
    if merge { boolean(body, &reflected, BoolKind::Union) } else { Ok(reflected) }
}

/// Cuts the solid by the plane through `origin` with `normal` and keeps the front (`normal`
/// side) or the back part.
pub fn split(body: &Brep, origin: [f64; 3], normal: [f64; 3], keep_front: bool) -> Result<Brep> {
    if !finite_all(&origin) {
        return Err(KernelError::Argument("split origin must be finite".into()));
    }
    let (_, u, v) = plane_frame(normal)?;
    // the cutting patch must reach over the whole body
    let reach = body
        .vertices
        .iter()
        .map(|p| {
            let d = [p.point[0] - origin[0], p.point[1] - origin[1], p.point[2] - origin[2]];
            (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
        })
        .fold(1.0_f64, f64::max)
        * 4.0;
    let tool = ops::split::Tool {
        surface: crate::record::brep::Surface::Plane { origin, u_axis: u, v_axis: v },
        domain: [[-reach, reach], [-reach, reach]],
    };
    let (back, front) = ops::split::split_body(&decode(body)?, &tool)
        .map_err(|e| op_err("split_body", e))?
        .ok_or_else(|| KernelError::Op { op: "split_body", message: "a side of the cut is empty".into() })?;
    encode(if keep_front { &front } else { &back })
}

/// Copies of a solid, the `k`-th moved by `placements[k]` (row-major rigid 4×4, absolute),
/// united. The first placement is normally the identity.
pub fn pattern_placed(body: &Brep, placements: &[[[f64; 4]; 4]]) -> Result<Brep> {
    if placements.is_empty() || placements.len() > 1000 {
        return Err(KernelError::Argument("pattern needs 1..=1000 placements".into()));
    }
    if placements.iter().flatten().flatten().any(|v| !v.is_finite()) {
        return Err(KernelError::Argument("pattern placements must be finite".into()));
    }
    let base = decode(body)?;
    let mut bodies = Vec::with_capacity(placements.len());
    for m in placements {
        let c = base.copy().map_err(|e| op_err("copy", e))?;
        c.transform(&crate::to_kernel_matrix(m)).map_err(|e| op_err("transform", e))?;
        bodies.push(c.copy().map_err(|e| op_err("copy", e))?);
    }
    let u = ops::boolean::union_all(bodies).map_err(|e| op_err("union_all", e))?;
    encode(&u)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::solid::volume;

    fn rect(w: f64, h: f64, dx: f64, dy: f64) -> Vec<ProfileCurve> {
        let p = [[dx, dy, 0.0], [dx + w, dy, 0.0], [dx + w, dy + h, 0.0], [dx, dy + h, 0.0]];
        (0..4).map(|i| ProfileCurve::Line { start: p[i], end: p[(i + 1) % 4] }).collect()
    }

    #[test]
    fn extrude_with_hole_revolve_boolean_shell_pattern() {
        let plate =
            extrude(&[rect(40.0, 20.0, 0.0, 0.0), rect(10.0, 10.0, 15.0, 5.0)], [0.0, 0.0, 1.0], 5.0, 0.0).unwrap();
        assert!((volume(&plate).unwrap() - (800.0 - 100.0) * 5.0).abs() < 1e-3);
        let ring = revolve(
            &[
                ProfileCurve::Line { start: [2.0, 0.0, 0.0], end: [4.0, 0.0, 0.0] },
                ProfileCurve::Line { start: [4.0, 0.0, 0.0], end: [4.0, 0.0, 3.0] },
                ProfileCurve::Line { start: [4.0, 0.0, 3.0], end: [2.0, 0.0, 3.0] },
                ProfileCurve::Line { start: [2.0, 0.0, 3.0], end: [2.0, 0.0, 0.0] },
            ],
            [0.0; 3],
            [0.0, 0.0, 1.0],
            std::f64::consts::TAU,
        )
        .unwrap();
        assert!((volume(&ring).unwrap() - 36.0 * std::f64::consts::PI).abs() < 1e-3);
        let b = crate::solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
        let shelled = shell(&b, &[0], 1.0).unwrap();
        assert!((volume(&shelled).unwrap() - 694.0).abs() < 1e-3);
        let pin = crate::solid::make_cylinder([0.0; 3], 2.0, 4.0).unwrap();
        let step = [[1.0, 0.0, 0.0, 10.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]];
        let row = pattern(&pin, 4, &step).unwrap();
        assert!((volume(&row).unwrap() - 64.0 * std::f64::consts::PI).abs() < 1e-3);
        let cut = boolean(&b, &crate::solid::make_cylinder([10.0, 5.0, -1.0], 2.0, 7.0).unwrap(), BoolKind::Subtract)
            .unwrap();
        assert!((volume(&cut).unwrap() - (1000.0 - 20.0 * std::f64::consts::PI)).abs() < 1e-3);
    }
}
