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
