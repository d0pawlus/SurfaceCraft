//! Cutting tools of the hybrid model: the solid of each shape and the half-space that keeps a side of a plane.

use super::vector::*;
use super::*;

/// The material side of a split by an oriented tool: `outward` is where the tool normal should
/// point; the kernel returns (behind the normal, in front of it).
pub(super) fn pick(parts: Option<(ops::Body, ops::Body)>, tool_normal: V, outward: V) -> Result<ops::Body> {
    let (back, front) = parts.ok_or_else(|| KernelError::Op { op: "split_body", message: "a side is empty".into() })?;
    Ok(if dot(tool_normal, outward) >= 0.0 { back } else { front })
}

fn plane_tool(point: V, normal: V, half: f64) -> Result<(ops::split::Tool, V)> {
    let n = unit(normal).ok_or_else(|| KernelError::Argument("plane normal is zero".into()))?;
    let u = perpendicular(n);
    let v = cross(n, u);
    let tool = ops::split::Tool {
        surface: Surface::Plane { origin: point, u_axis: u, v_axis: v },
        domain: [[-half, half], [-half, half]],
    };
    Ok((tool, n))
}

pub(super) fn half_space(body: &ops::Body, point: V, normal: V, half: f64) -> Result<ops::Body> {
    let (tool, n) = plane_tool(point, normal, half)?;
    let parts = ops::split::split_body(body, &tool).map_err(|e| op_err("split_body", e))?;
    // the plane's own normal is u × v = n, so material behind it is the back part
    pick(parts, n, n)
}

pub(super) fn shape_solid(shape: &Shape, lo: V, hi: V) -> Result<ops::Body> {
    let cs = corners(lo, hi);
    let diag = norm(sub(hi, lo)).max(1.0);
    match shape {
        Shape::Cylinder { point, axis, radius } => {
            let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
            let (mut t0, mut t1) = (f64::INFINITY, f64::NEG_INFINITY);
            for c in &cs {
                let t = dot(sub(*c, *point), a);
                t0 = t0.min(t);
                t1 = t1.max(t);
            }
            let m = 0.1 * diag + 1.0;
            let b = ops::make_cylinder([0.0, 0.0, t0 - m], *radius, t1 - t0 + 2.0 * m)
                .map_err(|e| op_err("make_cylinder", e))?;
            b.transform(&crate::to_kernel_matrix(&frame_matrix(*point, a))).map_err(|e| op_err("transform", e))?;
            Ok(b)
        }
        Shape::Cone { apex, axis, half_angle } => {
            let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
            // the cone from its apex out to past the farthest corner along the axis
            let mut depth: f64 = 0.0;
            for c in &cs {
                depth = depth.max(dot(sub(*c, *apex), a));
            }
            let depth = depth + 0.1 * diag + 1.0;
            let wide = depth * half_angle.tan();
            // the cone widens along +z (a record cone that narrows along z does not survive a
            // STEP round trip in solidcore), so it starts from a tiny disc of radius `tip`
            let tip = 0.01;
            let start = tip / half_angle.tan();
            let b = ops::make_cone([0.0; 3], tip, wide, depth - start).map_err(|e| op_err("make_cone", e))?;
            let origin = add(*apex, scale(a, start));
            b.transform(&crate::to_kernel_matrix(&frame_matrix(origin, a))).map_err(|e| op_err("transform", e))?;
            Ok(b)
        }
        Shape::Sphere { center, radius } => ops::make_sphere(*center, *radius).map_err(|e| op_err("make_sphere", e)),
        Shape::Torus { center, axis, major_radius, minor_radius } => {
            let a = unit(*axis).ok_or_else(|| KernelError::Argument("axis is zero".into()))?;
            let b = ops::make_torus([0.0; 3], *major_radius, *minor_radius).map_err(|e| op_err("make_torus", e))?;
            b.transform(&crate::to_kernel_matrix(&frame_matrix(*center, a))).map_err(|e| op_err("transform", e))?;
            Ok(b)
        }
        Shape::Plane { .. } => Err(KernelError::Argument("a plane is not a solid".into())),
    }
}

pub(super) fn check_shape(s: &Shape) -> Result<()> {
    let ok = match s {
        Shape::Plane { point, normal } => finite_all(point) && finite_all(normal),
        Shape::Cylinder { point, axis, radius } => {
            finite_all(point) && finite_all(axis) && *radius > 0.0 && radius.is_finite()
        }
        Shape::Cone { apex, axis, half_angle } => {
            finite_all(apex)
                && finite_all(axis)
                && *half_angle > 1e-6
                && *half_angle < std::f64::consts::FRAC_PI_2 - 1e-6
        }
        Shape::Sphere { center, radius } => finite_all(center) && *radius > 0.0 && radius.is_finite(),
        Shape::Torus { center, axis, major_radius, minor_radius } => {
            finite_all(center)
                && finite_all(axis)
                && *minor_radius > 0.0
                && major_radius > minor_radius
                && major_radius.is_finite()
        }
    };
    if ok { Ok(()) } else { Err(KernelError::Argument("shape parameters must be finite and in range".into())) }
}

/// The ball cut at a plane: the part of `body` in front of the plane (the side `normal` points to).
pub(crate) fn keep_front(body: &ops::Body, point: V, normal: V, half: f64) -> Result<ops::Body> {
    half_space(body, point, scale(normal, -1.0), half)
}
