//! Faces of the hybrid model: their vertices and frames, and which faces lie on a shape or a spline.

use super::vector::*;
use super::*;

/// The outward unit normal of a planar record face.
pub fn planar_face_outward(body: &Brep, face: usize) -> Result<V> {
    let f = body
        .faces
        .get(face)
        .ok_or_else(|| KernelError::Argument(format!("face indices must be in 0..{}", body.faces.len())))?;
    let n = match &f.surface {
        Surface::Plane { u_axis, v_axis, .. } => cross(*u_axis, *v_axis),
        Surface::PlaneThrough { origin, u_point, v_point } => cross(sub(*u_point, *origin), sub(*v_point, *origin)),
        _ => return Err(KernelError::Argument("only a planar face can be replaced".into())),
    };
    let n = unit(n).ok_or_else(|| KernelError::Argument("degenerate plane".into()))?;
    let forward = body
        .shells
        .iter()
        .find_map(|s| match s {
            Shell::Faces { faces, .. } => faces.iter().find(|o| o.face as usize == face).map(|o| o.forward),
            _ => None,
        })
        .unwrap_or(true);
    Ok(if forward { n } else { scale(n, -1.0) })
}

/// One face of a body as a sheet (the trimmed face on its own surface, bounded by the edges it
/// has in the body).
pub fn extract_face(body: &Brep, face: usize) -> Result<Brep> {
    let f = body
        .faces
        .get(face)
        .ok_or_else(|| KernelError::Argument(format!("face indices must be in 0..{}", body.faces.len())))?
        .clone();
    // the sheet uses the face on its natural side (the canonical walk of the record depends on it)
    if f.loops.iter().any(|l| matches!(l, Loop::Vertex { .. })) {
        return Err(KernelError::Argument("a face with a vertex loop cannot be extracted".into()));
    }
    let sheet = Brep {
        version: body.version,
        tolerance: body.tolerance,
        regions: vec![Region { shells: vec![0], void: false }],
        shells: vec![Shell::Faces {
            faces: vec![crate::record::brep::OrientedFace { face: 0, forward: true }],
            wires: vec![],
        }],
        faces: vec![f],
        edges: body.edges.clone(),
        vertices: body.vertices.clone(),
    };
    let sheet = canonicalize(sheet);
    sheet.validate().map_err(|e| op_err("record validate", e))?;
    let (b, _) = ops::Body::from_record(&sheet, false).map_err(|e| op_err("from_record", e))?;
    encode(&b)
}

pub(crate) fn face_vertices(body: &Brep, face: usize) -> Vec<V> {
    let mut out = Vec::new();
    if let Some(f) = body.faces.get(face) {
        for l in &f.loops {
            if let Loop::Edges { edges } = l {
                for oe in edges {
                    if let Some(e) = body.edges.get(oe.edge as usize) {
                        for &v in &e.vertices {
                            if let Some(v) = body.vertices.get(v as usize) {
                                out.push(v.point);
                            }
                        }
                    }
                }
            }
        }
    }
    out
}

pub(crate) fn frame_axis(f: &Frame) -> V {
    cross(f.x_axis, f.y_axis)
}

pub(crate) fn axis_distance(p: V, point: V, axis: V) -> f64 {
    let d = sub(p, point);
    norm(sub(d, scale(axis, dot(d, axis))))
}

/// The faces of `body` that lie on `shape` (same surface type, parameters within `tol`).
#[must_use]
pub fn faces_on_shape(body: &Brep, shape: &Shape, tol: f64) -> Vec<usize> {
    let mut out = Vec::new();
    for (i, f) in body.faces.iter().enumerate() {
        let hit = match (&f.surface, shape) {
            (Surface::Plane { origin, u_axis, v_axis }, Shape::Plane { point, normal }) => {
                unit(cross(*u_axis, *v_axis))
                    .is_some_and(|n| dot(n, *normal).abs() > 0.999 && dot(sub(*point, *origin), n).abs() < tol)
            }
            (Surface::Cylinder { frame, radius }, Shape::Cylinder { point, axis, radius: r }) => {
                unit(frame_axis(frame)).is_some_and(|a| dot(a, *axis).abs() > 0.999)
                    && (radius - r).abs() < tol
                    && axis_distance(frame.origin, *point, *axis) < tol
            }
            // a blend's cylinder comes back as a cone of half angle zero (SMLib's own form)
            (Surface::Cone { frame, radius_at_origin, half_angle }, Shape::Cylinder { point, axis, radius: r })
                if half_angle.abs() < 1e-9 =>
            {
                unit(frame_axis(frame)).is_some_and(|a| dot(a, *axis).abs() > 0.999)
                    && (radius_at_origin - r).abs() < tol
                    && axis_distance(frame.origin, *point, *axis) < tol
            }
            (Surface::Cone { frame, half_angle, .. }, Shape::Cone { apex, axis, half_angle: h }) => {
                unit(frame_axis(frame)).is_some_and(|a| dot(a, *axis).abs() > 0.999)
                    && (half_angle.abs() - h).abs() < 0.02
                    && axis_distance(frame.origin, *apex, *axis) < tol
            }
            (Surface::Sphere { frame, radius }, Shape::Sphere { center, radius: r }) => {
                (radius - r).abs() < tol && norm(sub(frame.origin, *center)) < tol
            }
            // the corner of three equal rounds
            (Surface::BallPatch { center: c, radius }, Shape::Sphere { center, radius: r }) => {
                (radius - r).abs() < tol && norm(sub(*c, *center)) < tol
            }
            (
                Surface::Torus { frame, major_radius, minor_radius, .. },
                Shape::Torus { center, axis, major_radius: mr, minor_radius: nr },
            ) => {
                unit(frame_axis(frame)).is_some_and(|a| dot(a, *axis).abs() > 0.999)
                    && (major_radius - mr).abs() < tol
                    && (minor_radius - nr).abs() < tol
                    && norm(sub(frame.origin, *center)) < tol
            }
            // a sweep of an arc (a pocket's rounded corner) or a revolve (a hole's wall, a drill
            // point) comes back as a rational spline: it is on the shape when its corners are
            (
                Surface::Spline { .. } | Surface::RationalSpline { .. },
                Shape::Cylinder { .. } | Shape::Cone { .. } | Shape::Torus { .. },
            ) => spline_face_on_revolved(body, i, shape, tol),
            _ => false,
        };
        if hit {
            out.push(i);
        }
    }
    out
}

/// Do all the boundary vertices of the spline face `face` lie within `tol` of the cylinder or
/// cone `shape` (and are there enough of them to say)?
fn spline_face_on_revolved(body: &Brep, face: usize, shape: &Shape, tol: f64) -> bool {
    let vs = face_vertices(body, face);
    if vs.len() < 2 {
        return false;
    }
    vs.iter().all(|&p| match shape {
        Shape::Cylinder { point, axis, radius } => {
            unit(*axis).is_some_and(|a| (axis_distance(p, *point, a) - radius).abs() < tol)
        }
        Shape::Cone { apex, axis, half_angle } => unit(*axis).is_some_and(|a| {
            let d = sub(p, *apex);
            let t = dot(d, a);
            let rho = axis_distance(p, *apex, a);
            (rho * half_angle.cos() - t * half_angle.sin()).abs() < tol
        }),
        Shape::Torus { center, axis, major_radius, minor_radius } => unit(*axis).is_some_and(|a| {
            let rho = axis_distance(p, *center, a);
            let t = dot(sub(p, *center), a);
            ((rho - major_radius).hypot(t) - minor_radius).abs() < tol
        }),
        _ => false,
    })
}

/// The faces of `body` whose boundary vertices all lie within `tol` of the spline `surface`.
pub fn faces_on_spline(body: &Brep, surface: &Surface, tol: f64) -> Result<Vec<usize>> {
    let probe = SurfaceProbe::new(surface)?;
    let mut out = Vec::new();
    for (i, f) in body.faces.iter().enumerate() {
        if !matches!(f.surface, Surface::Spline { .. } | Surface::RationalSpline { .. }) {
            continue;
        }
        let vs = face_vertices(body, i);
        if !vs.is_empty() && vs.iter().all(|&p| probe.closest(p).map(|(d, _)| d < tol).unwrap_or(false)) {
            out.push(i);
        }
    }
    Ok(out)
}
