//! Small geometry helpers of the relations: anchors, directions, radii and angles of primitives.

use super::*;

// ------------------------------------------------------------------------------------------
// small geometry helpers

fn v3(a: &[f64; 3]) -> Vec3 {
    Vec3::new(a[0], a[1], a[2])
}

fn arr(v: &Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

/// Unit direction of a primitive (normal or axis); none for a sphere.
pub(super) fn direction(p: &Primitive) -> Option<Vec3> {
    let d = match p {
        Primitive::Plane { normal, .. } => v3(normal),
        Primitive::Cylinder { axis, .. } | Primitive::Cone { axis, .. } | Primitive::Torus { axis, .. } => v3(axis),
        Primitive::Sphere { .. } => return None,
    };
    normalize(d)
}

/// The point that locates the primitive (plane point, sphere or torus centre, axis point, apex).
pub(super) fn anchor(p: &Primitive) -> Vec3 {
    match p {
        Primitive::Plane { point, .. } | Primitive::Cylinder { point, .. } => v3(point),
        Primitive::Sphere { center, .. } | Primitive::Torus { center, .. } => v3(center),
        Primitive::Cone { apex, .. } => v3(apex),
    }
}

pub(super) fn move_anchor(p: &mut Primitive, d: &Vec3) {
    let q = arr(&(anchor(p) + d));
    match p {
        Primitive::Plane { point, .. } | Primitive::Cylinder { point, .. } => *point = q,
        Primitive::Sphere { center, .. } | Primitive::Torus { center, .. } => *center = q,
        Primitive::Cone { apex, .. } => *apex = q,
    }
}

pub(super) fn shift_radii(p: &mut Primitive, d: [f64; 2]) {
    match p {
        Primitive::Cylinder { radius, .. } | Primitive::Sphere { radius, .. } => *radius += d[0],
        Primitive::Torus { major_radius, minor_radius, .. } => {
            *major_radius += d[0];
            *minor_radius += d[1];
        }
        _ => {}
    }
}

pub(super) fn set_direction(p: &mut Primitive, d: &Vec3) {
    match p {
        Primitive::Plane { normal: v, .. }
        | Primitive::Cylinder { axis: v, .. }
        | Primitive::Cone { axis: v, .. }
        | Primitive::Torus { axis: v, .. } => *v = arr(d),
        Primitive::Sphere { .. } => {}
    }
}

/// Angle between two directions ignoring orientation (degrees, 0..=90).
pub(super) fn angle_unsigned(a: &Vec3, b: &Vec3) -> f64 {
    a.dot(b).abs().clamp(0.0, 1.0).acos().to_degrees()
}

/// Angle between two oriented directions (degrees, 0..=180).
pub(super) fn angle_signed(a: &Vec3, b: &Vec3) -> f64 {
    a.dot(b).clamp(-1.0, 1.0).acos().to_degrees()
}

/// Two unit vectors perpendicular to the unit vector `d` and to each other.
pub(super) fn perp_basis(d: &Vec3) -> [Vec3; 2] {
    let e1 = any_perpendicular(d);
    [e1, d.cross(&e1)]
}

pub(super) fn xyz() -> [Vec3; 3] {
    [Vec3::x(), Vec3::y(), Vec3::z()]
}

/// Distance of the axes of two axial primitives measured perpendicular to the first axis.
pub(super) fn axis_separation(pa: &Primitive, pb: &Primitive) -> Option<f64> {
    let da = direction(pa)?;
    let w = anchor(pb) - anchor(pa);
    Some((w - da * w.dot(&da)).norm())
}
