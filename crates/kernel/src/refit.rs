//! Direct refit of analytic faces (plan P8-F1): the surface of a record face as an oriented
//! analytic shape, its signed distance, and the kernel move that offsets faces along their
//! outward normals (`ops::offset_faces`), with the relations (parallel, perpendicular, coaxial,
//! equal radius) counted so a caller can show they survive the rebuild.
//!
//! The only motion the kernel offers for a face is an offset along its normal: a plane moves, a
//! cylinder, sphere or torus changes its radius, a cone slides along its axis. Every other
//! parameter (tilt, axis position) is out of reach of this module, which also means that the
//! orientation relations of the body are kept by construction.
use crate::record::brep::{Brep, Frame, Shell, Surface};
use crate::{KernelError, Result, finite_all, op_err};
use solidcore_ops as ops;

type V = [f64; 3];

fn sub(a: V, b: V) -> V {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}
fn add(a: V, b: V) -> V {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}
fn scale(a: V, s: f64) -> V {
    [a[0] * s, a[1] * s, a[2] * s]
}
fn dot(a: V, b: V) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}
fn cross(a: V, b: V) -> V {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}
fn norm(a: V) -> f64 {
    dot(a, a).sqrt()
}
fn unit(a: V) -> Option<V> {
    let n = norm(a);
    (n.is_finite() && n > 1e-12).then(|| scale(a, 1.0 / n))
}
fn perpendicular(a: V) -> V {
    let t = if a[0].abs() < 0.6 { [1.0, 0.0, 0.0] } else { [0.0, 1.0, 0.0] };
    unit(sub(t, scale(a, dot(t, a)))).unwrap_or([0.0, 0.0, 1.0])
}
/// Distance of `p` from the axis line through `point` along unit `axis`.
fn axis_distance(p: V, point: V, axis: V) -> f64 {
    let d = sub(p, point);
    norm(sub(d, scale(axis, dot(d, axis))))
}
/// Unit vector from the axis line to `p` (any perpendicular on the axis itself).
fn radial(p: V, point: V, axis: V) -> V {
    let d = sub(p, point);
    unit(sub(d, scale(axis, dot(d, axis)))).unwrap_or_else(|| perpendicular(axis))
}
fn frame_axis(f: &Frame) -> Option<V> {
    unit(cross(f.x_axis, f.y_axis))
}

/// Two shapes are the same surface within this length (mm) when the kernel hands a face back.
pub const SAME_SURFACE_TOL: f64 = 1e-5;

/// The analytic surface of a face, oriented: distances are positive on the side the outward
/// normal (out of the material) points to.
#[derive(Clone, Debug, PartialEq)]
pub enum FaceShape {
    /// A plane through `point` with unit outward `normal`.
    Plane {
        /// A point of the plane.
        point: V,
        /// Unit outward normal.
        normal: V,
    },
    /// A cylinder about the axis through `point` along unit `axis`.
    Cylinder {
        /// A point of the axis.
        point: V,
        /// Unit axis.
        axis: V,
        /// Radius.
        radius: f64,
        /// The material is inside the cylinder (a boss, an outer fillet); false for a hole.
        convex: bool,
    },
    /// A cone: `apex`, unit `axis` pointing into the opening, `half_angle` in (0, π/2).
    Cone {
        /// Apex.
        apex: V,
        /// Unit axis into the opening.
        axis: V,
        /// Half angle, radians.
        half_angle: f64,
        /// The material is inside the cone.
        convex: bool,
    },
    /// A sphere.
    Sphere {
        /// Centre.
        center: V,
        /// Radius.
        radius: f64,
        /// The material is inside the sphere.
        convex: bool,
    },
    /// A ring torus.
    Torus {
        /// Centre.
        center: V,
        /// Unit axis.
        axis: V,
        /// Major radius.
        major_radius: f64,
        /// Minor radius.
        minor_radius: f64,
        /// The material is inside the tube.
        convex: bool,
    },
}

impl FaceShape {
    /// `plane`, `cylinder`, `cone`, `sphere` or `torus`.
    #[must_use]
    pub fn kind(&self) -> &'static str {
        match self {
            FaceShape::Plane { .. } => "plane",
            FaceShape::Cylinder { .. } => "cylinder",
            FaceShape::Cone { .. } => "cone",
            FaceShape::Sphere { .. } => "sphere",
            FaceShape::Torus { .. } => "torus",
        }
    }

    fn sense(&self) -> f64 {
        match self {
            FaceShape::Plane { .. } => 1.0,
            FaceShape::Cylinder { convex, .. }
            | FaceShape::Cone { convex, .. }
            | FaceShape::Sphere { convex, .. }
            | FaceShape::Torus { convex, .. } => {
                if *convex {
                    1.0
                } else {
                    -1.0
                }
            }
        }
    }

    /// Signed distance of `p` from the surface, positive on the outward side (out of the material).
    #[must_use]
    pub fn signed_distance(&self, p: V) -> f64 {
        match self {
            FaceShape::Plane { point, normal } => dot(*normal, sub(p, *point)),
            FaceShape::Cylinder { point, axis, radius, .. } => {
                self.sense() * (axis_distance(p, *point, *axis) - radius)
            }
            FaceShape::Cone { apex, axis, half_angle, .. } => {
                let s = dot(sub(p, *apex), *axis);
                let rho = axis_distance(p, *apex, *axis);
                self.sense() * (rho * half_angle.cos() - s * half_angle.sin())
            }
            FaceShape::Sphere { center, radius, .. } => self.sense() * (norm(sub(p, *center)) - radius),
            FaceShape::Torus { center, axis, major_radius, minor_radius, .. } => {
                let q = sub(p, *center);
                let t = dot(q, *axis);
                let rho = norm(sub(q, scale(*axis, t)));
                self.sense() * ((rho - major_radius).hypot(t) - minor_radius)
            }
        }
    }

    /// Unit outward normal of the surface at the point of it nearest `p`.
    #[must_use]
    pub fn outward_normal(&self, p: V) -> V {
        let n = match self {
            FaceShape::Plane { normal, .. } => *normal,
            FaceShape::Cylinder { point, axis, .. } => radial(p, *point, *axis),
            FaceShape::Cone { apex, axis, half_angle, .. } => {
                let r = radial(p, *apex, *axis);
                sub(scale(r, half_angle.cos()), scale(*axis, half_angle.sin()))
            }
            FaceShape::Sphere { center, .. } => unit(sub(p, *center)).unwrap_or([0.0, 0.0, 1.0]),
            FaceShape::Torus { center, axis, major_radius, .. } => {
                let tube_centre = add(*center, scale(radial(p, *center, *axis), *major_radius));
                unit(sub(p, tube_centre)).unwrap_or(*axis)
            }
        };
        if matches!(self, FaceShape::Plane { .. }) { n } else { scale(n, self.sense()) }
    }

    /// The shape with the surface moved `d` along its outward normal; `None` when a radius would
    /// vanish.
    #[must_use]
    pub fn offset(&self, d: f64) -> Option<FaceShape> {
        let s = self.sense();
        Some(match self {
            FaceShape::Plane { point, normal } => {
                FaceShape::Plane { point: add(*point, scale(*normal, d)), normal: *normal }
            }
            FaceShape::Cylinder { point, axis, radius, convex } => {
                let r = radius + s * d;
                if !(r > 1e-9) {
                    return None;
                }
                FaceShape::Cylinder { point: *point, axis: *axis, radius: r, convex: *convex }
            }
            FaceShape::Cone { apex, axis, half_angle, convex } => FaceShape::Cone {
                apex: sub(*apex, scale(*axis, s * d / half_angle.sin())),
                axis: *axis,
                half_angle: *half_angle,
                convex: *convex,
            },
            FaceShape::Sphere { center, radius, convex } => {
                let r = radius + s * d;
                if !(r > 1e-9) {
                    return None;
                }
                FaceShape::Sphere { center: *center, radius: r, convex: *convex }
            }
            FaceShape::Torus { center, axis, major_radius, minor_radius, convex } => {
                let r = minor_radius + s * d;
                if !(r > 1e-9) {
                    return None;
                }
                FaceShape::Torus {
                    center: *center,
                    axis: *axis,
                    major_radius: *major_radius,
                    minor_radius: r,
                    convex: *convex,
                }
            }
        })
    }

    /// The named numbers that describe the surface (for a before/after report). The plane's
    /// `offset` is the distance of the plane from the origin along its outward normal.
    #[must_use]
    pub fn params(&self) -> Vec<(&'static str, f64)> {
        match self {
            FaceShape::Plane { point, normal } => vec![("offset", dot(*normal, *point))],
            FaceShape::Cylinder { radius, .. } => vec![("radius", *radius)],
            FaceShape::Cone { apex, axis, half_angle, .. } => {
                vec![("half_angle_deg", half_angle.to_degrees()), ("apex_axial", dot(*apex, *axis))]
            }
            FaceShape::Sphere { radius, .. } => vec![("radius", *radius)],
            FaceShape::Torus { major_radius, minor_radius, .. } => {
                vec![("major_radius", *major_radius), ("minor_radius", *minor_radius)]
            }
        }
    }

    /// The radius that an equal-radius relation compares: a cylinder's or sphere's radius, a
    /// torus's tube radius.
    #[must_use]
    pub fn radius(&self) -> Option<f64> {
        match self {
            FaceShape::Cylinder { radius, .. } | FaceShape::Sphere { radius, .. } => Some(*radius),
            FaceShape::Torus { minor_radius, .. } => Some(*minor_radius),
            FaceShape::Plane { .. } | FaceShape::Cone { .. } => None,
        }
    }

    /// `+1` when moving the face outward by `d` grows the radius by `d` (material inside the
    /// round surface), `−1` when it shrinks it (a hole); `+1` for faces without a radius.
    #[must_use]
    pub fn radius_sign(&self) -> f64 {
        self.sense()
    }

    /// The normal of a plane, the axis of a cylinder, cone or torus (the direction relations speak of).
    #[must_use]
    pub fn direction(&self) -> Option<V> {
        match self {
            FaceShape::Plane { normal, .. } => Some(*normal),
            FaceShape::Cylinder { axis, .. } | FaceShape::Cone { axis, .. } | FaceShape::Torus { axis, .. } => {
                Some(*axis)
            }
            FaceShape::Sphere { .. } => None,
        }
    }

    /// A point of the axis (cylinder, cone, torus) for a coaxial relation.
    fn axis_point(&self) -> Option<V> {
        match self {
            FaceShape::Cylinder { point, .. } => Some(*point),
            FaceShape::Cone { apex, .. } => Some(*apex),
            FaceShape::Torus { center, .. } => Some(*center),
            FaceShape::Plane { .. } | FaceShape::Sphere { .. } => None,
        }
    }

    /// Is `other` the same surface within `tol` (same kind and side, same position)?
    #[must_use]
    pub fn same_surface(&self, other: &FaceShape, tol: f64) -> bool {
        let par = |a: V, b: V| dot(a, b).abs() > 1.0 - 1e-9;
        match (self, other) {
            (FaceShape::Plane { point: pa, normal: na }, FaceShape::Plane { point: pb, normal: nb }) => {
                dot(*na, *nb) > 1.0 - 1e-9 && dot(*na, sub(*pb, *pa)).abs() < tol
            }
            (
                FaceShape::Cylinder { point: pa, axis: aa, radius: ra, convex: ca },
                FaceShape::Cylinder { point: pb, axis: ab, radius: rb, convex: cb },
            ) => ca == cb && par(*aa, *ab) && (ra - rb).abs() < tol && axis_distance(*pb, *pa, *aa) < tol,
            (
                FaceShape::Cone { apex: pa, axis: aa, half_angle: ha, convex: ca },
                FaceShape::Cone { apex: pb, axis: ab, half_angle: hb, convex: cb },
            ) => ca == cb && dot(*aa, *ab) > 1.0 - 1e-9 && (ha - hb).abs() < 1e-7 && norm(sub(*pa, *pb)) < tol,
            (
                FaceShape::Sphere { center: ca, radius: ra, convex: xa },
                FaceShape::Sphere { center: cb, radius: rb, convex: xb },
            ) => xa == xb && (ra - rb).abs() < tol && norm(sub(*ca, *cb)) < tol,
            (
                FaceShape::Torus { center: ca, axis: aa, major_radius: ma, minor_radius: na, convex: xa },
                FaceShape::Torus { center: cb, axis: ab, major_radius: mb, minor_radius: nb, convex: xb },
            ) => {
                xa == xb && par(*aa, *ab) && (ma - mb).abs() < tol && (na - nb).abs() < tol && norm(sub(*ca, *cb)) < tol
            }
            _ => false,
        }
    }
}

fn oriented(body: &Brep) -> Vec<bool> {
    let mut forward = vec![true; body.faces.len()];
    for s in &body.shells {
        if let Shell::Faces { faces, .. } = s {
            for of in faces {
                if let Some(f) = forward.get_mut(of.face as usize) {
                    *f = of.forward;
                }
            }
        }
    }
    forward
}

/// The oriented analytic shape of every face of `body`, or the reason a face is not one (a
/// freeform surface, a corner patch, a swept profile, values that are not finite). A bounded
/// number of faces only: more than [`MAX_FACES`] is an error.
pub fn face_shapes(body: &Brep) -> Result<Vec<std::result::Result<FaceShape, String>>> {
    if body.faces.len() > MAX_FACES {
        return Err(KernelError::Argument(format!("a body of more than {MAX_FACES} faces is not refitted")));
    }
    let forward = oriented(body);
    let mut out = Vec::with_capacity(body.faces.len());
    for (i, f) in body.faces.iter().enumerate() {
        let fwd = forward.get(i).copied().unwrap_or(true);
        let flip = |n: V| if fwd { n } else { scale(n, -1.0) };
        let shape = match &f.surface {
            Surface::Plane { origin, u_axis, v_axis } => unit(cross(*u_axis, *v_axis))
                .filter(|_| finite_all(origin))
                .map(|n| FaceShape::Plane { point: *origin, normal: flip(n) })
                .ok_or_else(|| "degenerate plane".to_string()),
            Surface::PlaneThrough { origin, u_point, v_point } => {
                unit(cross(sub(*u_point, *origin), sub(*v_point, *origin)))
                    .filter(|_| finite_all(origin))
                    .map(|n| FaceShape::Plane { point: *origin, normal: flip(n) })
                    .ok_or_else(|| "degenerate plane".to_string())
            }
            Surface::Cylinder { frame, radius } => frame_axis(frame)
                .filter(|_| radius.is_finite() && *radius > 0.0 && finite_all(&frame.origin))
                .map(|axis| FaceShape::Cylinder { point: frame.origin, axis, radius: *radius, convex: fwd })
                .ok_or_else(|| "degenerate cylinder".to_string()),
            Surface::Cone { frame, radius_at_origin, half_angle } => {
                let a = half_angle.abs();
                match frame_axis(frame) {
                    Some(z) if a.is_finite() && finite_all(&frame.origin) && radius_at_origin.is_finite() => {
                        if a < 1e-9 {
                            // a blend's cylinder comes back as a cone of half angle zero
                            if *radius_at_origin > 0.0 {
                                Ok(FaceShape::Cylinder {
                                    point: frame.origin,
                                    axis: z,
                                    radius: *radius_at_origin,
                                    convex: fwd,
                                })
                            } else {
                                Err("degenerate cone".to_string())
                            }
                        } else if a < std::f64::consts::FRAC_PI_2 - 1e-6 {
                            // r(v) = r0 + v tan(h): the apex is at v = −r0 / tan(h); it widens along +z
                            // for a positive half angle, along −z for a negative one
                            let v0 = -radius_at_origin / half_angle.tan();
                            let apex = add(frame.origin, scale(z, v0));
                            let axis = if *half_angle > 0.0 { z } else { scale(z, -1.0) };
                            Ok(FaceShape::Cone { apex, axis, half_angle: a, convex: fwd })
                        } else {
                            Err("a cone this wide is a plane".to_string())
                        }
                    }
                    _ => Err("degenerate cone".to_string()),
                }
            }
            Surface::Sphere { frame, radius } => {
                if radius.is_finite() && *radius > 0.0 && finite_all(&frame.origin) {
                    Ok(FaceShape::Sphere { center: frame.origin, radius: *radius, convex: fwd })
                } else {
                    Err("degenerate sphere".to_string())
                }
            }
            Surface::Torus { frame, major_radius, minor_radius, .. } => frame_axis(frame)
                .filter(|_| {
                    major_radius.is_finite()
                        && minor_radius.is_finite()
                        && *minor_radius > 0.0
                        && major_radius > minor_radius
                        && finite_all(&frame.origin)
                })
                .map(|axis| FaceShape::Torus {
                    center: frame.origin,
                    axis,
                    major_radius: *major_radius,
                    minor_radius: *minor_radius,
                    convex: fwd,
                })
                .ok_or_else(|| "degenerate torus".to_string()),
            Surface::BallPatch { .. } => Err("a corner patch of three rounds is not an analytic face".to_string()),
            Surface::Spline { .. } | Surface::RationalSpline { .. } => {
                Err("a freeform (spline) surface has no analytic parameters".to_string())
            }
            Surface::PlacedRevolution { .. } => Err("a swept profile is a freeform surface".to_string()),
        };
        out.push(shape);
    }
    Ok(out)
}

/// Most faces a body may have for [`face_shapes`] and [`relation_counts`].
pub const MAX_FACES: usize = 3000;

/// How many pairs of faces stand in each relation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RelationCounts {
    /// Pairs whose normals or axes are parallel.
    pub parallel: usize,
    /// Pairs whose normals or axes are perpendicular.
    pub perpendicular: usize,
    /// Pairs of cylinders, cones or tori on one axis line.
    pub coaxial: usize,
    /// Pairs of cylinders, spheres or tori (tube) of equal radius.
    pub equal_radius: usize,
}

/// Counts the relations among the analytic faces `shapes` (`None` entries are skipped). Angular
/// tolerance `1e-6` rad, length tolerance [`SAME_SURFACE_TOL`] scaled by the radius.
#[must_use]
pub fn relation_counts(shapes: &[Option<&FaceShape>]) -> RelationCounts {
    let mut c = RelationCounts::default();
    for (i, a) in shapes.iter().enumerate() {
        let Some(a) = a else { continue };
        for b in shapes.iter().skip(i + 1).flatten() {
            if let (Some(da), Some(db)) = (a.direction(), b.direction()) {
                let d = dot(da, db).abs();
                if d > 1.0 - 1e-12 {
                    c.parallel += 1;
                    if let (Some(pa), Some(pb)) = (a.axis_point(), b.axis_point())
                        && axis_distance(pb, pa, da) < 1e-4
                    {
                        c.coaxial += 1;
                    }
                } else if d < 1e-6 {
                    c.perpendicular += 1;
                }
            }
            if let (Some(ra), Some(rb)) = (a.radius(), b.radius())
                && (ra - rb).abs() < 1e-5 * (1.0 + ra.abs())
            {
                c.equal_radius += 1;
            }
        }
    }
    c
}

/// One kernel move: the faces (record indices of the body given) of one surface, offset along
/// their outward normals.
#[derive(Clone, Debug, PartialEq)]
pub struct FaceMove {
    /// Faces on one surface (twins of a split face move together).
    pub faces: Vec<usize>,
    /// Distance along the outward normal (negative: into the material).
    pub distance: f64,
}

/// What became of one [`FaceMove`].
#[derive(Clone, Debug, PartialEq)]
pub struct MoveOutcome {
    /// Was it applied?
    pub applied: bool,
    /// Why not, when it was not.
    pub message: Option<String>,
}

fn find_faces(body: &Brep, want: &FaceShape) -> Vec<usize> {
    match face_shapes(body) {
        Ok(shapes) => shapes
            .iter()
            .enumerate()
            .filter_map(|(i, s)| s.as_ref().ok().filter(|s| s.same_surface(want, SAME_SURFACE_TOL)).map(|_| i))
            .collect(),
        Err(_) => Vec::new(),
    }
}

/// Offsets groups of faces, one kernel call per group (`ops::offset_faces`; the faces around a
/// moved one are extended or trimmed to meet it, tangent blends follow). Each group is found
/// again in the running result by its surface, so earlier moves cannot shift the indices; a
/// group that cannot be found or that the kernel refuses is reported and the body goes on as it
/// was. Returns the rebuilt body and one outcome per move.
pub fn offset_face_groups(body: &Brep, moves: &[FaceMove]) -> Result<(Brep, Vec<MoveOutcome>)> {
    if moves.len() > MAX_FACES {
        return Err(KernelError::Argument("too many face moves".into()));
    }
    let shapes = face_shapes(body)?;
    let (kb, mut order) = ops::Body::from_record(body, false).map_err(|e| op_err("from_record", e))?;
    let mut current: Option<Brep> = None;
    let mut out = Vec::with_capacity(moves.len());
    for m in moves {
        let fail = |msg: String| MoveOutcome { applied: false, message: Some(msg) };
        if !m.distance.is_finite() || m.distance == 0.0 || m.faces.is_empty() {
            out.push(fail("the distance must be finite and not zero".into()));
            continue;
        }
        let Some(first) = m.faces.first().and_then(|&f| shapes.get(f)) else {
            out.push(fail("face out of range".into()));
            continue;
        };
        let shape = match first {
            Ok(s) => s,
            Err(reason) => {
                out.push(fail(reason.clone()));
                continue;
            }
        };
        // the faces to move: the given indices until something has moved, then the faces of the
        // running result that lie on the original surface
        let indices = match &current {
            None => m.faces.clone(),
            Some(rec) => {
                let found = find_faces(rec, shape);
                if found.len() != m.faces.len() {
                    out.push(fail(format!(
                        "the face cannot be found again after earlier moves ({} found, {} expected)",
                        found.len(),
                        m.faces.len()
                    )));
                    continue;
                }
                found
            }
        };
        match ops::offset_faces(&kb, &order, &indices, m.distance) {
            Ok(()) => match kb.to_record() {
                Ok((rec, ord)) => {
                    order = ord;
                    current = Some(rec);
                    out.push(MoveOutcome { applied: true, message: None });
                }
                Err(e) => return Err(op_err("to_record", e)),
            },
            Err(e) => out.push(fail(format!("the kernel refused the move: {e}"))),
        }
    }
    let rebuilt = match current {
        Some(r) => r,
        None => body.clone(),
    };
    Ok((rebuilt, out))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::feature::{BoolKind, boolean};
    use crate::solid::{make_box, make_cylinder, volume};

    fn boss() -> Brep {
        let b = make_box([0.0; 3], [40.0, 30.0, 10.0]).unwrap();
        let c = make_cylinder([20.0, 15.0, 10.0], 6.0, 8.0).unwrap();
        boolean(&b, &c, BoolKind::Union).unwrap()
    }

    #[test]
    fn shapes_are_oriented_outward() {
        let b = boss();
        let shapes = face_shapes(&b).unwrap();
        assert_eq!(shapes.len(), b.faces.len());
        let on_boss = [20.0, 15.0, 14.0];
        let cyl = shapes.iter().flatten().find(|s| s.kind() == "cylinder").unwrap();
        assert!(matches!(cyl, FaceShape::Cylinder { convex: true, radius, .. } if (radius - 6.0).abs() < 1e-9));
        assert!(cyl.signed_distance([26.5, 15.0, 14.0]) > 0.0 && cyl.signed_distance(on_boss) < 0.0);
        let planes = shapes.iter().flatten().filter(|s| s.kind() == "plane").count();
        assert_eq!(planes, 7);
    }

    #[test]
    fn offset_moves_a_plane_and_a_boss_wall() {
        let b = boss();
        let shapes = face_shapes(&b).unwrap();
        let find = |pred: &dyn Fn(&FaceShape) -> bool| shapes.iter().position(|s| s.as_ref().is_ok_and(pred)).unwrap();
        let plus_x = find(&|s| matches!(s, FaceShape::Plane { normal, point } if normal[0] > 0.99 && point[0] > 39.0));
        let wall = find(&|s| matches!(s, FaceShape::Cylinder { .. }));
        let moves = [FaceMove { faces: vec![plus_x], distance: 0.4 }, FaceMove { faces: vec![wall], distance: 0.3 }];
        let (nb, outcomes) = offset_face_groups(&b, &moves).unwrap();
        assert!(outcomes.iter().all(|o| o.applied), "{outcomes:?}");
        assert_eq!(nb.faces.len(), b.faces.len());
        let ns = face_shapes(&nb).unwrap();
        let x = ns
            .iter()
            .flatten()
            .find_map(|s| match s {
                FaceShape::Plane { normal, point } if normal[0] > 0.99 => Some(point[0]),
                _ => None,
            })
            .unwrap();
        assert!((x - 40.4).abs() < 1e-6, "{x}");
        let r = ns
            .iter()
            .flatten()
            .find_map(|s| match s {
                FaceShape::Cylinder { radius, .. } => Some(*radius),
                _ => None,
            })
            .unwrap();
        assert!((r - 6.3).abs() < 1e-6, "{r}");
        let expect = 40.4 * 30.0 * 10.0 + std::f64::consts::PI * 6.3 * 6.3 * 8.0;
        assert!((volume(&nb).unwrap() - expect).abs() < 1e-3);
        let before = relation_counts(&shapes.iter().map(|s| s.as_ref().ok()).collect::<Vec<_>>());
        let after = relation_counts(&ns.iter().map(|s| s.as_ref().ok()).collect::<Vec<_>>());
        assert_eq!(before, after);
    }

    #[test]
    fn offset_of_shapes_matches_the_signed_distance() {
        let cone = FaceShape::Cone { apex: [0.0; 3], axis: [0.0, 0.0, 1.0], half_angle: 0.4, convex: true };
        let p = [3.0, 1.0, 5.0];
        let moved = cone.offset(0.25).unwrap();
        assert!((moved.signed_distance(p) - (cone.signed_distance(p) - 0.25)).abs() < 1e-12);
        let hole = FaceShape::Cylinder { point: [0.0; 3], axis: [0.0, 0.0, 1.0], radius: 4.0, convex: false };
        let moved = hole.offset(0.2).unwrap();
        assert!((moved.signed_distance(p) - (hole.signed_distance(p) - 0.2)).abs() < 1e-12);
        assert!(matches!(moved, FaceShape::Cylinder { radius, .. } if (radius - 3.8).abs() < 1e-12));
        let ring = FaceShape::Torus {
            center: [0.0; 3],
            axis: [0.0, 0.0, 1.0],
            major_radius: 5.0,
            minor_radius: 1.0,
            convex: true,
        };
        let moved = ring.offset(0.5).unwrap();
        assert!((moved.signed_distance(p) - (ring.signed_distance(p) - 0.5)).abs() < 1e-12);
        assert!(hole.offset(5.0).is_none());
    }
}
