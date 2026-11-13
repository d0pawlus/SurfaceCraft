//! The analytic surface a merged face lies on: its record form (label and NURBS form), parameter
//! box and side, the parameters of a point on it, and the distance of a point from it.
use super::revolve::{Arcs, Profile, surface as revolved};
use crate::hybrid::{Shape, V, add, cross, dot, norm, perpendicular, scale, sub, unit};
use crate::record::brep::{Frame, Surface};
use crate::{KernelError, Result};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// How far (radians) a face may come to the seam of a periodic surface, or to a pole.
const SEAM_MARGIN: f64 = 0.1;

/// The surface of the new face, ready for the record.
pub(super) struct Placement {
    /// The analytic label.
    pub surface: Surface,
    /// Its NURBS form (a plane has none).
    pub nurbs: Option<Surface>,
    /// The parameter box, in the parameters of the NURBS form (the plane's own for a plane).
    pub domain: [[f64; 2]; 2],
    /// The surface's natural normal points against the side the patches face.
    pub flip: bool,
    map: Map,
    forms: Option<(Arcs, Profile)>,
}

/// The geometry of the placed surface.
enum Map {
    Plane {
        origin: V,
        u: V,
        v: V,
    },
    /// Cylinder or cone: `origin`, `x` towards the angle zero, `y`, axis `z`.
    Around {
        origin: V,
        x: V,
        y: V,
        z: V,
    },
    Sphere {
        center: V,
        x: V,
        y: V,
        z: V,
    },
}

fn angle(q: V, x: V, y: V) -> f64 {
    let a = dot(q, y).atan2(dot(q, x));
    if a < 0.0 { a + TAU } else { a }
}

impl Placement {
    /// The geometric (angle or plane u, height or latitude or plane v) of `p`.
    fn geo(&self, p: V) -> [f64; 2] {
        match &self.map {
            Map::Plane { origin, u, v } => {
                let q = sub(p, *origin);
                [dot(q, *u), dot(q, *v)]
            }
            Map::Around { origin, x, y, z } => {
                let q = sub(p, *origin);
                [angle(q, *x, *y), dot(q, *z)]
            }
            Map::Sphere { center, x, y, z } => {
                let q = sub(p, *center);
                let r = norm(q).max(1e-300);
                [angle(q, *x, *y), (dot(q, *z) / r).clamp(-1.0, 1.0).asin()]
            }
        }
    }

    /// Is it the plane (whose pcurves are the edges' own projections)?
    pub(super) fn is_plane(&self) -> bool {
        matches!(self.map, Map::Plane { .. })
    }

    /// The parameters of `p` in the face's surface (the NURBS form's, or the plane's).
    pub(super) fn param(&self, p: V) -> [f64; 2] {
        let g = self.geo(p);
        match &self.forms {
            Some((u, v)) => [u.param(g[0]), v.param(g[1])],
            None => g,
        }
    }
}

/// Distance of `p` from the shape's surface (`None` for a shape the merge does not place).
pub(super) fn distance(shape: &Shape, p: V) -> Option<f64> {
    match shape {
        Shape::Plane { point, normal } => Some(dot(sub(p, *point), unit(*normal)?).abs()),
        Shape::Cylinder { point, axis, radius } => Some((radial(p, *point, unit(*axis)?).1 - radius).abs()),
        Shape::Cone { apex, axis, half_angle } => {
            let a = unit(*axis)?;
            let (t, rho) = (dot(sub(p, *apex), a), radial(p, *apex, a).1);
            Some((rho * half_angle.cos() - t * half_angle.sin()).abs())
        }
        Shape::Sphere { center, radius } => Some((norm(sub(p, *center)) - radius).abs()),
        Shape::Torus { .. } => None,
    }
}

/// The unit direction from the axis to `p` (None on the axis) and the distance from the axis.
fn radial(p: V, point: V, axis: V) -> (Option<V>, f64) {
    let q = sub(p, point);
    let r = sub(q, scale(axis, dot(q, axis)));
    (unit(r), norm(r))
}

fn invalid(what: &str) -> KernelError {
    KernelError::Argument(what.to_string())
}

/// The box of the geometric parameters of `points`, padded by a fraction of its size.
fn padded_box(placement: &Placement, points: &[V]) -> [[f64; 2]; 2] {
    let mut b = [[f64::INFINITY, f64::NEG_INFINITY], [f64::INFINITY, f64::NEG_INFINITY]];
    for p in points {
        for (k, c) in placement.geo(*p).into_iter().enumerate() {
            if let Some(r) = b.get_mut(k) {
                *r = [r[0].min(c), r[1].max(c)];
            }
        }
    }
    b.map(|[lo, hi]| {
        let pad = 0.02 * (hi - lo) + 1e-4;
        [lo - pad, hi + pad]
    })
}

/// Places `shape` for a face whose boundary samples are `points` and whose patches face
/// `outward` on average.
///
/// # Errors
/// A shape that is degenerate or not finite, a torus, and a face that would reach a seam or a
/// pole of its parameters (it wraps round the axis, or a sphere cap of 90 degrees or more).
pub(super) fn place(shape: &Shape, points: &[V], outward: V) -> Result<Placement> {
    let n = i32::try_from(points.len()).unwrap_or(i32::MAX);
    let c = scale(points.iter().fold([0.0; 3], |s, p| add(s, *p)), 1.0 / f64::from(n.max(1)));
    let mut placed = match shape {
        Shape::Plane { point, normal } => plane(*point, *normal, outward)?,
        Shape::Cylinder { point, axis, radius } => {
            let a = unit(*axis).ok_or_else(|| invalid("degenerate axis"))?;
            around(*point, a, c, outward, Round::Cylinder(*radius))?
        }
        Shape::Cone { apex, axis, half_angle } => {
            let a = unit(*axis).ok_or_else(|| invalid("degenerate axis"))?;
            let t0 = dot(sub(c, *apex), a);
            around(add(*apex, scale(a, t0)), a, c, outward, Round::Cone { t0, half_angle: *half_angle })?
        }
        Shape::Sphere { center, radius } => sphere(*center, *radius, c, outward)?,
        Shape::Torus { .. } => return Err(invalid("a torus cannot be placed yet")),
    };
    let geo = padded_box(&placed, points);
    check_reach(&placed, geo)?;
    finish(&mut placed, geo, shape);
    Ok(placed)
}

fn plane(point: V, normal: V, outward: V) -> Result<Placement> {
    let n = unit(normal).ok_or_else(|| invalid("degenerate plane normal"))?;
    let n = if dot(n, outward) < 0.0 { scale(n, -1.0) } else { n };
    let u = perpendicular(n);
    let v = cross(n, u);
    Ok(Placement {
        surface: Surface::Plane { origin: point, u_axis: u, v_axis: v },
        nurbs: None,
        domain: [[0.0; 2]; 2],
        flip: false,
        map: Map::Plane { origin: point, u, v },
        forms: None,
    })
}

/// Which surface of revolution about an axis.
#[derive(Clone, Copy)]
enum Round {
    Cylinder(f64),
    /// The frame origin is `t0` along the axis from the apex.
    Cone {
        t0: f64,
        half_angle: f64,
    },
}

fn around(origin: V, axis: V, c: V, outward: V, round: Round) -> Result<Placement> {
    let d = radial(c, origin, axis).0.ok_or_else(|| invalid("the face lies on the axis"))?;
    let x = scale(d, -1.0);
    let y = cross(axis, x);
    let frame = Frame { origin, x_axis: x, y_axis: y };
    let (surface, lean) = match round {
        Round::Cylinder(radius) => (Surface::Cylinder { frame, radius }, 0.0),
        Round::Cone { t0, half_angle } => {
            (Surface::Cone { frame, radius_at_origin: t0 * half_angle.tan(), half_angle }, -half_angle.tan())
        }
    };
    Ok(Placement {
        surface,
        nurbs: None,
        domain: [[0.0; 2]; 2],
        flip: dot(add(d, scale(axis, lean)), outward) < 0.0,
        map: Map::Around { origin, x, y, z: axis },
        forms: None,
    })
}

fn sphere(center: V, radius: f64, c: V, outward: V) -> Result<Placement> {
    let d = unit(sub(c, center)).ok_or_else(|| invalid("the face is at the sphere's centre"))?;
    let (x, z) = (scale(d, -1.0), perpendicular(d));
    let y = cross(z, x);
    Ok(Placement {
        surface: Surface::Sphere { frame: Frame { origin: center, x_axis: x, y_axis: y }, radius },
        nurbs: None,
        domain: [[0.0; 2]; 2],
        flip: dot(d, outward) < 0.0,
        map: Map::Sphere { center, x, y, z },
        forms: None,
    })
}

/// The face must stay clear of the seam (angle near 0 or 2 pi) and of a sphere's poles.
fn check_reach(p: &Placement, geo: [[f64; 2]; 2]) -> Result<()> {
    let [[u0, u1], [v0, v1]] = geo;
    if !(u0.is_finite() && u1.is_finite() && v0.is_finite() && v1.is_finite()) {
        return Err(invalid("the face's parameter box is not finite"));
    }
    if !p.is_plane() && (u0 < SEAM_MARGIN || u1 > TAU - SEAM_MARGIN) {
        return Err(invalid("the face wraps round the axis of its surface (it would need a seam)"));
    }
    if matches!(p.map, Map::Sphere { .. }) && (v0 < -PI / 2.0 + SEAM_MARGIN || v1 > PI / 2.0 - SEAM_MARGIN) {
        return Err(invalid("the face is a sphere cap of 90 degrees or more (it would reach a pole)"));
    }
    Ok(())
}

/// Builds the NURBS form (the kernel's own form of the whole surface over the heights `v` of the
/// box) and sets the domain to the box in its parameters.
fn finish(p: &mut Placement, geo: [[f64; 2]; 2], shape: &Shape) {
    let [[u0, u1], [v0, v1]] = geo;
    let (origin, basis, profile, u_scale) = match (&p.map, shape) {
        (Map::Plane { .. }, _) => {
            p.domain = geo;
            return;
        }
        (Map::Around { origin, x, y, z }, Shape::Cylinder { radius, .. }) => {
            (*origin, [*x, *y, *z], Profile::Line { v: [v0, v1], radius: [*radius; 2] }, *radius)
        }
        (Map::Around { origin, x, y, z }, Shape::Cone { half_angle, .. }) => {
            let r0 = match &p.surface {
                Surface::Cone { radius_at_origin, .. } => *radius_at_origin,
                _ => 0.0,
            };
            let s = half_angle.tan();
            (*origin, [*x, *y, *z], Profile::Line { v: [v0, v1], radius: [r0 + v0 * s, r0 + v1 * s] }, 1.0)
        }
        (Map::Sphere { center, x, y, z }, Shape::Sphere { radius, .. }) => {
            (*center, [*x, *y, *z], Profile::meridian(*radius), 1.0)
        }
        _ => return,
    };
    let scale = if matches!(shape, Shape::Sphere { .. }) { 0.25 } else { u_scale * FRAC_PI_2 };
    let arcs = Arcs::new(0.0, TAU, scale);
    p.nurbs = Some(revolved(origin, basis, &arcs, &profile));
    p.domain = [[arcs.param(u0), arcs.param(u1)], [profile.param(v0), profile.param(v1)]];
    p.forms = Some((arcs, profile));
}
