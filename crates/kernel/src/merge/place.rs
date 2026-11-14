//! The analytic surface a merged face lies on: its record form (label and NURBS form), parameter
//! box and side, and the kernel's evaluator of the surface (the parameters of a point on it, its
//! closest point and the distance of a point from it: `SurfaceEval`).
//!
use crate::hybrid::{Shape, V, add, cross, dot, norm, perpendicular, scale, sub, unit};
use crate::record::brep::{Frame, Surface};
use crate::{KernelError, Result, finite_all};
use solidcore::eval::{SurfaceEval, form};
use solidcore::math::Vec3;
use std::f64::consts::{PI, TAU};

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
    kind: Kind,
    /// The kernel's evaluator of the label: the angle and height (or latitude, or plane u and v) of
    /// the closest point of the surface to a point.
    eval: SurfaceEval,
    /// The parameters of the NURBS form are the kernel's quarter-grid ones ([`form::param`]).
    on_form: bool,
}

/// Which kind of surface a placement is.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Plane,
    /// Cylinder or cone.
    Round,
    Sphere,
}

/// The kernel's evaluator of a record surface.
fn evaluator(surface: &Surface) -> Result<SurfaceEval> {
    let s = solidcore_convert::surface(surface).map_err(|e| invalid(&e.to_string()))?;
    SurfaceEval::new(&s).map_err(|e| invalid(&format!("{e}")))
}

/// The closest point of the evaluated surface to `p`, found in closed form for the analytic kinds.
fn closest_on(eval: &SurfaceEval, p: V) -> Option<V> {
    let q = Vec3::new(p[0], p[1], p[2]);
    let uv = eval.project(q, None, f64::INFINITY)?;
    let c = eval.point(uv)?;
    Some([c.x, c.y, c.z])
}

impl Placement {
    /// The geometric (angle or plane u, height or latitude or plane v) of the closest point of the
    /// surface to `p`; not finite where the kernel cannot tell.
    fn geo(&self, p: V) -> [f64; 2] {
        self.eval.project(Vec3::new(p[0], p[1], p[2]), None, f64::INFINITY).unwrap_or([f64::NAN; 2])
    }

    /// Is it the plane (whose pcurves are the edges' own projections)?
    pub(super) fn is_plane(&self) -> bool {
        self.kind == Kind::Plane
    }

    /// The parameters of `p` in the face's surface (the NURBS form's, or the plane's).
    pub(super) fn param(&self, p: V) -> [f64; 2] {
        self.params_of_geo(self.geo(p))
    }

    fn params_of_geo(&self, g: [f64; 2]) -> [f64; 2] {
        match (self.kind, self.on_form) {
            (Kind::Sphere, true) => [form::param(g[0]), form::param(g[1])],
            (_, true) => [form::param(g[0]), g[1]],
            _ => g,
        }
    }
}

/// The kernel's evaluator of the primitive `shape`: distance of a point from it and its closest point.
pub(super) struct Gauge {
    eval: SurfaceEval,
}

impl Gauge {
    /// # Errors
    /// A shape that is not finite or is degenerate (a zero axis or normal, a radius or half angle
    /// outside its range), and a torus.
    pub(super) fn new(shape: &Shape) -> Result<Self> {
        Ok(Self { eval: evaluator(&surface_of(shape)?)? })
    }

    /// The point of the surface closest to `p`.
    pub(super) fn closest(&self, p: V) -> Option<V> {
        closest_on(&self.eval, p)
    }

    /// Distance of `p` from the surface.
    pub(super) fn distance(&self, p: V) -> Option<f64> {
        self.closest(p).map(|q| norm(sub(q, p))).filter(|d| d.is_finite())
    }
}

/// A record surface of the shape, in a frame of no importance (for evaluation only).
fn surface_of(shape: &Shape) -> Result<Surface> {
    let frame = |origin: V, z: V| {
        let x = perpendicular(z);
        Frame { origin, x_axis: x, y_axis: cross(z, x) }
    };
    let unit_axis = |a: V| unit(a).ok_or_else(|| invalid("degenerate axis"));
    let positive = |r: f64| if r.is_finite() && r > 0.0 { Ok(r) } else { Err(invalid("the radius must be positive")) };
    let ok = match shape {
        Shape::Plane { point, normal } => {
            let n = unit_axis(*normal)?;
            let u = perpendicular(n);
            Surface::Plane { origin: *point, u_axis: u, v_axis: cross(n, u) }
        }
        Shape::Cylinder { point, axis, radius } => {
            Surface::Cylinder { frame: frame(*point, unit_axis(*axis)?), radius: positive(*radius)? }
        }
        Shape::Cone { apex, axis, half_angle } => {
            if !(half_angle.is_finite() && half_angle.abs() > 0.0 && half_angle.abs() < PI / 2.0) {
                return Err(invalid("the half angle of a cone must be in (0, 90 degrees)"));
            }
            Surface::Cone { frame: frame(*apex, unit_axis(*axis)?), radius_at_origin: 0.0, half_angle: *half_angle }
        }
        Shape::Sphere { center, radius } => {
            Surface::Sphere { frame: frame(*center, [0.0, 0.0, 1.0]), radius: positive(*radius)? }
        }
        Shape::Torus { .. } => return Err(invalid("a torus cannot be placed yet")),
    };
    let points: &[V] = match shape {
        Shape::Plane { point, .. } | Shape::Cylinder { point, .. } => &[*point],
        Shape::Cone { apex, .. } => &[*apex],
        Shape::Sphere { center, .. } => &[*center],
        Shape::Torus { .. } => &[],
    };
    if points.iter().any(|p| !finite_all(p)) {
        return Err(invalid("the shape is not finite"));
    }
    Ok(ok)
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
            around(*point, a, c, outward, Round::Cylinder(*radius), points)?
        }
        Shape::Cone { apex, axis, half_angle } => {
            let a = unit(*axis).ok_or_else(|| invalid("degenerate axis"))?;
            let t0 = dot(sub(c, *apex), a);
            around(add(*apex, scale(a, t0)), a, c, outward, Round::Cone { t0, half_angle: *half_angle }, points)?
        }
        Shape::Sphere { center, radius } => sphere(*center, *radius, c, outward)?,
        Shape::Torus { .. } => return Err(invalid("a torus cannot be placed yet")),
    };
    let geo = padded_box(&placed, points);
    check_reach(&placed, geo)?;
    finish(&mut placed, geo)?;
    Ok(placed)
}

fn plane(point: V, normal: V, outward: V) -> Result<Placement> {
    let n = unit(normal).ok_or_else(|| invalid("degenerate plane normal"))?;
    let n = if dot(n, outward) < 0.0 { scale(n, -1.0) } else { n };
    let u = perpendicular(n);
    let v = cross(n, u);
    let surface = Surface::Plane { origin: point, u_axis: u, v_axis: v };
    let eval = evaluator(&surface)?;
    Ok(Placement { surface, nurbs: None, domain: [[0.0; 2]; 2], flip: false, kind: Kind::Plane, eval, on_form: false })
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

/// The direction from the axis into the widest gap in angle between the `points` (where the seam of a
/// surface of revolution goes so that the face is cut by it least).
fn seam_direction(points: &[V], origin: V, axis: V) -> Option<V> {
    let e1 = perpendicular(axis);
    let e2 = cross(axis, e1);
    let mut angles: Vec<f64> = points
        .iter()
        .filter_map(|p| {
            let q = sub(*p, origin);
            let (a, b) = (dot(q, e1), dot(q, e2));
            (a != 0.0 || b != 0.0).then(|| b.atan2(a))
        })
        .collect();
    angles.sort_by(f64::total_cmp);
    let first = *angles.first()?;
    let (mut widest, mut middle) = (0.0_f64, first + PI);
    for (a, b) in angles.iter().zip(angles.iter().skip(1).chain(std::iter::once(&(first + TAU)))) {
        if b - a > widest {
            (widest, middle) = (b - a, 0.5 * (a + b));
        }
    }
    Some(add(scale(e1, middle.cos()), scale(e2, middle.sin())))
}

fn around(origin: V, axis: V, c: V, outward: V, round: Round, points: &[V]) -> Result<Placement> {
    let d = radial(c, origin, axis).0.ok_or_else(|| invalid("the face lies on the axis"))?;
    let x = seam_direction(points, origin, axis).unwrap_or_else(|| scale(d, -1.0));
    let y = cross(axis, x);
    let frame = Frame { origin, x_axis: x, y_axis: y };
    let (surface, lean) = match round {
        Round::Cylinder(radius) => (Surface::Cylinder { frame, radius }, 0.0),
        Round::Cone { t0, half_angle } => {
            (Surface::Cone { frame, radius_at_origin: t0 * half_angle.tan(), half_angle }, -half_angle.tan())
        }
    };
    let flip = dot(add(d, scale(axis, lean)), outward) < 0.0;
    let eval = evaluator(&surface)?;
    Ok(Placement { surface, nurbs: None, domain: [[0.0; 2]; 2], flip, kind: Kind::Round, eval, on_form: false })
}

fn sphere(center: V, radius: f64, c: V, outward: V) -> Result<Placement> {
    let d = unit(sub(c, center)).ok_or_else(|| invalid("the face is at the sphere's centre"))?;
    let (x, z) = (scale(d, -1.0), perpendicular(d));
    let y = cross(z, x);
    let surface = Surface::Sphere { frame: Frame { origin: center, x_axis: x, y_axis: y }, radius };
    let eval = evaluator(&surface)?;
    Ok(Placement {
        surface,
        nurbs: None,
        domain: [[0.0; 2]; 2],
        flip: dot(d, outward) < 0.0,
        kind: Kind::Sphere,
        eval,
        on_form: false,
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
    if p.kind == Kind::Sphere && (v0 < -PI / 2.0 + SEAM_MARGIN || v1 > PI / 2.0 - SEAM_MARGIN) {
        return Err(invalid("the face is a sphere cap of 90 degrees or more (it would reach a pole)"));
    }
    Ok(())
}

/// Asks the kernel for the NURBS form of the surface (its whole turn about the axis, over the heights
/// of the box; a sphere pole to pole: the kernel keeps the label only for its own full forms) and sets
/// the domain to the box in the form's parameters.
fn finish(p: &mut Placement, geo: [[f64; 2]; 2]) -> Result<()> {
    if p.is_plane() {
        p.domain = geo;
        return Ok(());
    }
    let sphere = p.kind == Kind::Sphere;
    let v_range = if sphere { [-PI / 2.0, PI / 2.0] } else { geo[1] };
    let label = solidcore_convert::surface(&p.surface).map_err(|e| invalid(&format!("{e}")))?;
    let (nurbs, _) =
        form::surface(&label, [[0.0, TAU], v_range]).ok_or_else(|| invalid("the kernel has no form for the face"))?;
    p.nurbs = Some(solidcore_convert::record_surface(nurbs).map_err(|e| invalid(&format!("{e}")))?);
    p.on_form = true;
    let [lo, hi] = [[geo[0][0], geo[1][0]], [geo[0][1], geo[1][1]]].map(|c| p.params_of_geo(c));
    p.domain = [[lo[0], hi[0]], [lo[1], hi[1]]];
    Ok(())
}
