// Adapted from SolidCraft (https://github.com/tangency-labs/solidcraft) at fcd3186,
// crates/step/src/write/topology.rs (B-spline curves and surfaces, planes, directions and
// placements as entity instances).
// Copyright (c) 2026 SolidCraft contributors. Licensed under MIT OR Apache-2.0.
// Modified for SurfaceCraft (Apache-2.0 §4): works directly on solidcore-record without
// solidcraft-geom; one B-spline curve writer for 3-D curves and 2-D pcurves; adds circles,
// ellipses, cylinders, cones, spheres and tori (ISO 10303-42 parameterisations; the kernel
// crate turns a record surface into the one STEP writes).

//! Geometry entities of a sheet: points, directions, placements, curves and surfaces.

use super::p21::Param;
use super::topology::ShellWriter;
use super::{bad, e, ints, r, reals, s};
use crate::IoError;
use surfacecraft_kernel::record::brep::{Curve, Frame, Surface};

/// Distinct knots and their multiplicities.
pub(super) fn knots_of(full: &[f64]) -> (Vec<f64>, Vec<usize>) {
    let mut k: Vec<f64> = Vec::new();
    let mut m: Vec<usize> = Vec::new();
    for &x in full {
        match (k.last(), m.last_mut()) {
            (Some(&last), Some(c)) if (x - last).abs() <= 1e-14 * (1.0 + last.abs()) => *c += 1,
            _ => {
                k.push(x);
                m.push(1);
            }
        }
    }
    (k, m)
}

pub(super) fn degree(knots: &[f64], count: usize) -> Result<usize, IoError> {
    knots
        .len()
        .checked_sub(count + 1)
        .filter(|d| *d >= 1)
        .ok_or_else(|| bad(format!("{} knots for {count} control points", knots.len())))
}

/// `x` when it is finite and positive.
fn positive(x: f64, what: &str) -> Result<f64, IoError> {
    if x.is_finite() && x > 0.0 { Ok(x) } else { Err(bad(format!("{what} must be finite and positive, got {x}"))) }
}

/// A knot vector the writer can express: finite, non-decreasing, with at least two distinct values.
fn check_knots(knots: &[f64], entity: &str) -> Result<(), IoError> {
    let ordered = knots.iter().all(|k| k.is_finite()) && knots.windows(2).all(|w| w[0] <= w[1]);
    let spans = matches!((knots.first(), knots.last()), (Some(a), Some(b)) if b > a);
    if ordered && spans {
        Ok(())
    } else {
        Err(bad(format!("{entity} knots must be finite, non-decreasing and span a range: {knots:?}")))
    }
}

/// How far from unit length and square a frame may be before STEP's derived y axis would differ
/// from the record's.
const FRAME_TOLERANCE: f64 = 1e-6;

fn check_frame(f: &Frame) -> Result<(), IoError> {
    let (x, y) = (f.x_axis, f.y_axis);
    let dot = |a: [f64; 3], b: [f64; 3]| a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let square = (dot(x, x) - 1.0).abs() <= FRAME_TOLERANCE
        && (dot(y, y) - 1.0).abs() <= FRAME_TOLERANCE
        && dot(x, y).abs() <= FRAME_TOLERANCE;
    if square {
        Ok(())
    } else {
        Err(bad(format!("frame axes {x:?}, {y:?} are not orthonormal (AXIS2_PLACEMENT_3D derives y)")))
    }
}

fn finite(p: &[f64]) -> Result<(), IoError> {
    if p.iter().all(|v| v.is_finite()) { Ok(()) } else { Err(bad("non-finite coordinate")) }
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

/// Splits homogeneous coordinates into points and positive weights.
pub(super) fn rational_split(h: &[[f64; 4]]) -> Result<(Vec<[f64; 3]>, Vec<f64>), IoError> {
    let mut pts = Vec::with_capacity(h.len());
    let mut ws = Vec::with_capacity(h.len());
    for c in h {
        if !(c[3] > 0.0) || !c[3].is_finite() {
            return Err(bad("rational weight must be positive"));
        }
        pts.push([c[0] / c[3], c[1] / c[3], c[2] / c[3]]);
        ws.push(c[3]);
    }
    Ok((pts, ws))
}

impl ShellWriter<'_> {
    pub(super) fn point(&mut self, p: [f64; 3]) -> Result<u64, IoError> {
        finite(&p)?;
        Ok(self.w.add("CARTESIAN_POINT", &[s(), reals(&p)]))
    }

    pub(super) fn point2(&mut self, p: [f64; 2]) -> Result<u64, IoError> {
        finite(&p)?;
        Ok(self.w.add("CARTESIAN_POINT", &[s(), reals(&p)]))
    }

    pub(super) fn direction(&mut self, d: [f64; 3]) -> Result<u64, IoError> {
        let n = (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt();
        if !(n > 1e-300) || !n.is_finite() {
            return Err(bad("zero direction"));
        }
        Ok(self.w.add("DIRECTION", &[s(), reals(&[d[0] / n, d[1] / n, d[2] / n])]))
    }

    /// `AXIS2_PLACEMENT_3D` of a right-handed frame: z = x × y, x the reference direction.
    pub(super) fn placement(&mut self, f: &Frame) -> Result<u64, IoError> {
        check_frame(f)?;
        let o = self.point(f.origin)?;
        let z = self.direction(cross(f.x_axis, f.y_axis))?;
        let x = self.direction(f.x_axis)?;
        Ok(self.w.add("AXIS2_PLACEMENT_3D", &[s(), r(o), r(z), r(x)]))
    }

    /// A B-spline curve on `points` (references to 2-D or 3-D points), rational when `weights`
    /// are given. Written with distinct knots and multiplicities.
    pub(super) fn bspline_curve(
        &mut self,
        knots: &[f64],
        points: Vec<Param>,
        weights: Option<&[f64]>,
    ) -> Result<u64, IoError> {
        check_knots(knots, "B_SPLINE_CURVE")?;
        let deg = degree(knots, points.len())?;
        let (k, m) = knots_of(knots);
        let Some(ws) = weights else {
            return Ok(self.w.add(
                "B_SPLINE_CURVE_WITH_KNOTS",
                &[
                    s(),
                    Param::Int(deg as i64),
                    Param::List(points),
                    e("UNSPECIFIED"),
                    e("F"),
                    e("F"),
                    ints(&m),
                    reals(&k),
                    e("UNSPECIFIED"),
                ],
            ));
        };
        Ok(self.w.add_complex(&[
            ("BOUNDED_CURVE", vec![]),
            ("B_SPLINE_CURVE", vec![Param::Int(deg as i64), Param::List(points), e("UNSPECIFIED"), e("F"), e("F")]),
            ("B_SPLINE_CURVE_WITH_KNOTS", vec![ints(&m), reals(&k), e("UNSPECIFIED")]),
            ("CURVE", vec![]),
            ("GEOMETRIC_REPRESENTATION_ITEM", vec![]),
            ("RATIONAL_B_SPLINE_CURVE", vec![reals(ws)]),
            ("REPRESENTATION_ITEM", vec![s()]),
        ]))
    }

    fn point_refs(&mut self, pts: &[[f64; 3]]) -> Result<Vec<Param>, IoError> {
        pts.iter().map(|p| self.point(*p).map(r)).collect()
    }

    /// The 3-D curve of an edge.
    pub(super) fn curve(&mut self, c: &Curve) -> Result<u64, IoError> {
        match c {
            Curve::Line { start, end } => {
                let p = self.point(*start)?;
                let d = [end[0] - start[0], end[1] - start[1], end[2] - start[2]];
                let len = positive((d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt(), "LINE length")?;
                let dir = self.direction(d)?;
                let v = self.w.add("VECTOR", &[s(), r(dir), Param::Real(len)]);
                Ok(self.w.add("LINE", &[s(), r(p), r(v)]))
            }
            Curve::Circle { frame, radius } => {
                let radius = positive(*radius, "CIRCLE radius")?;
                let ax = self.placement(frame)?;
                Ok(self.w.add("CIRCLE", &[s(), r(ax), Param::Real(radius)]))
            }
            Curve::Ellipse { frame, radii } => {
                let (a, b) = (positive(radii[0], "ELLIPSE semi axis 1")?, positive(radii[1], "ELLIPSE semi axis 2")?);
                let ax = self.placement(frame)?;
                Ok(self.w.add("ELLIPSE", &[s(), r(ax), Param::Real(a), Param::Real(b)]))
            }
            Curve::Spline { knots, points } => {
                let ids = self.point_refs(points)?;
                self.bspline_curve(knots, ids, None)
            }
            Curve::RationalSpline { knots, homogeneous } => {
                let (pts, ws) = rational_split(homogeneous)?;
                let ids = self.point_refs(&pts)?;
                self.bspline_curve(knots, ids, Some(&ws))
            }
            other => Err(bad(format!("not supported yet: STEP export of a {} edge in a sheet", curve_name(other)))),
        }
    }

    fn net_refs(&mut self, rows: &[Vec<[f64; 3]>], nv: usize) -> Result<Vec<Param>, IoError> {
        let mut out = Vec::with_capacity(rows.len());
        for row in rows {
            if row.len() != nv {
                return Err(bad("control net is not rectangular"));
            }
            out.push(Param::List(self.point_refs(row)?));
        }
        Ok(out)
    }

    fn spline_surface(&mut self, u_knots: &[f64], v_knots: &[f64], points: &[Vec<[f64; 3]>]) -> Result<u64, IoError> {
        check_knots(u_knots, "B_SPLINE_SURFACE u")?;
        check_knots(v_knots, "B_SPLINE_SURFACE v")?;
        let nv = points.first().map_or(0, Vec::len);
        let (p, q) = (degree(u_knots, points.len())?, degree(v_knots, nv)?);
        let rows = self.net_refs(points, nv)?;
        let (uk, um) = knots_of(u_knots);
        let (vk, vm) = knots_of(v_knots);
        Ok(self.w.add(
            "B_SPLINE_SURFACE_WITH_KNOTS",
            &[
                s(),
                Param::Int(p as i64),
                Param::Int(q as i64),
                Param::List(rows),
                e("UNSPECIFIED"),
                e("F"),
                e("F"),
                e("F"),
                ints(&um),
                ints(&vm),
                reals(&uk),
                reals(&vk),
                e("UNSPECIFIED"),
            ],
        ))
    }

    fn rational_surface(&mut self, u_knots: &[f64], v_knots: &[f64], h: &[Vec<[f64; 4]>]) -> Result<u64, IoError> {
        check_knots(u_knots, "B_SPLINE_SURFACE u")?;
        check_knots(v_knots, "B_SPLINE_SURFACE v")?;
        let nv = h.first().map_or(0, Vec::len);
        let (p, q) = (degree(u_knots, h.len())?, degree(v_knots, nv)?);
        let (mut rows, mut wrows) = (Vec::with_capacity(h.len()), Vec::with_capacity(h.len()));
        for row in h {
            if row.len() != nv {
                return Err(bad("control net is not rectangular"));
            }
            let (pts, ws) = rational_split(row)?;
            rows.push(Param::List(self.point_refs(&pts)?));
            wrows.push(reals(&ws));
        }
        let (uk, um) = knots_of(u_knots);
        let (vk, vm) = knots_of(v_knots);
        Ok(self.w.add_complex(&[
            ("BOUNDED_SURFACE", vec![]),
            (
                "B_SPLINE_SURFACE",
                vec![
                    Param::Int(p as i64),
                    Param::Int(q as i64),
                    Param::List(rows),
                    e("UNSPECIFIED"),
                    e("F"),
                    e("F"),
                    e("F"),
                ],
            ),
            ("B_SPLINE_SURFACE_WITH_KNOTS", vec![ints(&um), ints(&vm), reals(&uk), reals(&vk), e("UNSPECIFIED")]),
            ("GEOMETRIC_REPRESENTATION_ITEM", vec![]),
            ("RATIONAL_B_SPLINE_SURFACE", vec![Param::List(wrows)]),
            ("REPRESENTATION_ITEM", vec![s()]),
            ("SURFACE", vec![]),
        ]))
    }

    /// The surface entity of a surface already in STEP's parameters: what
    /// `surfacecraft_kernel::pcurve` hands back for planes and analytic faces, or a spline.
    pub(super) fn step_surface(&mut self, surface: &Surface) -> Result<u64, IoError> {
        match surface {
            Surface::Spline { u_knots, v_knots, points } => self.spline_surface(u_knots, v_knots, points),
            Surface::RationalSpline { u_knots, v_knots, homogeneous } => {
                self.rational_surface(u_knots, v_knots, homogeneous)
            }
            Surface::Plane { origin, u_axis, v_axis } => {
                let ax = self.placement(&Frame { origin: *origin, x_axis: *u_axis, y_axis: *v_axis })?;
                Ok(self.w.add("PLANE", &[s(), r(ax)]))
            }
            Surface::Cylinder { frame, radius } => {
                let radius = positive(*radius, "CYLINDRICAL_SURFACE radius")?;
                let ax = self.placement(frame)?;
                Ok(self.w.add("CYLINDRICAL_SURFACE", &[s(), r(ax), Param::Real(radius)]))
            }
            Surface::Cone { frame, radius_at_origin, half_angle } => {
                let semi = positive(*half_angle, "CONICAL_SURFACE semi-angle")?;
                if !(radius_at_origin.is_finite() && *radius_at_origin >= 0.0) {
                    return Err(bad(format!(
                        "CONICAL_SURFACE radius must be finite and not negative, got {radius_at_origin}"
                    )));
                }
                let ax = self.placement(frame)?;
                Ok(self.w.add("CONICAL_SURFACE", &[s(), r(ax), Param::Real(*radius_at_origin), Param::Real(semi)]))
            }
            Surface::Sphere { frame, radius } => {
                let radius = positive(*radius, "SPHERICAL_SURFACE radius")?;
                let ax = self.placement(frame)?;
                Ok(self.w.add("SPHERICAL_SURFACE", &[s(), r(ax), Param::Real(radius)]))
            }
            Surface::Torus { frame, major_radius, minor_radius, .. } => {
                let big = positive(*major_radius, "TOROIDAL_SURFACE major radius")?;
                let small = positive(*minor_radius, "TOROIDAL_SURFACE minor radius")?;
                let ax = self.placement(frame)?;
                Ok(self.w.add("TOROIDAL_SURFACE", &[s(), r(ax), Param::Real(big), Param::Real(small)]))
            }
            other => {
                Err(bad(format!("not supported yet: STEP export of a {} surface in a sheet", surface_name(other))))
            }
        }
    }
}

fn curve_name(c: &Curve) -> &'static str {
    match c {
        Curve::Line { .. } => "line",
        Curve::Circle { .. } => "circle",
        Curve::Ellipse { .. } => "ellipse",
        Curve::Polyline { .. } => "polyline",
        Curve::Spline { .. } => "spline",
        Curve::RationalSpline { .. } => "rational spline",
        Curve::SurfaceIntersection { .. } => "intersection curve",
    }
}

pub(super) fn surface_name(s: &Surface) -> &'static str {
    match s {
        Surface::Plane { .. } => "plane",
        Surface::PlaneThrough { .. } => "plane through three points",
        Surface::Cylinder { .. } => "cylinder",
        Surface::Cone { .. } => "cone",
        Surface::Sphere { .. } => "sphere",
        Surface::Torus { .. } => "torus",
        Surface::BallPatch { .. } => "ball patch",
        Surface::Spline { .. } => "spline",
        Surface::RationalSpline { .. } => "rational spline",
        Surface::PlacedRevolution { .. } => "revolution",
    }
}
