//! Parameter-space curves of a sheet's edges, for STEP export (ISO 10303-42 parameterisations).
//!
//! A face of a record is parameterised on its own surface (a plane's axes need not be unit or
//! square) or on the NURBS form SMLib holds an analytic surface by (arc-length-like angle, form
//! turned about the axis). STEP's surfaces have fixed parameters: a plane by its unit axes, a
//! cylinder, cone, sphere and torus by angle (radians, the file's plane-angle unit) and height or
//! latitude or tube angle. This module turns a face into what STEP can say: the surface as it is
//! written ([`written_surface`], [`step_plane`]), whether its natural normal is the form's
//! ([`AnalyticFace::flip_sense`]) and each edge use as a pcurve in those parameters. All
//! evaluation is solidcore's (`solidcore::eval`); nothing here builds geometry of its own.
//!
//! An analytic face's pcurves are exact iso-parameter lines: every edge SurfaceCraft's sheets
//! carry on a cylinder, cone, sphere or torus is a ruling, a circle of latitude or a meridian,
//! so one parameter stays constant. Any other edge (a section by a slanted plane) is refused,
//! never approximated.

use crate::record::brep::{Curve, Curve2, Edge, Face, Frame, Loop, Surface};
use crate::{KernelError, Result, op_err};
use solidcore::eval::uv::unwrap;
use solidcore::eval::{CurveEval, SurfaceEval};
use solidcore::math::Vec3;
use std::collections::BTreeMap;

/// Samples along an edge when its parameters are traced (a full turn takes steps of 11°).
const TRACE_SAMPLES: usize = 33;
/// The least tolerance an edge is held to (mm): the record's own is used when larger.
const MIN_TOLERANCE_MM: f64 = 1e-6;
/// The most: a record's own tolerance is hostile input, and a large one would make any edge lie
/// on any surface (mm).
const MAX_TOLERANCE_MM: f64 = 1e-3;
/// How far a point may lie from its surface to be projected onto it, in edge tolerances.
const ACCEPT_TOLERANCES: f64 = 10.0;
/// Samples of a pcurve when its gap to the 3-D curve is measured.
const GAP_PCURVE_SAMPLES: usize = 65;
/// Samples of the 3-D curve the closest point is searched from.
const GAP_CURVE_SAMPLES: usize = 257;
/// Golden-section steps refining the closest point.
const GAP_REFINE_STEPS: usize = 60;
/// A parameter may run back this far against its direction before an edge counts as doubling back.
const MONOTONE_SLACK: f64 = 1e-12;
/// Where in a face's parameter box the two surfaces' normals are compared.
const NORMAL_PROBES: [[f64; 2]; 3] = [[0.5, 0.5], [0.37, 0.61], [0.71, 0.29]];
/// A normal shorter than this is a pole, not a direction.
const MIN_NORMAL: f64 = 1e-9;
/// Distance of a probed point from the written surface that is still the same surface (mm).
const SAME_SURFACE_MM: f64 = 1e-5;
/// Step in the form's u, as a fraction of the face's u range, that finds which way STEP's u runs.
const FORM_STEP: f64 = 1e-3;
/// Two uses of a seam whose form u differ by less than this cannot be ranked.
const MIN_SEAM_SEPARATION: f64 = 1e-9;
/// A half angle this close to 0 or a right angle is no cone STEP can hold.
const MIN_HALF_ANGLE: f64 = 1e-12;

fn unsupported(what: impl std::fmt::Display) -> KernelError {
    KernelError::Argument(format!("not supported yet: {what}"))
}

fn surface_eval(s: &Surface) -> Result<SurfaceEval> {
    let k = solidcore_convert::surface(s).map_err(|e| op_err("convert surface", e))?;
    SurfaceEval::new(&k).map_err(|e| op_err("surface evaluator", e))
}

fn curve_eval(c: &Curve) -> Result<CurveEval> {
    let k = solidcore_convert::curve(c).map_err(|e| op_err("convert curve", e))?;
    CurveEval::new(&k).map_err(|e| op_err("curve evaluator", e))
}

fn point_on(eval: &CurveEval, t: f64) -> Result<Vec3> {
    eval.point(t).ok_or_else(|| KernelError::Argument(format!("edge curve has no point at t = {t}")))
}

/// Whether `s` is one of the analytic kinds written with computed pcurves.
#[must_use]
pub fn is_analytic(s: &Surface) -> bool {
    matches!(s, Surface::Cylinder { .. } | Surface::Cone { .. } | Surface::Sphere { .. } | Surface::Torus { .. })
}

fn positive(x: f64, what: &str) -> Result<f64> {
    if x.is_finite() && x > 0.0 { Ok(x) } else { Err(KernelError::Argument(format!("{what} must be positive"))) }
}

fn flipped(frame: &Frame) -> Frame {
    Frame { origin: frame.origin, x_axis: frame.x_axis, y_axis: frame.y_axis.map(|c| -c) }
}

/// The surface as STEP writes it: same geometry, STEP's parameters. A cone narrowing along z
/// (negative half angle) turns its frame over (u and v change sign, the normal does not: STEP asks
/// a positive semi-angle); a torus's tube starts at its outer equator.
pub fn written_surface(s: &Surface) -> Result<Surface> {
    match s {
        Surface::Cylinder { frame, radius } => {
            Ok(Surface::Cylinder { frame: frame.clone(), radius: positive(*radius, "cylinder radius")? })
        }
        Surface::Sphere { frame, radius } => {
            Ok(Surface::Sphere { frame: frame.clone(), radius: positive(*radius, "sphere radius")? })
        }
        Surface::Cone { frame, radius_at_origin, half_angle } => {
            let (r, a) = (*radius_at_origin, *half_angle);
            if !(r.is_finite() && r >= 0.0) {
                return Err(unsupported("STEP export of a cone with a negative radius at its frame origin"));
            }
            let size = a.abs();
            if !(size.is_finite() && size > MIN_HALF_ANGLE && size < std::f64::consts::FRAC_PI_2) {
                return Err(unsupported("STEP export of a cone whose half angle is not in (0, 90°)"));
            }
            let frame = if a < 0.0 { flipped(frame) } else { frame.clone() };
            Ok(Surface::Cone { frame, radius_at_origin: r, half_angle: size })
        }
        Surface::Torus { frame, major_radius, minor_radius, .. } => {
            let (big, small) =
                (positive(*major_radius, "torus major radius")?, positive(*minor_radius, "torus minor radius")?);
            if big <= small {
                return Err(unsupported("STEP export of a torus whose tube reaches the axis"));
            }
            Ok(Surface::Torus { frame: frame.clone(), major_radius: big, minor_radius: small, tube_start: 0.0 })
        }
        _ => Err(unsupported("STEP export of a surface that is not a cylinder, cone, sphere or torus")),
    }
}

/// A plane as STEP writes it, and how the record's (u, v) become STEP's.
#[derive(Clone, Debug, PartialEq)]
pub struct StepPlane {
    /// The plane with unit, square axes: `origin + u x + v y`, `z = x × y` its natural normal.
    pub surface: Surface,
    /// Row-major 2×2 matrix taking the record's (u, v) to STEP's.
    map: [[f64; 2]; 2],
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross3(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn unit3(a: [f64; 3]) -> Result<[f64; 3]> {
    let n = dot3(a, a).sqrt();
    if n.is_finite() && n > MIN_NORMAL {
        Ok(a.map(|c| c / n))
    } else {
        Err(KernelError::Argument("plane axes are degenerate".into()))
    }
}

/// The unit-axis plane STEP writes for the record plane `origin + u u_axis + v v_axis`.
pub fn step_plane(origin: [f64; 3], u_axis: [f64; 3], v_axis: [f64; 3]) -> Result<StepPlane> {
    let x = unit3(u_axis)?;
    let z = unit3(cross3(u_axis, v_axis))?;
    let y = cross3(z, x);
    let map = [[dot3(u_axis, x), dot3(v_axis, x)], [dot3(u_axis, y), dot3(v_axis, y)]];
    Ok(StepPlane { surface: Surface::Plane { origin, u_axis: x, v_axis: y }, map })
}

impl StepPlane {
    fn apply(&self, u: f64, v: f64) -> [f64; 2] {
        let [[a, b], [c, d]] = self.map;
        [a * u + b * v, c * u + d * v]
    }

    /// A pcurve of the record plane, in STEP's parameters: the map is linear, so every control
    /// point (homogeneous ones by their first two coordinates) maps and the curve stays exact.
    #[must_use]
    pub fn pcurve(&self, c: &Curve2) -> Curve2 {
        match c {
            Curve2::Spline { knots, points } => {
                Curve2::Spline { knots: knots.clone(), points: points.iter().map(|p| self.apply(p[0], p[1])).collect() }
            }
            Curve2::RationalSpline { knots, homogeneous } => Curve2::RationalSpline {
                knots: knots.clone(),
                homogeneous: homogeneous
                    .iter()
                    .map(|h| {
                        let m = self.apply(h[0], h[1]);
                        [m[0], m[1], h[2]]
                    })
                    .collect(),
            },
        }
    }
}

/// An analytic face as STEP writes it.
#[derive(Clone, Debug, PartialEq)]
pub struct AnalyticFace {
    /// The surface in STEP's parameters ([`written_surface`]).
    pub surface: Surface,
    /// The written surface's natural normal is the opposite of the one the record's face
    /// parameters (its NURBS form) give: the shell's side must be flipped for the file.
    pub flip_sense: bool,
    /// `pcurves[loop][position]`, in STEP's parameters, on each edge's own interval and running
    /// min → max whatever the loop's direction; a vertex loop has none.
    pub pcurves: Vec<Vec<Curve2>>,
}

/// The unit normal `S_u × S_v` at `uv`, `None` at a pole.
fn normal(eval: &SurfaceEval, uv: [f64; 2]) -> Option<Vec3> {
    let (_, du, dv) = eval.derivs(uv[0], uv[1])?;
    let n = du.cross(dv);
    let len = n.length();
    (len > MIN_NORMAL && len.is_finite()).then(|| n / len)
}

/// Whether the written surface's normal opposes the form's, compared where both are defined.
fn normals_oppose(form: &Surface, written: &SurfaceEval, domain: [[f64; 2]; 2]) -> Result<bool> {
    let form = surface_eval(form)?;
    let at = |r: [f64; 2], f: f64| r[0] + f * (r[1] - r[0]);
    for [fu, fv] in NORMAL_PROBES {
        let uv = [at(domain[0], fu), at(domain[1], fv)];
        let Some((p, ..)) = form.derivs(uv[0], uv[1]) else { continue };
        let (Some(nf), Some(uv_w)) = (normal(&form, uv), written.project(p, None, SAME_SURFACE_MM)) else { continue };
        if let Some(nw) = normal(written, uv_w) {
            return Ok(nf.dot(nw) < 0.0);
        }
    }
    Err(unsupported("STEP export of a face whose normal is undefined where it was probed"))
}

/// One parameter traced along an edge use, in traversal order and unwrapped across periods.
struct Trace {
    uv: Vec<[f64; 2]>,
    points: Vec<Vec3>,
}

struct UseTracer<'a> {
    surface: &'a SurfaceEval,
    periods: [Option<f64>; 2],
    tolerance: f64,
}

impl UseTracer<'_> {
    fn project(&self, p: Vec3, near: Option<[f64; 2]>) -> Result<[f64; 2]> {
        self.surface
            .project(p, near, ACCEPT_TOLERANCES * self.tolerance)
            .ok_or_else(|| KernelError::Argument("an edge does not lie on its face's surface".into()))
    }

    fn unwrapped(&self, uv: [f64; 2], to: [f64; 2]) -> [f64; 2] {
        [unwrap(uv[0], to[0], self.periods[0]), unwrap(uv[1], to[1], self.periods[1])]
    }

    /// The edge's samples from the traversal start, the first one nearest `start` (the end of the
    /// previous use) when given.
    fn trace(&self, curve: &CurveEval, [t0, t1]: [f64; 2], forward: bool, start: Option<[f64; 2]>) -> Result<Trace> {
        let steps = TRACE_SAMPLES - 1;
        let mut points = Vec::with_capacity(TRACE_SAMPLES);
        for i in 0..TRACE_SAMPLES {
            let f = i as f64 / steps as f64;
            points.push(point_on(curve, if forward { t0 + f * (t1 - t0) } else { t1 - f * (t1 - t0) })?);
        }
        // a point on an axis has no angle: the middle of the edge seeds the one it falls back to
        let seed = match (start, points.get(steps / 2)) {
            (Some(s), _) => s,
            (None, Some(mid)) => self.project(*mid, None)?,
            (None, None) => return Err(KernelError::Argument("no samples".into())),
        };
        let mut uv: Vec<[f64; 2]> = Vec::with_capacity(TRACE_SAMPLES);
        let mut near = seed;
        for p in &points {
            let q = self.unwrapped(self.project(*p, Some(near))?, near);
            near = q;
            uv.push(q);
        }
        Ok(Trace { uv, points })
    }
}

fn first_last(t: &Trace) -> Result<([f64; 2], [f64; 2])> {
    match (t.uv.first(), t.uv.last()) {
        (Some(a), Some(b)) => Ok((*a, *b)),
        _ => Err(KernelError::Argument("no samples".into())),
    }
}

/// The coordinate that stays constant along the trace (v first, as circles are the commoner
/// edge) when every sample lies within `tol` of its iso-curve. The constant is the middle
/// sample's: an end at a pole has no angle of its own, and every iso-curve passes through it.
fn iso_coordinate(eval: &SurfaceEval, t: &Trace, tol: f64) -> Option<(usize, f64)> {
    let mid = t.uv.get(t.uv.len() / 2)?;
    for k in [1usize, 0] {
        let c = mid.get(k).copied()?;
        let on_iso = t.uv.iter().zip(&t.points).all(|(q, p)| {
            let moved = if k == 0 { [c, q[1]] } else { [q[0], c] };
            eval.point(moved).is_some_and(|s| s.distance(*p) <= tol)
        });
        if on_iso {
            return Some((k, c));
        }
    }
    None
}

fn monotone(t: &Trace, k: usize) -> bool {
    let w: Vec<f64> = t.uv.iter().filter_map(|q| q.get(k).copied()).collect();
    let up = w.windows(2).all(|p| p[1] - p[0] >= -MONOTONE_SLACK);
    let down = w.windows(2).all(|p| p[1] - p[0] <= MONOTONE_SLACK);
    up || down
}

/// The line `(c, w)` / `(w, c)` from the first sample's coordinate to the last's, as a pcurve on
/// the edge's interval, ascending in the edge's parameter whichever way the loop runs.
fn iso_line(k: usize, c: f64, [w_first, w_last]: [f64; 2], [t0, t1]: [f64; 2], forward: bool) -> Curve2 {
    let at = |w: f64| if k == 0 { [c, w] } else { [w, c] };
    let (a, b) = if forward { (w_first, w_last) } else { (w_last, w_first) };
    Curve2::Spline { knots: vec![t0, t0, t1, t1], points: vec![at(a), at(b)] }
}

/// The distance the edge's samples may lie from the surface: the record's tolerance, held between
/// `MIN_TOLERANCE_MM` and `MAX_TOLERANCE_MM` (NaN counts as the least).
fn edge_tolerance(edge: &Edge) -> f64 {
    if edge.tolerance.is_nan() { MIN_TOLERANCE_MM } else { edge.tolerance.clamp(MIN_TOLERANCE_MM, MAX_TOLERANCE_MM) }
}

fn use_pcurve(
    tracer: &UseTracer<'_>,
    eval: &SurfaceEval,
    edge: &Edge,
    forward: bool,
    start: Option<[f64; 2]>,
) -> Result<(Curve2, [f64; 2])> {
    let [t0, t1] = edge.interval;
    if !(t0.is_finite() && t1.is_finite() && t1 > t0) {
        return Err(KernelError::Argument(format!("an edge has an empty or non-finite interval [{t0}, {t1}]")));
    }
    let curve = curve_eval(edge.nurbs.as_ref().unwrap_or(&edge.curve))?;
    let trace = tracer.trace(&curve, edge.interval, forward, start)?;
    let (first, last) = first_last(&trace)?;
    let Some((k, c)) = iso_coordinate(eval, &trace, tracer.tolerance).filter(|(k, _)| monotone(&trace, 1 - k)) else {
        return Err(unsupported(
            "STEP export of an edge on a cylinder, cone, sphere or torus that is not an iso-parameter curve",
        ));
    };
    let ends = [first.get(1 - k).copied().unwrap_or(0.0), last.get(1 - k).copied().unwrap_or(0.0)];
    let line = iso_line(k, c, ends, edge.interval, forward);
    let end = if k == 0 { [c, ends[1]] } else { [ends[1], c] };
    Ok((line, end))
}

fn mid_of(c: &Curve2) -> Result<[f64; 2]> {
    let k = solidcore_convert::curve2(c);
    let [a, b] = k.interval().ok_or_else(|| KernelError::Argument("pcurve has no parameter interval".into()))?;
    k.eval(0.5 * (a + b)).ok_or_else(|| KernelError::Argument("pcurve has no midpoint".into()))
}

fn shifted_in_u(c: &Curve2, by: f64) -> Curve2 {
    match c {
        Curve2::Spline { knots, points } => {
            Curve2::Spline { knots: knots.clone(), points: points.iter().map(|p| [p[0] + by, p[1]]).collect() }
        }
        Curve2::RationalSpline { knots, homogeneous } => Curve2::RationalSpline {
            knots: knots.clone(),
            homogeneous: homogeneous.iter().map(|h| [h[0] + by * h[2], h[1], h[2]]).collect(),
        },
    }
}

/// Whether STEP's u grows with the form's u where `uv` is (the form is a different
/// parameterisation of the same surface: it may run the other way round the axis).
fn u_runs_with_form(face: &Face, written: &SurfaceEval, uv: [f64; 2]) -> Result<bool> {
    let Some(form) = &face.nurbs else { return Ok(true) };
    let form = surface_eval(form)?;
    let width = face.domain[0][1] - face.domain[0][0];
    let step = if uv[0] + FORM_STEP * width <= face.domain[0][1] { FORM_STEP * width } else { -FORM_STEP * width };
    let at = |u: f64| {
        let p = form.point([u, uv[1]]).ok_or_else(|| KernelError::Argument("form has no point".into()))?;
        written.project(p, None, SAME_SURFACE_MM).ok_or_else(|| KernelError::Argument("form leaves its surface".into()))
    };
    let (a, b) = (at(uv[0])?, at(uv[0] + step)?);
    let period = written.domain().period[0].unwrap_or(std::f64::consts::TAU);
    let d = unwrap(b[0] - a[0], 0.0, Some(period));
    Ok((d > 0.0) == (step > 0.0))
}

/// An edge used twice by one face (a seam, or a meridian through two poles) has its two uses a
/// turn apart in u. A chain of uses already puts them there when a circle lies between; where the
/// two meet at a pole it cannot, and the form's own u says which use is the later one.
fn separate_seam_uses(face: &Face, written: &SurfaceEval, rows: &mut [Vec<Curve2>]) -> Result<()> {
    let Some(period) = written.domain().period[0] else { return Ok(()) };
    let mut by_edge: BTreeMap<u32, Vec<(usize, usize)>> = BTreeMap::new();
    for (li, l) in face.loops.iter().enumerate() {
        if let Loop::Edges { edges } = l {
            for (pi, u) in edges.iter().enumerate() {
                by_edge.entry(u.edge).or_default().push((li, pi));
            }
        }
    }
    for uses in by_edge.values().filter(|u| u.len() == 2) {
        let (Some(&a), Some(&b)) = (uses.first(), uses.get(1)) else { continue };
        let (form_a, form_b) = (record_mid(face, a)?, record_mid(face, b)?);
        if (form_a[0] - form_b[0]).abs() < MIN_SEAM_SEPARATION {
            return Err(unsupported("STEP export of a face whose seam uses cannot be told apart"));
        }
        let (mid_a, mid_b) = (row_mid(rows, a)?, row_mid(rows, b)?);
        let runs_with = u_runs_with_form(face, written, form_a)?;
        let b_is_later = (form_b[0] > form_a[0]) == runs_with;
        let turns = ((mid_b[0] - mid_a[0]) / period).round();
        let (later, shift) = if b_is_later { (b, 1.0 - turns) } else { (a, 1.0 + turns) };
        if shift != 0.0
            && let Some(c) = rows.get_mut(later.0).and_then(|r| r.get_mut(later.1))
        {
            *c = shifted_in_u(c, shift * period);
        }
    }
    Ok(())
}

fn record_mid(face: &Face, (li, pi): (usize, usize)) -> Result<[f64; 2]> {
    match face.loops.get(li) {
        Some(Loop::Edges { edges }) => edges
            .get(pi)
            .map(|u| mid_of(&u.pcurve))
            .unwrap_or_else(|| Err(KernelError::Argument("loop position".into()))),
        _ => Err(KernelError::Argument("loop position".into())),
    }
}

fn row_mid(rows: &[Vec<Curve2>], (li, pi): (usize, usize)) -> Result<[f64; 2]> {
    rows.get(li)
        .and_then(|r| r.get(pi))
        .map(mid_of)
        .unwrap_or_else(|| Err(KernelError::Argument("loop position".into())))
}

/// An analytic face of `face` as STEP writes it, with a pcurve for each edge use of its loops.
/// `edges` is the record's edge list.
pub fn analytic_face(face: &Face, edges: &[Edge]) -> Result<AnalyticFace> {
    let surface = written_surface(&face.surface)?;
    let eval = surface_eval(&surface)?;
    let flip_sense = match &face.nurbs {
        Some(form) => normals_oppose(form, &eval, face.domain)?,
        None => false,
    };
    let periods = eval.domain().period;
    let mut pcurves = Vec::with_capacity(face.loops.len());
    for l in &face.loops {
        let Loop::Edges { edges: uses } = l else {
            pcurves.push(Vec::new());
            continue;
        };
        let mut row = Vec::with_capacity(uses.len());
        let mut end: Option<[f64; 2]> = None;
        for u in uses {
            let edge =
                edges.get(u.edge as usize).ok_or_else(|| KernelError::Argument(format!("edge {} missing", u.edge)))?;
            let tracer = UseTracer { surface: &eval, periods, tolerance: edge_tolerance(edge) };
            let (pc, to) = use_pcurve(&tracer, &eval, edge, u.forward, end)?;
            row.push(pc);
            end = Some(to);
        }
        pcurves.push(row);
    }
    separate_seam_uses(face, &eval, &mut pcurves)?;
    Ok(AnalyticFace { surface, flip_sense, pcurves })
}

/// The 3-D curve's point nearest `p`, found from `GAP_CURVE_SAMPLES` samples of `interval`.
fn distance_to_curve(curve: &CurveEval, [t0, t1]: [f64; 2], p: Vec3) -> Result<f64> {
    let steps = GAP_CURVE_SAMPLES - 1;
    let t_at = |i: usize| t0 + (t1 - t0) * i as f64 / steps as f64;
    let mut best = (f64::INFINITY, 0usize);
    for i in 0..GAP_CURVE_SAMPLES {
        let d = point_on(curve, t_at(i))?.distance(p);
        if d < best.0 {
            best = (d, i);
        }
    }
    let (mut lo, mut hi) = (t_at(best.1.saturating_sub(1)), t_at((best.1 + 1).min(steps)));
    let golden = 0.5 * (5f64.sqrt() - 1.0);
    let dist = |t: f64| point_on(curve, t).map(|q| q.distance(p));
    for _ in 0..GAP_REFINE_STEPS {
        let (a, b) = (hi - golden * (hi - lo), lo + golden * (hi - lo));
        if dist(a)? < dist(b)? {
            hi = b;
        } else {
            lo = a;
        }
    }
    Ok(best.0.min(dist(0.5 * (lo + hi))?))
}

/// How far the pcurve's image on `surface` strays from the 3-D `curve` over `interval`: the
/// largest distance from `surface(pcurve(t))` to the nearest point of the curve, over samples of
/// the pcurve. The parameters of the two need not agree: only the set of points is compared.
pub fn pcurve_gap(surface: &Surface, pcurve: &Curve2, curve: &Curve, interval: [f64; 2]) -> Result<f64> {
    let s = surface_eval(surface)?;
    let c = curve_eval(curve)?;
    let k = solidcore_convert::curve2(pcurve);
    let [t0, t1] = k.interval().ok_or_else(|| KernelError::Argument("pcurve has no parameter interval".into()))?;
    let steps = GAP_PCURVE_SAMPLES - 1;
    let mut worst: f64 = 0.0;
    for i in 0..GAP_PCURVE_SAMPLES {
        let t = t0 + (t1 - t0) * i as f64 / steps as f64;
        let uv = k.eval(t).ok_or_else(|| KernelError::Argument(format!("pcurve has no point at t = {t}")))?;
        let p = s.point(uv).ok_or_else(|| KernelError::Argument(format!("surface has no point at {uv:?}")))?;
        worst = worst.max(distance_to_curve(&c, interval, p)?);
    }
    Ok(worst)
}

/// The point of `surface` at `uv`, in its own parameters (STEP's for a written surface).
pub fn surface_point(surface: &Surface, uv: [f64; 2]) -> Result<[f64; 3]> {
    let p = surface_eval(surface)?
        .point(uv)
        .ok_or_else(|| KernelError::Argument(format!("surface has no point at {uv:?}")))?;
    Ok([p.x, p.y, p.z])
}

#[cfg(test)]
mod tests;
