//! Request checks and start parameters of the patch driver: the closed loop of sides, its
//! parameters (given by a flattening, or the best-fit plane), loop checks, the interior points,
//! curves and data in the same parameters, and the corner check of design doc §4.3.
use super::curve::smooth_sides;
use super::geom::{
    V2, V3, add, area2, cross, dist, dist2, dot, finite2, finite3, inside_polygon, polygon_self_intersects, scale, sub,
    thin_indices, unit,
};
use super::reference::RefProbe;
use super::{Continuity, Request};
use crate::vpatch::{MAX_CONTROL, MIN_CONTROL};
use crate::{KernelError, Result, poll};
use nalgebra::{DMatrix, Matrix3, SymmetricEigen};
use surfacecraft_geom::Cancel;

/// Most sides of a loop.
pub(super) const MAX_SIDES: usize = 64;
const MAX_SIDE_POINTS: usize = 10_000;
/// Most data points used (more are thinned deterministically by stride).
pub(super) const MAX_DATA: usize = 50_000;
const MAX_HARD_POINTS: usize = 2_000;
const MAX_SOFT_POINTS: usize = 2_000;
const MAX_CURVES: usize = 64;
const MAX_CURVE_POINTS: usize = 10_000;
/// Points of the loop checked for self-intersection (the sweep is quadratic in the worst case).
const MAX_CHECK_POINTS: usize = 6_000;
/// Points of a side the flattening's affine map is fitted from.
const MAX_AFFINE_POINTS: usize = 2_000;
/// Cosine of the largest angle between a reference normal and the plane normal (75°).
const COS_MAX_TILT: f64 = 0.258_819_045_102_520_8;
/// Share of a side's length from its ends where the tangent of a G0 side is taken.
const TANGENT_SPAN: f64 = 0.02;
/// Vertices of the polygon data points are tested against (thinned evenly per side).
const MAX_POLYGON: usize = 600;
/// Largest distance a point of a referenced side may move onto its reference face, mm.
pub(super) const SNAP_LIMIT: f64 = 0.1;
/// Search radius of the closest point on a reference face, as a share of the loop's size (at least 1 mm).
const SNAP_ACCEPT_REL: f64 = 1e-2;
/// Closest-point queries between two looks at the cancellation flag.
const POLL_EVERY: usize = 256;
const NOT_A_HEIGHT_FIELD: &str = "the outline is not a height field: give a mesh region";

fn arg(msg: impl Into<String>) -> KernelError {
    KernelError::Argument(msg.into())
}

/// `e` with the side it concerns named (the caller's number of the side).
pub(super) fn named(label: usize, e: KernelError) -> KernelError {
    match e {
        KernelError::Argument(m) => KernelError::Argument(format!("side {label}: {m}")),
        KernelError::Op { op, message } => KernelError::Op { op, message: format!("side {label}: {message}") },
        other => other,
    }
}

/// A polyline with its cumulative length.
pub(super) struct Poly {
    pub points: Vec<V3>,
    cum: Vec<f64>,
}

impl Poly {
    pub fn new(points: Vec<V3>) -> Self {
        let mut cum = Vec::with_capacity(points.len());
        let mut s = 0.0;
        for (i, p) in points.iter().enumerate() {
            if let Some(q) = i.checked_sub(1).and_then(|j| points.get(j)) {
                s += dist(*p, *q);
            }
            cum.push(s);
        }
        Self { points, cum }
    }

    pub fn length(&self) -> f64 {
        self.cum.last().copied().unwrap_or(0.0)
    }

    /// `(segment, fraction)` of the point at arc length `s` (clamped to the polyline).
    pub fn locate(&self, s: f64) -> (usize, f64) {
        let n = self.points.len();
        let s = s.clamp(0.0, self.length());
        let k = self.cum.partition_point(|c| *c < s);
        if k == 0 || n < 2 {
            return (0, 0.0);
        }
        if k >= n {
            return (n - 2, 1.0);
        }
        let (lo, hi) = (self.cum.get(k - 1).copied().unwrap_or(0.0), self.cum.get(k).copied().unwrap_or(0.0));
        (k - 1, if hi > lo { ((s - lo) / (hi - lo)).clamp(0.0, 1.0) } else { 0.0 })
    }

    pub fn point(&self, s: f64) -> V3 {
        let (k, t) = self.locate(s);
        match (self.points.get(k), self.points.get(k + 1)) {
            (Some(a), Some(b)) => lerp3(*a, *b, t),
            (Some(a), None) => *a,
            _ => [0.0; 3],
        }
    }
}

fn lerp3(a: V3, b: V3, t: f64) -> V3 {
    add(scale(a, 1.0 - t), scale(b, t))
}

fn lerp2(a: V2, b: V2, t: f64) -> V2 {
    [a[0] * (1.0 - t) + b[0] * t, a[1] * (1.0 - t) + b[1] * t]
}

/// The affine map from space to the parameter plane: `uv = A·p + b`.
#[derive(Clone, Copy, Debug)]
pub(super) struct Affine {
    a: [V3; 2],
    b: V2,
}

impl Affine {
    pub fn uv(&self, p: V3) -> V2 {
        [dot(self.a[0], p) + self.b[0], dot(self.a[1], p) + self.b[1]]
    }

    /// Unit normal of the plane the map projects onto, oriented so that a surface whose
    /// parameters `uv` agree with this map (positive Jacobian) has `S_u × S_v` along it.
    pub fn normal(&self) -> Option<V3> {
        unit(cross(self.a[0], self.a[1]))
    }

    /// The space direction (in the row space of `A`) whose image is the parameter direction `d`.
    pub fn lift(&self, d: V2) -> Option<V3> {
        let (a0, a1) = (self.a[0], self.a[1]);
        let (m00, m01, m11) = (dot(a0, a0), dot(a0, a1), dot(a1, a1));
        let det = m00 * m11 - m01 * m01;
        if !(det > 1e-12 * m00 * m11 && det.is_finite()) {
            return None;
        }
        let l0 = (m11 * d[0] - m01 * d[1]) / det;
        let l1 = (-m01 * d[0] + m00 * d[1]) / det;
        Some(add(scale(a0, l0), scale(a1, l1)))
    }
}

/// Least-squares affine map taking `points` to `uvs`.
fn fit_affine(points: &[V3], uvs: &[V2]) -> Result<Affine> {
    let n = points.len().min(uvs.len());
    if n < 3 {
        return Err(arg("the flattening of the outline has too few points"));
    }
    let mut design = DMatrix::<f64>::zeros(n, 4);
    let mut rhs = DMatrix::<f64>::zeros(n, 2);
    for (i, (p, q)) in points.iter().zip(uvs.iter()).enumerate() {
        for c in 0..3 {
            design[(i, c)] = p[c];
        }
        design[(i, 3)] = 1.0;
        rhs[(i, 0)] = q[0];
        rhs[(i, 1)] = q[1];
    }
    let x =
        design.svd(true, true).solve(&rhs, 1e-12).map_err(|e| arg(format!("the flattening cannot be mapped: {e}")))?;
    let row = |r: usize| [x[(0, r)], x[(1, r)], x[(2, r)]];
    let out = Affine { a: [row(0), row(1)], b: [x[(3, 0)], x[(3, 1)]] };
    if !(finite3(&out.a[0]) && finite3(&out.a[1]) && finite2(&out.b)) {
        return Err(arg("the flattening of the outline cannot be mapped"));
    }
    Ok(out)
}

/// One side of the loop in space and in parameters.
pub(super) struct SideData {
    /// The caller's number of the side (for messages).
    pub label: usize,
    pub poly: Poly,
    pub uv: Vec<V2>,
    pub continuity: Continuity,
    pub probe: Option<RefProbe>,
    /// The side is approximate: the patch follows the data, the gap is reported only.
    pub soft: bool,
    /// Largest move of a point onto the reference face (G1/G2 sides).
    pub snap: Option<f64>,
}

impl SideData {
    pub fn length(&self) -> f64 {
        self.poly.length()
    }

    /// The point at arc length `s` and its parameters.
    pub fn at(&self, s: f64) -> (V3, V2) {
        let (k, t) = self.poly.locate(s);
        let uv = match (self.uv.get(k), self.uv.get(k + 1)) {
            (Some(a), Some(b)) => lerp2(*a, *b, t),
            (Some(a), None) => *a,
            _ => [0.0; 2],
        };
        (self.poly.point(s), uv)
    }

    /// Unit direction of the side in the parameter plane at arc length `s`.
    pub fn tangent_uv(&self, s: f64) -> Result<V2> {
        let len = self.length();
        let d = TANGENT_SPAN * len;
        let (a, b) = (self.at((s - d).max(0.0)).1, self.at((s + d).min(len)).1);
        let t = [b[0] - a[0], b[1] - a[1]];
        let n = t[0].hypot(t[1]);
        if n.is_finite() && n > 1e-300 {
            Ok([t[0] / n, t[1] / n])
        } else {
            Err(arg("a side has no direction in the parameter plane"))
        }
    }

    /// Unit space tangent at an end (`end = true` for the last point), pointing along the side.
    pub fn end_tangent(&self, end: bool) -> Option<V3> {
        let len = self.length();
        let d = TANGENT_SPAN * len;
        let (a, b) = if end {
            (self.poly.point(len - d), self.poly.point(len))
        } else {
            (self.poly.point(0.0), self.poly.point(d))
        };
        unit(sub(b, a))
    }
}

/// The closed loop: sides, the map to parameters and what the later stages need of it.
pub(super) struct Loop {
    pub sides: Vec<SideData>,
    pub affine: Affine,
    /// The loop runs counter-clockwise in the parameter plane.
    pub ccw: bool,
    /// Bounding box of the loop in parameters.
    pub bbox: [[f64; 2]; 2],
    /// Diagonal of the loop's bounding box in space.
    pub diag: f64,
    /// The outline in the parameter plane (thinned): what lies outside it is not on the trimmed sheet.
    pub polygon: Vec<V2>,
}

impl Loop {
    /// Does the parameter point lie inside the outline?
    pub fn contains(&self, uv: V2) -> bool {
        inside_polygon(&self.polygon, uv)
    }

    /// The parameter direction normal to `t`, pointing into the loop.
    pub fn inward(&self, t: V2) -> V2 {
        if self.ccw { [-t[1], t[0]] } else { [t[1], -t[0]] }
    }
}

/// A sample with its parameters and weight.
#[derive(Clone, Copy, Debug)]
pub(super) struct Pt {
    pub xyz: V3,
    pub uv: V2,
    pub w: f64,
    /// Data only: the point lies outside the outline in the parameter plane and is ignored.
    pub out: bool,
}

/// Points of the interior: scan data (soft), soft points, hard points and hard curves.
pub(super) struct Interior {
    pub data: Vec<Pt>,
    /// Points the patch should pass near (`w` is their weight against a data point's 1). They are
    /// not data: they never count toward the minimum number of data points and are never thinned.
    pub soft: Vec<Pt>,
    pub hard: Vec<Pt>,
    pub curves: Vec<Vec<Pt>>,
}

/// A corner of the loop: where side `before` ends and side `after` starts.
#[derive(Clone, Copy, Debug)]
pub(super) struct Corner {
    pub at: V3,
    pub before: usize,
    pub after: usize,
    pub mismatch_deg: f64,
}

pub(super) struct Prepared {
    pub lp: Loop,
    pub interior: Interior,
    pub corners: Vec<Corner>,
    /// Sides that could not be smoothed (see [`smooth_sides`]).
    pub warnings: Vec<String>,
}

fn validate(req: &Request) -> Result<()> {
    if req.sides.is_empty() || req.sides.len() > MAX_SIDES {
        return Err(arg(format!("a loop has 1 to {MAX_SIDES} sides, not {}", req.sides.len())));
    }
    if !(req.tolerance.is_finite() && req.tolerance > 0.0 && req.tolerance <= 1e6) {
        return Err(arg("the tolerance must be finite and positive"));
    }
    if let Some(s) = req.smoothing
        && !(s.is_finite() && s > 0.0 && s <= 1e6)
    {
        return Err(arg("the smoothing must be finite and positive"));
    }
    if req.max_control < MIN_CONTROL || req.max_control > MAX_CONTROL {
        return Err(arg(format!("max_control must be {MIN_CONTROL} to {MAX_CONTROL}, not {}", req.max_control)));
    }
    if let Some(c) = req.control
        && c.iter().any(|n| *n < MIN_CONTROL || *n > req.max_control)
    {
        return Err(arg(format!(
            "the control net {} × {} must have {MIN_CONTROL} to {} points per direction",
            c[0], c[1], req.max_control
        )));
    }
    let flat = req.sides.iter().all(|s| s.uv.is_some());
    if req.sides.iter().any(|s| s.uv.is_some()) && !flat {
        return Err(arg("either every side has parameters or none"));
    }
    for (i, s) in req.sides.iter().enumerate() {
        if s.points.len() < 2 || s.points.len() > MAX_SIDE_POINTS {
            return Err(arg(format!("side {i} has {} points; 2 to {MAX_SIDE_POINTS} are accepted", s.points.len())));
        }
        if !s.points.iter().all(finite3) {
            return Err(arg(format!("side {i} has a point that is not finite")));
        }
        if s.continuity != Continuity::G0 && s.reference.is_none() {
            return Err(arg(format!("side {i} asks for G1 or G2 but gives no reference face")));
        }
        if let Some(uv) = &s.uv
            && (uv.len() != s.points.len() || !uv.iter().all(finite2))
        {
            return Err(arg(format!("side {i}: the parameters must be finite, one per point")));
        }
    }
    if req.data.iter().any(|p| !finite3(p)) {
        return Err(arg("a data point is not finite"));
    }
    if let Some(uv) = &req.data_uv
        && (!flat || uv.len() != req.data.len() || !uv.iter().all(finite2))
    {
        return Err(arg("data_uv needs a flattening of the sides and one finite pair per data point"));
    }
    if req.hard_points.len() > MAX_HARD_POINTS || req.hard_points.iter().any(|p| !finite3(p)) {
        return Err(arg(format!("at most {MAX_HARD_POINTS} finite hard points are accepted")));
    }
    if req.soft_points.len() > MAX_SOFT_POINTS
        || req.soft_points.iter().any(|(p, w)| !finite3(p) || !(w.is_finite() && *w > 0.0 && *w <= 1e6))
    {
        return Err(arg(format!(
            "at most {MAX_SOFT_POINTS} soft points are accepted, finite, with a positive weight of at most 1e6"
        )));
    }
    if req.curves.len() > MAX_CURVES
        || req.curves.iter().any(|c| c.len() < 2 || c.len() > MAX_CURVE_POINTS || !c.iter().all(finite3))
    {
        return Err(arg(format!("at most {MAX_CURVES} curves of 2 to {MAX_CURVE_POINTS} finite points are accepted")));
    }
    Ok(())
}

/// Points that differ by more than `tol`, with their parameters.
fn dedupe(points: &[V3], uv: Option<&Vec<V2>>, tol: f64) -> (Vec<V3>, Vec<V2>) {
    let mut out = Vec::with_capacity(points.len());
    let mut out_uv = Vec::new();
    for (i, p) in points.iter().enumerate() {
        if out.last().is_none_or(|l| dist(*l, *p) > tol) {
            out.push(*p);
            if let Some(q) = uv.and_then(|u| u.get(i)) {
                out_uv.push(*q);
            }
        }
    }
    (out, out_uv)
}

struct Plane {
    c: V3,
    e1: V3,
    e2: V3,
    n: V3,
}

/// Best-fit plane of the side points (weighted by the length each represents).
fn fit_plane(sides: &[Vec<V3>]) -> Result<Plane> {
    let mut weighted: Vec<(V3, f64)> = Vec::new();
    for pts in sides {
        for (i, p) in pts.iter().enumerate() {
            let mut w = 0.0;
            if let Some(q) = i.checked_sub(1).and_then(|j| pts.get(j)) {
                w += 0.5 * dist(*p, *q);
            }
            if let Some(q) = pts.get(i + 1) {
                w += 0.5 * dist(*p, *q);
            }
            weighted.push((*p, w));
        }
    }
    let total: f64 = weighted.iter().map(|(_, w)| w).sum();
    if !(total > 0.0 && total.is_finite()) {
        return Err(arg("the outline has no length"));
    }
    let mut c = [0.0; 3];
    for (p, w) in &weighted {
        c = add(c, scale(*p, w / total));
    }
    let mut cov = Matrix3::<f64>::zeros();
    for (p, w) in &weighted {
        let d = sub(*p, c);
        for r in 0..3 {
            for k in 0..3 {
                cov[(r, k)] += w * d[r] * d[k] / total;
            }
        }
    }
    let eig = SymmetricEigen::new(cov);
    let mut idx = [0usize, 1, 2];
    idx.sort_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b]));
    let col = |k: usize| {
        let v = eig.eigenvectors.column(idx[k]);
        [v[0], v[1], v[2]]
    };
    let (small, mid, big) = (eig.eigenvalues[idx[0]], eig.eigenvalues[idx[1]], eig.eigenvalues[idx[2]]);
    if !(mid > 1e-12 * big && big.is_finite() && small.is_finite()) {
        return Err(arg("the outline is degenerate (its points are collinear)"));
    }
    let (Some(n), Some(e1)) = (unit(col(0)), unit(col(2))) else {
        return Err(arg("the outline has no best-fit plane"));
    };
    Ok(Plane { c, e1, e2: cross(n, e1), n })
}

fn bbox2(points: impl Iterator<Item = V2>) -> [[f64; 2]; 2] {
    let mut b = [[f64::INFINITY, f64::NEG_INFINITY], [f64::INFINITY, f64::NEG_INFINITY]];
    for p in points {
        for d in 0..2 {
            b[d][0] = b[d][0].min(p[d]);
            b[d][1] = b[d][1].max(p[d]);
        }
    }
    b
}

/// Moves every point of a G1/G2 side onto its reference face (a closest point) and keeps the loop
/// closed: a corner takes the position on the referenced side, the middle of the two where both are
/// referenced. Returns the largest move per referenced side; a side that lies more than
/// [`SNAP_LIMIT`] off its face is refused.
fn snap_to_references(
    pts: &mut [Vec<V3>],
    probes: &[Option<RefProbe>],
    labels: &[usize],
    accept: f64,
    cancel: &Cancel,
) -> Result<Vec<Option<f64>>> {
    let before: Vec<Vec<V3>> = pts.to_vec();
    let n = pts.len();
    let mut snapped = vec![false; n];
    for (k, (side, probe)) in pts.iter_mut().zip(probes).enumerate() {
        let Some(probe) = probe else { continue };
        let label = labels.get(k).copied().unwrap_or(k);
        for (i, p) in side.iter_mut().enumerate() {
            if i % POLL_EVERY == 0 {
                poll(cancel)?;
            }
            match probe.closest(*p, accept).map_err(|e| named(label, e))? {
                Some(q) => *p = q,
                None => {
                    return Err(arg(format!(
                        "side {label} lies more than {accept:.3} mm off its reference face (is the face adjacent to the side?)"
                    )));
                }
            }
        }
        if let Some(flag) = snapped.get_mut(k) {
            *flag = true;
        }
    }
    for k in 0..n {
        let nxt = (k + 1) % n;
        let was = |i: usize| snapped.get(i).copied().unwrap_or(false);
        let ends = (pts.get(k).and_then(|p| p.last()).copied(), pts.get(nxt).and_then(|p| p.first()).copied());
        let (Some(end), Some(start)) = ends else { continue };
        let corner = match (was(k), was(nxt)) {
            (false, false) => continue,
            (true, false) => end,
            (false, true) => start,
            (true, true) => scale(add(end, start), 0.5),
        };
        if let Some(l) = pts.get_mut(k).and_then(|p| p.last_mut()) {
            *l = corner;
        }
        if let Some(f) = pts.get_mut(nxt).and_then(|p| p.first_mut()) {
            *f = corner;
        }
    }
    let mut out = Vec::with_capacity(n);
    for (k, (now, was)) in pts.iter().zip(&before).enumerate() {
        if !snapped.get(k).copied().unwrap_or(false) {
            out.push(None);
            continue;
        }
        let moved = now.iter().zip(was).map(|(a, b)| dist(*a, *b)).fold(0.0_f64, f64::max);
        if moved.is_nan() || moved > SNAP_LIMIT {
            let label = labels.get(k).copied().unwrap_or(k);
            return Err(arg(format!("side {label} lies {moved:.3} mm off its reference face (at most {SNAP_LIMIT} mm)")));
        }
        out.push(Some(moved));
    }
    Ok(out)
}

/// The size of the loop: its largest extent along an axis and the diagonal of its box.
struct Extent {
    span: f64,
    diag: f64,
}

fn extent(req: &Request) -> Extent {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for p in req.sides.iter().flat_map(|s| s.points.iter()) {
        for c in 0..3 {
            lo[c] = lo[c].min(p[c]);
            hi[c] = hi[c].max(p[c]);
        }
    }
    Extent { span: (0..3).map(|c| hi[c] - lo[c]).fold(0.0_f64, f64::max), diag: dist(lo, hi) }
}

/// The caller's number of every side.
fn labels(req: &Request) -> Vec<usize> {
    (0..req.sides.len()).map(|i| req.side_ids.get(i).copied().unwrap_or(i)).collect()
}

type SidePoints = (Vec<Vec<V3>>, Vec<Vec<V2>>);

/// The sides without repeated points (and their parameters).
fn dedupe_sides(req: &Request, labels: &[usize], tol: f64) -> Result<SidePoints> {
    let mut pts: Vec<Vec<V3>> = Vec::with_capacity(req.sides.len());
    let mut uvs: Vec<Vec<V2>> = Vec::with_capacity(req.sides.len());
    for (i, s) in req.sides.iter().enumerate() {
        let (p, q) = dedupe(&s.points, s.uv.as_ref(), tol);
        if p.len() < 2 {
            return Err(arg(format!("side {} has no length", labels.get(i).copied().unwrap_or(i))));
        }
        pts.push(p);
        uvs.push(q);
    }
    Ok((pts, uvs))
}

/// Closes the loop: the next side starts where this one ends (within `close_tol`).
fn close_loop(pts: &mut [Vec<V3>], uvs: &mut [Vec<V2>], flat: bool, close_tol: f64, labels: &[usize]) -> Result<()> {
    let n = pts.len();
    let end_of = |k: usize| {
        let end = pts.get(k).and_then(|p| p.last()).copied().unwrap_or([0.0; 3]);
        (end, uvs.get(k).and_then(|q| q.last()).copied().unwrap_or([0.0; 2]))
    };
    let ends: Vec<(V3, V2)> = (0..n).map(end_of).collect();
    let uv_close = ends.iter().map(|(_, q)| q[0].abs().max(q[1].abs())).fold(1.0_f64, f64::max) * 1e-7;
    let label = |k: usize| labels.get(k).copied().unwrap_or(k);
    for (k, &(end, end_uv)) in ends.iter().enumerate() {
        let nxt = (k + 1) % n;
        let first = pts.get(nxt).and_then(|p| p.first()).copied().ok_or_else(|| arg("a side has no points"))?;
        let d = dist(end, first);
        if d > close_tol {
            return Err(arg(format!(
                "the sides do not close: side {} ends {d:.3e} mm from the start of side {}",
                label(k),
                label(nxt)
            )));
        }
        let first_uv = uvs.get(nxt).and_then(|q| q.first()).copied();
        if flat && first_uv.is_none_or(|f| dist2(end_uv, f) > uv_close) {
            return Err(arg(format!(
                "the parameters of side {} and side {} do not meet at their corner",
                label(k),
                label(nxt)
            )));
        }
        if let Some(f) = pts.get_mut(nxt).and_then(|p| p.first_mut()) {
            *f = end;
        }
        if flat && let Some(f) = uvs.get_mut(nxt).and_then(|q| q.first_mut()) {
            *f = end_uv;
        }
    }
    Ok(())
}

/// The reference probes of the G1/G2 sides.
fn reference_probes(req: &Request, labels: &[usize]) -> Result<Vec<Option<RefProbe>>> {
    req.sides
        .iter()
        .enumerate()
        .map(|(i, s)| {
            if s.continuity == Continuity::G0 {
                return Ok(None);
            }
            let label = labels.get(i).copied().unwrap_or(i);
            s.reference.as_ref().map(RefProbe::new).transpose().map_err(|e| named(label, e))
        })
        .collect()
}

/// The affine map of the loop: fitted to the given parameters (a flattening), else the best-fit
/// plane, oriented like the first reference face's normal; a G1/G2 side whose reference leans more
/// than 75° off that plane makes the outline no height field.
fn loop_affine(
    pts: &[Vec<V3>],
    uvs: &[Vec<V2>],
    flat: bool,
    probes: &[Option<RefProbe>],
    labels: &[usize],
) -> Result<Affine> {
    if flat {
        let all_pts: Vec<V3> = pts.iter().flatten().copied().collect();
        let all_uv: Vec<V2> = uvs.iter().flatten().copied().collect();
        let keep = thin_indices(all_pts.len(), MAX_AFFINE_POINTS);
        let p: Vec<V3> = keep.iter().filter_map(|&i| all_pts.get(i).copied()).collect();
        let q: Vec<V2> = keep.iter().filter_map(|&i| all_uv.get(i).copied()).collect();
        return fit_affine(&p, &q);
    }
    let label = |k: usize| labels.get(k).copied().unwrap_or(k);
    let mut plane = fit_plane(pts)?;
    let first = probes.iter().zip(pts.iter()).enumerate().find_map(|(k, (pr, p))| pr.as_ref().zip(p.first()).map(|x| (k, x)));
    let flip = match first {
        Some((k, (pr, p))) => dot(plane.n, pr.locate(*p).map_err(|e| named(label(k), e))?.normal) < 0.0,
        None => {
            let k = (0..3).fold(0, |b, c| if plane.n[c].abs() > plane.n[b].abs() { c } else { b });
            plane.n[k] < 0.0
        }
    };
    if flip {
        plane.n = scale(plane.n, -1.0);
        plane.e2 = cross(plane.n, plane.e1);
    }
    for (k, (pr, p)) in probes.iter().zip(pts.iter()).enumerate() {
        let Some(pr) = pr else { continue };
        for i in thin_indices(p.len(), 5) {
            let Some(q) = p.get(i) else { continue };
            if dot(pr.locate(*q).map_err(|e| named(label(k), e))?.normal, plane.n).abs() < COS_MAX_TILT {
                return Err(arg(NOT_A_HEIGHT_FIELD));
            }
        }
    }
    Ok(Affine { a: [plane.e1, plane.e2], b: [-dot(plane.e1, plane.c), -dot(plane.e2, plane.c)] })
}

/// A side's parameters without the last point (which is the next side's first).
fn body(u: &[V2]) -> &[V2] {
    u.get(..u.len().saturating_sub(1)).unwrap_or_default()
}

/// What the outline in the parameter plane gives the later stages.
struct Outline {
    ccw: bool,
    bbox: [[f64; 2]; 2],
    polygon: Vec<V2>,
}

/// Checks the outline in the parameter plane (not degenerate, not crossing itself) and thins it.
fn check_outline(side_uv: &[Vec<V2>], flat: bool) -> Result<Outline> {
    let n = side_uv.len();
    let outline: Vec<V2> = side_uv.iter().flat_map(|u| body(u).iter().copied()).collect();
    let bbox = bbox2(side_uv.iter().flatten().copied());
    let ext = [bbox[0][1] - bbox[0][0], bbox[1][1] - bbox[1][0]];
    if !(ext[0].is_finite() && ext[1].is_finite() && ext[0].min(ext[1]) > 1e-9 * ext[0].max(ext[1])) {
        return Err(arg("the outline is degenerate in the parameter plane"));
    }
    let thinned = |cap: usize| -> Vec<V2> {
        side_uv
            .iter()
            .flat_map(|u| {
                let b = body(u);
                thin_indices(b.len(), cap).into_iter().filter_map(move |i| b.get(i).copied())
            })
            .collect()
    };
    // crossing check on a thinned copy
    if polygon_self_intersects(&thinned((MAX_CHECK_POINTS / n).max(8)), 1e-9 * ext[0].max(ext[1])) {
        return Err(arg(if flat { "the flattening of the outline crosses itself" } else { NOT_A_HEIGHT_FIELD }));
    }
    Ok(Outline { ccw: area2(&outline) > 0.0, bbox, polygon: thinned((MAX_POLYGON / n).max(8)) })
}

/// The loop with parameters, checked: closed, a height field (or a flattening that does not cross
/// itself), its references usable. Hard sides of three or more points are densified along a smooth
/// curve through their points; the warnings say which could not be.
fn build_loop(req: &Request, cancel: &Cancel) -> Result<(Loop, Vec<String>)> {
    let flat = req.sides.iter().all(|s| s.uv.is_some());
    let ex = extent(req);
    let labels = labels(req);
    let (mut pts, mut uvs) = dedupe_sides(req, &labels, 1e-9 * ex.span.max(1.0))?;
    close_loop(&mut pts, &mut uvs, flat, 1e-7 * ex.span.max(1.0), &labels)?;
    if pts.iter().map(|p| p.len().saturating_sub(1)).sum::<usize>() < 3 {
        return Err(arg("the outline needs at least 3 distinct points"));
    }
    let probes = reference_probes(req, &labels)?;
    let accept = (SNAP_ACCEPT_REL * ex.span).max(10.0 * SNAP_LIMIT);
    let snaps = snap_to_references(&mut pts, &probes, &labels, accept, cancel)?;
    poll(cancel)?;
    // the points are samples of a smooth curve: the dense samples between them follow it, not the chords
    let soft: Vec<bool> = req.sides.iter().map(|s| s.soft).collect();
    let notes = smooth_sides(&mut pts, &mut uvs, flat, &soft, ex.diag);
    poll(cancel)?;
    let affine = loop_affine(&pts, &uvs, flat, &probes, &labels)?;
    let side_uv: Vec<Vec<V2>> =
        if flat { uvs } else { pts.iter().map(|p| p.iter().map(|q| affine.uv(*q)).collect()).collect() };
    let Outline { ccw, bbox, polygon } = check_outline(&side_uv, flat)?;
    let sides: Vec<SideData> = req
        .sides
        .iter()
        .zip(pts)
        .zip(side_uv)
        .zip(probes)
        .zip(snaps)
        .zip(labels)
        .map(|(((((s, p), q), probe), snap), label)| SideData {
            label,
            poly: Poly::new(p),
            uv: q,
            continuity: s.continuity,
            probe,
            soft: s.soft,
            snap,
        })
        .collect();
    Ok((Loop { sides, affine, ccw, bbox, diag: ex.diag, polygon }, notes))
}

/// A curve as `n` points equally spaced in arc length.
fn resample(points: &[V3], n: usize, tol: f64) -> Result<Vec<V3>> {
    let (clean, _) = dedupe(points, None, tol);
    if clean.len() < 2 {
        return Err(arg("an interior curve has no length"));
    }
    let poly = Poly::new(clean);
    let len = poly.length();
    Ok((0..n).map(|j| poly.point(len * j as f64 / (n - 1) as f64)).collect())
}

fn interior(req: &Request, lp: &Loop) -> Result<Interior> {
    let pt = |p: V3, w: f64| Pt { xyz: p, uv: lp.affine.uv(p), w, out: false };
    let datum = |xyz: V3, uv: V2, w: f64| Pt { xyz, uv, w, out: !lp.contains(uv) };
    let mut data: Vec<Pt> = Vec::with_capacity(req.data.len());
    for (i, p) in req.data.iter().enumerate() {
        let uv = req.data_uv.as_ref().and_then(|u| u.get(i)).copied().unwrap_or_else(|| lp.affine.uv(*p));
        data.push(datum(*p, uv, 1.0));
    }
    let data: Vec<Pt> = thin_indices(data.len(), MAX_DATA).into_iter().filter_map(|i| data.get(i).copied()).collect();
    let hard: Vec<Pt> = req.hard_points.iter().map(|p| pt(*p, 1.0)).collect();
    // a soft point outside the outline cannot be followed (like a hard point it is an error, not ignored)
    let mut soft: Vec<Pt> = Vec::with_capacity(req.soft_points.len());
    for (k, (p, w)) in req.soft_points.iter().enumerate() {
        let q = pt(*p, *w);
        if !lp.contains(q.uv) {
            return Err(arg(format!("soft point {k} lies outside the outline")));
        }
        soft.push(q);
    }

    // curves: points enough for the finest net, equally spaced
    let cells = (req.max_control - 3) as f64;
    let cell = [(lp.bbox[0][1] - lp.bbox[0][0]) / cells, (lp.bbox[1][1] - lp.bbox[1][0]) / cells];
    let span = lp.diag.max(1.0);
    let mut curves = Vec::with_capacity(req.curves.len());
    for c in &req.curves {
        let uv: Vec<V2> = c.iter().map(|p| lp.affine.uv(*p)).collect();
        let path: f64 =
            uv.windows(2).map(|w| (w[1][0] - w[0][0]).abs() / cell[0] + (w[1][1] - w[0][1]).abs() / cell[1]).sum();
        let n = (20 * (path.ceil().clamp(1.0, 40.0) as usize) + 1).clamp(21, 401);
        let pts = resample(c, n, 1e-9 * span)?;
        curves.push(pts.into_iter().map(|p| pt(p, 1.0)).collect());
    }
    Ok(Interior { data, soft, hard, curves })
}

/// Angle between the tangent planes (or a tangent and a plane) of the two sides at each corner.
fn corners(lp: &Loop) -> Result<Vec<Corner>> {
    let n = lp.sides.len();
    let mut out = Vec::new();
    if n < 2 {
        return Ok(out);
    }
    for k in 0..n {
        let (Some(a), Some(b)) = (lp.sides.get(k), lp.sides.get((k + 1) % n)) else { continue };
        let at = a.poly.points.last().copied().unwrap_or([0.0; 3]);
        let normal = |s: &SideData| -> Result<Option<V3>> {
            s.probe.as_ref().map(|p| p.locate(at).map(|r| r.normal).map_err(|e| named(s.label, e))).transpose()
        };
        let off_plane = |t: Option<V3>, n: V3| t.map_or(0.0, |t| dot(t, n).abs().min(1.0).asin().to_degrees());
        let mismatch = match (normal(a)?, normal(b)?) {
            (Some(na), Some(nb)) => dot(na, nb).abs().min(1.0).acos().to_degrees(),
            (Some(na), None) => off_plane(b.end_tangent(false), na),
            (None, Some(nb)) => off_plane(a.end_tangent(true), nb),
            (None, None) => 0.0,
        };
        out.push(Corner { at, before: k, after: (k + 1) % n, mismatch_deg: mismatch });
    }
    Ok(out)
}

/// Checks the request and prepares the loop, the interior and the corner angles.
pub(super) fn prepare(req: &Request, cancel: &Cancel) -> Result<Prepared> {
    validate(req)?;
    let (lp, warnings) = build_loop(req, cancel)?;
    let interior = interior(req, &lp)?;
    let corners = corners(&lp)?;
    Ok(Prepared { lp, interior, corners, warnings })
}

/// Hard points and curves must lie in the parameter domain (a hard condition cannot be ignored).
pub(super) fn check_hard_inside(interior: &Interior, domain: &[[f64; 2]; 2]) -> Result<()> {
    let inside = |p: &Pt| (0..2).all(|d| p.uv[d] >= domain[d][0] && p.uv[d] <= domain[d][1]);
    if interior.hard.iter().any(|p| !inside(p)) || interior.curves.iter().flatten().any(|p| !inside(p)) {
        return Err(arg("a hard point or curve lies outside the patch's parameter domain (beyond the outline)"));
    }
    Ok(())
}
