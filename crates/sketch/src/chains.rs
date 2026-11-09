//! Whole section chains fitted into constrained sketches (design doc §6.4).
//!
//! Each chain (a polyline of a mesh section) becomes a connected sequence of lines and arcs:
//!
//! 1. *Split and merge* (Ramer–Douglas–Peucker splitting, then greedy merging of neighbours whose
//!    union is still one primitive, then absorbing slivers and sliding every boundary to the
//!    least-squares optimum). A piece is a line or an arc when its RMS deviation is within the
//!    tolerance and its maximum within three times the tolerance; an arc must also deviate from its
//!    chord by more than twice the tolerance, so a flat arc stays a line.
//! 2. The sketch frame is turned by the dominant line direction (when the lines lie within
//!    [`MAX_FRAME_OFFSET_DEGREES`] of the axes), so a part scanned a little off the plane axes
//!    still gets horizontal and vertical lines.
//! 3. Neighbouring entities share their junction point (the point nearest to both fitted
//!    primitives); a closed chain closes, an open one stays open.
//! 4. Relations (horizontal, vertical, tangent, concentric, collinear, equal radius, parallel,
//!    perpendicular) are proposed from the fitted geometry in that priority order and added one by
//!    one: a relation is kept only when the solver still converges, the sketch is not
//!    over-defined (no redundant relation is added) and the chain stays within the tolerance of
//!    the section points.
use crate::{Constraint, Entity, Sketch, SketchPlane, SolveReport, SolveStatus, polish, solve};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::f64::consts::{PI, TAU};

type P2 = [f64; 2];

/// Largest number of samples per chain that is fitted (longer chains are thinned evenly).
pub const MAX_SAMPLES: usize = 8000;
/// Chains with more entities than this get their lines and arcs but no relations.
pub const MAX_RELATION_ENTITIES: usize = 40;
/// Entities of all chains together; further chains are skipped.
pub const MAX_TOTAL_ENTITIES: usize = 500;
/// Lines whose direction is within this many degrees of an axis of the sketch frame turn the frame.
pub const MAX_FRAME_OFFSET_DEGREES: f64 = 5.0;
const MAX_PIECES: usize = 1500;
const MAX_CANDIDATES: usize = 300;
const MAX_REJECTED_LISTED: usize = 200;
const FIT_SAMPLES: usize = 256;
const RMS_FACTOR: f64 = 1.0;
const MAX_FACTOR: f64 = 3.0;
/// An arc must stand off its chord by more than this many tolerances, else it is a line.
const FLAT: f64 = 2.0;

/// A chain of section samples in sketch coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Chain2 {
    /// Samples in order.
    pub points: Vec<[f64; 2]>,
    /// Does the last sample connect to the first?
    pub closed: bool,
}

/// What to fit.
#[derive(Clone, Debug, PartialEq)]
pub struct ChainOptions {
    /// Fit tolerance (mm): RMS bound of a piece and of the chain after relations (the maximum may be
    /// three times as large). Use [`ChainOptions::tolerance_for_noise`] for scans of a known noise.
    pub tolerance: f64,
    /// Angle within which lines count as horizontal, vertical, parallel or perpendicular (degrees).
    pub angle_tolerance_degrees: f64,
    /// Turn the sketch frame to the dominant line direction?
    pub align_frame: bool,
}

impl Default for ChainOptions {
    fn default() -> Self {
        Self { tolerance: 0.05, angle_tolerance_degrees: 1.0, align_frame: true }
    }
}

impl ChainOptions {
    /// `max(0.05 mm, 2 σ)` for scan noise `sigma` (mm).
    #[must_use]
    pub fn tolerance_for_noise(sigma: f64) -> f64 {
        if sigma.is_finite() && sigma > 0.0 { (2.0 * sigma).max(0.05) } else { 0.05 }
    }
}

/// RMS and maximum distance of the section samples from their entities.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Deviation {
    /// Root mean square (mm).
    pub rms: f64,
    /// Largest (mm).
    pub max: f64,
}

/// A relation between sketch entities.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RelationKind {
    /// Line parallel to the sketch x axis.
    Horizontal,
    /// Line parallel to the sketch y axis.
    Vertical,
    /// Lines parallel.
    Parallel,
    /// Lines perpendicular.
    Perpendicular,
    /// Lines on one straight line.
    Collinear,
    /// Line and arc, or two arcs, tangent.
    Tangent,
    /// Arcs of equal radius.
    EqualRadius,
    /// Arcs around one centre.
    Concentric,
}

/// A relation that was kept (entity indices are those of the whole sketch).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Relation {
    /// Kind.
    pub kind: RelationKind,
    /// The entities it ties.
    pub entities: Vec<usize>,
}

/// A relation that was proposed and refused.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct RejectedRelation {
    /// Kind.
    pub kind: RelationKind,
    /// The entities it would tie.
    pub entities: Vec<usize>,
    /// Why it was refused.
    pub reason: String,
}

/// What became of one chain.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ChainReport {
    /// Index of the chain in the input.
    pub chain: usize,
    /// Is it closed?
    pub closed: bool,
    /// Samples fitted (after removing duplicates and thinning).
    pub samples: usize,
    /// Index of its first entity in the sketch.
    pub first_entity: usize,
    /// Number of entities (lines, arcs and circles).
    pub entities: usize,
    /// Lines among them.
    pub lines: usize,
    /// Arcs among them.
    pub arcs: usize,
    /// Circles among them.
    pub circles: usize,
    /// Relations kept, in the order they were added.
    pub relations_kept: Vec<Relation>,
    /// Relations refused (the first 200).
    pub relations_rejected: Vec<RejectedRelation>,
    /// How many relations were refused in all.
    pub rejected_total: usize,
    /// Deviation of the fitted lines and arcs, before any relation.
    pub before: Deviation,
    /// Deviation after the relations were solved.
    pub after: Deviation,
    /// Solve status of the chain after its relations.
    pub status: Option<SolveStatus>,
    /// Degrees of freedom left in the chain.
    pub dof: usize,
    /// Why the chain was skipped or has no relations.
    pub note: Option<String>,
}

/// The result of [`fit_chains`].
#[derive(Clone, Debug, PartialEq)]
pub struct ChainsFit {
    /// The sketch (its plane is the input plane turned by `frame_rotation_degrees`).
    pub sketch: Sketch,
    /// How far the sketch frame was turned (degrees, counter-clockwise).
    pub frame_rotation_degrees: f64,
    /// One report per input chain.
    pub chains: Vec<ChainReport>,
    /// Aggregated solve report of the chains (each chain is solved on its own).
    pub report: SolveReport,
}

// ---------------------------------------------------------------------------------------------
// plane geometry

fn sub(a: P2, b: P2) -> P2 {
    [a[0] - b[0], a[1] - b[1]]
}

fn dist(a: P2, b: P2) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

fn dot(a: P2, b: P2) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn cross(a: P2, b: P2) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn unit(v: P2) -> Option<P2> {
    let n = v[0].hypot(v[1]);
    (n > 1e-12 && n.is_finite()).then(|| [v[0] / n, v[1] / n])
}

/// A fitted curve: a line (point and unit direction) or a circle.
#[derive(Clone, Copy, Debug)]
enum Prim {
    Line { o: P2, d: P2 },
    Circle { c: P2, r: f64 },
}

impl Prim {
    fn dist(&self, q: P2) -> f64 {
        match *self {
            Prim::Line { o, d } => cross(sub(q, o), d).abs(),
            Prim::Circle { c, r } => (dist(q, c) - r).abs(),
        }
    }

    /// Signed distance and its gradient at `q`.
    fn signed(&self, q: P2) -> (f64, P2) {
        match *self {
            Prim::Line { o, d } => (cross(sub(q, o), d), [d[1], -d[0]]),
            Prim::Circle { c, r } => {
                let v = sub(q, c);
                let n = v[0].hypot(v[1]).max(1e-12);
                (n - r, [v[0] / n, v[1] / n])
            }
        }
    }

    /// The point of the curve nearest to `q`.
    fn project(&self, q: P2) -> P2 {
        match *self {
            Prim::Line { o, d } => {
                let t = dot(sub(q, o), d);
                [o[0] + d[0] * t, o[1] + d[1] * t]
            }
            Prim::Circle { c, r } => match unit(sub(q, c)) {
                Some(u) => [c[0] + u[0] * r, c[1] + u[1] * r],
                None => [c[0] + r, c[1]],
            },
        }
    }
}

/// The point nearest to both curves, found from `p0` (their intersection, or the point of tangency
/// when they touch). Falls back to the midpoint of the projections when it runs away.
fn junction(a: &Prim, b: &Prim, p0: P2, limit: f64) -> P2 {
    let mut p = p0;
    for _ in 0..40 {
        let ((ra, ga), (rb, gb)) = (a.signed(p), b.signed(p));
        let lam = 1e-6;
        let m00 = ga[0] * ga[0] + gb[0] * gb[0] + lam;
        let m01 = ga[0] * ga[1] + gb[0] * gb[1];
        let m11 = ga[1] * ga[1] + gb[1] * gb[1] + lam;
        let (b0, b1) = (-(ga[0] * ra + gb[0] * rb), -(ga[1] * ra + gb[1] * rb));
        let det = m00 * m11 - m01 * m01;
        if det.abs() < 1e-18 {
            break;
        }
        let (dx, dy) = ((b0 * m11 - b1 * m01) / det, (m00 * b1 - m01 * b0) / det);
        p = [p[0] + dx, p[1] + dy];
        if dx.hypot(dy) < 1e-12 {
            break;
        }
    }
    if p.iter().all(|v| v.is_finite()) && dist(p, p0) <= limit {
        p
    } else {
        let (pa, pb) = (a.project(p0), b.project(p0));
        [(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0]
    }
}

/// Where two curves that touch within three tolerances touch (midway between their nearest
/// points); `None` when they do not touch or cross at a corner. The point of tangency of a line and
/// an arc is not fixed by the samples along the line, so it is taken from the curves alone.
fn tangent_point(a: &Prim, b: &Prim, tol: f64) -> Option<P2> {
    let touch = 3.0 * tol;
    let mid = |p: P2, q: P2| [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0];
    match (a, b) {
        (line @ Prim::Line { .. }, Prim::Circle { c, r }) | (Prim::Circle { c, r }, line @ Prim::Line { .. }) => {
            let foot = line.project(*c);
            if (dist(foot, *c) - r).abs() > touch {
                return None;
            }
            let u = unit(sub(foot, *c))?;
            Some(mid(foot, [c[0] + u[0] * r, c[1] + u[1] * r]))
        }
        (Prim::Circle { c: c1, r: r1 }, Prim::Circle { c: c2, r: r2 }) => {
            let u = unit(sub(*c2, *c1))?;
            let d = dist(*c1, *c2);
            let (ext, int) = ((d - (r1 + r2)).abs(), (d - (r1 - r2).abs()).abs());
            if ext.min(int) > touch {
                return None;
            }
            let at = |c: &P2, r: f64, s: f64| [c[0] + s * u[0] * r, c[1] + s * u[1] * r];
            let (p1, p2) = if ext <= int {
                (at(c1, *r1, 1.0), at(c2, *r2, -1.0))
            } else if r1 >= r2 {
                (at(c1, *r1, 1.0), at(c2, *r2, 1.0))
            } else {
                (at(c1, *r1, -1.0), at(c2, *r2, -1.0))
            };
            Some(mid(p1, p2))
        }
        _ => None,
    }
}

// ---------------------------------------------------------------------------------------------
// fitting one piece

/// A fitted piece: its curve and how well it matches the samples.
#[derive(Clone, Copy, Debug)]
struct Fit {
    prim: Prim,
    rms: f64,
    max: f64,
    /// Distance of the arc from its chord (0 for lines).
    sagitta: f64,
    /// Signed angle swept by the samples around the centre (0 for lines).
    sweep: f64,
}

fn stats(prim: &Prim, pts: &[P2]) -> (f64, f64) {
    let (mut sum, mut max) = (0.0, 0.0f64);
    for p in pts {
        let d = prim.dist(*p);
        sum += d * d;
        max = max.max(d);
    }
    ((sum / pts.len().max(1) as f64).sqrt(), max)
}

/// Total least-squares line.
fn fit_line(pts: &[P2]) -> Option<Fit> {
    let n = pts.len();
    if n < 2 {
        return None;
    }
    let nf = n as f64;
    let (mx, my) = (pts.iter().map(|p| p[0]).sum::<f64>() / nf, pts.iter().map(|p| p[1]).sum::<f64>() / nf);
    let (mut sxx, mut syy, mut sxy) = (0.0, 0.0, 0.0);
    for p in pts {
        let (u, v) = (p[0] - mx, p[1] - my);
        sxx += u * u;
        syy += v * v;
        sxy += u * v;
    }
    if !(sxx + syy > 1e-24) {
        return None;
    }
    let th = 0.5 * (2.0 * sxy).atan2(sxx - syy);
    let prim = Prim::Line { o: [mx, my], d: [th.cos(), th.sin()] };
    let (rms, max) = stats(&prim, pts);
    (rms.is_finite() && max.is_finite()).then_some(Fit { prim, rms, max, sagitta: 0.0, sweep: 0.0 })
}

/// Least-squares circle: algebraic start, then Gauss–Newton on the distances.
fn fit_circle_raw(pts: &[P2]) -> Option<(P2, f64)> {
    let n = pts.len();
    if n < 3 {
        return None;
    }
    let nf = n as f64;
    let (mx, my) = (pts.iter().map(|p| p[0]).sum::<f64>() / nf, pts.iter().map(|p| p[1]).sum::<f64>() / nf);
    let (mut suu, mut svv, mut suv, mut sz, mut szu, mut szv) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for p in pts {
        let (u, v) = (p[0] - mx, p[1] - my);
        let z = u * u + v * v;
        suu += u * u;
        svv += v * v;
        suv += u * v;
        sz += z;
        szu += z * u;
        szv += z * v;
    }
    let det = suu * svv - suv * suv;
    if !(det.abs() > 1e-14 * (suu + svv).powi(2) && det.is_finite()) {
        return None;
    }
    let a = 0.5 * (szu * svv - szv * suv) / det;
    let b = 0.5 * (szv * suu - szu * suv) / det;
    let r2 = a * a + b * b + sz / nf;
    if !(r2 > 0.0 && r2.is_finite()) {
        return None;
    }
    let (mut c, mut r) = ([mx + a, my + b], r2.sqrt());
    let cost = |c: P2, r: f64| pts.iter().map(|p| (dist(*p, c) - r).powi(2)).sum::<f64>();
    let mut best = cost(c, r);
    for _ in 0..15 {
        let mut jtj = nalgebra::Matrix3::<f64>::zeros();
        let mut jtr = nalgebra::Vector3::<f64>::zeros();
        for p in pts {
            let (dx, dy) = (p[0] - c[0], p[1] - c[1]);
            let rho = dx.hypot(dy).max(1e-12);
            let row = nalgebra::Vector3::new(-dx / rho, -dy / rho, -1.0);
            jtj += row * row.transpose();
            jtr += row * (rho - r);
        }
        let lam = 1e-9 * (jtj.trace() + 1.0);
        for k in 0..3 {
            jtj[(k, k)] += lam;
        }
        let Some(step) = jtj.lu().solve(&(-jtr)) else { break };
        let (cn, rn) = ([c[0] + step[0], c[1] + step[1]], r + step[2]);
        let cn_cost = cost(cn, rn);
        if !(rn > 0.0 && cn_cost.is_finite()) || cn_cost >= best {
            break;
        }
        let small = step.norm() < 1e-12 * r.max(1.0);
        (c, r, best) = (cn, rn, cn_cost);
        if small {
            break;
        }
    }
    (c.iter().all(|v| v.is_finite()) && r.is_finite() && r > 0.0).then_some((c, r))
}

/// Signed angle swept by consecutive samples around `c`, and the sum of the absolute steps.
fn sweep(pts: &[P2], c: P2) -> (f64, f64) {
    let (mut net, mut abs) = (0.0, 0.0);
    let mut prev: Option<f64> = None;
    for p in pts {
        let a = (p[1] - c[1]).atan2(p[0] - c[0]);
        if let Some(q) = prev {
            let mut d = a - q;
            while d > PI {
                d -= TAU;
            }
            while d <= -PI {
                d += TAU;
            }
            net += d;
            abs += d.abs();
        }
        prev = Some(a);
    }
    (net, abs)
}

/// A circle through the samples with its sweep, without judging whether it is an arc.
fn fit_arc_raw(pts: &[P2]) -> Option<Fit> {
    let (c, r) = fit_circle_raw(pts)?;
    let prim = Prim::Circle { c, r };
    let (rms, max) = stats(&prim, pts);
    let (net, _) = sweep(pts, c);
    let sagitta = r * (1.0 - (net.abs().min(PI) / 2.0).cos());
    (rms.is_finite() && max.is_finite()).then_some(Fit { prim, rms, max, sagitta, sweep: net })
}

struct Ctx {
    tol: f64,
    /// Largest radius of an arc.
    rmax: f64,
}

fn accepts(f: &Fit, tol: f64) -> bool {
    f.rms <= RMS_FACTOR * tol && f.max <= MAX_FACTOR * tol
}

/// Is the sample run an arc: at least 5 samples, one direction around the centre, at most a full turn.
fn fit_arc(pts: &[P2], ctx: &Ctx) -> Option<Fit> {
    if pts.len() < 5 {
        return None;
    }
    let f = fit_arc_raw(pts)?;
    let Prim::Circle { c, r } = f.prim else { return None };
    let (net, abs) = sweep(pts, c);
    (r <= ctx.rmax && abs > 1e-9 && net.abs() >= 0.8 * abs && net.abs() <= TAU * 1.02).then_some(f)
}

/// The primitive the samples are, if any: a line, or an arc that is clearly curved.
fn best(pts: &[P2], ctx: &Ctx) -> Option<(Fit, bool)> {
    let tol = ctx.tol;
    let line = fit_line(pts).filter(|f| accepts(f, tol));
    if let Some(l) = line
        && (pts.len() < 5 || l.max <= 0.6 * tol)
    {
        return Some((l, false));
    }
    let arc = fit_arc(pts, ctx).filter(|f| accepts(f, tol) && f.sagitta > FLAT * tol);
    match (line, arc) {
        (Some(l), Some(a)) => Some(if l.rms > 1.5 * a.rms { (a, true) } else { (l, false) }),
        (Some(l), None) => Some((l, false)),
        (None, Some(a)) => Some((a, true)),
        (None, None) => None,
    }
}

fn force_line(pts: &[P2]) -> Fit {
    fit_line(pts).unwrap_or_else(|| {
        let o = pts.first().copied().unwrap_or([0.0, 0.0]);
        Fit { prim: Prim::Line { o, d: [1.0, 0.0] }, rms: 0.0, max: 0.0, sagitta: 0.0, sweep: 0.0 }
    })
}

fn refit_kind(pts: &[P2], arc: bool) -> Option<Fit> {
    if arc { fit_arc_raw(pts) } else { fit_line(pts) }
}

// ---------------------------------------------------------------------------------------------
// segmenting one chain

/// The samples of a chain; a closed chain is cyclic and has no repeated end sample.
struct Samples {
    q: Vec<P2>,
    closed: bool,
}

impl Samples {
    fn at(&self, i: usize) -> P2 {
        let m = self.q.len();
        if m == 0 {
            return [0.0, 0.0];
        }
        let k = if self.closed { i % m } else { i.min(m - 1) };
        self.q.get(k).copied().unwrap_or([0.0, 0.0])
    }

    /// Samples `s ..= s + len`.
    fn pts(&self, s: usize, len: usize) -> Vec<P2> {
        (s..=s + len).map(|i| self.at(i)).collect()
    }

    /// The same, thinned to about `FIT_SAMPLES` (both ends kept) for quick decisions.
    fn pts_fit(&self, s: usize, len: usize) -> Vec<P2> {
        if len < FIT_SAMPLES {
            return self.pts(s, len);
        }
        let stride = len.div_ceil(FIT_SAMPLES - 1);
        let mut v: Vec<P2> = (0..len).step_by(stride).map(|k| self.at(s + k)).collect();
        v.push(self.at(s + len));
        v
    }

    fn length(&self, s: usize, len: usize) -> f64 {
        let v = self.pts(s, len);
        v.windows(2).map(|w| dist(w[0], w[1])).sum()
    }

    fn norm_start(&self, s: usize) -> usize {
        if self.closed && !self.q.is_empty() { s % self.q.len() } else { s }
    }
}

/// A run of samples `s ..= s + len` described by one primitive.
#[derive(Clone, Copy, Debug)]
struct Piece {
    s: usize,
    len: usize,
    fit: Fit,
    arc: bool,
}

/// Offset (1 ..= len − 1) of the sample farthest from the chord of `s ..= s + len`.
fn far_from_chord(sm: &Samples, s: usize, len: usize) -> usize {
    let (a, b) = (sm.at(s), sm.at(s + len));
    let d = unit(sub(b, a));
    let far = (1..len)
        .map(|k| {
            let p = sm.at(s + k);
            (k, d.map_or_else(|| dist(p, a), |d| cross(sub(p, a), d).abs()))
        })
        .max_by(|x, y| x.1.total_cmp(&y.1))
        .map_or(len / 2, |(k, _)| k);
    far.clamp(1, len.saturating_sub(1).max(1))
}

/// Splits the chain until every piece is a line or an arc within the tolerance.
fn split(sm: &Samples, ctx: &Ctx) -> Vec<Piece> {
    let m = sm.q.len();
    let mut stack: Vec<(usize, usize)> = Vec::new();
    if sm.closed {
        let far = (1..m)
            .max_by(|&a, &b| dist(sm.at(a), sm.at(0)).total_cmp(&dist(sm.at(b), sm.at(0))))
            .unwrap_or(m / 2)
            .clamp(1, m.saturating_sub(1).max(1));
        stack.push((far, m - far));
        stack.push((0, far));
    } else {
        stack.push((0, m.saturating_sub(1)));
    }
    let mut out: Vec<Piece> = Vec::new();
    while let Some((s, len)) = stack.pop() {
        let pts = sm.pts_fit(s, len);
        if let Some((fit, arc)) = best(&pts, ctx) {
            out.push(Piece { s, len, fit, arc });
            continue;
        }
        if len <= 1 || out.len() + stack.len() >= MAX_PIECES {
            out.push(Piece { s, len, fit: force_line(&pts), arc: false });
            continue;
        }
        let k = far_from_chord(sm, s, len);
        stack.push((sm.norm_start(s + k), len - k));
        stack.push((s, k));
    }
    out
}

/// Merges neighbouring pieces while the union of the best pair is still one primitive.
fn merge(sm: &Samples, ctx: &Ctx, mut pieces: Vec<Piece>) -> Vec<Piece> {
    let m = sm.q.len();
    let mut cache: HashMap<(usize, usize), Option<(Fit, bool)>> = HashMap::new();
    loop {
        let n = pieces.len();
        if n < 2 {
            break;
        }
        let pairs = match (sm.closed, n) {
            (true, 2) => 1,
            (true, n) => n,
            (false, n) => n - 1,
        };
        let mut pick: Option<(usize, usize, usize, Fit, bool)> = None;
        for i in 0..pairs {
            let j = (i + 1) % n;
            let (Some(a), Some(b)) = (pieces.get(i), pieces.get(j)) else { continue };
            let len = a.len + b.len;
            if sm.closed && len > m {
                continue;
            }
            let key = (a.s, len);
            let res = *cache.entry(key).or_insert_with(|| best(&sm.pts_fit(a.s, len), ctx));
            if let Some((fit, arc)) = res
                && pick.is_none_or(|p| fit.rms < p.3.rms)
            {
                pick = Some((i, j, len, fit, arc));
            }
        }
        let Some((i, j, len, fit, arc)) = pick else { break };
        if let Some(p) = pieces.get_mut(i) {
            *p = Piece { s: p.s, len, fit, arc };
        }
        pieces.remove(j);
    }
    pieces
}

/// Absorbs pieces shorter than four tolerances into the neighbour that fits the union better.
fn absorb_tiny(sm: &Samples, ctx: &Ctx, pieces: &mut Vec<Piece>) {
    let min_len = 4.0 * ctx.tol;
    loop {
        let n = pieces.len();
        if n < 2 {
            return;
        }
        let Some((i, l)) =
            pieces.iter().enumerate().map(|(i, p)| (i, sm.length(p.s, p.len))).min_by(|a, b| a.1.total_cmp(&b.1))
        else {
            return;
        };
        if l >= min_len {
            return;
        }
        let Some(tiny) = pieces.get(i).copied() else { return };
        let mut options: Vec<(usize, Piece, f64)> = Vec::new();
        let prev = (sm.closed || i > 0).then(|| (i + n - 1) % n);
        let next = (sm.closed || i + 1 < n).then(|| (i + 1) % n);
        for (nb, before) in [(prev, true), (next, false)] {
            let Some(nb) = nb else { continue };
            let Some(o) = pieces.get(nb).copied() else { continue };
            let (s, len) = if before { (o.s, o.len + tiny.len) } else { (tiny.s, tiny.len + o.len) };
            if sm.closed && len > sm.q.len() {
                continue;
            }
            if let Some(fit) = refit_kind(&sm.pts(s, len), o.arc) {
                let cost = fit.rms * fit.rms * (len + 1) as f64;
                options.push((nb, Piece { s: sm.norm_start(s), len, fit, arc: o.arc }, cost));
            }
        }
        let Some((nb, merged, _)) = options.into_iter().min_by(|a, b| a.2.total_cmp(&b.2)) else { return };
        if let Some(slot) = pieces.get_mut(nb) {
            *slot = merged;
        }
        pieces.remove(i);
    }
}

/// Slides every boundary between two pieces to where the two refitted primitives have the least
/// squared error.
fn refine(sm: &Samples, pieces: &mut [Piece]) {
    const W: usize = 8;
    let m = sm.q.len();
    for _ in 0..3 {
        let n = pieces.len();
        if n < 2 {
            return;
        }
        let pairs = if sm.closed { n } else { n - 1 };
        let mut changed = false;
        for i in 0..pairs {
            let j = (i + 1) % n;
            let (Some(a), Some(b)) = (pieces.get(i).copied(), pieces.get(j).copied()) else { continue };
            let total = a.len + b.len;
            if total < 4 {
                continue;
            }
            let lo = 2.max(a.len.saturating_sub(W));
            let hi = (total - 2).min(a.len + W);
            let cost = |la: usize| -> f64 {
                match (refit_kind(&sm.pts(a.s, la), a.arc), refit_kind(&sm.pts(a.s + la, total - la), b.arc)) {
                    (Some(fa), Some(fb)) => {
                        fa.rms * fa.rms * (la + 1) as f64 + fb.rms * fb.rms * (total - la + 1) as f64
                    }
                    _ => f64::INFINITY,
                }
            };
            let costs: Vec<(usize, f64)> = (lo..=hi).map(|la| (la, cost(la))).collect();
            let Some((la, c)) = costs.into_iter().min_by(|x, y| x.1.total_cmp(&y.1)) else { continue };
            if la != a.len && c.is_finite() {
                if let Some(p) = pieces.get_mut(i) {
                    p.len = la;
                }
                if let Some(p) = pieces.get_mut(j) {
                    p.s = if sm.closed { (a.s + la) % m } else { a.s + la };
                    p.len = total - la;
                }
                changed = true;
            }
        }
        if !changed {
            return;
        }
    }
}

/// Refits every piece on all its samples; a piece whose arc cannot be fitted becomes a line.
fn refit_pieces(sm: &Samples, pieces: &mut [Piece]) {
    for p in pieces.iter_mut() {
        let pts = sm.pts(p.s, p.len);
        if let Some(f) = refit_kind(&pts, p.arc) {
            p.fit = f;
        } else {
            p.arc = false;
            p.fit = force_line(&pts);
        }
    }
}

/// Removes repeated samples; `None` when the chain cannot be fitted.
fn clean(chain: &Chain2) -> Result<Samples, String> {
    if chain.points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("the chain has non-finite coordinates".into());
    }
    let mut q: Vec<P2> = Vec::with_capacity(chain.points.len());
    for p in &chain.points {
        if q.last().is_none_or(|l| dist(*l, *p) > 1e-9) {
            q.push(*p);
        }
    }
    let mut closed = chain.closed;
    if closed
        && q.len() > 1
        && let (Some(a), Some(b)) = (q.first().copied(), q.last().copied())
        && dist(a, b) <= 1e-9
    {
        q.pop();
    }
    if closed && q.len() < 4 {
        closed = false;
    }
    if q.len() > MAX_SAMPLES {
        let stride = q.len().div_ceil(MAX_SAMPLES);
        let last = q.last().copied();
        let mut t: Vec<P2> = q.iter().step_by(stride).copied().collect();
        if !closed
            && let Some(l) = last
            && t.last() != Some(&l)
        {
            t.push(l);
        }
        q = t;
    }
    if q.len() < 2 {
        return Err("the chain has fewer than two distinct samples".into());
    }
    Ok(Samples { q, closed })
}

fn extent(q: &[P2]) -> f64 {
    let (mut lo, mut hi) = ([f64::MAX; 2], [f64::MIN; 2]);
    for p in q {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    dist(lo, hi)
}

/// Splits, merges and refines one chain into pieces.
fn segment(chain: &Chain2, tol: f64) -> Result<(Samples, Vec<Piece>), String> {
    let sm = clean(chain)?;
    let ctx = Ctx { tol, rmax: 20.0 * extent(&sm.q).max(1.0) };
    if sm.length(0, sm.q.len() - 1) < 4.0 * tol {
        return Err("the chain is shorter than four tolerances".into());
    }
    let mut pieces = split(&sm, &ctx);
    for _ in 0..4 {
        let before = pieces.len();
        pieces = merge(&sm, &ctx, pieces);
        absorb_tiny(&sm, &ctx, &mut pieces);
        refine(&sm, &mut pieces);
        refit_pieces(&sm, &mut pieces);
        // boundaries moved: an arc that turned out flat is a line, and neighbours may now merge
        let mut flat = false;
        for p in pieces.iter_mut().filter(|p| p.arc && p.fit.sagitta <= FLAT * tol) {
            flat = true;
            p.arc = false;
        }
        if flat {
            refit_pieces(&sm, &mut pieces);
        }
        if !flat && pieces.len() == before {
            break;
        }
    }
    // a closed chain left as one line is a circle
    if sm.closed
        && pieces.len() == 1
        && let Some(p) = pieces.first_mut()
        && !p.arc
    {
        let all = sm.pts(0, sm.q.len());
        let f = fit_arc_raw(&all).ok_or_else(|| "the closed chain could not be fitted".to_string())?;
        *p = Piece { s: 0, len: sm.q.len(), fit: f, arc: true };
    }
    Ok((sm, pieces))
}

// ---------------------------------------------------------------------------------------------
// the sketch of one chain

/// Sketch of one chain and, per entity, the sample range `(s, len)` it stands for.
struct Built {
    sk: Sketch,
    ranges: Vec<(usize, usize)>,
}

fn build(sm: &Samples, pieces: &[Piece], plane: &SketchPlane, tol: f64) -> Option<Built> {
    let n = pieces.len();
    let m = sm.q.len();
    let mut sk = Sketch { plane: plane.clone(), ..Sketch::default() };
    let mut ranges = Vec::new();
    if sm.closed && n == 1 {
        let p = pieces.first()?;
        let Prim::Circle { c, r } = p.fit.prim else { return None };
        let center = sk.add_point(c);
        sk.entities.push(Entity::Circle { center, radius: r });
        ranges.push((p.s, m));
        return Some(Built { sk, ranges });
    }
    let limit = (10.0 * tol).max(1.0);
    let mut pts: Vec<P2> = Vec::with_capacity(n + 1);
    for k in 0..n {
        let cur = pieces.get(k)?;
        let p0 = sm.at(cur.s);
        pts.push(if k == 0 && !sm.closed {
            cur.fit.prim.project(p0)
        } else {
            let prev = &pieces.get((k + n - 1) % n)?.fit.prim;
            tangent_point(prev, &cur.fit.prim, tol)
                .filter(|t| dist(*t, p0) <= limit)
                .unwrap_or_else(|| junction(prev, &cur.fit.prim, p0, limit))
        });
    }
    if !sm.closed {
        let last = pieces.last()?;
        pts.push(last.fit.prim.project(sm.at(last.s + last.len)));
    }
    let ids: Vec<usize> = pts.iter().map(|p| sk.add_point(*p)).collect();
    for (k, p) in pieces.iter().enumerate() {
        let k1 = if sm.closed { (k + 1) % n } else { k + 1 };
        let (a, b) = (*ids.get(k)?, *ids.get(k1)?);
        let (pa, pb) = (*pts.get(k)?, *pts.get(k1)?);
        if a == b || dist(pa, pb) < 1e-9 {
            continue;
        }
        match p.fit.prim {
            Prim::Line { .. } => sk.entities.push(Entity::Line { a, b }),
            Prim::Circle { c, .. } => {
                let center = sk.add_point(c);
                let (sa, sb) = if p.fit.sweep >= 0.0 { (a, b) } else { (b, a) };
                sk.entities.push(Entity::Arc { center, a: sa, b: sb });
            }
        }
        ranges.push((p.s, p.len));
    }
    (!sk.entities.is_empty() && sk.validate().is_ok()).then_some(Built { sk, ranges })
}

fn entity_prim(sk: &Sketch, e: usize) -> Option<Prim> {
    let pt = |i: usize| sk.points.get(i).copied();
    match sk.entities.get(e)? {
        Entity::Line { a, b } => {
            let (pa, pb) = (pt(*a)?, pt(*b)?);
            Some(Prim::Line { o: pa, d: unit(sub(pb, pa))? })
        }
        Entity::Arc { center, a, b } => {
            let (c, pa, pb) = (pt(*center)?, pt(*a)?, pt(*b)?);
            Some(Prim::Circle { c, r: 0.5 * (dist(c, pa) + dist(c, pb)) })
        }
        Entity::Circle { center, radius } => Some(Prim::Circle { c: pt(*center)?, r: *radius }),
    }
}

/// Distance of every sample from the entity it was fitted to.
fn deviation(sk: &Sketch, ranges: &[(usize, usize)], sm: &Samples) -> Deviation {
    let (mut sum, mut max, mut count) = (0.0, 0.0f64, 0usize);
    for (e, (s, len)) in ranges.iter().enumerate() {
        let Some(prim) = entity_prim(sk, e) else { continue };
        for i in *s..=*s + *len {
            let d = prim.dist(sm.at(i));
            sum += d * d;
            max = max.max(d);
            count += 1;
        }
    }
    let rms = if count == 0 { 0.0 } else { (sum / count as f64).sqrt() };
    Deviation { rms: if rms.is_finite() { rms } else { f64::MAX }, max: if max.is_finite() { max } else { f64::MAX } }
}

/// Signed distances of the samples from their entities (at most about 1500 of them), the data
/// term of the final least-squares polish.
fn data_residuals(sk: &Sketch, ranges: &[(usize, usize)], sm: &Samples) -> Vec<f64> {
    let total: usize = ranges.iter().map(|r| r.1 + 1).sum();
    let stride = (total / 1500).max(1);
    let mut out = Vec::with_capacity(total / stride + ranges.len());
    for (e, (s, len)) in ranges.iter().enumerate() {
        let Some(prim) = entity_prim(sk, e) else { continue };
        for i in (0..=*len).step_by(stride) {
            out.push(prim.signed(sm.at(s + i)).0);
        }
    }
    out
}

/// Least-squares polish under the constraints; keeps the sketch when that does not fit better.
fn polished(sk: &Sketch, ranges: &[(usize, usize)], sm: &Samples) -> Option<Sketch> {
    let mut p = sk.clone();
    polish(&mut p, &|t| data_residuals(t, ranges, sm)).ok()?;
    let mut check = p.clone();
    let rep = solve(&mut check).ok()?;
    let ok = matches!(rep.status, SolveStatus::FullyDefined | SolveStatus::UnderDefined) && rep.residual < 1e-7;
    let (before, after) = (deviation(sk, ranges, sm), deviation(&p, ranges, sm));
    (ok && after.rms <= before.rms && after.rms.is_finite()).then_some(p)
}

// ---------------------------------------------------------------------------------------------
// relations

struct Cand {
    kind: RelationKind,
    entities: Vec<usize>,
    cons: Vec<Constraint>,
    /// Tried when `cons` is redundant (collinear lines that are already parallel).
    fallback: Option<Vec<Constraint>>,
}

fn a1_of(sk: &Sketch, e: usize) -> usize {
    sk.endpoints(e).map_or(0, |(a, _)| a)
}

fn shared_points(sk: &Sketch, e1: usize, e2: usize) -> Vec<usize> {
    let (Some((a1, b1)), Some((a2, b2))) = (sk.endpoints(e1), sk.endpoints(e2)) else { return Vec::new() };
    let mut v = Vec::new();
    for p in [a1, b1] {
        if (p == a2 || p == b2) && !v.contains(&p) {
            v.push(p);
        }
    }
    v
}

/// Relations the fitted geometry suggests, most important first.
fn candidates(sk: &Sketch, tol: f64, opts: &ChainOptions) -> Vec<Cand> {
    let ne = sk.entities.len();
    let prims: Vec<Option<Prim>> = (0..ne).map(|e| entity_prim(sk, e)).collect();
    let ang = opts.angle_tolerance_degrees.to_radians();
    let tan_ang = (3.0 * ang).min(0.5);
    let dir = |e: usize| match prims.get(e) {
        Some(Some(Prim::Line { d, .. })) => Some(*d),
        _ => None,
    };
    let circ = |e: usize| match prims.get(e) {
        Some(Some(Prim::Circle { c, r })) => Some((*c, *r)),
        _ => None,
    };
    let pt = |i: usize| sk.points.get(i).copied();
    let center_of = |e: usize| match sk.entities.get(e) {
        Some(Entity::Arc { center, .. } | Entity::Circle { center, .. }) => Some(*center),
        _ => None,
    };
    let (mut hv, mut tangent, mut concentric, mut collinear) = (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    let (mut equal, mut parallel, mut perpendicular) = (Vec::new(), Vec::new(), Vec::new());
    let single = |kind, entities: Vec<usize>, c: Constraint| Cand { kind, entities, cons: vec![c], fallback: None };
    for e in 0..ne {
        if let Some(d) = dir(e) {
            if d[1].abs().asin() <= ang {
                hv.push(single(RelationKind::Horizontal, vec![e], Constraint::Horizontal { line: e }));
            } else if d[0].abs().asin() <= ang {
                hv.push(single(RelationKind::Vertical, vec![e], Constraint::Vertical { line: e }));
            }
        }
    }
    for e1 in 0..ne {
        for e2 in e1 + 1..ne {
            // tangency at a shared end point
            let shared = shared_points(sk, e1, e2);
            let tangent_at = |j: P2| -> bool {
                match (dir(e1), dir(e2), circ(e1), circ(e2)) {
                    (Some(d), _, _, Some((c, _))) | (_, Some(d), Some((c, _)), _) => {
                        unit(sub(j, c)).is_some_and(|u| dot(d, u).abs().asin() <= tan_ang)
                    }
                    (_, _, Some((c1, r1)), Some((c2, r2))) => {
                        dist(c1, c2) > 4.0 * tol
                            && r1.min(r2) > tol
                            && unit(sub(j, c1))
                                .zip(unit(sub(j, c2)))
                                .is_some_and(|(u1, u2)| cross(u1, u2).abs().asin() <= tan_ang)
                    }
                    _ => false,
                }
            };
            if shared.iter().filter_map(|&i| pt(i)).any(tangent_at) {
                // the line (or, between two arcs, the first arc) goes first
                let (first, second) = if dir(e2).is_some() && dir(e1).is_none() { (e2, e1) } else { (e1, e2) };
                tangent.push(single(
                    RelationKind::Tangent,
                    vec![first, second],
                    Constraint::Tangent { line: first, entity: second },
                ));
            }
            // concentric and equal radius
            if let (Some((c1, r1)), Some((c2, r2))) = (circ(e1), circ(e2)) {
                if let (Some(i1), Some(i2)) = (center_of(e1), center_of(e2))
                    && i1 != i2
                    && dist(c1, c2) <= tol
                {
                    concentric.push(single(
                        RelationKind::Concentric,
                        vec![e1, e2],
                        Constraint::Coincident { a: i1, b: i2 },
                    ));
                }
                if (r1 - r2).abs() <= tol + 0.02 * r1.min(r2) {
                    equal.push(single(RelationKind::EqualRadius, vec![e1, e2], Constraint::Equal { a: e1, b: e2 }));
                }
            }
            // lines
            if let (Some(d1), Some(d2), Some((_, _)), Some((a2, b2))) =
                (dir(e1), dir(e2), sk.endpoints(e1), sk.endpoints(e2))
            {
                if cross(d1, d2).abs().asin() <= ang {
                    let offset = pt(a1_of(sk, e1)).zip(pt(a2)).zip(pt(b2)).map(|((p1, q1), q2)| {
                        let line1 = Prim::Line { o: p1, d: d1 };
                        line1.dist(q1).max(line1.dist(q2))
                    });
                    if offset.is_some_and(|dv| dv <= 2.0 * tol) && shared.is_empty() {
                        collinear.push(Cand {
                            kind: RelationKind::Collinear,
                            entities: vec![e1, e2],
                            cons: vec![
                                Constraint::Parallel { a: e1, b: e2 },
                                Constraint::PointOnLine { point: a2, line: e1 },
                            ],
                            fallback: Some(vec![Constraint::PointOnLine { point: a2, line: e1 }]),
                        });
                    } else {
                        parallel.push(single(
                            RelationKind::Parallel,
                            vec![e1, e2],
                            Constraint::Parallel { a: e1, b: e2 },
                        ));
                    }
                } else if dot(d1, d2).abs().asin() <= ang {
                    perpendicular.push(single(
                        RelationKind::Perpendicular,
                        vec![e1, e2],
                        Constraint::Perpendicular { a: e1, b: e2 },
                    ));
                }
            }
        }
    }
    let mut all = hv;
    all.extend(tangent);
    all.extend(concentric);
    all.extend(collinear);
    all.extend(equal);
    all.extend(parallel);
    all.extend(perpendicular);
    all
}

/// Outcome of the relations of one chain.
struct Related {
    sk: Sketch,
    kept: Vec<Relation>,
    rejected: Vec<RejectedRelation>,
    rejected_total: usize,
    status: Option<SolveStatus>,
    dof: usize,
    residual: f64,
    iterations: usize,
}

fn try_add(
    cur: &Sketch,
    cons: &[Constraint],
    ranges: &[(usize, usize)],
    sm: &Samples,
    limits: (f64, f64),
) -> Result<Sketch, String> {
    let mut trial = cur.clone();
    trial.constraints.extend(cons.iter().cloned());
    let rep = solve(&mut trial).map_err(|e| format!("invalid: {e}"))?;
    match rep.status {
        SolveStatus::OverDefined if rep.conflicting.is_empty() => {
            return Err("implied by the relations already kept (redundant)".into());
        }
        SolveStatus::OverDefined => return Err("contradicts the relations already kept".into()),
        SolveStatus::Failed => return Err("cannot be satisfied together with the relations kept".into()),
        SolveStatus::FullyDefined | SolveStatus::UnderDefined => {}
    }
    let mut d = deviation(&trial, ranges, sm);
    if d.rms > limits.0 || d.max > limits.1 {
        // the least-norm solution may be far from the data although a better one exists
        if let Some(p) = polished(&trial, ranges, sm) {
            trial = p;
            d = deviation(&trial, ranges, sm);
        }
    }
    if d.rms > limits.0 || d.max > limits.1 {
        return Err(format!(
            "moves the chain too far from the section (rms {:.3} mm, max {:.3} mm after solving)",
            d.rms, d.max
        ));
    }
    Ok(trial)
}

fn relate(built: &Built, sm: &Samples, tol: f64, opts: &ChainOptions, first_entity: usize) -> Related {
    let mut cur = built.sk.clone();
    let (mut kept, mut rejected, mut rejected_total) = (Vec::new(), Vec::new(), 0usize);
    let base = deviation(&cur, &built.ranges, sm);
    let limits = ((RMS_FACTOR * tol).max(1.25 * base.rms), (MAX_FACTOR * tol).max(1.25 * base.max));
    let global = |e: &[usize]| e.iter().map(|k| k + first_entity).collect::<Vec<_>>();
    for (idx, cand) in candidates(&built.sk, tol, opts).into_iter().enumerate() {
        let result = if idx >= MAX_CANDIDATES {
            Err("not tried: more than 300 relations were proposed for this chain".to_string())
        } else {
            match try_add(&cur, &cand.cons, &built.ranges, sm, limits) {
                Ok(s) => Ok(s),
                Err(first) => match &cand.fallback {
                    Some(fb) if first.contains("redundant") => try_add(&cur, fb, &built.ranges, sm, limits),
                    _ => Err(first),
                },
            }
        };
        match result {
            Ok(s) => {
                cur = s;
                kept.push(Relation { kind: cand.kind, entities: global(&cand.entities) });
            }
            Err(reason) => {
                rejected_total += 1;
                if rejected.len() < MAX_REJECTED_LISTED {
                    rejected.push(RejectedRelation { kind: cand.kind, entities: global(&cand.entities), reason });
                }
            }
        }
    }
    if !kept.is_empty()
        && let Some(p) = polished(&cur, &built.ranges, sm)
    {
        cur = p;
    }
    let (mut status, mut dof, mut residual, mut iterations) = (None, 0, 0.0, 0);
    let mut last = cur.clone();
    if let Ok(rep) = solve(&mut last) {
        status = Some(rep.status);
        dof = rep.dof;
        residual = rep.residual;
        iterations = rep.iterations;
        cur = last;
    }
    Related { sk: cur, kept, rejected, rejected_total, status, dof, residual, iterations }
}

// ---------------------------------------------------------------------------------------------
// the whole fit

/// Worst-first order of solve outcomes (no outcome counts as failed).
fn rank(s: Option<SolveStatus>) -> u8 {
    match s {
        Some(SolveStatus::FullyDefined) => 0,
        Some(SolveStatus::UnderDefined) => 1,
        Some(SolveStatus::Failed) | None => 2,
        Some(SolveStatus::OverDefined) => 3,
    }
}

fn rotate(p: P2, c: f64, s: f64) -> P2 {
    [c * p[0] + s * p[1], -s * p[0] + c * p[1]]
}

/// The turn of the frame (radians): length-weighted mean offset of the lines from the axes.
fn frame_angle(lines: &[(f64, f64)]) -> f64 {
    let limit = MAX_FRAME_OFFSET_DEGREES.to_radians();
    let (mut sum, mut weight) = (0.0, 0.0);
    for (phi, w) in lines {
        let off = (phi + PI / 4.0).rem_euclid(PI / 2.0) - PI / 4.0;
        if off.abs() <= limit && w.is_finite() && *w > 0.0 {
            sum += w * off;
            weight += w;
        }
    }
    if weight > 0.0 { sum / weight } else { 0.0 }
}

fn skipped(index: usize, closed: bool, samples: usize, first_entity: usize, note: &str) -> ChainReport {
    let zero = Deviation { rms: 0.0, max: 0.0 };
    ChainReport {
        chain: index,
        closed,
        samples,
        first_entity,
        entities: 0,
        lines: 0,
        arcs: 0,
        circles: 0,
        relations_kept: Vec::new(),
        relations_rejected: Vec::new(),
        rejected_total: 0,
        before: zero,
        after: zero,
        status: None,
        dof: 0,
        note: Some(note.to_string()),
    }
}

/// Fits every chain into lines and arcs of one sketch on `plane` (turned to the part's axes when
/// `opts.align_frame` is set) and constrains it with the relations that hold within the tolerance.
#[must_use]
pub fn fit_chains(plane: &SketchPlane, chains: &[Chain2], opts: &ChainOptions) -> ChainsFit {
    let tol = if opts.tolerance.is_finite() && opts.tolerance > 0.0 { opts.tolerance.min(1e9) } else { 0.05 };
    let opts = ChainOptions {
        tolerance: tol,
        angle_tolerance_degrees: if opts.angle_tolerance_degrees.is_finite() {
            opts.angle_tolerance_degrees.clamp(0.001, 20.0)
        } else {
            1.0
        },
        align_frame: opts.align_frame,
    };
    // 1. segment every chain in the input frame
    let mut prepared: Vec<(usize, Result<(Samples, Vec<Piece>), String>)> = Vec::new();
    let mut lines: Vec<(f64, f64)> = Vec::new();
    for (i, c) in chains.iter().enumerate() {
        let r = segment(c, tol);
        if let Ok((sm, pieces)) = &r {
            for p in pieces.iter().filter(|p| !p.arc) {
                if let Prim::Line { d, .. } = p.fit.prim {
                    lines.push((d[1].atan2(d[0]), sm.length(p.s, p.len)));
                }
            }
        }
        prepared.push((i, r));
    }
    // 2. turn the frame
    let theta = if opts.align_frame { frame_angle(&lines) } else { 0.0 };
    let (c, s) = (theta.cos(), theta.sin());
    let mut plane = plane.clone();
    if theta != 0.0 {
        let (x, y) = (plane.x, plane.y);
        plane.x = [0, 1, 2].map(|k| c * x[k] + s * y[k]);
        plane.y = [0, 1, 2].map(|k| -s * x[k] + c * y[k]);
    }
    // 3. build, relate, merge
    let mut sketch = Sketch { plane: plane.clone(), ..Sketch::default() };
    let mut reports: Vec<ChainReport> = Vec::new();
    let (mut worst_rank, mut dof, mut residual, mut iterations) = (0u8, 0usize, 0.0f64, 0usize);
    let mut any = false;
    for (index, r) in prepared {
        let closed = chains.get(index).is_some_and(|c| c.closed);
        let first_entity = sketch.entities.len();
        let (mut sm, mut pieces) = match r {
            Ok(v) => v,
            Err(note) => {
                reports.push(skipped(index, closed, 0, first_entity, &note));
                continue;
            }
        };
        sm.q = sm.q.iter().map(|p| rotate(*p, c, s)).collect();
        refit_pieces(&sm, &mut pieces);
        let Some(built) = build(&sm, &pieces, &plane, tol) else {
            reports.push(skipped(index, sm.closed, sm.q.len(), first_entity, "no lines or arcs could be fitted"));
            continue;
        };
        if sketch.entities.len() + built.sk.entities.len() > MAX_TOTAL_ENTITIES {
            reports.push(skipped(
                index,
                sm.closed,
                sm.q.len(),
                first_entity,
                "skipped: the sketch would exceed the entity limit",
            ));
            continue;
        }
        let before = deviation(&built.sk, &built.ranges, &sm);
        let (related, note) = if built.sk.entities.len() > MAX_RELATION_ENTITIES {
            let note = format!(
                "{} entities (more than {MAX_RELATION_ENTITIES}): fitted, but no relations were inferred",
                built.sk.entities.len()
            );
            let mut sk = built.sk.clone();
            let rep = solve(&mut sk).ok();
            (
                Related {
                    sk: built.sk.clone(),
                    kept: Vec::new(),
                    rejected: Vec::new(),
                    rejected_total: 0,
                    status: rep.as_ref().map(|r| r.status),
                    dof: rep.as_ref().map_or(0, |r| r.dof),
                    residual: rep.as_ref().map_or(0.0, |r| r.residual),
                    iterations: rep.as_ref().map_or(0, |r| r.iterations),
                },
                Some(note),
            )
        } else {
            (relate(&built, &sm, tol, &opts, first_entity), None)
        };
        let after = deviation(&related.sk, &built.ranges, &sm);
        let count = |f: fn(&Entity) -> bool| related.sk.entities.iter().filter(|e| f(e)).count();
        let (nl, na, nc) = (
            count(|e| matches!(e, Entity::Line { .. })),
            count(|e| matches!(e, Entity::Arc { .. })),
            count(|e| matches!(e, Entity::Circle { .. })),
        );
        reports.push(ChainReport {
            chain: index,
            closed: sm.closed,
            samples: sm.q.len(),
            first_entity,
            entities: related.sk.entities.len(),
            lines: nl,
            arcs: na,
            circles: nc,
            relations_kept: related.kept,
            relations_rejected: related.rejected,
            rejected_total: related.rejected_total,
            before,
            after,
            status: related.status,
            dof: related.dof,
            note,
        });
        // aggregate
        any = true;
        dof += related.dof;
        residual = residual.max(related.residual);
        iterations += related.iterations;
        worst_rank = worst_rank.max(rank(related.status));
        // append to the sketch
        let (dp, de) = (sketch.points.len(), sketch.entities.len());
        sketch.points.extend(related.sk.points.iter().copied());
        for e in &related.sk.entities {
            sketch.entities.push(match e {
                Entity::Line { a, b } => Entity::Line { a: a + dp, b: b + dp },
                Entity::Arc { center, a, b } => Entity::Arc { center: center + dp, a: a + dp, b: b + dp },
                Entity::Circle { center, radius } => Entity::Circle { center: center + dp, radius: *radius },
            });
        }
        sketch.constraints.extend(related.sk.constraints.iter().map(|k| k.shifted(dp, de)));
    }
    let status = if !any {
        SolveStatus::Failed
    } else {
        match worst_rank {
            0 => SolveStatus::FullyDefined,
            1 => SolveStatus::UnderDefined,
            2 => SolveStatus::Failed,
            _ => SolveStatus::OverDefined,
        }
    };
    let n_ok = reports.iter().filter(|r| r.entities > 0).count();
    let report = SolveReport {
        status,
        dof,
        residual,
        iterations,
        redundant: Vec::new(),
        conflicting: Vec::new(),
        message: format!("{n_ok} of {} chains fitted; {dof} degrees of freedom left", reports.len()),
    };
    ChainsFit { sketch, frame_rotation_degrees: theta.to_degrees(), chains: reports, report }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::profiles;
    use surfacecraft_geom::Rng;

    enum Seg {
        Line([f64; 2], [f64; 2]),
        /// Centre, radius, start and end angle (degrees).
        Arc([f64; 2], f64, f64, f64),
    }

    /// Samples a path of segments about `step` mm apart.
    fn sample(segs: &[Seg], step: f64) -> Vec<P2> {
        let mut out: Vec<P2> = Vec::new();
        for s in segs {
            match s {
                Seg::Line(a, b) => {
                    let n = (dist(*a, *b) / step).ceil().max(1.0) as usize;
                    for k in 0..n {
                        let f = k as f64 / n as f64;
                        out.push([a[0] + (b[0] - a[0]) * f, a[1] + (b[1] - a[1]) * f]);
                    }
                }
                Seg::Arc(c, r, a0, a1) => {
                    let n = ((a1 - a0).abs().to_radians() * r / step).ceil().max(1.0) as usize;
                    for k in 0..n {
                        let a = (a0 + (a1 - a0) * k as f64 / n as f64).to_radians();
                        out.push([c[0] + r * a.cos(), c[1] + r * a.sin()]);
                    }
                }
            }
        }
        out
    }

    /// Rotates by `deg` about the origin, moves by `t` and adds Gaussian noise.
    fn scan(pts: &[P2], deg: f64, t: P2, sigma: f64, seed: u64) -> Vec<P2> {
        let mut rng = Rng::new(seed);
        let (c, s) = (deg.to_radians().cos(), deg.to_radians().sin());
        pts.iter()
            .map(|p| {
                [c * p[0] - s * p[1] + t[0] + sigma * rng.gauss(), s * p[0] + c * p[1] + t[1] + sigma * rng.gauss()]
            })
            .collect()
    }

    fn rounded_rect(w: f64, h: f64, r: f64) -> Vec<Seg> {
        vec![
            Seg::Line([r, 0.0], [w - r, 0.0]),
            Seg::Arc([w - r, r], r, -90.0, 0.0),
            Seg::Line([w, r], [w, h - r]),
            Seg::Arc([w - r, h - r], r, 0.0, 90.0),
            Seg::Line([w - r, h], [r, h]),
            Seg::Arc([r, h - r], r, 90.0, 180.0),
            Seg::Line([0.0, h - r], [0.0, r]),
            Seg::Arc([r, r], r, 180.0, 270.0),
        ]
    }

    fn radii(s: &Sketch) -> Vec<f64> {
        s.entities
            .iter()
            .filter_map(|e| match e {
                Entity::Arc { center, a, .. } => Some(dist(s.points[*center], s.points[*a])),
                Entity::Circle { radius, .. } => Some(*radius),
                Entity::Line { .. } => None,
            })
            .collect()
    }

    fn count(r: &ChainReport, k: RelationKind) -> usize {
        r.relations_kept.iter().filter(|x| x.kind == k).count()
    }

    fn fit(pts: Vec<P2>, closed: bool) -> ChainsFit {
        fit_chains(&SketchPlane::default(), &[Chain2 { points: pts, closed }], &ChainOptions::default())
    }

    #[test]
    fn rounded_rectangle_noisy_and_rotated() {
        let pts = scan(&sample(&rounded_rect(40.0, 20.0, 5.0), 0.25), 0.7, [12.3, -4.5], 0.02, 11);
        let f = fit(pts, true);
        let r = &f.chains[0];
        assert_eq!((r.lines, r.arcs, r.circles), (4, 4, 0), "{r:?}");
        assert!((f.frame_rotation_degrees - 0.7).abs() < 0.1, "{}", f.frame_rotation_degrees);
        assert_eq!(count(r, RelationKind::Horizontal), 2, "{:?}", r.relations_kept);
        assert_eq!(count(r, RelationKind::Vertical), 2);
        assert_eq!(count(r, RelationKind::Tangent), 8, "{:#?}", r);
        assert_eq!(count(r, RelationKind::EqualRadius), 3, "{:?}", r.relations_kept);
        assert_ne!(r.status, Some(SolveStatus::OverDefined));
        assert_ne!(f.report.status, SolveStatus::OverDefined);
        for rad in radii(&f.sketch) {
            assert!((rad - 5.0).abs() < 0.05, "{rad}");
        }
        let rr = radii(&f.sketch);
        assert!(rr.iter().all(|x| (x - rr[0]).abs() < 1e-6), "{rr:?}");
        assert!(r.after.rms <= 0.05 && r.after.max <= 0.15, "{:?}", r.after);
        assert_eq!(profiles(&f.sketch, 1e-6).len(), 1);
    }

    #[test]
    fn slot_d_shape_and_open_chain() {
        // slot: two lines and two half circles
        let slot = [
            Seg::Line([-15.0, -5.0], [15.0, -5.0]),
            Seg::Arc([15.0, 0.0], 5.0, -90.0, 90.0),
            Seg::Line([15.0, 5.0], [-15.0, 5.0]),
            Seg::Arc([-15.0, 0.0], 5.0, 90.0, 270.0),
        ];
        let f = fit(scan(&sample(&slot, 0.25), 0.7, [3.0, 2.0], 0.02, 5), true);
        let r = &f.chains[0];
        assert_eq!((r.lines, r.arcs), (2, 2), "{r:?}");
        assert_eq!(count(r, RelationKind::Tangent), 4, "{:#?}", r);
        assert_ne!(r.status, Some(SolveStatus::OverDefined));
        for rad in radii(&f.sketch) {
            assert!((rad - 5.0).abs() < 0.05, "{rad}");
        }
        // D: an arc of 253.7 degrees and its chord
        let a0 = 90.0 + 53.13;
        let d = [
            Seg::Arc([0.0, 0.0], 10.0, a0, a0 + 253.74),
            Seg::Line(
                [10.0 * (a0 + 253.74).to_radians().cos(), 10.0 * (a0 + 253.74).to_radians().sin()],
                [10.0 * a0.to_radians().cos(), 10.0 * a0.to_radians().sin()],
            ),
        ];
        let f = fit(scan(&sample(&d, 0.25), 0.7, [0.0, 0.0], 0.02, 6), true);
        let r = &f.chains[0];
        assert_eq!((r.lines, r.arcs), (1, 1), "{r:?}");
        assert_eq!(count(r, RelationKind::Vertical) + count(r, RelationKind::Horizontal), 1);
        assert!((radii(&f.sketch)[0] - 10.0).abs() < 0.05);
        assert_ne!(r.status, Some(SolveStatus::OverDefined));
        // an open chain: line, quarter arc, line
        let l = [
            Seg::Line([0.0, 0.0], [20.0, 0.0]),
            Seg::Arc([20.0, 5.0], 5.0, -90.0, 0.0),
            Seg::Line([25.0, 5.0], [25.0, 25.0]),
        ];
        let f = fit(scan(&sample(&l, 0.25), 0.7, [0.0, 0.0], 0.02, 7), false);
        let r = &f.chains[0];
        assert!(!r.closed);
        assert_eq!((r.lines, r.arcs), (2, 1), "{r:?}");
        assert_eq!(count(r, RelationKind::Tangent), 2);
        assert!(profiles(&f.sketch, 1e-6).is_empty(), "an open chain stays open");
        assert_eq!(f.sketch.points.len(), 4 + 1, "two lines + arc share their junctions, one centre");
    }

    /// The four gate shapes over many noise seeds, rotations and offsets: structure, radii, relations.
    #[test]
    fn shapes_are_recovered_for_every_seed() {
        let slot = [
            Seg::Line([-15.0, -5.0], [15.0, -5.0]),
            Seg::Arc([15.0, 0.0], 5.0, -90.0, 90.0),
            Seg::Line([15.0, 5.0], [-15.0, 5.0]),
            Seg::Arc([-15.0, 0.0], 5.0, 90.0, 270.0),
        ];
        let rect = rounded_rect(40.0, 20.0, 5.0);
        let (mut worst_dr, mut worst_after) = (0.0f64, 0.0f64);
        for seed in 0..40u64 {
            for (deg, t) in [(0.7, [12.3, -4.5]), (-0.9, [0.0, 0.0]), (90.6, [-30.0, 8.0]), (0.0, [1.0, 1.0])] {
                let f = fit(scan(&sample(&rect, 0.25), deg, t, 0.02, seed), true);
                let r = &f.chains[0];
                assert_eq!((r.lines, r.arcs), (4, 4), "seed {seed} {deg}: {r:#?}");
                assert_eq!(
                    count(r, RelationKind::Horizontal) + count(r, RelationKind::Vertical),
                    4,
                    "seed {seed} {deg}"
                );
                assert_eq!(count(r, RelationKind::Tangent), 8, "seed {seed} {deg}: {r:#?}");
                assert_eq!(count(r, RelationKind::EqualRadius), 3, "seed {seed} {deg}");
                assert_ne!(r.status, Some(SolveStatus::OverDefined));
                for rad in radii(&f.sketch) {
                    worst_dr = worst_dr.max((rad - 5.0).abs());
                }
                worst_after = worst_after.max(r.after.max);
                let f = fit(scan(&sample(&slot, 0.25), deg, t, 0.02, seed), true);
                let r = &f.chains[0];
                assert_eq!((r.lines, r.arcs), (2, 2), "seed {seed} {deg}: {r:#?}");
                assert_eq!(count(r, RelationKind::Tangent), 4, "seed {seed} {deg}: {r:#?}");
                assert_ne!(r.status, Some(SolveStatus::OverDefined));
                for rad in radii(&f.sketch) {
                    worst_dr = worst_dr.max((rad - 5.0).abs());
                }
            }
        }
        eprintln!("worst radius error {worst_dr:.4} mm, worst max deviation after {worst_after:.4} mm");
        assert!(worst_dr < 0.05, "{worst_dr}");
    }

    #[test]
    fn long_wavy_chain_is_fitted_but_not_related() {
        let pts: Vec<P2> = (0..3000)
            .map(|k| {
                let x = f64::from(k) * 0.02;
                [x, 5.0 * x.sin()]
            })
            .collect();
        let t = std::time::Instant::now();
        let f = fit(pts, false);
        let r = &f.chains[0];
        assert!(r.entities > MAX_RELATION_ENTITIES, "{}", r.entities);
        assert!(r.relations_kept.is_empty() && r.note.is_some());
        assert!(r.before.max <= 3.0 * 0.05 + 1e-9, "{:?}", r.before);
        assert!(t.elapsed().as_secs() < 20, "{:?}", t.elapsed());
    }

    #[test]
    fn two_chains_share_one_sketch_with_shifted_indices() {
        let a = scan(&sample(&rounded_rect(40.0, 20.0, 5.0), 0.25), 0.0, [0.0, 0.0], 0.01, 1);
        let hole: Vec<P2> = (0..120)
            .map(|k| {
                let t = f64::from(k) / 120.0 * TAU;
                [20.0 + 4.0 * t.cos(), 10.0 + 4.0 * t.sin()]
            })
            .collect();
        let f = fit_chains(
            &SketchPlane::default(),
            &[Chain2 { points: a, closed: true }, Chain2 { points: hole, closed: true }],
            &ChainOptions::default(),
        );
        assert_eq!(f.chains.len(), 2);
        assert_eq!((f.chains[0].entities, f.chains[1].entities), (8, 1));
        assert_eq!(f.chains[1].circles, 1);
        assert_eq!(f.chains[1].first_entity, 8);
        assert!(f.sketch.validate().is_ok());
        assert_eq!(profiles(&f.sketch, 1e-6).len(), 2);
        assert_ne!(f.report.status, SolveStatus::OverDefined);
        // the sketch as a whole solves without conflicts
        let mut sk = f.sketch.clone();
        let rep = solve(&mut sk).unwrap();
        assert_ne!(rep.status, SolveStatus::OverDefined, "{rep:?}");
    }

    #[test]
    fn random_chains_finish_quickly() {
        let mut rng = Rng::new(3);
        let noise: Vec<P2> = (0..20_000).map(|_| [rng.range(-50.0, 50.0), rng.range(-50.0, 50.0)]).collect();
        let walk: Vec<P2> = noise
            .iter()
            .scan([0.0, 0.0], |p, d| {
                *p = [p[0] + d[0] * 0.01, p[1] + d[1] * 0.01];
                Some(*p)
            })
            .collect();
        let t = std::time::Instant::now();
        for (pts, closed) in [(noise, true), (walk.clone(), false), (walk, true)] {
            let f = fit(pts, closed);
            assert_eq!(f.chains.len(), 1);
            assert!(f.chains[0].samples <= MAX_SAMPLES);
        }
        assert!(t.elapsed().as_secs() < 60, "{:?}", t.elapsed());
        eprintln!("random chains: {:?}", t.elapsed());
    }

    #[test]
    fn hostile_chains_do_not_panic() {
        let plane = SketchPlane::default();
        let o = ChainOptions::default();
        let weird = vec![
            Chain2 { points: vec![], closed: true },
            Chain2 { points: vec![[0.0, 0.0]], closed: false },
            Chain2 { points: vec![[0.0, 0.0]; 50], closed: true },
            Chain2 { points: vec![[f64::NAN, 0.0], [1.0, 1.0], [2.0, 0.0]], closed: false },
            Chain2 { points: vec![[0.0, 0.0], [1e300, 1e300], [-1e300, 5.0], [3.0, 3.0], [4.0, 1e-300]], closed: true },
            Chain2 { points: (0..300).map(|k| [f64::from(k % 7), f64::from(k % 5)]).collect(), closed: true },
            Chain2 { points: (0..100).map(|k| [f64::from(k), 0.0]).collect(), closed: true },
        ];
        let f = fit_chains(&plane, &weird, &o);
        assert_eq!(f.chains.len(), weird.len());
        for t in [f64::NAN, -1.0, 0.0, f64::INFINITY, 1e300] {
            let o = ChainOptions { tolerance: t, angle_tolerance_degrees: t, align_frame: true };
            let _ = fit_chains(&plane, &weird, &o);
        }
    }
}
