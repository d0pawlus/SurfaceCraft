//! Snapping and hit-testing in sketch (plane) coordinates: pure maths, no egui.
//!
//! Snap candidates, strongest first: existing points (end points and centres), midpoints of
//! lines, horizontal/vertical inference from the last placed point, the grid. The tolerance is
//! given in sketch units (the caller converts its pixel radius with the view scale).
use std::f64::consts::TAU;
use surfacecraft_sketch::{Entity, Sketch};

/// What a snapped position sits on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SnapKind {
    /// An end point of a line or arc, or a free point.
    Endpoint,
    /// The centre of an arc or circle.
    Centre,
    /// The middle of a line.
    Midpoint,
    /// Level with the last placed point.
    Horizontal,
    /// Straight above or below the last placed point.
    Vertical,
    /// A grid node.
    Grid,
    /// Nothing: the cursor position as is.
    Free,
}

impl SnapKind {
    /// The catalogue key of the snap's name.
    #[must_use]
    pub fn label_key(self) -> &'static str {
        match self {
            SnapKind::Endpoint => "sketch.snap.endpoint",
            SnapKind::Centre => "sketch.snap.centre",
            SnapKind::Midpoint => "sketch.snap.midpoint",
            SnapKind::Horizontal => "sketch.snap.horizontal",
            SnapKind::Vertical => "sketch.snap.vertical",
            SnapKind::Grid => "sketch.snap.grid",
            SnapKind::Free => "sketch.snap.free",
        }
    }
}

/// A snapped cursor position.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snap {
    /// Position in sketch coordinates.
    pub pos: [f64; 2],
    /// What it snapped to.
    pub kind: SnapKind,
}

impl Snap {
    /// A position that snapped to nothing.
    #[must_use]
    pub fn free(pos: [f64; 2]) -> Self {
        Self { pos, kind: SnapKind::Free }
    }
}

/// Which snaps are on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SnapConfig {
    /// End points and centres.
    pub points: bool,
    /// Midpoints of lines.
    pub midpoints: bool,
    /// Horizontal/vertical inference from the last placed point.
    pub inference: bool,
    /// Grid nodes.
    pub grid: bool,
}

impl Default for SnapConfig {
    fn default() -> Self {
        Self { points: true, midpoints: true, inference: true, grid: true }
    }
}

/// Distance between two sketch points.
#[must_use]
pub fn dist(a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}

/// The grid step for a view scale: the smallest 1–2–5 step that keeps nodes at least
/// `min_px` pixels apart.
#[must_use]
pub fn grid_step(mm_per_px: f64, min_px: f64) -> f64 {
    let want = (mm_per_px * min_px).max(1e-9);
    let mut mag = 10f64.powf(want.log10().floor());
    for _ in 0..4 {
        for m in [1.0, 2.0, 5.0] {
            if mag * m >= want {
                return mag * m;
            }
        }
        mag *= 10.0;
    }
    mag
}

fn round_to(v: f64, step: f64) -> f64 {
    if step > 0.0 { (v / step).round() * step } else { v }
}

/// Points that are the centre of an arc or circle and nothing else.
fn centre_points(sk: &Sketch) -> Vec<bool> {
    let mut centre = vec![false; sk.points.len()];
    let mut other = vec![false; sk.points.len()];
    let mark = |v: &mut Vec<bool>, i: usize| {
        if let Some(s) = v.get_mut(i) {
            *s = true;
        }
    };
    for e in &sk.entities {
        match e {
            Entity::Line { a, b } => {
                mark(&mut other, *a);
                mark(&mut other, *b);
            }
            Entity::Arc { center, a, b } => {
                mark(&mut centre, *center);
                mark(&mut other, *a);
                mark(&mut other, *b);
            }
            Entity::Circle { center, .. } => mark(&mut centre, *center),
        }
    }
    centre.iter().zip(&other).map(|(c, o)| *c && !*o).collect()
}

/// Snaps `p`. `from` is the last placed point of the gesture (for inference), `extra` points
/// that are not in the sketch yet (the gesture's own points), `grid` the grid step.
#[must_use]
pub fn snap_point(
    sk: &Sketch,
    p: [f64; 2],
    from: Option<[f64; 2]>,
    extra: &[[f64; 2]],
    tol: f64,
    grid: f64,
    cfg: &SnapConfig,
) -> Snap {
    if cfg.points {
        let centres = centre_points(sk);
        let mut best: Option<(f64, Snap)> = None;
        let mut offer = |d: f64, s: Snap| {
            if d <= tol && best.as_ref().is_none_or(|(bd, _)| d < *bd) {
                best = Some((d, s));
            }
        };
        for (i, q) in sk.points.iter().enumerate() {
            let kind = if centres.get(i).copied().unwrap_or(false) { SnapKind::Centre } else { SnapKind::Endpoint };
            offer(dist(p, *q), Snap { pos: *q, kind });
        }
        for q in extra {
            offer(dist(p, *q), Snap { pos: *q, kind: SnapKind::Endpoint });
        }
        if let Some((_, s)) = best {
            return s;
        }
    }
    if cfg.midpoints {
        let mut best: Option<(f64, [f64; 2])> = None;
        for e in &sk.entities {
            if let Entity::Line { a, b } = e
                && let (Some(pa), Some(pb)) = (sk.points.get(*a), sk.points.get(*b))
            {
                let m = [(pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0];
                let d = dist(p, m);
                if d <= tol && best.as_ref().is_none_or(|(bd, _)| d < *bd) {
                    best = Some((d, m));
                }
            }
        }
        if let Some((_, m)) = best {
            return Snap { pos: m, kind: SnapKind::Midpoint };
        }
    }
    let gx = if cfg.grid { round_to(p[0], grid) } else { p[0] };
    let gy = if cfg.grid { round_to(p[1], grid) } else { p[1] };
    if cfg.inference
        && let Some(f) = from
    {
        let (dx, dy) = ((p[0] - f[0]).abs(), (p[1] - f[1]).abs());
        if dy <= tol && dy <= dx {
            return Snap { pos: [gx, f[1]], kind: SnapKind::Horizontal };
        }
        if dx <= tol {
            return Snap { pos: [f[0], gy], kind: SnapKind::Vertical };
        }
    }
    if cfg.grid {
        return Snap { pos: [gx, gy], kind: SnapKind::Grid };
    }
    Snap::free(p)
}

/// Distance from `p` to the segment `a`–`b`.
#[must_use]
pub fn dist_to_segment(p: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
    let l2 = dx * dx + dy * dy;
    if l2 < 1e-24 {
        return dist(p, a);
    }
    let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2).clamp(0.0, 1.0);
    dist(p, [a[0] + t * dx, a[1] + t * dy])
}

/// An arc as centre, radius, start angle and counter-clockwise sweep (a full turn when its ends
/// coincide).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArcGeom {
    /// Centre.
    pub c: [f64; 2],
    /// Radius (distance to the start point).
    pub r: f64,
    /// Start angle, radians.
    pub a0: f64,
    /// Counter-clockwise sweep, radians in (0, 2π].
    pub sweep: f64,
}

/// The geometry of arc entity `e` (`None` for another entity or a bad index).
#[must_use]
pub fn arc_geom(sk: &Sketch, e: usize) -> Option<ArcGeom> {
    let Entity::Arc { center, a, b } = sk.entities.get(e)? else { return None };
    let (c, pa, pb) = (*sk.points.get(*center)?, *sk.points.get(*a)?, *sk.points.get(*b)?);
    let a0 = (pa[1] - c[1]).atan2(pa[0] - c[0]);
    let a1 = (pb[1] - c[1]).atan2(pb[0] - c[0]);
    let mut sweep = (a1 - a0).rem_euclid(TAU);
    if sweep < 1e-9 {
        sweep = TAU;
    }
    Some(ArcGeom { c, r: dist(pa, c), a0, sweep })
}

/// Centre and radius of arc or circle entity `e`.
#[must_use]
pub fn circle_geom(sk: &Sketch, e: usize) -> Option<([f64; 2], f64)> {
    match sk.entities.get(e)? {
        Entity::Circle { center, radius } => Some((*sk.points.get(*center)?, *radius)),
        Entity::Arc { .. } => arc_geom(sk, e).map(|g| (g.c, g.r)),
        Entity::Line { .. } => None,
    }
}

/// Points along entity `e` for drawing (a line: its two ends; an arc or circle: a polyline).
#[must_use]
pub fn entity_polyline(sk: &Sketch, e: usize, segments: usize) -> Vec<[f64; 2]> {
    let n = segments.max(4);
    match sk.entities.get(e) {
        Some(Entity::Line { a, b }) => [a, b].iter().filter_map(|i| sk.points.get(**i).copied()).collect(),
        Some(Entity::Circle { .. }) => circle_geom(sk, e).map_or_else(Vec::new, |(c, r)| sample_arc(c, r, 0.0, TAU, n)),
        Some(Entity::Arc { .. }) => arc_geom(sk, e).map_or_else(Vec::new, |g| sample_arc(g.c, g.r, g.a0, g.sweep, n)),
        None => Vec::new(),
    }
}

/// `n + 1` points along an arc.
#[must_use]
pub fn sample_arc(c: [f64; 2], r: f64, a0: f64, sweep: f64, n: usize) -> Vec<[f64; 2]> {
    (0..=n)
        .map(|k| {
            let a = a0 + sweep * (k as f64) / (n as f64);
            [c[0] + r * a.cos(), c[1] + r * a.sin()]
        })
        .collect()
}

/// Distance from `p` to entity `e` (infinite when the entity does not exist).
#[must_use]
pub fn entity_distance(sk: &Sketch, e: usize, p: [f64; 2]) -> f64 {
    match sk.entities.get(e) {
        Some(Entity::Line { a, b }) => match (sk.points.get(*a), sk.points.get(*b)) {
            (Some(pa), Some(pb)) => dist_to_segment(p, *pa, *pb),
            _ => f64::INFINITY,
        },
        Some(Entity::Circle { .. }) => circle_geom(sk, e).map_or(f64::INFINITY, |(c, r)| (dist(p, c) - r).abs()),
        Some(Entity::Arc { a, b, .. }) => {
            let (Some(g), Some(pa), Some(pb)) = (arc_geom(sk, e), sk.points.get(*a), sk.points.get(*b)) else {
                return f64::INFINITY;
            };
            let ang = (p[1] - g.c[1]).atan2(p[0] - g.c[0]);
            if (ang - g.a0).rem_euclid(TAU) <= g.sweep {
                (dist(p, g.c) - g.r).abs()
            } else {
                dist(p, *pa).min(dist(p, *pb))
            }
        }
        None => f64::INFINITY,
    }
}

/// What a click can pick.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Hit {
    /// A sketch point (index).
    Point(usize),
    /// A sketch entity (index).
    Entity(usize),
}

/// What a tool is looking for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Want {
    /// A line.
    Line,
    /// An arc or circle.
    Circular,
    /// A line, an arc or a circle.
    LineOrCircular,
    /// A point.
    Point,
    /// A point first, else any entity.
    PointOrEntity,
}

/// The thing of kind `want` nearest to `p` within `tol`.
#[must_use]
pub fn pick(sk: &Sketch, p: [f64; 2], tol: f64, want: Want) -> Option<Hit> {
    let nearest = |it: &mut dyn Iterator<Item = (f64, usize)>| {
        it.filter(|(d, _)| *d <= tol).min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
    };
    if matches!(want, Want::Point | Want::PointOrEntity) {
        let mut it = sk.points.iter().enumerate().map(|(i, q)| (dist(p, *q), i));
        if let Some((_, i)) = nearest(&mut it) {
            return Some(Hit::Point(i));
        }
        if want == Want::Point {
            return None;
        }
    }
    let mut it = sk
        .entities
        .iter()
        .enumerate()
        .filter(|(_, e)| match want {
            Want::Line => matches!(e, Entity::Line { .. }),
            Want::Circular => matches!(e, Entity::Arc { .. } | Entity::Circle { .. }),
            _ => true,
        })
        .map(|(i, _)| (entity_distance(sk, i, p), i));
    nearest(&mut it).map(|(_, i)| Hit::Entity(i))
}

/// The circle through three points: its centre (`None` when they are collinear).
#[must_use]
pub fn circumcentre(a: [f64; 2], b: [f64; 2], c: [f64; 2]) -> Option<[f64; 2]> {
    let d = 2.0 * (a[0] * (b[1] - c[1]) + b[0] * (c[1] - a[1]) + c[0] * (a[1] - b[1]));
    let scale = dist(a, b).max(dist(b, c)).max(dist(a, c));
    if d.abs() < 1e-12 * scale * scale || !d.is_finite() {
        return None;
    }
    let (a2, b2, c2) = (a[0] * a[0] + a[1] * a[1], b[0] * b[0] + b[1] * b[1], c[0] * c[0] + c[1] * c[1]);
    let ux = (a2 * (b[1] - c[1]) + b2 * (c[1] - a[1]) + c2 * (a[1] - b[1])) / d;
    let uy = (a2 * (c[0] - b[0]) + b2 * (a[0] - c[0]) + c2 * (b[0] - a[0])) / d;
    Some([ux, uy])
}

/// Counter-clockwise angle from direction `from` to `to` about `c`, radians in [0, 2π).
#[must_use]
pub fn ccw_angle(c: [f64; 2], from: [f64; 2], to: [f64; 2]) -> f64 {
    let a = (from[1] - c[1]).atan2(from[0] - c[0]);
    let b = (to[1] - c[1]).atan2(to[0] - c[0]);
    (b - a).rem_euclid(TAU)
}

/// A counter-clockwise arc as (centre, start, end) through `start`, `mid`, `end`: the arc that
/// passes through `mid` (the ends swap when that is the clockwise way round).
#[must_use]
pub fn arc_through(start: [f64; 2], mid: [f64; 2], end: [f64; 2]) -> Option<([f64; 2], [f64; 2], [f64; 2])> {
    let c = circumcentre(start, mid, end)?;
    if ccw_angle(c, start, mid) <= ccw_angle(c, start, end) { Some((c, start, end)) } else { Some((c, end, start)) }
}

/// A counter-clockwise arc from `start` about `c` to the direction of `end`, the end put on the
/// start's radius.
#[must_use]
pub fn arc_from_centre(c: [f64; 2], start: [f64; 2], end: [f64; 2]) -> Option<([f64; 2], [f64; 2], [f64; 2])> {
    let r = dist(c, start);
    let d = dist(c, end);
    if r < 1e-9 || d < 1e-9 {
        return None;
    }
    let e = [c[0] + (end[0] - c[0]) * r / d, c[1] + (end[1] - c[1]) * r / d];
    if dist(e, start) < 1e-9 { None } else { Some((c, start, e)) }
}
