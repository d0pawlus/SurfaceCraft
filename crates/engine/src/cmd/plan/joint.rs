//! Where two elements of a profile meet, judged by the data near the meeting point (the ends of a
//! region are not to be trusted: hole filling leaves stray points along a wall's line).
use super::NEIGHBOUR_GAP;
use super::profile::{Corner, Wall, d2, solve, tangent};
use std::f64::consts::TAU;

/// How far a line may miss being tangent to an arc (or an arc another arc) and still be taken for
/// tangent (mm).
const TANGENT_TOL: f64 = 0.5;
/// Data this near a meeting point (mm) says on which side of it an element lies.
const EVIDENCE_RADIUS: f64 = 2.0 * NEIGHBOUR_GAP;
/// Fewest data points near a meeting point for an element to be judged there.
const MIN_EVIDENCE: usize = 3;

/// A wall or an arc, with the data it was found from.
#[derive(Clone, Copy)]
pub(super) enum Part<'a> {
    Wall(&'a Wall),
    Arc(&'a Corner),
}

impl Part<'_> {
    fn pts(&self) -> &[[f64; 2]] {
        match self {
            Part::Wall(w) => &w.pts,
            Part::Arc(k) => &k.pts,
        }
    }
}

/// The meeting point of two elements and how well the data supports it (the sum of the distances
/// of both elements' nearest points to it: smaller is better).
pub(super) struct Joint {
    pub(super) vertex: [f64; 2],
    pub(super) score: f64,
}

fn nearest(pts: &[[f64; 2]], v: [f64; 2]) -> f64 {
    pts.iter().map(|q| d2(*q, v)).fold(f64::INFINITY, f64::min)
}

fn wrap(x: f64) -> f64 {
    x - TAU * (x / TAU).round()
}

fn angle_of(p: [f64; 2], c: [f64; 2]) -> f64 {
    (p[1] - c[1]).atan2(p[0] - c[0])
}

/// The tangent point of a line and a circle when they touch (within `TANGENT_TOL`), else the two
/// points where they cross.
fn line_circle(w: &Wall, k: &Corner) -> Vec<[f64; 2]> {
    let dd = w.n[0] * k.center[0] + w.n[1] * k.center[1] - w.c;
    let foot = [k.center[0] - w.n[0] * dd, k.center[1] - w.n[1] * dd];
    if dd.abs() > k.r || (k.r - dd.abs()) <= TANGENT_TOL {
        return if dd.abs() - k.r <= TANGENT_TOL { vec![foot] } else { Vec::new() };
    }
    let (h, t) = ((k.r * k.r - dd * dd).sqrt(), tangent(w.n));
    vec![[foot[0] + t[0] * h, foot[1] + t[1] * h], [foot[0] - t[0] * h, foot[1] - t[1] * h]]
}

/// The tangent point of two circles when they touch, else the two points where they cross.
fn circle_circle(a: &Corner, b: &Corner) -> Vec<[f64; 2]> {
    let d = d2(a.center, b.center);
    if d < 1e-9 {
        return Vec::new();
    }
    let u = [(b.center[0] - a.center[0]) / d, (b.center[1] - a.center[1]) / d];
    let at = |s: f64| [a.center[0] + u[0] * s, a.center[1] + u[1] * s];
    if (d - (a.r + b.r)).abs() <= TANGENT_TOL {
        return vec![at(a.r)];
    }
    if (d - (a.r - b.r).abs()).abs() <= TANGENT_TOL {
        return vec![at(if a.r >= b.r { a.r } else { -a.r })];
    }
    let s = (d * d + a.r * a.r - b.r * b.r) / (2.0 * d);
    let h2 = a.r * a.r - s * s;
    if h2 < 0.0 {
        return Vec::new();
    }
    let (m, h, perp) = (at(s), h2.sqrt(), [-u[1], u[0]]);
    vec![[m[0] + perp[0] * h, m[1] + perp[1] * h], [m[0] - perp[0] * h, m[1] - perp[1] * h]]
}

fn candidates(a: Part, b: Part) -> Vec<[f64; 2]> {
    match (a, b) {
        (Part::Wall(x), Part::Wall(y)) => solve(x.n, x.c, y.n, y.c).into_iter().collect(),
        (Part::Wall(w), Part::Arc(k)) | (Part::Arc(k), Part::Wall(w)) => line_circle(w, k),
        (Part::Arc(x), Part::Arc(y)) => circle_circle(x, y),
    }
}

/// How far the data point `q` lies ahead of the point `v` in the element's direction of travel
/// (a length along a wall, an angle about an arc).
fn progress(p: Part, q: [f64; 2], v: [f64; 2]) -> f64 {
    match p {
        Part::Wall(w) => {
            let t = tangent(w.n);
            (q[0] - v[0]) * t[0] + (q[1] - v[1]) * t[1]
        }
        Part::Arc(k) => {
            let d = wrap(angle_of(q, k.center) - angle_of(v, k.center));
            if k.concave { d } else { -d }
        }
    }
}

/// Does the element's data near `v` lie before it (the element leads into `v`) or after it (the
/// element leads out of `v`)?
fn lies_on_side(p: Part, v: [f64; 2], before: bool) -> bool {
    let near: Vec<f64> = p.pts().iter().filter(|q| d2(**q, v) <= EVIDENCE_RADIUS).map(|q| progress(p, *q, v)).collect();
    if near.len() < MIN_EVIDENCE {
        return false;
    }
    let mean = near.iter().sum::<f64>() / near.len() as f64;
    if before { mean < 0.0 } else { mean > 0.0 }
}

/// Do the bounding boxes of two point sets come within `gap` of each other?
fn boxes_near(a: &[[f64; 2]], b: &[[f64; 2]], gap: f64) -> bool {
    let bounds = |pts: &[[f64; 2]]| {
        pts.iter().fold(([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]), |(lo, hi), p| {
            ([lo[0].min(p[0]), lo[1].min(p[1])], [hi[0].max(p[0]), hi[1].max(p[1])])
        })
    };
    let ((alo, ahi), (blo, bhi)) = (bounds(a), bounds(b));
    (0..2).all(|k| alo[k] - gap <= bhi[k] && blo[k] - gap <= ahi[k])
}

/// Where `a` leads into `b`, if it does: the meeting point lies by data of both, the data of `a`
/// before it and the data of `b` after it.
pub(super) fn join(a: Part, b: Part) -> Option<Joint> {
    if !boxes_near(a.pts(), b.pts(), NEIGHBOUR_GAP) {
        return None;
    }
    candidates(a, b)
        .into_iter()
        .filter_map(|v| {
            let (ga, gb) = (nearest(a.pts(), v), nearest(b.pts(), v));
            let near = ga <= NEIGHBOUR_GAP && gb <= NEIGHBOUR_GAP;
            (near && lies_on_side(a, v, true) && lies_on_side(b, v, false))
                .then_some(Joint { vertex: v, score: ga + gb })
        })
        .min_by(|x, y| x.score.total_cmp(&y.score))
}
