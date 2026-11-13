//! The NURBS form of a patch of a cylinder, cone or sphere: a rational quadratic arc in the angle
//! about the axis times the profile (a line, or a rational quadratic arc for a sphere's meridian).
//! The record needs this form for every analytic label of this kind; it is exact on the surface.
use crate::hybrid::{V, add, scale};
use crate::record::brep::Surface;
use std::f64::consts::FRAC_PI_2;

/// Rational quadratic arcs over an angle range.
pub(super) struct Arcs {
    knots: Vec<f64>,
    points: Vec<[f64; 2]>,
    weights: Vec<f64>,
    start: f64,
    span: f64,
    count: usize,
    scale: f64,
}

impl Arcs {
    /// Arcs of the unit circle from `a0` to `a1` (each at most a quarter turn); the knots are
    /// the arc counts times `scale` (the kernel's own forms: a cylinder's is its radius, a
    /// sphere's 1/4 for a full turn and 1/2 for the meridian).
    pub(super) fn new(a0: f64, a1: f64, scale: f64) -> Self {
        let count = (((a1 - a0) / FRAC_PI_2 - 1e-9).ceil() as usize).max(1);
        let span = (a1 - a0) / count as f64;
        let half = 0.5 * span;
        let at = |t: f64, r: f64| [r * t.cos(), r * t.sin()];
        let mut points = vec![at(a0, 1.0)];
        let mut weights = vec![1.0];
        for k in 0..count {
            let t = a0 + span * k as f64;
            points.push(at(t + half, 1.0 / half.cos()));
            weights.push(half.cos());
            points.push(at(t + span, 1.0));
            weights.push(1.0);
        }
        let mut knots = vec![0.0; 3];
        for k in 1..count {
            knots.extend([k as f64 * scale; 2]);
        }
        knots.extend([count as f64 * scale; 3]);
        Self { knots, points, weights, start: a0, span, count, scale }
    }

    /// The parameter of the point at angle `angle` (clamped into the range).
    pub(super) fn param(&self, angle: f64) -> f64 {
        let rel = ((angle - self.start) / self.span).clamp(0.0, self.count as f64);
        let k = rel.floor().min(self.count as f64 - 1.0);
        let (half, target) = (0.5 * self.span, (rel - k) * self.span - 0.5 * self.span);
        let psi = |t: f64| {
            let y = half.sin() * (2.0 * t - 1.0);
            let x = half.cos() * (1.0 - 2.0 * t + 2.0 * t * t) + 2.0 * t * (1.0 - t);
            y.atan2(x)
        };
        let (mut lo, mut hi) = (0.0_f64, 1.0_f64);
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if psi(mid) < target { lo = mid } else { hi = mid }
        }
        (k + 0.5 * (lo + hi)) * self.scale
    }
}

/// How the profile runs along v.
pub(super) enum Profile {
    /// A line: control points `(radius, height)` at the two ends of `[v0, v1]`; v is the parameter.
    Line { v: [f64; 2], radius: [f64; 2] },
    /// The full meridian of a sphere of `radius`, pole to pole.
    Meridian { radius: f64, arcs: Arcs },
}

impl Profile {
    pub(super) fn meridian(radius: f64) -> Self {
        Self::Meridian { radius, arcs: Arcs::new(-FRAC_PI_2, FRAC_PI_2, 0.5) }
    }

    /// The parameter of geometric v (a height, or a latitude).
    pub(super) fn param(&self, v: f64) -> f64 {
        match self {
            Self::Line { v: r, .. } => v.clamp(r[0], r[1]),
            Self::Meridian { arcs, .. } => arcs.param(v),
        }
    }

    fn knots(&self) -> Vec<f64> {
        match self {
            Self::Line { v, .. } => vec![v[0], v[0], v[1], v[1]],
            Self::Meridian { arcs, .. } => arcs.knots.clone(),
        }
    }

    /// Control points `(radius, height, weight)`.
    fn controls(&self) -> Vec<[f64; 3]> {
        match self {
            Self::Line { v, radius } => vec![[radius[0], v[0], 1.0], [radius[1], v[1], 1.0]],
            Self::Meridian { radius, arcs } => {
                arcs.points.iter().zip(&arcs.weights).map(|(p, w)| [radius * p[0], radius * p[1], *w]).collect()
            }
        }
    }
}

/// The surface of revolution about `axis` through `origin`, angle `u_arcs` (from `x`, towards
/// `y`) times `profile`, as homogeneous control points.
pub(super) fn surface(origin: V, [x, y, axis]: [V; 3], u_arcs: &Arcs, profile: &Profile) -> Surface {
    let mut rows = Vec::with_capacity(u_arcs.points.len());
    for (up, uw) in u_arcs.points.iter().zip(&u_arcs.weights) {
        let dir = add(scale(x, up[0]), scale(y, up[1]));
        let row = profile
            .controls()
            .into_iter()
            .map(|[rho, h, w]| {
                let p = add(origin, add(scale(dir, rho), scale(axis, h)));
                let weight = uw * w;
                [p[0] * weight, p[1] * weight, p[2] * weight, weight]
            })
            .collect();
        rows.push(row);
    }
    Surface::RationalSpline { u_knots: u_arcs.knots.clone(), v_knots: profile.knots(), homogeneous: rows }
}
