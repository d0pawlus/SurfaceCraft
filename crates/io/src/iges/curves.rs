//! Curves of the IGES file: B-spline and conic curves (126, 100, 104, 110) and the names of curve kinds.

use super::format::*;
use super::writer::*;
use super::*;

/// Degree of a spline with `count` control points on `knots` (full, clamped or not).
pub(super) fn degree(knots: &[f64], count: usize) -> Result<usize, IoError> {
    knots
        .len()
        .checked_sub(count.saturating_add(1))
        .filter(|d| *d >= 1 && count >= 2)
        .ok_or_else(|| bad(format!("{} knots for {count} control points", knots.len())))
}

/// Points and weights of homogeneous coefficients.
pub(super) fn split_rational(h: &[[f64; 4]]) -> Result<(Vec<V3>, Vec<f64>), IoError> {
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

/// A B-spline curve (126) on full `knots` over `[t0, t1]`; rational when `weights` is given.
fn bspline_curve(
    w: &mut W,
    knots: &[f64],
    weights: Option<&[f64]>,
    points: &[V3],
    [t0, t1]: [f64; 2],
) -> Result<i64, IoError> {
    let deg = degree(knots, points.len())?;
    let rational = weights.is_some_and(|ws| ws.iter().any(|x| (x - 1.0).abs() > 1e-12));
    let mut p = vec![
        (points.len() - 1).to_string(),
        deg.to_string(),
        "0".into(),
        "0".into(),
        if rational { "0" } else { "1" }.into(),
        "0".into(),
    ];
    for k in knots {
        p.push(num(*k)?);
    }
    match weights {
        Some(ws) if ws.len() == points.len() => {
            for x in ws {
                p.push(num(*x)?);
            }
        }
        Some(_) => return Err(bad("weights and control points differ in number")),
        None => p.extend(std::iter::repeat_n(real(1.0), points.len())),
    }
    for q in points {
        p.extend(w.coords(*q)?);
    }
    p.extend([num(t0)?, num(t1)?, real(0.0), real(0.0), real(0.0)]);
    Ok(w.add(126, 0, p))
}

fn curve_name(c: &Curve) -> &'static str {
    match c {
        Curve::Line { .. } => "line",
        Curve::Circle { .. } => "circle",
        Curve::Ellipse { .. } => "ellipse",
        Curve::Polyline { .. } => "polyline",
        Curve::Spline { .. } => "spline",
        Curve::RationalSpline { .. } => "rational spline",
        Curve::SurfaceIntersection { .. } => "surface intersection",
    }
}

/// The edge `ei` as an IGES curve running from `a` to `b` (its vertices at interval min and
/// max); `closed` when they are one vertex. The DE pointer.
pub(super) fn curve(w: &mut W, ei: u32, edge: &Edge, a: V3, b: V3, closed: bool) -> Result<i64, IoError> {
    let c = edge.parameters();
    match c {
        Curve::Line { .. } => {
            let (ca, cb) = (w.coords(a)?, w.coords(b)?);
            Ok(w.add(110, 0, ca.into_iter().chain(cb).collect()))
        }
        Curve::Circle { frame, .. } | Curve::Ellipse { frame, .. } => {
            let (o, x, y) = (frame.origin, frame.x_axis, frame.y_axis);
            let local = |q: V3| {
                let v = sub(q, o);
                [dot(v, x), dot(v, y)]
            };
            let (la, lb) = (local(a), if closed { local(a) } else { local(b) });
            let tr = w.frame(o, x, y)?;
            let de = match c {
                Curve::Ellipse { radii: [ra, rb], .. } => {
                    if !(*ra > 0.0 && *rb > 0.0) {
                        return Err(bad(format!("edge {ei}: ellipse radii must be positive")));
                    }
                    // A x² + C y² + F = 0 with F = -1: A = 1/ra², C = 1/rb².
                    let p = vec![
                        num(1.0 / (ra * ra))?,
                        real(0.0),
                        num(1.0 / (rb * rb))?,
                        real(0.0),
                        real(0.0),
                        real(-1.0),
                        real(0.0),
                        num(la[0])?,
                        num(la[1])?,
                        num(lb[0])?,
                        num(lb[1])?,
                    ];
                    w.add(104, 1, p)
                }
                _ => {
                    let p = vec![real(0.0), real(0.0), real(0.0), num(la[0])?, num(la[1])?, num(lb[0])?, num(lb[1])?];
                    w.add(100, 0, p)
                }
            };
            if let Some(ent) = w.get_mut(de) {
                ent.xform = tr;
            }
            Ok(de)
        }
        // Segment i on [i, i + 1]: a degree-1 B-spline on knots 0, 0, 1, …, n − 1, n − 1.
        Curve::Polyline { points } => {
            let n = points.len();
            let knots: Vec<f64> = std::iter::once(0.0)
                .chain((0..n).map(|i| i as f64))
                .chain(std::iter::once(n.saturating_sub(1) as f64))
                .collect();
            bspline_curve(w, &knots, None, points, edge.interval)
        }
        Curve::Spline { knots, points } => bspline_curve(w, knots, None, points, edge.interval),
        Curve::RationalSpline { knots, homogeneous } => {
            let (pts, ws) = split_rational(homogeneous)?;
            bspline_curve(w, knots, Some(&ws), &pts, edge.interval)
        }
        Curve::SurfaceIntersection { .. } => {
            Err(bad(format!("edge {ei}: a {} curve is not written (no exact IGES form)", curve_name(c))))
        }
    }
}

/// The (u, v) points of a pcurve's control polygon.
pub(super) fn pcurve_points(c: &Curve2) -> Vec<[f64; 2]> {
    match c {
        Curve2::Spline { points, .. } => points.clone(),
        Curve2::RationalSpline { homogeneous, .. } => {
            homogeneous.iter().filter(|h| h[2] > 0.0).map(|h| [h[0] / h[2], h[1] / h[2]]).collect()
        }
    }
}
