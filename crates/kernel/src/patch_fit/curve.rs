//! Smooth densification of the sides of a loop (design doc §4.2). The points of a side are samples
//! of a smooth curve, so the dense samples between them come from a global cubic interpolant
//! (solidcore `crv_fit::p2::interp`, chord-length parameters) and not from chords: a chord lies off
//! the curve by its sag, and a side that disagrees with the surface by that much cannot meet the
//! G0 target and pulls the fit off the data. Only calls solidcore; the given points stay.
//!
//! A side is cut at its corners (a turn of more than 45° at a point) and each smooth stretch is
//! interpolated on its own; a stretch of two points stays straight. Soft sides, and sides whose
//! points are already dense, are left alone.
use super::G0_TARGET;
use super::geom::{V2, V3, add, dist, dot, norm, scale, sub};
use solidcore::math::Vec3;
use solidcore::nurbs::crv_basic::Curve;
use solidcore::nurbs::crv_fit::p1::{ParamMethod, interp_given_params, interp_knot_vector, param_values};
use solidcore::nurbs::crv_fit::p2::interp;
use solidcore::nurbs::knots::{KnotVector, Side};

/// Degree of the interpolant (lower for stretches of three points).
const DEGREE: usize = 3;
/// Points whose largest spacing is below this share of the loop's size are dense already.
const DENSE_SHARE: f64 = 1.0 / 200.0;
/// Largest distance of the densified polyline from its smooth curve, mm.
const SAG_TARGET: f64 = 0.05 * G0_TARGET;
/// Most pieces one given chord is cut into.
const MAX_SPLIT: usize = 64;
/// Most points of a densified side, and of each smooth stretch of it.
pub(super) const MAX_OUT: usize = 4_096;
/// A point where the polyline turns by more than 45° is a corner of the side.
const CORNER_COS: f64 = std::f64::consts::FRAC_1_SQRT_2;

/// Is `b` a corner of the polyline `a`, `b`, `c`?
fn is_corner(a: V3, b: V3, c: V3) -> bool {
    let (d1, d2) = (sub(b, a), sub(c, b));
    let (n1, n2) = (norm(d1), norm(d2));
    n1 > 0.0 && n2 > 0.0 && dot(d1, d2) < CORNER_COS * n1 * n2
}

/// The point of `curve` at `t`, when it is finite.
fn point_at(curve: &Curve, t: f64) -> Option<V3> {
    let v = curve.eval(t, Side::Left).ok()?;
    let p = [v.x, v.y, v.z];
    p.iter().all(|c| c.is_finite()).then_some(p)
}

/// The curve through the `(u, v, 0)` of `uv`, on the same parameters and knots as the curve
/// through the points, so that both are evaluated at the same `t`.
fn interpolate_uv(uv: &[V2], params: &[f64], degree: usize) -> Option<Curve> {
    let pts: Vec<Vec3> = uv.iter().map(|q| Vec3::new(q[0], q[1], 0.0)).collect();
    let knots = interp_knot_vector(params, degree).ok()?;
    let kv = KnotVector::from_slice(&knots).ok()?;
    interp_given_params(&pts, params, &kv, degree).ok()
}

/// One smooth stretch (at least two points) as a dense polyline through its points, with the
/// parameters of the new points when `uv` is given.
fn smooth_piece(piece: &[V3], uv: Option<&[V2]>) -> Option<(Vec<V3>, Vec<V2>)> {
    let m = piece.len();
    if m < 3 {
        return Some((piece.to_vec(), uv.map(<[V2]>::to_vec).unwrap_or_default()));
    }
    let pts: Vec<Vec3> = piece.iter().map(|p| Vec3::new(p[0], p[1], p[2])).collect();
    let degree = DEGREE.min(m - 1);
    let curve = interp(&pts, degree, ParamMethod::ChordLength).ok()?;
    let params = param_values(&pts, ParamMethod::ChordLength).ok()?;
    let uv_curve = match uv {
        Some(q) => Some(interpolate_uv(q, &params, degree)?),
        None => None,
    };

    // pieces per chord: the sag of the curve over its chord, quartered by every doubling
    let mut splits: Vec<usize> = Vec::with_capacity(m - 1);
    for (i, w) in piece.windows(2).enumerate() {
        let [p0, p1] = w else { return None };
        let (t0, t1) = (*params.get(i)?, *params.get(i + 1)?);
        let mid = point_at(&curve, 0.5 * (t0 + t1))?;
        let sag = dist(mid, scale(add(*p0, *p1), 0.5));
        // a curve that leaves the chord by more than half its length is not a smooth side
        if !(sag.is_finite() && sag <= 0.5 * dist(*p0, *p1)) {
            return None;
        }
        let k =
            if sag <= SAG_TARGET { 1 } else { (sag / SAG_TARGET).sqrt().ceil().clamp(1.0, MAX_SPLIT as f64) as usize };
        splits.push(k);
    }
    let total: usize = splits.iter().sum();
    if total > MAX_OUT {
        for s in splits.iter_mut() {
            *s = (*s * MAX_OUT / total).max(1);
        }
    }
    let total: usize = splits.iter().sum();

    let mut out: Vec<V3> = Vec::with_capacity(total + 1);
    let mut out_uv: Vec<V2> = Vec::new();
    for (i, w) in piece.windows(2).enumerate() {
        let [p0, p1] = w else { return None };
        let k = splits.get(i).copied().unwrap_or(1);
        let (t0, t1) = (*params.get(i)?, *params.get(i + 1)?);
        out.push(*p0);
        if let Some(q) = uv {
            out_uv.push(*q.get(i)?);
        }
        let chord = dist(*p0, *p1);
        for j in 1..k {
            let f = j as f64 / k as f64;
            let t = t0 + (t1 - t0) * f;
            let q = point_at(&curve, t)?;
            let on_chord = add(scale(*p0, 1.0 - f), scale(*p1, f));
            if dist(q, on_chord) > 0.5 * chord {
                return None;
            }
            out.push(q);
            if let Some(c) = &uv_curve {
                let v = c.eval(t, Side::Left).ok()?;
                if !(v.x.is_finite() && v.y.is_finite()) {
                    return None;
                }
                out_uv.push([v.x, v.y]);
            }
        }
    }
    out.push(*piece.last()?);
    if let Some(q) = uv {
        out_uv.push(*q.last()?);
    }
    Some((out, out_uv))
}

/// A side as a dense polyline along its smooth curve; `None` when solidcore cannot interpolate it.
fn smooth_side(points: &[V3], uv: Option<&[V2]>) -> Option<(Vec<V3>, Vec<V2>)> {
    let n = points.len();
    let mut cuts = vec![0usize];
    for (i, w) in points.windows(3).enumerate() {
        if let [a, b, c] = w
            && is_corner(*a, *b, *c)
        {
            cuts.push(i + 1);
        }
    }
    cuts.push(n.checked_sub(1)?);
    let mut out: Vec<V3> = Vec::with_capacity(n);
    let mut out_uv: Vec<V2> = Vec::new();
    for w in cuts.windows(2) {
        let [a, b] = w else { return None };
        let piece = points.get(*a..=*b)?;
        let piece_uv = match uv {
            Some(u) => Some(u.get(*a..=*b)?),
            None => None,
        };
        let (p, q) = smooth_piece(piece, piece_uv)?;
        // the corner between two stretches belongs to both: keep it once
        let skip = usize::from(!out.is_empty());
        // the stretches together stay within the limit too: a side that would grow beyond it is not smoothed
        if out.len() + p.len().saturating_sub(skip) > MAX_OUT {
            return None;
        }
        out.extend(p.into_iter().skip(skip));
        out_uv.extend(q.into_iter().skip(skip));
    }
    Some((out, out_uv))
}

/// Densifies every hard side of at least three points along its smooth curve, in place (`uvs` with
/// it when the sides come with parameters, `flat`). Returns the warnings of the sides that could
/// not be smoothed and are used as polylines. `diag` is the size of the loop.
pub(super) fn smooth_sides(
    pts: &mut [Vec<V3>],
    uvs: &mut [Vec<V2>],
    flat: bool,
    soft: &[bool],
    diag: f64,
) -> Vec<String> {
    let mut warnings = Vec::new();
    for (k, (side, side_uv)) in pts.iter_mut().zip(uvs.iter_mut()).enumerate() {
        if side.len() < 3 || soft.get(k).copied().unwrap_or(false) {
            continue;
        }
        let widest = side.windows(2).filter_map(|w| Some(dist(*w.first()?, *w.get(1)?))).fold(0.0_f64, f64::max);
        if widest <= DENSE_SHARE * diag {
            continue;
        }
        match smooth_side(side, flat.then_some(side_uv.as_slice())) {
            Some((p, q)) => {
                *side = p;
                if flat {
                    *side_uv = q;
                }
            }
            None => warnings.push(format!("side {k}: could not be smoothed, its points are used as a polyline")),
        }
    }
    warnings
}
