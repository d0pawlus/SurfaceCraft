//! Record geometry evaluated by solidcore (`solidcore::eval`): the point of a surface at a
//! parameter pair, and how far an edge use's pcurve strays from its 3-D curve. Used to check the
//! pcurves of records read back from files; nothing here builds geometry.

use crate::record::brep::{Curve, Curve2, Surface};
use crate::{KernelError, Result, op_err};
use solidcore::eval::{CurveEval, SurfaceEval};
use solidcore::math::Vec3;

/// Samples of a pcurve when its gap to the 3-D curve is measured.
const GAP_PCURVE_SAMPLES: usize = 65;
/// Samples of the 3-D curve the closest point is searched from.
const GAP_CURVE_SAMPLES: usize = 257;
/// Golden-section steps refining the closest point.
const GAP_REFINE_STEPS: usize = 60;

fn surface_eval(s: &Surface) -> Result<SurfaceEval> {
    let k = solidcore_convert::surface(s).map_err(|e| op_err("convert surface", e))?;
    SurfaceEval::new(&k).map_err(|e| op_err("surface evaluator", e))
}

pub(crate) fn curve_eval(c: &Curve) -> Result<CurveEval> {
    let k = solidcore_convert::curve(c).map_err(|e| op_err("convert curve", e))?;
    CurveEval::new(&k).map_err(|e| op_err("curve evaluator", e))
}

pub(crate) fn point_on(eval: &CurveEval, t: f64) -> Result<Vec3> {
    eval.point(t).ok_or_else(|| KernelError::Argument(format!("edge curve has no point at t = {t}")))
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

/// The point of `surface` at `uv`, in its own parameters.
pub fn surface_point(surface: &Surface, uv: [f64; 2]) -> Result<[f64; 3]> {
    let p = surface_eval(surface)?
        .point(uv)
        .ok_or_else(|| KernelError::Argument(format!("surface has no point at {uv:?}")))?;
    Ok([p.x, p.y, p.z])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::record::brep::Frame;

    #[test]
    fn a_ruling_of_a_cylinder_lies_on_it_and_a_shifted_one_does_not() {
        let frame = Frame { origin: [0.0; 3], x_axis: [1.0, 0.0, 0.0], y_axis: [0.0, 1.0, 0.0] };
        let s = Surface::Cylinder { frame, radius: 2.0 };
        assert!((surface_point(&s, [0.0, 3.0]).unwrap()[0] - 2.0).abs() < 1e-12);
        let line = Curve::Line { start: [2.0, 0.0, 0.0], end: [2.0, 0.0, 1.0] };
        let on = Curve2::Spline { knots: vec![0.0, 0.0, 1.0, 1.0], points: vec![[0.0, 0.0], [0.0, 1.0]] };
        assert!(pcurve_gap(&s, &on, &line, [0.0, 1.0]).unwrap() < 1e-9);
        let off = Curve2::Spline { knots: vec![0.0, 0.0, 1.0, 1.0], points: vec![[0.5, 0.0], [0.5, 1.0]] };
        assert!(pcurve_gap(&s, &off, &line, [0.0, 1.0]).unwrap() > 0.4);
    }
}
