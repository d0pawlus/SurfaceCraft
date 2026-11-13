//! Surfaces of the IGES file: B-spline surfaces (128) and the faces' surfaces as their (u, v) are on them.

use super::curves::*;
use super::format::*;
use super::writer::*;
use super::*;

fn surface_name(s: &Surface) -> &'static str {
    match s {
        Surface::Plane { .. } => "plane",
        Surface::PlaneThrough { .. } => "plane through points",
        Surface::Cylinder { .. } => "cylinder",
        Surface::Cone { .. } => "cone",
        Surface::Sphere { .. } => "sphere",
        Surface::Torus { .. } => "torus",
        Surface::BallPatch { .. } => "ball patch",
        Surface::Spline { .. } => "spline",
        Surface::RationalSpline { .. } => "rational spline",
        Surface::PlacedRevolution { .. } => "placed revolution",
    }
}

/// A rational or polynomial B-spline surface (128): `points[i][j]`, i along u.
fn bspline_surface(
    w: &mut W,
    u: &[f64],
    v: &[f64],
    points: &[Vec<V3>],
    weights: Option<&[Vec<f64>]>,
) -> Result<i64, IoError> {
    let nu = points.len();
    let nv = points.first().map_or(0, Vec::len);
    if points.iter().any(|r| r.len() != nv) {
        return Err(bad("control net is not rectangular"));
    }
    let (p, q) = (degree(u, nu)?, degree(v, nv)?);
    let wt = |i: usize, j: usize| weights.and_then(|w| w.get(i)?.get(j).copied());
    if weights.is_some() && (0..nu).any(|i| (0..nv).any(|j| wt(i, j).is_none())) {
        return Err(bad("weights and control points differ in number"));
    }
    let rational = (0..nu).any(|i| (0..nv).any(|j| wt(i, j).is_some_and(|x| (x - 1.0).abs() > 1e-12)));
    let mut par = vec![
        (nu - 1).to_string(),
        (nv - 1).to_string(),
        p.to_string(),
        q.to_string(),
        "0".into(),
        "0".into(),
        if rational { "0" } else { "1" }.into(),
        "0".into(),
        "0".into(),
    ];
    for k in u.iter().chain(v) {
        par.push(num(*k)?);
    }
    // The first index (u) runs fastest.
    for j in 0..nv {
        for i in 0..nu {
            par.push(num(wt(i, j).unwrap_or(1.0))?);
        }
    }
    for j in 0..nv {
        for row in points {
            let c = row.get(j).copied().ok_or_else(|| bad("control net is not rectangular"))?;
            par.extend(w.coords(c)?);
        }
    }
    let range = |k: &[f64], d: usize, n: usize| -> Result<(f64, f64), IoError> {
        match (k.get(d), k.get(n)) {
            (Some(a), Some(b)) if a < b => Ok((*a, *b)),
            _ => Err(bad("empty spline domain")),
        }
    };
    let ((u0, u1), (v0, v1)) = (range(u, p, nu)?, range(v, q, nv)?);
    par.extend([num(u0)?, num(u1)?, num(v0)?, num(v1)?]);
    Ok(w.add(128, 0, par))
}

/// The face's surface as the face's (u, v) are on it: its NURBS form when it has one, a plane as
/// the exact bilinear patch over the face's domain. The DE pointer.
pub(super) fn surface(w: &mut W, fi: u32, face: &Face) -> Result<i64, IoError> {
    let s = face.parameters();
    let plane = |w: &mut W, o: V3, du: V3, dv: V3| -> Result<i64, IoError> {
        let [[u0, u1], [v0, v1]] = face.domain;
        if !(u0 < u1 && v0 < v1) {
            return Err(bad(format!("face {fi}: empty domain")));
        }
        let at = |u: f64, v: f64| [0, 1, 2].map(|k| o[k] + u * du[k] + v * dv[k]);
        let points = vec![vec![at(u0, v0), at(u0, v1)], vec![at(u1, v0), at(u1, v1)]];
        bspline_surface(w, &[u0, u0, u1, u1], &[v0, v0, v1, v1], &points, None)
    };
    match s {
        Surface::Spline { u_knots, v_knots, points } => bspline_surface(w, u_knots, v_knots, points, None),
        Surface::RationalSpline { u_knots, v_knots, homogeneous } => {
            let mut pts = Vec::with_capacity(homogeneous.len());
            let mut ws = Vec::with_capacity(homogeneous.len());
            for row in homogeneous {
                let (p, wr) = split_rational(row)?;
                pts.push(p);
                ws.push(wr);
            }
            bspline_surface(w, u_knots, v_knots, &pts, Some(&ws))
        }
        Surface::Plane { origin, u_axis, v_axis } => plane(w, *origin, *u_axis, *v_axis),
        Surface::PlaneThrough { origin, u_point, v_point } => {
            plane(w, *origin, sub(*u_point, *origin), sub(*v_point, *origin))
        }
        other => {
            Err(bad(format!("face {fi}: a {} surface without its NURBS form is not written yet", surface_name(other))))
        }
    }
}
