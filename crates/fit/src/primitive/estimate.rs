//! Estimates shared by the fits: parameter ranges, the fitted result, eigenvectors, circles and subsets of points.

use super::model::*;
use super::*;

pub(super) fn linear_range(it: impl Iterator<Item = f64>) -> (f64, f64) {
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for x in it.filter(|x| x.is_finite()) {
        lo = lo.min(x);
        hi = hi.max(x);
    }
    if lo > hi { (0.0, 1.0) } else { (lo, hi) }
}

/// Smallest arc covering the angles (full turn when they wrap around).
pub(super) fn angular_range(it: impl Iterator<Item = f64>) -> (f64, f64) {
    let mut a: Vec<f64> = it.filter(|x| x.is_finite()).map(|x| x.rem_euclid(TAU)).collect();
    if a.len() < 2 {
        return (0.0, TAU);
    }
    a.sort_by(f64::total_cmp);
    let mut gap = 0.0;
    let mut start = 0.0;
    for w in a.windows(2) {
        if let [x, y] = w
            && y - x > gap
        {
            gap = y - x;
            start = *y;
        }
    }
    let (first, last) = (a.first().copied().unwrap_or(0.0), a.last().copied().unwrap_or(TAU));
    let wrap = first + TAU - last;
    if wrap >= gap {
        return if wrap > 0.3 { (first, last) } else { (0.0, TAU) };
    }
    if gap < 0.3 {
        return (0.0, TAU);
    }
    (start, start + TAU - gap)
}

/// A primitive with its fit statistics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct PrimitiveFit {
    /// The surface.
    pub primitive: Primitive,
    /// RMS orthogonal distance (mm).
    pub rms: f64,
    /// Largest |distance| (mm).
    pub max: f64,
    /// Points used.
    pub count: usize,
}

pub(super) fn stats(prim: Primitive, pts: &[Point3]) -> Option<PrimitiveFit> {
    if !prim.is_valid() || pts.is_empty() {
        return None;
    }
    let mut sq = 0.0;
    let mut max: f64 = 0.0;
    for p in pts {
        let d = prim.distance(p);
        sq += d * d;
        max = max.max(d.abs());
    }
    let rms = (sq / pts.len() as f64).sqrt();
    rms.is_finite().then_some(PrimitiveFit { primitive: prim, rms, max, count: pts.len() })
}

/// Basis perpendicular to `d`.
pub(super) fn basis(d: &Vec3) -> (Vec3, Vec3) {
    let u = any_perpendicular(d);
    (u, d.cross(&u))
}

pub(super) fn axis_from(d0: &Vec3, u: &Vec3, w: &Vec3, a: f64, b: f64) -> Vec3 {
    normalize(d0 + u * a + w * b).unwrap_or(*d0)
}

/// Eigenvector of the smallest eigenvalue of a symmetric 3×3 matrix.
pub(super) fn min_eigvec(m: &Mat3) -> Vec3 {
    let e = m.symmetric_eigen();
    let mut k = 0;
    for i in 1..3 {
        if e.eigenvalues[i] < e.eigenvalues[k] {
            k = i;
        }
    }
    normalize(e.eigenvectors.column(k).into_owned()).unwrap_or_else(Vec3::z)
}

pub(super) fn centroid(pts: &[Point3]) -> Vec3 {
    pts.iter().map(|p| p.coords).sum::<Vec3>() / pts.len().max(1) as f64
}

/// Algebraic circle in 2-D (Kåsa): centre and radius.
pub(super) fn circle_2d(xy: &[(f64, f64)]) -> Option<(f64, f64, f64)> {
    let mut a = Matrix3::zeros();
    let mut b = Vector3::zeros();
    for &(x, y) in xy {
        let row = Vector3::new(x, y, 1.0);
        a += row * row.transpose();
        b += row * -(x * x + y * y);
    }
    let s = a.lu().solve(&b)?;
    let (cx, cy) = (-s[0] / 2.0, -s[1] / 2.0);
    let r2 = cx * cx + cy * cy - s[2];
    (r2 > 0.0 && r2.is_finite()).then(|| (cx, cy, r2.sqrt()))
}

pub(super) const MAX_LM_POINTS: usize = 4000;

/// An evenly strided subset for the iterative refinement (statistics use all points).
pub(super) fn subset(pts: &[Point3]) -> Vec<Point3> {
    let stride = pts.len().div_ceil(MAX_LM_POINTS).max(1);
    pts.iter().step_by(stride).copied().collect()
}
