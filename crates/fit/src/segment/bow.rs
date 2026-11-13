//! How much a region bows away from its primitive, against a general smooth fit of the same points.
//!
//! A cast, worn or drafted face has form error: its residual to the best primitive is far above the
//! scanner noise, yet the face is still that primitive for a model. A fragment of a freeform surface
//! also has a residual to its best primitive, but that residual is smooth: a second-order fit of
//! small patches explains it. So two residuals over the same points tell them apart: `primitive_rms`
//! (the best primitive over the whole region) and `quadric_rms` (a height field `h = a + bu + cv +
//! du^2 + euv + fv^2` fitted in each cell of a grid over the region, the cells small enough for a
//! quadric to be a fair model). A region that spans little of its primitive (under [`WRAP_HALF_SPAN`] either side
//! about the axis or the centre, which is every gentle patch) is charted over its best-fit plane,
//! the heights being the distances to that plane; the frame is the principal axes of the points, a
//! square-ish grid whose result does not depend on the sign of an axis. A region that wraps a
//! cylinder or sphere further would fold under that projection, the quadric would fit worse than the
//! primitive and the bow would read zero: it is charted in the primitive's own parameter space
//! instead: a cylinder's angle (as arc length) and axial position, a cone's angle and slant position,
//! with the distance to the surface as height; a sphere's equal-area azimuthal chart about the mean
//! direction, with the radial offset as height.
//! `quadric_rms` is noise plus roughness; the difference in quadrature, [`Bow::bow`], is the smooth
//! deviation from the primitive that roughness and noise do not explain. When the general fit is
//! clearly worse than the primitive (a chart that does not suit the region), the bow is not
//! measured: [`region_bow`] returns `None`, which is never the same as "no bow".
use crate::primitive::Primitive;
use nalgebra::{DMatrix, DVector, Matrix3, SymmetricEigen};
use surfacecraft_geom::{Mesh, Point3, Vec3, any_perpendicular, finite, normalize};

/// Triangles looked at at most (evenly strided).
const SAMPLE_TRIANGLES: usize = 4000;
/// The region is cut into at most this many cells along each side of its box over the best-fit
/// plane (fewer for a small region: about [`POINTS_PER_CELL`] points a cell on average).
const CELLS: usize = 8;
/// Points a cell should hold on average.
const POINTS_PER_CELL: usize = 40;
/// A cell with fewer points than this is not fitted (the quadric has six terms).
const MIN_CELL_POINTS: usize = 24;
/// A region wraps its primitive when its points reach this far (rad) from the mean direction about
/// the axis or centre (a quarter turn each side: past that a projection over a plane folds).
const WRAP_HALF_SPAN: f64 = std::f64::consts::FRAC_PI_4;
/// Terms of the quadric.
const TERMS: usize = 6;
/// The general fit may be at most this many times the primitive's RMS: more means the chart does not
/// suit the region and the bow is not measured (a quadric per cell has more freedom than a primitive).
const RELIABLE_RATIO: f64 = 1.25;

/// The two residuals of a region over the same points, and how many points that was.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bow {
    /// RMS of the distance of the points to the primitive (mm).
    pub primitive_rms: f64,
    /// RMS of the points to the quadrics of their cells (mm, degrees of freedom taken out).
    pub quadric_rms: f64,
    /// Points in the cells that were fitted.
    pub points: usize,
    /// Extent of the region over its best-fit plane: the larger side of the box (mm).
    pub extent: f64,
}

/// How far above the scanner noise the general fit's RMS is credited (it may explain roughness, but
/// a residual that is mostly noise must not make a region look flat): at most this factor times the
/// noise.
const NOISE_CREDIT: f64 = 1.1;

impl Bow {
    /// [`Bow::bow`] when the scanner noise `sigma` (mm) is known: the general fit is credited with
    /// explaining at most [`NOISE_CREDIT`] times the noise, so a region whose residual is mostly noise
    /// does not read as flat (with `sigma` 0 this is the plain bow).
    #[must_use]
    pub fn bow_given(&self, sigma: f64) -> f64 {
        if sigma <= 0.0 {
            return self.bow();
        }
        let credited = self.quadric_rms.min(NOISE_CREDIT * sigma);
        (self.primitive_rms.powi(2) - credited.powi(2)).max(0.0).sqrt()
    }

    /// The smooth deviation from the primitive that the general fit explains and noise does not
    /// (mm): `sqrt(primitive_rms^2 - quadric_rms^2)`, zero when the quadric is no better.
    #[must_use]
    pub fn bow(&self) -> f64 {
        (self.primitive_rms.powi(2) - self.quadric_rms.powi(2)).max(0.0).sqrt()
    }
}

/// The vertices of a sample of `triangles`, once each.
fn sample_vertices(mesh: &Mesh, triangles: &[u32]) -> Vec<Point3> {
    let stride = triangles.len().div_ceil(SAMPLE_TRIANGLES).max(1);
    let mut ids: Vec<u32> = triangles
        .iter()
        .step_by(stride)
        .filter_map(|&t| mesh.triangles.get(t as usize))
        .flat_map(|t| t.iter().copied())
        .collect();
    ids.sort_unstable();
    ids.dedup();
    ids.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()).filter(finite).collect()
}

/// Unit vector of an array, `None` when it has no length.
fn unit(v: &[f64; 3]) -> Option<Vec3> {
    normalize(Vec3::new(v[0], v[1], v[2]))
}

/// Axial position and distance from the axis of `q` (a vector from a point of the axis).
fn radial(q: &Vec3, a: &Vec3) -> (f64, f64) {
    let z = q.dot(a);
    (z, (q - a * z).norm())
}

/// Chart of a surface of revolution about the axis `(o, a)`: angle about the axis (centred on the
/// mean direction, as an arc `arc_radius` long), axial position, and `height(z, r)`; with the
/// largest angle from the mean direction (rad).
fn revolved(
    pts: &[Point3],
    (o, a): (Vec3, Vec3),
    arc_radius: f64,
    height: impl Fn(f64, f64) -> f64,
) -> (Vec<[f64; 3]>, f64) {
    let e1 = any_perpendicular(&a);
    let e2 = a.cross(&e1);
    let angles: Vec<f64> = pts.iter().map(|p| (p.coords - o).dot(&e2).atan2((p.coords - o).dot(&e1))).collect();
    let mean = angles.iter().map(|t| t.sin()).sum::<f64>().atan2(angles.iter().map(|t| t.cos()).sum::<f64>());
    let pi = std::f64::consts::PI;
    let wrapped: Vec<f64> = angles.iter().map(|t| (t - mean + pi).rem_euclid(std::f64::consts::TAU) - pi).collect();
    let chart = pts
        .iter()
        .zip(&wrapped)
        .map(|(p, &t)| {
            let (z, r) = radial(&(p.coords - o), &a);
            [t * arc_radius.abs(), z, height(z, r)]
        })
        .collect();
    (chart, wrapped.iter().fold(0.0, |m, t| f64::max(m, t.abs())))
}

/// Equal-area azimuthal chart of a sphere about the mean direction of the points, radial offset as
/// height, with the largest angle from the mean direction (rad); `None` when the points have no
/// mean direction (a region around the whole sphere).
fn sphere_chart(pts: &[Point3], c: Vec3, radius: f64) -> Option<(Vec<[f64; 3]>, f64)> {
    let dirs: Vec<Vec3> = pts.iter().map(|p| p.coords - c).collect();
    let mean = normalize(dirs.iter().filter_map(|d| normalize(*d)).sum::<Vec3>())?;
    let e1 = any_perpendicular(&mean);
    let e2 = mean.cross(&e1);
    let mut widest = 0.0f64;
    let chart = dirs
        .iter()
        .map(|d| {
            let unit = normalize(*d).unwrap_or(mean);
            let psi = unit.dot(&mean).clamp(-1.0, 1.0).acos();
            widest = widest.max(psi);
            let phi = unit.dot(&e2).atan2(unit.dot(&e1));
            let rho = 2.0 * radius * (psi / 2.0).sin();
            [rho * phi.cos(), rho * phi.sin(), d.norm() - radius]
        })
        .collect();
    Some((chart, widest))
}

/// The chart in the primitive's own parameter space, for a region that wraps its cylinder, cone or
/// sphere by more than [`WRAP_HALF_SPAN`] about the axis or centre; `None` for every other region.
fn wrapped_chart(primitive: &Primitive, pts: &[Point3]) -> Option<Vec<[f64; 3]>> {
    let (chart, span) = match primitive {
        Primitive::Cylinder { point, axis, radius } => {
            let o = Vec3::new(point[0], point[1], point[2]);
            revolved(pts, (o, unit(axis)?), *radius, |_, r| r - radius.abs())
        }
        Primitive::Cone { apex, axis, half_angle } => {
            let (a, o) = (unit(axis)?, Vec3::new(apex[0], apex[1], apex[2]));
            let (sin, cos) = half_angle.sin_cos();
            let mean_r = pts.iter().map(|p| radial(&(p.coords - o), &a).1).sum::<f64>() / pts.len().max(1) as f64;
            let (chart, span) = revolved(pts, (o, a), mean_r, |z, r| r * cos - z * sin);
            (chart.into_iter().map(|[u, v, h]| [u, v / cos, h]).collect(), span)
        }
        Primitive::Sphere { center, radius } => {
            sphere_chart(pts, Vec3::new(center[0], center[1], center[2]), radius.abs())?
        }
        Primitive::Plane { .. } | Primitive::Torus { .. } => return None,
    };
    (span >= WRAP_HALF_SPAN).then_some(chart)
}

/// Frame `(u, v, n)` of the best-fit plane of `pts` (n: the least-variance direction).
fn plane_frame(pts: &[Point3]) -> (Vec3, Vec3, Vec3) {
    let c = pts.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / pts.len().max(1) as f64;
    let cov = pts.iter().fold(Matrix3::zeros(), |m, p| m + (p.coords - c) * (p.coords - c).transpose());
    let eig = SymmetricEigen::new(cov);
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b]));
    let col = |k: usize| eig.eigenvectors.column(order[k]).into_owned();
    (col(2), col(1), col(0))
}

/// The points over their best-fit plane: `(u, v, distance to the plane)`.
fn plane_projection(pts: &[Point3]) -> Vec<[f64; 3]> {
    let (fu, fv, fnorm) = plane_frame(pts);
    let c = pts[0].coords;
    pts.iter().map(|p| [(p.coords - c).dot(&fu), (p.coords - c).dot(&fv), (p.coords - c).dot(&fnorm)]).collect()
}

/// Sum of squared residuals and degrees of freedom of the quadric through the points of one cell
/// (`uvh`: coordinates over the best-fit plane), `None` when it cannot be solved.
fn cell_fit(uvh: &[[f64; 3]]) -> Option<(f64, usize)> {
    let n = uvh.len();
    let m = uvh.iter().fold([0.0; 2], |a, p| [a[0] + p[0] / n as f64, a[1] + p[1] / n as f64]);
    let scale = uvh.iter().map(|p| (p[0] - m[0]).abs().max((p[1] - m[1]).abs())).fold(1e-12, f64::max);
    let a = DMatrix::from_fn(n, TERMS, |i, j| {
        let (x, y) = ((uvh[i][0] - m[0]) / scale, (uvh[i][1] - m[1]) / scale);
        [1.0, x, y, x * x, x * y, y * y][j]
    });
    let b = DVector::from_fn(n, |i, _| uvh[i][2]);
    let x = a.clone().svd(true, true).solve(&b, 1e-10).ok()?;
    let r = &a * x - b;
    Some((r.norm_squared(), n - TERMS))
}

/// The two residuals of the region `triangles` of `mesh` against `primitive`; `None` when the
/// primitive has no chart (a torus), the region has too few points for most of it to be fitted in
/// cells, or the general fit is clearly worse than the primitive ([`RELIABLE_RATIO`]): the bow is
/// then not measured.
#[must_use]
pub fn region_bow(mesh: &Mesh, triangles: &[u32], primitive: &Primitive) -> Option<Bow> {
    let pts = sample_vertices(mesh, triangles);
    if pts.len() < MIN_CELL_POINTS {
        return None;
    }
    let uvh = wrapped_chart(primitive, &pts).unwrap_or_else(|| plane_projection(&pts));
    if uvh.iter().any(|p| p.iter().any(|x| !x.is_finite())) {
        return None;
    }
    let lo = uvh.iter().fold([f64::INFINITY; 2], |a, p| [a[0].min(p[0]), a[1].min(p[1])]);
    let hi = uvh.iter().fold([f64::NEG_INFINITY; 2], |a, p| [a[0].max(p[0]), a[1].max(p[1])]);
    let side = (((uvh.len() / POINTS_PER_CELL) as f64).sqrt() as usize).clamp(1, CELLS);
    let cell =
        |p: &[f64; 3], k: usize| (((p[k] - lo[k]) / (hi[k] - lo[k]).max(1e-12) * side as f64) as usize).min(side - 1);
    let mut cells: Vec<Vec<usize>> = vec![Vec::new(); side * side];
    for (i, p) in uvh.iter().enumerate() {
        cells[cell(p, 0) * side + cell(p, 1)].push(i);
    }
    let (mut sse, mut dof, mut used, mut sq_p) = (0.0, 0usize, 0usize, 0.0);
    for ids in cells.iter().filter(|c| c.len() >= MIN_CELL_POINTS) {
        let group: Vec<[f64; 3]> = ids.iter().map(|&i| uvh[i]).collect();
        let Some((s, d)) = cell_fit(&group) else { continue };
        sse += s;
        dof += d;
        used += ids.len();
        sq_p += ids.iter().map(|&i| primitive.distance(&pts[i]).powi(2)).sum::<f64>();
    }
    if dof == 0 || used * 2 < uvh.len() {
        return None;
    }
    let bow = Bow {
        primitive_rms: (sq_p / used as f64).sqrt(),
        quadric_rms: (sse / dof as f64).sqrt(),
        points: used,
        extent: (hi[0] - lo[0]).max(hi[1] - lo[1]),
    };
    (bow.quadric_rms <= RELIABLE_RATIO * bow.primitive_rms).then_some(bow)
}
