//! How much a region bows away from its primitive, against a general smooth fit of the same points.
//!
//! A cast, worn or drafted face has form error: its residual to the best primitive is far above the
//! scanner noise, yet the face is still that primitive for a model. A fragment of a freeform surface
//! also has a residual to its best primitive, but that residual is smooth: a second-order fit of
//! small patches explains it. So two residuals over the same points tell them apart: `primitive_rms`
//! (the best primitive over the whole region) and `quadric_rms` (a height field `h = a + bu + cv +
//! du^2 + euv + fv^2` fitted in each cell of a grid over the region, the cells small enough for a
//! quadric to be a fair model). The height field lives in the primitive's own parameter space:
//! a plane's plane; a cylinder's angle (as arc length) and axial position, a cone's angle and slant
//! position, with the distance to the surface as height; a sphere's equal-area azimuthal chart about
//! the mean direction (valid beyond a hemisphere), with the radial offset as height. A projection
//! over a best-fit plane would fold for a region that wraps a cylinder or sphere. The frames come from
//! the primitive's parameters, so nothing depends on the sign or the order of an eigenvector.
//! `quadric_rms` is noise plus roughness; the difference in quadrature, [`Bow::bow`], is the smooth
//! deviation from the primitive that roughness and noise do not explain. When the general fit is
//! clearly worse than the primitive (a chart that does not suit the region), the bow is not
//! measured: [`region_bow`] returns `None`, which is never the same as "no bow".
use crate::primitive::Primitive;
use nalgebra::{DMatrix, DVector};
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

impl Bow {
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

/// The points in the chart of `primitive`: `(u, v, h)` in mm (`None` for a primitive without one).
fn chart(primitive: &Primitive, pts: &[Point3]) -> Option<Vec<[f64; 3]>> {
    match primitive {
        Primitive::Plane { point, normal } => {
            let n = unit(normal)?;
            let (u, v) = (any_perpendicular(&n), n.cross(&any_perpendicular(&n)));
            let o = Vec3::new(point[0], point[1], point[2]);
            Some(pts.iter().map(|p| [(p.coords - o).dot(&u), (p.coords - o).dot(&v), (p.coords - o).dot(&n)]).collect())
        }
        Primitive::Cylinder { point, axis, radius } => {
            let a = unit(axis)?;
            Some(revolved(pts, Vec3::new(point[0], point[1], point[2]), a, *radius, |_, r, _| r - radius.abs()))
        }
        Primitive::Cone { apex, axis, half_angle } => {
            let a = unit(axis)?;
            let (sin, cos) = half_angle.sin_cos();
            let mean_r =
                pts.iter().map(|p| radial(&(p.coords - Vec3::new(apex[0], apex[1], apex[2])), &a).1).sum::<f64>()
                    / pts.len().max(1) as f64;
            let o = Vec3::new(apex[0], apex[1], apex[2]);
            Some(
                revolved(pts, o, a, mean_r, |z, r, _| r * cos - z * sin)
                    .into_iter()
                    .map(|[u, v, h]| [u, v / cos, h])
                    .collect(),
            )
        }
        Primitive::Sphere { center, radius } => {
            sphere_chart(pts, Vec3::new(center[0], center[1], center[2]), radius.abs())
        }
        Primitive::Torus { .. } => None,
    }
}

/// Axial position and distance from the axis of `q` (a vector from a point of the axis).
fn radial(q: &Vec3, a: &Vec3) -> (f64, f64) {
    let z = q.dot(a);
    (z, (q - a * z).norm())
}

/// Chart of a surface of revolution about the axis `(o, a)`: angle about the axis (centred on the
/// mean direction, as an arc `arc_radius` long), axial position, and `height(z, r, _)`.
fn revolved(pts: &[Point3], o: Vec3, a: Vec3, arc_radius: f64, height: impl Fn(f64, f64, f64) -> f64) -> Vec<[f64; 3]> {
    let e1 = any_perpendicular(&a);
    let e2 = a.cross(&e1);
    let angles: Vec<f64> = pts.iter().map(|p| (p.coords - o).dot(&e2).atan2((p.coords - o).dot(&e1))).collect();
    let mean = angles.iter().map(|t| t.sin()).sum::<f64>().atan2(angles.iter().map(|t| t.cos()).sum::<f64>());
    pts.iter()
        .zip(&angles)
        .map(|(p, &t)| {
            let (z, r) = radial(&(p.coords - o), &a);
            let wrapped = (t - mean + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI;
            [wrapped * arc_radius.abs(), z, height(z, r, wrapped)]
        })
        .collect()
}

/// Equal-area azimuthal chart of a sphere about the mean direction of the points, radial offset as
/// height; `None` when the points have no mean direction (a region around the whole sphere).
fn sphere_chart(pts: &[Point3], c: Vec3, radius: f64) -> Option<Vec<[f64; 3]>> {
    let dirs: Vec<Vec3> = pts.iter().map(|p| p.coords - c).collect();
    let mean = normalize(dirs.iter().filter_map(|d| normalize(*d)).sum::<Vec3>())?;
    let e1 = any_perpendicular(&mean);
    let e2 = mean.cross(&e1);
    Some(
        dirs.iter()
            .map(|d| {
                let r = d.norm();
                let unit = normalize(*d).unwrap_or(mean);
                let psi = unit.dot(&mean).clamp(-1.0, 1.0).acos();
                let phi = unit.dot(&e2).atan2(unit.dot(&e1));
                let rho = 2.0 * radius * (psi / 2.0).sin();
                [rho * phi.cos(), rho * phi.sin(), r - radius]
            })
            .collect(),
    )
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
    let uvh = chart(primitive, &pts)?;
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
