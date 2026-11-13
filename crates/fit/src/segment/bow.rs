//! How much a region bows away from its primitive, against a general smooth fit of the same points.
//!
//! A cast, worn or drafted face has form error: its residual to the best primitive is far above the
//! scanner noise, yet the face is still that primitive for a model. A fragment of a freeform surface
//! also has a residual to its best primitive, but that residual is smooth: a second-order fit of
//! small patches explains it. So two residuals over the same points tell them apart: `primitive_rms`
//! (the best primitive over the whole region) and `quadric_rms` (a height field `h = a + bu + cv +
//! du^2 + euv + fv^2` over the best-fit plane, fitted in each cell of a grid over the region, the
//! cells small enough for a quadric to be a fair model). `quadric_rms` is noise plus roughness; the
//! difference in quadrature, [`Bow::bow`], is the smooth deviation from the primitive that roughness
//! and noise do not explain.
use crate::primitive::Primitive;
use nalgebra::{DMatrix, DVector, Matrix3, SymmetricEigen};
use surfacecraft_geom::{Mesh, Point3, Vec3};

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
    ids.iter().filter_map(|&i| mesh.positions.get(i as usize).copied()).collect()
}

/// Orthonormal frame `(u, v, n)` of the best-fit plane of `pts` (n: the least-variance direction).
fn plane_frame(pts: &[Point3]) -> (Vec3, Vec3, Vec3) {
    let c = pts.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / pts.len().max(1) as f64;
    let cov = pts.iter().fold(Matrix3::zeros(), |m, p| m + (p.coords - c) * (p.coords - c).transpose());
    let eig = SymmetricEigen::new(cov);
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b]));
    let col = |k: usize| eig.eigenvectors.column(order[k]).into_owned();
    (col(2), col(1), col(0))
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
/// region has too few points for a cell to be fitted.
#[must_use]
pub fn region_bow(mesh: &Mesh, triangles: &[u32], primitive: &Primitive) -> Option<Bow> {
    let pts = sample_vertices(mesh, triangles);
    if pts.len() < MIN_CELL_POINTS {
        return None;
    }
    let (fu, fv, fnorm) = plane_frame(&pts);
    let c = pts[0].coords;
    let uvh: Vec<[f64; 3]> =
        pts.iter().map(|p| [(p.coords - c).dot(&fu), (p.coords - c).dot(&fv), (p.coords - c).dot(&fnorm)]).collect();
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
    (dof > 0).then(|| Bow {
        primitive_rms: (sq_p / used as f64).sqrt(),
        quadric_rms: (sse / dof as f64).sqrt(),
        points: used,
        extent: (hi[0] - lo[0]).max(hi[1] - lo[1]),
    })
}
