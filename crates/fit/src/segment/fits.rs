//! Fits of regions: the automatic fit on a bounded subsample, refits of one kind, RMS.
use super::region_points;
use crate::auto::fit_auto;
use crate::primitive::{Primitive, PrimitiveFit, PrimitiveKind};
use surfacecraft_geom::{Mesh, Point3, Vec3};

/// Points used for one fit at most (evenly strided); the final RMS is over every point.
pub(super) const FIT_POINTS: usize = 4000;

/// Automatic fit of a set of triangles on at most [`FIT_POINTS`] of their vertices.
pub(super) fn fit_tris(mesh: &Mesh, normals: &[Vec3], tris: &[u32]) -> Option<PrimitiveFit> {
    let (p, n) = region_points(mesh, normals, tris);
    if p.len() <= FIT_POINTS {
        return fit_auto(&p, &n).map(|a| a.best);
    }
    let stride = p.len().div_ceil(FIT_POINTS);
    let ps: Vec<Point3> = p.iter().step_by(stride).copied().collect();
    let ns: Vec<Vec3> = n.iter().step_by(stride).copied().collect();
    let mut f = fit_auto(&ps, &ns)?.best;
    // distances in parallel, summed in index order (the same value on every thread count)
    let prim = &f.primitive;
    let sq: f64 = surfacecraft_geom::par::map_slice(&p, |q| prim.distance(q).powi(2)).iter().sum();
    f.rms = (sq / p.len() as f64).sqrt();
    f.count = p.len();
    Some(f)
}

/// Refit of a region keeping its `kind` on its points: the fit uses an even subsample, the RMS is
/// over every point (parallel, summed in index order); `None` for no points.
pub(super) fn refit_kind_points(p: &[Point3], n: &[Vec3], kind: PrimitiveKind) -> Option<PrimitiveFit> {
    if p.is_empty() {
        return None;
    }
    let mut f = fit_kind_points(p, n, kind)?;
    f.rms = rms_of(p, &f.primitive);
    f.count = p.len();
    Some(f)
}

/// RMS of the points to `prim` (parallel, summed in index order).
pub(super) fn rms_of(p: &[Point3], prim: &Primitive) -> f64 {
    let sq: f64 = surfacecraft_geom::par::map_slice(p, |q| prim.distance(q).powi(2)).iter().sum();
    (sq / p.len() as f64).sqrt()
}

/// Fit of one `kind` to a set of triangles on at most [`FIT_POINTS`] / 2 of their vertices; the
/// RMS is over that subsample (it only steers the growth).
pub(super) fn fit_kind_tris(mesh: &Mesh, normals: &[Vec3], tris: &[u32], kind: PrimitiveKind) -> Option<PrimitiveFit> {
    let (p, n) = region_points(mesh, normals, tris);
    fit_kind_points(&p, &n, kind)
}

/// [`fit_kind_tris`] on the points of the region.
fn fit_kind_points(p: &[Point3], n: &[Vec3], kind: PrimitiveKind) -> Option<PrimitiveFit> {
    let stride = p.len().div_ceil(FIT_POINTS / 2).max(1);
    let ps: Vec<Point3> = p.iter().step_by(stride).copied().collect();
    let ns: Vec<Vec3> = n.iter().step_by(stride).copied().collect();
    crate::primitive::fit(kind, &ps, &ns)
}

/// The kind with the lowest RMS among the candidates of the automatic fit of a set of triangles
/// (on the same subsample as [`fit_tris`]): `fit_auto` prefers a simpler kind within 20 % of it.
pub(super) fn best_rms_kind(mesh: &Mesh, normals: &[Vec3], tris: &[u32]) -> Option<PrimitiveKind> {
    let (p, n) = region_points(mesh, normals, tris);
    let stride = p.len().div_ceil(FIT_POINTS).max(1);
    let ps: Vec<Point3> = p.iter().step_by(stride).copied().collect();
    let ns: Vec<Vec3> = n.iter().step_by(stride).copied().collect();
    let candidates = fit_auto(&ps, &ns)?.candidates;
    candidates.into_iter().min_by(|a, b| a.1.total_cmp(&b.1)).map(|(kind, _)| kind)
}
