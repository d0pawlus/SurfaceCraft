//! Unstructured layouts (P4): `quad.retopo`, the layout helpers shared with `quad.split` and
//! `quad.snap`, and the unstructured branch of `surface.auto`.
use surfacecraft_fit::mls::ScanSurface;
use surfacecraft_geom::{Mesh, Point3, Vec3};

mod auto;
mod field;
mod layout;
mod retopo;
#[cfg(test)]
mod tests;

pub(super) use auto::auto_unstructured;
pub use layout::QuadIn;
pub(super) use layout::{check_layout, info_of, layout_of, layout_radius, scan_for, split_unstructured};
pub(super) use retopo::retopo_cmd;
pub use retopo::{RetopoIn, RetopoOut};

/// Mean triangle edge length of `tris` (the scan's sampling distance), mm.
pub(super) fn mean_edge(m: &Mesh, tris: &[u32]) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for t in tris.iter().step_by(tris.len().div_ceil(20_000).max(1)) {
        if let Some([a, b, c]) = m.triangle(*t as usize) {
            sum += (b - a).norm() + (c - b).norm() + (a - c).norm();
            n += 3;
        }
    }
    sum / n.max(1) as f64
}

/// The projection used for layout vertices and patch edges: moving least squares over the scan.
pub(super) fn projector(scan: &ScanSurface, radius: f64) -> impl Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)> + '_ {
    move |p, n| scan.project(p, radius, n)
}
