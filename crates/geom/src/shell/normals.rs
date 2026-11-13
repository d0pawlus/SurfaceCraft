//! Vertex normals for the offset: angle-weighted (Thürmer and Wüthrich, "Computing vertex normals
//! from polygonal facets", Journal of Graphics Tools 3(1), 1998, so the normal does not depend on how
//! a fan of triangles around a vertex is cut), then smoothed over the one-ring with a few
//! deterministic Laplacian passes so that scanner noise does not fold the offset.
use super::topology::Csr;
use crate::{Point3, Vec3, normalize, par};

/// Passes of the one-ring averaging of the normals. Each pass averages the normal with the mean of
/// its neighbours; three take the noise of a scan (σ well below the edge length) down to a fraction
/// of a degree and leave the normals of smooth surfaces unchanged (the averaging is symmetric).
pub(super) const SMOOTH_PASSES: usize = 3;

/// Angle-weighted normals of the vertices (zero for a vertex whose triangles have no area).
pub(super) fn angle_weighted(positions: &[Point3], triangles: &[[u32; 3]]) -> Vec<Vec3> {
    let mut n = vec![Vec3::zeros(); positions.len()];
    for t in triangles {
        let (Some(a), Some(b), Some(c)) =
            (positions.get(t[0] as usize), positions.get(t[1] as usize), positions.get(t[2] as usize))
        else {
            continue;
        };
        let Some(face) = normalize((b - a).cross(&(c - a))) else { continue };
        for (k, o, p, q) in [(0usize, a, b, c), (1, b, c, a), (2, c, a, b)] {
            let (u, v) = (p - o, q - o);
            let angle = u.cross(&v).norm().atan2(u.dot(&v));
            if let Some(slot) = n.get_mut(t[k] as usize) {
                *slot += face * angle;
            }
        }
    }
    n.into_iter().map(|v| normalize(v).unwrap_or_else(Vec3::zeros)).collect()
}

/// One averaging pass: the normal plus the mean of its neighbours, renormalised (a vertex whose sum
/// vanishes keeps its normal).
fn pass(normals: &[Vec3], adjacent: &Csr) -> Vec<Vec3> {
    par::map(normals.len(), |i| {
        let own = normals.get(i).copied().unwrap_or_else(Vec3::zeros);
        let ring = adjacent.row(i);
        if ring.is_empty() {
            return own;
        }
        let sum: Vec3 = ring.iter().filter_map(|&j| normals.get(j as usize)).sum();
        normalize(own + sum / ring.len() as f64).unwrap_or(own)
    })
}

/// The smoothed normals, or `None` when a vertex ends up without one.
pub(super) fn smoothed(positions: &[Point3], triangles: &[[u32; 3]], adjacent: &Csr) -> Option<Vec<Vec3>> {
    let mut n = angle_weighted(positions, triangles);
    for _ in 0..SMOOTH_PASSES {
        n = pass(&n, adjacent);
    }
    n.iter().all(|v| v.norm() > 0.5).then_some(n)
}
