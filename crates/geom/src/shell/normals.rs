//! Vertex normals for the offset: angle-weighted (Thürmer and Wüthrich, "Computing vertex normals
//! from polygonal facets", Journal of Graphics Tools 3(1), 1998, so the normal does not depend on how
//! a fan of triangles around a vertex is cut), then averaged over a ball of half the wall so that
//! scanner noise does not fold the offset.
use crate::{KdTree, Point3, Vec3, normalize, par};

/// Neighbours at most that average a normal (the nearest ones when a ball holds more: a fine scan).
const MAX_BALL: usize = 256;
/// Normals that make a larger angle than this cosine with the vertex's own are not averaged in (the
/// other side of a thin wall, the far side of a crease).
const MIN_AGREEMENT_COS: f64 = 0.5;

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

/// The normals averaged over the vertices within `radius` (a distance, mm) that face about the same
/// way, then renormalised; `None` when a vertex ends up without a normal. Noise finer than the
/// radius cannot fold an offset of that size, so the radius is half the wall (but at least one edge).
pub(super) fn smoothed(positions: &[Point3], triangles: &[[u32; 3]], radius: f64) -> Option<Vec<Vec3>> {
    let raw = angle_weighted(positions, triangles);
    let tree = KdTree::build(positions);
    let n = par::map(positions.len(), |i| {
        let own = raw.get(i).copied().unwrap_or_else(Vec3::zeros);
        let Some(p) = positions.get(i) else { return own };
        let mut ids = tree.within(p, radius);
        if ids.len() > MAX_BALL {
            ids = tree.knn(p, MAX_BALL).into_iter().map(|(j, _)| j).collect();
        }
        ids.sort_unstable();
        let sum: Vec3 = ids.iter().filter_map(|&j| raw.get(j)).filter(|v| v.dot(&own) > MIN_AGREEMENT_COS).sum();
        normalize(sum).unwrap_or(own)
    });
    n.iter().all(|v| v.norm() > 0.5).then_some(n)
}
