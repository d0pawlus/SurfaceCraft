//! Display level of detail for large scans (P2): vertex clustering on a uniform grid
//! (Rossignac & Borrel 1993). Linear time, so a 10 M-triangle scan gets a few-million-triangle
//! display mesh in well under a second, and it keeps the map from original vertices and triangles
//! to the coarse ones, so per-vertex colours (deviation maps, regions) and triangle selections
//! carry over without recomputing anything on the coarse mesh.
use crate::{Mesh, Point3, Vec3};
use std::collections::HashMap;

/// A coarse display mesh and how the original maps onto it.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Lod {
    /// The coarse mesh.
    pub mesh: Mesh,
    /// Original vertex → coarse vertex.
    pub vertex_map: Vec<u32>,
    /// Original triangle → coarse triangle (`u32::MAX` when it collapsed).
    pub triangle_map: Vec<u32>,
}

impl Lod {
    /// The largest distance from an original vertex of `original` to its coarse vertex: the
    /// measured counterpart of the `√3·h` bound of [`cluster_on_grid`].
    #[must_use]
    pub fn max_displacement(&self, original: &Mesh) -> f64 {
        original
            .positions
            .iter()
            .zip(&self.vertex_map)
            .filter_map(|(p, &c)| self.mesh.positions.get(c as usize).map(|q| (p - q).norm()))
            .fold(0.0, f64::max)
    }

    /// Coarse per-vertex colours: the mean of the original vertices of each cluster.
    #[must_use]
    pub fn map_colors(&self, colors: &[[f32; 4]]) -> Vec<[f32; 4]> {
        let n = self.mesh.positions.len();
        let mut acc = vec![[0f32; 4]; n];
        let mut cnt = vec![0u32; n];
        for (i, &c) in self.vertex_map.iter().enumerate() {
            let (Some(src), Some(dst), Some(k)) = (colors.get(i), acc.get_mut(c as usize), cnt.get_mut(c as usize))
            else {
                continue;
            };
            for j in 0..4 {
                dst[j] += src[j];
            }
            *k += 1;
        }
        for (a, k) in acc.iter_mut().zip(&cnt) {
            if *k > 0 {
                for v in a.iter_mut() {
                    *v /= *k as f32;
                }
            }
        }
        acc
    }

    /// Coarse triangles covering a set of original triangles (sorted, unique).
    #[must_use]
    pub fn map_triangles(&self, tris: &[u32]) -> Vec<u32> {
        let mut v: Vec<u32> = tris
            .iter()
            .filter_map(|&t| self.triangle_map.get(t as usize).copied())
            .filter(|&t| t != u32::MAX)
            .collect();
        v.sort_unstable();
        v.dedup();
        v
    }
}

/// Clusters vertices on a grid chosen so the result has about `budget` triangles (or fewer).
/// Meshes already within the budget come back unchanged with identity maps. `None` for an invalid
/// mesh.
#[must_use]
pub fn cluster(mesh: &Mesh, budget: usize) -> Option<Lod> {
    mesh.validate().ok()?;
    let nt = mesh.triangles.len();
    if nt <= budget.max(4) {
        return Some(Lod {
            mesh: mesh.clone(),
            vertex_map: (0..mesh.positions.len() as u32).collect(),
            triangle_map: (0..nt as u32).collect(),
        });
    }
    let b = crate::mesh::source_bbox(mesh);
    if b.is_empty() {
        return None;
    }
    // a surface of area A sampled at cell size h has about 2 A / h² triangles
    let area = crate::mesh::source_area(mesh).max(1e-12);
    let mut h = (2.0 * area / budget.max(4) as f64).sqrt();
    for _attempt in 0..6 {
        let lod = cluster_on_grid(mesh, &b.min, h);
        if lod.mesh.triangles.len() <= budget.max(4) {
            return Some(lod);
        }
        h *= 1.25;
    }
    Some(cluster_on_grid(mesh, &b.min, h))
}

/// Clusters the vertices of `mesh` on the uniform grid of cell size `h` anchored at `origin`; each
/// coarse vertex is the mean of its cell. Triangles whose corners fall in fewer than three cells
/// disappear (`triangle_map` holds `u32::MAX`), equal coarse triangles are kept once.
///
/// Error bound: a cell has diagonal `√3·h` and the mean of its vertices lies in it, so every
/// original vertex ends up within `√3·h` of its coarse vertex ([`Lod::max_displacement`] measures
/// the actual worst case). A point of an original triangle is a barycentric mix of its corners; the
/// same mix of their coarse vertices is a point of the coarse triangle (or its edge, or vertex) at
/// most `√3·h` away, so every point of the *coarse* surface lies within `√3·h` of the original
/// surface. The reverse direction holds for every triangle that survives; a triangle that
/// collapsed keeps its corners covered by coarse vertices but its interior is not bounded.
#[must_use]
pub fn cluster_on_grid(mesh: &Mesh, origin: &Point3, h: f64) -> Lod {
    let inv = 1.0 / h.max(1e-12);
    let mut cell_id: HashMap<[i64; 3], u32> = HashMap::with_capacity(mesh.positions.len() / 4);
    let mut sum: Vec<(Vec3, u32)> = Vec::new();
    let mut vertex_map = Vec::with_capacity(mesh.positions.len());
    for p in &mesh.positions {
        let d = (p - origin) * inv;
        let key = [d.x.floor() as i64, d.y.floor() as i64, d.z.floor() as i64];
        let id = *cell_id.entry(key).or_insert_with(|| {
            sum.push((Vec3::zeros(), 0));
            (sum.len() - 1) as u32
        });
        if let Some(s) = sum.get_mut(id as usize) {
            s.0 += p.coords;
            s.1 += 1;
        }
        vertex_map.push(id);
    }
    let positions: Vec<Point3> = sum.iter().map(|(s, n)| Point3::from(s / f64::from((*n).max(1)))).collect();
    let mut seen: HashMap<[u32; 3], u32> = HashMap::new();
    let mut triangles = Vec::new();
    let mut triangle_map = Vec::with_capacity(mesh.triangles.len());
    for t in &mesh.triangles {
        let m = [
            vertex_map.get(t[0] as usize).copied().unwrap_or(0),
            vertex_map.get(t[1] as usize).copied().unwrap_or(0),
            vertex_map.get(t[2] as usize).copied().unwrap_or(0),
        ];
        if m[0] == m[1] || m[1] == m[2] || m[0] == m[2] {
            triangle_map.push(u32::MAX);
            continue;
        }
        // the same coarse triangle from several fine ones is kept once (rotation-invariant key)
        let r = m.iter().enumerate().min_by_key(|(_, v)| **v).map_or(0, |(i, _)| i);
        let key = [m[r], m[(r + 1) % 3], m[(r + 2) % 3]];
        let id = *seen.entry(key).or_insert_with(|| {
            triangles.push(m);
            (triangles.len() - 1) as u32
        });
        triangle_map.push(id);
    }
    Lod { mesh: Mesh { positions, triangles }, vertex_map, triangle_map }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth;

    #[test]
    fn small_meshes_pass_through_large_ones_shrink_with_maps() {
        let mut s = synth::sphere(Point3::origin(), 20.0, 256, 128);
        s.weld(1e-9);
        let same = cluster(&s, 1_000_000).unwrap();
        assert_eq!(same.mesh, s);
        let n = s.triangle_count();
        let lod = cluster(&s, n / 10).unwrap();
        assert!(lod.mesh.triangle_count() <= n / 10, "{}", lod.mesh.triangle_count());
        assert!(lod.mesh.triangle_count() >= n / 40, "{}", lod.mesh.triangle_count());
        // the coarse surface stays close to the sphere (cluster means sit inside by < h²/2R)
        let err = lod.mesh.positions.iter().map(|p| (p.coords.norm() - 20.0).abs()).fold(0.0, f64::max);
        assert!(err < 0.5, "{err}");
        assert_eq!(lod.vertex_map.len(), s.vertex_count());
        assert_eq!(lod.triangle_map.len(), n);
        // colours: every coarse vertex is the mean of its cluster
        let colors: Vec<[f32; 4]> =
            s.positions.iter().map(|p| if p.z > 0.0 { [1.0, 0.0, 0.0, 1.0] } else { [0.0, 0.0, 1.0, 1.0] }).collect();
        let c = lod.map_colors(&colors);
        assert_eq!(c.len(), lod.mesh.vertex_count());
        for (v, col) in lod.mesh.positions.iter().zip(&c) {
            if v.z > 1.0 {
                assert_eq!(*col, [1.0, 0.0, 0.0, 1.0]);
            }
        }
        // a selection of the upper half maps to coarse triangles of the upper half
        let upper: Vec<u32> = (0..n as u32).filter(|&t| s.face_centroid(t as usize).unwrap().z > 5.0).collect();
        let mapped = lod.map_triangles(&upper);
        assert!(!mapped.is_empty());
        assert!(mapped.iter().all(|&t| lod.mesh.face_centroid(t as usize).unwrap().z > 3.0));
        assert!(cluster(&Mesh { positions: vec![], triangles: vec![[0, 1, 2]] }, 10).is_none());
    }

    #[test]
    fn grid_clustering_stays_within_the_stated_bound() {
        let mut s = synth::torus(Point3::origin(), Vec3::z(), 30.0, 8.0, 96, 48);
        s.weld(1e-9);
        let origin = s.bbox().min;
        let bvh = crate::Bvh::build(&s);
        for h in [1.0, 2.5, 6.0] {
            let lod = cluster_on_grid(&s, &origin, h);
            let bound = 3f64.sqrt() * h;
            let measured = lod.max_displacement(&s);
            assert!(measured > 0.0 && measured <= bound + 1e-9, "h {h}: {measured} vs {bound}");
            // every point of the coarse surface (corners and centroids) lies within the bound of the original
            for i in 0..lod.mesh.triangle_count() {
                let [a, b, c] = lod.mesh.triangle(i).unwrap();
                let mid = Point3::from((a.coords + b.coords + c.coords) / 3.0);
                for q in [a, b, c, mid] {
                    let d = bvh.closest(&q, f64::INFINITY).unwrap().distance;
                    assert!(d <= bound + 1e-9, "h {h}: coarse point {d} from the original, bound {bound}");
                }
            }
        }
    }
}
