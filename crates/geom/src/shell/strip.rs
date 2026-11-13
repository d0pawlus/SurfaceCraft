//! The strip along the rim and the shell mesh built from the offsets: the two layers, and between
//! them, for every boundary edge, rows of triangles that interpolate linearly between the rim of the
//! outer layer and the rim of the inner one. The relaxation tests this very mesh (all rows), the
//! assembly returns it.
use crate::{Mesh, Point3, Vec3};

/// Rows of the strip along the rim at most.
const MAX_STRIP_ROWS: usize = 64;
/// Triangles of the strip at most (the rows are cut back for a long rim).
const MAX_STRIP_TRIANGLES: usize = 4_000_000;

/// Mean length of the boundary edges (mm; 0 without any).
pub(super) fn mean_rim_edge(positions: &[Point3], boundary: &[(u32, u32)]) -> f64 {
    let total: f64 = boundary
        .iter()
        .filter_map(|&(a, b)| Some((positions.get(a as usize)? - positions.get(b as usize)?).norm()))
        .sum();
    if boundary.is_empty() { 0.0 } else { total / boundary.len() as f64 }
}

/// Rows of the strip: about one edge length each, so no triangle of it is a sliver, within
/// [`MAX_STRIP_ROWS`] and [`MAX_STRIP_TRIANGLES`].
pub(super) fn strip_rows(thickness: f64, rim_edge: f64, rim_edges: usize) -> usize {
    let by_size = if rim_edge > 0.0 { (thickness.abs() / rim_edge).ceil() as usize } else { 1 };
    let by_count = MAX_STRIP_TRIANGLES / (2 * rim_edges.max(1));
    by_size.clamp(1, MAX_STRIP_ROWS).min(by_count.max(1))
}

/// Ids of the vertices of the strip: layer `k` of rim vertex `v`, `0` the outer layer and `rows` the
/// inner one; the layers between come after the two copies of the mesh's vertices.
pub(super) struct Rim {
    vertices: u32,
    /// Rows of triangles between the outer and the inner layer.
    pub rows: usize,
    slot: Vec<u32>,
    /// The rim vertices in the order of their slots.
    order: Vec<u32>,
}

impl Rim {
    /// The rim of `boundary` over `vertices` vertices per layer, with `rows` rows.
    pub(super) fn new(vertices: u32, rows: usize, boundary: &[(u32, u32)]) -> Rim {
        let mut slot = vec![u32::MAX; vertices as usize];
        let mut order: Vec<u32> = Vec::new();
        let mut starts: Vec<u32> = boundary.iter().map(|&(a, _)| a).collect();
        starts.sort_unstable();
        for v in starts {
            if let Some(s) = slot.get_mut(v as usize) {
                *s = order.len() as u32;
                order.push(v);
            }
        }
        Rim { vertices, rows, slot, order }
    }

    /// Vertices of the shell mesh: two layers and the rows between them.
    pub(super) fn total_vertices(&self) -> usize {
        2 * self.vertices as usize + self.rows.saturating_sub(1) * self.order.len()
    }

    /// Id (in the shell mesh) of layer `k` of rim vertex `v`.
    pub(super) fn id(&self, v: u32, k: usize) -> u32 {
        if k == 0 {
            return v;
        }
        if k >= self.rows {
            return v + self.vertices;
        }
        let slot = self.slot.get(v as usize).copied().unwrap_or(0) as usize;
        (2 * self.vertices as usize + (k - 1) * self.order.len() + slot) as u32
    }

    /// The vertex of the scan that shell vertex `v` stems from (`None` for an id past the strip).
    pub(super) fn origin(&self, v: u32) -> Option<u32> {
        if v < 2 * self.vertices {
            return Some(v % self.vertices.max(1));
        }
        let slot = (v - 2 * self.vertices) as usize % self.order.len().max(1);
        self.order.get(slot).copied()
    }
}

/// The vertices of the layer at `offset` (each moved along its normal by its share of the offset).
pub(super) fn layer(positions: &[Point3], normals: &[Vec3], scale: &[f64], offset: f64) -> Vec<Point3> {
    (0..positions.len())
        .map(|i| match (positions.get(i), normals.get(i), scale.get(i)) {
            (Some(p), Some(nm), Some(s)) => p + nm * (offset * s),
            _ => Point3::origin(),
        })
        .collect()
}

/// Vertices of the shell: the outer layer, the inner layer, then the rows between them (row `k` of
/// every rim vertex, interpolated linearly).
pub(super) fn shell_positions(outer: &[Point3], inner: &[Point3], rim: &Rim) -> Vec<Point3> {
    let mut positions = Vec::with_capacity(rim.total_vertices());
    positions.extend_from_slice(outer);
    positions.extend_from_slice(inner);
    for k in 1..rim.rows {
        let f = k as f64 / rim.rows as f64;
        for &v in &rim.order {
            let (Some(o), Some(i)) = (outer.get(v as usize), inner.get(v as usize)) else { continue };
            positions.push(Point3::from(o.coords * (1.0 - f) + i.coords * f));
        }
    }
    positions
}

/// The triangles: the outer layer, the inner layer reversed, the strip.
pub(super) fn shell_triangles(triangles: &[[u32; 3]], rim: &Rim, boundary: &[(u32, u32)]) -> Vec<[u32; 3]> {
    let n = rim.vertices;
    let mut all = triangles.to_vec();
    all.extend(triangles.iter().map(|t| [t[0] + n, t[2] + n, t[1] + n]));
    for &(u, v) in boundary {
        for k in 0..rim.rows {
            let (ou, ov, iu, iv) = (rim.id(u, k), rim.id(v, k), rim.id(u, k + 1), rim.id(v, k + 1));
            all.push([ov, ou, iu]);
            all.push([ov, iu, iv]);
        }
    }
    all
}

/// The shell mesh of a scan with `triangles` whose layers are `outer` and `inner`.
pub(super) fn shell_mesh(
    triangles: &[[u32; 3]],
    outer: &[Point3],
    inner: &[Point3],
    rim: &Rim,
    boundary: &[(u32, u32)],
) -> Mesh {
    Mesh { positions: shell_positions(outer, inner, rim), triangles: shell_triangles(triangles, rim, boundary) }
}
