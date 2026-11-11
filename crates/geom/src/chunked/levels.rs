//! One chunk, finalised: the welded level-0 mesh and its coarser levels (pure, no file access).
use crate::lod::cluster_on_grid;
use crate::{Aabb, Mesh, Point3, edit, par};

/// How the levels of a store are built.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LevelParams {
    /// Origin of the clustering grid shared by all chunks.
    pub grid_origin: Point3,
    /// Cell size `h_1` of level 1, mm; level `l` uses `h_1 · 2^(l-1)`.
    pub base_cell: f64,
    /// Coarse levels above full resolution.
    pub levels: usize,
    /// Keep the vertex and triangle maps.
    pub keep_maps: bool,
}

/// One level of one chunk.
#[derive(Clone, Debug, PartialEq)]
pub struct LevelBlocks {
    /// The mesh at this level.
    pub mesh: Mesh,
    /// Measured worst distance of a level-0 vertex from its coarse vertex (0 at level 0).
    pub error: f64,
    /// Level-0 vertex → vertex here, level-0 triangle → triangle here (`u32::MAX` = collapsed).
    pub maps: Option<(Vec<u32>, Vec<u32>)>,
}

/// All levels of one chunk, level 0 first.
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkLevels {
    /// The levels.
    pub levels: Vec<LevelBlocks>,
}

impl ChunkLevels {
    /// Tight box around the level-0 vertices.
    #[must_use]
    pub fn bbox(&self) -> Aabb {
        self.levels.first().map_or_else(Aabb::empty, |l| l.mesh.bbox())
    }
}

/// Cell size of coarse level `level ≥ 1`.
#[must_use]
pub fn cell_size(base_cell: f64, level: usize) -> f64 {
    base_cell * 2f64.powi(level.saturating_sub(1).min(60) as i32)
}

/// Stated error bound of clustering on cells of size `cell`: their half-open diagonal, `√3·h`.
#[must_use]
pub fn error_bound(cell: f64) -> f64 {
    3f64.sqrt() * cell
}

/// A sensible `h_1`: twice the mean edge length of an equilateral triangulation of `area` with
/// `triangles` triangles (about four times fewer triangles per level), falling back to a
/// thousandth of the box diagonal for degenerate input.
#[must_use]
pub fn base_cell_size(area: f64, triangles: u64, bbox: &Aabb) -> f64 {
    let mean_edge =
        if triangles > 0 && area > 0.0 { (4.0 * area / (3f64.sqrt() * triangles as f64)).sqrt() } else { 0.0 };
    let h = 2.0 * mean_edge;
    if h.is_finite() && h > 0.0 {
        return h;
    }
    let fallback = bbox.diagonal() / 1024.0;
    if fallback.is_finite() && fallback > 0.0 { fallback } else { 1.0 }
}

/// Turns the triangle soup of a chunk into its level-0 mesh, with the same rules as the in-memory
/// import: exact-duplicate vertices welded (`1e-9 mm` grid key), collapsed and zero-area triangles
/// dropped, unreferenced vertices removed.
#[must_use]
pub fn finalize_soup(mut soup: Mesh) -> Mesh {
    soup.weld(0.0);
    soup.remove_degenerate(0.0);
    edit::compact(&mut soup);
    soup
}

/// Builds every level of a chunk from its level-0 mesh. Each level is clustered from level 0 (not
/// from the previous level), so errors do not add up; levels are built in parallel.
#[must_use]
pub fn build_levels(base: Mesh, p: &LevelParams) -> ChunkLevels {
    let coarse = par::map(p.levels, |k| {
        let lod = cluster_on_grid(&base, &p.grid_origin, cell_size(p.base_cell, k + 1));
        let error = lod.max_displacement(&base);
        let maps = p.keep_maps.then_some((lod.vertex_map, lod.triangle_map));
        LevelBlocks { mesh: lod.mesh, error, maps }
    });
    let mut levels = Vec::with_capacity(p.levels + 1);
    levels.push(LevelBlocks { mesh: base, error: 0.0, maps: None });
    levels.extend(coarse);
    ChunkLevels { levels }
}
