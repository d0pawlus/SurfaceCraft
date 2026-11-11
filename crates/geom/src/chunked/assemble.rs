//! Pieces shared by the on-disk build and the in-memory one, and the in-memory build itself
//! ([`chunk_in_memory`]: chunk a mesh that is already in memory, for tests and for callers that
//! want the chunked view without a disk).
use super::format::{ChunkEntry, LevelInfo, StoreIndex};
use super::levels::{LevelParams, base_cell_size, build_levels, cell_size, error_bound, finalize_soup};
use super::plan::{CellGrid, Partition};
use super::view::{Chunked, MemBlocks};
use super::writer::StoreWriter;
use super::{StoreError, StoreParams};
use crate::{Aabb, Mesh, Point3, tri};
use std::collections::HashMap;

/// Pass 1 of a build: bounding box, count and area of the input.
pub(crate) struct Scan {
    pub(crate) bbox: Aabb,
    pub(crate) triangles: u64,
    pub(crate) area: f64,
    max: u64,
}

impl Scan {
    pub(crate) fn new(max_triangles: u64) -> Self {
        Self { bbox: Aabb::empty(), triangles: 0, area: 0.0, max: max_triangles }
    }

    pub(crate) fn add(&mut self, batch: &[[Point3; 3]]) -> Result<(), StoreError> {
        for t in batch {
            if !t.iter().all(crate::finite) {
                return Err(StoreError::Input("a triangle has a non-finite coordinate".into()));
            }
            t.iter().for_each(|p| self.bbox.grow(p));
            self.area += tri::area(&t[0], &t[1], &t[2]);
            self.triangles += 1;
        }
        if self.triangles > self.max {
            return Err(StoreError::Limit(format!("more than {} triangles", self.max)));
        }
        Ok(())
    }
}

pub(crate) fn centroid(t: &[Point3; 3]) -> Point3 {
    Point3::from((t[0].coords + t[1].coords + t[2].coords) / 3.0)
}

pub(crate) fn changed() -> StoreError {
    StoreError::Input("the input changed while it was being imported (the passes disagree)".into())
}

pub(crate) fn level_params(params: &StoreParams, scan: &Scan) -> LevelParams {
    LevelParams {
        grid_origin: scan.bbox.min,
        base_cell: base_cell_size(scan.area, scan.triangles, &scan.bbox),
        levels: params.levels,
        keep_maps: params.keep_maps,
    }
}

pub(crate) fn make_index(
    params: &StoreParams,
    scan: &Scan,
    lp: &LevelParams,
    chunks: Vec<ChunkEntry>,
    area: f64,
    bbox: Aabb,
) -> StoreIndex {
    let levels = (1..=params.levels)
        .map(|l| {
            let cell = cell_size(lp.base_cell, l);
            LevelInfo { cell_size: cell, error_bound: error_bound(cell) }
        })
        .collect();
    StoreIndex {
        chunk_triangles: params.chunk_triangles as u32,
        has_maps: params.keep_maps,
        source_triangles: scan.triangles,
        area,
        bbox,
        grid_origin: scan.bbox.min,
        levels,
        chunks,
    }
}

/// Chunks a mesh held in memory with the same partition, welding and levels as the disk build.
pub fn chunk_in_memory(mesh: &Mesh, params: &StoreParams) -> Result<Chunked<MemBlocks>, StoreError> {
    params.validate()?;
    let soup: Vec<[Point3; 3]> = (0..mesh.triangle_count()).filter_map(|i| mesh.triangle(i)).collect();
    let mut scan = Scan::new(params.max_triangles);
    scan.add(&soup)?;
    if soup.is_empty() {
        return Err(StoreError::Input("the mesh has no triangles".into()));
    }
    let grid = CellGrid::new(&scan.bbox)?;
    let mut counts = vec![0u32; grid.cell_count()];
    for t in &soup {
        if let Some(c) = counts.get_mut(grid.cell_of(&centroid(t))) {
            *c += 1;
        }
    }
    let part = Partition::plan(grid, &counts, params.chunk_triangles as u32);
    let mut leaves: Vec<Vec<[Point3; 3]>> = vec![Vec::new(); part.leaf_counts.len()];
    let mut seen = HashMap::new();
    for t in &soup {
        let leaf = part.leaf_for(grid.cell_of(&centroid(t)), &mut seen).ok_or_else(changed)?;
        leaves.get_mut(leaf as usize).ok_or_else(changed)?.push(*t);
    }
    let lp = level_params(params, &scan);
    let mut writer = StoreWriter::new(params.levels, params.keep_maps, |_| Ok(Vec::new()))?;
    let (mut chunks, mut bbox, mut area) = (Vec::new(), Aabb::empty(), 0.0);
    for leaf in leaves {
        let base = finalize_soup(Mesh {
            triangles: (0..leaf.len() as u32).map(|i| [3 * i, 3 * i + 1, 3 * i + 2]).collect(),
            positions: leaf.into_iter().flatten().collect(),
        });
        if base.triangles.is_empty() {
            continue;
        }
        area += base.area();
        let entry = writer.append(&build_levels(base, &lp))?;
        bbox.grow(&entry.bbox.min);
        bbox.grow(&entry.bbox.max);
        chunks.push(entry);
    }
    if chunks.is_empty() {
        return Err(StoreError::Input("every triangle of the mesh is degenerate".into()));
    }
    let index = make_index(params, &scan, &lp, chunks, area, bbox);
    Chunked::new(index, MemBlocks::from_sinks(writer.into_sinks()))
}
