//! Building a chunk store from a stream of triangles without ever holding the mesh.
//!
//! The caller supplies a *replay*: a function that pushes every triangle of the input through a sink
//! and can be called several times (a file is simply read again). The build makes four passes:
//!
//! 1. bounding box, triangle count, area (a few numbers);
//! 2. a histogram of triangle centroids on a grid of at most [`MAX_GRID_CELLS`](super::plan::MAX_GRID_CELLS)
//!    cells, split into leaves of at most one chunk of triangles ([`Partition`]);
//! 3. every triangle is routed to its leaf and parked in a spill file, at most
//!    [`SPILL_BUFFER_TRIANGLES`] triangles buffered;
//! 4. leaf by leaf: read the spill, weld, build the levels, append to the data files.
//!
//! **Memory bound.** Peak memory is `memory_bound_bytes(params)` (formula there, from the structures in
//! this module and `levels`; one factor is measured and labelled so): the histogram and the cell→leaf
//! table (`8 bytes × cells`, cells ≤ 2²¹), the spill buffers (`72 bytes × 2¹⁸` triangles), one chunk
//! at a time (`chunk_triangles × (struct + levels × level bytes)`) and the index entries
//! (`O(triangles / chunk_triangles)`). None of it grows with the number of triangles, only the index
//! does, by a few hundred bytes per chunk.
//!
//! **Disk.** The build writes into `<dir>.building` next to the target and moves the files into the
//! target only when it succeeded, so an earlier store survives a failed import. Peak disk use is the
//! store plus the spill files, `72 bytes ×` triangles (`spill_disk_bytes`), which are deleted as they
//! are read; on failure the work directory is removed.
use super::assemble::{Scan, centroid, changed, level_params, make_index};
use super::format::{INDEX_FILE, MAX_INDEX_BYTES, StoreSummary, projected_index_bytes};
use super::levels::{build_levels, finalize_soup};
use super::plan::{CellGrid, Partition};
use super::spill::{SPILL_BUFFER_TRIANGLES, SPILL_TRIANGLE_BYTES, Spill, take_spill};
use super::writer::StoreWriter;
use super::{MAX_CHUNKS, StoreError, StoreParams, disk::DiskBlocks, target, view::Chunked};
use crate::{Aabb, Mesh, Point3};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// A destination for triangles: the replay calls it with batches.
pub type Sink<'a> = &'a mut dyn FnMut(&[[Point3; 3]]) -> Result<(), StoreError>;

/// Bytes of one spilled triangle on disk (nine `f64`).
pub const SPILL_DISK_BYTES_PER_TRIANGLE: u64 = SPILL_TRIANGLE_BYTES as u64;
/// Bytes per chunk triangle of the structures alive while a chunk is finished at full resolution:
/// the soup (72 B), its indexed mesh (84 B), the weld keys and remaps (about 130 B), the welded mesh
/// (about 25 B), rounded up.
pub const CHUNK_STRUCT_BYTES_PER_TRIANGLE: usize = 400;
/// Bytes per chunk triangle of one coarse level while it is built: the clustering hash maps (about
/// 40 B per vertex, one vertex per two triangles), the maps (6 B) and the level itself, rounded up.
pub const LEVEL_BYTES_PER_TRIANGLE: usize = 80;
/// Factor between the structures above and the resident memory seen: allocator retention and the
/// levels of a chunk being built on several threads. MEASURED, not derived: a full 65 536-triangle
/// chunk with 5 levels peaked at 243 MB resident for the whole process (`perf_store_scale`, 4 M
/// triangles) against 52 MB of structures, so 4 is the smallest round factor above that.
pub const ALLOCATOR_FACTOR: usize = 4;

/// Upper bound of the memory a build needs, bytes, as a function of the parameters only:
/// the grid (histogram plus cell table, 8 B per cell, at most 2^21 cells), the spill buffers
/// (`Vec`s that grow by doubling, so twice 2^18 triangles of 72 B), and one chunk at a time:
/// `ALLOCATOR_FACTOR × chunk_triangles × (CHUNK_STRUCT + levels × LEVEL)` bytes. With the
/// command's limits (`chunk_triangles` ≤ 2^18, `levels` ≤ 8) the bound is under 1.1 GB; at the
/// defaults (65 536, 5) it is 254 MiB.
#[must_use]
pub fn memory_bound_bytes(params: &StoreParams) -> usize {
    let grid = super::plan::MAX_GRID_CELLS * 8;
    let spill = 2 * SPILL_BUFFER_TRIANGLES * SPILL_TRIANGLE_BYTES;
    let per_triangle = CHUNK_STRUCT_BYTES_PER_TRIANGLE + params.levels * LEVEL_BYTES_PER_TRIANGLE;
    grid + spill + ALLOCATOR_FACTOR * params.chunk_triangles * per_triangle
}

/// Disk the spill files take at their peak, bytes: every triangle is parked once, 72 B each, before
/// the chunks are written (on top of the store itself).
#[must_use]
pub fn spill_disk_bytes(triangles: u64) -> u64 {
    triangles.saturating_mul(SPILL_DISK_BYTES_PER_TRIANGLE)
}

/// What the build measured about itself.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct BuildStats {
    /// Most triangles buffered in memory at once (≤ [`SPILL_BUFFER_TRIANGLES`]).
    pub peak_buffered_triangles: usize,
    /// Most triangles of any chunk before welding.
    pub max_chunk_triangles: usize,
    /// Cells of the partition grid.
    pub grid_cells: usize,
    /// Leaves of the partition.
    pub leaves: usize,
    /// Cells cut by input order because one cell held more than a chunk.
    pub heavy_cells: usize,
    /// Times the spill buffers were flushed.
    pub spill_flushes: usize,
    /// Disk the spill files took at their peak (72 B per triangle), deleted again.
    pub peak_spill_bytes: u64,
    /// Bytes of all store files.
    pub bytes_on_disk: u64,
}

/// The result of a build.
#[derive(Clone, Debug, PartialEq)]
pub struct BuildReport {
    /// What the store holds.
    pub summary: StoreSummary,
    /// How the build went.
    pub stats: BuildStats,
}

/// Builds the store in `dir`. `dir` must not exist or be empty; with `overwrite` the files of an
/// earlier store (and only those: `index.scx`, `mesh-*.dat`, `map-*.dat`, `spill-*.tmp`) are removed
/// first. `replay` pushes the whole input through the sink it is given, and must give the same
/// triangles every time it is called.
pub fn build_store<E: From<StoreError>>(
    dir: &Path,
    params: &StoreParams,
    overwrite: bool,
    replay: &mut dyn FnMut(Sink<'_>) -> Result<(), E>,
) -> Result<BuildReport, E> {
    params.validate()?;
    let work = target::prepare(dir, overwrite)?;
    let result = run(&work, params, replay);
    match result {
        Ok(mut report) => {
            target::install(&work, dir)?;
            report.stats.bytes_on_disk = target::dir_bytes(dir);
            Ok(report)
        }
        Err(e) => {
            target::discard(&work);
            Err(e)
        }
    }
}

fn run<E: From<StoreError>>(
    dir: &Path,
    params: &StoreParams,
    replay: &mut dyn FnMut(Sink<'_>) -> Result<(), E>,
) -> Result<BuildReport, E> {
    let mut scan = Scan::new(params.max_triangles);
    replay(&mut |b| scan.add(b))?;
    if scan.triangles == 0 {
        return Err(StoreError::Input("the input has no triangles".into()).into());
    }
    let grid = CellGrid::new(&scan.bbox)?;
    let mut counts = vec![0u32; grid.cell_count()];
    let mut seen_total = 0u64;
    replay(&mut |b| {
        for t in b {
            let c = counts.get_mut(grid.cell_of(&centroid(t))).ok_or_else(changed)?;
            *c = c.saturating_add(1);
            seen_total += 1;
        }
        Ok(())
    })?;
    if seen_total != scan.triangles {
        return Err(changed().into());
    }
    let part = Partition::plan(grid, &counts, params.chunk_triangles as u32);
    drop(counts);
    check_projection(&part, params)?;
    let spill = route(dir, &part, replay)?;
    let stats = BuildStats {
        peak_buffered_triangles: spill.peak_buffered,
        grid_cells: grid.cell_count(),
        leaves: part.leaf_counts.len(),
        heavy_cells: part.heavy.len(),
        spill_flushes: spill.flushes,
        peak_spill_bytes: spill_disk_bytes(scan.triangles),
        ..BuildStats::default()
    };
    Ok(finish(dir, params, &scan, &part, stats)?)
}

/// Refuses, before any triangle is spilled, what the reader would refuse later: too many chunks, an
/// index over the size limit. (The vertex-id limit is enforced by `MAX_STORE_TRIANGLES`, three
/// vertices per triangle at most, while the input is counted.)
fn check_projection(part: &Partition, params: &StoreParams) -> Result<(), StoreError> {
    let leaves = part.leaf_counts.len();
    if leaves > MAX_CHUNKS {
        return Err(StoreError::Limit(format!(
            "the input would make {leaves} chunks, over the limit of {MAX_CHUNKS}; use a larger chunk_triangles"
        )));
    }
    let bytes = projected_index_bytes(leaves, params.levels);
    if bytes > MAX_INDEX_BYTES {
        return Err(StoreError::Limit(format!(
            "the index would take {bytes} bytes, over the limit of {MAX_INDEX_BYTES}; use a larger chunk_triangles"
        )));
    }
    Ok(())
}

fn route<E: From<StoreError>>(
    dir: &Path,
    part: &Partition,
    replay: &mut dyn FnMut(Sink<'_>) -> Result<(), E>,
) -> Result<Spill, E> {
    let mut spill = Spill::new(dir, part.leaf_counts.len());
    let mut heavy_seen: HashMap<u32, u64> = HashMap::new();
    replay(&mut |b| {
        for t in b {
            let cell = part.grid.cell_of(&centroid(t));
            let leaf = part.leaf_for(cell, &mut heavy_seen).ok_or_else(changed)?;
            spill.push(leaf as usize, t)?;
        }
        Ok(())
    })?;
    spill.finish()?;
    if spill.routed != part.leaf_counts {
        return Err(changed().into());
    }
    Ok(spill)
}

fn finish(
    dir: &Path,
    params: &StoreParams,
    scan: &Scan,
    part: &Partition,
    mut stats: BuildStats,
) -> Result<BuildReport, StoreError> {
    let lp = level_params(params, scan);
    let io = |p: &Path, e: std::io::Error| StoreError::Io(format!("{}: {e}", p.display()));
    let mut writer = StoreWriter::new(params.levels, params.keep_maps, |kind| {
        let path = dir.join(kind.file_name());
        File::create(&path).map(BufWriter::new).map_err(|e| io(&path, e))
    })?;
    let (mut chunks, mut bbox, mut area) = (Vec::new(), Aabb::empty(), 0.0);
    for (leaf, &count) in part.leaf_counts.iter().enumerate() {
        let soup = take_spill(dir, leaf, count as usize)?;
        stats.max_chunk_triangles = stats.max_chunk_triangles.max(soup.len());
        let base = finalize_soup(soup_mesh(&soup));
        drop(soup);
        if base.triangles.is_empty() {
            continue;
        }
        area += base.area();
        let levels = build_levels(base, &lp);
        let entry = writer.append(&levels)?;
        bbox.grow(&entry.bbox.min);
        bbox.grow(&entry.bbox.max);
        chunks.push(entry);
    }
    for (kind, mut w) in writer.into_sinks() {
        w.flush().map_err(|e| io(&dir.join(kind.file_name()), e))?;
    }
    if chunks.is_empty() {
        return Err(StoreError::Input("every triangle of the input is degenerate".into()));
    }
    let index = make_index(params, scan, &lp, chunks, area, bbox);
    // the index is only written once it agrees with the files, and last
    let store = Chunked::new(index, DiskBlocks::new(dir))?;
    let index_path = dir.join(INDEX_FILE);
    std::fs::write(&index_path, store.index().encode()).map_err(|e| io(&index_path, e))?;
    stats.bytes_on_disk = target::dir_bytes(dir);
    Ok(BuildReport { summary: store.summary(), stats })
}

fn soup_mesh(soup: &[[Point3; 3]]) -> Mesh {
    let mut positions = Vec::with_capacity(soup.len() * 3);
    let mut triangles = Vec::with_capacity(soup.len());
    for t in soup {
        let i = positions.len() as u32;
        positions.extend_from_slice(t);
        triangles.push([i, i + 1, i + 2]);
    }
    Mesh { positions, triangles }
}
