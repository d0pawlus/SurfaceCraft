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
//! **Memory bound.** Peak memory is `memory_bound_bytes(params)`: the histogram and the cell→leaf
//! table (`8 bytes × cells`, cells ≤ 2²¹), the spill buffers (`72 bytes × 2¹⁸` triangles, twice for
//! `Vec` slack), one chunk at a time (`chunk_triangles ×` [`CHUNK_WORK_BYTES_PER_TRIANGLE`]) and the index entries
//! (`O(triangles / chunk_triangles)`). None of it grows with the number of triangles, only the index
//! does, by a few hundred bytes per chunk. The spill files and the data files live in the directory
//! the caller gave; spill files are deleted as soon as they are read, and on failure everything the
//! build wrote is removed.
use super::assemble::{Scan, centroid, changed, level_params, make_index};
use super::format::{INDEX_FILE, StoreSummary};
use super::levels::{build_levels, finalize_soup};
use super::plan::{CellGrid, Partition};
use super::spill::{SPILL_BUFFER_TRIANGLES, SPILL_TRIANGLE_BYTES, Spill, take_spill};
use super::writer::StoreWriter;
use super::{StoreError, StoreParams, disk::DiskBlocks, view::Chunked};
use crate::{Aabb, Mesh, Point3};
use std::collections::HashMap;
use std::fs::File;
use std::io::{BufWriter, Write};
use std::path::Path;

/// A destination for triangles: the replay calls it with batches.
pub type Sink<'a> = &'a mut dyn FnMut(&[[Point3; 3]]) -> Result<(), StoreError>;

/// Working memory of finishing one chunk, per triangle of the chunk. The structures themselves (soup,
/// welded mesh, weld keys, the coarse levels with their hash maps and maps) add up to well under
/// 1 KB per triangle; the constant is empirical and also covers allocator retention and the levels
/// being built on several threads: a full 65 536-triangle chunk peaked at 243 MB resident for the
/// whole process (`perf_store_scale`, 4 M triangles), which is below the bound this gives.
pub const CHUNK_WORK_BYTES_PER_TRIANGLE: usize = 4096;

/// Upper bound of the memory a build needs, bytes (see the module docs).
#[must_use]
pub fn memory_bound_bytes(params: &StoreParams) -> usize {
    let grid = super::plan::MAX_GRID_CELLS * 8;
    // the buffers are `Vec`s that grow by doubling: allow twice their contents
    let spill = 2 * SPILL_BUFFER_TRIANGLES * SPILL_TRIANGLE_BYTES;
    grid + spill + params.chunk_triangles * CHUNK_WORK_BYTES_PER_TRIANGLE
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
    prepare_dir(dir, overwrite)?;
    let result = run(dir, params, replay);
    if result.is_err() {
        remove_store_files(dir);
    }
    result
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
    let spill = route(dir, &part, replay)?;
    let stats = BuildStats {
        peak_buffered_triangles: spill.peak_buffered,
        grid_cells: grid.cell_count(),
        leaves: part.leaf_counts.len(),
        heavy_cells: part.heavy.len(),
        spill_flushes: spill.flushes,
        ..BuildStats::default()
    };
    Ok(finish(dir, params, &scan, &part, stats)?)
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
    stats.bytes_on_disk = dir_bytes(dir);
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

/// Is `name` a file this module writes?
fn is_store_file(name: &str) -> bool {
    let numbered = |prefix: &str, suffix: &str| {
        name.strip_prefix(prefix)
            .and_then(|r| r.strip_suffix(suffix))
            .is_some_and(|n| !n.is_empty() && n.bytes().all(|b| b.is_ascii_digit()))
    };
    name == INDEX_FILE || numbered("mesh-", ".dat") || numbered("map-", ".dat") || numbered("spill-", ".tmp")
}

fn prepare_dir(dir: &Path, overwrite: bool) -> Result<(), StoreError> {
    let io = |e: std::io::Error| StoreError::Io(format!("{}: {e}", dir.display()));
    if !dir.exists() {
        return std::fs::create_dir_all(dir).map_err(io);
    }
    let names: Vec<String> = std::fs::read_dir(dir)
        .map_err(io)?
        .filter_map(|e| e.ok().map(|e| e.file_name().to_string_lossy().into_owned()))
        .collect();
    if names.is_empty() {
        return Ok(());
    }
    if !overwrite {
        return Err(StoreError::Params(format!(
            "{} is not empty; pass overwrite to replace a store there",
            dir.display()
        )));
    }
    if let Some(other) = names.iter().find(|n| !is_store_file(n)) {
        return Err(StoreError::Params(format!(
            "{} holds {other}, which is not a store file; nothing was deleted",
            dir.display()
        )));
    }
    remove_store_files(dir);
    Ok(())
}

/// Removes the files of a store (best effort), nothing else.
fn remove_store_files(dir: &Path) {
    let Ok(rd) = std::fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        if is_store_file(&e.file_name().to_string_lossy()) {
            let _ = std::fs::remove_file(e.path());
        }
    }
}

fn dir_bytes(dir: &Path) -> u64 {
    let Ok(rd) = std::fs::read_dir(dir) else { return 0 };
    rd.flatten()
        .filter(|e| is_store_file(&e.file_name().to_string_lossy()))
        .filter_map(|e| e.metadata().ok())
        .map(|m| m.len())
        .sum()
}

#[cfg(test)]
mod name_tests {
    use super::*;
    use crate::chunked::spill::spill_name;

    #[test]
    fn only_our_own_file_names_count_as_store_files() {
        assert!(is_store_file("index.scx") && is_store_file("mesh-0.dat") && is_store_file("map-12.dat"));
        assert!(is_store_file(&spill_name(3)));
        for other in ["notes.txt", "mesh-.dat", "mesh-x.dat", "mesh-1.dat.bak", "index.scx.old", "spill-1.tmp.x"] {
            assert!(!is_store_file(other), "{other}");
        }
    }
}
