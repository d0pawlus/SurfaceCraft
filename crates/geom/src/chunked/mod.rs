//! Chunked meshes and the on-disk chunk store (P8-G1, design doc §9: out-of-core).
//!
//! A scan too large for memory is cut into *chunks* of at most a few ten thousand triangles that are
//! spatially coherent, each with its own bounding box, welded vertices and a pyramid of coarser
//! levels. The whole is addressed through a small index and read back chunk by chunk, or as one
//! [`PointSource`](crate::PointSource) / [`TriangleSource`](crate::TriangleSource) ([`Chunked`]).
//!
//! # Rules (format version 1)
//!
//! * **Ownership.** Every triangle belongs to exactly one chunk: the one whose cell of the
//!   partition grid holds its centroid. Chunks are ordered along the depth-first order of the
//!   partition (left before right), so neighbouring chunk ids are near each other in space.
//! * **Welding.** Vertices are welded *inside* a chunk only (`Mesh::weld(0.0)`: the same
//!   `1e-9 mm` grid key as the in-memory import), unreferenced vertices are dropped, and triangles
//!   that collapse or have zero area are removed (as `read_mesh` does). Across chunks nothing is
//!   merged: a vertex on a seam is stored once per chunk that uses it, with bit-identical
//!   coordinates, so two copies can be matched by their position.
//! * **Numbering.** Level `l` of the store numbers its vertices (and triangles) chunk after chunk:
//!   the global id of a vertex is the sum of the vertex counts of the earlier chunks plus its local
//!   id; triangle ids likewise. Counts must fit `u32`.
//! * **Levels.** Level 0 is the full resolution. Level `l ≥ 1` is built from level 0 by vertex
//!   clustering ([`cluster_on_grid`](crate::lod::cluster_on_grid)) on one grid that is global to
//!   the store (origin at the bounding-box minimum, cell size `h_l = h_1 · 2^(l-1)` with `h_1`
//!   twice the mean edge length), so levels are independent (errors do not add up) and clusters line
//!   up across chunks. Stated bound: every level-0 vertex lies within `√3·h_l` of its coarse
//!   vertex and every point of the coarse surface within `√3·h_l` of the full surface; the index
//!   also holds the measured worst case per chunk. Coarse vertices are cluster means *per chunk*, so
//!   a coarse level can open hairline cracks at chunk seams, as wide as twice the stated bound.
//! * **Maps.** With `keep_maps` the store keeps, per level, `vertex_map` (level-0 vertex → coarse
//!   vertex) and `triangle_map` (level-0 triangle → coarse triangle, `u32::MAX` when it collapsed),
//!   chunk-local, so per-vertex colours (deviation maps, regions) carry to a level without
//!   recomputing.
//!
//! # Files in the store directory
//!
//! `index.scx` (written last: its presence means the store is complete), `mesh-<l>.dat` for every
//! level and `map-<l>.dat` for every level ≥ 1. A mesh block is `f64 × 3` per vertex then `u32 × 3`
//! per triangle (little endian); a map block is `u32 × vertices` then `u32 × triangles` of level 0.
//! The index lists per chunk and level the counts, the measured error and the byte span and
//! checksum of each block, and ends with a checksum of its own, so a damaged store is an error and
//! never a silently wrong surface.
use thiserror::Error;

mod assemble;
pub mod format;
pub mod levels;
pub mod plan;
pub mod view;
pub mod writer;

#[cfg(not(target_arch = "wasm32"))]
pub mod build;
#[cfg(not(target_arch = "wasm32"))]
pub mod disk;
#[cfg(not(target_arch = "wasm32"))]
pub mod spill;

pub use assemble::chunk_in_memory;
pub use format::{BlockRef, ChunkEntry, FileKind, LevelInfo, Span, StoreIndex, StoreSummary};
pub use levels::{ChunkLevels, LevelBlocks, LevelParams};
pub use view::{BlockStore, Chunked, LevelMaps, LevelView, MemBlocks, VerifyReport};
pub use writer::StoreWriter;

/// Default triangles per chunk.
pub const DEFAULT_CHUNK_TRIANGLES: usize = 65_536;
/// Smallest allowed chunk size.
pub const MIN_CHUNK_TRIANGLES: usize = 64;
/// Largest allowed chunk size.
pub const MAX_CHUNK_TRIANGLES: usize = 1 << 22;
/// Default number of coarse levels above full resolution.
pub const DEFAULT_LEVELS: usize = 5;
/// Most coarse levels.
pub const MAX_LEVELS: usize = 16;
/// Most chunks a store may hold (a hostile index cannot ask for more).
pub const MAX_CHUNKS: usize = 1 << 24;
/// Most triangles a store may hold.
pub const MAX_STORE_TRIANGLES: u64 = 4_000_000_000;

/// Why a store could not be built or read.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum StoreError {
    /// The file system refused.
    #[error("chunk store i/o: {0}")]
    Io(String),
    /// The store on disk is damaged or not a store of this version.
    #[error("damaged chunk store: {0}")]
    Corrupt(String),
    /// A hard limit was exceeded.
    #[error("{0}")]
    Limit(String),
    /// A parameter is out of range.
    #[error("{0}")]
    Params(String),
    /// The input mesh is unusable (non-finite coordinate, changed between passes).
    #[error("{0}")]
    Input(String),
}

pub(crate) fn corrupt(msg: impl Into<String>) -> StoreError {
    StoreError::Corrupt(msg.into())
}

/// How a store is cut.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StoreParams {
    /// Most triangles in a chunk (also the memory unit of the build).
    pub chunk_triangles: usize,
    /// Coarse levels above full resolution.
    pub levels: usize,
    /// Keep the level-0 → level-`l` vertex and triangle maps.
    pub keep_maps: bool,
    /// Refuse inputs with more triangles than this.
    pub max_triangles: u64,
}

impl Default for StoreParams {
    fn default() -> Self {
        Self {
            chunk_triangles: DEFAULT_CHUNK_TRIANGLES,
            levels: DEFAULT_LEVELS,
            keep_maps: true,
            max_triangles: MAX_STORE_TRIANGLES,
        }
    }
}

impl StoreParams {
    /// Checks the ranges.
    pub fn validate(&self) -> Result<(), StoreError> {
        if !(MIN_CHUNK_TRIANGLES..=MAX_CHUNK_TRIANGLES).contains(&self.chunk_triangles) {
            return Err(StoreError::Params(format!(
                "chunk_triangles must be {MIN_CHUNK_TRIANGLES} to {MAX_CHUNK_TRIANGLES}, got {}",
                self.chunk_triangles
            )));
        }
        if self.levels > MAX_LEVELS {
            return Err(StoreError::Params(format!("levels must be at most {MAX_LEVELS}, got {}", self.levels)));
        }
        if self.max_triangles == 0 || self.max_triangles > MAX_STORE_TRIANGLES {
            return Err(StoreError::Params(format!("max_triangles must be 1 to {MAX_STORE_TRIANGLES}")));
        }
        Ok(())
    }
}
