//! Reading a chunk store: [`Chunked`] over any [`BlockStore`] (files on disk, or bytes in memory),
//! chunk by chunk or as one [`PointSource`] / [`TriangleSource`] per level.
use super::format::{FileKind, Span, StoreIndex, StoreSummary, checksum, decode_map_block, decode_mesh_block};
use super::{StoreError, corrupt};
use crate::{Aabb, Mesh, Point3, PointSource, TriangleSource};
use std::collections::HashMap;
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};

/// Decoded chunk meshes kept for random access through the source traits.
const CACHE_ENTRIES: usize = 8;

/// Where the bytes of the data files come from.
pub trait BlockStore: Send + Sync {
    /// Length of a data file.
    fn len(&self, kind: FileKind) -> Result<u64, StoreError>;
    /// `len` bytes of a data file starting at `offset`; fewer bytes available is an error.
    fn read(&self, kind: FileKind, offset: u64, len: usize) -> Result<Vec<u8>, StoreError>;
}

/// An in-memory [`BlockStore`] (tests, small meshes, callers that chunk a mesh they already hold).
#[derive(Clone, Debug, Default)]
pub struct MemBlocks {
    files: HashMap<FileKind, Vec<u8>>,
}

impl MemBlocks {
    /// From the sinks of a [`StoreWriter`](super::writer::StoreWriter) over `Vec<u8>`.
    #[must_use]
    pub fn from_sinks(sinks: Vec<(FileKind, Vec<u8>)>) -> Self {
        Self { files: sinks.into_iter().collect() }
    }

    /// Mutable access to the bytes of a file (to damage them in tests).
    pub fn file_mut(&mut self, kind: FileKind) -> Option<&mut Vec<u8>> {
        self.files.get_mut(&kind)
    }
}

impl BlockStore for MemBlocks {
    fn len(&self, kind: FileKind) -> Result<u64, StoreError> {
        self.files.get(&kind).map(|f| f.len() as u64).ok_or_else(|| corrupt(format!("{} is missing", kind.file_name())))
    }

    fn read(&self, kind: FileKind, offset: u64, len: usize) -> Result<Vec<u8>, StoreError> {
        let f = self.files.get(&kind).ok_or_else(|| corrupt(format!("{} is missing", kind.file_name())))?;
        let start = usize::try_from(offset).map_err(|_| corrupt("offset overflow"))?;
        let end = start.checked_add(len).ok_or_else(|| corrupt("length overflow"))?;
        f.get(start..end)
            .map(<[u8]>::to_vec)
            .ok_or_else(|| corrupt(format!("{} is shorter than the index says", kind.file_name())))
    }
}

/// The maps of a coarse level of one chunk.
#[derive(Clone, Debug, PartialEq)]
pub struct LevelMaps {
    /// Level-0 vertex → vertex of this level (chunk-local ids).
    pub vertex_map: Vec<u32>,
    /// Level-0 triangle → triangle of this level (`u32::MAX` = collapsed), chunk-local.
    pub triangle_map: Vec<u32>,
}

/// What [`Chunked::verify`] read.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VerifyReport {
    /// Chunks.
    pub chunks: usize,
    /// Blocks read and checked.
    pub blocks: usize,
    /// Bytes read.
    pub bytes: u64,
}

#[derive(Default)]
struct Cache {
    /// Most recently used last.
    entries: Vec<((usize, usize), Arc<Mesh>)>,
}

/// A chunked mesh: an index plus the blocks it points at.
pub struct Chunked<P: BlockStore> {
    index: StoreIndex,
    blocks: P,
    vertex_offsets: Vec<Vec<u64>>,
    triangle_offsets: Vec<Vec<u64>>,
    cache: Mutex<Cache>,
    first_error: Mutex<Option<StoreError>>,
}

fn lock<T>(m: &Mutex<T>) -> MutexGuard<'_, T> {
    m.lock().unwrap_or_else(PoisonError::into_inner)
}

impl<P: BlockStore> Chunked<P> {
    /// Checks the index against the data files (every span in range, files exactly as long as the
    /// index accounts for, vertex ids of every level fitting `u32`) and wraps them.
    pub fn new(index: StoreIndex, blocks: P) -> Result<Self, StoreError> {
        index.check_layout(&|k| blocks.len(k))?;
        let levels = index.levels.len() + 1;
        let mut vertex_offsets = Vec::with_capacity(levels);
        let mut triangle_offsets = Vec::with_capacity(levels);
        for l in 0..levels {
            let (mut vo, mut to) = (vec![0u64], vec![0u64]);
            for c in &index.chunks {
                let b = c.levels.get(l).ok_or_else(|| corrupt("chunk without its levels"))?;
                vo.push(vo.last().copied().unwrap_or(0) + u64::from(b.vertices));
                to.push(to.last().copied().unwrap_or(0) + u64::from(b.triangles));
            }
            if vo.last().copied().unwrap_or(0) > u64::from(u32::MAX) {
                return Err(StoreError::Limit(format!("level {l} has more vertices than a u32 index addresses")));
            }
            vertex_offsets.push(vo);
            triangle_offsets.push(to);
        }
        Ok(Self {
            index,
            blocks,
            vertex_offsets,
            triangle_offsets,
            cache: Mutex::new(Cache::default()),
            first_error: Mutex::new(None),
        })
    }

    /// The index.
    #[must_use]
    pub fn index(&self) -> &StoreIndex {
        &self.index
    }

    /// The block store.
    #[must_use]
    pub fn blocks(&self) -> &P {
        &self.blocks
    }

    /// Takes the store apart again (to damage or copy it in tests).
    #[must_use]
    pub fn into_parts(self) -> (StoreIndex, P) {
        (self.index, self.blocks)
    }

    /// Totals per level.
    #[must_use]
    pub fn summary(&self) -> StoreSummary {
        self.index.summary()
    }

    /// Number of chunks.
    #[must_use]
    pub fn chunk_count(&self) -> usize {
        self.index.chunks.len()
    }

    /// Number of levels including full resolution.
    #[must_use]
    pub fn level_count(&self) -> usize {
        self.index.levels.len() + 1
    }

    /// Bounding box of a chunk.
    #[must_use]
    pub fn chunk_bbox(&self, chunk: usize) -> Option<Aabb> {
        self.index.chunks.get(chunk).map(|c| c.bbox)
    }

    /// The first error a [`PointSource`] / [`TriangleSource`] call swallowed (those return `None`
    /// on a failed read); taking it clears it.
    pub fn take_error(&self) -> Option<StoreError> {
        lock(&self.first_error).take()
    }

    fn note(&self, e: StoreError) {
        lock(&self.first_error).get_or_insert(e);
    }

    fn block_bytes(&self, kind: FileKind, span: &Span) -> Result<Vec<u8>, StoreError> {
        let len = usize::try_from(span.len).map_err(|_| corrupt("block length overflow"))?;
        let bytes = self.blocks.read(kind, span.offset, len)?;
        if checksum(&bytes) != span.checksum {
            return Err(corrupt(format!("{}: checksum mismatch at byte {}", kind.file_name(), span.offset)));
        }
        Ok(bytes)
    }

    fn load_mesh(&self, chunk: usize, level: usize) -> Result<Mesh, StoreError> {
        let b = self
            .index
            .chunks
            .get(chunk)
            .and_then(|c| c.levels.get(level))
            .ok_or_else(|| StoreError::Params(format!("no chunk {chunk} at level {level}")))?;
        let bytes = self.block_bytes(FileKind::Mesh(level), &b.mesh)?;
        decode_mesh_block(&bytes, b.vertices as usize, b.triangles as usize)
    }

    /// The mesh of `chunk` at `level` (checksum and structure verified, kept in a small cache).
    pub fn read_mesh(&self, chunk: usize, level: usize) -> Result<Arc<Mesh>, StoreError> {
        {
            let mut cache = lock(&self.cache);
            if let Some(pos) = cache.entries.iter().position(|(k, _)| *k == (chunk, level)) {
                let hit = cache.entries.remove(pos);
                let mesh = Arc::clone(&hit.1);
                cache.entries.push(hit);
                return Ok(mesh);
            }
        }
        let mesh = Arc::new(self.load_mesh(chunk, level)?);
        let mut cache = lock(&self.cache);
        if cache.entries.len() >= CACHE_ENTRIES {
            cache.entries.remove(0);
        }
        cache.entries.push(((chunk, level), Arc::clone(&mesh)));
        Ok(mesh)
    }

    /// The vertex and triangle maps of `chunk` for coarse `level ≥ 1` (stores built with `keep_maps`).
    pub fn read_maps(&self, chunk: usize, level: usize) -> Result<LevelMaps, StoreError> {
        if level == 0 || !self.index.has_maps {
            return Err(StoreError::Params("maps exist for coarse levels of stores built with keep_maps".into()));
        }
        let entry = self.index.chunks.get(chunk).ok_or_else(|| StoreError::Params(format!("no chunk {chunk}")))?;
        let (base, b) = entry
            .levels
            .first()
            .zip(entry.levels.get(level))
            .ok_or_else(|| StoreError::Params(format!("no level {level}")))?;
        let bytes = self.block_bytes(FileKind::Map(level), &b.map)?;
        let (vertex_map, triangle_map) = decode_map_block(
            &bytes,
            (base.vertices as usize, base.triangles as usize),
            (b.vertices as usize, b.triangles as usize),
        )?;
        Ok(LevelMaps { vertex_map, triangle_map })
    }

    /// Reads and checks every block of the store.
    pub fn verify(&self) -> Result<VerifyReport, StoreError> {
        let mut rep = VerifyReport { chunks: self.chunk_count(), ..VerifyReport::default() };
        for (ci, c) in self.index.chunks.iter().enumerate() {
            let mesh0 = self.load_mesh(ci, 0)?;
            for (l, b) in c.levels.iter().enumerate() {
                if l > 0 {
                    self.load_mesh(ci, l)?;
                    rep.blocks += 1;
                    rep.bytes += b.mesh.len;
                    if self.index.has_maps {
                        self.read_maps(ci, l)?;
                        rep.blocks += 1;
                        rep.bytes += b.map.len;
                    }
                }
            }
            rep.blocks += 1;
            rep.bytes += c.levels.first().map_or(0, |b| b.mesh.len);
            if mesh0.bbox() != c.bbox {
                return Err(corrupt(format!("chunk {ci}: bounding box does not match its vertices")));
            }
        }
        Ok(rep)
    }

    /// One level of the whole store as a single mesh (global numbering of this module), refused
    /// when it would hold more than `max_triangles`.
    pub fn to_mesh(&self, level: usize, max_triangles: usize) -> Result<Mesh, StoreError> {
        let total = self
            .triangle_offsets
            .get(level)
            .and_then(|o| o.last())
            .copied()
            .ok_or_else(|| StoreError::Params(format!("no level {level}")))?;
        if total > max_triangles as u64 {
            return Err(StoreError::Limit(format!(
                "level {level} has {total} triangles, over the limit of {max_triangles}"
            )));
        }
        let mut out = Mesh::default();
        for c in 0..self.chunk_count() {
            let m = self.read_mesh(c, level)?;
            let base = out.positions.len() as u32;
            out.positions.extend_from_slice(&m.positions);
            out.triangles.extend(m.triangles.iter().map(|t| [t[0] + base, t[1] + base, t[2] + base]));
        }
        Ok(out)
    }

    /// One level as a point and triangle source.
    #[must_use]
    pub fn level(&self, level: usize) -> Option<LevelView<'_, P>> {
        (level < self.level_count()).then_some(LevelView { store: self, level })
    }
}

/// One level of a [`Chunked`] as a [`PointSource`] and [`TriangleSource`] with the global numbering
/// of the module docs. A read that fails (I/O error, damaged block) makes the call return `None`
/// and is remembered for [`Chunked::take_error`]; [`Chunked::verify`] finds damage up front, and
/// [`Chunked::read_mesh`] per chunk is the path for bulk work.
pub struct LevelView<'a, P: BlockStore> {
    store: &'a Chunked<P>,
    level: usize,
}

/// `(chunk, local id)` of the global id `i` for prefix sums `offsets` (`chunks + 1` of them).
fn locate(offsets: &[u64], i: usize) -> Option<(usize, usize)> {
    let i = i as u64;
    let chunk = offsets.partition_point(|&o| o <= i).checked_sub(1)?;
    let start = *offsets.get(chunk)?;
    (chunk + 1 < offsets.len()).then_some((chunk, (i - start) as usize))
}

impl<P: BlockStore> LevelView<'_, P> {
    fn mesh_of(&self, chunk: usize) -> Option<Arc<Mesh>> {
        self.store.read_mesh(chunk, self.level).map_err(|e| self.store.note(e)).ok()
    }
}

impl<P: BlockStore> PointSource for LevelView<'_, P> {
    fn point_count(&self) -> usize {
        self.store.vertex_offsets.get(self.level).and_then(|o| o.last()).map_or(0, |&n| n as usize)
    }

    fn point_at(&self, i: usize) -> Option<Point3> {
        let (chunk, local) = locate(self.store.vertex_offsets.get(self.level)?, i)?;
        self.mesh_of(chunk)?.positions.get(local).copied()
    }
}

impl<P: BlockStore> TriangleSource for LevelView<'_, P> {
    fn triangle_count_of(&self) -> usize {
        self.store.triangle_offsets.get(self.level).and_then(|o| o.last()).map_or(0, |&n| n as usize)
    }

    fn triangle_indices_at(&self, i: usize) -> Option<[u32; 3]> {
        let (chunk, local) = locate(self.store.triangle_offsets.get(self.level)?, i)?;
        let t = self.mesh_of(chunk)?.triangles.get(local).copied()?;
        let base = u32::try_from(*self.store.vertex_offsets.get(self.level)?.get(chunk)?).ok()?;
        Some([t[0].checked_add(base)?, t[1].checked_add(base)?, t[2].checked_add(base)?])
    }
}

/// Level 0 of the store.
impl<P: BlockStore> PointSource for Chunked<P> {
    fn point_count(&self) -> usize {
        self.level(0).map_or(0, |v| v.point_count())
    }

    fn point_at(&self, i: usize) -> Option<Point3> {
        self.level(0)?.point_at(i)
    }
}

impl<P: BlockStore> TriangleSource for Chunked<P> {
    fn triangle_count_of(&self) -> usize {
        self.level(0).map_or(0, |v| v.triangle_count_of())
    }

    fn triangle_indices_at(&self, i: usize) -> Option<[u32; 3]> {
        self.level(0)?.triangle_indices_at(i)
    }
}
