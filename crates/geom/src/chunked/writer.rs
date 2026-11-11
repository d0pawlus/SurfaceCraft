//! Appends finished chunks to the data files of a store and collects the index entries. Generic
//! over the sink, so the same code writes `BufWriter<File>`s (the on-disk store) or `Vec<u8>`s (an
//! in-memory store).
use super::format::{BlockRef, ChunkEntry, FileKind, Span, checksum, encode_map_block, encode_mesh_block};
use super::levels::ChunkLevels;
use super::{StoreError, corrupt};
use std::collections::HashMap;
use std::io::Write;

/// Writes the blocks of chunks, in order, to one sink per data file.
pub struct StoreWriter<W: Write> {
    sinks: HashMap<FileKind, (W, u64)>,
    has_maps: bool,
}

impl<W: Write> StoreWriter<W> {
    /// A writer for `level_count` coarse levels; `open` makes the sink of each data file.
    pub fn new(
        level_count: usize,
        has_maps: bool,
        mut open: impl FnMut(FileKind) -> Result<W, StoreError>,
    ) -> Result<Self, StoreError> {
        let mut sinks = HashMap::new();
        for l in 0..=level_count {
            sinks.insert(FileKind::Mesh(l), (open(FileKind::Mesh(l))?, 0));
            if l > 0 && has_maps {
                sinks.insert(FileKind::Map(l), (open(FileKind::Map(l))?, 0));
            }
        }
        Ok(Self { sinks, has_maps })
    }

    fn put(&mut self, kind: FileKind, bytes: &[u8]) -> Result<Span, StoreError> {
        let (w, at) = self.sinks.get_mut(&kind).ok_or_else(|| corrupt("writer has no sink for this file"))?;
        w.write_all(bytes).map_err(|e| StoreError::Io(format!("{}: {e}", kind.file_name())))?;
        let span = Span { offset: *at, len: bytes.len() as u64, checksum: checksum(bytes) };
        *at += bytes.len() as u64;
        Ok(span)
    }

    /// Appends all levels of a chunk and returns its index entry.
    pub fn append(&mut self, chunk: &ChunkLevels) -> Result<ChunkEntry, StoreError> {
        let mut levels = Vec::with_capacity(chunk.levels.len());
        for (l, lv) in chunk.levels.iter().enumerate() {
            let mesh = self.put(FileKind::Mesh(l), &encode_mesh_block(&lv.mesh))?;
            let map = match (&lv.maps, self.has_maps) {
                (Some((vm, tm)), true) => self.put(FileKind::Map(l), &encode_map_block(vm, tm))?,
                _ => Span::default(),
            };
            levels.push(BlockRef {
                vertices: u32::try_from(lv.mesh.positions.len())
                    .map_err(|_| StoreError::Limit("vertex count".into()))?,
                triangles: u32::try_from(lv.mesh.triangles.len())
                    .map_err(|_| StoreError::Limit("triangle count".into()))?,
                error: lv.error,
                mesh,
                map,
            });
        }
        Ok(ChunkEntry { bbox: chunk.bbox(), levels })
    }

    /// Hands back the sinks (to flush them or take their bytes).
    #[must_use]
    pub fn into_sinks(self) -> Vec<(FileKind, W)> {
        self.sinks.into_iter().map(|(k, (w, _))| (k, w)).collect()
    }
}
