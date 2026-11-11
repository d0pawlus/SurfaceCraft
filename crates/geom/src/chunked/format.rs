//! Byte layout of the chunk store: blocks, index, checksum. Pure functions on byte slices (no file
//! access), so a damaged or hostile store can be fuzzed without a disk and the same code serves an
//! in-memory store. Everything read here is hostile: counts are checked against the bytes present
//! before anything is allocated, and every arithmetic step is checked.
use super::{MAX_CHUNK_TRIANGLES, MAX_CHUNKS, MAX_LEVELS, StoreError, corrupt};
use crate::{Aabb, Mesh, Point3};

/// Version of the layout described in [`super`].
pub const FORMAT_VERSION: u32 = 1;
/// File name of the index.
pub const INDEX_FILE: &str = "index.scx";
const INDEX_MAGIC: [u8; 4] = *b"SCIX";
const HEADER_BYTES: usize = 4 + 4 * 4 + 3 * 8 + 6 * 8 + 3 * 8;
const BBOX_BYTES: usize = 6 * 8;
const BLOCK_REF_BYTES: usize = 4 + 4 + 8 + 2 * SPAN_BYTES;
const SPAN_BYTES: usize = 3 * 8;
const SUM_BYTES: usize = 8;
/// Bytes of one vertex (three `f64`) and one triangle (three `u32`) in a mesh block.
pub const VERTEX_BYTES: usize = 24;
/// Bytes of one triangle in a mesh block.
pub const TRIANGLE_BYTES: usize = 12;
/// Slack on the stated error bound when a stored measurement is checked against it.
const BOUND_SLACK: f64 = 1e-9;

/// A 64-bit integrity checksum (FNV-1a style over 8-byte words with an extra xor-shift, so one
/// changed bit always changes the result). Detects damage; it is not a security hash.
#[must_use]
pub fn checksum(bytes: &[u8]) -> u64 {
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    const BASIS: u64 = 0xcbf2_9ce4_8422_2325;
    let mut h = BASIS;
    let (words, rest) = bytes.as_chunks::<8>();
    for w in words {
        h = (h ^ u64::from_le_bytes(*w)).wrapping_mul(PRIME);
        h ^= h >> 29;
    }
    for &b in rest {
        h = (h ^ u64::from(b)).wrapping_mul(PRIME);
    }
    (h ^ bytes.len() as u64).wrapping_mul(PRIME)
}

/// Which data file of the store.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum FileKind {
    /// `mesh-<level>.dat`.
    Mesh(usize),
    /// `map-<level>.dat` (levels ≥ 1).
    Map(usize),
}

impl FileKind {
    /// The file name inside the store directory.
    #[must_use]
    pub fn file_name(self) -> String {
        match self {
            FileKind::Mesh(l) => format!("mesh-{l}.dat"),
            FileKind::Map(l) => format!("map-{l}.dat"),
        }
    }
}

/// A byte range of a data file with the checksum of its contents.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Span {
    /// First byte.
    pub offset: u64,
    /// Length in bytes.
    pub len: u64,
    /// [`checksum`] of the bytes.
    pub checksum: u64,
}

/// One chunk at one level: counts, measured error and where its blocks are.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct BlockRef {
    /// Vertices at this level.
    pub vertices: u32,
    /// Triangles at this level.
    pub triangles: u32,
    /// Measured worst distance of a level-0 vertex of the chunk from its coarse vertex (0 at level 0).
    pub error: f64,
    /// The mesh block.
    pub mesh: Span,
    /// The map block (empty at level 0 and when the store keeps no maps).
    pub map: Span,
}

/// One chunk: its bounding box and its blocks per level (`levels[0]` is full resolution).
#[derive(Clone, Debug, PartialEq)]
pub struct ChunkEntry {
    /// Tight box around the vertices of the chunk.
    pub bbox: Aabb,
    /// One entry per level, `level_count + 1` of them.
    pub levels: Vec<BlockRef>,
}

/// A coarse level of the store.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LevelInfo {
    /// Cell size `h` of the clustering grid, mm.
    pub cell_size: f64,
    /// Stated error bound `√3·h`, mm.
    pub error_bound: f64,
}

/// The index of a store.
#[derive(Clone, Debug, PartialEq)]
pub struct StoreIndex {
    /// Chunk capacity the store was cut with.
    pub chunk_triangles: u32,
    /// Does the store keep vertex and triangle maps for the coarse levels?
    pub has_maps: bool,
    /// Triangles read from the source, before degenerate ones were dropped.
    pub source_triangles: u64,
    /// Total area of the stored surface, mm².
    pub area: f64,
    /// Bounding box of the stored surface.
    pub bbox: Aabb,
    /// Origin of the clustering grid.
    pub grid_origin: Point3,
    /// Coarse levels `1..=n`.
    pub levels: Vec<LevelInfo>,
    /// The chunks in store order.
    pub chunks: Vec<ChunkEntry>,
}

/// Totals of one level.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LevelSummary {
    /// Level number (0 = full resolution).
    pub level: usize,
    /// Grid cell size (0 at level 0).
    pub cell_size: f64,
    /// Stated error bound (0 at level 0).
    pub error_bound: f64,
    /// Largest measured error over all chunks.
    pub max_error: f64,
    /// Vertices over all chunks.
    pub vertices: u64,
    /// Triangles over all chunks.
    pub triangles: u64,
}

/// What an index says about the whole store.
#[derive(Clone, Debug, PartialEq)]
pub struct StoreSummary {
    /// Chunks.
    pub chunks: usize,
    /// Chunk capacity.
    pub chunk_triangles: u32,
    /// Triangles read from the source.
    pub source_triangles: u64,
    /// Bounding box.
    pub bbox: Aabb,
    /// Area, mm².
    pub area: f64,
    /// Maps kept?
    pub has_maps: bool,
    /// One entry per level, level 0 first.
    pub levels: Vec<LevelSummary>,
}

impl StoreIndex {
    /// Totals per level.
    #[must_use]
    pub fn summary(&self) -> StoreSummary {
        let levels = (0..=self.levels.len())
            .map(|l| {
                let info = l.checked_sub(1).and_then(|i| self.levels.get(i));
                let mut s = LevelSummary {
                    level: l,
                    cell_size: info.map_or(0.0, |i| i.cell_size),
                    error_bound: info.map_or(0.0, |i| i.error_bound),
                    max_error: 0.0,
                    vertices: 0,
                    triangles: 0,
                };
                for b in self.chunks.iter().filter_map(|c| c.levels.get(l)) {
                    s.max_error = s.max_error.max(b.error);
                    s.vertices += u64::from(b.vertices);
                    s.triangles += u64::from(b.triangles);
                }
                s
            })
            .collect();
        StoreSummary {
            chunks: self.chunks.len(),
            chunk_triangles: self.chunk_triangles,
            source_triangles: self.source_triangles,
            bbox: self.bbox,
            area: self.area,
            has_maps: self.has_maps,
            levels,
        }
    }

    /// Serialises the index (with its trailing checksum).
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        let mut o = Vec::with_capacity(HEADER_BYTES + self.chunks.len() * 400);
        o.extend_from_slice(&INDEX_MAGIC);
        put_u32(&mut o, FORMAT_VERSION);
        put_u32(&mut o, self.chunk_triangles);
        put_u32(&mut o, self.levels.len() as u32);
        put_u32(&mut o, u32::from(self.has_maps));
        put_u64(&mut o, self.chunks.len() as u64);
        put_u64(&mut o, self.source_triangles);
        put_f64(&mut o, self.area);
        put_box(&mut o, &self.bbox);
        for c in self.grid_origin.iter() {
            put_f64(&mut o, *c);
        }
        for l in &self.levels {
            put_f64(&mut o, l.cell_size);
            put_f64(&mut o, l.error_bound);
        }
        for c in &self.chunks {
            put_box(&mut o, &c.bbox);
            for b in &c.levels {
                put_u32(&mut o, b.vertices);
                put_u32(&mut o, b.triangles);
                put_f64(&mut o, b.error);
                put_span(&mut o, &b.mesh);
                put_span(&mut o, &b.map);
            }
        }
        let sum = checksum(&o);
        put_u64(&mut o, sum);
        o
    }

    /// Parses and validates an index.
    pub fn decode(bytes: &[u8]) -> Result<Self, StoreError> {
        let body_len = bytes.len().checked_sub(SUM_BYTES).filter(|&n| n >= HEADER_BYTES);
        let Some(body_len) = body_len else { return Err(corrupt("index is shorter than its header")) };
        let (body, tail) = bytes.split_at(body_len);
        let mut tr = Reader::new(tail);
        if tr.u64()? != checksum(body) {
            return Err(corrupt("index checksum does not match"));
        }
        let mut r = Reader::new(body);
        if r.take(4)? != INDEX_MAGIC {
            return Err(corrupt("not a chunk store index (bad magic)"));
        }
        let version = r.u32()?;
        if version != FORMAT_VERSION {
            return Err(corrupt(format!("store format version {version}, this build reads {FORMAT_VERSION}")));
        }
        let (mut idx, declared) = read_header(&mut r)?;
        let level_count = idx.levels.len();
        let record = BBOX_BYTES + (level_count + 1) * BLOCK_REF_BYTES;
        let expected = declared.checked_mul(record).and_then(|n| n.checked_add(r.at()));
        if expected != Some(body.len()) {
            return Err(corrupt("index length does not match its chunk count"));
        }
        idx.chunks.reserve(declared);
        for _ in 0..declared {
            idx.chunks.push(read_chunk(&mut r, level_count, &idx)?);
        }
        Ok(idx)
    }
}

fn put_u32(o: &mut Vec<u8>, v: u32) {
    o.extend_from_slice(&v.to_le_bytes());
}
fn put_u64(o: &mut Vec<u8>, v: u64) {
    o.extend_from_slice(&v.to_le_bytes());
}
fn put_f64(o: &mut Vec<u8>, v: f64) {
    o.extend_from_slice(&v.to_le_bytes());
}
fn put_box(o: &mut Vec<u8>, b: &Aabb) {
    for c in b.min.iter().chain(b.max.iter()) {
        put_f64(o, *c);
    }
}
fn put_span(o: &mut Vec<u8>, s: &Span) {
    put_u64(o, s.offset);
    put_u64(o, s.len);
    put_u64(o, s.checksum);
}

/// A cursor over hostile bytes: every read is bounds-checked.
pub(crate) struct Reader<'a> {
    b: &'a [u8],
    at: usize,
}

impl<'a> Reader<'a> {
    pub(crate) fn new(b: &'a [u8]) -> Self {
        Self { b, at: 0 }
    }
    fn at(&self) -> usize {
        self.at
    }
    fn take(&mut self, n: usize) -> Result<&'a [u8], StoreError> {
        let end = self.at.checked_add(n).ok_or_else(|| corrupt("length overflow"))?;
        let s = self.b.get(self.at..end).ok_or_else(|| corrupt("truncated"))?;
        self.at = end;
        Ok(s)
    }
    fn u32(&mut self) -> Result<u32, StoreError> {
        let a = <[u8; 4]>::try_from(self.take(4)?).map_err(|_| corrupt("truncated"))?;
        Ok(u32::from_le_bytes(a))
    }
    fn u64(&mut self) -> Result<u64, StoreError> {
        let a = <[u8; 8]>::try_from(self.take(8)?).map_err(|_| corrupt("truncated"))?;
        Ok(u64::from_le_bytes(a))
    }
    fn f64(&mut self) -> Result<f64, StoreError> {
        self.u64().map(f64::from_bits)
    }
    fn finite(&mut self, what: &str) -> Result<f64, StoreError> {
        let v = self.f64()?;
        if v.is_finite() { Ok(v) } else { Err(corrupt(format!("{what} is not finite"))) }
    }
    fn span(&mut self) -> Result<Span, StoreError> {
        Ok(Span { offset: self.u64()?, len: self.u64()?, checksum: self.u64()? })
    }
    fn bbox(&mut self) -> Result<Aabb, StoreError> {
        let mut v = [0.0; 6];
        for x in &mut v {
            *x = self.finite("bounding box")?;
        }
        let b = Aabb { min: Point3::new(v[0], v[1], v[2]), max: Point3::new(v[3], v[4], v[5]) };
        if b.is_empty() { Err(corrupt("empty bounding box")) } else { Ok(b) }
    }
}

fn read_header(r: &mut Reader<'_>) -> Result<(StoreIndex, usize), StoreError> {
    let chunk_triangles = r.u32()?;
    let level_count = r.u32()? as usize;
    let has_maps = r.u32()?;
    let chunk_count = r.u64()?;
    if chunk_triangles == 0 || chunk_triangles as usize > MAX_CHUNK_TRIANGLES {
        return Err(corrupt(format!("chunk capacity {chunk_triangles} out of range")));
    }
    if level_count > MAX_LEVELS || has_maps > 1 {
        return Err(corrupt("level count or map flag out of range"));
    }
    if chunk_count > MAX_CHUNKS as u64 {
        return Err(corrupt(format!("{chunk_count} chunks exceed the limit of {MAX_CHUNKS}")));
    }
    let source_triangles = r.u64()?;
    let area = r.finite("area")?;
    if chunk_count == 0 {
        return Err(corrupt("store has no chunks"));
    }
    let bbox = r.bbox()?;
    let grid_origin = Point3::new(r.finite("grid origin")?, r.finite("grid origin")?, r.finite("grid origin")?);
    let mut levels = Vec::with_capacity(level_count);
    for _ in 0..level_count {
        let cell_size = r.finite("cell size")?;
        let error_bound = r.finite("error bound")?;
        if cell_size <= 0.0 || error_bound < 0.0 {
            return Err(corrupt("level cell size or error bound out of range"));
        }
        levels.push(LevelInfo { cell_size, error_bound });
    }
    let idx = StoreIndex {
        chunk_triangles,
        has_maps: has_maps == 1,
        source_triangles,
        area,
        bbox,
        grid_origin,
        levels,
        chunks: Vec::new(),
    };
    Ok((idx, chunk_count as usize))
}

fn read_chunk(r: &mut Reader<'_>, level_count: usize, idx: &StoreIndex) -> Result<ChunkEntry, StoreError> {
    let bbox = r.bbox()?;
    let mut levels: Vec<BlockRef> = Vec::with_capacity(level_count + 1);
    for l in 0..=level_count {
        let b = BlockRef {
            vertices: r.u32()?,
            triangles: r.u32()?,
            error: r.finite("block error")?,
            mesh: r.span()?,
            map: r.span()?,
        };
        check_block(&b, l, levels.first(), idx)?;
        levels.push(b);
    }
    Ok(ChunkEntry { bbox, levels })
}

fn check_block(b: &BlockRef, level: usize, base: Option<&BlockRef>, idx: &StoreIndex) -> Result<(), StoreError> {
    let (v, t) = (b.vertices as usize, b.triangles as usize);
    let mesh_len = v.checked_mul(VERTEX_BYTES).zip(t.checked_mul(TRIANGLE_BYTES)).and_then(|(a, c)| a.checked_add(c));
    if mesh_len.map(|n| n as u64) != Some(b.mesh.len) {
        return Err(corrupt(format!("level {level}: mesh block length does not match its counts")));
    }
    let Some(base) = base else {
        // level 0 of a chunk
        let cap = idx.chunk_triangles as usize;
        if t == 0 || t > cap || v == 0 || v > 3 * t || b.error != 0.0 || b.map != Span::default() {
            return Err(corrupt("level 0 block has impossible counts"));
        }
        return Ok(());
    };
    let bound = idx.levels.get(level - 1).map_or(0.0, |l| l.error_bound);
    if v > base.vertices as usize
        || t > base.triangles as usize
        || b.error < 0.0
        || b.error > bound * (1.0 + BOUND_SLACK)
    {
        return Err(corrupt(format!("level {level}: counts or error exceed what level 0 allows")));
    }
    let want_map = if idx.has_maps { (base.vertices as u64 + u64::from(base.triangles)) * 4 } else { 0 };
    if b.map.len != want_map || (want_map == 0 && b.map != Span::default()) {
        return Err(corrupt(format!("level {level}: map block length does not match level 0")));
    }
    Ok(())
}

impl StoreIndex {
    /// Checks that, in every data file, the spans of the chunks follow each other from offset 0 and
    /// end exactly at the length of the file (`file_len`), so nothing is missing or left over.
    pub fn check_layout(&self, file_len: &dyn Fn(FileKind) -> Result<u64, StoreError>) -> Result<(), StoreError> {
        for l in 0..=self.levels.len() {
            let mut kinds = vec![FileKind::Mesh(l)];
            if l > 0 && self.has_maps {
                kinds.push(FileKind::Map(l));
            }
            for kind in kinds {
                let mut next = 0u64;
                for c in &self.chunks {
                    let Some(b) = c.levels.get(l) else { return Err(corrupt("chunk without its levels")) };
                    let span = if matches!(kind, FileKind::Mesh(_)) { b.mesh } else { b.map };
                    if span.offset != next {
                        return Err(corrupt(format!("{}: blocks are not contiguous", kind.file_name())));
                    }
                    next = next.checked_add(span.len).ok_or_else(|| corrupt("span overflow"))?;
                }
                let have = file_len(kind)?;
                if have != next {
                    return Err(corrupt(format!(
                        "{} holds {have} bytes, the index accounts for {next}",
                        kind.file_name()
                    )));
                }
            }
        }
        Ok(())
    }
}

/// Serialises a mesh block.
#[must_use]
pub fn encode_mesh_block(m: &Mesh) -> Vec<u8> {
    let mut o = Vec::with_capacity(m.positions.len() * VERTEX_BYTES + m.triangles.len() * TRIANGLE_BYTES);
    for p in &m.positions {
        for c in p.iter() {
            put_f64(&mut o, *c);
        }
    }
    for t in &m.triangles {
        for i in t {
            put_u32(&mut o, *i);
        }
    }
    o
}

/// Parses a mesh block of `vertices` and `triangles`: exact length, finite coordinates, indices in range.
pub fn decode_mesh_block(bytes: &[u8], vertices: usize, triangles: usize) -> Result<Mesh, StoreError> {
    let want = vertices
        .checked_mul(VERTEX_BYTES)
        .zip(triangles.checked_mul(TRIANGLE_BYTES))
        .and_then(|(a, b)| a.checked_add(b));
    if want != Some(bytes.len()) {
        return Err(corrupt("mesh block length does not match its counts"));
    }
    let mut r = Reader::new(bytes);
    let mut positions = Vec::with_capacity(vertices);
    for _ in 0..vertices {
        positions.push(Point3::new(r.f64()?, r.f64()?, r.f64()?));
    }
    let mut tris = Vec::with_capacity(triangles);
    for _ in 0..triangles {
        tris.push([r.u32()?, r.u32()?, r.u32()?]);
    }
    let m = Mesh { positions, triangles: tris };
    m.validate().map_err(|e| corrupt(format!("mesh block: {e}")))?;
    Ok(m)
}

/// Serialises a map block: the vertex map then the triangle map.
#[must_use]
pub fn encode_map_block(vertex_map: &[u32], triangle_map: &[u32]) -> Vec<u8> {
    let mut o = Vec::with_capacity((vertex_map.len() + triangle_map.len()) * 4);
    for v in vertex_map.iter().chain(triangle_map) {
        put_u32(&mut o, *v);
    }
    o
}

/// Parses a map block for a chunk of `vertices` / `triangles` at level 0 into a level of
/// `coarse_vertices` / `coarse_triangles`; every entry must point into that level.
pub fn decode_map_block(
    bytes: &[u8],
    (vertices, triangles): (usize, usize),
    (coarse_vertices, coarse_triangles): (usize, usize),
) -> Result<(Vec<u32>, Vec<u32>), StoreError> {
    let want = vertices.checked_add(triangles).and_then(|n| n.checked_mul(4));
    if want != Some(bytes.len()) {
        return Err(corrupt("map block length does not match its counts"));
    }
    let mut r = Reader::new(bytes);
    let mut vm = Vec::with_capacity(vertices);
    for _ in 0..vertices {
        let v = r.u32()?;
        if v as usize >= coarse_vertices {
            return Err(corrupt("vertex map points past the level"));
        }
        vm.push(v);
    }
    let mut tm = Vec::with_capacity(triangles);
    for _ in 0..triangles {
        let t = r.u32()?;
        if t != u32::MAX && t as usize >= coarse_triangles {
            return Err(corrupt("triangle map points past the level"));
        }
        tm.push(t);
    }
    Ok((vm, tm))
}
