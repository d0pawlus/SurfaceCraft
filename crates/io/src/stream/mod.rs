//! Streaming reads of large mesh files (P8-G1): triangles come out in small batches, so a file of
//! any size is read with memory that does not depend on it. Binary and ASCII STL, and binary PLY
//! (vertices fetched from the file through a small page cache, so a PLY needs no vertex array in
//! memory). Everything read is hostile, as in [`crate::mesh`]: counts are checked against the file
//! length, indices against the vertex count, and every coordinate must be finite.
//!
//! A reader needs `Read + Seek` and a way to open the file again (`open`), because the chunk store
//! build reads its input more than once and a PLY needs a second handle for vertex lookups.
//! [`import`] (not on wasm) feeds a file into the chunk store of `surfacecraft_geom::chunked`.
use crate::mesh::{Limits, MeshFormat};
use crate::{IoError, bad};
use std::io::{Read, Seek, SeekFrom};
use surfacecraft_geom::Point3;

mod ply;
mod stl;

#[cfg(not(target_arch = "wasm32"))]
pub mod import;

/// Triangles per batch handed to the sink.
pub const BATCH_TRIANGLES: usize = 4096;
/// Bytes read from the start of a file to recognise its format.
const SNIFF_BYTES: usize = 512;

/// Receives batches of triangles.
pub type TriSink<'a> = &'a mut dyn FnMut(&[[Point3; 3]]) -> Result<(), IoError>;

/// The formats that can be streamed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StreamFormat {
    /// Binary STL.
    StlBinary,
    /// ASCII STL.
    StlAscii,
    /// Binary PLY (little or big endian).
    PlyBinary,
}

/// What a stream applies to the triangles it reads.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StreamOptions {
    /// Hard limits (triangles emitted, vertices, polygon corners).
    pub limits: Limits,
    /// Factor applied to every coordinate (1 = none).
    pub scale: f64,
}

/// Opens the input afresh.
pub type Open<'a, R> = &'a dyn Fn() -> std::io::Result<R>;

/// Collects triangles into batches, applying the scale and the limits.
pub(crate) struct Batcher<'a> {
    buf: Vec<[Point3; 3]>,
    sink: TriSink<'a>,
    /// Triangles emitted so far.
    pub(crate) count: u64,
    opts: StreamOptions,
}

impl<'a> Batcher<'a> {
    pub(crate) fn new(sink: TriSink<'a>, opts: StreamOptions) -> Self {
        Self { buf: Vec::with_capacity(BATCH_TRIANGLES), sink, count: 0, opts }
    }

    pub(crate) fn push(&mut self, t: [Point3; 3]) -> Result<(), IoError> {
        let s = self.opts.scale;
        let t = t.map(|p| Point3::from(p.coords * s));
        if !t.iter().all(surfacecraft_geom::finite) {
            return Err(bad("mesh", format!("triangle {} has a non-finite coordinate (after scaling)", self.count)));
        }
        if self.count >= self.opts.limits.max_triangles as u64 {
            return Err(IoError::Limit(format!("more than {} triangles", self.opts.limits.max_triangles)));
        }
        self.buf.push(t);
        self.count += 1;
        if self.buf.len() >= BATCH_TRIANGLES { self.flush() } else { Ok(()) }
    }

    /// Refuses a declared item count over the triangle limit before anything is read.
    pub(crate) fn check_declared(&self, items: usize) -> Result<(), IoError> {
        if items > self.opts.limits.max_triangles {
            return Err(IoError::Limit(format!(
                "{items} items declared, over the limit of {}",
                self.opts.limits.max_triangles
            )));
        }
        Ok(())
    }

    pub(crate) fn flush(&mut self) -> Result<(), IoError> {
        if self.buf.is_empty() {
            return Ok(());
        }
        (self.sink)(&self.buf)?;
        self.buf.clear();
        Ok(())
    }
}

pub(crate) fn io_err(e: &std::io::Error) -> IoError {
    bad("mesh", format!("read failed: {e}"))
}

/// Decides how to stream a file: `hint` (from the extension) or the leading bytes pick STL or PLY.
pub fn detect<R: Read + Seek>(r: &mut R, hint: Option<MeshFormat>) -> Result<StreamFormat, IoError> {
    let len = r.seek(SeekFrom::End(0)).map_err(|e| io_err(&e))?;
    r.seek(SeekFrom::Start(0)).map_err(|e| io_err(&e))?;
    let mut head = Vec::with_capacity(SNIFF_BYTES);
    r.by_ref().take(SNIFF_BYTES as u64).read_to_end(&mut head).map_err(|e| io_err(&e))?;
    let format = hint.or_else(|| MeshFormat::sniff(&head));
    match format {
        Some(MeshFormat::Ply) => Ok(StreamFormat::PlyBinary),
        Some(MeshFormat::Stl) => {
            Ok(if stl::is_binary(&head, len) { StreamFormat::StlBinary } else { StreamFormat::StlAscii })
        }
        Some(MeshFormat::Obj | MeshFormat::Xyz) => Err(IoError::Unsupported(
            "streaming import reads STL and binary PLY; use mesh.import for OBJ and point lists".into(),
        )),
        None => Err(IoError::Unsupported("cannot tell the mesh format; pass `format`".into())),
    }
}

/// Reads the whole input through `sink` in batches; returns the number of triangles emitted.
pub fn for_each_batch<R: Read + Seek>(
    open: Open<'_, R>,
    format: StreamFormat,
    opts: &StreamOptions,
    sink: TriSink<'_>,
) -> Result<u64, IoError> {
    let mut batcher = Batcher::new(sink, *opts);
    match format {
        StreamFormat::StlBinary => stl::read_binary(open().map_err(|e| io_err(&e))?, &mut batcher)?,
        StreamFormat::StlAscii => stl::read_ascii(open().map_err(|e| io_err(&e))?, &mut batcher)?,
        StreamFormat::PlyBinary => ply::read_binary(open, &opts.limits, &mut batcher)?,
    }
    batcher.flush()?;
    Ok(batcher.count)
}
