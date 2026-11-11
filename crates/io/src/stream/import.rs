//! A mesh file into a chunk store on disk, streaming: the file is read in batches, several times,
//! and never held (see `surfacecraft_geom::chunked::build` for the memory bound).
use super::{StreamFormat, StreamOptions, detect, for_each_batch};
use crate::IoError;
use crate::mesh::{Limits, MeshFormat};
use std::fs::File;
use std::path::Path;
use surfacecraft_geom::chunked::StoreParams;
use surfacecraft_geom::chunked::build::{BuildReport, Sink, build_store};

/// How a file is imported into a store.
#[derive(Clone, Debug)]
pub struct StreamImportOptions {
    /// File format; `None` = from the extension, else sniffed.
    pub format: Option<MeshFormat>,
    /// Scale applied to every coordinate (e.g. 25.4 for inches); must be positive and finite.
    pub scale: f64,
    /// How the store is cut.
    pub store: StoreParams,
    /// Replace the files of an earlier store in the directory.
    pub overwrite: bool,
    /// Longest polygon of a PLY face (fan-triangulated).
    pub max_polygon: usize,
    /// Most vertices of a PLY file.
    pub max_vertices: usize,
}

impl Default for StreamImportOptions {
    fn default() -> Self {
        let l = Limits::default();
        Self {
            format: None,
            scale: 1.0,
            store: StoreParams::default(),
            overwrite: false,
            max_polygon: l.max_polygon,
            max_vertices: usize::MAX,
        }
    }
}

/// The streaming format of the file at `path`.
fn format_of(path: &Path, hint: Option<MeshFormat>) -> Result<StreamFormat, IoError> {
    let ext = path.extension().and_then(|e| e.to_str()).and_then(MeshFormat::from_extension);
    let mut f = File::open(path)
        .map_err(|e| IoError::Malformed { format: "mesh", message: format!("cannot open {}: {e}", path.display()) })?;
    detect(&mut f, hint.or(ext))
}

/// Imports the mesh file `path` into the store directory `dir`.
pub fn import_to_store(path: &Path, dir: &Path, opts: &StreamImportOptions) -> Result<BuildReport, IoError> {
    if !(opts.scale.is_finite() && opts.scale > 0.0) {
        return Err(IoError::Malformed { format: "mesh", message: "scale must be positive".into() });
    }
    let format = format_of(path, opts.format)?;
    let limits = Limits {
        max_triangles: usize::try_from(opts.store.max_triangles).unwrap_or(usize::MAX),
        max_vertices: opts.max_vertices,
        max_polygon: opts.max_polygon,
    };
    let stream = StreamOptions { limits, scale: opts.scale };
    let open = || File::open(path);
    let mut replay = |sink: Sink<'_>| -> Result<(), IoError> {
        for_each_batch(&open, format, &stream, &mut |b| sink(b).map_err(IoError::from)).map(|_| ())
    };
    build_store(dir, &opts.store, opts.overwrite, &mut replay)
}
