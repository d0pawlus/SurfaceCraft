//! `mesh.import_stream`, `mesh.store_info`: meshes too big for memory (P8-G1, design doc §9).
//! A scan is read from its file in batches into a chunk store on disk (spatially coherent chunks
//! with bounding boxes and a level-of-detail pyramid, `surfacecraft_geom::chunked`); the store is
//! the input of the out-of-core work that follows (display from LOD, deviation, alignment).
use super::{CmdError, CmdResult, CommandSpec};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::LengthUnit;
#[cfg(not(target_arch = "wasm32"))]
use surfacecraft_geom::chunked::{StoreParams, StoreSummary};
use surfacecraft_io::MeshFormat;

use super::mesh::size_check;

/// Parameters of `mesh.import_stream`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImportStreamIn {
    /// Mesh file: binary or ASCII STL, or binary PLY (read in batches, never held in memory).
    pub path: String,
    /// Directory of the chunk store: created, or empty (or `overwrite`).
    pub store_dir: String,
    /// `stl` or `ply` (default: from the extension, else sniffed).
    #[serde(default)]
    pub format: Option<MeshFormat>,
    /// Factor on import, times `units`. Default 1.
    #[serde(default)]
    pub scale: Option<f64>,
    /// Unit of the file's coordinates, as for `mesh.import`: the store is written in millimetres.
    #[serde(default)]
    pub units: Option<LengthUnit>,
    /// Triangles per chunk, 64 - 262144 (default 65536). Also the memory unit of the import: the
    /// import needs about 54 MiB + 4 x chunk_triangles x (400 + 80 x levels) bytes (252 MiB at the
    /// defaults, at most about 1.1 GB), whatever the size of the file; the spill files take 72
    /// bytes per triangle of disk while it runs.
    #[serde(default)]
    #[schemars(range(min = 64, max = 262144))]
    pub chunk_triangles: Option<usize>,
    /// Coarse levels of detail above full resolution, 0 - 8 (default 5).
    #[serde(default)]
    #[schemars(range(min = 0, max = 8))]
    pub levels: Option<usize>,
    /// Keep per-level vertex and triangle maps, so per-vertex data carries to a level (default true).
    #[serde(default)]
    pub keep_maps: Option<bool>,
    /// Replace the files of an earlier store in `store_dir` (nothing else is ever deleted).
    #[serde(default)]
    pub overwrite: Option<bool>,
}

/// Parameters of `mesh.store_info`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct StoreInfoIn {
    /// Directory of the chunk store.
    pub store_dir: String,
    /// Also read every block and check its checksum (default false: the index only).
    #[serde(default)]
    pub verify: Option<bool>,
}

/// One level of a store.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct LevelOut {
    /// Level (0 = full resolution).
    pub level: usize,
    /// Triangles over all chunks.
    pub triangles: u64,
    /// Vertices over all chunks (a vertex on a chunk seam is stored once per chunk).
    pub vertices: u64,
    /// Clustering cell size, mm (0 at level 0).
    pub cell_size: f64,
    /// Stated error bound, mm: no vertex of level 0 is farther than this from its vertex here, and
    /// no point of this level farther from the full surface (0 at level 0).
    pub error_bound: f64,
    /// Largest measured error over all chunks, mm.
    pub max_error: f64,
}

/// What the import measured about itself.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct BuildOut {
    /// Most triangles held in memory at once for routing.
    pub peak_buffered_triangles: usize,
    /// Most triangles of one chunk before welding.
    pub max_chunk_triangles: usize,
    /// Disk the spill files took at their peak, bytes (72 per triangle; deleted again).
    pub peak_spill_bytes: u64,
    /// Upper bound of the memory the import needs, MiB (independent of the size of the file).
    pub memory_bound_mib: f64,
}

/// A chunk store.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct StoreOut {
    /// The store directory.
    pub store_dir: String,
    /// Chunks.
    pub chunks: usize,
    /// Chunk capacity.
    pub chunk_triangles: u32,
    /// Triangles read from the file.
    pub source_triangles: u64,
    /// Triangles stored at full resolution (degenerate ones are dropped).
    pub triangles: u64,
    /// Vertices stored at full resolution.
    pub vertices: u64,
    /// Bounding box minimum.
    pub bbox_min: [f64; 3],
    /// Bounding box maximum.
    pub bbox_max: [f64; 3],
    /// Surface area, mm².
    pub area: f64,
    /// Does the store keep per-level maps?
    pub has_maps: bool,
    /// Levels, full resolution first.
    pub levels: Vec<LevelOut>,
    /// Bytes of the store files.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bytes_on_disk: Option<u64>,
    /// True when every block was read back and checked.
    pub verified: bool,
    /// Present after an import.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub build: Option<BuildOut>,
    /// The unit the store was built in (none for a store that did not record one).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub units: Option<LengthUnit>,
    /// The size check on the bounds, when the unit was not stated and the size looks wrong.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scale_warning: Option<size_check::ScaleWarning>,
}

/// The unit a store was built in, kept beside its files.
#[cfg(not(target_arch = "wasm32"))]
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
struct UnitsRecord {
    units: LengthUnit,
    stated: bool,
}

#[cfg(not(target_arch = "wasm32"))]
const UNITS_FILE: &str = "units.json";

#[cfg(not(target_arch = "wasm32"))]
fn write_units(dir: &std::path::Path, record: UnitsRecord) -> CmdResult<()> {
    let text = serde_json::to_string(&record).map_err(|e| CmdError::Failed(e.to_string()))?;
    std::fs::write(dir.join(UNITS_FILE), text).map_err(|e| CmdError::Failed(format!("cannot record the units: {e}")))
}

/// Largest units record read or replaced: a real record is a few dozen bytes.
#[cfg(not(target_arch = "wasm32"))]
const MAX_UNITS_RECORD: u64 = 4096;

/// Takes the units record out of the way of a store that is replaced, and returns its bytes so that
/// a rejected call can put them back. A record too large to be ours is refused before anything
/// changes.
#[cfg(not(target_arch = "wasm32"))]
fn take_units_record(dir: &std::path::Path) -> CmdResult<Option<Vec<u8>>> {
    let path = dir.join(UNITS_FILE);
    let size = match std::fs::metadata(&path) {
        Ok(meta) => meta.len(),
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => return Ok(None),
        Err(e) => return Err(CmdError::Failed(format!("cannot read the units record of the store: {e}"))),
    };
    if size > MAX_UNITS_RECORD {
        return Err(CmdError::Failed("the units record of the store is too large to replace: remove it first".into()));
    }
    let bytes = std::fs::read(&path).map_err(|e| CmdError::Failed(format!("cannot read the units record: {e}")))?;
    std::fs::remove_file(&path).map_err(|e| CmdError::Failed(format!("cannot replace the units record: {e}")))?;
    Ok(Some(bytes))
}

/// Puts a units record taken by [`take_units_record`] back, after a rejected call.
#[cfg(not(target_arch = "wasm32"))]
fn put_back_units_record(dir: &std::path::Path, previous: Option<Vec<u8>>) -> CmdResult<()> {
    match previous {
        Some(bytes) => std::fs::write(dir.join(UNITS_FILE), bytes)
            .map_err(|e| CmdError::Failed(format!("cannot restore the units record: {e}"))),
        None => Ok(()),
    }
}

/// The units record of a store. A record that is missing, oversized, not UTF-8 or not a record is
/// nothing remembered: the store's files are not touched, and the size check then applies.
#[cfg(not(target_arch = "wasm32"))]
fn read_units(dir: &std::path::Path) -> Option<UnitsRecord> {
    let path = dir.join(UNITS_FILE);
    if std::fs::metadata(&path).ok()?.len() > MAX_UNITS_RECORD {
        return None;
    }
    let text = String::from_utf8(std::fs::read(&path).ok()?).ok()?;
    serde_json::from_str(&text).ok()
}

/// The size check on the bounds of a store (its largest side), unless its unit was stated.
#[cfg(not(target_arch = "wasm32"))]
fn store_warning(out: &StoreOut, stated: bool) -> Option<size_check::ScaleWarning> {
    if stated || out.triangles == 0 {
        return None;
    }
    let extent = out.bbox_max.iter().zip(&out.bbox_min).map(|(hi, lo)| hi - lo).fold(0.0, f64::max);
    size_check::warning_for_extent(extent, false)
}

#[cfg(not(target_arch = "wasm32"))]
fn store_out(dir: &str, s: &StoreSummary) -> StoreOut {
    let level0 = s.levels.first();
    let (lo, hi) =
        if s.bbox.is_empty() { ([0.0; 3], [0.0; 3]) } else { (s.bbox.min.coords.into(), s.bbox.max.coords.into()) };
    StoreOut {
        store_dir: dir.to_string(),
        chunks: s.chunks,
        chunk_triangles: s.chunk_triangles,
        source_triangles: s.source_triangles,
        triangles: level0.map_or(0, |l| l.triangles),
        vertices: level0.map_or(0, |l| l.vertices),
        bbox_min: lo,
        bbox_max: hi,
        area: s.area,
        has_maps: s.has_maps,
        levels: s
            .levels
            .iter()
            .map(|l| LevelOut {
                level: l.level,
                triangles: l.triangles,
                vertices: l.vertices,
                cell_size: l.cell_size,
                error_bound: l.error_bound,
                max_error: l.max_error,
            })
            .collect(),
        bytes_on_disk: None,
        verified: false,
        build: None,
        units: None,
        scale_warning: None,
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn params_of(i: &ImportStreamIn) -> CmdResult<StoreParams> {
    let d = StoreParams::default();
    let p = StoreParams {
        chunk_triangles: i.chunk_triangles.unwrap_or(d.chunk_triangles),
        levels: i.levels.unwrap_or(d.levels),
        keep_maps: i.keep_maps.unwrap_or(d.keep_maps),
        max_triangles: d.max_triangles,
    };
    p.validate().map_err(|e| CmdError::Params(e.to_string()))?;
    if i.scale.is_some_and(|s| !(s.is_finite() && s > 0.0)) {
        return Err(CmdError::Params("scale must be positive".into()));
    }
    Ok(p)
}

#[cfg(not(target_arch = "wasm32"))]
fn import_stream(s: &mut Session, i: ImportStreamIn) -> CmdResult<StoreOut> {
    use std::path::Path;
    use surfacecraft_geom::chunked::build::memory_bound_bytes;
    use surfacecraft_io::stream::import::{StreamImportOptions, import_to_store};
    super::require_files(s)?;
    if i.path.is_empty() || i.store_dir.is_empty() {
        return Err(CmdError::Params("`path` and `store_dir` must not be empty".into()));
    }
    let store = params_of(&i)?;
    let opts = StreamImportOptions {
        format: i.format,
        scale: crate::cmd::mesh::units::import_scale(i.scale, i.units).unwrap_or(1.0),
        store,
        overwrite: i.overwrite.unwrap_or(false),
        ..StreamImportOptions::default()
    };
    let dir = Path::new(&i.store_dir);
    let previous = if i.overwrite.unwrap_or(false) { take_units_record(dir)? } else { None };
    let report = match import_to_store(Path::new(&i.path), dir, &opts) {
        Ok(report) => report,
        Err(e) => {
            put_back_units_record(dir, previous)?;
            return Err(e.into());
        }
    };
    let stated = i.units.is_some();
    let units = i.units.unwrap_or_default();
    write_units(Path::new(&i.store_dir), UnitsRecord { units, stated })?;
    let mut out = store_out(&i.store_dir, &report.summary);
    out.bytes_on_disk = Some(report.stats.bytes_on_disk);
    out.units = Some(units);
    out.scale_warning = store_warning(&out, stated);
    out.build = Some(BuildOut {
        peak_buffered_triangles: report.stats.peak_buffered_triangles,
        max_chunk_triangles: report.stats.max_chunk_triangles,
        peak_spill_bytes: report.stats.peak_spill_bytes,
        memory_bound_mib: memory_bound_bytes(&store) as f64 / (1024.0 * 1024.0),
    });
    Ok(out)
}

#[cfg(target_arch = "wasm32")]
fn import_stream(_s: &mut Session, _i: ImportStreamIn) -> CmdResult<StoreOut> {
    Err(CmdError::Unavailable("out-of-core import needs files, which the browser build does not have".into()))
}

#[cfg(not(target_arch = "wasm32"))]
fn store_info(s: &mut Session, i: StoreInfoIn) -> CmdResult<StoreOut> {
    super::require_files(s)?;
    if i.store_dir.is_empty() {
        return Err(CmdError::Params("`store_dir` must not be empty".into()));
    }
    let store = surfacecraft_geom::chunked::disk::open(std::path::Path::new(&i.store_dir))
        .map_err(|e| CmdError::Failed(e.to_string()))?;
    let verified = i.verify.unwrap_or(false);
    if verified {
        store.verify().map_err(|e| CmdError::Failed(e.to_string()))?;
    }
    let mut out = store_out(&i.store_dir, &store.summary());
    out.bytes_on_disk = Some(surfacecraft_geom::chunked::disk::store_bytes(&store, std::path::Path::new(&i.store_dir)));
    out.verified = verified;
    let record = read_units(std::path::Path::new(&i.store_dir));
    out.units = record.map(|r| r.units);
    out.scale_warning = store_warning(&out, record.is_some_and(|r| r.stated));
    Ok(out)
}

#[cfg(target_arch = "wasm32")]
fn store_info(_s: &mut Session, _i: StoreInfoIn) -> CmdResult<StoreOut> {
    Err(CmdError::Unavailable("chunk stores need files, which the browser build does not have".into()))
}

/// `mesh.import_stream` and `mesh.store_info`.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "mesh.import_stream",
        "Import large mesh...",
        "Mesh",
        false,
        "Imports a large STL/binary PLY from `path` into a chunk store in `store_dir` without holding it in memory: spatially coherent chunks (`chunk_triangles`) with bounding boxes and `levels` of detail with stated error bounds; optional `scale`, `format`, `keep_maps`, `overwrite`. Does not add an object to the document.",
        ImportStreamIn,
        StoreOut,
        import_stream
    ),
    command!(
        "mesh.store_info",
        "Chunk store info",
        "Mesh",
        false,
        "Counts, bounding box, area and per-level error bounds of the chunk store in `store_dir`; `verify` reads every block and checks it (a damaged store is an error).",
        StoreInfoIn,
        StoreOut,
        store_info
    ),
];
