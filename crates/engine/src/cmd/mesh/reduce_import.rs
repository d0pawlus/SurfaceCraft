//! `mesh.import` with `reduce_to`: a scan with more triangles than wanted (or than this platform
//! can hold: the browser's limit is far below the desktop's) is reduced while its file is read,
//! by vertex clustering on a grid (`surfacecraft_io::stream::reduce`), so the memory of the import
//! follows the reduced mesh and not the file. Formats that cannot be streamed (OBJ, ASCII PLY) are
//! read whole, within the ordinary limits, and clustered in memory.
use super::read_limits;
use crate::cmd::{CmdError, CmdResult};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Mesh, lod};
use surfacecraft_io::stream::StreamOptions;
use surfacecraft_io::stream::reduce::{declared_triangles as declared_in_header, read_reduced};
use surfacecraft_io::{IoError, Limits, MeshFormat};

/// Fewest triangles a reduction on import may be asked for.
pub const MIN_REDUCE_TO: usize = 1000;

/// Triangles the header of a mesh file declares (binary STL: its count; PLY: its faces), from the
/// leading bytes of the file (`head`: the first mebibyte is enough) and its extension. `None` when
/// the file does not say. For a front end that offers a reduction before it imports.
#[must_use]
pub fn declared_triangles(head: &[u8], extension: &str) -> Option<u64> {
    declared_in_header(head, MeshFormat::from_extension(extension))
}

/// Most triangles a mesh imported here may have (the browser's limit is far below the desktop's);
/// also the largest `reduce_to`.
#[must_use]
pub fn import_triangle_limit() -> usize {
    read_limits().max_triangles
}

/// What reducing a scan on import did.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ReducedOut {
    /// Triangles in the file.
    pub source_triangles: u64,
    /// No vertex of the file is farther than this from its vertex in the mesh, mm (vertex
    /// clustering on a grid: `√3` x the cell size, or the measured distance for a file read whole).
    pub max_error: f64,
}

/// The mesh of a reduced import and what the reduction did.
pub(crate) type ReducedMesh = (Mesh, ReducedOut);

/// The hint added to a limit error of an import that asked for no reduction.
pub(crate) fn limit_hint(e: &IoError, reduce_to: Option<usize>) -> CmdError {
    match e {
        IoError::Limit(m) if reduce_to.is_none() => CmdError::Failed(format!(
            "{m}; import it with `reduce_to` (at most {}) to reduce it while it is read",
            read_limits().max_triangles
        )),
        other => CmdError::Failed(other.to_string()),
    }
}

/// Checks `reduce_to` against what an import here can hold.
pub(crate) fn check_target(target: usize) -> CmdResult<()> {
    let max = read_limits().max_triangles;
    if (MIN_REDUCE_TO..=max).contains(&target) {
        Ok(())
    } else {
        Err(CmdError::Params(format!("reduce_to must be between {MIN_REDUCE_TO} and {max}")))
    }
}

/// Reads `bytes` reduced to at most `target` triangles, scaled by `scale`. `None` when the file
/// is to be read the ordinary way: it has no more than `target` triangles, or its format cannot
/// be streamed (the caller then reduces what it read with [`reduce_in_memory`]).
pub(crate) fn read(bytes: &[u8], fmt: MeshFormat, scale: f64, target: usize) -> CmdResult<Option<ReducedMesh>> {
    // the file is never held as a mesh, so no count bounds it: the readers still check every
    // declared count against the length of the file
    let limits = Limits { max_triangles: usize::MAX, max_vertices: usize::MAX, ..Limits::default() };
    let opts = StreamOptions { limits, scale };
    match read_reduced(bytes, Some(fmt), &opts, target) {
        Ok(Some(r)) => {
            let max_error = 3f64.sqrt() * r.cell_size;
            Ok(Some((r.mesh, ReducedOut { source_triangles: r.source_triangles, max_error })))
        }
        Ok(None) | Err(IoError::Unsupported(_)) => Ok(None),
        Err(e) => Err(CmdError::Failed(e.to_string())),
    }
}

/// Reduces a mesh that was read whole to at most `target` triangles; `None` when it is within it.
pub(crate) fn reduce_in_memory(mesh: &Mesh, target: usize) -> CmdResult<Option<ReducedMesh>> {
    let source = mesh.triangle_count();
    if source <= target {
        return Ok(None);
    }
    let coarse = lod::cluster(mesh, target).ok_or_else(|| CmdError::Failed("the mesh could not be reduced".into()))?;
    let max_error = coarse.max_displacement(mesh);
    let mut reduced = coarse.mesh;
    reduced.remove_degenerate(0.0);
    surfacecraft_geom::edit::compact(&mut reduced);
    Ok(Some((reduced, ReducedOut { source_triangles: source as u64, max_error })))
}
