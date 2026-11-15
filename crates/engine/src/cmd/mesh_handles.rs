//! `mesh.close_handles`: the small handles of a scan (tunnels, bridges, the sponge of tiny tunnels
//! where the two sides of a thin wall come closer than the scanner resolves) closed (`geom::handles`),
//! as a new mesh. The scan stays; the rim of an open scan and every vertex away from the handles are
//! kept; the report says what was cut out and how far it lies from the result.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::Mesh;
use surfacecraft_geom::handles::{HandleError, Params, Report, close_handles};

/// Default largest size of the handles closed, in median edges of the scan.
pub(crate) const DEFAULT_SIZE_EDGES: f64 = 12.0;
/// Default tolerance (mm).
const TOLERANCE: f64 = 0.2;

/// Parameters of `mesh.close_handles`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct CloseHandlesIn {
    /// Scan mesh id or name (open or closed).
    pub object: ObjRef,
    /// Handles in neighbourhoods of the surface at most this wide are closed (mm, default 12 median edges of the scan). A tunnel or bridge narrower than about half of it is looked for (a check may still refuse to close it); a real hole through a thin wall narrower than it is a tunnel too and is closed, a wider one is kept.
    #[serde(default)]
    pub max_size: Option<f64>,
    /// A fill of which more than a quarter lies farther than this from the surface it replaces is refused; what was cut out is measured against the result with it (mm, default 0.2).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Name of the new mesh (default "handles closed").
    #[serde(default)]
    pub name: Option<String>,
}

/// Handles closed at one scale.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct HandleScale {
    /// Width of the neighbourhoods looked at (mm).
    pub scale: f64,
    /// Parts cut out and filled.
    pub parts: usize,
    /// Handles they closed.
    pub handles: usize,
}

/// Parts that were left, by reason.
#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct HandlesRefused {
    /// Cutting the part would split the surface in two.
    pub splits: usize,
    /// A hole the part left could not be filled.
    pub unfillable: usize,
    /// More than a quarter of a patch of the fill (one per hole) lay farther than the tolerance from the surface it replaced: a cap over the mouth of a real opening.
    pub fill_far: usize,
    /// What was cut out is not covered by the result (more than a quarter of the points sampled over it farther than the tolerance, or one farther than a quarter of the neighbourhood), or the fill left a pinched vertex.
    pub uncovered: usize,
    /// Whole closed pieces with genus small enough to be cut out whole: left, never deleted.
    pub whole_pieces: usize,
}

/// Result of `mesh.close_handles`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct CloseHandlesOut {
    /// The new mesh; absent when nothing was closed (the scan is left as it is and no mesh is made).
    pub id: Option<ObjectId>,
    /// Was anything closed?
    pub changed: bool,
    /// Largest size used (mm).
    pub max_size: f64,
    /// Median edge of the scan (mm).
    pub median_edge: f64,
    /// Handles of the scan before, from its Euler characteristic and boundary loops (an estimate when the scan has pinched vertices).
    pub handles_before: i64,
    /// ... and after.
    pub handles_after: i64,
    /// Connected pieces before; the same after (no part is cut loose).
    pub pieces_before: usize,
    /// ... and after.
    pub pieces_after: usize,
    /// Boundary loops before (the rim of an open scan and its holes).
    pub loops_before: usize,
    /// ... and after: the same, the rim is never touched.
    pub loops_after: usize,
    /// Handles closed.
    pub handles_closed: usize,
    /// Parts cut out and filled.
    pub parts: usize,
    /// What was closed at each scale, smallest first.
    pub by_scale: Vec<HandleScale>,
    /// Parts with handles that were left, by reason (counted once per pass that refused them).
    pub refused: HandlesRefused,
    /// Triangles cut out.
    pub removed_triangles: usize,
    /// Area cut out (mm²).
    pub removed_area: f64,
    /// Share of the area of the scan cut out (0..1).
    pub removed_area_share: f64,
    /// Triangles the fills added.
    pub added_triangles: usize,
    /// Points sampled over the triangles of the scan that were cut out (corners, edge midpoints, centres and a lattice), so that a tunnel wall of long triangles counts.
    pub removed_samples: usize,
    /// Of them, within the tolerance of the result.
    pub removed_within: usize,
    /// Largest distance of such a point from the result (mm): the tunnel walls that went.
    pub removed_max_distance: f64,
    /// Triangles of the result.
    pub triangles: usize,
}

/// Handles of a mesh from its Euler characteristic, loops and pieces: `sum (2 − χ − b) / 2`.
pub(crate) fn handle_estimate(mesh: &Mesh) -> (i64, usize) {
    let ps = surfacecraft_geom::pieces::pieces(mesh);
    let handles = ps.iter().map(|p| (2 - p.euler - p.loops.len() as i64).max(0) / 2).sum();
    (handles, ps.iter().map(|p| p.loops.len()).sum())
}

/// The largest size: given, or [`DEFAULT_SIZE_EDGES`] median edges; `None` when it is not given and the
/// mesh has no edge length to take it from (nothing can be closed).
pub(crate) fn size_of(given: Option<f64>, median_edge: f64) -> CmdResult<Option<f64>> {
    match given {
        Some(size) if size.is_finite() && size > 0.0 => Ok(Some(size)),
        Some(size) => Err(CmdError::Params(format!("max_size must be a positive number of mm, not {size}"))),
        None => {
            let size = DEFAULT_SIZE_EDGES * median_edge;
            Ok((size.is_finite() && size > 0.0).then_some(size))
        }
    }
}

/// Closes the handles of `mesh` (a copy is changed). The error of a cancellation is the command's.
pub(crate) fn close(mesh: &Mesh, max_size: f64, tolerance: f64, s: &Session) -> CmdResult<(Mesh, Report)> {
    let mut m = mesh.clone();
    let rep = close_handles(&mut m, &Params { max_size, tolerance }, &s.cancel).map_err(|e| match e {
        HandleError::Cancelled => super::cancelled(),
        HandleError::Params(p) => CmdError::Params(p),
        HandleError::Invalid(p) => CmdError::Failed(format!("the mesh is invalid: {p}")),
    })?;
    Ok((m, rep))
}

/// What was closed at each scale.
pub(crate) fn by_scale(rep: &Report) -> Vec<HandleScale> {
    rep.scales
        .iter()
        .map(|&scale| {
            let at = rep.closed.iter().filter(|c| c.scale == scale);
            HandleScale { scale, parts: at.clone().count(), handles: at.map(|c| c.handles).sum() }
        })
        .collect()
}

fn out_of(id: Option<ObjectId>, rep: &Report, max_size: f64, (before, after): (i64, i64), m: &Mesh) -> CloseHandlesOut {
    CloseHandlesOut {
        id,
        changed: rep.changed,
        max_size,
        median_edge: rep.median_edge,
        handles_before: before,
        handles_after: after,
        pieces_before: rep.pieces_before,
        pieces_after: rep.pieces_after,
        loops_before: rep.loops_before,
        loops_after: rep.loops_after,
        handles_closed: rep.handles_closed,
        parts: rep.closed.len(),
        by_scale: by_scale(rep),
        refused: HandlesRefused {
            splits: rep.refused.splits,
            unfillable: rep.refused.unfillable,
            fill_far: rep.refused.fill_far,
            uncovered: rep.refused.uncovered,
            whole_pieces: rep.refused.whole_pieces,
        },
        removed_triangles: rep.removed_triangles,
        removed_area: rep.removed_area,
        removed_area_share: rep.removed_area_share,
        added_triangles: rep.added_triangles,
        removed_samples: rep.removed_samples,
        removed_within: rep.removed_within,
        removed_max_distance: rep.removed_max_distance,
        triangles: m.triangle_count(),
    }
}

pub(crate) fn close_handles_cmd(s: &mut Session, i: CloseHandlesIn) -> CmdResult<CloseHandlesOut> {
    if s.cancel.is_cancelled() {
        return Err(super::cancelled());
    }
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let scan = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let tolerance = super::deviation::positive(i.tolerance.unwrap_or(TOLERANCE), "tolerance")?;
    let h = surfacecraft_geom::quality::edge_census(&scan).lengths.median;
    let Some(max_size) = size_of(i.max_size, h)? else {
        return Err(CmdError::Failed("the mesh has no edges to measure".into()));
    };
    let (before, _) = handle_estimate(&scan);
    let (mesh, rep) = close(&scan, max_size, tolerance, s)?;
    let (after, _) = handle_estimate(&mesh);
    if !rep.changed {
        return Ok(out_of(None, &rep, max_size, (before, before), &mesh));
    }
    let out = out_of(None, &rep, max_size, (before, after), &mesh);
    let new = s.doc.add_mesh(i.name.as_deref().unwrap_or("handles closed"), mesh);
    Ok(CloseHandlesOut { id: Some(new), ..out })
}

/// The `mesh.close_handles` command.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "mesh.close_handles",
    "Close small handles",
    "Mesh",
    true,
    "Closes the small handles of the scan `object` (open or closed) as a new mesh: tunnels, bridges and the sponge of tiny tunnels a scanner makes where the two sides of a thin wall come closer than it resolves. They stall the mesh reduction and cost the patch budget of `surface.wrap`. Every neighbourhood of the surface up to `max_size` wide (default 12 median edges) that is not a disk is cut out and its holes filled with the smooth hole fill; each closes its genus plus its loops less one. The rim of an open scan, edges of three or more triangles and every vertex away from the handles are kept; a part whose fill does not follow the surface it replaces (`tolerance`) or whose removal would split the surface is left. A real hole through a thin wall narrower than `max_size` is a tunnel too. Reports the handles before and after, what was cut out by scale, and how far the vertices cut out lie from the result. When nothing is closed no mesh is made.",
    CloseHandlesIn,
    CloseHandlesOut,
    close_handles_cmd
)];
