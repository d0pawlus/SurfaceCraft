//! `quad.retopo`: its parameters and result, the source of the layout, and the command.
use super::super::{CmdError, CmdResult, ObjRef};
use super::field::*;
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::retopo::RetopoOptions;
use surfacecraft_geom::Mesh;

/// Retopology parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RetopoIn {
    /// Region to cover (wrapped or closed regions are fine).
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh to cover whole (when no region).
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Faces wanted in the field layout (default 100). Faces that are not quads force one
    /// 4-to-1 split of the whole layout into quads, so the result can hold several times more.
    #[serde(default)]
    pub target_quads: Option<usize>,
    /// Orientation-field smoothing sweeps per level (default 40, at most 500).
    #[serde(default)]
    pub orientation_sweeps: Option<usize>,
    /// Position-field smoothing sweeps per level (default 40, at most 500).
    #[serde(default)]
    pub position_sweeps: Option<usize>,
    /// Experimental: conform the layout of an open region to a smoothed outline of the region and put split boundary vertices back onto it (default false). Falls back to the plain layout when any quad would be improper.
    #[serde(default)]
    pub conform_boundary: Option<bool>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// Retopology result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RetopoOut {
    /// New unstructured layout.
    pub id: ObjectId,
    /// Quads in the layout.
    pub quads: usize,
    /// Layout vertices.
    pub vertices: usize,
    /// `field` (orientation/position field extraction) or `fallback` (decimation split into quads).
    pub layout: String,
    /// Faces the field extraction gave.
    pub field_faces: usize,
    /// Quads among the field faces.
    pub field_quads: usize,
    /// Share of interior vertices whose valence is not 4 (after the split to quads).
    pub irregular_fraction: f64,
    /// Edge length asked of the field (mm).
    pub edge_length: f64,
    /// Euler characteristic V − E + F of the layout.
    pub euler: i64,
    /// Closed (no free boundary)?
    pub closed: bool,
    /// Quads whose orientation is inverted against the scan normals (should be 0).
    pub folded: usize,
    /// Open regions: is every free boundary loop of the layout conformed to the region outline (false
    /// when any loop is left as it was, or when the final check reverted the conform)?
    pub outline_conformed: bool,
    /// Open regions: free boundary loops of the layout conformed to the region outline.
    pub outline_loops_conformed: usize,
    /// Open regions: free boundary loops left exactly as they were (no matching outline loop, order
    /// mismatch, a span too short for its vertices, or an invalid face after snapping).
    pub outline_loops_skipped: usize,
    /// Why the first such loop was left as it was.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub outline_unconformed_reason: Option<String>,
    /// Open regions: faces added between the layout and the outline.
    pub outline_ring_faces: usize,
    /// Quads whose patch was turned against the scan and that were turned around (repaired).
    pub repaired_patches: usize,
    /// Quads whose patch still runs against the scan (should be 0; `surface.auto` then refuses the layout).
    pub flipped_patches: usize,
    /// Quads whose patch could not be checked against the scan orientation (no samples on its side).
    pub unverified_patches: usize,
    /// Largest distance of a layout vertex from the scan (mm).
    pub snap_max: f64,
    /// Why the fallback was used.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

/// The mesh and region that `quad.retopo` covers: a region with its mesh, or a whole mesh.
fn retopo_source(s: &Session, i: &RetopoIn) -> CmdResult<(ObjectId, Option<ObjectId>)> {
    match (&i.region, &i.mesh) {
        (Some(r), None) => {
            let rid = r.resolve(s, ObjectKind::Region)?;
            let mesh = s.doc.region(rid).map(|r| r.mesh).ok_or_else(|| CmdError::Params("no region".into()))?;
            Ok((mesh, Some(rid)))
        }
        (None, Some(m)) => Ok((m.resolve(s, ObjectKind::Mesh)?, None)),
        _ => Err(CmdError::Params("give `region` or `mesh`".into())),
    }
}

/// A sweep count of the input: default 40, at most 500.
fn sweeps_in(v: Option<usize>, what: &str) -> CmdResult<usize> {
    let v = v.unwrap_or(40);
    if (1..=500).contains(&v) { Ok(v) } else { Err(CmdError::Params(format!("{what} must be 1..=500"))) }
}

/// The retopology options of the input, with the defaults and limits of `quad.retopo`.
fn retopo_options(i: &RetopoIn) -> CmdResult<RetopoOptions> {
    let target = i.target_quads.unwrap_or(100);
    if !(8..=20_000).contains(&target) {
        return Err(CmdError::Params("target_quads must be 8..=20000".into()));
    }
    Ok(RetopoOptions {
        target_faces: target,
        orientation_sweeps: sweeps_in(i.orientation_sweeps, "orientation_sweeps")?,
        position_sweeps: sweeps_in(i.position_sweeps, "position_sweeps")?,
        force_fallback: false,
        conform_boundary: i.conform_boundary.unwrap_or(false),
    })
}

/// The triangles that `quad.retopo` covers: the region's, or every triangle of `m`.
fn retopo_triangles(s: &Session, m: &Mesh, region: Option<ObjectId>) -> CmdResult<Vec<u32>> {
    let tris: Vec<u32> = match region {
        Some(r) => s.doc.region(r).map(|r| r.triangles.to_vec()).ok_or_else(|| CmdError::Params("no region".into()))?,
        None => (0..m.triangle_count() as u32).collect(),
    };
    if tris.len() < 20 {
        return Err(CmdError::Params("too few triangles to retopologise (need 20)".into()));
    }
    Ok(tris)
}
pub(in crate::cmd::quad) fn retopo_cmd(s: &mut Session, i: RetopoIn) -> CmdResult<RetopoOut> {
    let (mesh, region) = retopo_source(s, &i)?;
    let opts = retopo_options(&i)?;
    let m = s.doc.mesh(mesh).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {mesh}")))?;
    let tris = retopo_triangles(s, &m, region)?;
    let sub = m.submesh(&tris);
    let scan = LayoutScan::of_region(&m, &tris);
    let (fit, attempt, reverted) = netted_layout(&sub, &scan, &opts)?;
    let boundary = fit.quads.boundary_edges();
    let irregular = interior_irregularity(&fit.quads, &boundary);
    let snap_max = layout_snap_max(&fit.quads, &sub);
    let id = add_retopo_layout(s, mesh, region, i.name.unwrap_or_else(|| "retopo".into()), &fit.quads);
    Ok(RetopoOut {
        id,
        quads: fit.quads.faces.len(),
        vertices: fit.quads.vertices.len(),
        layout: match fit.rep.layout {
            surfacecraft_fit::retopo::Layout::Field => "field".into(),
            surfacecraft_fit::retopo::Layout::Fallback => "fallback".into(),
        },
        field_faces: fit.field_faces,
        field_quads: fit.field_quads,
        irregular_fraction: irregular,
        edge_length: fit.rep.edge_length,
        euler: fit.quads.euler(),
        closed: boundary.is_empty(),
        folded: fit.folded,
        outline_loops_conformed: fit.rep.boundary.conformed,
        outline_loops_skipped: fit.rep.boundary.skipped + fit.rep.boundary.rolled_back + fit.rep.boundary.unchanged,
        outline_conformed: fit.rep.boundary.all_conformed(),
        outline_unconformed_reason: reverted.or(fit.rep.boundary.reason).map(str::to_string),
        outline_ring_faces: fit.rep.boundary.ring_faces,
        repaired_patches: fit.flips.repaired,
        flipped_patches: fit.flips.failed,
        unverified_patches: fit.flips.unverified,
        snap_max,
        note: if attempt > 0 {
            Some("the field layout had folded or flipped quads; the fallback layout was used".into())
        } else {
            fit.rep.note
        },
    })
}
