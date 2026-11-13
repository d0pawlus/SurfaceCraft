//! `mesh.thicken`: an open mesh (a one-sided scan) made into a closed shell of constant thickness,
//! as a new mesh. The computation is `surfacecraft_geom::shell`; `scan.to_solid` with a `thickness`
//! uses the same function before it wraps the shell in patches.
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::Mesh;
use surfacecraft_geom::quality::edge_census;
use surfacecraft_geom::shell::{ThickenError, ThickenParams, Thickened, thicken};

/// Parameters of `mesh.thicken`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ThickenMeshIn {
    /// The open mesh to thicken (mesh id or name; a closed mesh has nothing to thicken).
    pub object: ObjRef,
    /// Wall thickness in mm (no default; not 0, at most half the size of the mesh): positive puts the material behind the scanned face (against its normals), negative in front of it (along them).
    pub thickness: f64,
    /// Put half of the thickness on each side of the scanned face (default false; the sign of `thickness` is then ignored).
    #[serde(default)]
    pub symmetric: bool,
    /// Name of the new mesh (default: the name of `object` and " thickened").
    #[serde(default)]
    pub name: Option<String>,
}

/// Result of `mesh.thicken`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ThickenMeshOut {
    /// The new mesh.
    pub id: ObjectId,
    /// Its name.
    pub name: String,
    /// Triangles of the new mesh.
    pub triangles: usize,
    /// Vertices of the new mesh.
    pub vertices: usize,
    /// Boundary loops of the open mesh that were stitched to their offset copies (the rim, windows).
    pub boundary_loops: usize,
    /// Vertices whose offset was reduced because the surface curves more tightly than the thickness.
    pub clamped_vertices: usize,
    /// Largest reduction of the thickness at a vertex (mm; 0 when nothing was reduced).
    pub largest_reduction: f64,
    /// Triangles removed from the mesh before the offset (fins on edges of three or more triangles, duplicates).
    pub removed_non_manifold: usize,
    /// Triangles turned around to make the orientation consistent.
    pub flipped_triangles: usize,
    /// Vertices that stay folded at the floor of the offset (a tenth of the thickness): sharp creases.
    pub unresolved_vertices: usize,
    /// Thinnest wall found (mm), sampled as the distance between the two sheets.
    pub min_wall: f64,
    /// Median wall found (mm).
    pub median_wall: f64,
    /// Always false: only local folds are detected, not a part of the offset sheet meeting another part of the shell (a channel narrower than the wall).
    pub self_intersection_checked: bool,
    /// Is the new mesh closed (every edge shared by two triangles)? Always true when the command succeeds.
    pub closed: bool,
}

/// A refusal of `shell::thicken` as a command error: unusable parameters or mesh.
pub(crate) fn command_error(e: &ThickenError) -> CmdError {
    match e {
        ThickenError::BadThickness(_) | ThickenError::TooThick { .. } => CmdError::Params(e.to_string()),
        _ => CmdError::Failed(e.to_string()),
    }
}

/// Thickens `mesh`; the error is a command error with the reason.
pub(crate) fn thicken_mesh(mesh: &Mesh, thickness: f64, symmetric: bool) -> CmdResult<Thickened> {
    thicken(mesh, &ThickenParams { thickness, symmetric }).map_err(|e| command_error(&e))
}

pub(super) fn thicken_cmd(s: &mut Session, i: ThickenMeshIn) -> CmdResult<ThickenMeshOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let obj = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let shell = thicken_mesh(obj.editable(), i.thickness, i.symmetric)?;
    let census = edge_census(&shell.mesh);
    let default = format!("{} thickened", obj.name);
    let (triangles, vertices) = (shell.mesh.triangle_count(), shell.mesh.vertex_count());
    let new_id = s.doc.add_mesh(i.name.as_deref().unwrap_or(&default), shell.mesh);
    let name = s.doc.mesh(new_id).map(|m| m.name.clone()).unwrap_or_default();
    Ok(ThickenMeshOut {
        id: new_id,
        name,
        triangles,
        vertices,
        boundary_loops: shell.report.boundary_loops,
        clamped_vertices: shell.report.clamped_vertices,
        largest_reduction: shell.report.largest_reduction,
        removed_non_manifold: shell.report.removed_non_manifold,
        flipped_triangles: shell.report.flipped_triangles,
        unresolved_vertices: shell.report.unresolved_vertices,
        min_wall: shell.report.min_wall,
        median_wall: shell.report.median_wall,
        self_intersection_checked: false,
        closed: census.boundary == 0 && census.non_manifold == 0,
    })
}
