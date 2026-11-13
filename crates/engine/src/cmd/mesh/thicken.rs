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
    /// Vertices whose offset was bounded up front by the distance to another part of the scan close to them along the surface (a bead, a crease: the local feature size), so the offset sheet stays on its side of the scanned sheet.
    pub limited_vertices: usize,
    /// Largest reduction of the thickness at a vertex (mm; 0 when nothing was reduced).
    pub largest_reduction: f64,
    /// Triangles removed from the mesh before the offset (fins on edges of three or more triangles, duplicates).
    pub removed_non_manifold: usize,
    /// Triangles turned around to make the orientation consistent.
    pub flipped_triangles: usize,
    /// Vertices on a local crossing (neighbouring offset triangles, the flanks of a crease, the two sheets of a wall, the strip along the rim) that the relaxation could not remove even at the floor of the offset (a tenth of the thickness): sharp creases. More than 0.1 % of the vertices is refused.
    pub unresolved_vertices: usize,
    /// Thinnest wall found (mm), sampled as the distance between the two sheets.
    pub min_wall: f64,
    /// Median wall found (mm).
    pub median_wall: f64,
    /// Always true: an even sample of the new mesh was tested for crossing triangles.
    pub self_intersection_checked: bool,
    /// Triangles of the sample tested.
    pub self_intersection_samples: usize,
    /// Of them, triangles that cross only triangles close to them along the scan (a fold of the offset). The relaxation removes these, so the count is 0 unless `unresolved_vertices` is not.
    pub self_intersections_local: usize,
    /// Of them, triangles that cross a triangle far from them along the scan: the offset sheet meets another part of the shell, e.g. in a channel narrower than the wall (real geometry, not relaxed).
    pub self_intersections: usize,
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
        limited_vertices: shell.report.limited_vertices,
        largest_reduction: shell.report.largest_reduction,
        removed_non_manifold: shell.report.removed_non_manifold,
        flipped_triangles: shell.report.flipped_triangles,
        unresolved_vertices: shell.report.unresolved_vertices,
        min_wall: shell.report.min_wall,
        median_wall: shell.report.median_wall,
        self_intersection_checked: true,
        self_intersection_samples: shell.report.crossing_samples,
        self_intersections_local: shell.report.crossing_local,
        self_intersections: shell.report.crossing_triangles,
        closed: census.boundary == 0 && census.non_manifold == 0,
    })
}
