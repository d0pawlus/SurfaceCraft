//! `mesh.thicken`: an open mesh (a one-sided scan) made into a closed shell of constant thickness,
//! as a new mesh. The computation is `surfacecraft_geom::shell`; `scan.to_solid` with a `thickness`
//! uses the same function before it wraps the shell in patches.
use crate::Session;
use crate::cmd::pipeline::Issue;
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
    /// Vertices of the scan moved (by at most one median edge) to remove crossings of the scan with itself (a fold of the sheet): the thickened face differs from the scan there by that much.
    pub repaired_vertices: usize,
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
    /// Vertices of the scan moved by the repair (`repaired_vertices`): the largest shift (mm).
    pub repair_largest_shift: f64,
    /// What a caller must not overlook, the same issues `scan.to_solid` raises for the shell: `self_intersection` (any known crossing: `self_intersections`, `self_intersections_local` or `unresolved_vertices` above 0) and `scan_repaired` (the scan was changed by more than the default tolerance of 0.2 mm, or at more than 0.5 % of its vertices).
    pub issues: Vec<Issue>,
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

/// The tolerance (mm) `mesh.thicken` judges a repair of the scan by (that of `scan.to_solid`).
const DEFAULT_TOLERANCE: f64 = 0.2;
/// A repair of the scan that touches more than this share of its vertices is an issue.
pub(crate) const REPAIRED_ISSUE_SHARE: f64 = 0.005;

/// The numbers the issues about crossings and about the repair of the scan are made from.
#[derive(Clone, Copy, Debug)]
pub(crate) struct ShellCounts {
    /// Sampled triangles that cross a neighbouring one.
    pub local: usize,
    /// Sampled triangles that cross a part far away along the surface.
    pub far: usize,
    /// Vertices that stay on a crossing.
    pub unresolved: usize,
    /// Triangles sampled.
    pub samples: usize,
    /// Triangles of the shell.
    pub triangles: usize,
    /// Vertices of the scan moved by the repair, and the largest shift (mm).
    pub repaired: usize,
    pub largest_shift: f64,
    /// Vertices of the scan.
    pub vertices: usize,
}

fn counts_of(shell: &Thickened) -> ShellCounts {
    ShellCounts {
        local: shell.report.crossing_local,
        far: shell.report.crossing_triangles,
        unresolved: shell.report.unresolved_vertices,
        samples: shell.report.crossing_samples,
        triangles: shell.mesh.triangle_count(),
        repaired: shell.report.repaired_vertices,
        largest_shift: shell.report.repair_largest_shift,
        vertices: shell.scale.len(),
    }
}

/// The issues about crossings (any known crossing: never a pass) and about a repair of the scan that is
/// larger than `tolerance` or touches more than [`REPAIRED_ISSUE_SHARE`] of its vertices.
pub(crate) fn crossing_issues(c: &ShellCounts, tolerance: f64) -> Vec<Issue> {
    let mut list = Vec::new();
    if c.far > 0 || c.local > 0 || c.unresolved > 0 {
        list.push(Issue { code: "self_intersection".to_string(), message: crossing_message(c) });
    }
    if c.repaired > 0 && (c.largest_shift > tolerance || c.repaired as f64 > REPAIRED_ISSUE_SHARE * c.vertices as f64) {
        let message = format!(
            "the scan crossed itself (folds, spikes) and {} of {} vertices were smoothed before the wall was made, by {:.3} mm at most (tolerance {tolerance} mm): the shell follows the smoothed scan and the deviation is measured against the scan as it was",
            c.repaired, c.vertices, c.largest_shift
        );
        list.push(Issue { code: "scan_repaired".to_string(), message });
    }
    list
}

/// What a shell that still crosses itself says: how many sampled triangles cross far apart along the
/// surface (the wall is thicker than a channel or gap), how many close to each other, and how many
/// vertices stay on a crossing the relaxation could not remove.
fn crossing_message(c: &ShellCounts) -> String {
    let whole = |n: usize| n as f64 * c.triangles as f64 / c.samples.max(1) as f64;
    let mut parts = Vec::new();
    if c.far > 0 {
        parts.push(format!(
            "{} of {} sampled triangles cut a part of the scan that is far from them along the surface (about {:.0} of {} in all): the wall is thicker than a channel or gap of the part",
            c.far, c.samples, whole(c.far), c.triangles
        ));
    }
    if c.local > 0 {
        parts.push(format!(
            "{} of {} sampled triangles cut a neighbouring one (about {:.0} in all): the offset could not be relaxed there",
            c.local, c.samples, whole(c.local)
        ));
    }
    if c.unresolved > 0 {
        parts.push(format!(
            "{} vertices stay on a crossing that no wall down to a tenth of the thickness removes (sharp creases, collapsed walls)",
            c.unresolved
        ));
    }
    format!(
        "the thickened mesh still crosses itself: {}; a thinner wall, or splitting the part (`scan.split`), is the way out",
        parts.join("; ")
    )
}

pub(super) fn thicken_cmd(s: &mut Session, i: ThickenMeshIn) -> CmdResult<ThickenMeshOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let obj = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let shell = thicken_mesh(obj.editable(), i.thickness, i.symmetric)?;
    let census = edge_census(&shell.mesh);
    let default = format!("{} thickened", obj.name);
    let (triangles, vertices) = (shell.mesh.triangle_count(), shell.mesh.vertex_count());
    let issues = crossing_issues(&counts_of(&shell), DEFAULT_TOLERANCE);
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
        repaired_vertices: shell.report.repaired_vertices,
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
        repair_largest_shift: shell.report.repair_largest_shift,
        issues,
        closed: census.boundary == 0 && census.non_manifold == 0,
    })
}
