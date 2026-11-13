//! `mesh.remesh`: a clean, closed, manifold, consistently oriented surface rebuilt from a messy
//! closed scan (`geom::remesh`), as a new mesh. The original stays; the report says what the
//! remesh cost (deviation both ways) and what it removed.
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_geom::remesh::{self, Deviation, Params, RemeshReport};

/// Parameters of `mesh.remesh`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RemeshIn {
    /// Scan mesh id or name (closed, or with holes up to `max_hole` wide).
    pub object: ObjRef,
    /// Tolerance the result is held to against the scan (mm, default 0.2); sets the default voxel.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Voxel edge (mm). Default 1.5 x tolerance, made finer when the scan has a thin wall. A thin wall with an explicit voxel is refused.
    #[serde(default)]
    pub voxel: Option<f64>,
    /// Tunnels, bridges and spurs up to about this wide are removed (mm, default two voxels). Walls thicker than this are kept.
    #[serde(default)]
    pub min_feature: Option<f64>,
    /// Holes whose loop is at most this wide (mm, default 5) are filled first; a wider opening refuses the remesh.
    #[serde(default)]
    pub max_hole: Option<f64>,
    /// Pieces with fewer triangles than this share of the largest are debris and dropped (default 0.01).
    #[serde(default)]
    pub min_component_fraction: Option<f64>,
    /// Name of the new mesh (default "remesh").
    #[serde(default)]
    pub name: Option<String>,
}

/// One direction of the measured deviation (mm).
#[derive(Clone, Copy, Debug, Deserialize, Serialize, JsonSchema)]
pub struct DeviationFigures {
    /// Points measured.
    pub samples: usize,
    /// Every vertex was measured (else an even sample, whose maximum can miss one removed fin).
    pub exact: bool,
    /// Median.
    pub p50: f64,
    /// 95th percentile.
    pub p95: f64,
    /// 99th percentile.
    pub p99: f64,
    /// Largest.
    pub max: f64,
}

impl From<Deviation> for DeviationFigures {
    fn from(d: Deviation) -> Self {
        Self { samples: d.samples, exact: d.exact, p50: d.p50, p95: d.p95, p99: d.p99, max: d.max }
    }
}

/// Result of `mesh.remesh`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct RemeshOut {
    /// The new mesh.
    pub id: ObjectId,
    /// Its triangles.
    pub triangles: usize,
    /// Voxel edge used (mm).
    pub voxel: f64,
    /// The voxel was made finer than the default because of a thin wall.
    pub voxel_refined: bool,
    /// Tunnels, bridges and spurs up to this wide were removed (mm).
    pub feature_removed: f64,
    /// Thickness of the thinnest wall probed (mm).
    pub thickness: Option<f64>,
    /// Holes filled before the remesh.
    pub holes_filled: usize,
    /// Triangles of debris dropped before the remesh.
    pub debris_triangles: usize,
    /// The scan was inside out and was read reversed.
    pub flipped: bool,
    /// Grid cells along x, y, z.
    pub grid: [usize; 3],
    /// Ray crossings the scan conversion made.
    pub crossings: usize,
    /// Columns whose crossings did not alternate (fins, coincident shells).
    pub irregular_columns: usize,
    /// Pinched vertices split to make the surface manifold.
    pub pinches_split: usize,
    /// Genus per piece of the scan.
    pub genus_before: Vec<Option<usize>>,
    /// Genus per piece of the result.
    pub genus_after: Vec<Option<usize>>,
    /// Non-manifold edges of the scan.
    pub non_manifold_before: usize,
    /// Bytes of the voxel stage (not the whole process).
    pub peak_bytes: usize,
    /// Scan vertices to the new surface: the error of the result and, in its tail, what was removed.
    pub original_to_result: DeviationFigures,
    /// New vertices to the scan surface.
    pub result_to_original: DeviationFigures,
    /// Share of the scan vertices farther from the result than the tolerance: fins and bridges removed on purpose (or lost), not error.
    pub removed_share: f64,
    /// Duplicate triangles dropped before the remesh.
    pub duplicate_triangles: usize,
    /// Triangles of the result that cross another (of an even sample of the result).
    pub self_intersections: usize,
    /// Triangles tested for that.
    pub self_intersection_samples: usize,
    /// What the caller should know (thin features removed).
    pub warnings: Vec<String>,
}

impl RemeshOut {
    fn new(id: ObjectId, r: RemeshReport) -> Self {
        Self {
            id,
            triangles: r.triangles,
            voxel: r.voxel,
            voxel_refined: r.voxel_refined,
            feature_removed: r.feature_removed,
            thickness: r.thickness,
            holes_filled: r.holes_filled,
            debris_triangles: r.debris_triangles,
            flipped: r.flipped,
            grid: r.grid,
            crossings: r.crossings,
            irregular_columns: r.irregular_columns,
            pinches_split: r.pinches_split,
            genus_before: r.genus_before,
            genus_after: r.genus_after,
            non_manifold_before: r.non_manifold_before,
            peak_bytes: r.peak_bytes,
            original_to_result: r.original_to_result.into(),
            result_to_original: r.result_to_original.into(),
            removed_share: r.removed_share,
            duplicate_triangles: r.duplicate_triangles,
            self_intersections: r.self_intersections,
            self_intersection_samples: r.self_intersection_samples,
            warnings: r.warnings,
        }
    }
}

pub(crate) fn remesh_cmd(s: &mut Session, i: RemeshIn) -> CmdResult<RemeshOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let scan = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let d = Params::default();
    let params = Params {
        tolerance: i.tolerance.unwrap_or(d.tolerance),
        voxel: i.voxel,
        min_feature: i.min_feature,
        max_hole: i.max_hole.unwrap_or(d.max_hole),
        min_component_fraction: i.min_component_fraction.unwrap_or(d.min_component_fraction),
        ..d
    };
    let (mesh, report) = remesh::remesh(&scan, &params)?;
    let new = s.doc.add_mesh(i.name.as_deref().unwrap_or("remesh"), mesh);
    Ok(RemeshOut::new(new, report))
}

/// The `mesh.remesh` command.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "mesh.remesh",
    "Remesh scan",
    "Mesh",
    true,
    "Rebuilds a clean, closed, manifold, consistently oriented surface from the messy closed scan `object` as a new mesh: the solid is scan-converted onto a sparse voxel grid (voxel 1.5 x `tolerance` by default, finer for thin walls), closing and opening remove tunnels, bridges, fins and self-intersections up to `min_feature` wide, surface nets extract the boundary and the vertices are projected back onto the scan, so accuracy is that of the scan. Genuine handles and walls of at least two voxels beyond `min_feature` are kept. Refuses open scans (no skin is invented over an opening wider than `max_hole`), resolutions beyond the memory limits, and walls the voxel cannot resolve. Reports the deviation from the scan both ways.",
    RemeshIn,
    RemeshOut,
    remesh_cmd
)];
