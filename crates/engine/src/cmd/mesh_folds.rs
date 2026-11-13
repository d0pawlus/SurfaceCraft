//! The report of the fold removal of `mesh.clean` (see `surfacecraft_geom::folds`).
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::folds::{FoldInfo, FoldReport};

/// A group of triangles that faces against the surface around it and was left alone.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FoldOut {
    /// Triangles of the group.
    pub triangles: usize,
    /// Size of its bounding box (mm).
    pub extent: f64,
    /// Distance from the surface around it (mm); null when it has none.
    pub height: Option<f64>,
    /// Area of the group (mm²).
    pub area: f64,
    /// The group lies on the rim of the scan.
    pub on_rim: bool,
}

impl From<&FoldInfo> for FoldOut {
    fn from(f: &FoldInfo) -> Self {
        Self {
            triangles: f.triangles,
            extent: f.extent,
            height: Some(f.height).filter(|h| h.is_finite()),
            area: f.area,
            on_rim: f.on_rim,
        }
    }
}

/// What the removal of folds did: small groups of triangles that face against the surface around them (a flap turned over, a knot, a sliver on the rim), where the scan passes through itself and no wall can be built over it. A group is a fold when its triangles pass through other triangles of the scan; it is removed when it is at most 8 median edges across, has at most 200 triangles and lies within 1.5 median edges of the surface around it (with two rings of triangles around it, so that the hole is bounded by clean surface); its hole is filled (a hole that cannot be filled puts the group back). The rim is never closed. Larger folds stay and are listed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct FoldsOut {
    /// Groups of triangles facing against the surface around them found before anything was removed (whether the scan crosses itself there or not; only those that do are folds).
    pub found: usize,
    /// Groups removed.
    pub removed: usize,
    /// Triangles removed with them.
    pub removed_triangles: usize,
    /// Their area (mm²).
    pub removed_area: f64,
    /// The largest distance of a removed group from the surface around it (mm).
    pub largest_removed_height: f64,
    /// Triangles added by filling the holes they left.
    pub added_triangles: usize,
    /// Small groups whose hole could not be filled: put back.
    pub unfillable: usize,
    /// Folds still there afterwards (too large or too far from the surface): the shell of `scan.to_solid` can still refuse because of them.
    pub left: usize,
    /// The largest of them, widest first (at most ten).
    pub left_largest: Vec<FoldOut>,
}

impl From<&FoldReport> for FoldsOut {
    fn from(r: &FoldReport) -> Self {
        Self {
            found: r.found,
            removed: r.removed,
            removed_triangles: r.removed_triangles,
            removed_area: r.removed_area,
            largest_removed_height: r.largest_removed_height,
            added_triangles: r.added_triangles,
            unfillable: r.unfillable,
            left: r.left_total,
            left_largest: r.left.iter().map(FoldOut::from).collect(),
        }
    }
}
