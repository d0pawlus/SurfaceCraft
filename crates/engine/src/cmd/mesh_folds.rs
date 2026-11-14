//! The report of the fold removal of `mesh.clean` (see `surfacecraft_geom::folds`).
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::folds::{FoldInfo, FoldReport, Refused};

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

/// Why the folds that cross the sheet and stayed stayed.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct RefusedOut {
    /// Wider than 8 median edges, more than 200 triangles, or higher than 1.5 median edges above the surface around it.
    pub too_large: usize,
    /// At a hub vertex (a fan of more than 64 triangles) or with a margin of more than 800 triangles.
    pub hub_or_margin: usize,
    /// A crease or a corner runs through the two rings of triangles around it: a smooth patch would round it off.
    pub beside_feature: usize,
    /// The patch would replace clean surface (or, for a few triangles, what is left of the ring) by more than three quarters of the tolerance.
    pub fill_refused: usize,
    /// The hole could not be filled, or the patch has a flat triangle.
    pub unfillable: usize,
}

impl From<&Refused> for RefusedOut {
    fn from(r: &Refused) -> Self {
        Self {
            too_large: r.too_large,
            hub_or_margin: r.hub_or_margin,
            beside_feature: r.beside_feature,
            fill_refused: r.fill_refused,
            unfillable: r.unfillable,
        }
    }
}

/// What the removal of folds did: small groups of triangles that face against the surface around them (a flap turned over, a knot, a sliver on the rim), where the scan passes through itself and no wall can be built over it. A group is a fold when its triangles pass through other triangles of the scan; it is removed when it is at most 8 median edges across, has at most 200 triangles and lies within 1.5 median edges of the surface around it (with one or two rings of triangles around it, so that the hole is bounded by clean surface) and the smooth patch stays within three quarters of the tolerance of the clean surface it replaces; a group at a hub, beside a crease or a corner (neighbouring triangles more than 30 degrees apart around it) or with a patch that would stray is left; its hole is filled (a hole that cannot be filled puts the group back). The rim is never closed. Larger folds stay and are listed. The scan object is changed in place, like the hole filling of the same call, and by the same undo step.
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
    /// That area over the area of the scan before.
    pub removed_area_share: f64,
    /// The largest distance of a vertex of a removed fold from the surface that replaced it (mm): how far the scan stood out of the surface there. The scan is judged as it was, so this is part of the deviation of the verdict.
    pub largest_removed_height: f64,
    /// The largest distance of any vertex or triangle centre of the scan that was taken out from the patch that replaced it (mm): how far the repaired scan differs from the scan as it was. A fill that would move the clean surface around a fold (not what touches it) by more than three quarters of the tolerance is not made.
    pub largest_fill_deviation: f64,
    /// Triangles added by filling the holes they left.
    pub added_triangles: usize,
    /// The folds that stayed, by reason (counted once, at the end).
    pub refused: RefusedOut,
    /// Triangles that were not looked at because a vertex far beyond the rest of the scan put them out of range.
    pub skipped_triangles: usize,
    /// The vertices of the removed triangles that the judge of `scan.to_solid` measured against the scan layer of the body (0 when the call did not judge).
    #[serde(default)]
    pub removed_points: usize,
    /// Of them, the points not measured because the nearest face of the body faces the other way: they lie deeper in the wall than its middle (a fold standing that deep into the wall).
    #[serde(default)]
    pub removed_points_not_facing: usize,
    /// Their share within the tolerance.
    #[serde(default)]
    pub removed_points_within: f64,
    /// The largest distance of one of them from the body (mm): the scan as it was, not the repaired one.
    #[serde(default)]
    pub removed_points_max_deviation: f64,
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
            removed_area_share: r.removed_area_share,
            largest_removed_height: r.largest_removed_height,
            largest_fill_deviation: r.largest_fill_deviation,
            added_triangles: r.added_triangles,
            refused: RefusedOut::from(&r.refused),
            skipped_triangles: r.skipped_triangles,
            removed_points: 0,
            removed_points_not_facing: 0,
            removed_points_within: 0.0,
            removed_points_max_deviation: 0.0,
            left: r.left_total,
            left_largest: r.left.iter().map(FoldOut::from).collect(),
        }
    }
}
