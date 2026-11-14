//! Folds of a scan: small groups of triangles that face against the surface around them and pass
//! through the sheet (a flap turned over, a knot of a few triangles, a sliver on the rim). The sheet
//! crosses itself there, so a shell of constant thickness cannot be built over it: the offset of the
//! two sheets collides however small the wall. It is a defect of the scan, not geometry.
//!
//! What counts. A triangle faces against its surroundings when its normal is more than [`FLIP_COS`]
//! past a right angle from the area-weighted normal of the triangles within [`CELL_EDGES`] median
//! edges around it, and those triangles face one way ([`COHERENCE`]: a thin plate or a sharp edge is
//! not told apart by chance). Flipped triangles that share a vertex form a group. Only a group whose
//! triangles pass through another triangle (an edge pierces it; folded flat onto a neighbour) is a
//! fold: steep real geometry (an undercut, a fin standing on a panel, a bump) never crosses itself.
//!
//! What is removed. A fold at most [`MAX_EXTENT_EDGES`] median edges across, of at most
//! [`MAX_TRIANGLES`] triangles, within [`MAX_HEIGHT_EDGES`] median edges of the surface around it is
//! taken out together with the rings of triangles around it (see `remove`) and the hole is filled with the
//! existing smooth hole fill. The fill is accepted only when the clean surface it replaced stays within
//! [`FILL_ACCEPT_SHARE`] of the tolerance (a crease or the foot of a boss that the smooth patch would round
//! off is not a fold to remove); otherwise the group is put back and counted. A group on the rim has no
//! margin and only trims the rim; a margin never reaches the rim, and a group at a hub vertex or with a
//! margin of more than four times [`MAX_TRIANGLES`] triangles is left, and so is a group beside a crease or a
//! corner (neighbouring triangles more than 30 degrees apart in the two rings around it: a smooth patch would
//! round it off). Larger folds are left and reported.
//!
//! What is reported: the folds found, removed and left; the area removed and its share of the scan; the
//! height of the removed folds above the new surface and the deviation of the fill from the clean surface
//! it replaced; and the scan as it was where it was removed (`removed_mesh`), so that a judge can measure
//! against the original.
mod detect;
mod remove;

use crate::{Cancel, Mesh, edit};
use detect::{Incidence, Site, facets, flipped, median_edge, sites};
use remove::{Attempt, near_a_feature, try_remove, with_margin};

/// A triangle faces against its surroundings at more than this cosine past a right angle
/// (a cosine of -0.3: 107 degrees from the surface around it; a thin plate standing on a panel is at
/// 90 degrees and is not touched).
pub const FLIP_COS: f64 = 0.3;
/// The surroundings face one way when the summed area-weighted normals are at least this share of the
/// summed areas (a sharp edge or two faces of a thin plate have less).
pub const COHERENCE: f64 = 0.5;
/// A removed group is at most this many median edges across (the cell that says which way the surface
/// faces has the same size, so a fold is a minority in it). The folds that were removed on the real
/// scans of the local corpus are 1 to 7 edges across; larger ones stay and are reported.
pub const MAX_EXTENT_EDGES: f64 = 8.0;
/// A removed group has at most this many triangles.
pub const MAX_TRIANGLES: usize = 200;
/// A removed group lies within this many median edges of the surface around it.
pub const MAX_HEIGHT_EDGES: f64 = 1.5;
/// Side of the cells that say which way the surface faces around a triangle, in median edges.
pub const CELL_EDGES: f64 = 8.0;
/// A fill is accepted when the clean surface it replaced stays within this share of the tolerance.
pub const FILL_ACCEPT_SHARE: f64 = 0.75;
/// Rounds of finding, removing and filling (removing a fold can show one behind it).
const MAX_ROUNDS: usize = 3;
/// Attempts to remove a set of groups whose holes can all be filled and whose patches stay close.
const MAX_ATTEMPTS: usize = 4;
/// Folds left that are listed in the report (the widest first).
const MAX_LISTED: usize = 10;

/// One group of triangles facing against their surroundings.
#[derive(Clone, Debug, PartialEq)]
pub struct FoldInfo {
    /// Triangles of the group.
    pub triangles: usize,
    /// Size of its bounding box (mm).
    pub extent: f64,
    /// Distance from the surface around it (mm; infinite when it has no surroundings).
    pub height: f64,
    /// Area of the group (mm²).
    pub area: f64,
    /// A triangle of the group lies on the rim of the scan.
    pub on_rim: bool,
    /// A triangle of the group passes through a triangle it shares no vertex with (the scan crosses itself there).
    pub crossing: bool,
    /// Small enough to be removed.
    pub small: bool,
}

/// What the fold removal did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FoldReport {
    /// Groups of triangles facing against their surroundings found before anything was removed (whether they cross the sheet or not).
    pub found: usize,
    /// Groups removed.
    pub removed: usize,
    /// Triangles of the scan taken out with them (margins included).
    pub removed_triangles: usize,
    /// The area of the scan's own triangles among them (mm²).
    pub removed_area: f64,
    /// That area over the area of the scan before.
    pub removed_area_share: f64,
    /// The largest distance of a vertex of a removed fold from the surface that replaced it (mm): how much of
    /// the scan stood out from the surface there.
    pub largest_removed_height: f64,
    /// The largest distance of any vertex or triangle centre of the scan that was taken out from the patch that
    /// replaced it (mm): how far the repaired scan differs from the scan as it was.
    pub largest_fill_deviation: f64,
    /// The same for the clean surface that went with the margins (not what touches a fold): a fill that would
    /// exceed [`FILL_ACCEPT_SHARE`] of the tolerance there is not made.
    pub largest_clean_deviation: f64,
    /// Triangles added by filling the holes.
    pub added_triangles: usize,
    /// Small folds that were not removed because the hole could not be filled, the patch strayed from the
    /// clean surface, or the margin was too large (put back; counted once, at the end).
    pub unfillable: usize,
    /// Folds (groups that cross the sheet) remaining after the repair (too large, too far from the surface, or not removed), widest first, at most ten.
    pub left: Vec<FoldInfo>,
    /// All folds remaining.
    pub left_total: usize,
    /// The scan as it was where the folds were taken out: the triangles of the original mesh, as a soup of
    /// triangles with their own vertices and winding (empty when nothing was removed).
    pub removed_mesh: Mesh,
    /// The call was cancelled; what was done before stays (every round is complete or not applied).
    pub cancelled: bool,
}

fn small(site: &Site, edge: f64) -> bool {
    site.crossing
        && site.triangles.len() <= MAX_TRIANGLES
        && site.extent <= MAX_EXTENT_EDGES * edge
        && site.height <= MAX_HEIGHT_EDGES * edge
}

/// Finds the groups of `mesh`.
fn survey(mesh: &Mesh, edge: f64) -> (Vec<Site>, Vec<bool>) {
    let inc = Incidence::new(mesh);
    let f = facets(mesh);
    let mask = flipped(mesh, &f, CELL_EDGES * edge, FLIP_COS, COHERENCE);
    let all = sites(mesh, &inc, &f, &mask);
    let small_ones = all.iter().map(|s| small(s, edge)).collect();
    (all, small_ones)
}

fn info_of(s: &Site, edge: f64) -> FoldInfo {
    FoldInfo {
        triangles: s.triangles.len(),
        extent: s.extent,
        height: s.height,
        area: s.area,
        on_rim: s.on_rim,
        crossing: s.crossing,
        small: small(s, edge),
    }
}

/// Every group of triangles facing against their surroundings, with its measures (nothing is changed).
#[must_use]
pub fn find_folds(mesh: &Mesh) -> Vec<FoldInfo> {
    if mesh.triangles.is_empty() || mesh.validate().is_err() {
        return Vec::new();
    }
    let edge = median_edge(mesh);
    survey(mesh, edge).0.iter().map(|s| info_of(s, edge)).collect()
}

/// The scan being repaired: which of its triangles are the scan as it was, and the tolerance.
struct State {
    origin: Vec<bool>,
    tolerance: f64,
}

/// Removes the small folds of `mesh` and fills their holes; see the module documentation. `tolerance` (mm)
/// bounds how far a patch may stray from the surface it replaces. The first `scan_triangles` triangles are
/// the scan as it was recorded, the rest are patches (hole filling): only the scan's own triangles and
/// vertices count as points of the scan in the heights, deviations, area and `removed_mesh` (a fold in a patch
/// is a defect of the fill, not of the scan, and there is no scan point there to be measured against). An invalid or empty mesh, or a
/// tolerance that is not a positive number, leaves the mesh alone.
pub fn remove_folds(mesh: &mut Mesh, tolerance: f64, cancel: &Cancel, scan_triangles: usize) -> FoldReport {
    let mut report = FoldReport::default();
    if mesh.triangles.is_empty() || mesh.validate().is_err() || !(tolerance.is_finite() && tolerance > 0.0) {
        return report;
    }
    let (edge, area_before) = (median_edge(mesh), mesh.area());
    let origin = (0..mesh.triangles.len()).map(|t| t < scan_triangles).collect();
    let mut state = State { origin, tolerance };
    let mut all_left: Vec<FoldInfo> = Vec::new();
    for round in 0..=MAX_ROUNDS {
        if cancel.is_cancelled() {
            report.cancelled = true;
            break;
        }
        let (all, is_small) = survey(mesh, edge);
        if round == 0 {
            report.found = all.len();
        }
        let take: Vec<&Site> = all.iter().zip(&is_small).filter(|(_, s)| **s).map(|(a, _)| a).collect();
        let progress =
            round < MAX_ROUNDS && !take.is_empty() && remove_checked(mesh, &mut state, &take, &mut report, cancel);
        if !progress {
            all_left = all.iter().filter(|s| s.crossing).map(|s| info_of(s, edge)).collect();
            report.unfillable = take.len();
            break;
        }
    }
    report.removed_area_share = if area_before > 0.0 { report.removed_area / area_before } else { 0.0 };
    all_left.sort_by(|a, b| b.extent.total_cmp(&a.extent));
    report.left_total = all_left.len();
    all_left.truncate(MAX_LISTED);
    report.left = all_left;
    report
}

/// The triangles `ids` of the original mesh, appended to `into` as a soup.
fn record_removed(mesh: &Mesh, ids: &[u32], into: &mut Mesh) {
    for corners in ids.iter().filter_map(|&t| mesh.triangle(t as usize)) {
        let base = into.positions.len() as u32;
        into.positions.extend(corners);
        into.triangles.push([base, base + 1, base + 2]);
    }
}

/// Removes `take`, retrying without the groups whose hole stays open or whose patch strays; false when
/// nothing was removed (or the call was cancelled).
fn remove_checked(
    mesh: &mut Mesh,
    state: &mut State,
    take: &[&Site],
    report: &mut FoldReport,
    cancel: &Cancel,
) -> bool {
    let inc = Incidence::new(mesh);
    let neighbours = mesh.triangle_neighbours();
    let rim_tris: Vec<bool> = neighbours.iter().map(|n| n.contains(&u32::MAX)).collect();
    // a group at a hub, with a margin that is too large, or beside a crease or a boss is left
    let plans: Vec<Option<Vec<u32>>> = take
        .iter()
        .map(|s| with_margin(mesh, &inc, &rim_tris, s))
        .map(|p| p.filter(|plan| !near_a_feature(mesh, &inc, &neighbours, plan)))
        .collect();
    let mut skip: Vec<bool> = plans.iter().map(Option::is_none).collect();
    let tris: Vec<Vec<u32>> = plans.into_iter().map(Option::unwrap_or_default).collect();
    for _ in 0..MAX_ATTEMPTS {
        if cancel.is_cancelled() {
            report.cancelled = true;
            return false;
        }
        let attempt = try_remove(mesh, &state.origin, take, &tris, &skip, FILL_ACCEPT_SHARE * state.tolerance);
        if attempt.failed.is_empty() {
            return commit(mesh, state, &skip, attempt, report);
        }
        for g in attempt.failed {
            if let Some(s) = skip.get_mut(g) {
                *s = true;
            }
        }
    }
    false
}

/// Applies an attempt in which every group that was not skipped came out clean.
fn commit(mesh: &mut Mesh, state: &mut State, skip: &[bool], attempt: Attempt, report: &mut FoldReport) -> bool {
    let gone: Vec<usize> = (0..skip.len()).filter(|&g| !skip.get(g).copied().unwrap_or(true)).collect();
    if gone.is_empty() {
        return false;
    }
    report.removed += gone.len();
    for t in gone.iter().filter_map(|&g| attempt.taken.get(g)) {
        report.removed_triangles += t.triangles;
        report.removed_area += t.area;
        report.largest_removed_height = report.largest_removed_height.max(t.height);
        report.largest_fill_deviation = report.largest_fill_deviation.max(t.fill_deviation);
        report.largest_clean_deviation = report.largest_clean_deviation.max(t.clean_deviation);
    }
    report.added_triangles += attempt.added;
    record_removed(mesh, &attempt.removed_original, &mut report.removed_mesh);
    state.origin = attempt.origin;
    *mesh = attempt.mesh;
    edit::compact(mesh);
    true
}

#[cfg(test)]
mod tests;
