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
//! taken out together with [`MARGIN_RINGS`] rings of triangles around it (the vertices of the fold
//! are tangled with the sheet it crosses; the hole must be bounded by clean surface) and the hole is
//! filled with the existing smooth hole fill. A group on the rim has no margin and only trims the
//! rim; a margin never reaches the rim. A group whose hole cannot be filled (pinched, not a disk, a
//! patch with a flat triangle) is put back and counted. Larger folds are left and reported.
mod detect;

use crate::{Mesh, edit};
use detect::{Incidence, Site, facets, flipped, median_edge, sites};

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
/// Rounds of finding, removing and filling (removing a fold can show one behind it).
/// Side of the cells that say which way the surface faces around a triangle, in median edges.
pub const CELL_EDGES: f64 = 8.0;
/// Rings of triangles removed around a group, so that the hole is bounded by clean surface.
const MARGIN_RINGS: usize = 2;
const MAX_ROUNDS: usize = 3;
/// Attempts to remove a set of groups whose holes can all be filled.
const MAX_ATTEMPTS: usize = 4;
/// Longest hole filled after removing a group (boundary edges).
const FILL_MAX_EDGES: usize = 2 * MAX_TRIANGLES;
/// Groups left that are listed in the report (the largest first).
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
    /// Triangles removed with them.
    pub removed_triangles: usize,
    /// Their area (mm²).
    pub removed_area: f64,
    /// The largest distance of a removed group from the surface around it (mm).
    pub largest_removed_height: f64,
    /// Triangles added by filling the holes.
    pub added_triangles: usize,
    /// Groups that were small enough but whose hole could not be filled: put back.
    pub unfillable: usize,
    /// Folds (groups that cross the sheet) remaining after the repair (too large, too far from the surface, or not filled), widest first, at most ten.
    pub left: Vec<FoldInfo>,
    /// All folds remaining.
    pub left_total: usize,
}

fn small(site: &Site, edge: f64) -> bool {
    site.crossing
        && site.triangles.len() <= MAX_TRIANGLES
        && site.extent <= MAX_EXTENT_EDGES * edge
        && site.height <= MAX_HEIGHT_EDGES * edge
}

/// The triangles of `site` and the rings of triangles around them ([`MARGIN_RINGS`]), so that the
/// hole's edge lies on clean surface and not on the vertices the fold is tangled with. The margin stops
/// short of the rim (a hole that reaches it would notch the rim), and a group on the rim has none.
fn with_margin(mesh: &Mesh, inc: &Incidence, rim_tris: &[bool], site: &Site) -> Vec<u32> {
    let mut set: std::collections::BTreeSet<u32> = site.triangles.iter().copied().collect();
    if site.on_rim {
        return set.into_iter().collect();
    }
    for _ in 0..MARGIN_RINGS {
        let ring: std::collections::BTreeSet<u32> = set
            .iter()
            .filter_map(|&t| mesh.triangles.get(t as usize))
            .flatten()
            .flat_map(|&v| inc.row(v).iter().copied())
            .collect();
        if ring.iter().any(|&t| rim_tris.get(t as usize).copied().unwrap_or(true)) {
            break;
        }
        set.extend(ring);
    }
    set.into_iter().collect()
}

/// What one attempt to take groups out made.
struct Attempt {
    mesh: Mesh,
    /// Groups (indices into the list given) whose hole stayed open.
    failed: Vec<usize>,
    added: usize,
    /// Triangles and area taken out, per group.
    taken: Vec<(usize, f64)>,
}

/// Vertices on a boundary loop of `mesh`.
fn rim_vertices(mesh: &Mesh) -> Vec<bool> {
    let mut r = vec![false; mesh.positions.len()];
    for &v in edit::boundary_loops(mesh).iter().flatten() {
        if let Some(x) = r.get_mut(v as usize) {
            *x = true;
        }
    }
    r
}

/// The mesh with the triangles of `remove` (and their margin) taken out and the holes they leave filled.
fn try_remove(mesh: &Mesh, remove: &[&Site], skip: &[bool]) -> Attempt {
    let inc = Incidence::new(mesh);
    let rim_tris: Vec<bool> = mesh.triangle_neighbours().iter().map(|n| n.contains(&u32::MAX)).collect();
    let mut vertex_group = vec![u32::MAX; mesh.positions.len()];
    let mut dead = vec![false; mesh.triangles.len()];
    let mut taken = vec![(0usize, 0.0f64); remove.len()];
    for (g, site) in remove.iter().enumerate().filter(|(g, _)| !skip.get(*g).copied().unwrap_or(false)) {
        for t in with_margin(mesh, &inc, &rim_tris, site) {
            let (Some(d), Some(c)) = (dead.get_mut(t as usize), mesh.triangles.get(t as usize)) else { continue };
            if !*d && let Some(x) = taken.get_mut(g) {
                *x = (x.0 + 1, x.1 + mesh.face_area(t as usize));
            }
            *d = true;
            for &v in c {
                if let Some(s) = vertex_group.get_mut(v as usize) {
                    *s = g as u32;
                }
            }
        }
    }
    let rim = rim_vertices(mesh);
    let mut out = mesh.clone();
    let mut k = dead.iter();
    out.triangles.retain(|_| !k.next().copied().unwrap_or(false));
    // only the hole a group left (never the rim of the scan, nor a window that was already there)
    let on_rim = |v: &u32| rim.get(*v as usize).copied().unwrap_or(false);
    let touches = |_: &Mesh, lp: &[u32]| {
        !lp.iter().any(on_rim) && lp.iter().any(|&v| vertex_group.get(v as usize).is_some_and(|&g| g != u32::MAX))
    };
    let before = out.triangles.len();
    crate::holes::fill_holes_filtered(&mut out, FILL_MAX_EDGES, true, true, &touches);
    let added = out.triangles.len() - before;
    // a short loop of the new boundary that does not run along the old rim is a hole that stayed open
    let mut failed: Vec<usize> = edit::boundary_loops(&out)
        .iter()
        .filter(|lp| lp.len() <= FILL_MAX_EDGES && !lp.iter().any(|&v| rim.get(v as usize).copied().unwrap_or(false)))
        .flat_map(|lp| lp.iter().filter_map(|&v| vertex_group.get(v as usize).copied().filter(|&g| g != u32::MAX)))
        .map(|g| g as usize)
        .collect();
    // a patch with a triangle without a normal would leave a vertex that no wall can be built on
    let flat = (before..out.triangles.len()).filter(|&i| out.face_normal(i).is_none());
    failed.extend(
        flat.filter_map(|i| out.triangles.get(i))
            .flatten()
            .filter_map(|&v| vertex_group.get(v as usize).copied().filter(|&g| g != u32::MAX))
            .map(|g| g as usize),
    );
    failed.sort_unstable();
    failed.dedup();
    Attempt { mesh: out, failed, added, taken }
}

/// Finds the groups of `mesh`, small ones first.
fn survey(mesh: &Mesh, edge: f64) -> (Vec<Site>, Vec<bool>) {
    let inc = Incidence::new(mesh);
    let f = facets(mesh);
    let mask = flipped(mesh, &f, CELL_EDGES * edge, FLIP_COS, COHERENCE);
    let all = sites(mesh, &inc, &f, &mask);
    let small_ones = all.iter().map(|s| small(s, edge)).collect();
    (all, small_ones)
}

/// Removes the small folds of `mesh` and fills their holes; see the module documentation. An invalid
/// or empty mesh is left alone.
pub fn remove_folds(mesh: &mut Mesh) -> FoldReport {
    let mut report = FoldReport::default();
    if mesh.triangles.is_empty() || mesh.validate().is_err() {
        return report;
    }
    let edge = median_edge(mesh);
    let mut all_left: Vec<FoldInfo> = Vec::new();
    for round in 0..=MAX_ROUNDS {
        let (all, is_small) = survey(mesh, edge);
        if round == 0 {
            report.found = all.len();
        }
        let take: Vec<&Site> = all.iter().zip(&is_small).filter(|(_, s)| **s).map(|(a, _)| a).collect();
        let done = round == MAX_ROUNDS || take.is_empty() || remove_checked(mesh, &take, &mut report).is_none();
        if done {
            all_left = all.iter().filter(|s| s.crossing).map(|s| info_of(s, edge)).collect();
            break;
        }
    }
    all_left.sort_by(|a, b| b.extent.total_cmp(&a.extent));
    report.left_total = all_left.len();
    all_left.truncate(MAX_LISTED);
    report.left = all_left;
    report
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

/// Removes `take`, retrying without the groups whose hole stays open; `None` when nothing was removed.
fn remove_checked(mesh: &mut Mesh, take: &[&Site], report: &mut FoldReport) -> Option<()> {
    let mut skip = vec![false; take.len()];
    for _ in 0..MAX_ATTEMPTS {
        let attempt = try_remove(mesh, take, &skip);
        if attempt.failed.is_empty() {
            let gone: Vec<usize> = (0..take.len()).filter(|&g| !skip.get(g).copied().unwrap_or(true)).collect();
            if gone.is_empty() {
                return None;
            }
            report.removed += gone.len();
            for &g in &gone {
                let (n, area) = attempt.taken.get(g).copied().unwrap_or((0, 0.0));
                report.removed_triangles += n;
                report.removed_area += area;
                report.largest_removed_height =
                    report.largest_removed_height.max(take.get(g).map_or(0.0, |s| s.height));
            }
            report.added_triangles += attempt.added;
            report.unfillable += skip.iter().filter(|s| **s).count();
            *mesh = attempt.mesh;
            edit::compact(mesh);
            return Some(());
        }
        for g in attempt.failed {
            if let Some(s) = skip.get_mut(g) {
                *s = true;
            }
        }
    }
    report.unfillable += take.len();
    None
}

#[cfg(test)]
mod tests;
