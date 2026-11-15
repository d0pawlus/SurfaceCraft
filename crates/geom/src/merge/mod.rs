//! Merging aligned scans of one part into one mesh: overlaps removed, not stacked; the seams closed
//! without making up surface; every removal measured against the result.
//!
//! Method (zippering after Turk and Levoy 1994, with the seam closed by ball pivoting after
//! Bernardini et al. 1999 instead of clipping):
//!
//! 1. Each scan is checked (orientation, non-manifold edges) and every vertex gets its distance
//!    along the edges from the border of its scan.
//! 2. Every pair of scans is compared on points sampled over each by area: where one lies on facing
//!    surface of the other within [`REACH_TOLERANCES`] tolerances, the distances give the residual
//!    of the pair (rms, 95th percentile, worst). Scans that disagree there (more than
//!    [`MISALIGNED_SHARE`] of their shared surface farther apart than the tolerance) are not
//!    averaged and not stacked: the merge refuses with `Misaligned` and the numbers. Scans whose
//!    normals oppose where they meet (one is turned inside out) are refused the same way.
//! 3. Ownership ([`own`]): where several scans cover the surface, the one in which it lies farthest
//!    from its own border keeps it; the others lose those vertices and every triangle touching
//!    one. Triangles on the edge of a kept region that still lie wholly on another scan's kept
//!    surface are eaten back (Turk and Levoy's removal of redundant triangles).
//! 4. The kept triangles are put together ([`union`]); seam vertices of different scans that
//!    coincide are welded, and removed triangles whose corners all exist in the union at exactly
//!    their positions are restored (pieces of one scan share their tessellation).
//! 5. The band between the kept regions is closed by ball pivoting from the cut edges ([`seam`]).
//!    A triangle is built only on surface a scan saw and where it crosses nothing.
//! 6. Checks ([`check`]): the removed triangles are sampled by area and measured against the result;
//!    added triangles are tested for crossings; the topology before and after is counted.
//!
//! Kept vertices stay where they are, except that a seam vertex of a later scan moves onto a
//! coinciding seam vertex of an earlier scan (see [`weld`]: by at most a quarter of the tolerance and a tenth of the median
//! edge). Scans that share nothing are a union of pieces, reported as such. The work is
//! O(T log T) in the triangles of all scans; nothing is quadratic in them.
mod check;
mod fill;
mod guard;
mod orient;
mod overlap;
mod own;
mod prep;
mod recover;
mod seam;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_round2;
#[cfg(test)]
mod tests_units;
mod union;
mod weld;

pub use check::{Coverage, DUPLICATE_SHARE, MAX_REMOVED_SAMPLES};
pub use fill::SMALL_LOOP_EDGES;
pub use guard::{MAX_CELL_SPAN, SUPPORT_SLOPE};
pub use orient::FLIPPED_REFUSE_SHARE;
pub use overlap::{PAIR_SAMPLES, PairResidual};
pub use prep::{INVERTED_MIN_SHARE, Topology, topology};
pub use recover::Recovered;
pub use seam::{RADII_EDGES, SeamReport};
pub use weld::{AREA_KEEP, WELD_EDGE_SHARE, WELD_SHARE};

use crate::{Bvh, Cancel, Mesh, Point3};
use prep::Scan;
use union::{Origin, Union};

/// The overlap of two scans is searched up to this many tolerances away (so that scans misaligned
/// by several tolerances are seen as misaligned, not as sharing nothing).
pub const REACH_TOLERANCES: f64 = 8.0;
/// A pair whose shared surface lies farther apart than the tolerance on more than this share is
/// misaligned.
pub const MISALIGNED_SHARE: f64 = 0.1;
/// Samples on shared surface a pair needs before it is judged (misaligned, opposed).
pub const MIN_SHARED_SAMPLES: usize = 50;
/// Below this share of the removed area within the tolerance of the result, `RemovedUncovered` is raised.
pub const COVERED_SHARE: f64 = 0.99;
/// Above this share of the scans' area in the result lying on another scan's surface,
/// `Stacked` is raised.
pub const STACKED_SHARE: f64 = 0.005;
/// Rounds of recovery and refill at most.
pub const RECOVER_ROUNDS: usize = 3;
/// The guard's grid cell, in largest ball diameters (two radii).
const CELL_RADII: f64 = 2.0;
/// Triangles of all scans together at most. The merge holds, per triangle of the input, about 300
/// bytes at its peak: the scan as prepared (indices, neighbours, a copy for its BVH of 72 bytes, vertex
/// normals and distances), the union, the result and its BVH (72 bytes again) and the list of removed
/// triangles (8 bytes); the coverage samples are a fixed budget, not per triangle. 20 million
/// triangles therefore need about 6 GB on a native target; the browser build (32-bit, 4 GB of
/// address space shared with everything else) takes 2 million (about 600 MB).
#[cfg(not(target_arch = "wasm32"))]
pub const MAX_TRIANGLES: usize = 20_000_000;
/// Triangles of all scans together at most in the browser build (see the native value).
#[cfg(target_arch = "wasm32")]
pub const MAX_TRIANGLES: usize = 2_000_000;

/// Settings of [`merge`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    /// Surface of two scans within this distance (mm) is the same surface.
    pub tolerance: f64,
}

/// Why nothing was done.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum MergeError {
    /// A setting or the list of scans is out of range.
    #[error("{0}")]
    Params(String),
    /// A scan is not a valid mesh.
    #[error("scan {0} is invalid: {1}")]
    Invalid(usize, String),
    /// The caller cancelled.
    #[error("cancelled")]
    Cancelled,
}

/// Something the caller should know, with its numbers. Scans are numbered from 0 in the order given.
#[derive(Clone, Debug, PartialEq)]
pub enum Issue {
    /// The scans disagree in their overlap by more than the tolerance (refused).
    /// `over`: samples lying over the other scan displaced beyond the reach. `beyond_share`: of the
    /// near samples, the share beyond the tolerance; when the scans are displaced along their normals
    /// (no near samples), the share of all samples lying over the other.
    Misaligned { a: usize, b: usize, near: usize, over: usize, beyond_share: f64, rms: f64, p95: f64 },
    /// The normals of two scans oppose where they meet: one is turned inside out (refused).
    Opposed { a: usize, b: usize, opposed: usize, facing: usize },
    /// Triangles of a scan wound against the rest of their piece cover more than
    /// [`FLIPPED_REFUSE_SHARE`] of its area (refused).
    Winding { scan: usize, flipped: usize, share: f64 },
    /// A few triangles of a scan were wound against the rest of their piece and were left out.
    FlippedDropped { scan: usize, flipped: usize, share: f64 },
    /// The result has edges whose two triangles run the same way.
    Unoriented { edges: usize },
    /// Vertices of different scans left unwelded at (nearly) one point.
    Unwelded { vertices: usize },
    /// More than [`STACKED_SHARE`] of the scans' area in the result lies on another scan's surface
    /// (`triangles` of them).
    Stacked { share: f64, triangles: usize },
    /// Closed pieces of a scan with a negative volume that are not refused: inner skins (inside another
    /// closed piece: the inside of a hollow part) and debris below [`prep::INVERTED_MIN_SHARE`].
    InnerSkins { scan: usize, skins: usize, debris: usize },
    /// A closed piece of a scan has a negative volume: turned inside out (refused).
    Inverted { scan: usize, pieces: usize },
    /// Exact duplicate triangles and triangles on edges of three or more triangles were left out of a
    /// scan (and count as removed).
    NonManifold { scan: usize, dropped: usize },
    /// Groups of scans that share no surface: the result is a union of pieces.
    Disjoint { groups: Vec<Vec<usize>> },
    /// A scan lies wholly on others: nothing of it is in the result.
    FullyCovered { scan: usize },
    /// Part of the seam stayed open (no triangle there lay on scanned surface or could avoid crossing).
    SeamOpen { edges: usize, length: f64 },
    /// Less than [`COVERED_SHARE`] of the removed area lies within the tolerance of the result.
    RemovedUncovered { share: f64, worst: f64 },
    /// The result has more pinched vertices than the scans together.
    Pinched { before: usize, after: usize },
}

impl Issue {
    /// Does this issue stop the merge?
    #[must_use]
    pub fn refuses(&self) -> bool {
        matches!(
            self,
            Issue::Misaligned { .. } | Issue::Opposed { .. } | Issue::Winding { .. } | Issue::Inverted { .. }
        )
    }
}

/// One scan in the merge.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ScanRow {
    /// Triangles given.
    pub triangles: usize,
    /// Left out: exact duplicates, on an edge of three or more triangles, or wound against the rest
    /// of their piece.
    pub dropped: usize,
    /// Kept as they were.
    pub kept: usize,
    /// Not in the result as themselves (removed by the ownership or eaten back, left out as dropped),
    /// restored, recovered and collapsed ones not counted: `triangles = kept + restored + collapsed +
    /// removed`.
    pub removed: usize,
    /// Kept triangles a weld collapsed (two of their corners welded into one; their area is in
    /// `removed_area`).
    pub collapsed: usize,
    /// Removed and put back: the result had their corners at exactly their places (pieces of one
    /// scan), or the seam left them uncovered and they came back on the scan's own vertices.
    pub restored: usize,
    /// Area (mm²).
    pub area: f64,
    /// Area of the triangles not in the result as themselves: removed, dropped and collapsed (mm²).
    pub removed_area: f64,
    /// Share of the scan's area within the tolerance of facing surface of another scan.
    pub overlap_share: f64,
    /// Median edge (mm).
    pub median_edge: f64,
    /// Topology of the scan.
    pub topology: Topology,
}

/// The checks of point 3 of the design, measured.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Checks {
    /// The removed triangles against the result.
    pub removed: Coverage,
    /// Vertices of the scans in the result, those a weld moved and the largest move (mm).
    pub kept_vertices: usize,
    pub moved_vertices: usize,
    pub max_vertex_move: f64,
    /// Welds undone because they would have flipped, folded or doubled a triangle.
    pub welds_undone: usize,
    /// Added triangles (restored or seam) crossing another triangle of the result.
    pub added_crossings: usize,
    /// Vertices of the result with a vertex of another scan closer than [`DUPLICATE_SHARE`] median
    /// edges and no edge between them (copies of one point left unwelded).
    pub unwelded_vertices: usize,
    /// Share of the area of the scans' triangles in the result that lies within the tolerance of facing
    /// surface of another scan's triangles: two layers left (should be near 0).
    pub stacked_share: f64,
    /// Scan triangles of the result counted in [`Checks::stacked_share`].
    pub stacked_triangles: usize,
}

/// The stages of [`merge`], announced to the caller as they start (for timing).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Stage {
    Prepare,
    Overlap,
    Ownership,
    Union,
    Seam,
    Check,
}

/// What [`merge`] did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    pub tolerance: f64,
    /// How far the overlap was searched (mm).
    pub reach: f64,
    pub scans: Vec<ScanRow>,
    /// Every ordered pair of scans with surface near each other.
    pub pairs: Vec<PairResidual>,
    /// Redundant triangles eaten back after the ownership.
    pub eaten: usize,
    pub seam: SeamReport,
    /// Topology of the scans together (sums) and of the result.
    pub before: Topology,
    pub after: Topology,
    pub checks: Checks,
    pub issues: Vec<Issue>,
    /// The merge refused: no mesh (see the issues that refuse).
    pub refused: bool,
}

fn check_input(scans: &[Mesh], p: &Params) -> Result<(), MergeError> {
    if scans.len() < 2 {
        return Err(MergeError::Params(format!("merging needs at least two scans, not {}", scans.len())));
    }
    if !(p.tolerance.is_finite() && p.tolerance > 0.0) {
        return Err(MergeError::Params(format!("tolerance must be a positive number of mm, not {}", p.tolerance)));
    }
    let total: usize = scans.iter().map(Mesh::triangle_count).sum();
    if total > MAX_TRIANGLES {
        return Err(MergeError::Params(format!("{total} triangles in all exceed the limit of {MAX_TRIANGLES}")));
    }
    for (k, m) in scans.iter().enumerate() {
        m.validate().map_err(|e| MergeError::Invalid(k, e.to_string()))?;
        if m.triangles.is_empty() {
            return Err(MergeError::Params(format!("scan {k} has no triangles")));
        }
    }
    Ok(())
}

fn add(a: Topology, b: Topology) -> Topology {
    Topology {
        pieces: a.pieces + b.pieces,
        loops: a.loops + b.loops,
        genus: a.genus.zip(b.genus).map(|(x, y)| x + y),
        non_manifold_edges: a.non_manifold_edges + b.non_manifold_edges,
        unoriented_edges: a.unoriented_edges + b.unoriented_edges,
        pinched_vertices: a.pinched_vertices + b.pinched_vertices,
    }
}

/// Issues of the scans on their own (orientation, non-manifold edges).
fn scan_issues(scans: &[Scan]) -> Vec<Issue> {
    let mut out = Vec::new();
    for (k, s) in scans.iter().enumerate() {
        if s.non_manifold > 0 {
            out.push(Issue::NonManifold { scan: k, dropped: s.non_manifold });
        }
        let (flipped, share) = s.flipped;
        if share > FLIPPED_REFUSE_SHARE {
            out.push(Issue::Winding { scan: k, flipped, share });
        } else if flipped > 0 {
            out.push(Issue::FlippedDropped { scan: k, flipped, share });
        }
        let (inverted, skins, debris) = s.negative;
        if inverted > 0 {
            out.push(Issue::Inverted { scan: k, pieces: inverted });
        }
        if skins + debris > 0 {
            out.push(Issue::InnerSkins { scan: k, skins, debris });
        }
    }
    out
}

/// The pair residual of `a` on `b`, if any.
fn pair(pairs: &[PairResidual], a: usize, b: usize) -> Option<&PairResidual> {
    pairs.iter().find(|r| r.a == a && r.b == b)
}

/// Issues of the pairs: misaligned, opposed, and the groups of scans that share nothing.
fn pair_issues(n: usize, pairs: &[PairResidual]) -> Vec<Issue> {
    let mut out = Vec::new();
    let mut group: Vec<usize> = (0..n).collect();
    for a in 0..n {
        for b in a + 1..n {
            let (x, y) = (pair(pairs, a, b), pair(pairs, b, a));
            out.extend(misaligned(a, b, x, y));
            let flipped =
                [x, y].into_iter().flatten().find(|r| r.opposed >= MIN_SHARED_SAMPLES && r.opposed > r.within);
            if let Some(r) = flipped {
                out.push(Issue::Opposed { a, b, opposed: r.opposed, facing: r.within });
            }
            // scans are together in space when one lies on the other within the reach, or over it
            // displaced (a stray ray hit of pieces that share nothing does not join them)
            if [x, y].into_iter().flatten().any(|r| r.near > 0 || r.displaced(MIN_SHARED_SAMPLES)) {
                let (ga, gb) = (group[a], group[b]);
                group.iter_mut().filter(|g| **g == gb).for_each(|g| *g = ga);
            }
        }
    }
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for k in 0..n {
        match groups.iter_mut().find(|g| g.first().is_some_and(|&f| group[f] == group[k])) {
            Some(g) => g.push(k),
            None => groups.push(vec![k]),
        }
    }
    if groups.len() > 1 {
        out.push(Issue::Disjoint { groups });
    }
    out
}

/// Are scans `a` and `b` misaligned (their residuals `x` of a on b and `y` of b on a): more than
/// [`MISALIGNED_SHARE`] of the shared surface beyond the tolerance, or lying over each other displaced
/// beyond the reach?
fn misaligned(a: usize, b: usize, x: Option<&PairResidual>, y: Option<&PairResidual>) -> Option<Issue> {
    let both = || [x, y].into_iter().flatten();
    let worse =
        both().filter(|r| r.near >= MIN_SHARED_SAMPLES).max_by(|p, q| p.beyond_share().total_cmp(&q.beyond_share()));
    let issue = |r: &PairResidual, share: f64| Issue::Misaligned {
        a,
        b,
        near: r.near,
        over: r.over,
        beyond_share: share,
        rms: r.rms,
        p95: r.p95,
    };
    if let Some(r) = worse.filter(|r| r.beyond_share() > MISALIGNED_SHARE) {
        return Some(issue(r, r.beyond_share()));
    }
    // displaced along the normal: every sample over the other lies beyond the tolerance; the share
    // reported is that of the samples over the other (rms and p95 are of the few near ones, if any)
    both().find(|r| r.displaced(MIN_SHARED_SAMPLES)).map(|r| issue(r, r.over as f64 / r.samples.max(1) as f64))
}

/// Median of the scans' median edges: the length scale of the tie band, the weld radius cap, the
/// guard's grid and the unwelded-vertex check. (The default tolerance of the command uses the coarsest
/// scan's median edge instead: a tolerance must hold for every pair.)
fn scale(scans: &[Scan]) -> f64 {
    let mut e: Vec<f64> = scans.iter().map(|s| s.median_edge).filter(|e| *e > 0.0).collect();
    e.sort_by(f64::total_cmp);
    e.get(e.len() / 2).copied().unwrap_or(1.0)
}

/// Merges the aligned `scans` into one mesh (see the module). Returns the mesh (`None` when the
/// merge refused: see [`Report::refused`] and the issues) and the report. `stage` is told when each
/// stage starts.
pub fn merge(
    scans: &[Mesh],
    p: &Params,
    cancel: &Cancel,
    stage: &mut dyn FnMut(Stage),
) -> Result<(Option<Mesh>, Report), MergeError> {
    check_input(scans, p)?;
    let tol = p.tolerance;
    stage(Stage::Prepare);
    let prepared = scans
        .iter()
        .map(|m| prep::prepare(m, cancel).filter(|_| !cancel.is_cancelled()))
        .collect::<Option<Vec<Scan>>>();
    let prepared = prepared.ok_or(MergeError::Cancelled)?;
    let mut rep = Report { tolerance: tol, reach: REACH_TOLERANCES * tol, ..Report::default() };
    rep.before = prepared.iter().fold(Topology { genus: Some(0), ..Topology::default() }, |a, s| add(a, s.topology));
    rep.issues = scan_issues(&prepared);
    stage(Stage::Overlap);
    let (pairs, covered) = overlap::residuals(&prepared, rep.reach, tol, cancel).ok_or(MergeError::Cancelled)?;
    rep.issues.extend(pair_issues(prepared.len(), &pairs));
    rep.pairs = pairs;
    rep.scans = prepared.iter().zip(&covered).map(|(s, c)| first_row(s, *c)).collect();
    if rep.issues.iter().any(Issue::refuses) {
        rep.refused = true;
        return Ok((None, rep));
    }
    let mesh = build(&prepared, tol, &mut rep, cancel, stage)?;
    Ok((Some(mesh), rep))
}

/// The row of a scan before the merge.
fn first_row(s: &Scan, overlap_share: f64) -> ScanRow {
    ScanRow {
        triangles: s.mesh.triangles.len() + s.dropped.len(),
        dropped: s.dropped.len(),
        area: s.mesh.area() + s.dropped.iter().map(|[a, b, c]| crate::tri::area(a, b, c)).sum::<f64>(),
        overlap_share,
        median_edge: s.median_edge,
        topology: s.topology,
        ..ScanRow::default()
    }
}

/// Ownership, union, seam and checks.
fn build(
    scans: &[Scan],
    tol: f64,
    rep: &mut Report,
    cancel: &Cancel,
    stage: &mut dyn FnMut(Stage),
) -> Result<Mesh, MergeError> {
    stage(Stage::Ownership);
    let (mut removed, lost) = own::removed_by_owner(scans, tol, scale(scans), cancel).ok_or(MergeError::Cancelled)?;
    rep.eaten = own::eat_back(scans, &mut removed, tol, cancel).ok_or(MergeError::Cancelled)?;
    stage(Stage::Union);
    let polled = || if cancel.is_cancelled() { Err(MergeError::Cancelled) } else { Ok(()) };
    let mut u = union::assemble(scans, &removed);
    polled()?;
    u.weld_front(scans, weld::weld_radius(tol, scale(scans)), cancel).ok_or(MergeError::Cancelled)?;
    let cell = RADII_EDGES.iter().fold(0.0_f64, |m, k| m.max(*k)) * CELL_RADII * scale(scans);
    let mut guard = guard::Guard::new(scans, &u, tol, cell);
    polled()?;
    rep.seam.restored = union::restore(&mut u, scans, (&removed, &lost), &mut |p, t| guard.restorable(p, t), cancel)
        .ok_or(MergeError::Cancelled)?;
    stage(Stage::Seam);
    seam::close(&mut u, scans, &mut guard, &mut rep.seam, cancel).ok_or(MergeError::Cancelled)?;
    recover_rounds(&mut u, scans, &removed, &mut guard, tol, &mut rep.seam, cancel).ok_or(MergeError::Cancelled)?;
    rep.seam.unsupported = guard.unsupported;
    rep.seam.crossing = guard.crossing;
    rep.seam.bad_corners = guard.bad_corners;
    polled()?;
    let (dropped, count, recovered) = fill::drop_islands(&mut u, tol, cancel).ok_or(MergeError::Cancelled)?;
    rep.seam.islands_dropped = count;
    rep.seam.recovered.triangles = rep.seam.recovered.triangles.saturating_sub(recovered);
    for (s, t) in dropped {
        if let Some(r) = removed.get_mut(s as usize).and_then(|r| r.get_mut(t as usize)) {
            *r = true;
        }
    }
    polled()?;
    seam::measure_open(&u, scans, &mut rep.seam);
    polled()?;
    stage(Stage::Check);
    finish(scans, &removed, u, tol, rep, cancel)
}

/// Recovery, the specks it leaves taken back, and the holes filled again, until a round puts nothing
/// back (the refill may take seam triangles back and uncover surface again), at most
/// [`RECOVER_ROUNDS`] rounds. `None` when cancelled.
fn recover_rounds(
    u: &mut Union,
    scans: &[Scan],
    removed: &[Vec<bool>],
    guard: &mut guard::Guard<'_>,
    tol: f64,
    rep: &mut SeamReport,
    cancel: &Cancel,
) -> Option<()> {
    for _ in 0..RECOVER_ROUNDS {
        let r = recover::recover(u, scans, removed, guard, (tol, cancel))?;
        if r.triangles == 0 {
            break;
        }
        let specks = fill::drop_specks(u, guard);
        if cancel.is_cancelled() {
            return None;
        }
        rep.recovered.rounds += 1;
        rep.recovered.uncovered += r.uncovered;
        rep.recovered.triangles += r.triangles;
        rep.recovered.specks_dropped += specks;
        seam::refill(u, scans, guard, rep, cancel)?;
        if specks >= r.triangles {
            break;
        }
    }
    Some(())
}

/// The triangles of the scans that are not in the result as themselves: removed and not put back,
/// and kept ones a weld collapsed, as `(scan, triangle)`.
fn gone_ids(scans: &[Scan], removed: &[Vec<bool>], u: &Union) -> Vec<(u32, u32)> {
    let back: std::collections::HashSet<(u32, u32)> = u
        .origin
        .iter()
        .filter_map(|o| if let Origin::Restored(s, t) | Origin::Recovered(s, t) = o { Some((*s, *t)) } else { None })
        .collect();
    let mut out: Vec<(u32, u32)> = u.collapsed.clone();
    for (s, scan) in scans.iter().enumerate() {
        let gone = removed.get(s).map_or(&[][..], Vec::as_slice);
        out.extend(
            (0..scan.mesh.triangles.len())
                .filter(|&t| gone.get(t).copied().unwrap_or(false))
                .map(|t| (s as u32, t as u32))
                .filter(|k| !back.contains(k)),
        );
    }
    out
}

/// Per scan: kept, put back, collapsed by a weld, removed (the rest), and the area not in the result.
fn fill_rows(scans: &[Scan], gone: &[(u32, u32)], u: &Union, rows: &mut [ScanRow]) {
    for o in &u.origin {
        match o {
            Origin::Kept(s, _) => rows.get_mut(*s as usize).into_iter().for_each(|r| r.kept += 1),
            Origin::Restored(s, _) | Origin::Recovered(s, _) => {
                rows.get_mut(*s as usize).into_iter().for_each(|r| r.restored += 1)
            }
            Origin::Seam => {}
        }
    }
    for (s, _) in &u.collapsed {
        rows.get_mut(*s as usize).into_iter().for_each(|r| r.collapsed += 1);
    }
    for &(s, t) in gone {
        if let (Some(row), Some(scan)) = (rows.get_mut(s as usize), scans.get(s as usize)) {
            row.removed_area += scan.mesh.face_area(t as usize);
        }
    }
    for (row, scan) in rows.iter_mut().zip(scans) {
        row.removed = row.triangles.saturating_sub(row.kept + row.restored + row.collapsed);
        row.removed_area += scan.dropped.iter().map(|[a, b, c]| crate::tri::area(a, b, c)).sum::<f64>();
    }
}

/// The checks, the result's topology and the issues they raise; the result mesh.
fn finish(
    scans: &[Scan],
    removed: &[Vec<bool>],
    u: Union,
    tol: f64,
    rep: &mut Report,
    cancel: &Cancel,
) -> Result<Mesh, MergeError> {
    let gone = gone_ids(scans, removed, &u);
    fill_rows(scans, &gone, &u, &mut rep.scans);
    rep.checks.moved_vertices = u.welded;
    rep.checks.max_vertex_move = u.max_move;
    rep.checks.welds_undone = u.welds_undone;
    let Union { positions, triangles, origin, scan_of, .. } = u;
    let mut mesh = Mesh { positions, triangles };
    let bvh = Bvh::build(&mesh);
    let built = check::Built { mesh: &mesh, bvh: &bvh, origin: &origin, scan_of: &scan_of };
    rep.checks.added_crossings = check::added_crossings(&built, cancel).ok_or(MergeError::Cancelled)?;
    rep.checks.unwelded_vertices =
        check::unwelded(&built, DUPLICATE_SHARE * scale(scans), cancel).ok_or(MergeError::Cancelled)?;
    let (stacked_share, stacked_triangles) = check::stacked_share(&built, tol, cancel).ok_or(MergeError::Cancelled)?;
    rep.checks.stacked_share = stacked_share;
    rep.checks.stacked_triangles = stacked_triangles;
    let dropped: Vec<&[Point3; 3]> = scans.iter().flat_map(|s| s.dropped.iter()).collect();
    let tri_of = |i: usize| -> Option<[Point3; 3]> {
        match dropped.get(i) {
            Some(t) => Some(**t),
            None => {
                let &(s, t) = gone.get(i - dropped.len())?;
                scans.get(s as usize)?.mesh.triangle(t as usize)
            }
        }
    };
    rep.checks.removed =
        check::coverage(&built, (dropped.len() + gone.len(), &tri_of), tol, cancel).ok_or(MergeError::Cancelled)?;
    drop(bvh);
    crate::edit::compact(&mut mesh);
    rep.checks.kept_vertices = mesh.positions.len();
    rep.after = topology(&mesh);
    result_issues(rep);
    Ok(mesh)
}

/// Issues of the result.
fn result_issues(rep: &mut Report) {
    for (k, r) in rep.scans.iter().enumerate() {
        if r.kept + r.restored == 0 {
            rep.issues.push(Issue::FullyCovered { scan: k });
        }
    }
    if rep.seam.open_edges > 0 {
        rep.issues.push(Issue::SeamOpen { edges: rep.seam.open_edges, length: rep.seam.open_length });
    }
    let cov = rep.checks.removed;
    if cov.share() < COVERED_SHARE {
        rep.issues.push(Issue::RemovedUncovered { share: cov.share(), worst: cov.worst });
    }
    if rep.checks.stacked_share > STACKED_SHARE {
        rep.issues.push(Issue::Stacked { share: rep.checks.stacked_share, triangles: rep.checks.stacked_triangles });
    }
    if rep.checks.unwelded_vertices > 0 {
        rep.issues.push(Issue::Unwelded { vertices: rep.checks.unwelded_vertices });
    }
    if rep.after.unoriented_edges > 0 {
        rep.issues.push(Issue::Unoriented { edges: rep.after.unoriented_edges });
    }
    if rep.after.pinched_vertices > rep.before.pinched_vertices {
        rep.issues.push(Issue::Pinched { before: rep.before.pinched_vertices, after: rep.after.pinched_vertices });
    }
}
