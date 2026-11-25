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
//!    averaged and not stacked: the merge refuses with `Misaligned` and the numbers. Scans that
//!    lie over each other beyond the reach and that nothing ties (no agreement of their own or
//!    through others) are refused as `Displaced`; scans that meet somewhere and also lie over each
//!    other are reported as `LayersApart` (a shelf over a floor looks the same). Samples whose search
//!    along the normal ran out of its budget are not judged (`Unjudged`). Scans whose normals oppose
//!    where they meet (one is turned inside out) are refused.
//! 3. Ownership ([`own`]), for scans that pass step 2: where several scans cover the surface, the
//!    one in which it lies farthest from its own border keeps it; the others lose those vertices and every triangle touching
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
//! O(T log T) in the triangles of all scans; nothing is quadratic in them (the pair statistics are
//! quadratic in the number of scans, at most [`MAX_SCANS`]).
mod check;
mod fill;
mod guard;
mod nest;
mod orient;
mod overlap;
mod own;
mod prep;
mod recover;
mod seam;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod tests_nest;
#[cfg(test)]
mod tests_pairs;
#[cfg(test)]
mod tests_round2;
#[cfg(test)]
mod tests_units;
mod union;
mod weld;

pub use check::{Coverage, DUPLICATE_SHARE, MAX_REMOVED_SAMPLES};
pub use fill::SMALL_LOOP_EDGES;
pub use guard::{MAX_CELL_SPAN, SUPPORT_SLOPE};
pub use nest::INVERTED_MIN_SHARE;
pub use orient::FLIPPED_REFUSE_SHARE;
pub use overlap::{PAIR_SAMPLES, PairResidual};
pub use prep::{Topology, topology};
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
/// Scans merged at once at most (the pair statistics are quadratic in them).
pub const MAX_SCANS: usize = 64;
/// Coordinates of the scans at most this far from the origin along an axis (mm).
pub const MAX_COORDINATE: f64 = 1e9;
/// Rounds of recovery and refill at most.
pub const RECOVER_ROUNDS: usize = 3;
/// The guard's grid cell, in largest ball diameters (two radii).
const CELL_RADII: f64 = 2.0;
/// Triangles of all scans together at most. The merge holds, per triangle of the input, about 450
/// bytes at its peak: the scan as prepared (indices, neighbours, a copy for its BVH of 72 bytes, vertex
/// normals and distances, about 150), the union and the guard's copy of it with its BVH (about 110),
/// the hash sets of the restoration (vertex sets and directed edges, about 100 with their overhead),
/// the result and its BVH (about 50), the set of triangles put back, the list of removed triangles
/// and the stacked-layer flags (16 a triangle, in blocks); the coverage samples are a fixed budget,
/// not per triangle. 20 million triangles therefore need about 9 GB on a native target; the browser
/// build (32-bit, 4 GB of address space shared with everything else) takes 2 million (about 900 MB).
#[cfg(not(target_arch = "wasm32"))]
pub const MAX_TRIANGLES: usize = 20_000_000;
/// Triangles of all scans together at most in the browser build (see the native value).
#[cfg(target_arch = "wasm32")]
pub const MAX_TRIANGLES: usize = 2_000_000;

/// Settings of [`merge`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Params {
    /// Surface of two scans within this distance (mm) is the same surface.
    pub tolerance: f64,
    /// What a misaligned or displaced pair does: refuse the merge (the default) or only warn.
    pub alignment: Alignment,
}

/// What the alignment findings of a pair (`Misaligned`, `Displaced`) do to the merge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Alignment {
    /// They refuse it (nothing is made).
    #[default]
    Refuse,
    /// They are reported and the merge proceeds (the caller knows the scans are aligned, or accepts
    /// what the overlap gives). Refusals that protect the merge itself (a scan turned inside out,
    /// opposed normals, a patch wound the wrong way) still refuse.
    Warn,
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
    /// near samples, the share beyond the tolerance.
    Misaligned { a: usize, b: usize, near: usize, over: usize, beyond_share: f64, rms: f64, p95: f64 },
    /// Scan `a` lies over scan `b` displaced along its normals: `over` of `samples` samples of `a` lie
    /// over `b` beyond the reach (median `offset` mm along the normal), `within` agree with it, and
    /// no scan that shares surface with both ties them (refused).
    Displaced { a: usize, b: usize, samples: usize, over: usize, within: usize, offset: f64 },
    /// Two scans that meet somewhere also lie over each other: `over` of the `related` samples of `a`
    /// (on `b` within the reach, or over it) lie over `b` beyond the reach, at a median `offset` mm
    /// along the normals with `spread` mm between the quartiles (`one_offset`: a shift as a whole;
    /// else a tilt or a curved surface). A scan displaced along a face parallel to the displacement
    /// looks so, and so does a shelf over a floor seen by different scans: not refused, reported.
    LayersApart { a: usize, b: usize, over: usize, related: usize, offset: f64, spread: f64, one_offset: bool },
    /// Scans `a` and `b` touch (`near` samples of each on the other, `over` over it) by too little for
    /// any verdict on their alignment: fewer than [`MIN_SHARED_SAMPLES`] of each (reported, not
    /// judged).
    SmallOverlap { a: usize, b: usize, near: [usize; 2], over: [usize; 2] },
    /// Samples of `a` whose cast toward `b` ran out of its budget (a pile of triangles): not judged.
    Unjudged { a: usize, b: usize, samples: usize },
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
    /// (`triangles` of them). Two layers of one scan are its own data (a double skin, a thin wall) and
    /// raise no notice; which scan a triangle belongs to is told by its vertices, so a seam triangle
    /// with three corners of one scan counts as that scan.
    Stacked { share: f64, triangles: usize },
    /// Closed pieces of a scan with a negative volume that are not refused: inner skins (inside another
    /// closed piece: the inside of a hollow part) and debris below [`INVERTED_MIN_SHARE`].
    /// `unjudged`: of the inner skins, pieces whose inside test could not be completed within its
    /// budget (taken as inner skins, not refused).
    InnerSkins { scan: usize, skins: usize, debris: usize, unjudged: usize },
    /// A closed piece of a scan has a negative volume: turned inside out (refused).
    Inverted { scan: usize, pieces: usize },
    /// Closed pieces of a scan with a negative volume whose inside test ran out of its budget (a pile
    /// of pieces, many nested shells) cover at least [`INVERTED_MIN_SHARE`] of its area together:
    /// they may be turned inside out, so the scan is refused (not lifted by [`Alignment::Warn`]).
    InvertedUnjudged { scan: usize, pieces: usize },
    /// Exact duplicate triangles and triangles on edges of three or more triangles were left out of a
    /// scan (and count as removed).
    NonManifold { scan: usize, dropped: usize },
    /// Groups of scans that share no surface: the result is a union of pieces.
    Disjoint { groups: Vec<Vec<usize>> },
    /// A scan lies wholly on others: nothing of it is in the result.
    FullyCovered { scan: usize },
    /// Triangles of the seam or the restoration refused because the crossing test would have compared
    /// more triangles than allowed: not judged, left out.
    OverBudget { refusals: usize },
    /// Part of the seam stayed open (no triangle there lay on scanned surface or could avoid crossing).
    SeamOpen { edges: usize, length: f64 },
    /// Less than [`COVERED_SHARE`] of the removed area lies within the tolerance of the result.
    RemovedUncovered { share: f64, worst: f64 },
    /// The result has more pinched vertices than the scans together.
    Pinched { before: usize, after: usize },
}

impl Issue {
    /// Is this a finding about the alignment of a pair (refused by default, a warning with
    /// [`Alignment::Warn`])?
    #[must_use]
    pub fn is_alignment(&self) -> bool {
        matches!(self, Issue::Misaligned { .. } | Issue::Displaced { .. })
    }

    /// Does this issue stop the merge under `alignment`?
    #[must_use]
    pub fn refuses_under(&self, alignment: Alignment) -> bool {
        self.refuses() && !(alignment == Alignment::Warn && self.is_alignment())
    }

    /// Does this issue stop the merge (by default)?
    #[must_use]
    pub fn refuses(&self) -> bool {
        matches!(
            self,
            Issue::Misaligned { .. }
                | Issue::Displaced { .. }
                | Issue::Opposed { .. }
                | Issue::Winding { .. }
                | Issue::Inverted { .. }
                | Issue::InvertedUnjudged { .. }
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
    // the pair statistics are quadratic in the scans and the grouping cubic: bounded here
    if scans.len() > MAX_SCANS {
        return Err(MergeError::Params(format!(
            "at most {MAX_SCANS} scans can be merged at once, not {}",
            scans.len()
        )));
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
        // beyond this distance from the origin the volumes and areas of the checks lose their
        // precision (a part far out collapses to points, an inverted piece reads as flat) or
        // overflow: refused, not merged blind
        if m.positions.iter().any(|q| q.coords.amax() > MAX_COORDINATE) {
            return Err(MergeError::Params(format!(
                "scan {k} has points farther than {MAX_COORDINATE:e} mm from the origin along an axis: move it nearer the origin"
            )));
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
        let (inverted, skins, debris) = s.negative.counts();
        if inverted > 0 {
            out.push(Issue::Inverted { scan: k, pieces: inverted });
        }
        if s.negative.unjudged_refuses {
            out.push(Issue::InvertedUnjudged { scan: k, pieces: s.negative.unjudged });
        }
        if skins + debris > 0 {
            let unjudged = if s.negative.unjudged_refuses { 0 } else { s.negative.unjudged };
            out.push(Issue::InnerSkins { scan: k, skins, debris, unjudged });
        }
    }
    out
}

/// The pair residual of `a` on `b`, if any.
fn pair(pairs: &[PairResidual], a: usize, b: usize) -> Option<&PairResidual> {
    pairs.iter().find(|r| r.a == a && r.b == b)
}

/// Issues of the pairs: misaligned, opposed, and the groups of scans that share nothing.
fn pair_issues(n: usize, pairs: &[PairResidual], tol: f64) -> Vec<Issue> {
    let both = |a: usize, b: usize| [pair(pairs, a, b), pair(pairs, b, a)].into_iter().flatten();
    // scans tied by shared surface, directly or through others: one lying over another displaced is
    // then the part's own shape (a step, a shelf), not a misalignment, since both agree with a third
    let tied = groups_of(n, |a, b| both(a, b).any(|r| r.agrees(MIN_SHARED_SAMPLES)));
    let mut out = Vec::new();
    for a in 0..n {
        for b in a + 1..n {
            let (x, y) = (pair(pairs, a, b), pair(pairs, b, a));
            let refused = misaligned(a, b, x, y, tied.get(a) == tied.get(b));
            // a pair refused as displaced says it already
            if !matches!(refused, Some(Issue::Displaced { .. })) {
                out.extend(layers_apart(x, y, tol));
            }
            out.extend(refused);
            out.extend(too_small(a, b, x, y));
            for r in [x, y].into_iter().flatten().filter(|r| r.unknown > 0) {
                out.push(Issue::Unjudged { a: r.a, b: r.b, samples: r.unknown });
            }
            let flipped =
                [x, y].into_iter().flatten().find(|r| r.opposed >= MIN_SHARED_SAMPLES && r.opposed > r.within);
            if let Some(r) = flipped {
                out.push(Issue::Opposed { a, b, opposed: r.opposed, facing: r.within });
            }
        }
    }
    // scans are together in space when one lies on the other within the reach, or over it displaced
    // (a stray ray hit of pieces that share nothing does not join them)
    let group = groups_of(n, |a, b| both(a, b).any(|r| r.near > 0 || r.displaced(MIN_SHARED_SAMPLES)));
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for k in 0..n {
        match groups.iter_mut().find(|g| g.first().is_some_and(|&f| group.get(f) == group.get(k))) {
            Some(g) => g.push(k),
            None => groups.push(vec![k]),
        }
    }
    if groups.len() > 1 {
        out.push(Issue::Disjoint { groups });
    }
    out
}

/// The group of every one of `n` scans, two scans in one group when `linked` ties them, directly or
/// through others.
fn groups_of(n: usize, linked: impl Fn(usize, usize) -> bool) -> Vec<usize> {
    let mut group: Vec<usize> = (0..n).collect();
    for a in 0..n {
        for b in a + 1..n {
            if linked(a, b) {
                let (ga, gb) = (group[a], group[b]);
                group.iter_mut().filter(|g| **g == gb).for_each(|g| *g = ga);
            }
        }
    }
    group
}

/// Are scans `a` and `b` misaligned (their residuals `x` of a on b and `y` of b on a): more than
/// [`MISALIGNED_SHARE`] of the shared surface beyond the tolerance, or lying over each other displaced
/// beyond the reach while nothing ties them (`tied`: both share surface with others that agree)?
fn misaligned(a: usize, b: usize, x: Option<&PairResidual>, y: Option<&PairResidual>, tied: bool) -> Option<Issue> {
    let both = || [x, y].into_iter().flatten();
    let worse =
        both().filter(|r| r.near >= MIN_SHARED_SAMPLES).max_by(|p, q| p.beyond_share().total_cmp(&q.beyond_share()));
    if let Some(r) = worse.filter(|r| r.beyond_share() > MISALIGNED_SHARE) {
        return Some(Issue::Misaligned {
            a,
            b,
            near: r.near,
            over: r.over,
            beyond_share: r.beyond_share(),
            rms: r.rms,
            p95: r.p95,
        });
    }
    if tied {
        return None;
    }
    both().find(|r| r.displaced(MIN_SHARED_SAMPLES)).map(|r| Issue::Displaced {
        a: r.a,
        b: r.b,
        samples: r.samples,
        over: r.over,
        within: r.within,
        offset: r.over_offset,
    })
}

/// Do scans `a` and `b` touch (some near samples) by too little for any verdict: fewer than
/// [`MIN_SHARED_SAMPLES`] near and over in both directions?
fn too_small(a: usize, b: usize, x: Option<&PairResidual>, y: Option<&PairResidual>) -> Option<Issue> {
    let count = |r: Option<&PairResidual>, f: fn(&PairResidual) -> usize| r.map_or(0, f);
    let near = [count(x, |r| r.near), count(y, |r| r.near)];
    let over = [count(x, |r| r.over), count(y, |r| r.over)];
    let touch = near.iter().any(|&n| n > 0);
    let small = near.iter().chain(&over).all(|&n| n < MIN_SHARED_SAMPLES);
    (touch && small).then_some(Issue::SmallOverlap { a, b, near, over })
}

/// Do two scans that meet somewhere also lie over each other (see [`Issue::LayersApart`]): in either
/// direction a material share of the related samples over the other? (A tight cluster at one offset
/// was tried for the end caps of a slender part slid along its axis and left out: a correct scan of
/// the local corpus has one as tight at a share within 1.4 times of the rod's.)
fn layers_apart(x: Option<&PairResidual>, y: Option<&PairResidual>, tol: f64) -> Option<Issue> {
    let both = || [x, y].into_iter().flatten();
    // only scans that meet somewhere: pieces that share nothing and lie over each other are tied by a
    // third scan (or refused as displaced)
    if !both().any(|r| r.near >= MIN_SHARED_SAMPLES) {
        return None;
    }
    let r =
        both().filter(|r| r.lies_over(MIN_SHARED_SAMPLES)).max_by(|p, q| p.over_share().total_cmp(&q.over_share()))?;
    Some(Issue::LayersApart {
        a: r.a,
        b: r.b,
        over: r.over,
        related: r.related(),
        offset: r.over_offset,
        spread: r.over_spread,
        one_offset: r.one_offset(tol),
    })
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
    rep.issues.extend(pair_issues(prepared.len(), &pairs, tol));
    rep.pairs = pairs;
    rep.scans = prepared.iter().zip(&covered).map(|(s, c)| first_row(s, *c)).collect();
    if rep.issues.iter().any(|i| i.refuses_under(p.alignment)) {
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
    rep.seam.over_budget = guard.over_budget;
    polled()?;
    let (dropped, count, [recovered, restored]) =
        fill::drop_islands(&mut u, tol, cancel).ok_or(MergeError::Cancelled)?;
    rep.seam.islands_dropped = count;
    rep.seam.recovered.triangles = rep.seam.recovered.triangles.saturating_sub(recovered);
    rep.seam.restored = rep.seam.restored.saturating_sub(restored);
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
        let before = u.triangles.len();
        let r = recover::recover(u, scans, removed, guard, (tol, cancel))?;
        if r.triangles == 0 {
            // nothing changed since that look: its count is what is still uncovered
            rep.recovered.uncovered = r.uncovered;
            return Some(());
        }
        let specks = fill::drop_specks(u, guard);
        if cancel.is_cancelled() {
            return None;
        }
        rep.recovered.rounds += 1;
        rep.recovered.triangles += r.triangles;
        rep.recovered.specks_dropped += specks;
        // this round's own progress: the specks may include triangles of earlier rounds
        let gained = u.triangles.len() > before;
        seam::refill(u, scans, guard, rep, cancel)?;
        if !gained {
            break;
        }
    }
    rep.recovered.uncovered = recover::uncovered(u, scans, removed, tol, cancel)?.len();
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
    if rep.seam.over_budget > 0 {
        rep.issues.push(Issue::OverBudget { refusals: rep.seam.over_budget });
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
