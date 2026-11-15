//! Closing small handles of a mesh: tunnels, bridges and the sponge of tiny tunnels a scanner makes
//! where the two sides of a thin wall come closer than its resolution. They stall edge collapse (a
//! collapse that would close a tunnel is refused, so the reduction stops with the handle region at
//! full resolution) and cost a surface fit its patch budget.
//!
//! Method (topological noise removal in the spirit of Guskov and Wood 2001: find the small
//! neighbourhoods that are not disks, cut them out, fill the holes): around seeds a quarter of a
//! scale apart along the surface, the geodesic balls one scale across are looked at, from four
//! median edges up to `max_size`, each scale √2 times the one before, and at `max_size`; every
//! handle at most half a scale across lies wholly in the ball of its nearest seed. A connected part
//! of a ball that is not a disk (genus above zero, or several boundary loops that are joined
//! outside it) is cut out and each of its loops filled with the smooth hole fill. Each such part
//! closes `genus + loops - 1` handles. Nothing else moves: vertices outside the cut parts keep
//! their positions and the rim of an open mesh is never touched (a part that reaches the rim, an
//! edge of three or more triangles or a pinched vertex is left). A part is refused when cutting it
//! would split the surface or leave a piece loose, when it is a whole closed piece, when a hole stays
//! open, when more than a quarter of one patch of its fill lies farther than `tolerance` from what
//! it replaced (a cap over the mouth of a real opening), or when what it removed is not covered by
//! the result: more than a quarter of the points sampled over its triangles farther than
//! `tolerance`, or any farther than a quarter of the ball. A mesh where nothing is cut is returned
//! unchanged, bit for bit. The size is a property of the neighbourhood, not of the hole: a real hole
//! through a thin wall narrower than `max_size` is a tunnel and is closed too, a wider one is kept.
mod cells;
mod cut;

use crate::{Bvh, Cancel, Mesh, Point3, edit, par, pieces};
use cells::{Candidate, Topo};
use cut::Refusal;

/// The smallest scale, in median edges: a ball narrower than this holds no ring of triangles around
/// a tunnel.
pub const SMALLEST_EDGES: f64 = 4.0;
/// Each scale is this much larger than the one before (a tunnel one edge wide needs a ball of about
/// five edges, one between two others a ball of about four).
const SCALE_STEP: f64 = std::f64::consts::SQRT_2;
/// Attempts per pass: the candidates that failed are grown or left out and the rest is cut again.
const ATTEMPTS: usize = 5;
/// A candidate that failed is grown by a ring of triangles at most this many times.
const MAX_GROWTH: usize = 2;
/// Passes per scale at most: a pass cuts candidates that are apart; the next one, with seeds taken
/// from another vertex on (other balls), finds those that overlapped a cut one or whose ball cut a
/// tunnel in two.
const PASSES: usize = 6;

/// Settings of [`close_handles`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Params {
    /// Handles in patches of the surface at most this wide (mm) are looked for (at most the diagonal of
    /// the mesh).
    pub max_size: f64,
    /// A fill of which more than a quarter lies farther than this (mm) from the surface it replaces is
    /// refused; removed vertices are measured against the result with it.
    pub tolerance: f64,
}

/// Why nothing was done.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum HandleError {
    /// A setting is out of range.
    #[error("{0}")]
    Params(String),
    /// The mesh is not valid (an index out of range, a coordinate that is not finite).
    #[error("the mesh is invalid: {0}")]
    Invalid(String),
    /// The caller cancelled.
    #[error("cancelled")]
    Cancelled,
}

/// One part that was cut out and filled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Closed {
    /// Scale of the patches it was found at (mm).
    pub scale: f64,
    /// Handles it closed.
    pub handles: usize,
    /// Loops it left (holes filled).
    pub loops: usize,
    /// Triangles cut out.
    pub triangles: usize,
    /// Area cut out (mm²).
    pub area: f64,
    /// Centre of the triangles cut out.
    pub centre: Point3,
    /// Largest distance of its fill from the surface it replaced (mm).
    pub fill_distance: f64,
}

/// Candidates that were left, by reason (counted once per pass in which they were refused).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Refused {
    /// Cutting would split the surface (the loops are not joined outside the part, or a piece hanging
    /// on it would come loose).
    pub splits: usize,
    /// A hole could not be filled.
    pub unfillable: usize,
    /// More than a quarter of the fill lay farther than the tolerance from what it replaced.
    pub fill_far: usize,
    /// More than a quarter of what was cut out lay farther than the tolerance from the result (the fill
    /// did not cover it), a point of it lay farther than a quarter of the ball, or the fill left a
    /// pinched vertex.
    pub uncovered: usize,
    /// Whole closed pieces with genus that fit in a ball: cutting one out would delete the piece.
    pub whole_pieces: usize,
}

/// What [`close_handles`] did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Report {
    /// Median edge of the mesh (mm).
    pub median_edge: f64,
    /// Scales of the balls (mm), smallest first; empty when `max_size` is below four median edges.
    pub scales: Vec<f64>,
    /// Genus of the mesh before (sum over its pieces; `None` when a piece has none: pinched, non-manifold).
    pub genus_before: Option<usize>,
    /// ... and after.
    pub genus_after: Option<usize>,
    /// Connected pieces before (a part is never cut loose, so the same after).
    pub pieces_before: usize,
    /// ... and after.
    pub pieces_after: usize,
    /// Boundary loops before.
    pub loops_before: usize,
    /// ... and after.
    pub loops_after: usize,
    /// Handles closed (sum over the parts cut out).
    pub handles_closed: usize,
    /// The parts cut out.
    pub closed: Vec<Closed>,
    /// Candidates left.
    pub refused: Refused,
    /// Triangles of the mesh given that were cut out.
    pub removed_triangles: usize,
    /// Area of the mesh given that was cut out (mm²).
    pub removed_area: f64,
    /// Share of the area cut out.
    pub removed_area_share: f64,
    /// Triangles the fills added.
    pub added_triangles: usize,
    /// Points sampled over the triangles of the mesh given that were cut out (corners, edge midpoints,
    /// centres and a lattice of steps of at most two tolerances): the walls of a tunnel count even when
    /// they are long triangles without a vertex inside.
    pub removed_samples: usize,
    /// Of them, within `tolerance` of the result.
    pub removed_within: usize,
    /// Largest distance of such a point from the result (mm).
    pub removed_max_distance: f64,
    /// Did the mesh change?
    pub changed: bool,
}

fn check(p: &Params) -> Result<(), HandleError> {
    if !(p.max_size.is_finite() && p.max_size > 0.0) {
        return Err(HandleError::Params(format!("max_size must be a positive number of mm, not {}", p.max_size)));
    }
    if !(p.tolerance.is_finite() && p.tolerance > 0.0) {
        return Err(HandleError::Params(format!("tolerance must be a positive number of mm, not {}", p.tolerance)));
    }
    Ok(())
}

/// Median length of the edges (0 for a mesh without edges).
fn median_edge(mesh: &Mesh) -> f64 {
    let all: Vec<u32> = (0..mesh.triangles.len() as u32).collect();
    let mut l: Vec<f64> = cells::edge_counts(mesh, &all)
        .iter()
        .filter_map(|&((a, b), _)| Some((mesh.positions.get(a as usize)? - mesh.positions.get(b as usize)?).norm()))
        .collect();
    if l.is_empty() {
        return 0.0;
    }
    let k = l.len() / 2;
    *l.select_nth_unstable_by(k, f64::total_cmp).1
}

/// The scales from [`SMALLEST_EDGES`] median edges up to `max_size`, by [`SCALE_STEP`], and `max_size` itself
/// last.
fn scales(h: f64, max_size: f64) -> Vec<f64> {
    let mut out = Vec::new();
    let mut r = SMALLEST_EDGES * h;
    while r.is_finite() && r > 0.0 && r < max_size && out.len() < 64 {
        out.push(r);
        r *= SCALE_STEP;
    }
    if r.is_finite() && r > 0.0 && SMALLEST_EDGES * h <= max_size {
        out.push(max_size);
    }
    out
}

/// Genus (when every piece has one), boundary loops, pieces, and the handles from the Euler
/// characteristic of every piece (what is left to close, also on a mesh with pinches).
fn topology(mesh: &Mesh) -> (Option<usize>, usize, usize, i64) {
    let ps = pieces::pieces(mesh);
    let genus = ps.iter().map(pieces::Piece::genus).sum::<Option<usize>>();
    let estimate = ps.iter().map(|p| (2 - p.euler - p.loops.len() as i64).max(0) / 2).sum();
    (genus, ps.iter().map(|p| p.loops.len()).sum(), ps.len(), estimate)
}

/// The mesh as the passes change it.
struct Work {
    mesh: Mesh,
    /// Per triangle: it is a triangle of the mesh given (not a fill).
    origin: Vec<bool>,
    /// Points sampled over the triangles of the mesh given that were cut out.
    removed: Vec<Point3>,
}

/// Closes the handles of `mesh` that lie in patches at most `max_size` wide (see the module). The
/// mesh is replaced only when something was closed.
pub fn close_handles(mesh: &mut Mesh, p: &Params, cancel: &Cancel) -> Result<Report, HandleError> {
    check(p)?;
    mesh.validate().map_err(|e| HandleError::Invalid(e.to_string()))?;
    let (genus_before, loops_before, pieces_before, estimate) = topology(mesh);
    let h = median_edge(mesh);
    // no neighbourhood is wider than the mesh
    let max_size = p.max_size.min(mesh.bbox().diagonal().max(f64::MIN_POSITIVE));
    let mut rep = Report {
        median_edge: h,
        scales: scales(h, max_size),
        genus_before,
        genus_after: genus_before,
        loops_before,
        loops_after: loops_before,
        pieces_before,
        pieces_after: pieces_before,
        ..Report::default()
    };
    if genus_before == Some(0) || estimate == 0 || rep.scales.is_empty() {
        return Ok(rep);
    }
    let mut work = Work { mesh: mesh.clone(), origin: vec![true; mesh.triangles.len()], removed: Vec::new() };
    for scale in rep.scales.clone() {
        if rep.handles_closed as i64 >= estimate {
            break;
        }
        work = scale_passes(work, scale, p.tolerance, &mut rep, cancel)?;
    }
    if rep.closed.is_empty() {
        return Ok(rep);
    }
    finish(mesh, work, p.tolerance, &mut rep);
    Ok(rep)
}

/// The passes at one scale, until two in a row cut nothing or no handle is left.
fn scale_passes(
    mut work: Work,
    scale: f64,
    tolerance: f64,
    rep: &mut Report,
    cancel: &Cancel,
) -> Result<Work, HandleError> {
    let mut idle = 0;
    for k in 0..PASSES {
        if cancel.is_cancelled() {
            return Err(HandleError::Cancelled);
        }
        let first = k * work.mesh.positions.len() / PASSES;
        let bounds = cut::Bounds { tolerance, far_limit: (0.25 * scale).max(tolerance) };
        let (next, cut) = pass(work, (scale, first), bounds, rep, cancel)?;
        work = next;
        idle = if cut { 0 } else { idle + 1 };
        if idle >= 2 {
            break;
        }
    }
    Ok(work)
}

/// One pass at one scale: the candidates that are apart, cut and filled. A candidate that fails is
/// grown by a ring (up to [`MAX_GROWTH`] times: its ball may have cut a tunnel in two) or left out, and
/// the rest is cut again. Returns the mesh as it is after the pass and whether anything was cut.
fn pass(
    work: Work,
    (scale, first): (f64, usize),
    bounds: cut::Bounds,
    rep: &mut Report,
    cancel: &Cancel,
) -> Result<(Work, bool), HandleError> {
    let topo = Topo::new(&work.mesh);
    let (found, whole) = cells::candidates(&work.mesh, &topo, scale, first);
    rep.refused.whole_pieces += whole.len();
    let mut cands = cells::pick(&topo, found, work.mesh.positions.len());
    let mut skip = vec![false; cands.len()];
    let mut grown = vec![0usize; cands.len()];
    for g in cut::splitting(&work.mesh, &cands, &skip) {
        refuse(rep, Refusal::Loosens);
        set(&mut skip, g);
    }
    for _ in 0..ATTEMPTS {
        if skip.iter().all(|&s| s) {
            return Ok((work, false));
        }
        let Some(mut a) = cut::attempt(&work.mesh, &cands, &skip, bounds, cancel) else {
            return Err(HandleError::Cancelled);
        };
        if a.failed.is_empty() {
            a.failed =
                cut::loosened(&work.mesh, &a.mesh, &cands, &skip).into_iter().map(|g| (g, Refusal::Loosens)).collect();
        }
        if a.failed.is_empty() {
            record(&work, &cands, &skip, &a, scale, rep);
            return Ok((accepted(work, a, bounds.tolerance), true));
        }
        retry_or_skip(&work.mesh, &topo, (&mut cands, &mut skip, &mut grown), &a.failed, rep);
    }
    Ok((work, false))
}

fn set(flags: &mut [bool], g: usize) {
    if let Some(s) = flags.get_mut(g) {
        *s = true;
    }
}

/// Each failed candidate is grown by a ring when it may grow and stays apart from the others, else it
/// is left out and its refusal counted.
fn retry_or_skip(
    mesh: &Mesh,
    topo: &Topo,
    (cands, skip, grown): (&mut [Candidate], &mut [bool], &mut [usize]),
    failed: &[(usize, Refusal)],
    rep: &mut Report,
) {
    let live: Vec<bool> = skip.iter().map(|s| !s).collect();
    let owner = cells::claims(topo, cands, &live, mesh.positions.len());
    for &(g, why) in failed {
        let bigger = (why != Refusal::Loosens && grown.get(g).is_some_and(|&n| n < MAX_GROWTH))
            .then(|| cands.get(g).and_then(|c| cells::grow(mesh, topo, c)))
            .flatten()
            .filter(|c| {
                c.vertices.iter().all(|&v| owner.get(v as usize).is_some_and(|&o| o == u32::MAX || o == g as u32))
            });
        match (bigger, cands.get_mut(g), grown.get_mut(g)) {
            (Some(c), Some(slot), Some(n)) => {
                *slot = c;
                *n += 1;
            }
            _ => {
                refuse(rep, why);
                set(skip, g);
            }
        }
    }
}

/// The mesh of an accepted attempt, with the origin of its triangles and the points sampled over the
/// triangles of the mesh given that it cut out.
fn accepted(work: Work, a: cut::Attempt, tolerance: f64) -> Work {
    let mut removed = work.removed;
    let mut origin = Vec::with_capacity(a.mesh.triangles.len());
    for (t, (&dead, &orig)) in a.dead.iter().zip(&work.origin).enumerate() {
        if !dead {
            origin.push(orig);
        } else if orig && let Some(tri) = work.mesh.triangle(t) {
            removed.extend(cut::samples(tri, 2.0 * tolerance));
        }
    }
    origin.resize(a.mesh.triangles.len(), false);
    Work { mesh: a.mesh, origin, removed }
}

fn refuse(rep: &mut Report, why: Refusal) {
    match why {
        Refusal::Unfillable => rep.refused.unfillable += 1,
        Refusal::FillFar => rep.refused.fill_far += 1,
        Refusal::Uncovered => rep.refused.uncovered += 1,
        Refusal::Loosens => rep.refused.splits += 1,
    }
}

/// Adds the parts cut in an accepted attempt to the report; only triangles of the mesh given count as
/// cut out (fills of earlier passes do not).
fn record(work: &Work, cands: &[Candidate], skip: &[bool], a: &cut::Attempt, scale: f64, rep: &mut Report) {
    let m = &work.mesh;
    for (g, c) in cands.iter().enumerate().filter(|(g, _)| !skip.get(*g).copied().unwrap_or(true)) {
        let own: Vec<u32> =
            c.triangles.iter().copied().filter(|&t| work.origin.get(t as usize).copied().unwrap_or(false)).collect();
        let area: f64 = own.iter().map(|&t| m.face_area(t as usize)).sum();
        let n = c.triangles.len().max(1) as f64;
        let sum = c
            .triangles
            .iter()
            .filter_map(|&t| m.face_centroid(t as usize))
            .fold(Point3::origin().coords, |s, q| s + q.coords);
        rep.closed.push(Closed {
            scale,
            handles: c.handles(),
            loops: c.loops.len(),
            triangles: own.len(),
            area,
            centre: Point3::from(sum / n),
            fill_distance: a.fill_distance.get(g).copied().unwrap_or(0.0),
        });
        rep.handles_closed += c.handles();
        rep.removed_triangles += own.len();
        rep.removed_area += area;
    }
    rep.added_triangles += a.added;
}

/// Measures the points sampled over what was cut out against the result, drops the unused vertices,
/// and puts the result in place.
fn finish(mesh: &mut Mesh, work: Work, tolerance: f64, rep: &mut Report) {
    let mut result = work.mesh;
    let bvh = Bvh::build(&result);
    let d: Vec<f64> =
        par::map_slice(&work.removed, |q| bvh.closest(q, f64::INFINITY).map_or(f64::INFINITY, |hit| hit.distance));
    rep.removed_samples = d.len();
    rep.removed_within = d.iter().filter(|&&x| x <= tolerance).count();
    rep.removed_max_distance = d.iter().copied().fold(0.0, f64::max);
    let area = mesh.area();
    rep.removed_area_share = if area > 0.0 { rep.removed_area / area } else { 0.0 };
    edit::compact(&mut result);
    let (genus_after, loops_after, pieces_after, _) = topology(&result);
    rep.genus_after = genus_after;
    rep.loops_after = loops_after;
    rep.pieces_after = pieces_after;
    rep.changed = true;
    *mesh = result;
}

#[cfg(test)]
mod tests;
