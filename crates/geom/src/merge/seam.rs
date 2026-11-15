//! Closing the seam: ball pivoting on the band between the kept regions ([`crate::recon::band`]),
//! from the edges the removal cut and only through their vertices. A triangle is built only when
//! it lies on surface some scan saw (its centre and the middles of its edges within the tolerance of
//! facing surface of the scans as given: no surface is made up over an opening nobody scanned) and
//! crosses no triangle there already.
use super::guard::{Corners, Guard, Pts};
use super::prep::Scan;
use super::union::{Origin, Union};
use crate::Cancel;
use crate::recon::band::{BandInput, pivot_band};

/// Ball radii of the passes, in median lengths of the front edges.
pub const RADII_EDGES: [f64; 3] = [1.0, 2.0, 4.0];
/// Rounds of taking back the seam triangles at a pinched vertex and filling the holes again.
const UNPINCH_ROUNDS: usize = 4;
/// Holes of at most this many edges that no triangle could fill are widened by a ring ...
const WIDEN_EDGES: usize = 8;
/// ... in this many rounds at most.
const WIDEN_ROUNDS: usize = 1;

/// What the seam step did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SeamReport {
    /// Border edges the removal cut (both sides of the seam).
    pub front_edges: usize,
    /// Their length (mm).
    pub front_length: f64,
    /// Median length of those edges (mm).
    pub front_median: f64,
    /// Ball radii used (mm).
    pub radii: Vec<f64>,
    /// Removed triangles restored because the union had all their corners at exactly their places.
    pub restored: usize,
    /// Triangles built to close the seam by pivoting.
    pub built: usize,
    /// Rounds of widening done (at most one per merge).
    pub widen_rounds: usize,
    /// Small holes left by the pivoting that were filled, and the triangles that took.
    pub filled_loops: usize,
    pub fill_triangles: usize,
    /// Seam triangles taken back because they met another fan at a vertex only (a pinch) or bordered a
    /// small hole no triangle could fill, before the hole was filled again.
    pub unpinched: usize,
    /// Candidate triangles refused because no scan saw surface there.
    pub unsupported: usize,
    /// Candidate triangles refused because they crossed a triangle there already.
    pub crossing: usize,
    /// Candidate triangles refused by the corner rule (two rims bridged, a rim facing away).
    pub bad_corners: usize,
    /// Refusals because the crossing test would have compared more triangles than allowed (a pile, or
    /// a very long triangle among many new ones): candidate triangles, and holes the fill left open
    /// for that reason (once per fill pass).
    pub over_budget: usize,
    /// Removed triangles the seam left uncovered, and those put back.
    pub recovered: super::recover::Recovered,
    /// Triangles of small pieces dropped because they lay wholly on the rest of the result.
    pub islands_dropped: usize,
    /// Front edges left open (no triangle on their other side) after the seam step.
    pub open_edges: usize,
    /// Their length (mm).
    pub open_length: f64,
    /// Length of the cut edges that were closed (both sides; half of it is the seam's length) (mm).
    pub closed_length: f64,
}

/// Median of the lengths (0 for none).
fn median(mut l: Vec<f64>) -> f64 {
    if l.is_empty() {
        return 0.0;
    }
    let k = l.len() / 2;
    *l.select_nth_unstable_by(k, f64::total_cmp).1
}

fn edge_length(u: &Union, f: &[u32; 3]) -> f64 {
    match (u.positions.get(f[0] as usize), u.positions.get(f[1] as usize)) {
        (Some(p), Some(q)) => (p - q).norm(),
        _ => 0.0,
    }
}

/// Closes the seam of `u` with the guard `guard`; `None` when cancelled.
pub(super) fn close(
    u: &mut Union,
    scans: &[Scan],
    guard: &mut Guard<'_>,
    rep: &mut SeamReport,
    cancel: &Cancel,
) -> Option<()> {
    let front = u.front(scans);
    u.initial_front = front.iter().map(|f| [f[0], f[1]]).collect();
    let lengths: Vec<f64> = front.iter().map(|f| edge_length(u, f)).collect();
    rep.front_edges = front.len();
    rep.front_length = lengths.iter().fold(0.0, |s, l| s + l);
    rep.front_median = median(lengths);
    rep.radii = RADII_EDGES.iter().map(|k| k * rep.front_median).collect();
    if front.is_empty() || !(rep.front_median > 0.0) {
        return Some(());
    }
    guard.corners = Some(Corners::new(u, scans, &front));
    if cancel.is_cancelled() {
        return None;
    }
    let pts = Pts::of(u);
    let mut accept = |t: [u32; 3]| guard.accept(pts, t);
    let input = BandInput { points: &u.positions, normals: &u.normals, triangles: &u.triangles, front: &front };
    let built = pivot_band(&input, &rep.radii, &mut accept, cancel)?;
    rep.built = built.len();
    u.origin.extend(std::iter::repeat_n(Origin::Seam, built.len()));
    u.triangles.extend(built);
    refill(u, scans, guard, rep, cancel)?;
    measure_open(u, scans, rep);
    Some(())
}

/// Fills the small holes of the seam, takes back the seam triangles at pinched vertices and fills
/// again, a few rounds.
pub(super) fn refill(
    u: &mut Union,
    scans: &[Scan],
    guard: &mut Guard<'_>,
    rep: &mut SeamReport,
    cancel: &Cancel,
) -> Option<()> {
    for round in 0..UNPINCH_ROUNDS {
        let (loops, tris, left) = super::fill::fill_small(u, scans, guard, cancel)?;
        rep.filled_loops += loops;
        rep.fill_triangles += tris;
        // a twisted corner is a hole of a few edges; widening it once may let it be filled, widening
        // again and again only opens the seam
        let tiny: Vec<Vec<u32>> = left.into_iter().filter(|l| l.len() <= WIDEN_EDGES).collect();
        // once per merge: the refill after the recovery does not widen again
        let widened = if round < WIDEN_ROUNDS && rep.widen_rounds == 0 {
            let w = super::fill::widen(u, guard, &tiny);
            // counted only when something was widened: a later refill may still widen once
            rep.widen_rounds += usize::from(w > 0);
            w
        } else {
            0
        };
        let undone = super::fill::unpinch(u, guard) + widened;
        if cancel.is_cancelled() {
            return None;
        }
        rep.unpinched += undone;
        if undone == 0 {
            break;
        }
    }
    // the last round may have taken triangles back: fill what that opened
    let (loops, tris, _) = super::fill::fill_small(u, scans, guard, cancel)?;
    rep.filled_loops += loops;
    rep.fill_triangles += tris;
    Some(())
}

/// The open seam edges of `u` and their length.
pub(super) fn measure_open(u: &Union, scans: &[Scan], rep: &mut SeamReport) {
    let open = u.front(scans);
    rep.open_edges = open.len();
    rep.open_length = open.iter().fold(0.0, |s, f| s + edge_length(u, f));
    let edges = u.directed_edges();
    rep.closed_length = u
        .initial_front
        .iter()
        .filter(|e| edges.contains(&super::union::directed(e[1], e[0])))
        .fold(0.0, |s, e| s + edge_length(u, &[e[0], e[1], e[1]]));
}
