//! `align_global`: several scans of one part into one common pose at once (design doc §6.1, P9-M3).
//!
//! Method:
//! 1. Pairs: every pair of scans for a small set, the pairs of neighbours in the list first and
//!    cut at `max_pairs` for a larger one; each solved by [`crate::pair_coarse`] (or started from
//!    the poses the scans already have). Only a `unique` pair with enough overlap is an edge; an
//!    `ambiguous` or `no_match` pair is not.
//! 2. Graph: edges weighted by overlap over squared residual; the maximum spanning tree from the
//!    fixed scan (strongest first) places the scans it reaches by chaining pair poses.
//! 3. Relaxation ([`relax`]): all poses move together to the least-squares agreement with every
//!    edge, so the disagreement around a loop is spread over the loop. An edge that contradicts
//!    the rest by far more than its tolerance is left out and the relaxation is repeated.
//! 4. Report: poses, edges, loop closure before and after, and a verdict that is `aligned` only
//!    when every scan is connected to the fixed one and no edge disagrees with the final poses by
//!    more than its tolerance.
//!
//! Scans the graph does not connect to the fixed one are not moved. Cost: the pair stage, at most
//! `max_pairs` pair alignments (about 36 by default; a pair is roughly 1 to 6 s on scans of a few
//! hundred thousand triangles), then BVHs per scan and a dense solve of 6 unknowns per scan.
mod graph;
mod loops;
mod pairs;
mod relax;
mod report;
mod solve;
mod verdict;

use crate::AlignError;
pub use relax::{Edge, Misfit, Relaxed, misfit, relax};
pub use report::{
    EdgeReport, EdgeStatus, GlobalIssue, GlobalResult, GlobalVerdict, LoopReport, LoopStats, ScanPose, Unplaced,
};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::{Cancel, Mesh};

/// Most scans one call takes.
pub const MAX_SCANS: usize = 24;
/// Pairs tried by default.
pub const DEFAULT_MAX_PAIRS: usize = 36;
/// Most pairs a call may ask for (all pairs of [`MAX_SCANS`] scans).
pub const MAX_PAIRS_LIMIT: usize = MAX_SCANS * (MAX_SCANS - 1) / 2;
/// An edge that disagrees with the final poses by more than this many tolerances is left out.
pub const REJECT_FACTOR: f64 = 2.5;
/// `aligned` needs every used edge to agree with the final poses within this many tolerances.
pub const LOOP_BOUND: f64 = 1.0;
/// Fewest triangles a scan may have.
const MIN_TRIANGLES: usize = 4;

/// Where the search for pair poses starts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum StartMode {
    /// No assumption: the full pair search (axes and shape descriptors) for every pair.
    #[default]
    Search,
    /// The scans are already roughly in place: every pair starts from the current poses (plus the principal-axes poses), and pairs whose boxes do not meet are skipped.
    Current,
}

/// Settings of [`align_global`].
#[derive(Clone, Debug, PartialEq)]
pub struct GlobalOptions {
    /// The scan that stays where it is.
    pub fixed: usize,
    /// Tolerance (mm) of every pair; `None` takes it from each pair's own residual.
    pub tolerance: Option<f64>,
    /// How the pair poses start.
    pub start: StartMode,
    /// Most pairs tried.
    pub max_pairs: usize,
}

impl Default for GlobalOptions {
    fn default() -> Self {
        Self { fixed: 0, tolerance: None, start: StartMode::Search, max_pairs: DEFAULT_MAX_PAIRS }
    }
}

fn check_scan(k: usize, m: &Mesh) -> Result<(), AlignError> {
    if m.triangle_count() < MIN_TRIANGLES {
        let n = m.triangle_count();
        return Err(AlignError::NotEnough(format!("scan {k} has {n} triangles; at least {MIN_TRIANGLES} are needed")));
    }
    m.validate().map_err(|e| AlignError::Degenerate(format!("scan {k} is invalid: {e}")))?;
    let (area, diagonal) = (m.area(), m.bbox().diagonal());
    if !(area.is_finite() && area > 0.0 && diagonal.is_finite() && diagonal > 0.0) {
        return Err(AlignError::Degenerate(format!("scan {k} has no finite surface area")));
    }
    Ok(())
}

fn check_input(scans: &[&Mesh], opts: &GlobalOptions) -> Result<(), AlignError> {
    if scans.len() < 2 || scans.len() > MAX_SCANS {
        return Err(AlignError::NotEnough(format!("{} scans given; between 2 and {MAX_SCANS} are needed", scans.len())));
    }
    if opts.fixed >= scans.len() {
        return Err(AlignError::Degenerate(format!("the fixed scan {} is not among the {} scans", opts.fixed, scans.len())));
    }
    if opts.max_pairs == 0 || opts.max_pairs > MAX_PAIRS_LIMIT {
        return Err(AlignError::Degenerate(format!("max_pairs must be between 1 and {MAX_PAIRS_LIMIT}")));
    }
    scans.iter().enumerate().try_for_each(|(k, m)| check_scan(k, m))
}

/// Brings `scans` into the frame of `scans[opts.fixed]`: the motion for every scan (the fixed one
/// and those that could not be connected get the identity and are reported as not placed), the
/// pair evidence, the loop closure before and after relaxation and a verdict. Nothing is moved:
/// the caller applies the poses it trusts.
///
/// # Errors
/// Input that cannot be tried (fewer than two or more than [`MAX_SCANS`] scans, a scan that is
/// empty or not finite, a bad `fixed` or `max_pairs`), or [`AlignError::Cancelled`].
pub fn align_global(scans: &[&Mesh], opts: &GlobalOptions, cancel: &Cancel) -> Result<GlobalResult, AlignError> {
    check_input(scans, opts)?;
    let (list, possible) = pairs::pair_list(scans, opts);
    let tried = pairs::run_pairs(scans, &list, opts, cancel)?;
    let cands = solve::candidates(scans, &tried, cancel)?;
    solve::solve(scans.len(), opts, cands, possible, tried.len(), cancel)
}
