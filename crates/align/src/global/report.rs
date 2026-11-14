//! What [`super::align_global`] reports.
use super::relax::Misfit;
use crate::pair::{CoarseMethod, PairVerdict};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::Transform;

/// The outcome of the whole alignment.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum GlobalVerdict {
    /// Every scan is placed and every pair pose agrees with the final poses within the tolerance.
    Aligned,
    /// Some scans could not be connected to the fixed one (they are not moved), or the poses do not agree within the tolerance around a loop.
    Partial,
    /// Nothing could be connected to the fixed scan.
    Failed,
}

/// Why the verdict is not a clean `aligned`, or something to know about it.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum GlobalIssue {
    /// No scan could be tied to the fixed one: every pair was ambiguous, a non-match or failed.
    NotConnected,
    /// Some scans are not connected to the fixed one and stay where they are (see `scans`).
    UnplacedScans,
    /// After relaxation a pair pose still disagrees with the final poses by more than the tolerance: a loop does not close.
    LoopErrorHigh,
    /// A pair pose contradicted the others around a loop by far more than the tolerance and was left out (see `edges`).
    EdgeRejected,
    /// Not every pair was tried (the cap `max_pairs` was reached): a scan left unplaced may fit one that was not tried.
    PairsCapped,
    /// The relaxation stopped on its iteration limit, or could not take a first step.
    NotConverged,
    /// Some placed scans hang on a single pair that no loop confirms (`held_by_loop` false in `scans`): the pair was `unique`, but a wrong pose of that kind (see `edges` for its confidence, overlap, stability and probes) cannot be told from a right one by the other scans. Check them by eye.
    WeaklyHeld,
    /// More pairs contradicted the others than agreed with them (over 1.5 rejected per used edge): most of the pair stage was wrong, and so may be the part that survived.
    ManyEdgesRejected,
    /// The vote that left pairs out was too close: the best consistent set of pairs beat the best different one by less than 2 edges (1 with fewer than 6 pairs), see `consensus_margin`; which pairs are wrong is nearly a coin toss, and the vote is sampled.
    ConsensusUnclear,
}

/// Why a pair did or did not become an edge of the pose graph.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum EdgeStatus {
    /// Used: the pair pose is a measurement in the relaxation.
    Used,
    /// The pair verdict was `ambiguous`: no edge.
    Ambiguous,
    /// The pair verdict was `no_match`: no edge.
    NoMatch,
    /// The pair could not be computed (see `note`): no edge.
    Failed,
    /// Unique, but the common part is too small to serve as a measurement: no edge.
    TooLittleOverlap,
    /// Unique, but between scans that are not connected to the fixed one: not used.
    Unplaced,
    /// Unique, but contradicted the other poses around a loop by far more than the tolerance: left out.
    Rejected,
}

/// Why a scan was left where it is.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Unplaced {
    /// No pair with another scan was `unique`.
    NoUniquePair,
    /// It has unique pairs, but only with scans that are not connected to the fixed one either.
    OtherGroup,
    /// Its unique pairs all contradicted the poses of the others and were left out.
    Contradicted,
    /// None of its pairs was tried (the cap `max_pairs` was reached first).
    PairsNotTried,
}

/// One scan.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ScanPose {
    /// Position in the list of scans.
    pub index: usize,
    /// Is it the fixed scan?
    pub fixed: bool,
    /// Was it connected to the fixed scan? An unplaced scan is not moved (its `pose` is the identity).
    pub placed: bool,
    /// Why it was not placed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub unplaced: Option<Unplaced>,
    /// The motion that brings the scan into the common frame (that of the fixed scan), after relaxation.
    #[schemars(with = "[[f64; 4]; 4]")]
    pub pose: Transform,
    /// The same motion chained along the spanning tree only, before relaxation.
    #[schemars(with = "[[f64; 4]; 4]")]
    pub chained: Transform,
    /// Pair poses that tie the scan to the graph (used edges).
    pub edges_used: usize,
    /// Is the scan held by a closed loop: every pair on its path to the fixed scan lies on a cycle of used pairs? False for a scan that hangs on a single pair (two scans, a chain, a leaf): that pair is unconfirmed. The fixed scan counts as held; an unplaced scan is not.
    pub held_by_loop: bool,
    /// For a placed scan that is not held: the pair on its path to the fixed scan that nothing confirms (the one nearest the scan), as `[fixed, moving]` scan indices; it is a bridge, or lies only in loops with a pair of weak evidence.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub weak_pair: Option<[usize; 2]>,
}

/// One pair of scans.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct EdgeReport {
    /// The scan the pair was solved in (the lower index).
    pub fixed: usize,
    /// The scan that was moved onto `fixed`.
    pub moving: usize,
    /// The pair verdict.
    pub verdict: PairVerdict,
    /// Did the pair become an edge, and if not why.
    pub status: EdgeStatus,
    /// The pair pose (moving -> fixed), before relaxation; the identity when it failed.
    #[schemars(with = "[[f64; 4]; 4]")]
    pub transform: Transform,
    /// Share (0..1) of the moving scan on the fixed one.
    pub overlap_moving: f64,
    /// Share (0..1) of the fixed scan on the moving one.
    pub overlap_fixed: f64,
    /// RMS residual (mm) of the pair's own alignment.
    pub rms: f64,
    /// Tolerance (mm) of the pair.
    pub tolerance: f64,
    /// Confidence (0..1) of the pair pose.
    pub confidence: f64,
    /// Stage that proposed the pair pose (absent when the pair failed).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub method: Option<CoarseMethod>,
    /// Disagreement of the pair pose with the spanning-tree chain (0 on a tree edge); only for used edges.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub misfit_before: Option<Misfit>,
    /// Disagreement of the pair pose with the relaxed poses; only for used edges.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub misfit_after: Option<Misfit>,
    /// RMS residual of the pair as a share of the diagonal of the fixed scan.
    pub rms_per_diagonal: f64,
    /// Does the pair carry strong evidence of its own: the residual below 0.3 % of the diagonal (see `rms_per_diagonal`)? Confidence does not count (it read 1.00 on wrong poses). A loop confirms its scans only when all its pairs are strong.
    pub strong_evidence: bool,
    /// How firmly the matched part pins the pair pose (see `align.pair_coarse`).
    pub stability: f64,
    /// Displaced starts the pair pose was pushed from.
    pub probed: usize,
    /// Of those, the starts whose ICP found no pairs.
    pub probes_failed: usize,
    /// Of those, the starts that settled in a different, equally good pose.
    pub slides: usize,
    /// Smallest clearance of the pair pose over the starts that ended elsewhere (absent when all came back).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub probe_margin: Option<f64>,
    /// Largest distance (mm) a displaced start ended from the pair pose.
    pub probe_return_gap: f64,
    /// Candidate poses that were refined and compared.
    pub refined: usize,
    /// Why the pair failed (for `failed`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

/// Loop-closure figures over a set of cycles.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct LoopStats {
    /// Worst cycle: its largest edge misfit, RMS distance in mm.
    pub worst_mm: f64,
    /// Mean over cycles of that distance, mm.
    pub mean_mm: f64,
    /// Worst cycle: its largest edge misfit as an angle, degrees.
    pub worst_degrees: f64,
    /// Mean over cycles of that angle, degrees.
    pub mean_degrees: f64,
}

/// Loop closure before and after relaxation, over the independent cycles of the graph (used edges
/// minus scans plus one). Per cycle the figure is the largest misfit among the edges of the cycle:
/// before relaxation the whole error sits on the one edge that closes it, after it is spread.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct LoopReport {
    /// Independent cycles (0: the used edges form a tree and nothing can be checked).
    pub cycles: usize,
    /// With the poses chained along the spanning tree.
    pub before: LoopStats,
    /// With the relaxed poses.
    pub after: LoopStats,
    /// Largest misfit among the used edges after relaxation, as a share of that edge's tolerance (the bound for `aligned` is 1).
    pub worst_edge_ratio: f64,
}

/// Everything [`super::align_global`] found.
#[derive(Clone, Debug)]
pub struct GlobalResult {
    /// The overall verdict.
    pub verdict: GlobalVerdict,
    /// Reasons for a verdict other than `aligned`, and notes.
    pub issues: Vec<GlobalIssue>,
    /// One entry per scan, in input order.
    pub scans: Vec<ScanPose>,
    /// Every pair that was tried, in the order tried.
    pub edges: Vec<EdgeReport>,
    /// Loop closure before and after relaxation.
    pub loops: LoopReport,
    /// Pairs tried.
    pub pairs_tried: usize,
    /// Pairs there are (`n (n - 1) / 2`, or those whose boxes meet when starting from the current poses).
    pub pairs_possible: usize,
    /// Relaxation iterations.
    pub iterations: usize,
    /// Pairs used as edges.
    pub edges_used: usize,
    /// Unique pairs left out because they contradicted the others (their misfit at the final poses is in `edges`).
    pub edges_rejected: usize,
    /// How decisive the last vote that left pairs out was: inliers of the winning set minus those of the best different set (0 a tie; a margin below 2 with 6 or more pairs, below 1 with fewer, makes the verdict `partial`); `None` when no pair was left out by a vote.
    pub consensus_margin: Option<usize>,
    /// Trees the vote tried: the strongest-first tree from each scan and seeded random ones. The vote is sampled, not exhaustive.
    pub consensus_trials: usize,
}
