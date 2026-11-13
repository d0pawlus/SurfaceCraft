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
    /// The relaxation stopped on its iteration limit.
    NotConverged,
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
}
