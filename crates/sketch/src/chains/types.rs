//! Chain types: the inputs, options, reports and result of the chain fit.

use super::*;

/// A chain of section samples in sketch coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct Chain2 {
    /// Samples in order.
    pub points: Vec<[f64; 2]>,
    /// Does the last sample connect to the first?
    pub closed: bool,
}

/// What to fit.
#[derive(Clone, Debug, PartialEq)]
pub struct ChainOptions {
    /// Fit tolerance (mm): RMS bound of a piece and of the chain after relations (the maximum may be
    /// three times as large). Use [`ChainOptions::tolerance_for_noise`] for scans of a known noise.
    pub tolerance: f64,
    /// Angle within which lines count as horizontal, vertical, parallel or perpendicular (degrees).
    pub angle_tolerance_degrees: f64,
    /// Turn the sketch frame to the dominant line direction?
    pub align_frame: bool,
}

impl Default for ChainOptions {
    fn default() -> Self {
        Self { tolerance: 0.05, angle_tolerance_degrees: 1.0, align_frame: true }
    }
}

impl ChainOptions {
    /// `max(0.05 mm, 2 σ)` for scan noise `sigma` (mm).
    #[must_use]
    pub fn tolerance_for_noise(sigma: f64) -> f64 {
        if sigma.is_finite() && sigma > 0.0 { (2.0 * sigma).max(0.05) } else { 0.05 }
    }
}

/// RMS and maximum distance of the section samples from their entities.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Deviation {
    /// Root mean square (mm).
    pub rms: f64,
    /// Largest (mm).
    pub max: f64,
}

/// A relation between sketch entities.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RelationKind {
    /// Line parallel to the sketch x axis.
    Horizontal,
    /// Line parallel to the sketch y axis.
    Vertical,
    /// Lines parallel.
    Parallel,
    /// Lines perpendicular.
    Perpendicular,
    /// Lines on one straight line.
    Collinear,
    /// Line and arc, or two arcs, tangent.
    Tangent,
    /// Arcs of equal radius.
    EqualRadius,
    /// Arcs around one centre.
    Concentric,
}

/// A relation that was kept (entity indices are those of the whole sketch).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Relation {
    /// Kind.
    pub kind: RelationKind,
    /// The entities it ties.
    pub entities: Vec<usize>,
}

/// A relation that was proposed and refused.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct RejectedRelation {
    /// Kind.
    pub kind: RelationKind,
    /// The entities it would tie.
    pub entities: Vec<usize>,
    /// Why it was refused.
    pub reason: String,
}

/// What became of one chain.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ChainReport {
    /// Index of the chain in the input.
    pub chain: usize,
    /// Is it closed?
    pub closed: bool,
    /// Samples fitted (after removing duplicates and thinning).
    pub samples: usize,
    /// Index of its first entity in the sketch.
    pub first_entity: usize,
    /// Number of entities (lines, arcs and circles).
    pub entities: usize,
    /// Lines among them.
    pub lines: usize,
    /// Arcs among them.
    pub arcs: usize,
    /// Circles among them.
    pub circles: usize,
    /// Relations kept, in the order they were added.
    pub relations_kept: Vec<Relation>,
    /// Relations refused (the first 200).
    pub relations_rejected: Vec<RejectedRelation>,
    /// How many relations were refused in all.
    pub rejected_total: usize,
    /// Deviation of the fitted lines and arcs, before any relation.
    pub before: Deviation,
    /// Deviation after the relations were solved.
    pub after: Deviation,
    /// Solve status of the chain after its relations.
    pub status: Option<SolveStatus>,
    /// Degrees of freedom left in the chain.
    pub dof: usize,
    /// Why the chain was skipped or has no relations.
    pub note: Option<String>,
}

/// The result of [`fit_chains`].
#[derive(Clone, Debug, PartialEq)]
pub struct ChainsFit {
    /// The sketch (its plane is the input plane turned by `frame_rotation_degrees`).
    pub sketch: Sketch,
    /// How far the sketch frame was turned (degrees, counter-clockwise).
    pub frame_rotation_degrees: f64,
    /// One report per input chain.
    pub chains: Vec<ChainReport>,
    /// Aggregated solve report of the chains (each chain is solved on its own).
    pub report: SolveReport,
}
