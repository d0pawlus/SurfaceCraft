//! The result types of `scan.to_solid`: issues, next steps, the scan summary, what was modelled and
//! how it compares with the scan.
use super::{Method, PrepOut, Verdict};
use crate::cmd::body::BodyOut;
use crate::cmd::qa::Finding;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use surfacecraft_deviation::faces::FaceDeviation;
use surfacecraft_doc::ObjectId;
use surfacecraft_fit::PrimitiveKind;

/// One thing that kept the run from a pass.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct Issue {
    /// Short identifier, stable across versions.
    pub code: String,
    /// What was found.
    pub message: String,
}

/// A command worth running next, with its parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct NextStep {
    /// Command id.
    pub command: String,
    /// Parameters to call it with.
    pub params: Value,
    /// Why.
    pub why: String,
}

/// What the cleaning did to the scan.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ScanSummary {
    /// Triangles when the command started.
    pub triangles_before: usize,
    /// Triangles after cleaning.
    pub triangles: usize,
    /// Holes found before cleaning.
    pub holes_before: usize,
    /// Holes filled.
    pub holes_filled: usize,
    /// Holes left open (above the size limit).
    pub holes_left: usize,
    /// Edges shared by three or more triangles, after cleaning.
    pub non_manifold_edges: usize,
    /// Estimated scanner noise sigma after cleaning (mm); null when it cannot be estimated.
    pub noise_sigma: Option<f64>,
    /// Findings of the quality check before cleaning.
    pub findings: Vec<Finding>,
}

/// A region that became a primitive of the solid.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct Modelled {
    /// Region.
    pub region: ObjectId,
    /// Primitive fitted to it.
    pub primitive: ObjectId,
    /// Kind of the primitive.
    pub kind: PrimitiveKind,
    /// Triangles of the region.
    pub triangles: usize,
    /// RMS of the fit (mm).
    pub rms: f64,
    /// Role in the solid, as `body.from_regions` reports it (`global`, `local-subtract`,
    /// `local-add`, `blend`, `skipped`); empty when no solid was built.
    pub role: String,
    /// Faces of the solid on this primitive.
    pub faces: Vec<usize>,
    /// Why it was skipped, when it was.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub skipped: Option<String>,
}

/// A region that was not modelled.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct LeftOut {
    /// Region.
    pub region: ObjectId,
    /// Triangles.
    pub triangles: usize,
    /// Classified kind.
    pub kind: Option<PrimitiveKind>,
    /// RMS of the classification (mm).
    pub rms: Option<f64>,
    /// Why it was left out.
    pub reason: String,
}

/// The solid against the scan.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct DeviationSummary {
    /// Scan points measured.
    pub count: usize,
    /// Share of the scan's points that were measured (0..1): the rest lie farther than 5 mm from
    /// the solid or face away from it.
    pub coverage: f64,
    /// Share of the scan's points (0..1) that lie on the solid but face away from it, and so are
    /// not in `count`: a solid that is inside out, or has faces turned the wrong way, shows here.
    #[serde(default)]
    pub not_facing: f64,
    /// Share of the measured points within the tolerance (0..1).
    pub within: f64,
    /// RMS deviation (mm).
    pub rms: f64,
    /// Largest |deviation| (mm).
    pub max_abs: f64,
    /// Mean signed deviation (mm); positive when the scan lies outside the solid.
    pub mean: f64,
}

/// Result of `scan.to_solid`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ToSolidOut {
    /// How the run ended: `pass`, `review` or `fail`.
    pub verdict: Verdict,
    /// The method used: `primitives` or `freeform`.
    pub method: Method,
    /// The scan.
    pub scan: ObjectId,
    /// The solid, when one was built.
    pub body: Option<BodyOut>,
    /// Patches of the network (method `freeform`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub patches: Option<usize>,
    /// The network was built a second time with a larger patch limit (method `freeform`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retry: Option<super::freeform::CapRetry>,
    /// Tolerance used (mm).
    pub tolerance: f64,
    /// The solid against the scan, when a solid was built.
    pub deviation: Option<DeviationSummary>,
    /// Faces with points outside the tolerance, worst first (at most 5).
    pub worst_faces: Vec<FaceDeviation>,
    /// The scan before and after cleaning.
    pub scan_report: ScanSummary,
    /// Regions modelled, largest first.
    pub modelled: Vec<Modelled>,
    /// Regions not modelled, largest first (at most 20).
    pub left_out: Vec<LeftOut>,
    /// Regions not modelled, all of them.
    pub left_out_regions: usize,
    /// Triangles of the regions not modelled.
    pub left_out_triangles: usize,
    /// Triangles of the scan in no region.
    pub unassigned_triangles: usize,
    /// Relations imposed between the primitives.
    pub relations: usize,
    /// What kept the run from a pass; empty on a pass.
    pub issues: Vec<Issue>,
    /// Commands worth running next, most useful first.
    pub next_steps: Vec<NextStep>,
    /// Preparation: what was done to the scan before modelling it, one result per part when it was split.
    #[serde(flatten)]
    pub prep: PrepOut,
}
