//! The size check (P9-U1): is a scan plausibly a mechanical part in millimetres? A scan file has no
//! unit, so a file in metres or inches read as millimetres is far too small, and one in
//! micrometres is far too large. The check only reports it; it never rescales anything.
//!
//! The rule. The robust extent is the largest of the three axis extents of the vertices, each
//! measured between the extreme vertices left once the most extreme ones at each end are cut: one
//! vertex from every 100 (at least one from eight vertices up), so that a far-off vertex does not
//! decide it. Below 1.0 mm the size looks like metres (or inches). Below 0.02 mm it is definitely not
//! millimetres. Above 20 000 mm it looks like micrometres or another unit. Anything in between is
//! fine and gives no warning.
use crate::cmd::qa::{Finding, Severity};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::Mesh;

/// The code of the warning, in `mesh.import`, `mesh.info`, `scan.qa` and `scan.to_solid`.
pub(crate) const SUSPICIOUS_SCALE: &str = "suspicious_scale";
/// Share of the vertices cut from each end of an axis before its extent is taken.
const TRIM: f64 = 0.01;
/// Meshes with fewer vertices are measured as they are (cutting would drop real corners).
const MIN_TRIMMED_VERTICES: usize = 8;
/// Extents below this (mm) look like metres or inches.
const METRES_BELOW_MM: f64 = 1.0;
/// Extents below this (mm) are definitely not millimetres.
const NOT_MILLIMETRES_BELOW_MM: f64 = 0.02;
/// Extents above this (mm) look like micrometres or another unit.
const MICROMETRES_ABOVE_MM: f64 = 20_000.0;

/// What the size looks like wrong.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ScaleProblem {
    /// Below 1 mm: metres, or inches.
    MetresOrInches,
    /// Below 0.02 mm: definitely not millimetres.
    NotMillimetres,
    /// Above 20 000 mm: micrometres, or another unit.
    Micrometres,
}

/// The `suspicious_scale` warning.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ScaleWarning {
    /// Always `suspicious_scale`.
    pub code: String,
    /// What the size looks like.
    pub problem: ScaleProblem,
    /// Robust extent of the mesh, the largest axis (mm).
    pub extent: f64,
    /// What was found.
    pub message: String,
    /// The calls that fix it: a re-import with `units`, or `mesh.set_units` for this mesh.
    pub suggestion: String,
}

/// How many of the most extreme vertices are cut at each end of an axis, for `n` vertices.
fn cut_per_end(n: usize) -> usize {
    if n < MIN_TRIMMED_VERTICES { 0 } else { ((n as f64 * TRIM).floor() as usize).max(1) }
}

/// The value of rank `rank` (0 = smallest) of `values`, which it reorders.
fn order_statistic(values: &mut [f64], rank: usize) -> f64 {
    let (_, value, _) = values.select_nth_unstable_by(rank, f64::total_cmp);
    *value
}

/// The largest of the trimmed axis extents of the vertices (mm); `None` for a mesh without
/// vertices or with a coordinate that is not a number.
#[must_use]
pub fn robust_extent(mesh: &Mesh) -> Option<f64> {
    if mesh.positions.is_empty() || !mesh.positions.iter().all(surfacecraft_geom::finite) {
        return None;
    }
    let n = mesh.positions.len();
    let cut = cut_per_end(n);
    let mut axis: Vec<f64> = Vec::with_capacity(n);
    let mut largest = 0.0_f64;
    for k in 0..3 {
        axis.clear();
        axis.extend(mesh.positions.iter().map(|p| p[k]));
        let low = order_statistic(&mut axis, cut);
        let high = order_statistic(&mut axis, n - 1 - cut);
        largest = largest.max(high - low);
    }
    Some(largest)
}

/// Which problem a robust extent (mm) shows, if any.
fn problem_of(extent: f64) -> Option<ScaleProblem> {
    if extent < NOT_MILLIMETRES_BELOW_MM {
        Some(ScaleProblem::NotMillimetres)
    } else if extent < METRES_BELOW_MM {
        Some(ScaleProblem::MetresOrInches)
    } else if extent > MICROMETRES_ABOVE_MM {
        Some(ScaleProblem::Micrometres)
    } else {
        None
    }
}

/// The unit a file would have to be in for the size to be plausible, and its exact text.
fn unit_to_try(problem: ScaleProblem) -> &'static str {
    match problem {
        ScaleProblem::MetresOrInches | ScaleProblem::NotMillimetres => "m",
        ScaleProblem::Micrometres => "um",
    }
}

fn message_of(problem: ScaleProblem, extent: f64) -> String {
    match problem {
        ScaleProblem::MetresOrInches => {
            format!("the largest extent is {extent:.4} mm: this looks like metres (or inches), not millimetres")
        }
        ScaleProblem::NotMillimetres => {
            format!("the largest extent is {extent:.6} mm: definitely not millimetres")
        }
        ScaleProblem::Micrometres => {
            format!("the largest extent is {extent:.0} mm: this looks like micrometres or another unit")
        }
    }
}

fn suggestion_of(problem: ScaleProblem) -> String {
    let unit = unit_to_try(problem);
    let extra = match problem {
        ScaleProblem::MetresOrInches => " (\"in\" if the file is in inches)",
        ScaleProblem::NotMillimetres | ScaleProblem::Micrometres => "",
    };
    format!(
        "import it again with mesh.import and \"units\": \"{unit}\"{extra}, or convert this mesh with mesh.set_units and \"from\": \"{unit}\""
    )
}

/// The warning for `mesh`, when its size looks like a unit mistake.
#[must_use]
pub fn scale_warning(mesh: &Mesh) -> Option<ScaleWarning> {
    let extent = robust_extent(mesh)?;
    let problem = problem_of(extent)?;
    Some(ScaleWarning {
        code: SUSPICIOUS_SCALE.to_string(),
        problem,
        extent,
        message: message_of(problem, extent),
        suggestion: suggestion_of(problem),
    })
}

/// The warning as a finding of the quality report (a warning to clean before modelling).
#[must_use]
pub fn scale_finding(mesh: &Mesh) -> Option<Finding> {
    let w = scale_warning(mesh)?;
    Some(Finding {
        severity: Severity::Warning,
        code: w.code.clone(),
        message: format!("{}. {}", w.message, w.suggestion),
    })
}
