//! The size check: is a scan plausibly a mechanical part in millimetres? A scan file has no unit,
//! so a file in metres or inches read as millimetres is far too small, and one in micrometres is
//! far too large. The check only reports it; it never rescales anything, and a unit the caller
//! stated (or a size set from a known length) is taken as it is.
//!
//! The rule. The robust extent is the largest of the three axis extents of the vertices. A mesh of
//! at least [`MIN_TRIMMED_VERTICES`] vertices has its most extreme vertices cut at each end of every
//! axis (one per hundred, at least one), so that a far-off vertex does not decide it; a smaller mesh
//! is measured as it is, so a small part keeps its apex. A larger mesh is measured on at most
//! [`SAMPLE_VERTICES`] vertices, taken evenly through the vertex list, so the work and the memory do
//! not grow with the size of the file. Below 1.0 mm the size looks like metres (or inches). Below
//! 0.02 mm it is definitely not millimetres. Above 20 000 mm it looks like micrometres or another
//! unit. Anything in between is fine and gives no warning.
use crate::cmd::mesh::units::left_alone;
use crate::cmd::qa::{Finding, Severity};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{Document, ObjectId};
use surfacecraft_geom::{Mesh, Point3};

/// The code of the warning, in `mesh.import`, `mesh.info`, `scan.qa`, `scan.to_solid` and stores.
pub(crate) const SUSPICIOUS_SCALE: &str = "suspicious_scale";
/// Share of the measured vertices cut from each end of an axis.
const TRIM: f64 = 0.01;
/// Meshes with fewer vertices are measured as they are (cutting would drop the corners of a small
/// part).
const MIN_TRIMMED_VERTICES: usize = 100;
/// Larger meshes are measured on this many vertices, taken evenly through the vertex list.
pub(crate) const SAMPLE_VERTICES: usize = 100_000;
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
    /// Calls that fix it, and that work for this mesh.
    pub suggestion: String,
}

/// The vertices the extent is measured on: all of them up to [`SAMPLE_VERTICES`], else that many
/// taken evenly through the list.
fn sampled(mesh: &Mesh) -> Vec<Point3> {
    let n = mesh.positions.len();
    let count = n.min(SAMPLE_VERTICES);
    (0..count).map(|i| mesh.positions[i * n / count]).collect()
}

/// How many of the most extreme measured vertices are cut at each end of an axis.
fn cut_per_end(measured: usize) -> usize {
    if measured < MIN_TRIMMED_VERTICES { 0 } else { ((measured as f64 * TRIM).floor() as usize).max(1) }
}

/// The value of rank `rank` (0 = smallest) of `values`, which it reorders.
fn order_statistic(values: &mut [f64], rank: usize) -> f64 {
    let (_, value, _) = values.select_nth_unstable_by(rank, f64::total_cmp);
    *value
}

/// The largest of the (trimmed) axis extents of the vertices (mm); `None` for a mesh without
/// vertices or with a coordinate that is not a number.
#[must_use]
pub fn robust_extent(mesh: &Mesh) -> Option<f64> {
    if mesh.positions.is_empty() || !mesh.positions.iter().all(surfacecraft_geom::finite) {
        return None;
    }
    let sample = sampled(mesh);
    let m = sample.len();
    let cut = cut_per_end(m);
    let mut axis: Vec<f64> = Vec::with_capacity(m);
    let mut largest = 0.0_f64;
    for k in 0..3 {
        axis.clear();
        axis.extend(sample.iter().map(|p| p[k]));
        let low = order_statistic(&mut axis, cut);
        let high = order_statistic(&mut axis, m - 1 - cut);
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

/// The fixes that work for the mesh: a re-import, or `mesh.set_units` (which needs `force` while
/// bodies or sketches made from the document stay at the old size).
fn suggestion_of(problem: ScaleProblem, derived: bool) -> String {
    let unit = unit_to_try(problem);
    let inches = match problem {
        ScaleProblem::MetresOrInches => " (\"in\" if the file is in inches)",
        ScaleProblem::NotMillimetres | ScaleProblem::Micrometres => "",
    };
    let force = if derived { ", \"force\": true (the bodies and sketches keep their old size)" } else { "" };
    format!(
        "import it again with mesh.import and \"units\": \"{unit}\"{inches}, or convert this mesh with mesh.set_units and \"from\": \"{unit}\"{force}"
    )
}

/// The warning for a robust extent (mm), when it looks like a unit mistake. `derived`: the document
/// holds bodies or sketches that a unit change would leave at the old size.
#[must_use]
pub(crate) fn warning_for_extent(extent: f64, derived: bool) -> Option<ScaleWarning> {
    let problem = problem_of(extent)?;
    Some(ScaleWarning {
        code: SUSPICIOUS_SCALE.to_string(),
        problem,
        extent,
        message: message_of(problem, extent),
        suggestion: suggestion_of(problem, derived),
    })
}

/// The warning of mesh `id`: none when its unit was stated or its size set from a length.
#[must_use]
pub fn warning_of(doc: &Document, id: ObjectId) -> Option<ScaleWarning> {
    let obj = doc.mesh(id)?;
    if obj.units_stated || obj.rescaled {
        return None;
    }
    let derived = !left_alone(doc).is_empty();
    warning_for_extent(robust_extent(&obj.mesh)?, derived)
}

/// The warning as a finding of the quality report (a warning to clean before modelling).
#[must_use]
pub fn scale_finding(doc: &Document, id: ObjectId) -> Option<Finding> {
    let w = warning_of(doc, id)?;
    Some(Finding {
        severity: Severity::Warning,
        code: w.code.clone(),
        message: format!("{}. {}", w.message, w.suggestion),
    })
}

/// Keeps the sample and the rule in one place for the tests of the module.
#[cfg(test)]
pub(crate) fn sample_len(mesh: &Mesh) -> usize {
    sampled(mesh).len()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mesh_of(points: Vec<[f64; 3]>) -> Mesh {
        Mesh { positions: points.into_iter().map(Point3::from).collect(), triangles: Vec::new() }
    }

    #[test]
    fn a_pin_of_one_point_two_millimetres_keeps_its_apex_and_is_not_read_as_metres() {
        // A small mesh: its apex is a real corner, not an outlier.
        let pin = mesh_of(vec![[0.0, 0.0, 0.0], [0.6, 0.0, 0.0], [-0.6, 0.0, 0.0], [0.0, 0.0, 1.2]]);
        assert!(warning_of_extent_for(&pin).is_none(), "1.2 mm is plausible");
    }

    #[test]
    fn a_large_mesh_is_measured_on_a_bounded_sample() {
        let big = mesh_of((0..2_000_000).map(|i| [i as f64 * 1e-5, 0.0, 0.0]).collect());
        assert_eq!(sample_len(&big), SAMPLE_VERTICES);
        let ext = robust_extent(&big).unwrap();
        assert!((ext - 20.0).abs() < 0.5, "the extent of the sample: {ext}");
    }

    fn warning_of_extent_for(mesh: &Mesh) -> Option<ScaleWarning> {
        warning_for_extent(robust_extent(mesh)?, false)
    }
}
