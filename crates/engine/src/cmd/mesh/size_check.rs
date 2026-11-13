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
use surfacecraft_doc::{Document, LengthUnit, ObjectId};
use surfacecraft_geom::{Aabb, Mesh, Point3};

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
/// Extents at or below this (mm) are a degenerate mesh (a point, or coincident vertices): no size to judge.
const DEGENERATE_EXTENT_MM: f64 = 1e-9;
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
    /// A unit was stated, and the size it gives is implausible, while read as millimetres it is
    /// plausible: the file may already be in millimetres.
    StatedUnitImplausible,
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
    (0..count).map(|k| mesh.positions[sample_index(k, n, count)]).collect()
}

/// Index of the `k`-th of `count` samples taken evenly through `n` vertices (`k < count <= n`). The
/// product is computed in 128 bits, so it cannot overflow on any target, wasm32 included.
pub(crate) fn sample_index(k: usize, n: usize, count: usize) -> usize {
    let index = (k as u128 * n as u128) / count.max(1) as u128;
    usize::try_from(index).unwrap_or(n.saturating_sub(1))
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
        ScaleProblem::Micrometres | ScaleProblem::StatedUnitImplausible => "um",
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
        ScaleProblem::StatedUnitImplausible => format!(
            "the largest extent is {extent:.0} mm after the stated unit, but read as millimetres it would be plausible: the file may already be in millimetres"
        ),
    }
}

/// The fixes that work for the mesh, and always the millimetre confirmation: a re-import, or
/// `mesh.set_units` (which needs `force` while bodies, sketches or features stay at the old size).
fn suggestion_of(problem: ScaleProblem, derived: bool) -> String {
    let unit = match problem {
        ScaleProblem::StatedUnitImplausible => "mm",
        other => unit_to_try(other),
    };
    let inches = match problem {
        ScaleProblem::MetresOrInches => " (\"in\" if the file is in inches)",
        _ => "",
    };
    let force = if derived { ", \"force\": true (the bodies and sketches keep their old size)" } else { "" };
    let fix = match problem {
        ScaleProblem::StatedUnitImplausible => "import it again without \"units\"".to_string(),
        _ => format!("import it again with mesh.import and \"units\": \"{unit}\"{inches}"),
    };
    format!(
        "{fix}, or convert this mesh with mesh.set_units and \"from\": \"{unit}\"{force}, or confirm millimetres with mesh.set_units from mm"
    )
}

/// The warning for a robust extent (mm), when it looks like a unit mistake. `derived`: the document
/// holds bodies or sketches that a unit change would leave at the old size.
#[must_use]
pub(crate) fn warning_for_extent(extent: f64, derived: bool) -> Option<ScaleWarning> {
    if !(extent > DEGENERATE_EXTENT_MM) {
        return None;
    }
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
    if obj.rescaled {
        return None;
    }
    let derived = !left_alone(doc).is_empty();
    let extent = robust_extent(&obj.mesh)?;
    if obj.units_stated {
        return stated_warning(extent, obj.source_units, derived);
    }
    warning_for_extent(extent, derived)
}

/// The warning of a mesh whose unit was stated: when the size after the stated conversion is
/// implausible but the same numbers read as millimetres are plausible, the file may already be in
/// millimetres. A stated millimetre unit is taken as it is.
fn stated_warning(extent: f64, source: LengthUnit, derived: bool) -> Option<ScaleWarning> {
    if source.is_mm() || !(extent > DEGENERATE_EXTENT_MM) || problem_of(extent).is_none() {
        return None;
    }
    if problem_of(extent / source.mm_per_unit()).is_some() {
        return None;
    }
    let problem = ScaleProblem::StatedUnitImplausible;
    Some(ScaleWarning {
        code: SUSPICIOUS_SCALE.to_string(),
        problem,
        extent,
        message: message_of(problem, extent),
        suggestion: suggestion_of(problem, derived),
    })
}

/// The robust extent (mm) of boxes, such as the chunks of a store: the same trimming as for the
/// vertices, applied to the box faces. `None` when there are no boxes.
#[must_use]
pub(crate) fn box_extent(boxes: &[Aabb]) -> Option<f64> {
    let boxes: Vec<&Aabb> = boxes.iter().filter(|b| !b.is_empty()).collect();
    let n = boxes.len();
    if n == 0 {
        return None;
    }
    let cut = cut_per_end(n);
    let mut values: Vec<f64> = Vec::with_capacity(n);
    let mut largest = 0.0_f64;
    for k in 0..3 {
        values.clear();
        values.extend(boxes.iter().map(|b| b.min[k]));
        let low = order_statistic(&mut values, cut);
        values.clear();
        values.extend(boxes.iter().map(|b| b.max[k]));
        let high = order_statistic(&mut values, n - 1 - cut);
        largest = largest.max(high - low);
    }
    largest.is_finite().then_some(largest)
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
    fn sample_index_stays_inside_the_mesh_for_huge_vertex_counts() {
        // above u32::MAX vertices, where a 32-bit product would overflow
        let n = u32::MAX as usize + 7;
        for k in [0, 1, SAMPLE_VERTICES / 2, SAMPLE_VERTICES - 1] {
            let i = sample_index(k, n, SAMPLE_VERTICES);
            assert!(i < n, "index {i} outside {n} vertices");
            assert_eq!(i as u128, k as u128 * n as u128 / SAMPLE_VERTICES as u128);
        }
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
