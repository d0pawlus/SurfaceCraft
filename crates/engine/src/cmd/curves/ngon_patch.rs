//! `surface.from_curves` with `ngon: "patch"`: every region that is not four-sided is fitted as
//! one trimmed patch by `kernel::patch_fit::fit_patch`. Its sides are the shared feature curves the
//! four-sided neighbours use (G0, no reference face), its data are the region's vertices.
use super::super::{CmdError, cancelled};
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, SurfaceObject};
use surfacecraft_fit::features::NgonRegion;
use surfacecraft_geom::Cancel;
use surfacecraft_kernel::KernelError;
use surfacecraft_kernel::patch_fit::{Conflicts, Continuity, Patch, Request, Side, fit_patch_cancellable};
use surfacecraft_kernel::tess::tessellate_all;
use surfacecraft_kernel::vpatch::DEGREE;

/// Most control points per direction of a trimmed patch.
const MAX_CONTROL: usize = 16;

/// One region that was fitted as a single trimmed patch.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct TrimmedPatchOut {
    /// The patch as a surface object (`<name>_patch_<k>`).
    pub surface: ObjectId,
    /// Corners of the region.
    pub corners: usize,
    /// Control net of the patch (points per direction).
    pub control: [usize; 2],
    /// Largest distance of the region's vertices from the patch (mm).
    pub max: f64,
    /// RMS distance of the region's vertices from the patch (mm).
    pub rms: f64,
    /// Largest distance of a shared feature curve from the patch (mm).
    pub side_gap_max_mm: f64,
    /// Is `max` within the target tolerance? Never for a patch that measured no vertex.
    pub within_tolerance: bool,
    /// Vertices of the region measured against the patch.
    pub points: usize,
    /// Vertices of the region that lay outside the patch's outline and were ignored.
    pub outside: usize,
    /// What the fit reported about this patch (corners, data, the control net).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub warnings: Vec<String>,
}

/// What fitting all n-sided regions produced.
#[derive(Default)]
pub(super) struct NgonFit {
    /// The fitted patches, in region order.
    pub patches: Vec<TrimmedPatchOut>,
    /// Regions that could not be fitted, with the reason.
    pub skipped: Vec<String>,
    /// Patches that measured no vertex of their region (their `within_tolerance` is false).
    pub unmeasured: Vec<String>,
    /// Sum of the squared deviations of the data of all patches.
    pub sq: f64,
    /// Largest deviation of the data of all patches.
    pub max: f64,
    /// Number of data points measured.
    pub count: usize,
}

/// Where the patches go in the document.
pub(super) struct Target<'a> {
    /// Name of the command's result; patch `k` is `<name>_patch_<k>`.
    pub name: &'a str,
    /// The region the scan came from.
    pub region: Option<ObjectId>,
    /// Target maximum deviation (mm).
    pub tolerance: f64,
}

fn request(rg: &NgonRegion, tolerance: f64) -> Request {
    Request {
        sides: rg
            .sides
            .iter()
            .map(|sd| Side {
                points: sd.points.clone(),
                continuity: Continuity::G0,
                reference: None,
                uv: sd.uv.clone(),
                // the sides are the feature curves traced on the scan: they follow the scan's rounded
                // creases, so the patch follows the data and reports its distance from them
                soft: true,
            })
            .collect(),
        data: rg.data.clone(),
        data_uv: rg.data_uv.clone(),
        hard_points: Vec::new(),
        soft_points: Vec::new(),
        curves: Vec::new(),
        tolerance,
        smoothing: None,
        control: None,
        max_control: MAX_CONTROL,
        conflicts: Conflicts::Relax,
        side_ids: Vec::new(),
    }
}

/// Fits `rg` and returns the patch with its display mesh.
fn fit_region(rg: &NgonRegion, tolerance: f64, cancel: &Cancel) -> Result<(Patch, surfacecraft_geom::Mesh), FitFail> {
    let patch = fit_patch_cancellable(&request(rg, tolerance), cancel).map_err(|e| match e {
        KernelError::Cancelled => FitFail::Cancelled,
        other => FitFail::Failed(other.to_string()),
    })?;
    let display = tessellate_all(&patch.sheet, None).map_err(|e| FitFail::Failed(e.to_string()))?;
    Ok((patch, display))
}

/// Why a region was not fitted: the call was cancelled (it ends), or the fit failed (the region is skipped).
enum FitFail {
    Cancelled,
    Failed(String),
}

/// Fits every region as one trimmed patch and adds each as its own surface object. A region that
/// cannot be fitted is listed in `skipped` with the error text; the others are still made.
pub(super) fn fit_regions(s: &mut Session, regions: &[NgonRegion], target: &Target<'_>) -> Result<NgonFit, CmdError> {
    fit_regions_with(s, regions, target, &fit_region)
}

/// How one region is fitted (a seam for the tests).
type Fitter<'a> = &'a dyn Fn(&NgonRegion, f64, &Cancel) -> Result<(Patch, surfacecraft_geom::Mesh), FitFail>;

/// [`fit_regions`] with the fitter given.
fn fit_regions_with(
    s: &mut Session,
    regions: &[NgonRegion],
    target: &Target<'_>,
    fitter: Fitter<'_>,
) -> Result<NgonFit, CmdError> {
    let mut out = NgonFit::default();
    for rg in regions {
        let (patch, display) = match fitter(rg, target.tolerance, &s.cancel) {
            Ok(x) => x,
            Err(FitFail::Cancelled) => return Err(cancelled()),
            Err(FitFail::Failed(e)) => {
                out.skipped.push(format!(
                    "region of {} triangles with {} corners: {e}",
                    rg.triangles.len(),
                    rg.corners.len()
                ));
                continue;
            }
        };
        let report = &patch.report;
        let side_gap_max_mm = report.sides.iter().map(|sd| sd.gap_max).fold(0.0_f64, f64::max);
        if report.points > 0 {
            out.sq += report.rms * report.rms * report.points as f64;
            out.max = out.max.max(report.max);
            out.count += report.points;
        }
        // a patch that measured nothing has not met the tolerance, whatever its (empty) maximum says
        let within_tolerance = report.within_tolerance && report.points > 0;
        if report.points == 0 {
            out.unmeasured.push(format!(
                "region of {} triangles with {} corners: no vertex was measured against its patch",
                rg.triangles.len(),
                rg.corners.len()
            ));
        }
        let (control, rms, max) = (report.control, report.rms, report.max);
        let (points, outside, warnings) = (report.points, report.outside, report.warnings.clone());
        let surface = s.doc.add_surface(SurfaceObject {
            id: ObjectId(0),
            name: format!("{}_patch_{}", target.name, out.patches.len()),
            sheet: Arc::new(patch.sheet),
            region: target.region,
            control,
            degree: DEGREE,
            rms,
            max,
            display: Arc::new(display),
            parent: None,
            visible: true,
        });
        out.patches.push(TrimmedPatchOut {
            surface,
            corners: rg.corners.len(),
            control,
            max,
            rms,
            side_gap_max_mm,
            within_tolerance,
            points,
            outside,
            warnings,
        });
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::Cell;
    use surfacecraft_fit::features::NgonSide;

    fn pentagon_region() -> NgonRegion {
        let corners: Vec<[f64; 3]> = (0..5)
            .map(|k| {
                let a = std::f64::consts::TAU * k as f64 / 5.0;
                [15.0 * a.cos(), 15.0 * a.sin(), 0.0]
            })
            .collect();
        let sides = (0..5).map(|k| NgonSide { points: vec![corners[k], corners[(k + 1) % 5]], uv: None }).collect();
        NgonRegion { corners: vec![0, 1, 2, 3, 4], sides, data: Vec::new(), data_uv: None, triangles: Vec::new() }
    }

    /// A cancellation after the first patch was added ends the call with the cancellation error; the
    /// patch stays in the session's document until the engine rolls the undoable command back.
    #[test]
    fn a_cancellation_after_the_first_patch_is_the_cancellation_error() {
        let regions = [pentagon_region(), pentagon_region()];
        let calls = Cell::new(0);
        let fitter = |rg: &NgonRegion, tol: f64, cancel: &Cancel| {
            calls.set(calls.get() + 1);
            if calls.get() == 2 {
                return Err(FitFail::Cancelled);
            }
            fit_region(rg, tol, cancel)
        };
        let mut s = Session::new();
        let target = Target { name: "n", region: None, tolerance: 0.2 };
        let r = fit_regions_with(&mut s, &regions, &target, &fitter);
        assert!(matches!(r, Err(ref e) if crate::cmd::is_cancelled(e)));
        assert_eq!(s.doc.surfaces.len(), 1, "the first patch was added before the stop");
    }

    #[test]
    fn from_curves_is_undoable_so_a_stopped_call_is_rolled_back() {
        let spec = crate::registry().iter().find(|c| c.id == "surface.from_curves").unwrap();
        assert!(spec.undoable);
    }
}
