//! `surface.patch`: one trimmed bicubic B-spline patch over a closed outline of sides, with G0, G1 or
//! G2 to the neighbouring faces, interior points and curves, and scan data (design doc
//! `2026-11-13-surface-patch-design.md` §4.1).
//!
//! The command resolves what the caller gave (edges of bodies, polylines, a mesh region), chains
//! the sides into one closed loop (`sides`), finds the parameters of the outline and the data
//! (`data`: the flattening of the region when the sides are its boundary, else the driver projects on
//! the plane of the outline), and hands the request to `kernel::patch_fit`. The result is a surface
//! object (a sheet of one trimmed face) and the measured report; nothing is stitched into a body.
mod data;
mod sides;

use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, SurfaceObject};
use surfacecraft_kernel::KernelError;
use surfacecraft_kernel::patch_fit::{Conflicts, Continuity, Patch, Request, Side, fit_patch_cancellable};
use surfacecraft_kernel::tess::tessellate_all;
use surfacecraft_kernel::vpatch::{DEGREE, MAX_CONTROL, MIN_CONTROL};

const DEFAULT_TOLERANCE: f64 = 0.2;
const DEFAULT_MAX_CONTROL: usize = 16;
const DEFAULT_GAP_TOLERANCE: f64 = 0.01;
/// Most interior points of one call.
const MAX_POINTS: usize = 10_000;
/// Most scan points handed to the fit (the kernel's own limit).
const MAX_DATA: usize = 50_000;

/// Continuity to the neighbouring face.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ContinuityIn {
    /// Positional: the patch meets the side.
    G0,
    /// Tangent plane: also the same tangent plane along the side.
    G1,
    /// Curvature: also the same normal curvature across the side.
    G2,
}

impl ContinuityIn {
    fn kernel(self) -> Continuity {
        match self {
            ContinuityIn::G0 => Continuity::G0,
            ContinuityIn::G1 => Continuity::G1,
            ContinuityIn::G2 => Continuity::G2,
        }
    }

    fn from_kernel(c: Continuity) -> Self {
        match c {
            Continuity::G0 => ContinuityIn::G0,
            Continuity::G1 => ContinuityIn::G1,
            Continuity::G2 => ContinuityIn::G2,
        }
    }
}

/// What to do at a corner where the sides ask for different tangent planes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum ConflictsIn {
    /// Relax the tangent conditions near the corner and report it.
    Relax,
    /// Refuse.
    Fail,
}

/// A face of a body.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct FaceRefIn {
    /// Body or sheet (id or name).
    pub body: ObjRef,
    /// Index of the face in the body.
    pub face: usize,
}

/// One side of the outline: a body edge (`body` and `edge`) or a polyline (`points`).
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SideIn {
    /// Body or sheet (id or name) whose edge is the side.
    #[serde(default)]
    pub body: Option<ObjRef>,
    /// Index of the edge in the body.
    #[serde(default)]
    pub edge: Option<usize>,
    /// Polyline in space: at least 2 points `[x, y, z]`; 2 points are a segment.
    #[serde(default)]
    pub points: Option<Vec<[f64; 3]>>,
    /// Continuity to the face across the side: `g0` (default), `g1` or `g2`.
    #[serde(default)]
    pub continuity: Option<ContinuityIn>,
    /// The face the patch must continue (G1/G2); omitted for a body edge that has a single face.
    #[serde(default)]
    pub face: Option<FaceRefIn>,
    /// The side is approximate (traced on a scan, e.g. a feature curve): the patch follows the data and comes as close to the side as the data allow; its gap is reported, not a target (default false).
    #[serde(default)]
    pub soft: bool,
}

/// An interior point of the patch.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PointIn {
    /// Position `[x, y, z]` (mm).
    pub at: [f64; 3],
    /// Weight of a soft point against a scan point's 1 (default 1).
    #[serde(default)]
    pub weight: Option<f64>,
    /// The patch passes through the point exactly (default false).
    #[serde(default)]
    pub hard: Option<bool>,
}

/// Parameters of `surface.patch`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PatchIn {
    /// Sides of the closed outline in any order (each a body edge or a polyline): they are chained into one loop.
    pub sides: Vec<SideIn>,
    /// Interior points the patch passes through (hard) or near (weighted).
    #[serde(default)]
    pub points: Vec<PointIn>,
    /// Interior polylines the patch must contain (hard, G0), each `[[x, y, z], ...]`.
    #[serde(default)]
    pub curves: Vec<Vec<[f64; 3]>>,
    /// Region whose vertices are the scan data to fit (soft).
    #[serde(default)]
    pub region: Option<ObjRef>,
    /// Mesh (with `triangles`) when no region is given.
    #[serde(default)]
    pub mesh: Option<ObjRef>,
    /// Triangles of `mesh` (all when omitted with `mesh`).
    #[serde(default)]
    pub triangles: Option<Vec<u32>>,
    /// Target maximum deviation of the scan data from the patch (mm, default 0.2).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Relative stiffness of the bending energy against the data (default 0.001); higher is smoother.
    #[serde(default)]
    pub smoothing: Option<f64>,
    /// Start control net `[u, v]`, 4 to `max_control` per direction; omitted: from the number of data points.
    #[serde(default)]
    pub control: Option<[usize; 2]>,
    /// Most control points per direction (4 to 24, default 16).
    #[serde(default)]
    pub max_control: Option<usize>,
    /// Largest gap (mm, default 0.01) between the ends of two sides that is closed; a larger one is an error.
    #[serde(default)]
    pub gap_tolerance: Option<f64>,
    /// What to do where two sides ask for different tangent planes at their corner: `relax` (default) or `fail`.
    #[serde(default)]
    pub conflicts: Option<ConflictsIn>,
    /// Name.
    #[serde(default)]
    pub name: Option<String>,
}

/// How one side of the outline fared.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SideOut {
    /// Index of the side in the request's `sides` (the report lists the sides in loop order).
    pub side: usize,
    /// Continuity asked for (not what was achieved: see `met`).
    pub continuity: ContinuityIn,
    /// The side meets its targets: gap 1e-3 mm and, for g1/g2, angle 0.05 degrees, curvature 5 % and no fold. Always true for a soft side, which has none. A warning names the target that was missed.
    pub met: bool,
    /// Largest distance from the side to the patch, measured densely (mm). For a g1/g2 side the side's points are first moved onto the reference face (`snap_max_mm`), and the gap is measured against those points.
    pub gap_max_mm: f64,
    /// Largest angle between the neighbouring face's normal and the patch's along the side (degrees), G1/G2.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub angle_max_deg: Option<f64>,
    /// Largest relative error of the normal curvature against the neighbouring face, G2.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub curvature_error_max: Option<f64>,
    /// Samples measured along the side.
    pub samples: usize,
    /// The patch folds back over the neighbouring face.
    pub folded: bool,
    /// The side was soft: its gap is reported, not a target.
    pub soft: bool,
    /// Largest distance the side's points were moved onto the neighbouring face (mm), G1/G2.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub snap_max_mm: Option<f64>,
}

/// A corner whose sides ask for different tangent planes.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct CornerOut {
    /// Where.
    pub at: [f64; 3],
    /// Angle between the two tangent planes (degrees).
    pub mismatch_deg: f64,
    /// Length of the side stretches whose tangent conditions were relaxed (mm).
    pub relaxed_length_mm: f64,
}

/// Result of `surface.patch`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct PatchOut {
    /// The new surface object: a sheet of one trimmed face.
    pub surface: ObjectId,
    /// Control net of the patch.
    pub control: [usize; 2],
    /// Degree.
    pub degree: usize,
    /// Rounds of solving and refining the net.
    pub rounds: u32,
    /// RMS distance of the scan data from the patch (mm).
    pub rms: f64,
    /// 99th percentile of that distance (mm).
    pub p99: f64,
    /// Largest of that distance (mm).
    pub max: f64,
    /// Scan points measured.
    pub points: usize,
    /// Scan points that lie outside the outline and were ignored.
    pub outside: usize,
    /// The data's largest distance is within `tolerance`.
    pub within_tolerance: bool,
    /// Every hard side within its targets (gap 1e-3 mm, angle 0.05 degrees, curvature 5 %), interior points and curves on the patch, nothing folded, the hard conditions not contradicting each other, the data within `tolerance` (`within_tolerance`) and no corner relaxed.
    pub within_targets: bool,
    /// The patch folds: back over a neighbouring face along a side, or over itself inside the outline.
    pub folded: bool,
    /// The control net stopped growing because the solve of the next net exceeds the cost bound (about 8 s per solve); the result is that of the net reached (a warning says which).
    pub cost_limited: bool,
    /// Largest miss of a hard condition after the solve, as the control point motion that would satisfy it (mm): about 0 when consistent, the size of the contradiction otherwise.
    pub hard_residual_mm: f64,
    /// How (u, v) were found: `flattened` (the region's mesh) or `plane` (best-fit plane of the outline).
    pub parameters: String,
    /// One entry per side, in loop order.
    pub sides: Vec<SideOut>,
    /// Corners where the tangent conditions were relaxed.
    pub corners: Vec<CornerOut>,
    /// Largest distance of a weighted interior point (a soft `points` entry) from the patch (mm); absent without one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub soft_points_max_mm: Option<f64>,
    /// Hard conditions found dependent and dropped.
    pub dropped: usize,
    /// Things the caller should know.
    pub warnings: Vec<String>,
}

fn params(msg: impl Into<String>) -> CmdError {
    CmdError::Params(msg.into())
}

/// The numbers of the request, checked.
struct Settings {
    tolerance: f64,
    max_control: usize,
    gap_tolerance: f64,
}

fn settings(i: &PatchIn) -> CmdResult<Settings> {
    let tolerance = i.tolerance.unwrap_or(DEFAULT_TOLERANCE);
    if !(tolerance.is_finite() && tolerance > 0.0) {
        return Err(params("tolerance must be positive"));
    }
    let gap_tolerance = i.gap_tolerance.unwrap_or(DEFAULT_GAP_TOLERANCE);
    if !(gap_tolerance.is_finite() && gap_tolerance > 0.0 && gap_tolerance <= 1e3) {
        return Err(params("gap_tolerance must be positive, at most 1000 mm"));
    }
    // a start net larger than the default limit raises the limit with it
    let wanted = i.control.map_or(0, |c| c[0].max(c[1])).min(MAX_CONTROL);
    let max_control = i.max_control.unwrap_or_else(|| DEFAULT_MAX_CONTROL.max(wanted));
    if !(MIN_CONTROL..=MAX_CONTROL).contains(&max_control) {
        return Err(params(format!("max_control must be {MIN_CONTROL} to {MAX_CONTROL}")));
    }
    Ok(Settings { tolerance, max_control, gap_tolerance })
}

/// The hard and the soft interior points of the request.
fn interior_points(points: &[PointIn]) -> CmdResult<(Vec<[f64; 3]>, Vec<([f64; 3], f64)>)> {
    if points.len() > MAX_POINTS {
        return Err(params(format!("at most {MAX_POINTS} interior points are accepted, not {}", points.len())));
    }
    let (mut hard, mut soft) = (Vec::new(), Vec::new());
    for (k, p) in points.iter().enumerate() {
        if !p.at.iter().all(|c| c.is_finite()) {
            return Err(params(format!("point {k} is not finite")));
        }
        if p.hard.unwrap_or(false) {
            hard.push(p.at);
        } else {
            let w = p.weight.unwrap_or(1.0);
            if !(w.is_finite() && w > 0.0 && w <= 1e6) {
                return Err(params(format!("point {k}: weight must be positive, at most 1e6")));
            }
            soft.push((p.at, w));
        }
    }
    Ok((hard, soft))
}

/// Notes on what the request gave that has no effect: a `face` on a g0 side.
fn ignored_inputs(sides: &[SideIn]) -> Vec<String> {
    sides
        .iter()
        .enumerate()
        .filter(|(_, sd)| sd.face.is_some() && matches!(sd.continuity, None | Some(ContinuityIn::G0)))
        .map(|(k, _)| format!("side {k}: `face` is ignored for a g0 side (give continuity g1 or g2 to continue it)"))
        .collect()
}

/// The kernel's sides, in loop order: a body edge is densified, a polyline goes as given.
fn kernel_sides(outline: Vec<sides::Resolved>, got: &data::Gathered) -> Vec<Side> {
    outline
        .into_iter()
        .enumerate()
        .map(|(k, r)| {
            let given_uv = got.side_uv.as_ref().and_then(|all| all.get(k));
            let (points, uv) = if r.from_edge {
                sides::densify(&r.points, given_uv.map(Vec::as_slice))
            } else {
                (r.points, given_uv.cloned())
            };
            Side { points, continuity: r.continuity, reference: r.reference, uv, soft: r.soft }
        })
        .collect()
}

fn side_outs(report: &surfacecraft_kernel::patch_fit::Report, order: &[usize]) -> Vec<SideOut> {
    report
        .sides
        .iter()
        .zip(order)
        .map(|(r, &side)| SideOut {
            side,
            continuity: ContinuityIn::from_kernel(r.continuity),
            met: r.met,
            gap_max_mm: r.gap_max,
            angle_max_deg: r.angle_max_deg,
            curvature_error_max: r.curvature_error_max,
            samples: r.samples,
            folded: r.folded,
            soft: r.soft,
            snap_max_mm: r.snap_max,
        })
        .collect()
}

fn conflicts(c: Option<ConflictsIn>) -> Conflicts {
    match c.unwrap_or(ConflictsIn::Relax) {
        ConflictsIn::Relax => Conflicts::Relax,
        ConflictsIn::Fail => Conflicts::Fail,
    }
}

fn patch_cmd(s: &mut Session, i: PatchIn) -> CmdResult<PatchOut> {
    let st = settings(&i)?;
    let (hard_points, soft_points) = interior_points(&i.points)?;
    let resolved = sides::resolve(s, &i.sides)?;
    let (outline, mut warnings) = sides::chain(resolved, st.gap_tolerance)?;
    warnings.extend(ignored_inputs(&i.sides));
    let scan = data::resolve_scan(s, &i)?;
    let got = data::gather(scan.as_ref(), &outline, st.gap_tolerance, MAX_DATA, &mut warnings, &s.cancel)?;
    let order: Vec<usize> = outline.iter().map(|r| r.index).collect();
    let parameters = if got.side_uv.is_some() { "flattened" } else { "plane" };
    let request = Request {
        sides: kernel_sides(outline, &got),
        data: got.data,
        data_uv: got.data_uv,
        hard_points,
        soft_points,
        curves: i.curves,
        tolerance: st.tolerance,
        smoothing: i.smoothing,
        control: i.control,
        max_control: st.max_control,
        conflicts: conflicts(i.conflicts),
        side_ids: order.clone(),
    };
    let Patch { sheet, report, .. } = fit_patch_cancellable(&request, &s.cancel).map_err(|e| match e {
        KernelError::Cancelled => super::cancelled(),
        other => other.into(),
    })?;

    let id = add_patch(s, i.name, scan.as_ref().and_then(|sc| sc.region), sheet, &report)?;
    Ok(patch_out(id, report, parameters, &order, warnings))
}

/// Adds the sheet as a surface object.
fn add_patch(
    s: &mut Session,
    name: Option<String>,
    region: Option<ObjectId>,
    sheet: surfacecraft_kernel::record::brep::Brep,
    report: &surfacecraft_kernel::patch_fit::Report,
) -> CmdResult<ObjectId> {
    let display = tessellate_all(&sheet, None)?;
    Ok(s.doc.add_surface(SurfaceObject {
        id: ObjectId(0),
        name: name.unwrap_or_else(|| "patch".into()),
        sheet: Arc::new(sheet),
        region,
        control: report.control,
        degree: DEGREE,
        rms: report.rms,
        max: report.max,
        display: Arc::new(display),
        parent: None,
        visible: true,
    }))
}

/// The command's result: the kernel's report, the caller's side numbers, and the warnings of the
/// command followed by the kernel's.
fn patch_out(
    id: ObjectId,
    report: surfacecraft_kernel::patch_fit::Report,
    parameters: &str,
    order: &[usize],
    mut warnings: Vec<String>,
) -> PatchOut {
    let sides = side_outs(&report, order);
    warnings.extend(report.warnings);
    PatchOut {
        surface: id,
        control: report.control,
        degree: DEGREE,
        rounds: report.rounds,
        rms: report.rms,
        p99: report.p99,
        max: report.max,
        points: report.points,
        outside: report.outside,
        within_tolerance: report.within_tolerance,
        within_targets: report.within_targets,
        folded: report.folded,
        cost_limited: report.cost_limited,
        hard_residual_mm: report.hard_residual,
        parameters: parameters.into(),
        sides,
        corners: report
            .corners
            .iter()
            .map(|c| CornerOut { at: c.at, mismatch_deg: c.mismatch_deg, relaxed_length_mm: c.relaxed_length })
            .collect(),
        soft_points_max_mm: report.soft_points_max,
        dropped: report.dropped,
        warnings,
    }
}

/// The commands of this file.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "surface.patch",
    "Constrained patch",
    "Surface",
    true,
    "Fills the closed outline of `sides` (body edges or polylines) with one trimmed bicubic patch: G0/G1/G2 to the neighbouring faces, through `points` and `curves`, fitted to the scan of `region` or `mesh`.",
    PatchIn,
    PatchOut,
    patch_cmd
)];
