//! The hybrid method of `scan.to_solid` (P9-O6b): the freeform wrap of the scan in which the patches
//! that lie on a primitive are merged into one analytic face each (plane, cylinder, cone, sphere).
//!
//! The primitives are fitted as for `body.from_regions`. A copy of the scan is prepared in which the
//! vertices of every protected region lie exactly on their primitive (`snap`), and wrapped exactly as
//! the freeform method wraps a scan. The groups of patches whose whole footprint lies deep inside a
//! region are then replaced, by editing the topology of the body, by one face on the primitive
//! (`pipeline_merge`, `kernel::merge`): nothing is cut, so the body is as closed, or as open, as the
//! wrap, and the band of patches along the border of a region stays patches. The body is measured by
//! the unchanged judge against the original scan, and is returned only when it is not worse than the
//! wrap it came from. A failure ends the attempt with an issue code and the document as it was.
use super::freeform::{judge_stage, wrap_mesh_at, wrap_stage};
use super::merging::{MIN_AREA_IN_TOLERANCES_SQUARED, Merging, Protected, merge_groups};
use super::regions::{candidates, fit_regions};
use super::{
    LeftOut, MAX_LISTED_LEFT_OUT, Method, Modelled, ScanSummary, Settings, ToSolidIn, ToSolidOut, Verdict, apply_roles,
    issue, message_of, region_limits, relate, step, triangle_count,
};
use crate::Session;
use crate::cmd::body::body_out;
use crate::cmd::hybrid::FaceOf;
use crate::cmd::wrap::WrapOut;
use crate::cmd::{CmdError, cancelled as cancelled_error, is_cancelled};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::cmp::Ordering;
use std::sync::Arc;
use surfacecraft_doc::ObjectId;
use surfacecraft_fit::PrimitiveKind;
use surfacecraft_geom::Mesh;
use surfacecraft_kernel::hybrid;
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::tess::tessellate_topo;

#[path = "pipeline_hybrid_snap.rs"]
mod snap;
use snap::{projected, shape_of};

/// The network is held to this share of the tolerance: the coarse mesh the patches are laid on may
/// leave the scan by a few of these, and along a sharp edge the patch boundary is a chord of it, which
/// is what a group's boundary gap to its primitive is made of.
const WRAP_TOLERANCE_SHARE: f64 = 0.5;
/// Vertices of a protected region within this share of the tolerance of their primitive are put on it.
/// Measured on scans 1 and 4: 0.5 gave 2 and 9 analytic faces (1.2 % and 7.5 % of the area), 1.0 gave 7
/// and 24 (6.4 % and 16.6 %), and the deviation from the scan stays within the tolerance either way.
const PROJECT_SHARE_OF_TOLERANCE: f64 = 1.0;
/// A merged face whose boundary is farther than this share of the tolerance from its primitive is
/// reported as the issue `hybrid_edge_gap`.
const EDGE_GAP_NOTE_SHARE: f64 = 0.5;
/// Chord of the tessellation the area of the faces is measured on, as a share of the tolerance.
const AREA_CHORD_SHARE: f64 = 1.0;

/// A face of the body that merged patches of one primitive.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct MergedFace {
    /// The primitive.
    pub primitive: ObjectId,
    /// Its kind (plane, cylinder, cone, sphere).
    pub kind: String,
    /// The face of the body.
    pub face: usize,
    /// Patches the face replaced.
    pub patches: usize,
    /// The largest distance of the face's boundary edges from the primitive (mm).
    pub max_edge_gap: f64,
    /// The area of the face (mm2), from a tessellation at the tolerance's chord.
    #[serde(default)]
    pub area: f64,
}

/// Patches of a protected region that stay patches.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct LeftPatches {
    /// The primitive.
    pub primitive: ObjectId,
    /// Its kind.
    pub kind: String,
    /// Patches in the group.
    pub patches: usize,
    /// Why they were not merged.
    pub reason: String,
}

/// How much of the body is exact geometry.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct AnalyticSummary {
    /// Faces on a plane, cylinder, cone, sphere or torus.
    pub faces: usize,
    /// All faces of the body.
    pub total_faces: usize,
    /// Share of the body's surface area on those faces (0..1).
    pub area_share: f64,
    /// The merged faces, each with the largest distance of its boundary from its primitive.
    #[serde(default)]
    pub merged: Vec<MergedFace>,
    /// The largest distance of the boundary edges of any merged face from its primitive (mm), sampled
    /// at 25 points per edge (not a certified maximum). 0 when nothing was merged.
    #[serde(default)]
    pub max_edge_gap: f64,
    /// Groups of patches of a protected region that were not merged, with the reason.
    #[serde(default)]
    pub left_as_patches: Vec<LeftPatches>,
}

/// Why an attempt stopped.
enum Halt {
    Cancelled(CmdError),
    Failed { code: &'static str, message: String },
}

type Step<T> = Result<T, Halt>;

fn halt(code: &'static str, message: impl Into<String>) -> Halt {
    Halt::Failed { code, message: message.into() }
}

/// The halt for a command's error: a cancellation stays one, anything else is `code`.
fn failed_at(code: &'static str) -> impl Fn(CmdError) -> Halt {
    move |e| if is_cancelled(&e) { Halt::Cancelled(e) } else { halt(code, message_of(&e)) }
}

fn check_cancel(s: &Session) -> Step<()> {
    if s.cancel.is_cancelled() { Err(Halt::Cancelled(cancelled_error())) } else { Ok(()) }
}

/// The primitives fitted to the regions of the scan.
struct Fitted {
    modelled: Vec<Modelled>,
    left: Vec<LeftOut>,
    unassigned: usize,
    relations: usize,
}

fn fit_primitives(s: &mut Session, i: &ToSolidIn, set: &Settings, report: &ScanSummary) -> Step<Fitted> {
    let (list, unassigned) = candidates(s, set.scan);
    let total = triangle_count(s, set.scan);
    let (min_triangles, rms_limit) = region_limits(set, total, report);
    let (modelled, left) = fit_regions(s, &list, min_triangles, rms_limit);
    let relations = if i.constrain { relate(s, &modelled, report.noise_sigma) } else { 0 };
    Ok(Fitted { modelled, left, unassigned, relations })
}

/// The protected primitives, largest region first (the order the regions are given): every fitted
/// primitive the merge can place. Regions of a torus are not protected.
fn protect(s: &Session, modelled: &[Modelled], scan: &Mesh) -> Vec<Protected> {
    let mut out = Vec::new();
    for m in modelled {
        let Some(o) = s.doc.primitive(m.primitive) else { continue };
        if m.kind == PrimitiveKind::Torus {
            continue;
        }
        let Some(region) = o.region.and_then(|r| s.doc.region(r)) else { continue };
        let kind = match m.kind {
            PrimitiveKind::Plane => "plane",
            PrimitiveKind::Cylinder => "cylinder",
            PrimitiveKind::Cone => "cone",
            _ => "sphere",
        };
        let fit = o.primitive.clone();
        out.push(Protected {
            primitive: m.primitive,
            kind,
            shape: shape_of(&fit),
            fit,
            triangles: region.triangles.to_vec(),
            area: region
                .triangles
                .iter()
                .filter_map(|&t| scan.triangle(t as usize))
                .map(|[a, b, c]| 0.5 * (b - a).cross(&(c - a)).norm())
                .sum(),
        });
    }
    out
}

/// Wraps the copy of the scan whose protected regions lie on their primitives (the copy is removed
/// again), or the scan itself when no primitive is protected: then the result is the freeform
/// method's. A wrap that fails ends the attempt.
fn wrap_prepared(s: &mut Session, set: &Settings, protected: &[Protected], scan: &Mesh) -> Step<(WrapOut, Retry)> {
    let staged = if protected.is_empty() {
        wrap_stage(s, set, set.scan, |s, cap| wrap_mesh_at(s, set, set.scan, cap, set.tolerance))
    } else {
        let regions: Vec<(&[u32], &surfacecraft_fit::Primitive)> =
            protected.iter().map(|p| (p.triangles.as_slice(), &p.fit)).collect();
        let copy = s.doc.add_mesh(
            &format!("{}-prepared", set.name),
            projected(scan, &regions, PROJECT_SHARE_OF_TOLERANCE * set.tolerance),
        );
        let held = WRAP_TOLERANCE_SHARE * set.tolerance;
        let staged = wrap_stage(s, set, copy, |s, cap| wrap_mesh_at(s, set, copy, cap, held));
        s.doc.remove(copy);
        staged
    };
    match staged.map_err(failed_at("hybrid_wrap_failed"))? {
        Ok(pair) => Ok(pair),
        Err(why) => Err(halt("hybrid_wrap_failed", format!("the scan could not be wrapped in patches: {why}"))),
    }
}

type Retry = Option<super::freeform::CapRetry>;

/// Analytic faces and their share of the surface area, from a tessellation that knows its faces.
fn analytic_summary(brep: &Brep, tolerance: f64, m: &Merging) -> Option<AnalyticSummary> {
    let analytic = hybrid::analytic_faces(brep);
    let topo = tessellate_topo(brep, Some(AREA_CHORD_SHARE * tolerance)).ok()?;
    if topo.tri_face.len() != topo.mesh.triangle_count() {
        return None;
    }
    let (mut on, mut all) = (0.0, 0.0);
    let mut per_face = vec![0.0; brep.faces.len()];
    for (t, &face) in topo.tri_face.iter().enumerate() {
        let Some([a, b, c]) = topo.mesh.triangle(t) else { continue };
        let area = 0.5 * (b - a).cross(&(c - a)).norm();
        all += area;
        if let Some(slot) = per_face.get_mut(face as usize) {
            *slot += area;
        }
        if analytic.binary_search(&(face as usize)).is_ok() {
            on += area;
        }
    }
    let share = if all > 0.0 { (on / all).clamp(0.0, 1.0) } else { 0.0 };
    Some(AnalyticSummary {
        faces: analytic.len(),
        total_faces: brep.faces.len(),
        area_share: share,
        max_edge_gap: m.merged.iter().map(|g| g.max_edge_gap).fold(0.0, f64::max),
        merged: m
            .merged
            .iter()
            .map(|g| MergedFace { area: per_face.get(g.face).copied().unwrap_or(0.0), ..g.clone() })
            .collect(),
        left_as_patches: m.left.clone(),
    })
}

/// Per fitted primitive: its merged faces (or none) for the result's `modelled` list.
fn faces_of(modelled: &[Modelled], merged: &[MergedFace]) -> Vec<FaceOf> {
    modelled
        .iter()
        .map(|p| {
            let mine: Vec<&MergedFace> = merged.iter().filter(|g| g.primitive == p.primitive).collect();
            let faces = mine.iter().map(|g| g.face).collect();
            let patches: usize = mine.iter().map(|g| g.patches).sum();
            let (how, role) = if mine.is_empty() {
                ("stays patches".to_string(), "patches")
            } else {
                (format!("{patches} patches merged into {} face(s)", mine.len()), "merged")
            };
            FaceOf { object: p.primitive, faces, how, role: role.to_string() }
        })
        .collect()
}

/// Puts the merged body in the wrap's document body and refreshes what the result reports of it.
fn install(s: &mut Session, w: &WrapOut, body: Brep) -> Step<WrapOut> {
    let slot = s.doc.body_mut(w.body.id).ok_or_else(|| halt("hybrid_failed", "the wrapped body is gone"))?;
    slot.brep = Arc::new(body);
    let info = body_out(s, w.body.id).map_err(failed_at("hybrid_failed"))?;
    Ok(WrapOut { body: info, ..w.clone() })
}

/// The band the merged body's largest and RMS deviation from the scan may exceed the wrap's by, as
/// shares of the tolerance, before it is refused (the comparison of `auto` alone lets a share within
/// the tolerance drop by half a point). A merged face's boundary may be a tolerance from its patches
/// (the gap limit), so the largest deviation may grow by that near it (scan 4: 0.54 to 0.67 mm at a
/// tolerance of 0.2); the RMS, which a wrong face moves, may grow by 0.02 of the tolerance.
const MAX_DEVIATION_BAND: f64 = 1.0;
const RMS_DEVIATION_BAND: f64 = 0.02;

/// Why `merged` is not an upgrade of the wrap it came from, if it is not: worse by the comparison of
/// `auto`, or a larger maximum or RMS deviation from the scan than the wrap's by more than a band.
pub(super) fn rejection(merged: &ToSolidOut, wrap: &ToSolidOut, tol: f64) -> Option<String> {
    if super::auto::compare(merged, wrap) == Ordering::Less {
        let show = |r: &ToSolidOut| {
            let d = r.deviation.as_ref();
            format!(
                "{:?}, {:.2} % within of {:.2} % measured, {}",
                r.verdict,
                100.0 * d.map_or(0.0, |d| d.within),
                100.0 * d.map_or(0.0, |d| d.coverage),
                if r.body.as_ref().is_some_and(|b| b.volume.is_some()) { "closed" } else { "a sheet" }
            )
        };
        return Some(format!(
            "the merged body is worse than the wrap by verdict, share within the tolerance or closedness (merged: {}; wrap: {})",
            show(merged),
            show(wrap)
        ));
    }
    let (m, w) = (merged.deviation.as_ref()?, wrap.deviation.as_ref()?);
    let worse = |a: f64, b: f64, band: f64| a > b + band * tol;
    if worse(m.max_abs, w.max_abs, MAX_DEVIATION_BAND) || worse(m.rms, w.rms, RMS_DEVIATION_BAND) {
        return Some(format!(
            "the merged body deviates more from the scan (max {:.3} against {:.3} mm, RMS {:.3} against {:.3} mm)",
            m.max_abs, w.max_abs, m.rms, w.rms
        ));
    }
    None
}

/// The merging step on the wrapped body: the merged result when it is not worse than the wrap (by the
/// comparison of `auto`), else the wrap as it was. Returns the result, the body's merging and the
/// analytic summary.
fn merged_or_wrap(
    s: &mut Session,
    set: &Settings,
    report: &ScanSummary,
    wrapped: (&WrapOut, Retry),
    mut wrap_result: ToSolidOut,
    protected: &[Protected],
    scan: &Mesh,
) -> Step<(ToSolidOut, Merging)> {
    let (w, retry) = wrapped;
    let brep =
        s.doc.body(w.body.id).map(|b| b.brep.clone()).ok_or_else(|| halt("hybrid_failed", "the body is gone"))?;
    let none = |left: Vec<LeftPatches>| Merging { body: (*brep).clone(), merged: Vec::new(), left };
    if protected.is_empty() {
        return Ok((wrap_result, none(Vec::new())));
    }
    let tried = merge_groups(s, scan, &brep, protected, set.tolerance)
        .map_err(|e| if is_cancelled(&e) { Halt::Cancelled(e) } else { halt("hybrid_merge_failed", message_of(&e)) })?;
    if tried.merged.is_empty() {
        let left = tried.left;
        return Ok((wrap_result, none(left)));
    }
    // a merged body the mesher cannot take at the display chord or the tolerance's is not returned
    let meshes = |c| tessellate_topo(&tried.body, c).is_ok();
    if !(meshes(None) && meshes(Some(AREA_CHORD_SHARE * set.tolerance))) {
        return Ok((wrap_result, none(tried.left)));
    }
    let merged_w = install(s, w, tried.body.clone())?;
    let merged = judge_stage(s, set, report.clone(), &merged_w, retry).map_err(failed_at("hybrid_failed"))?;
    if let Some(why) = rejection(&merged, &wrap_result, set.tolerance) {
        install(s, w, (*brep).clone())?;
        wrap_result.issues.push(issue("hybrid_merge_rejected", format!("the merge was not kept: {why}")));
        return Ok((wrap_result, none(tried.left)));
    }
    Ok((merged, tried))
}

/// Fills in what the fits of the call contribute to the result.
fn conclude(mut out: ToSolidOut, mut fitted: Fitted, merged: &[MergedFace], analytic: AnalyticSummary) -> ToSolidOut {
    let roles = faces_of(&fitted.modelled, merged);
    apply_roles(&mut fitted.modelled, &roles);
    out.method = Method::Hybrid;
    out.analytic = Some(analytic);
    out.relations = fitted.relations;
    out.left_out_regions = fitted.left.len();
    out.left_out_triangles = fitted.left.iter().map(|l| l.triangles).sum();
    out.left_out = fitted.left.into_iter().take(MAX_LISTED_LEFT_OUT).collect();
    out.unassigned_triangles = fitted.unassigned;
    out.modelled = fitted.modelled;
    out
}

/// No protected region can give a group: an explicit `method: hybrid` returns the freeform result
/// (no second wrap is made) with an issue saying why; under `auto` the attempt is a recorded failure,
/// since `auto` has the freeform result.
fn without_merge(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    report: &ScanSummary,
    fitted: Fitted,
    why: &str,
) -> Step<ToSolidOut> {
    if i.method != Method::Hybrid {
        return Err(halt("hybrid_no_analytic_face", why));
    }
    let mut out = super::freeform::freeform_solid(s, set, report.clone()).map_err(failed_at("hybrid_failed"))?;
    out.issues.push(issue("hybrid_no_analytic_face", format!("no analytic face is possible: {why}")));
    let faces = out.body.as_ref().map_or(0, |b| b.faces);
    let analytic = AnalyticSummary {
        faces: 0,
        total_faces: faces,
        area_share: 0.0,
        max_edge_gap: 0.0,
        merged: Vec::new(),
        left_as_patches: Vec::new(),
    };
    Ok(conclude(out, fitted, &[], analytic))
}

/// The hybrid attempt without the clean-up of a failure.
fn build(s: &mut Session, i: &ToSolidIn, set: &Settings, report: &ScanSummary) -> Step<ToSolidOut> {
    let fitted = fit_primitives(s, i, set, report)?;
    check_cancel(s)?;
    let scan = s.doc.mesh(set.scan).map(|m| m.mesh.clone()).ok_or_else(|| halt("hybrid_failed", "the scan is gone"))?;
    let protected = protect(s, &fitted.modelled, &scan);
    let least = MIN_AREA_IN_TOLERANCES_SQUARED * set.tolerance * set.tolerance;
    if !protected.iter().any(|p| p.area >= least) {
        let why = format!("no region a primitive fits (plane, cylinder, cone, sphere) has {least:.1} mm2 or more");
        return without_merge(s, i, set, report, fitted, &why);
    }
    let (w, retry) = wrap_prepared(s, set, &protected, &scan)?;
    check_cancel(s)?;
    let wrap_result = judge_stage(s, set, report.clone(), &w, retry.clone()).map_err(failed_at("hybrid_failed"))?;
    let (mut out, merging) = merged_or_wrap(s, set, report, (&w, retry), wrap_result, &protected, &scan)?;
    let body =
        s.doc.body(w.body.id).map(|b| b.brep.clone()).ok_or_else(|| halt("hybrid_failed", "the body is gone"))?;
    let analytic = analytic_summary(&body, set.tolerance, &merging)
        .ok_or_else(|| halt("hybrid_failed", "the faces could not be measured"))?;
    if analytic.max_edge_gap > EDGE_GAP_NOTE_SHARE * set.tolerance {
        let msg = format!(
            "a merged face's boundary is up to {:.3} mm from its primitive (sampled, not certified); its edges carry that as their tolerance, which STEP keeps",
            analytic.max_edge_gap
        );
        out.issues.push(issue("hybrid_edge_gap", msg));
    }
    Ok(conclude(out, fitted, &merging.merged, analytic))
}

/// The result of an attempt that ended in `code`: no body, and nothing in the document.
fn failure(set: &Settings, report: ScanSummary, code: &str, message: &str) -> ToSolidOut {
    let why = "what each verdict and issue means and how to go on by hand";
    let next = vec![
        step(
            "scan.to_solid",
            json!({"scan": set.scan, "method": "freeform", "tolerance": set.tolerance}),
            "wraps the whole scan in patches instead",
        ),
        step("guide.read", json!({"topic": "scan-to-solid"}), why),
    ];
    let issues = vec![issue(code, format!("no hybrid body was built: {message}"))];
    let mut out = super::freeform::out(set, None, report, Verdict::Fail, None, None, issues, next);
    out.method = Method::Hybrid;
    out
}

/// The hybrid method: the regions of the scan (segmented by the caller) become primitives where they
/// fit, the patches that lie on them one analytic face each. A failure other than a cancellation is a
/// result with an issue code and the document as it was.
pub(super) fn hybrid_solid(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    report: ScanSummary,
) -> Result<ToSolidOut, CmdError> {
    let before = s.doc.clone();
    match build(s, i, set, &report) {
        Ok(out) => Ok(out),
        Err(Halt::Cancelled(e)) => {
            s.doc = before;
            Err(e)
        }
        Err(Halt::Failed { code, message }) => {
            s.doc = before;
            Ok(failure(set, report, code, &message))
        }
    }
}
