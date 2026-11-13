//! The freeform method of `scan.to_solid`: the whole scan wrapped in a patch network
//! (`surface.wrap`) and measured against the scan, for parts no set of primitives describes.
use super::{
    DeviationSummary, MAX_FILL_EDGES, MIN_COVERAGE, Method, NextStep, ScanSummary, Settings, ToSolidOut, Verdict,
    coverage_issue, issue, message_of, step, summary,
};
use crate::Session;
use crate::cmd::qa::{CompareIn, compare};
use crate::cmd::wrap::{PATCH_LIMIT, WrapIn, WrapOut, robust_diagonal, scale_warning, wrap};
use crate::cmd::{CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use surfacecraft_doc::ObjectId;

/// The thickness the next step offers for an open scan (a placeholder the reason explains).
const PLACEHOLDER_THICKNESS_MM: f64 = 2.0;
/// Smallest `max_patches` (the one of `surface.wrap`).
pub(super) const MIN_PATCHES: usize = 12;
/// The first attempt must have put at least this share of the scan within the tolerance for a retry:
/// below it a larger limit is unlikely to reach a pass (measured on real scans: 30 % at 12 000
/// patches became 59 % at 30 000 and 77 % at 60 000).
const RETRY_MIN_WITHIN: f64 = 0.25;
/// Scans with more triangles are not wrapped a second time (the reduction of the scan is the cost).
const RETRY_MAX_TRIANGLES: usize = 1_500_000;
/// The limit of the second attempt is this many times the first, at most `PATCH_LIMIT`.
const RETRY_FACTOR: usize = 5;
/// The second attempt must put this much more of the scan (a share, 0.5 percentage points) within
/// the tolerance to replace the first: a smaller difference is rounding noise of the measurement.
const RETRY_MIN_GAIN: f64 = 0.005;

/// Why the second attempt is, or is not, the result.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum RetryReason {
    /// It puts clearly more of the scan within the tolerance (or closes an open network).
    Better,
    /// It is not clearly better than the first: the first stands.
    NotBetter,
    /// It is open where the first is closed: the first stands.
    OpenResult,
    /// The second wrap failed (see `error`): the first stands.
    Failed,
    /// The call had used its extra attempts: no second wrap was made, the first stands.
    BudgetSpent,
}

/// The second attempt of the network with a larger patch limit.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct CapRetry {
    /// `max_patches` of the first attempt, which stopped at it short of the tolerance.
    pub cap_from: usize,
    /// `max_patches` of the second attempt.
    pub cap_to: usize,
    /// Patches of the first attempt.
    pub patches_from: usize,
    /// Patches of the second attempt (absent when it failed).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub patches_to: Option<usize>,
    /// Share of the scan within the tolerance after the first attempt (0..1).
    pub within_from: f64,
    /// Share of the scan within the tolerance after the second attempt (absent when it failed).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub within_to: Option<f64>,
    /// Is the second attempt the result?
    pub kept: bool,
    /// Why it is, or is not.
    pub reason: RetryReason,
    /// The error of the second attempt when it failed.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
}

/// What a retry is judged on.
#[derive(Clone, Copy, Debug)]
pub(super) struct Attempt {
    pub(super) closed: bool,
    pub(super) within: f64,
}

impl Attempt {
    fn of(w: &WrapOut) -> Self {
        Self { closed: w.closed, within: w.deviation.within }
    }
}

/// The one rule of the retry: the second network replaces the first only when it is not worse on
/// closedness (a closed body is never replaced by an open one) and wins by a margin of
/// [`RETRY_MIN_GAIN`] in share within the tolerance; closing an open network at no loss also counts.
pub(super) fn judge(first: Attempt, second: Attempt) -> RetryReason {
    if first.closed && !second.closed {
        return RetryReason::OpenResult;
    }
    let clearly_more = second.within > first.within + RETRY_MIN_GAIN;
    let closes = second.closed && !first.closed && second.within >= first.within;
    if clearly_more || closes { RetryReason::Better } else { RetryReason::NotBetter }
}

/// What keeps a wrapped scan from a pass.
pub(super) fn issues_of(w: &WrapOut, d: &DeviationSummary, set: &Settings, report: &ScanSummary) -> Vec<super::Issue> {
    let mut list = Vec::new();
    if !w.closed {
        let msg = "the patch network is not closed (the scan has open boundaries): the result is a sheet, not a solid";
        list.push(issue("open_surface", msg.to_string()));
    }
    list.extend(coverage_issue(d, "the patches"));
    if d.count > 0 && d.within < set.min_within {
        let msg = format!(
            "{:.2} % of the measured points are within {} mm ({:.0} % required); max {:.3} mm, RMS {:.3} mm; {} patches{}",
            100.0 * d.within,
            set.tolerance,
            100.0 * set.min_within,
            d.max_abs,
            d.rms,
            w.patches,
            if w.capped { ", stopped by the patch limit" } else { "" }
        );
        list.push(issue("out_of_tolerance", msg));
    }
    if !list.is_empty() && report.holes_left > 0 {
        list.push(issue("scan_defects", format!("{} holes of the scan were too large to fill", report.holes_left)));
    }
    if !list.is_empty() && report.non_manifold_edges > 0 {
        let msg = format!("{} edges are shared by three or more triangles", report.non_manifold_edges);
        list.push(issue("non_manifold_edges", msg));
    }
    list
}

pub(super) fn next_steps(w: &WrapOut, issues: &[super::Issue], set: &Settings, retried: bool) -> Vec<NextStep> {
    let has = |code: &str| issues.iter().any(|i| i.code == code);
    if issues.is_empty() {
        let why = "the solid passed: export it (pass `path` when files are allowed, else read `data`)";
        return vec![step("export.step", json!({"objects": [w.body.id]}), why)];
    }
    let mut steps = Vec::new();
    if has("scan_defects") {
        let why = "fills the holes that were too large, so the patch network can close";
        steps.push(step("mesh.fill_holes", json!({"object": set.scan, "max_edges": MAX_FILL_EDGES}), why));
        let again = json!({"scan": set.scan, "method": "freeform", "tolerance": set.tolerance});
        steps.push(step("scan.to_solid", again, "wraps the repaired scan"));
    }
    // a larger cap that was already tried in this call is not suggested again
    if has("out_of_tolerance") && w.cap_used < PATCH_LIMIT && !retried {
        let why = "a finer network: more and smaller patches (a second body; delete the first)";
        let params = json!({"mesh": set.scan, "tolerance": set.tolerance, "max_patches": PATCH_LIMIT});
        steps.push(step("surface.wrap", params, why));
    }
    if has("open_surface") {
        let why = "the scan is open, a sheet: if the part is a panel or a shell scanned from one side, give the wall `thickness` in mm (2 is only a placeholder: measure the part; negative puts the material in front of the scanned face, `symmetric` half on each side) and the scan is thickened into a closed solid";
        let params = json!({"scan": set.scan, "thickness": PLACEHOLDER_THICKNESS_MM, "tolerance": set.tolerance});
        steps.insert(0, step("scan.to_solid", params, why));
    }
    steps.push(step(
        "guide.read",
        json!({"topic": "scan-to-solid"}),
        "what each verdict and issue means and how to go on by hand",
    ));
    steps
}

/// The result of a run in which nothing could be wrapped.
fn failed(set: &Settings, report: ScanSummary, why: &str) -> ToSolidOut {
    let issues = vec![issue("no_solid", format!("the scan could not be wrapped in patches: {why}"))];
    let next = vec![step(
        "guide.read",
        json!({"topic": "scan-to-solid"}),
        "what each verdict and issue means and how to go on by hand",
    )];
    out(set, None, report, Verdict::Fail, None, None, issues, next)
}

pub(super) fn out(
    set: &Settings,
    retry: Option<CapRetry>,
    report: ScanSummary,
    verdict: Verdict,
    w: Option<&WrapOut>,
    deviation: Option<DeviationSummary>,
    issues: Vec<super::Issue>,
    next_steps: Vec<NextStep>,
) -> ToSolidOut {
    ToSolidOut {
        verdict,
        method: Method::Freeform,
        scan: set.scan,
        body: w.map(|w| w.body.clone()),
        patches: w.map(|w| w.patches),
        retry,
        tolerance: set.tolerance,
        deviation,
        worst_faces: Vec::new(),
        scan_report: report,
        modelled: Vec::new(),
        left_out: Vec::new(),
        left_out_regions: 0,
        left_out_triangles: 0,
        unassigned_triangles: 0,
        relations: 0,
        issues,
        next_steps,
        attempts: Vec::new(),
        prep: super::PrepOut::default(),
        thickened: None,
    }
}

/// A failed retry never leaves a second body behind.
fn remove_bodies_added_since(s: &mut Session, before: &[ObjectId]) {
    let added: Vec<ObjectId> = s.doc.bodies.iter().map(|b| b.id).filter(|id| !before.contains(id)).collect();
    for id in added {
        s.doc.remove(id);
    }
}

/// The cap of the larger-cap retry when one is worth making: the first attempt stopped at its limit
/// short of the tolerance, reached at least [`RETRY_MIN_WITHIN`], the wrapped mesh is not too big,
/// and the caller neither gave `max_patches` (an explicit cap is a promise) nor switched
/// `allow_retry` off.
fn retry_cap(s: &Session, set: &Settings, wrapped: ObjectId, first: &WrapOut) -> Option<usize> {
    let cap_to = first.cap_used.saturating_mul(RETRY_FACTOR).min(PATCH_LIMIT);
    // the size that counts is that of the mesh actually wrapped (a rebuilt scan can be much larger)
    let triangles = s.doc.mesh(wrapped).map_or(usize::MAX, |m| m.mesh.triangle_count());
    let worth = set.allow_retry
        && set.max_patches.is_none()
        && first.capped
        && cap_to > first.cap_used
        && first.deviation.within >= RETRY_MIN_WITHIN
        && triangles <= RETRY_MAX_TRIANGLES;
    worth.then_some(cap_to)
}

/// The record of a retry that has not run yet.
fn retry_note(first: &WrapOut, cap_to: usize, reason: RetryReason) -> CapRetry {
    CapRetry {
        cap_from: first.cap_used,
        cap_to,
        patches_from: first.patches,
        patches_to: None,
        within_from: first.deviation.within,
        within_to: None,
        kept: false,
        reason,
        error: None,
    }
}

/// Keeps the better of the two wraps by [`judge`] and removes the other body.
fn settle(s: &mut Session, set: &Settings, first: WrapOut, second: WrapOut, base: CapRetry) -> (WrapOut, CapRetry) {
    let reason = judge(Attempt::of(&first), Attempt::of(&second));
    let kept = reason == RetryReason::Better;
    let within_to = Some(second.deviation.within).filter(|v| v.is_finite());
    let info = CapRetry { patches_to: Some(second.patches), within_to, kept, reason, ..base };
    let (keep, drop) = if kept { (second, first) } else { (first, second) };
    s.doc.remove(drop.body.id);
    if kept {
        // the second wrap ran while the first body still held the name: the result gets it back
        s.doc.rename(keep.body.id, &set.name);
    }
    (keep, info)
}

/// Wraps the scan a second time with a larger limit (see [`retry_cap`]) and keeps the better of the
/// two. The retry is an extra attempt of the call's budget; when the budget is spent the first wrap
/// stands and the result says so (`budget_spent`). One retry at most; a failed one leaves the first
/// attempt as it is and says why. `wrap_with` makes the second wrap (the command's own, or a test's).
pub(super) fn retry_larger_on(
    s: &mut Session,
    set: &Settings,
    wrapped: ObjectId,
    first: WrapOut,
    wrap_with: impl FnOnce(&mut Session, usize) -> CmdResult<WrapOut>,
) -> CmdResult<(WrapOut, Option<CapRetry>)> {
    // a call cancelled meanwhile does not start the second wrap (the caller's next check ends it)
    let Some(cap_to) = retry_cap(s, set, wrapped, &first).filter(|_| !s.cancel.is_cancelled()) else {
        return Ok((first, None));
    };
    let base = retry_note(&first, cap_to, RetryReason::Failed);
    if !set.budget.draw() {
        return Ok((first, Some(CapRetry { reason: RetryReason::BudgetSpent, ..base })));
    }
    let before: Vec<ObjectId> = s.doc.bodies.iter().map(|b| b.id).collect();
    match wrap_with(s, cap_to) {
        Ok(second) => {
            let (kept, info) = settle(s, set, first, second, base);
            Ok((kept, Some(info)))
        }
        Err(e) => {
            remove_bodies_added_since(s, &before);
            if crate::cmd::is_cancelled(&e) {
                return Err(e);
            }
            Ok((first, Some(CapRetry { error: Some(message_of(&e).to_string()), ..base })))
        }
    }
}

/// [`retry_larger_on`] for the scan itself.
#[cfg(test)]
fn retry_larger(
    s: &mut Session,
    set: &Settings,
    first: WrapOut,
    wrap_with: impl FnOnce(&mut Session, usize) -> CmdResult<WrapOut>,
) -> (WrapOut, Option<CapRetry>) {
    retry_larger_on(s, set, set.scan, first, wrap_with).unwrap_or_else(|e| panic!("{e:?}"))
}

#[cfg(test)]
fn wrap_with(s: &mut Session, set: &Settings, max_patches: Option<usize>) -> CmdResult<WrapOut> {
    wrap_mesh(s, set, set.scan, max_patches)
}

fn wrap_mesh(s: &mut Session, set: &Settings, mesh: ObjectId, max_patches: Option<usize>) -> CmdResult<WrapOut> {
    wrap(
        s,
        WrapIn {
            mesh: ObjRef::Id(mesh.0),
            tolerance: Some(set.tolerance),
            min_within: Some(set.min_within),
            max_patches,
            refine: None,
            crease_deg: None,
            continuity: None,
            name: Some(set.name.clone()),
        },
    )
}

/// Wraps the scan and judges the result (with the preparation of `pipeline_prepare` where it is needed).
pub(super) fn freeform_solid(s: &mut Session, set: &Settings, report: ScanSummary) -> CmdResult<ToSolidOut> {
    super::rebuild::freeform_prepared(s, set, report)
}

/// Wraps `mesh` (the scan, or a rebuilt copy of it) and judges the result against the scan.
pub(super) fn freeform_on(
    s: &mut Session,
    set: &Settings,
    mesh: ObjectId,
    report: ScanSummary,
) -> CmdResult<ToSolidOut> {
    freeform_with_mesh(s, set, mesh, report, |s, cap| wrap_mesh(s, set, mesh, cap))
}

/// [`freeform_solid`] with the wrap made by `wrap_fn` (given the patch limit, `None` for the default):
/// the command's own, or a test's.
#[cfg(test)]
fn freeform_with(
    s: &mut Session,
    set: &Settings,
    report: ScanSummary,
    wrap_fn: impl FnMut(&mut Session, Option<usize>) -> CmdResult<WrapOut>,
) -> CmdResult<ToSolidOut> {
    freeform_with_mesh(s, set, set.scan, report, wrap_fn)
}

/// [`freeform_with`] for the wrap of `wrapped` (the scan, or a rebuilt copy of it).
fn freeform_with_mesh(
    s: &mut Session,
    set: &Settings,
    wrapped: ObjectId,
    report: ScanSummary,
    mut wrap_fn: impl FnMut(&mut Session, Option<usize>) -> CmdResult<WrapOut>,
) -> CmdResult<ToSolidOut> {
    let first = match wrap_fn(s, set.max_patches) {
        Ok(w) => w,
        Err(e) if crate::cmd::is_cancelled(&e) => return Err(e),
        Err(e) => return Ok(failed(set, report, message_of(&e))),
    };
    let (w, retry) = retry_larger_on(s, set, wrapped, first, |s, cap| wrap_fn(s, Some(cap)))?;
    super::prepare::check_cancel(s)?;
    let cmp = compare(
        s,
        CompareIn {
            body: ObjRef::Id(w.body.id.0),
            scan: ObjRef::Id(set.scan.0),
            tolerance: Some(set.tolerance),
            max_distance: None,
            chord: None,
            facing: true,
        },
    )?;
    let vertices = s.doc.mesh(set.scan).map_or(0, |m| m.mesh.vertex_count());
    let d = summary(&cmp, vertices);
    let mut issues = issues_of(&w, &d, set, &report);
    let diagonal = s.doc.mesh(set.scan).map_or(f64::NAN, |m| robust_diagonal(&m.mesh));
    if let Some(msg) = scale_warning(set.tolerance, diagonal) {
        issues.insert(0, issue("tolerance_exceeds_part", msg));
    }
    let tried = retry.as_ref().is_some_and(|r| r.reason != RetryReason::BudgetSpent);
    let next = next_steps(&w, &issues, set, tried);
    let verdict = if issues.is_empty() {
        Verdict::Pass
    } else if d.coverage >= MIN_COVERAGE {
        Verdict::Review
    } else {
        Verdict::Fail
    };
    Ok(out(set, retry, report, verdict, Some(&w), Some(d), issues, next))
}

#[cfg(test)]
#[path = "pipeline_freeform_tests.rs"]
mod tests;
