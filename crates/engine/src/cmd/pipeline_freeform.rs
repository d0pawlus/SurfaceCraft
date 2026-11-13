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
fn issues_of(w: &WrapOut, d: &DeviationSummary, set: &Settings, report: &ScanSummary) -> Vec<super::Issue> {
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

fn next_steps(w: &WrapOut, issues: &[super::Issue], set: &Settings, retried: bool) -> Vec<NextStep> {
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
    if has("open_surface") && !has("scan_defects") {
        let why = "turns the sheet into a solid of constant thickness when the part is a shell (give `thickness`)";
        steps.push(step("guide.read", json!({"topic": "fit-surfaces"}), why));
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

fn out(
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
        prep: super::PrepOut::default(),
    }
}

/// Wraps the scan a second time with a larger limit when the first attempt stopped at its limit short
/// of the tolerance, reached at least [`RETRY_MIN_WITHIN`] and the scan is not too big, and keeps
/// the better of the two by [`judge`] (the other body is removed). Never done when the caller gave
/// `max_patches` (an explicit cap is a promise) or switched `allow_retry` off. One retry at most; a
/// failed one leaves the first attempt as it is and says why in the result. `wrap_with` makes the
/// second wrap (the command's own, or a test's).
fn retry_larger_on(
    s: &mut Session,
    set: &Settings,
    wrapped: ObjectId,
    first: WrapOut,
    wrap_with: impl FnOnce(&mut Session, usize) -> CmdResult<WrapOut>,
) -> CmdResult<(WrapOut, Option<CapRetry>)> {
    let cap_from = first.cap_used;
    let cap_to = cap_from.saturating_mul(RETRY_FACTOR).min(PATCH_LIMIT);
    // the size that counts is that of the mesh actually wrapped (a rebuilt scan can be much larger)
    let triangles = s.doc.mesh(wrapped).map_or(usize::MAX, |m| m.mesh.triangle_count());
    let worth = set.allow_retry
        && set.max_patches.is_none()
        && first.capped
        && cap_to > cap_from
        && first.deviation.within >= RETRY_MIN_WITHIN
        && triangles <= RETRY_MAX_TRIANGLES;
    if !worth {
        return Ok((first, None));
    }
    let base = CapRetry {
        cap_from,
        cap_to,
        patches_from: first.patches,
        patches_to: None,
        within_from: first.deviation.within,
        within_to: None,
        kept: false,
        reason: RetryReason::Failed,
        error: None,
    };
    let before: Vec<ObjectId> = s.doc.bodies.iter().map(|b| b.id).collect();
    let second = match wrap_with(s, cap_to) {
        Ok(w) => w,
        Err(e) => {
            // a failed retry never leaves a second body behind
            let added: Vec<ObjectId> = s.doc.bodies.iter().map(|b| b.id).filter(|id| !before.contains(id)).collect();
            for id in added {
                s.doc.remove(id);
            }
            if crate::cmd::is_cancelled(&e) {
                return Err(e);
            }
            return Ok((first, Some(CapRetry { error: Some(message_of(&e).to_string()), ..base })));
        }
    };
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
    Ok((keep, Some(info)))
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
    let next = next_steps(&w, &issues, set, retry.is_some());
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
mod tests {
    use super::*;
    use crate::cmd::CmdError;
    use surfacecraft_doc::ObjectId;
    use surfacecraft_geom::{Point3, Vec3, synth};

    const TOLERANCE: f64 = 0.2;

    /// A closed sphere rippled all over: needs far more patches than a cap of a few hundred.
    fn rippled() -> surfacecraft_geom::Mesh {
        let k = std::f64::consts::TAU * 20.0 / 5.0;
        let mut m = synth::sphere(Point3::origin(), 1.0, 120, 60);
        for p in &mut m.positions {
            let d: Vec3 = p.coords;
            p.coords = d * (20.0 + 0.7 * (k * d.x + 0.3).sin() * (k * d.y + 0.7).sin() * (0.5 * k * d.z).cos());
        }
        m
    }

    fn session() -> (Session, Settings) {
        let mut s = Session::new();
        let scan = s.doc.add_mesh("scan", rippled());
        let set = Settings {
            scan,
            tolerance: TOLERANCE,
            min_within: 0.99,
            min_triangles: None,
            max_patches: None,
            allow_retry: true,
            rebuild: true,
            name: "w".into(),
        };
        (s, set)
    }

    /// A wrap with `cap` patches whose result is forced to look capped and short of the tolerance.
    fn first_attempt(s: &mut Session, set: &Settings, cap: usize) -> WrapOut {
        let mut w = wrap_with(s, set, Some(cap)).unwrap();
        w.capped = true;
        w
    }

    fn bodies(s: &Session) -> usize {
        s.doc.bodies.len()
    }

    #[test]
    fn a_retry_that_covers_more_but_is_open_is_rejected() {
        let closed = Attempt { closed: true, within: 0.5 };
        assert_eq!(judge(closed, Attempt { closed: false, within: 0.9 }), RetryReason::OpenResult);
    }

    #[test]
    fn a_retry_that_ties_or_gains_less_than_the_margin_is_rejected() {
        let first = Attempt { closed: true, within: 0.80 };
        assert_eq!(judge(first, Attempt { closed: true, within: 0.80 }), RetryReason::NotBetter);
        assert_eq!(judge(first, Attempt { closed: true, within: 0.80 + RETRY_MIN_GAIN }), RetryReason::NotBetter);
        assert_eq!(judge(first, Attempt { closed: true, within: 0.80 + 2.0 * RETRY_MIN_GAIN }), RetryReason::Better);
    }

    #[test]
    fn closing_an_open_network_at_no_loss_counts_as_better() {
        let open = Attempt { closed: false, within: 0.8 };
        assert_eq!(judge(open, Attempt { closed: true, within: 0.8 }), RetryReason::Better);
        assert_eq!(judge(open, Attempt { closed: true, within: 0.7 }), RetryReason::NotBetter);
    }

    #[test]
    fn a_retry_with_a_larger_cap_that_is_clearly_better_replaces_the_first_body() {
        // Arrange
        let (mut s, set) = session();
        let first = first_attempt(&mut s, &set, 300);
        assert!(first.deviation.within >= RETRY_MIN_WITHIN);

        // Act
        let (kept, retry) = retry_larger(&mut s, &set, first.clone(), |s, cap| wrap_with(s, &set, Some(cap)));

        // Assert
        let retry = retry.unwrap();
        assert_eq!((retry.cap_from, retry.cap_to), (300, 1500));
        assert!(retry.kept && retry.reason == RetryReason::Better, "{retry:?}");
        assert!(kept.patches > first.patches && kept.patches <= 1500);
        assert_eq!(bodies(&s), 1);
        assert_eq!(s.doc.body(kept.body.id).unwrap().name, set.name, "the result keeps the requested name");
    }

    #[test]
    fn a_retry_that_loses_leaves_the_first_body_and_says_why() {
        // Arrange: the second wrap is the real one, forced to tie
        let (mut s, set) = session();
        let first = first_attempt(&mut s, &set, 300);
        let (first_id, first_within) = (first.body.id, first.deviation.within);

        // Act
        let (kept, retry) = retry_larger(&mut s, &set, first, |s, cap| {
            let mut w = wrap_with(s, &set, Some(cap))?;
            w.deviation.within = first_within;
            Ok(w)
        });

        // Assert
        let retry = retry.unwrap();
        assert!(!retry.kept && retry.reason == RetryReason::NotBetter, "{retry:?}");
        assert_eq!(kept.body.id, first_id);
        assert_eq!(bodies(&s), 1);
        assert!(s.doc.body(ObjectId(first_id.0)).is_some());
    }

    #[test]
    fn a_failed_retry_is_reported_with_its_error() {
        // Arrange
        let (mut s, set) = session();
        let first = first_attempt(&mut s, &set, 300);

        // Act
        let (_, retry) = retry_larger(&mut s, &set, first, |_, _| Err(CmdError::Failed("boom".into())));

        // Assert
        let retry = retry.unwrap();
        assert_eq!(retry.reason, RetryReason::Failed);
        assert_eq!(retry.error.as_deref(), Some("boom"));
        assert!(!retry.kept && retry.patches_to.is_none());
        assert_eq!(bodies(&s), 1);
    }

    #[test]
    fn an_explicit_cap_or_allow_retry_false_is_never_exceeded() {
        for (max_patches, allow_retry) in [(Some(300), true), (None, false)] {
            // Arrange
            let (mut s, set) = session();
            let set = Settings { max_patches, allow_retry, ..set };
            let first = first_attempt(&mut s, &set, 300);

            // Act
            let (_, retry) = retry_larger(&mut s, &set, first, |_, _| panic!("no second wrap may be made"));

            // Assert
            assert!(retry.is_none());
            assert_eq!(bodies(&s), 1);
        }
    }

    #[test]
    fn the_larger_cap_is_not_suggested_again_after_a_retry() {
        // Arrange
        let (mut s, set) = session();
        let w = first_attempt(&mut s, &set, 300);
        let issues = vec![issue("out_of_tolerance", "x".into())];

        // Act
        let fresh = next_steps(&w, &issues, &set, false);
        let retried = next_steps(&w, &issues, &set, true);

        // Assert
        assert!(fresh.iter().any(|st| st.command == "surface.wrap"));
        assert!(retried.iter().all(|st| st.command != "surface.wrap"));
    }

    #[test]
    fn a_failed_retry_removes_any_body_it_added() {
        // Arrange
        let (mut s, set) = session();
        let first = first_attempt(&mut s, &set, 300);
        let (id, name) = (first.body.id, s.doc.body(first.body.id).unwrap().name.clone());

        // Act: the wrap adds a body and then fails
        let (kept, retry) = retry_larger(&mut s, &set, first, |s, cap| {
            wrap_with(s, &set, Some(cap))?;
            Err(CmdError::Failed("late failure".into()))
        });

        // Assert
        assert_eq!(retry.unwrap().reason, RetryReason::Failed);
        assert_eq!(bodies(&s), 1);
        assert_eq!(kept.body.id, id);
        assert_eq!(s.doc.body(id).unwrap().name, name);
    }

    #[test]
    fn a_share_that_is_not_a_number_is_not_reported_as_one() {
        // Arrange
        let (mut s, set) = session();
        let first = first_attempt(&mut s, &set, 300);

        // Act
        let (_, retry) = retry_larger(&mut s, &set, first, |s, cap| {
            let mut w = wrap_with(s, &set, Some(cap))?;
            w.deviation.within = f64::NAN;
            Ok(w)
        });

        // Assert
        let retry = retry.unwrap();
        assert!(retry.within_to.is_none() && !retry.kept, "{retry:?}");
        assert!(serde_json::to_value(&retry).unwrap().get("within_to").is_none());
    }

    fn scan_summary() -> ScanSummary {
        ScanSummary {
            triangles_before: 0,
            triangles: 0,
            holes_before: 0,
            holes_filled: 0,
            holes_left: 0,
            non_manifold_edges: 0,
            noise_sigma: None,
            findings: Vec::new(),
        }
    }

    /// The assembled output of `freeform_with` when the first wrap is forced to be capped and the
    /// second is the real wrap, optionally forced to tie.
    fn assembled(second_ties: bool) -> (Session, Settings, ToSolidOut) {
        let (mut s, set) = session();
        let mut first_within = 0.0;
        let out = freeform_with(&mut s, &set, scan_summary(), |s, cap| {
            let mut w = wrap_with(s, &set, Some(cap.unwrap_or(300)))?;
            if cap.is_none() {
                w.capped = true;
                first_within = w.deviation.within;
            } else if second_ties {
                w.deviation.within = first_within;
            }
            Ok(w)
        })
        .unwrap();
        (s, set, out)
    }

    #[test]
    fn the_assembled_output_of_a_winning_retry_has_the_second_body_under_the_requested_name() {
        // Act
        let (s, set, out) = assembled(false);

        // Assert
        let retry = out.retry.as_ref().unwrap();
        assert!(retry.kept && retry.reason == RetryReason::Better, "{retry:?}");
        let id = out.body.as_ref().unwrap().id;
        assert_eq!(s.doc.body(id).unwrap().name, set.name);
        assert_eq!(bodies(&s), 1);
        assert_eq!(out.patches, retry.patches_to);
        assert!(out.next_steps.iter().all(|st| st.command != "surface.wrap"));
    }

    #[test]
    fn the_assembled_output_of_a_losing_retry_keeps_the_first_body_and_suggests_no_larger_cap() {
        // Act
        let (s, set, out) = assembled(true);

        // Assert
        let retry = out.retry.as_ref().unwrap();
        assert!(!retry.kept && retry.reason == RetryReason::NotBetter, "{retry:?}");
        let id = out.body.as_ref().unwrap().id;
        assert_eq!(s.doc.body(id).unwrap().name, set.name);
        assert_eq!(bodies(&s), 1);
        assert_eq!(out.patches, Some(retry.patches_from));
        assert!(out.next_steps.iter().all(|st| st.command != "surface.wrap"));
    }

    #[test]
    fn a_cancelled_wrap_is_a_cancelled_error_not_a_failed_result() {
        let (mut s, set) = session();
        let r = freeform_with(&mut s, &set, empty_report(), |_, _| Err(crate::cmd::cancelled()));
        assert!(matches!(r, Err(ref e) if crate::cmd::is_cancelled(e)));
        let other = freeform_with(&mut s, &set, empty_report(), |_, _| Err(CmdError::Failed("boom".into())));
        assert!(other.is_ok_and(|o| o.verdict == Verdict::Fail), "any other failure is a failed result");
    }

    fn empty_report() -> ScanSummary {
        ScanSummary {
            triangles_before: 0,
            triangles: 0,
            holes_before: 0,
            holes_filled: 0,
            holes_left: 0,
            non_manifold_edges: 0,
            noise_sigma: None,
            findings: Vec::new(),
        }
    }

    #[test]
    fn a_cancelled_larger_cap_wrap_is_a_cancelled_error_and_leaves_no_second_body() {
        let (mut s, set) = session();
        let first = first_attempt(&mut s, &set, 300);
        let bodies = s.doc.bodies.len();
        let r = retry_larger_on(&mut s, &set, set.scan, first, |_, _| Err(crate::cmd::cancelled()));
        assert!(matches!(r, Err(ref e) if crate::cmd::is_cancelled(e)));
        assert_eq!(s.doc.bodies.len(), bodies);
    }

    #[test]
    fn the_size_limit_of_the_retry_is_that_of_the_mesh_wrapped() {
        let (mut s, set) = session();
        let first = first_attempt(&mut s, &set, 300);
        let big = surfacecraft_geom::synth::sphere(Point3::origin(), 20.0, 1300, 700);
        assert!(big.triangle_count() > RETRY_MAX_TRIANGLES);
        let huge = s.doc.add_mesh("rebuilt", big);
        let r = retry_larger_on(&mut s, &set, huge, first, |_, _| panic!("no second wrap for a mesh this large"));
        assert!(matches!(r, Ok((_, None))));
    }

    #[test]
    fn the_wraps_of_a_call_are_one_unless_the_retry_is_allowed() {
        for (allow, expected) in [(true, 2usize), (false, 1)] {
            let (mut s, set) = session();
            let set = Settings { allow_retry: allow, ..set };
            let mut calls = 0usize;
            let out = freeform_with(&mut s, &set, empty_report(), |s, cap| {
                calls += 1;
                let mut w = wrap_with(s, &set, Some(cap.unwrap_or(300)))?;
                w.capped = true;
                w.deviation.within = 0.5;
                Ok(w)
            });
            assert!(out.is_ok());
            assert_eq!(calls, expected, "allow_retry {allow}");
        }
    }
}
