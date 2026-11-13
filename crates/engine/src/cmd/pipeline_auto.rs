//! `scan.to_solid` with `method: auto`: the choice of method is by outcome. The share of the scan in
//! regions a primitive fits only orders the attempts; when the first does not end in a pass the
//! other is run too (unless the caller forbids it, or the scan is too large for a second run to be
//! worth its time), and the better result is returned. The losing attempt leaves nothing in the
//! document, and the whole run is still one undo step.
use super::super::{CANCELLED, CmdError, cancelled};
use super::budget::CallBudget;
use super::{
    CmdResult, Issue, MIN_MODELLED_SHARE, Method, NextStep, ScanSummary, Settings, ToSolidIn, ToSolidOut, Verdict,
    freeform, message_of, primitives_solid, scan_primitive_share, segment_scan, step,
};
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use surfacecraft_doc::ObjectId;

/// The share of the scan in primitive regions below which a primitives attempt is not tried after a
/// failed freeform one (half the share that makes it the first choice).
const PLAUSIBLE_SHARE: f64 = MIN_MODELLED_SHARE / 2.0;
/// A second method is not tried on a scan of more triangles than this: first attempts of several
/// minutes occur at a million triangles and more, and a second full run doubles that. A caller can
/// force it with the call the skipped entry lists.
const SECOND_ATTEMPT_MAX_TRIANGLES: usize = 600_000;
/// ... nor after a first result of this many patches or more (half the patch limit): such a network
/// took minutes on the real scans (a 60 000-patch one 450 s). The bounds are functions of the input
/// and of the first result, never of the clock: a time limit would skip the second method on a slow
/// or loaded machine and run it on a fast one.
const SECOND_ATTEMPT_MAX_PATCHES: usize = 30_000;
/// Two results explain the scan equally well when their shares differ by no more than this (a
/// sheet at 0.995 does not beat a closed solid at 0.990).
const EXPLAINED_BAND: f64 = 0.005;
/// Longest error text kept in an attempt.
const ERROR_TEXT_LIMIT: usize = 200;

/// What bounds the second attempt.
#[derive(Clone, Copy, Debug)]
pub(super) struct Limits {
    pub max_triangles: usize,
    pub max_patches: usize,
}

impl Default for Limits {
    fn default() -> Self {
        Self { max_triangles: SECOND_ATTEMPT_MAX_TRIANGLES, max_patches: SECOND_ATTEMPT_MAX_PATCHES }
    }
}

/// Why a second attempt was not run, and the call that forces it.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct Skipped {
    /// Why.
    pub reason: String,
    /// The call that runs that method alone.
    pub call: NextStep,
}

/// One method tried (or skipped) by `scan.to_solid`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct Attempt {
    /// The method tried.
    pub method: Method,
    /// How it ended; absent when it was skipped or failed with an error.
    pub verdict: Option<Verdict>,
    /// The codes of the issues that kept it from a pass.
    pub issues: Vec<String>,
    /// Wall time of the attempt (s; 0 where there is no clock).
    pub seconds: f64,
    /// This attempt's result is the one returned.
    pub returned: bool,
    /// The error that ended it, when it ended in one (the other attempt's result is returned).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Set when the attempt was not run.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub skipped: Option<Skipped>,
}

/// Result of `scan.to_solid`: the result of the method returned, and the attempts.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ToSolidReport {
    /// The returned attempt's result.
    #[serde(flatten)]
    pub result: ToSolidOut,
    /// Every method tried, in order; one when the first passed or the method was given.
    pub attempts: Vec<Attempt>,
}

/// A clock that is absent on wasm.
struct Timer(#[cfg(not(target_arch = "wasm32"))] std::time::Instant);

impl Timer {
    fn start() -> Self {
        #[cfg(not(target_arch = "wasm32"))]
        {
            Self(std::time::Instant::now())
        }
        #[cfg(target_arch = "wasm32")]
        {
            Self()
        }
    }

    fn seconds(&self) -> f64 {
        #[cfg(not(target_arch = "wasm32"))]
        {
            self.0.elapsed().as_secs_f64()
        }
        #[cfg(target_arch = "wasm32")]
        {
            0.0
        }
    }
}

fn rank(v: Verdict) -> u8 {
    match v {
        Verdict::Pass => 2,
        Verdict::Review => 1,
        Verdict::Fail => 0,
    }
}

/// Share of ALL the scan's points that the result measures and finds within the tolerance (points
/// facing away from the solid are not measured, so they do not count).
fn explained(r: &ToSolidOut) -> f64 {
    r.deviation.as_ref().map_or(0.0, |d| d.within * d.coverage)
}

/// 2 for a closed solid, 1 for a sheet, 0 for no body.
fn closedness(r: &ToSolidOut) -> u8 {
    r.body.as_ref().map_or(0, |b| if b.volume.is_some() { 2 } else { 1 })
}

/// Orders results: by verdict; then by the share of the scan explained, shares within
/// [`EXPLAINED_BAND`] being equal; then a closed solid over a sheet over no body. Antisymmetric.
fn compare(a: &ToSolidOut, b: &ToSolidOut) -> std::cmp::Ordering {
    let by_verdict = rank(a.verdict).cmp(&rank(b.verdict));
    let (ea, eb) = (explained(a), explained(b));
    let by_share = if (ea - eb).abs() > EXPLAINED_BAND + 1e-9 { ea.total_cmp(&eb) } else { std::cmp::Ordering::Equal };
    by_verdict.then(by_share).then(closedness(a).cmp(&closedness(b)))
}

/// Is `b` a better result than `a`? A tie keeps `a`.
fn is_better(b: &ToSolidOut, a: &ToSolidOut) -> bool {
    compare(b, a) == std::cmp::Ordering::Greater
}

/// What `orchestrate` needs to know about the scan.
pub(super) struct Plan {
    pub scan: ObjectId,
    pub tolerance: f64,
    /// Share of the scan in regions a primitive fits.
    pub share: f64,
    pub triangles: usize,
    /// `retry` of the call.
    pub retry: bool,
    pub limits: Limits,
    /// The call's budget of extra attempts, shared with the freeform fit's retry.
    pub budget: CallBudget,
}

fn record(method: Method, result: &ToSolidOut, seconds: f64) -> Attempt {
    Attempt {
        method,
        verdict: Some(result.verdict),
        issues: result.issues.iter().map(|x| x.code.clone()).collect(),
        seconds,
        returned: false,
        error: None,
        skipped: None,
    }
}

fn is_cancelled(e: &CmdError) -> bool {
    matches!(e, CmdError::Failed(m) if m == CANCELLED)
}

fn method_name(m: Method) -> &'static str {
    if m == Method::Freeform { "freeform" } else { "primitives" }
}

/// The reason not to run a second attempt, with the call that forces it.
fn skip_reason(plan: &Plan, first: &ToSolidOut) -> Option<String> {
    if !plan.retry {
        return Some("`retry` is false".to_string());
    }
    if plan.triangles > plan.limits.max_triangles {
        return Some(format!(
            "the scan has {} triangles, more than the {} a second attempt is worth",
            plan.triangles, plan.limits.max_triangles
        ));
    }
    first.patches.filter(|&p| p >= plan.limits.max_patches).map(|p| {
        format!("the first result has {p} patches, at least the {} a second attempt is worth", plan.limits.max_patches)
    })
}

fn skipped_attempt(plan: &Plan, method: Method, reason: String) -> Attempt {
    let call = step(
        "scan.to_solid",
        json!({"scan": plan.scan.0, "method": method_name(method), "tolerance": plan.tolerance}),
        "runs the method that was not tried",
    );
    Attempt {
        method,
        verdict: None,
        issues: Vec::new(),
        seconds: 0.0,
        returned: false,
        error: None,
        skipped: Some(Skipped { reason, call }),
    }
}

fn short(text: &str) -> String {
    text.chars().take(ERROR_TEXT_LIMIT).collect()
}

/// The winner after both methods ran: next steps that name a method `auto` tried are dropped (the
/// advice would send the caller where it has been), and a winner that is no pass says so.
fn finish(mut winner: ToSolidOut, loser_method: Method, loser: &ToSolidOut) -> ToSolidOut {
    winner.next_steps.retain(|st| {
        st.command != "scan.to_solid" || !matches!(st.params["method"].as_str(), Some("primitives" | "freeform"))
    });
    if winner.verdict != Verdict::Pass {
        winner.issues.push(Issue {
            code: "other_method_tried".into(),
            message: format!(
                "`auto` tried `{}` too: {:?}, {:.0} % of the scan explained against {:.0} %; it did not do better",
                method_name(loser_method),
                loser.verdict,
                100.0 * explained(loser),
                100.0 * explained(&winner)
            ),
        });
    }
    winner
}

/// Runs the attempts of `auto` with `exec` (one method on the document) and returns the best.
pub(super) fn orchestrate(
    s: &mut Session,
    plan: &Plan,
    exec: &mut dyn FnMut(&mut Session, Method) -> CmdResult<ToSolidOut>,
) -> CmdResult<ToSolidReport> {
    let (first, second) = if plan.share >= MIN_MODELLED_SHARE {
        (Method::Primitives, Method::Freeform)
    } else {
        (Method::Freeform, Method::Primitives)
    };
    let base = s.doc.clone();
    let timer = Timer::start();
    let first_result = exec(s, first)?;
    let first_seconds = timer.seconds();
    let mut first_record = record(first, &first_result, first_seconds);
    first_record.returned = true;
    let plausible = first == Method::Primitives || plan.share >= PLAUSIBLE_SHARE;
    if first_result.verdict == Verdict::Pass || !plausible {
        return Ok(ToSolidReport { result: first_result, attempts: vec![first_record] });
    }
    // a cancelled call does not start another attempt (the first one has ended; its result is dropped)
    if s.cancel.is_cancelled() {
        return Err(cancelled());
    }
    let refusal = skip_reason(plan, &first_result).or_else(|| (!plan.budget.draw()).then(|| plan.budget.refusal()));
    if let Some(reason) = refusal {
        let skipped = skipped_attempt(plan, second, reason);
        return Ok(ToSolidReport { result: first_result, attempts: vec![first_record, skipped] });
    }
    let after_first = std::mem::replace(&mut s.doc, base);
    let timer = Timer::start();
    let second_outcome = exec(s, second);
    // a call cancelled while the second attempt ran ends in the cancellation, whatever it returned
    if s.cancel.is_cancelled() {
        return Err(cancelled());
    }
    let second_result = match second_outcome {
        Ok(r) => r,
        Err(e) if is_cancelled(&e) => return Err(e),
        Err(e) => {
            // a second method that errors is no better than the first result: keep that, say why
            s.doc = after_first;
            let mut failed = skipped_attempt(plan, second, String::new());
            failed.skipped = None;
            failed.error = Some(short(message_of(&e)));
            failed.seconds = timer.seconds();
            return Ok(ToSolidReport { result: first_result, attempts: vec![first_record, failed] });
        }
    };
    let mut second_record = record(second, &second_result, timer.seconds());
    let second_wins = is_better(&second_result, &first_result);
    if !second_wins {
        s.doc = after_first;
    }
    first_record.returned = !second_wins;
    second_record.returned = second_wins;
    let result = if second_wins {
        finish(second_result, first, &first_result)
    } else {
        finish(first_result, second, &second_result)
    };
    Ok(ToSolidReport { result, attempts: vec![first_record, second_record] })
}

fn single(result: ToSolidOut, method: Method, seconds: f64) -> ToSolidReport {
    let mut record = record(method, &result, seconds);
    record.returned = true;
    ToSolidReport { result, attempts: vec![record] }
}

/// One method alone, timed.
fn alone(
    s: &mut Session,
    method: Method,
    exec: &mut dyn FnMut(&mut Session, Method) -> CmdResult<ToSolidOut>,
) -> CmdResult<ToSolidReport> {
    let timer = Timer::start();
    let result = exec(s, method)?;
    if s.cancel.is_cancelled() {
        return Err(cancelled());
    }
    Ok(single(result, method, timer.seconds()))
}

/// The noise issue, with the tolerance that would work: three times the estimated sigma.
fn noise_issue(report: &ScanSummary, scan: ObjectId, name: &str, result: &mut ToSolidOut) {
    let sigma = report.noise_used.unwrap_or(0.0);
    // three times the noise, rounded down to 0.001 so that it never exceeds it
    let tolerance = ((3.0 * sigma * 1000.0).floor() / 1000.0).min(3.0 * sigma);
    result.issues.push(Issue {
        code: "noise_above_tolerance".into(),
        message: format!(
            "the scan noise (sigma {sigma:.3} mm) is above the tolerance {:.3} mm: no primitive can be judged and the regions were not merged",
            result.tolerance
        ),
    });
    result.next_steps.insert(
        0,
        step(
            "scan.to_solid",
            json!({"scan": scan.0, "tolerance": tolerance, "name": name}),
            "a tolerance of about three times the scan noise, which the scan can meet",
        ),
    );
}

/// `scan.to_solid` after cleaning: the given method once, or `auto`'s ordered attempts.
pub(super) fn run(s: &mut Session, i: &ToSolidIn, set: &Settings, mut report: ScanSummary) -> CmdResult<ToSolidReport> {
    let mut exec_report = report.clone();
    if i.method == Method::Freeform {
        let mut exec = |s: &mut Session, m: Method| attempt(s, i, set, &exec_report, m);
        return alone(s, Method::Freeform, &mut exec);
    }
    if let Some(noise) = segment_scan(s, i, set)? {
        report.noise_used = noise.noise_used;
        report.noise_source = noise.noise_source;
        report.noise_spread = noise.noise_spread;
        report.noise_above_tolerance = noise.noise_above_tolerance;
        exec_report = report.clone();
    }
    let mut exec = |s: &mut Session, m: Method| attempt(s, i, set, &exec_report, m);
    let mut out = if i.method == Method::Primitives || i.reuse_regions {
        alone(s, Method::Primitives, &mut exec)?
    } else {
        let plan = Plan {
            scan: set.scan,
            tolerance: set.tolerance,
            share: scan_primitive_share(s, set, &report),
            triangles: report.triangles,
            retry: i.retry.unwrap_or(true),
            limits: Limits::default(),
            budget: set.budget.clone(),
        };
        orchestrate(s, &plan, &mut exec)?
    };
    if report.noise_above_tolerance && out.result.verdict != Verdict::Pass {
        noise_issue(&report, set.scan, &set.name, &mut out.result);
    }
    Ok(out)
}

/// One method on the document.
fn attempt(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    report: &ScanSummary,
    method: Method,
) -> CmdResult<ToSolidOut> {
    match method {
        Method::Freeform => freeform::freeform_solid(s, set, report.clone()),
        _ => primitives_solid(s, i, set, report.clone()),
    }
}

#[cfg(test)]
mod tests {
    use super::super::{DeviationSummary, ScanSummary};
    use super::*;
    use crate::cmd::body::BodyOut;
    use surfacecraft_doc::ObjectId;

    /// A result for the comparison: `body` is `Some(Some(volume))` for a closed solid, `Some(None)`
    /// for a sheet, `None` for no body; `within` of the measured points, `coverage` measured.
    fn result(verdict: Verdict, body: Option<Option<f64>>, within: f64, coverage: f64) -> ToSolidOut {
        ToSolidOut {
            verdict,
            method: Method::Primitives,
            scan: ObjectId(1),
            body: body.map(|volume| BodyOut { id: ObjectId(2), faces: 6, edges: 12, vertices: 8, volume }),
            patches: None,
            retry: None,
            tolerance: 0.2,
            deviation: Some(DeviationSummary {
                count: 1000,
                coverage,
                not_facing: 0.0,
                within,
                rms: 0.05,
                max_abs: 0.3,
                mean: 0.0,
            }),
            worst_faces: Vec::new(),
            scan_report: ScanSummary {
                triangles_before: 0,
                triangles: 0,
                holes_before: 0,
                holes_filled: 0,
                holes_left: 0,
                non_manifold_edges: 0,
                noise_sigma: None,
                noise_used: None,
                noise_source: None,
                noise_spread: None,
                noise_above_tolerance: false,
                findings: Vec::new(),
            },
            modelled: Vec::new(),
            left_out: Vec::new(),
            left_out_regions: 0,
            left_out_triangles: 0,
            unassigned_triangles: 0,
            relations: 0,
            issues: Vec::new(),
            next_steps: Vec::new(),
        }
    }

    const SOLID: Option<Option<f64>> = Some(Some(1000.0));
    const SHEET: Option<Option<f64>> = Some(None);

    /// A: both fail; one measures 3 % of the scan (all of it within), the other 95 % (90 % within):
    /// the one that describes the scan wins.
    #[test]
    fn the_result_that_explains_more_of_the_scan_wins_a_tie_of_verdict() {
        let almost_nothing = result(Verdict::Fail, SOLID, 1.0, 0.03);
        let most_of_it = result(Verdict::Fail, SOLID, 0.9, 0.95);
        assert!(is_better(&most_of_it, &almost_nothing));
        assert!(!is_better(&almost_nothing, &most_of_it));
    }

    /// B: an open sheet at 0.995 does not beat a closed solid at 0.990 of the same verdict: the
    /// difference is inside the band, so the closed solid decides.
    #[test]
    fn a_closed_solid_beats_a_sheet_within_the_band() {
        let sheet = result(Verdict::Review, SHEET, 0.995, 1.0);
        let solid = result(Verdict::Review, SOLID, 0.990, 1.0);
        assert!(is_better(&solid, &sheet));
        assert!(!is_better(&sheet, &solid));
        // outside the band the share decides
        let far = result(Verdict::Review, SHEET, 0.90, 1.0);
        assert!(is_better(&far, &result(Verdict::Review, SOLID, 0.80, 1.0)));
    }

    #[test]
    fn a_missing_body_ranks_last_within_its_verdict() {
        let none = result(Verdict::Fail, None, 0.0, 0.0);
        let sheet = result(Verdict::Fail, SHEET, 0.0, 0.0);
        assert!(is_better(&sheet, &none) && !is_better(&none, &sheet));
        // but a better verdict wins whatever the body
        assert!(is_better(&result(Verdict::Review, None, 0.0, 0.0), &result(Verdict::Fail, SOLID, 1.0, 1.0)));
    }

    #[test]
    fn an_exact_tie_keeps_the_first_and_the_order_is_never_both_ways() {
        let a = result(Verdict::Review, SOLID, 0.95, 0.98);
        let b = result(Verdict::Review, SOLID, 0.95, 0.98);
        assert!(!is_better(&a, &b) && !is_better(&b, &a));
        let verdicts = [Verdict::Pass, Verdict::Review, Verdict::Fail];
        let bodies = [SOLID, SHEET, None];
        let shares = [(0.5, 1.0), (0.99, 0.5), (0.99, 0.502), (1.0, 1.0)];
        let mut all: Vec<ToSolidOut> = Vec::new();
        for &v in &verdicts {
            for &b in &bodies {
                for &(w, c) in &shares {
                    all.push(result(v, b, w, c));
                }
            }
        }
        for x in &all {
            for y in &all {
                assert!(!(is_better(x, y) && is_better(y, x)));
            }
        }
    }

    // --- the attempts ---------------------------------------------------------------------------

    use crate::cmd::cancelled;
    use std::collections::HashMap;
    use surfacecraft_geom::{Point3, Vec3, synth};

    type Outcomes = HashMap<&'static str, CmdResult<ToSolidOut>>;

    fn plan(share: f64) -> Plan {
        Plan {
            scan: ObjectId(1),
            tolerance: 0.2,
            share,
            triangles: 1000,
            retry: true,
            limits: Limits::default(),
            budget: CallBudget::default(),
        }
    }

    /// Runs `orchestrate` with a fake that leaves a mesh named after the method in the document and
    /// returns the outcome given for it.
    fn run_fake(plan: &Plan, mut outcomes: Outcomes) -> (Session, CmdResult<ToSolidReport>, Vec<&'static str>) {
        let mut s = Session::new();
        let mut called = Vec::new();
        let report = orchestrate(&mut s, plan, &mut |s, m| {
            called.push(method_name(m));
            s.doc.add_mesh(
                &format!("m-{}", method_name(m)),
                synth::cuboid(Point3::origin(), Vec3::new(1.0, 1.0, 1.0), 1),
            );
            outcomes.remove(method_name(m)).expect("an outcome for the method")
        });
        (s, report, called)
    }

    fn outcomes(primitives: ToSolidOut, freeform: ToSolidOut) -> Outcomes {
        HashMap::from([("primitives", Ok(primitives)), ("freeform", Ok(freeform))])
    }

    fn has_mesh(s: &Session, name: &str) -> bool {
        s.doc.find_by_name(name).is_some()
    }

    /// The first attempt wins after the second ran: the document is the first attempt's alone.
    #[test]
    fn when_the_first_attempt_wins_its_document_is_restored() {
        let (s, report, called) = run_fake(
            &plan(0.9),
            outcomes(result(Verdict::Fail, SOLID, 1.0, 0.5), result(Verdict::Fail, SOLID, 1.0, 0.1)),
        );
        let report = report.expect("a report");
        assert_eq!(called, ["primitives", "freeform"]);
        assert!(has_mesh(&s, "m-primitives") && !has_mesh(&s, "m-freeform"), "the losing attempt left its mark");
        assert_eq!(s.doc.meshes.len(), 1);
        assert_eq!(report.attempts.iter().map(|a| a.returned).collect::<Vec<_>>(), [true, false]);
    }

    #[test]
    fn when_the_second_attempt_wins_only_its_document_remains() {
        let (s, report, _) = run_fake(
            &plan(0.9),
            outcomes(result(Verdict::Fail, SOLID, 1.0, 0.1), result(Verdict::Pass, SOLID, 1.0, 1.0)),
        );
        let report = report.expect("a report");
        assert_eq!(report.result.verdict, Verdict::Pass);
        assert!(has_mesh(&s, "m-freeform") && !has_mesh(&s, "m-primitives"));
        assert_eq!(report.attempts.iter().map(|a| a.returned).collect::<Vec<_>>(), [false, true]);
    }

    /// A second attempt that errors: the first result stays, with its document, and the attempt keeps
    /// the message and the time; a cancellation is re-raised.
    #[test]
    fn a_second_attempt_that_errors_keeps_the_first_result_and_the_message() {
        let long = "x".repeat(500);
        let (s, report, _) = run_fake(
            &plan(0.9),
            HashMap::from([
                ("primitives", Ok(result(Verdict::Fail, SOLID, 1.0, 0.5))),
                ("freeform", Err(CmdError::Failed(long.clone()))),
            ]),
        );
        let report = report.expect("the first result is returned");
        assert_eq!(report.result.verdict, Verdict::Fail);
        assert!(has_mesh(&s, "m-primitives") && !has_mesh(&s, "m-freeform"));
        let failed = &report.attempts[1];
        assert!(failed.verdict.is_none() && failed.seconds >= 0.0);
        assert_eq!(failed.error.as_deref().map(str::len), Some(ERROR_TEXT_LIMIT));
        assert!(report.attempts[0].returned && !failed.returned);
    }

    #[test]
    fn a_cancellation_of_the_second_attempt_is_not_swallowed() {
        let (_, report, _) = run_fake(
            &plan(0.9),
            HashMap::from([("primitives", Ok(result(Verdict::Fail, SOLID, 1.0, 0.5))), ("freeform", Err(cancelled()))]),
        );
        assert!(matches!(report, Err(e) if is_cancelled(&e)));
    }

    /// Between a quarter and a half of the scan in primitive regions: freeform first, and when it
    /// does not pass the primitives are tried; below a quarter they are not.
    #[test]
    fn the_order_is_freeform_then_primitives_for_a_share_between_a_quarter_and_a_half() {
        let both = || outcomes(result(Verdict::Pass, SOLID, 1.0, 1.0), result(Verdict::Fail, SHEET, 0.5, 0.5));
        let (_, report, called) = run_fake(&plan(0.3), both());
        let report = report.expect("a report");
        assert_eq!(called, ["freeform", "primitives"]);
        assert_eq!(report.result.verdict, Verdict::Pass);
        let (_, report, called) = run_fake(&plan(0.1), both());
        assert_eq!(called, ["freeform"]);
        assert_eq!(report.expect("a report").attempts.len(), 1);
    }

    #[test]
    fn a_pass_with_the_first_method_is_one_attempt() {
        let (_, report, called) = run_fake(
            &plan(0.9),
            outcomes(result(Verdict::Pass, SOLID, 1.0, 1.0), result(Verdict::Pass, SOLID, 1.0, 1.0)),
        );
        assert_eq!(called, ["primitives"]);
        assert_eq!(report.expect("a report").attempts.len(), 1);
    }

    /// `retry: false`, a scan over the triangle limit and a first result with many patches skip the
    /// second attempt, and say how to force it. The decision is a function of the input and the first
    /// result: nothing here depends on time.
    #[test]
    fn the_second_attempt_is_bounded_and_the_skip_names_the_call_that_forces_it() {
        let few = || Limits { max_triangles: 10, max_patches: 30_000 };
        let heavy = || Limits { max_triangles: 600_000, max_patches: 100 };
        for (reason, p, patches) in [
            ("`retry` is false", Plan { retry: false, ..plan(0.9) }, None),
            ("triangles", Plan { limits: few(), ..plan(0.9) }, None),
            ("patches", Plan { limits: heavy(), ..plan(0.9) }, Some(500)),
        ] {
            let mut first = result(Verdict::Fail, SOLID, 1.0, 0.5);
            first.patches = patches;
            let (_, report, called) = run_fake(&p, outcomes(first, result(Verdict::Pass, SOLID, 1.0, 1.0)));
            let report = report.expect("a report");
            assert_eq!(called, ["primitives"], "{reason}");
            let skipped = report.attempts[1].skipped.as_ref().expect("a skipped entry");
            assert!(skipped.reason.contains(reason), "{}", skipped.reason);
            assert_eq!(skipped.call.params["method"], "freeform");
            assert_eq!(skipped.call.command, "scan.to_solid");
            assert!(report.attempts[1].verdict.is_none() && !report.attempts[1].returned);
        }
    }

    /// The winner of a both-fail comparison does not advise the method `auto` just tried, and says
    /// the other was tried.
    #[test]
    fn next_steps_naming_a_tried_method_are_dropped_and_the_winner_says_so() {
        let mut first = result(Verdict::Fail, SOLID, 1.0, 0.5);
        first.next_steps = vec![
            step("scan.to_solid", json!({"scan": 1, "method": "freeform"}), "run again with method freeform"),
            step("body.plan_regions", json!({}), "look at the planner"),
        ];
        let (_, report, _) = run_fake(&plan(0.9), outcomes(first, result(Verdict::Fail, SOLID, 1.0, 0.1)));
        let result = report.expect("a report").result;
        assert!(result.next_steps.iter().all(|st| st.params["method"].is_null()), "{:?}", result.next_steps);
        assert_eq!(result.next_steps.len(), 1);
        assert!(result.issues.iter().any(|i| i.code == "other_method_tried" && i.message.contains("freeform")));
    }

    // --- cancellation, truncation, the suggested tolerance ----------------------------------------

    /// A call cancelled during the first attempt does not start the second.
    #[test]
    fn a_cancelled_call_does_not_start_the_second_attempt() {
        let mut s = Session::new();
        let mut called = Vec::new();
        let report = orchestrate(&mut s, &plan(0.9), &mut |s, m| {
            called.push(method_name(m));
            s.cancel.cancel();
            Ok(result(Verdict::Fail, SOLID, 1.0, 0.5))
        });
        assert!(matches!(report, Err(e) if is_cancelled(&e)));
        assert_eq!(called, ["primitives"]);
    }

    /// A call cancelled while the second attempt runs ends in the cancellation, even if that attempt
    /// returns a result.
    #[test]
    fn a_cancellation_during_the_second_attempt_ends_the_call() {
        let mut s = Session::new();
        let report = orchestrate(&mut s, &plan(0.9), &mut |s, m| {
            if m == Method::Freeform {
                s.cancel.cancel();
            }
            Ok(result(Verdict::Fail, SOLID, 1.0, 0.5))
        });
        assert!(matches!(report, Err(e) if is_cancelled(&e)));
    }

    /// An error text of multi-byte characters is cut between characters, never inside one.
    #[test]
    fn truncating_multibyte_text_never_splits_a_character() {
        for unit in ["é", "日本語", "🙂", "aé日🙂"] {
            let text = unit.repeat(300);
            let cut = short(&text);
            assert_eq!(cut.chars().count(), ERROR_TEXT_LIMIT);
            assert!(text.starts_with(&cut), "the cut is a prefix of the text");
        }
        assert_eq!(short("abc"), "abc");
    }

    /// The suggested tolerance is three times the noise, never more.
    #[test]
    fn the_suggested_tolerance_is_at_most_three_times_the_noise() {
        for sigma in [0.2004, 0.4127, 1.0, 0.2501] {
            let mut r = result(Verdict::Fail, None, 0.0, 0.0);
            let mut report = r.scan_report.clone();
            report.noise_used = Some(sigma);
            noise_issue(&report, ObjectId(1), "solid", &mut r);
            let tolerance = r.next_steps[0].params["tolerance"].as_f64().expect("a tolerance");
            assert!(tolerance <= 3.0 * sigma && tolerance > 3.0 * sigma - 0.001, "{tolerance} for {sigma}");
        }
    }

    /// Extra attempts already run inside the call (the freeform fit's retry) use up the budget the
    /// second method draws from: with two drawn there is no third, and the skip says so.
    #[test]
    fn a_spent_call_budget_skips_the_second_method() {
        let p = plan(0.9);
        assert!(p.budget.draw() && p.budget.draw(), "two extra attempts drawn by others");
        let (_, report, called) =
            run_fake(&p, outcomes(result(Verdict::Fail, SOLID, 1.0, 0.5), result(Verdict::Pass, SOLID, 1.0, 1.0)));
        let report = report.expect("a report");
        assert_eq!(called, ["primitives"]);
        let skipped = report.attempts[1].skipped.as_ref().expect("a skipped entry");
        assert!(skipped.reason.contains("extra attempts"), "{}", skipped.reason);
    }
}
