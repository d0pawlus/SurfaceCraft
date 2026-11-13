//! `scan.to_solid` with `method: auto`: the choice of method is by outcome. The share of the scan in
//! regions a primitive fits only orders the attempts; when the first does not end in a pass the
//! other is run too (unless the caller forbids it, or the scan is too large for a second run to be
//! worth its time), and the better result is returned. The losing attempt leaves nothing in the
//! document, and the whole run is still one undo step.
use super::super::{CANCELLED, CmdError, cancelled};
use super::budget::CallBudget;
use super::{
    CmdResult, Issue, MIN_MODELLED_SHARE, Method, NextStep, ScanSummary, Settings, ToSolidIn, ToSolidOut, Verdict,
    freeform, hybrid_method, message_of, primitives_solid, scan_primitive_share, segment_scan, step,
};
use crate::Session;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use surfacecraft_doc::{Document, ObjectId};

/// The share of the scan in primitive regions below which a primitives attempt is not tried after a
/// failed freeform one (half the share that makes it the first choice).
const PLAUSIBLE_SHARE: f64 = MIN_MODELLED_SHARE / 2.0;
/// The share of the scan in primitive regions from which `auto` also tries the hybrid method: the
/// same quarter as for the primitives attempt after a failed freeform one. Below it the primitives
/// are too few to be worth a second wrap.
const HYBRID_MIN_SHARE: f64 = PLAUSIBLE_SHARE;
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

/// The result of the method returned, and the attempts (merged into `ToSolidOut` by `solve`).
pub(super) struct ToSolidReport {
    pub result: ToSolidOut,
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

/// Share of the body's area on analytic faces: all of it for a primitives solid, what the result
/// reports for a hybrid one, none for a patch network.
fn analytic_share(r: &ToSolidOut) -> f64 {
    match r.method {
        Method::Primitives if r.body.is_some() => 1.0,
        _ => r.analytic.as_ref().map_or(0.0, |a| a.area_share),
    }
}

/// Orders results: by verdict; then by the share of the scan explained, shares within
/// [`EXPLAINED_BAND`] being equal; then a closed solid over a sheet over no body; then, when all of
/// that is a tie, the one with more of its area on analytic faces (a hybrid body is worth more than
/// a network of patches that explains the scan equally well). Antisymmetric.
pub(super) fn compare(a: &ToSolidOut, b: &ToSolidOut) -> std::cmp::Ordering {
    let by_verdict = rank(a.verdict).cmp(&rank(b.verdict));
    let (ea, eb) = (explained(a), explained(b));
    let by_share = if (ea - eb).abs() > EXPLAINED_BAND + 1e-9 { ea.total_cmp(&eb) } else { std::cmp::Ordering::Equal };
    let (xa, xb) = (analytic_share(a), analytic_share(b));
    let by_analytic =
        if (xa - xb).abs() > EXPLAINED_BAND + 1e-9 { xa.total_cmp(&xb) } else { std::cmp::Ordering::Equal };
    by_verdict.then(by_share).then(closedness(a).cmp(&closedness(b))).then(by_analytic)
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
    /// The primitives cover enough of the scan for `auto` to offer, or after a failure to try, the
    /// hybrid method.
    pub hybrid: bool,
    /// Regions a primitive fits (for the hint of the hybrid method).
    pub primitive_regions: usize,
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
    match m {
        Method::Freeform => "freeform",
        Method::Hybrid => "hybrid",
        Method::Primitives | Method::Auto => "primitives",
    }
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
        st.command != "scan.to_solid"
            || !matches!(st.params["method"].as_str(), Some("primitives" | "freeform" | "hybrid"))
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

/// The first attempt of `auto`, with its method and record.
struct First {
    method: Method,
    result: ToSolidOut,
    record: Attempt,
}

/// The methods in the order `auto` tries them: primitives first when at least half of the scan lies
/// in regions a primitive fits.
fn order(plan: &Plan) -> (Method, Method) {
    if plan.share >= MIN_MODELLED_SHARE {
        (Method::Primitives, Method::Freeform)
    } else {
        (Method::Freeform, Method::Primitives)
    }
}

/// The reason the second method is not run, if there is one. Only a method that wraps (freeform,
/// hybrid) is an extra attempt of the call's budget (the primitives method makes no wrap and costs
/// nothing), so only those draw from it.
fn refusal(plan: &Plan, first: &ToSolidOut, second: Method) -> Option<String> {
    skip_reason(plan, first)
        .or_else(|| (second != Method::Primitives && !plan.budget.draw()).then(|| plan.budget.refusal()))
}

/// Runs the attempts of `auto` with `exec` (one method on the document) and returns the best: the
/// two methods of [`order`], then, when the best is no pass and the primitives cover enough of the
/// scan, the hybrid method as a third attempt (it wins a tie of the others by its analytic faces).
/// A result that is a pass is never swapped for a hybrid body and costs no hybrid attempt (it was
/// 4 x slower and not better on the real scans); a next step names the call that would add one.
pub(super) fn orchestrate(
    s: &mut Session,
    plan: &Plan,
    exec: &mut dyn FnMut(&mut Session, Method) -> CmdResult<ToSolidOut>,
) -> CmdResult<ToSolidReport> {
    let base = s.doc.clone();
    let report = two_methods(s, plan, exec, base.clone())?;
    if !plan.hybrid {
        return Ok(report);
    }
    if report.result.verdict == Verdict::Pass {
        return Ok(offer_hybrid(plan, report));
    }
    hybrid_attempt(s, plan, exec, base, report)
}

/// A passing result with enough primitives in the scan gets the next step that builds the hybrid body
/// (a passing primitives solid is analytic all over and gets none).
fn offer_hybrid(plan: &Plan, mut report: ToSolidReport) -> ToSolidReport {
    if report.result.method == Method::Primitives {
        return report;
    }
    let why = format!(
        "would add analytic faces (plane, cylinder, cone, sphere) on up to {} primitive regions covering {:.0} % of the scan, with patches on the rest (not run: it is slower and is not tried after a pass)",
        plan.primitive_regions,
        100.0 * plan.share
    );
    let params = json!({"scan": plan.scan.0, "method": "hybrid", "tolerance": plan.tolerance});
    report.result.next_steps.push(step("scan.to_solid", params, &why));
    report
}

/// The first method and, when it does not pass, the second (see [`orchestrate`]).
fn two_methods(
    s: &mut Session,
    plan: &Plan,
    exec: &mut dyn FnMut(&mut Session, Method) -> CmdResult<ToSolidOut>,
    base: Document,
) -> CmdResult<ToSolidReport> {
    let (method, second) = order(plan);
    let timer = Timer::start();
    let result = exec(s, method)?;
    let mut record = record(method, &result, timer.seconds());
    record.returned = true;
    let plausible = method == Method::Primitives || plan.share >= PLAUSIBLE_SHARE;
    if result.verdict == Verdict::Pass || !plausible {
        return Ok(ToSolidReport { result, attempts: vec![record] });
    }
    // a cancelled call does not start another attempt (the first one has ended; its result is dropped)
    if s.cancel.is_cancelled() {
        return Err(cancelled());
    }
    if let Some(reason) = refusal(plan, &result, second) {
        let skipped = skipped_attempt(plan, second, reason);
        return Ok(ToSolidReport { result, attempts: vec![record, skipped] });
    }
    second_attempt(s, plan, exec, base, First { method, result, record }, second)
}

/// Runs the hybrid method on the document as it was before the first attempt and keeps it when it is
/// better than the result so far (a tie goes to the one with more analytic area). The hybrid attempt
/// draws from the call's budget and is skipped by the same bounds as a second method; one that ends
/// in an error leaves the result so far as it is.
fn hybrid_attempt(
    s: &mut Session,
    plan: &Plan,
    exec: &mut dyn FnMut(&mut Session, Method) -> CmdResult<ToSolidOut>,
    base: Document,
    report: ToSolidReport,
) -> CmdResult<ToSolidReport> {
    let ToSolidReport { result: best, mut attempts } = report;
    if s.cancel.is_cancelled() {
        return Err(cancelled());
    }
    if let Some(reason) = refusal(plan, &best, Method::Hybrid) {
        attempts.push(skipped_attempt(plan, Method::Hybrid, reason));
        return Ok(ToSolidReport { result: best, attempts });
    }
    let after_best = std::mem::replace(&mut s.doc, base);
    let timer = Timer::start();
    let outcome = exec(s, Method::Hybrid);
    if s.cancel.is_cancelled() {
        return Err(cancelled());
    }
    let hybrid = match outcome {
        Ok(r) => r,
        Err(e) if is_cancelled(&e) => return Err(e),
        Err(e) => {
            s.doc = after_best;
            let mut failed = skipped_attempt(plan, Method::Hybrid, String::new());
            failed.skipped = None;
            failed.error = Some(short(message_of(&e)));
            failed.seconds = timer.seconds();
            attempts.push(failed);
            return Ok(ToSolidReport { result: best, attempts });
        }
    };
    let mut hybrid_record = record(Method::Hybrid, &hybrid, timer.seconds());
    let wins = is_better(&hybrid, &best);
    if wins {
        attempts.iter_mut().for_each(|a| a.returned = false);
    } else {
        s.doc = after_best;
    }
    hybrid_record.returned = wins;
    attempts.push(hybrid_record);
    let result = if wins { finish(hybrid, best.method, &best) } else { finish(best, Method::Hybrid, &hybrid) };
    Ok(ToSolidReport { result, attempts })
}

/// Runs the second method on the document as it was before the first, and keeps the better result.
fn second_attempt(
    s: &mut Session,
    plan: &Plan,
    exec: &mut dyn FnMut(&mut Session, Method) -> CmdResult<ToSolidOut>,
    base: Document,
    first: First,
    second: Method,
) -> CmdResult<ToSolidReport> {
    let First { method, result: first_result, record: mut first_record } = first;
    let after_first = std::mem::replace(&mut s.doc, base);
    let timer = Timer::start();
    let outcome = exec(s, second);
    // a call cancelled while the second attempt ran ends in the cancellation, whatever it returned
    if s.cancel.is_cancelled() {
        return Err(cancelled());
    }
    let second_result = match outcome {
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
        finish(second_result, method, &first_result)
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
    let mut out = if i.method == Method::Hybrid {
        alone(s, Method::Hybrid, &mut exec)?
    } else if i.method == Method::Primitives || i.reuse_regions {
        alone(s, Method::Primitives, &mut exec)?
    } else {
        let (share, primitive_regions) = scan_primitive_share(s, set, &report);
        let plan = Plan {
            scan: set.scan,
            tolerance: set.tolerance,
            share,
            triangles: report.triangles,
            retry: i.retry.unwrap_or(true),
            limits: Limits::default(),
            budget: set.budget.clone(),
            hybrid: share >= HYBRID_MIN_SHARE,
            primitive_regions,
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
        Method::Hybrid => hybrid_method::hybrid_solid(s, i, set, report.clone()),
        Method::Primitives | Method::Auto => primitives_solid(s, i, set, report.clone()),
    }
}

#[cfg(test)]
#[path = "pipeline_auto_tests.rs"]
mod tests;
