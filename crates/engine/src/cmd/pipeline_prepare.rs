//! Preparation of messy scans for `scan.to_solid` (P9-O9), part one: a scan that holds several parts
//! is split, and every part that counts is modelled on its own. A scan that already works takes
//! exactly the path it took before: no preparation, an empty `preparation` list.
//!
//! What counts: a piece of the cleaned scan with at least `min_share` of the triangles or the area
//! is a part (`scan.split`); a part with at least `part_share` (default 10 %) of the area of the
//! largest piece is *co-equal*. Pieces between the two are minor, pieces below `min_share` are
//! debris; neither is modelled, and when their area is more than [`UNMODELLED_REVIEW_SHARE`] of the
//! scan the verdict is at most `review` and an issue says so.
use super::freeform::RetryReason;
use super::rebuild::Stopwatch;
use super::{
    CmdError, CmdResult, DeviationSummary, Issue, Method, NextStep, ScanSummary, Settings, ToSolidIn, ToSolidOut,
    Verdict, issue, message_of, prepare, settings, solve,
};
use crate::Session;
use crate::cmd::body::BodyOut;
use crate::cmd::scan_split::{self, Analysis, MIN_SHARE};
use crate::cmd::{ObjRef, cancelled, is_cancelled};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;

/// Parts modelled in one run at most (the largest); the rest are reported.
pub(super) const MAX_COMPONENTS: usize = 24;
/// A part is co-equal when its area is at least this share of the largest piece's.
pub(super) const PART_SHARE: f64 = 0.1;
/// Unmodelled area above this share of the scan caps the verdict at `review`.
const UNMODELLED_REVIEW_SHARE: f64 = 0.02;
/// Parts (largest first) that may be rebuilt and wrapped again in one call.
const REBUILD_PARTS: usize = 4;

/// Which co-equal parts of a scan are modelled.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum Components {
    /// `all` (the largest 24) or `largest`.
    Word(String),
    /// The number of largest parts.
    Count(usize),
}

impl Components {
    /// How many parts were asked for: `Some` when the caller chose a number (or `largest`).
    fn requested(c: &Option<Components>) -> CmdResult<Option<usize>> {
        match c {
            None => Ok(None),
            Some(Components::Word(w)) if w == "all" => Ok(None),
            Some(Components::Word(w)) if w == "largest" => Ok(Some(1)),
            Some(Components::Count(n)) if *n >= 1 => Ok(Some((*n).min(MAX_COMPONENTS))),
            Some(_) => Err(CmdError::Params(
                "components must be \"largest\", \"all\" or a number of parts (at least 1)".into(),
            )),
        }
    }
}

/// One preparation step.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct PrepStep {
    /// `split`, `remesh` or `freeform_fallback`.
    pub step: String,
    /// Why it was done (or not possible).
    pub reason: String,
    /// The figures behind it, with `ms` (time spent) where it did work.
    pub numbers: Value,
}

/// What preparation adds to the result of `scan.to_solid`.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
pub struct PrepOut {
    /// What was done to the scan before modelling it (`split`, `remesh`) and why; empty when the scan was modelled as it is.
    #[serde(default)]
    pub preparation: Vec<PrepStep>,
    /// One result per modelled part when the scan was split (the fields above then describe the
    /// largest part, `deviation` the area-weighted aggregate, `verdict` the worst part's); empty otherwise.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<ComponentOut>,
    /// The solids of all modelled parts, largest first (empty unless the scan was split).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bodies: Vec<BodyOut>,
}

/// The result for one part of a scan.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ComponentOut {
    /// The mesh of the part (made by the split).
    pub mesh: ObjectId,
    /// Its name.
    pub name: String,
    /// Its triangles.
    pub triangles: usize,
    /// Its surface area (mm2).
    pub area: f64,
    /// How its run ended.
    pub verdict: Verdict,
    /// The method used.
    pub method: Method,
    /// Its solid.
    pub body: Option<BodyOut>,
    /// Patches (freeform).
    pub patches: Option<usize>,
    /// The solid against the part.
    pub deviation: Option<DeviationSummary>,
    /// What kept it from a pass.
    pub issues: Vec<Issue>,
    /// What was done to the part first.
    pub preparation: Vec<PrepStep>,
    /// The methods tried for this part (the top level of a split scan lists the largest part's).
    #[serde(default)]
    pub attempts: Vec<super::auto::Attempt>,
    /// The larger-cap retry of this part, or the note that the call's budget was spent.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub retry: Option<super::freeform::CapRetry>,
}

pub(super) fn rank(v: Verdict) -> u8 {
    match v {
        Verdict::Pass => 2,
        Verdict::Review => 1,
        Verdict::Fail => 0,
    }
}

fn worst(a: Verdict, b: Verdict) -> Verdict {
    if rank(a) <= rank(b) { a } else { b }
}

pub(super) fn check_cancel(s: &Session) -> CmdResult<()> {
    if s.cancel.is_cancelled() { Err(cancelled()) } else { Ok(()) }
}

/// The part share of the call: a share of the largest piece's area, `0..=1`.
fn part_share_of(i: &ToSolidIn) -> CmdResult<f64> {
    let v = i.part_share.unwrap_or(PART_SHARE);
    if v.is_finite() && (0.0..=1.0).contains(&v) {
        Ok(v)
    } else {
        Err(CmdError::Params("part_share must be within 0..=1".into()))
    }
}

/// The pieces of the scan by what becomes of them.
struct Plan {
    /// Co-equal parts to model, largest area first.
    chosen: Vec<usize>,
    /// Co-equal parts left out because the caller asked for fewer.
    not_requested: Vec<usize>,
    /// Co-equal parts left out by the cap of [`MAX_COMPONENTS`].
    beyond_cap: Vec<usize>,
    /// Parts that are neither co-equal nor the inner skin of one.
    minor: Vec<usize>,
    /// Inner skins of hollow parts.
    cavities: Vec<usize>,
}

fn plan_of(a: &Analysis, part_share: f64, requested: Option<usize>) -> Plan {
    let area = |k: &usize| a.parts.get(*k).map_or(0.0, |p| p.area);
    let distinct = a.distinct();
    // the yardstick is the largest piece that is not the inner skin of another: a cavity skin may have
    // far more area than the shell around it and must not turn that shell into a "minor" piece
    let largest = distinct.iter().map(area).fold(0.0, f64::max);
    let cavities: Vec<usize> = (0..a.parts.len()).filter(|k| !distinct.contains(k)).collect();
    let (mut equal, minor): (Vec<usize>, Vec<usize>) =
        distinct.iter().copied().partition(|k| area(k) >= part_share * largest);
    equal.sort_by(|x, y| area(y).total_cmp(&area(x)).then(x.cmp(y)));
    if equal.is_empty() {
        // never a result without a body: fall back to the largest outer piece (or the largest piece)
        let pool = if distinct.is_empty() { (0..a.parts.len()).collect() } else { distinct };
        equal.extend(pool.into_iter().max_by(|x, y| area(x).total_cmp(&area(y))));
    }
    let take = requested.unwrap_or(MAX_COMPONENTS).min(MAX_COMPONENTS);
    let rest = equal.split_off(take.min(equal.len()));
    let (not_requested, beyond_cap) = if requested.is_some() { (rest, Vec::new()) } else { (Vec::new(), rest) };
    let minor = minor.into_iter().filter(|k| !equal.contains(k)).collect();
    Plan { chosen: equal, not_requested, beyond_cap, minor, cavities }
}

/// What was left out, from the plan: pieces and share of the surface of the scan.
fn tally_of(a: &Analysis, plan: &Plan, ms: u64) -> Tally {
    let all_area: f64 = a.parts.iter().chain(&a.extra).chain(&a.rest).map(|p| p.area).sum::<f64>().max(1e-12);
    let area_of =
        |ks: &[usize]| ks.iter().filter_map(|k| a.parts.get(*k)).map(|p| p.area).sum::<f64>() / all_area + 0.0;
    let (rest_area, extra_area) =
        (a.rest.iter().map(|p| p.area).sum::<f64>() / all_area, a.extra.iter().map(|p| p.area).sum::<f64>() / all_area);
    Tally {
        minor: (plan.minor.len() + a.rest.len(), area_of(&plan.minor) + rest_area),
        cavities: (plan.cavities.len(), area_of(&plan.cavities)),
        beyond_cap: (plan.beyond_cap.len() + a.extra.len(), area_of(&plan.beyond_cap) + extra_area),
        not_requested: (plan.not_requested.len(), area_of(&plan.not_requested)),
        modelled: plan.chosen.len(),
        debris_pieces: a.rest.len(),
        ms,
    }
}

/// Number of co-equal parts of `mesh` (those a split run would model; 1 for a single piece).
pub(super) fn coequal_parts(mesh: &surfacecraft_geom::Mesh) -> usize {
    let a = scan_split::analyse(mesh, MIN_SHARE, true);
    if a.parts.len() < 2 { a.parts.len() } else { plan_of(&a, PART_SHARE, None).chosen.len() }
}

/// Splits a scan with several parts and models each co-equal one. `None` for a scan with one piece
/// that counts (the usual case: nothing changes).
pub(super) fn split_parts(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    report: &ScanSummary,
) -> CmdResult<Option<ToSolidOut>> {
    let requested = Components::requested(&i.components)?;
    let part_share = part_share_of(i)?;
    if i.reuse_regions {
        return Ok(None);
    }
    let Some(mesh) = s.doc.mesh(set.scan).map(|m| m.mesh.clone()) else { return Ok(None) };
    let watch = Stopwatch::start();
    let a = scan_split::analyse(&mesh, MIN_SHARE, true);
    if a.parts.len() < 2 {
        return Ok(None);
    }
    let plan = plan_of(&a, part_share, requested);
    let base = s.doc.mesh(set.scan).map_or_else(|| "scan".to_string(), |m| m.name.clone());
    let total_tris = mesh.triangle_count() as f64;
    let mut done: Vec<(ComponentOut, ToSolidOut)> = Vec::new();
    for (n, &k) in plan.chosen.iter().enumerate() {
        check_cancel(s)?;
        let Some(piece) = a.parts.get(k) else { continue };
        let part = scan_split::add_part(s, &base, k, piece, &mesh, total_tris);
        let allow = (i.rebuild && n < REBUILD_PARTS, retry_allowed(i.allow_retry, &done));
        done.push(run_part(s, i, set, (&part.name, part.id, piece.area), n + 1, allow)?);
        #[cfg(test)]
        after_part_hook(n);
    }
    check_cancel(s)?;
    let tally = tally_of(&a, &plan, watch.ms());
    Ok(Some(combine(set, report.clone(), done, &tally)))
}

/// Test hook: called after each part of a split run (index from 0), so that a test can cancel at a
/// chosen moment without relying on time.
#[cfg(test)]
static AFTER_PART: std::sync::Mutex<Option<Box<dyn Fn(usize) + Send>>> = std::sync::Mutex::new(None);

#[cfg(test)]
fn after_part_hook(n: usize) {
    if let Ok(g) = AFTER_PART.lock()
        && let Some(f) = g.as_ref()
    {
        f(n);
    }
}

/// What was left unmodelled, as `(pieces, share of the area)`.
struct Tally {
    minor: (usize, f64),
    cavities: (usize, f64),
    beyond_cap: (usize, f64),
    not_requested: (usize, f64),
    modelled: usize,
    debris_pieces: usize,
    ms: u64,
}

/// The larger-cap retry is made once per call: for the first part, in area order, whose first result
/// qualifies for it (capped by the patch limit and short of the tolerance, which is the retry's own test).
/// The parts after it do not get it; a part that does not qualify leaves it for the next.
pub(super) fn retry_allowed(allow: bool, done: &[(ComponentOut, ToSolidOut)]) -> bool {
    allow && done.iter().all(|(_, o)| o.retry.as_ref().is_none_or(|r| r.reason == RetryReason::BudgetSpent))
}

/// Models one part on its own (`(name, mesh, area)`); a cancelled run is an error, any other failure a failed part.
fn run_part(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    part: (&str, ObjectId, f64),
    n: usize,
    (allow_rebuild, allow_retry): (bool, bool),
) -> CmdResult<(ComponentOut, ToSolidOut)> {
    let (name, id, area) = part;
    let part_in = ToSolidIn {
        scan: ObjRef::Id(id.0),
        clean: false,
        name: Some(format!("{}-{n}", set.name)),
        components: None,
        rebuild: allow_rebuild,
        allow_retry,
        ..i.clone()
    };
    let triangles = s.doc.mesh(id).map_or(0, |m| m.mesh.triangle_count());
    let mut out = model_part(s, &part_in, set, id)?;
    if needs_freeform(i.method, &out) {
        // the primitives found no closed solid in this part: model it as freeform instead
        let watch = Stopwatch::start();
        let again = model_part(s, &ToSolidIn { method: Method::Freeform, ..part_in.clone() }, set, id)?;
        if again.body.is_some() {
            let step = PrepStep {
                step: "freeform_fallback".into(),
                reason: "the primitives built no solid for this part, so it was wrapped in patches".into(),
                numbers: json!({"primitive_issues": out.issues.iter().map(|x| x.code.clone()).collect::<Vec<_>>(), "ms": watch.ms()}),
            };
            // the attempts that came before stay in the list (none of them is the returned one now)
            let mut tried = std::mem::take(&mut out.attempts);
            tried.iter_mut().for_each(|a| a.returned = false);
            out = again;
            tried.append(&mut out.attempts);
            out.attempts = tried;
            out.prep.preparation.insert(0, step);
        }
    }
    let comp = ComponentOut {
        mesh: id,
        name: name.to_string(),
        triangles,
        area,
        verdict: out.verdict,
        method: out.method,
        body: out.body.clone(),
        patches: out.patches,
        deviation: out.deviation.clone(),
        issues: out.issues.clone(),
        preparation: out.prep.preparation.clone(),
        attempts: out.attempts.clone(),
        retry: out.retry.clone(),
    };
    Ok((comp, out))
}

/// Did the primitives (not a freeform wrap) leave this part without a solid, and has no freeform wrap
/// been run for it yet (`auto` may have run one as its second method)? Then the part gets its one wrap.
fn needs_freeform(method: Method, out: &ToSolidOut) -> bool {
    let wrapped_already = out.attempts.iter().any(|a| a.method == Method::Freeform && a.skipped.is_none());
    method == Method::Auto
        && out.verdict == Verdict::Fail
        && out.body.is_none()
        && out.method != Method::Freeform
        && !wrapped_already
}

/// Runs `run` on the document; a failure other than a cancellation takes back everything the run
/// added (a rebuilt mesh, a body, regions) and is returned as the inner `Err`; a cancellation is the
/// outer one and leaves the document to the caller's own clean-up.
pub(super) fn restoring_on_failure<T>(
    s: &mut Session,
    run: impl FnOnce(&mut Session) -> CmdResult<T>,
) -> CmdResult<Result<T, CmdError>> {
    let before = s.doc.clone();
    match run(s) {
        Ok(v) => Ok(Ok(v)),
        Err(e) if is_cancelled(&e) => Err(e),
        Err(e) => {
            s.doc = before;
            Ok(Err(e))
        }
    }
}

/// One run of the single-scan pipeline on a part; an error other than a cancellation becomes a failed
/// result and leaves nothing of the run in the document.
fn model_part(s: &mut Session, part_in: &ToSolidIn, set: &Settings, id: ObjectId) -> CmdResult<ToSolidOut> {
    // the extra attempts of all parts of a call draw from one budget (the call's own)
    let run = |s: &mut Session| {
        let pset = Settings { budget: set.budget.clone(), ..settings(s, part_in)? };
        let scan = pset.scan;
        let report = prepare(s, part_in, scan)?;
        solve(s, part_in, pset, report)
    };
    Ok(restoring_on_failure(s, run)?.unwrap_or_else(|e| failure(set, id, message_of(&e))))
}

#[cfg(test)]
pub(super) fn component_for_tests(out: &ToSolidOut) -> ComponentOut {
    ComponentOut {
        mesh: out.scan,
        name: String::new(),
        triangles: 0,
        area: 0.0,
        verdict: out.verdict,
        method: out.method,
        body: out.body.clone(),
        patches: out.patches,
        deviation: out.deviation.clone(),
        issues: out.issues.clone(),
        preparation: Vec::new(),
        attempts: out.attempts.clone(),
        retry: out.retry.clone(),
    }
}

#[cfg(test)]
pub(super) fn failure_for_tests(set: &Settings, id: ObjectId) -> ToSolidOut {
    failure(set, id, "test")
}

fn failure(set: &Settings, id: ObjectId, why: &str) -> ToSolidOut {
    let report = ScanSummary {
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
    };
    ToSolidOut {
        verdict: Verdict::Fail,
        method: Method::Freeform,
        scan: id,
        body: None,
        patches: None,
        retry: None,
        tolerance: set.tolerance,
        deviation: None,
        worst_faces: Vec::new(),
        scan_report: report,
        modelled: Vec::new(),
        left_out: Vec::new(),
        left_out_regions: 0,
        left_out_triangles: 0,
        unassigned_triangles: 0,
        relations: 0,
        issues: vec![issue("no_solid", format!("the part could not be modelled: {why}"))],
        next_steps: Vec::new(),
        attempts: Vec::new(),
        prep: PrepOut::default(),
        thickened: None,
        analytic: None,
    }
}

/// The deviation over all modelled parts: shares and RMS weighted by the area of the part, the largest maximum.
fn aggregate(parts: &[(ComponentOut, ToSolidOut)]) -> Option<DeviationSummary> {
    let with: Vec<(&ComponentOut, &DeviationSummary)> =
        parts.iter().filter_map(|(c, _)| c.deviation.as_ref().map(|d| (c, d))).collect();
    let total: f64 = with.iter().map(|(c, _)| c.area).sum();
    if with.is_empty() || total <= 0.0 {
        return with.first().map(|(_, d)| (*d).clone());
    }
    let w = |f: &dyn Fn(&DeviationSummary) -> f64| with.iter().map(|(c, d)| c.area * f(d)).sum::<f64>() / total;
    Some(DeviationSummary {
        count: with.iter().map(|(_, d)| d.count).sum(),
        coverage: w(&|d| d.coverage),
        not_facing: w(&|d| d.not_facing),
        within: w(&|d| d.within),
        rms: w(&|d| d.rms * d.rms).sqrt(),
        max_abs: with.iter().map(|(_, d)| d.max_abs).fold(0.0, f64::max),
        mean: w(&|d| d.mean),
    })
}

/// The issues of the parts (named by part when there are several) and of what was left unmodelled;
/// whether the verdict must come down to `review` for them.
fn collect_issues(parts: &[(ComponentOut, ToSolidOut)], tally: &Tally) -> (Vec<Issue>, bool) {
    let mut issues: Vec<Issue> = Vec::new();
    let several = parts.len() > 1;
    for (n, (c, _)) in parts.iter().enumerate() {
        for x in &c.issues {
            let message =
                if several { format!("part {} ({}): {}", n + 1, c.name, x.message) } else { x.message.clone() };
            issues.push(Issue { code: x.code.clone(), message });
        }
    }
    let pct = |share: f64| 100.0 * share;
    let mut cap = false;
    if tally.minor.1 > UNMODELLED_REVIEW_SHARE {
        let msg = format!(
            "{} pieces ({} of them debris), {:.1} % of the surface of the scan, were not modelled (each is below `part_share` of the largest part); lower `part_share` to model them with the rest, or run `scan.split` and model a piece on its own with `scan.to_solid`",
            tally.minor.0,
            tally.debris_pieces,
            pct(tally.minor.1)
        );
        issues.push(issue("unmodelled_pieces", msg));
        cap = true;
    }
    if tally.cavities.1 > UNMODELLED_REVIEW_SHARE {
        let msg = format!(
            "{} inner shells (cavity skins of hollow parts, {:.1} % of the surface) were not modelled: the solids are the outer shells; `scan.split` makes a mesh of each shell if a cavity has to be modelled on its own",
            tally.cavities.0,
            pct(tally.cavities.1)
        );
        issues.push(issue("nested_shell_skipped", msg));
        cap = true;
    }
    if tally.beyond_cap.0 > 0 {
        let msg = format!(
            "{} more parts ({:.1} % of the surface) were not modelled: at most {MAX_COMPONENTS} are per call, the largest first; run `scan.split` and model the others on their own (`scan.to_solid` on each mesh), or raise `part_share` to leave the small ones out",
            tally.beyond_cap.0,
            pct(tally.beyond_cap.1)
        );
        issues.push(issue("parts_beyond_limit", msg));
        cap = true;
    }
    (issues, cap)
}

/// The next steps of a split result: every part passed and nothing left out: one export of all the
/// solids; else the steps of each part that did not pass, each labelled with its part, an export of
/// the solids that did, and (when pieces were left out) the split that makes them meshes of their own.
fn next_steps_of(parts: &[(ComponentOut, ToSolidOut)], scan: ObjectId, left_out: bool) -> Vec<NextStep> {
    let passed: Vec<u64> = parts
        .iter()
        .filter(|(c, _)| c.verdict == Verdict::Pass)
        .filter_map(|(c, _)| c.body.as_ref().map(|b| b.id.0))
        .collect();
    let mut next = Vec::new();
    let export =
        |ids: &[u64], why: String| NextStep { command: "export.step".into(), params: json!({"objects": ids}), why };
    if passed.len() == parts.len() {
        let why = if left_out {
            format!(
                "the {} solids that were modelled passed (see the issues for what was left out): export them",
                passed.len()
            )
        } else {
            "every part passed: export all the solids".to_string()
        };
        next.push(export(&passed, why));
    } else {
        for (n, (c, o)) in parts.iter().enumerate().filter(|(_, (c, _))| c.verdict != Verdict::Pass) {
            for st in o.next_steps.iter().take(3) {
                let why = format!("part {} ({}): {}", n + 1, c.name, st.why);
                next.push(NextStep { why, ..st.clone() });
            }
        }
        if !passed.is_empty() {
            next.push(export(&passed, format!("export the {} parts that passed", passed.len())));
        }
    }
    if left_out {
        let why = "makes a mesh of every piece, so that what was left out can be modelled on its own";
        next.push(NextStep {
            command: "scan.split".into(),
            params: json!({"object": scan.0, "keep_debris": true}),
            why: why.into(),
        });
    }
    next
}

/// One result for the whole scan: the worst co-equal part's verdict (at most `review` with unmodelled
/// surface), the largest part's body, the aggregate deviation, every body in `bodies`.
fn combine(set: &Settings, report: ScanSummary, parts: Vec<(ComponentOut, ToSolidOut)>, tally: &Tally) -> ToSolidOut {
    let (issues, capped) = collect_issues(&parts, tally);
    let mut verdict = parts.iter().fold(Verdict::Pass, |v, c| worst(v, c.0.verdict));
    if capped && verdict == Verdict::Pass {
        verdict = Verdict::Review;
    }
    let deviation = aggregate(&parts);
    let next = next_steps_of(&parts, set.scan, capped);
    let largest = parts.iter().enumerate().max_by(|a, b| a.1.0.area.total_cmp(&b.1.0.area)).map_or(0, |(k, _)| k);
    let bodies: Vec<BodyOut> = parts.iter().filter_map(|(c, _)| c.body.clone()).collect();
    let step = PrepStep {
        step: "split".into(),
        reason: format!("the scan holds {} co-equal parts, modelled one by one", tally.modelled),
        numbers: json!({
            "modelled": tally.modelled, "minor_and_debris_pieces": tally.minor.0, "minor_and_debris_area_share": tally.minor.1,
            "debris_pieces": tally.debris_pieces, "cavity_shells": tally.cavities.0, "cavity_area_share": tally.cavities.1,
            "beyond_limit": tally.beyond_cap.0, "beyond_limit_area_share": tally.beyond_cap.1,
            "not_requested": tally.not_requested.0, "not_requested_area_share": tally.not_requested.1,
            "limit": MAX_COMPONENTS, "ms": tally.ms,
        }),
    };
    let comps: Vec<ComponentOut> = parts.iter().map(|p| p.0.clone()).collect();
    let mut out = parts.into_iter().nth(largest).map_or_else(|| failure(set, set.scan, "no part"), |p| p.1);
    out.verdict = verdict;
    out.scan = set.scan;
    out.scan_report = report;
    out.deviation = deviation;
    out.issues = issues;
    out.next_steps = next;
    out.prep = PrepOut { preparation: vec![step], components: comps, bodies };
    out
}

#[cfg(test)]
#[path = "pipeline_prepare_tests.rs"]
mod tests;
