//! Preparation of messy scans for `scan.to_solid` (P9-O9), part one: a scan that holds several parts
//! is split, and every part that counts is modelled on its own. A scan that already works takes
//! exactly the path it took before: no preparation, an empty `preparation` list.
//!
//! What counts (`pipeline_parts`): a co-equal part (at least `part_share` of the area of the largest
//! piece) and a small part (a closed, consistently oriented solid that is no debris) are modelled;
//! every other piece is left out with its reason. The result of the whole scan is put together in
//! `pipeline_parts_report`.
use super::freeform::RetryReason;
use super::parts::{self, LeftOutReason, MAX_COMPONENTS, Pieces, Plan};
use super::parts_report::{PieceOut, Tally, combine};
use super::rebuild::Stopwatch;
use super::{
    CmdError, CmdResult, DeviationSummary, Issue, Method, ScanSummary, Settings, ToSolidIn, ToSolidOut, Verdict, issue,
    message_of, prepare, settings, solve,
};
use crate::Session;
use crate::cmd::body::BodyOut;
use crate::cmd::scan_split::{self, MIN_SHARE};
use crate::cmd::{ObjRef, cancelled, is_cancelled};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};
use surfacecraft_doc::ObjectId;

pub(super) use super::parts::{PART_SHARE, coequal_parts};
use super::parts_report::rank;

/// Co-equal parts (largest first) that may be rebuilt and wrapped again in one call. Small parts are
/// never rebuilt: on the scan of many parts the rebuild of a watertight non-manifold small part took
/// 400 s and was discarded (the plain wrap passed in 6 s), or was refused.
const REBUILD_PARTS: usize = 4;

/// Which co-equal parts of a scan are modelled.
#[derive(Clone, Debug, PartialEq, Eq, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum Components {
    /// `all` (every co-equal and small part, at most 128) or `largest`.
    Word(String),
    /// The number of largest co-equal parts (no small parts then).
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
    /// One result per modelled part when the scan was split, co-equal parts first (the fields above then
    /// describe the largest part, `deviation` the area-weighted aggregate, `verdict` the worst co-equal
    /// part's, at most `review` when small parts did not pass or pieces were left out); empty otherwise.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub components: Vec<ComponentOut>,
    /// The solids of all modelled parts, in the order of `components` (empty unless the scan was split).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bodies: Vec<BodyOut>,
    /// The largest pieces that were not modelled (at most 16), with the reason; the counts of all of
    /// them are in the `split` step of `preparation`.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub pieces_left_out: Vec<PieceOut>,
}

/// The result for one part of a scan.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ComponentOut {
    /// The mesh of the part (made by the split).
    pub mesh: ObjectId,
    /// Its name.
    pub name: String,
    /// Co-equal (its verdict is the call's) or a small part (a solid below `part_share` of the largest).
    #[serde(default = "crate::cmd::deviation::yes")]
    pub co_equal: bool,
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

/// `(pieces, share of the surface)` of the pieces `ks`.
fn share_of(p: &Pieces, ks: &[usize]) -> (usize, f64) {
    (ks.len(), ks.iter().map(|k| p.area(*k)).sum::<f64>() / p.total_area() + 0.0)
}

/// The pieces left out, by reason, in the order of the reasons.
fn by_reason(p: &Pieces, plan: &Plan) -> Vec<(LeftOutReason, usize, f64)> {
    let mut reasons: Vec<LeftOutReason> = plan.left_out.iter().map(|(_, r)| *r).collect();
    reasons.sort();
    reasons.dedup();
    reasons
        .into_iter()
        .map(|r| {
            let ks: Vec<usize> = plan.left_out.iter().filter(|(_, x)| *x == r).map(|(k, _)| *k).collect();
            let (n, a) = share_of(p, &ks);
            (r, n, a)
        })
        .collect()
}

/// The largest pieces left out, one by one.
fn listed_pieces(p: &Pieces, plan: &Plan) -> Vec<PieceOut> {
    let mut left = plan.left_out.clone();
    left.sort_by(|x, y| p.area(y.0).total_cmp(&p.area(x.0)).then(x.0.cmp(&y.0)));
    let total = p.total_area();
    left.iter()
        .take(super::parts_report::MAX_LISTED_PIECES)
        .filter_map(|(k, r)| p.all.get(*k).map(|q| (q, *r)))
        .map(|(q, reason)| PieceOut {
            reason,
            triangles: q.triangles.len(),
            area: q.area,
            share: q.area / total,
            bbox_min: [q.bbox.min.x, q.bbox.min.y, q.bbox.min.z],
            bbox_max: [q.bbox.max.x, q.bbox.max.y, q.bbox.max.z],
        })
        .collect()
}

/// What was left out, from the plan: pieces and share of the surface of the scan.
fn tally_of(p: &Pieces, plan: &Plan, ms: u64) -> Tally {
    let left: Vec<usize> = plan.left_out.iter().map(|(k, _)| *k).collect();
    Tally {
        left_out: share_of(p, &left),
        by_reason: by_reason(p, plan),
        cavities: share_of(p, &plan.cavities),
        beyond_cap: share_of(p, &plan.beyond_cap),
        not_requested: share_of(p, &plan.not_requested),
        modelled: plan.chosen.len(),
        co_equal: plan.co_equal,
        debris_pieces: plan.left_out.iter().filter(|(_, r)| *r == LeftOutReason::Debris).count(),
        listed: listed_pieces(p, plan),
        total_area: p.total_area(),
        ms,
    }
}

/// Splits a scan with several parts and models each co-equal and small part. `None` for a scan with
/// one piece that counts (the usual case: nothing changes).
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
    let listed = a.parts.len();
    let pieces = Pieces::of(&mesh, a);
    let plan = parts::plan_of(&pieces, (part_share, set.tolerance), requested);
    if listed < 2 && !parts::worth_splitting(&plan) {
        return Ok(None);
    }
    let base = s.doc.mesh(set.scan).map_or_else(|| "scan".to_string(), |m| m.name.clone());
    let total_tris = mesh.triangle_count() as f64;
    let mut done: Vec<(ComponentOut, ToSolidOut)> = Vec::new();
    for (n, &k) in plan.chosen.iter().enumerate() {
        check_cancel(s)?;
        let Some(piece) = pieces.all.get(k) else { continue };
        let part = scan_split::add_part(s, &base, k, piece, &mesh, total_tris);
        let co_equal = n < plan.co_equal;
        let allow = (i.rebuild && co_equal && n < REBUILD_PARTS, retry_allowed(i.allow_retry, &done));
        let mut one = run_part(s, i, set, (&part.name, part.id, piece.area), n + 1, allow)?;
        one.0.co_equal = co_equal;
        done.push(one);
        #[cfg(test)]
        after_part_hook(n);
    }
    check_cancel(s)?;
    let tally = tally_of(&pieces, &plan, watch.ms());
    Ok(Some(combine(set, report.clone(), done, tally)))
}

/// Test hook: called after each part of a split run (index from 0), so that a test can cancel at a
/// chosen moment without relying on time.
#[cfg(test)]
static AFTER_PART: std::sync::Mutex<Option<Box<dyn Fn(usize) + Send>>> = std::sync::Mutex::new(None);

/// Test hook: the part of this name ends in an error inside its run (as a part whose pipeline fails).
#[cfg(test)]
static FAIL_NAME: std::sync::Mutex<Option<String>> = std::sync::Mutex::new(None);

#[cfg(test)]
fn fail_hook(part_in: &ToSolidIn) -> CmdResult<()> {
    let hit = FAIL_NAME.lock().ok().is_some_and(|g| g.is_some() && *g == part_in.name);
    if hit { Err(CmdError::Failed("the test made this part fail".into())) } else { Ok(()) }
}

#[cfg(test)]
fn after_part_hook(n: usize) {
    if let Ok(g) = AFTER_PART.lock()
        && let Some(f) = g.as_ref()
    {
        f(n);
    }
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
    let before = object_ids(s);
    let mut out = model_part(s, &part_in, set, id)?;
    if needs_freeform(i.method, &out, &set.budget.refusal()) {
        let first = added_since(s, &before);
        out = freeform_fallback(s, &part_in, set, (id, &first), out)?;
    }
    let comp = ComponentOut {
        mesh: id,
        name: name.to_string(),
        co_equal: true,
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

/// Should this part get its one freeform wrap after the primitives? When they left it without a solid,
/// or when they did not pass and `auto`'s freeform attempt was refused because the call's extra
/// attempts were spent (`refusal`): a part's own wrap is no extra attempt. On the scan of many parts,
/// twelve small parts ended without a pass that way; each passed as freeform on its own.
fn needs_freeform(method: Method, out: &ToSolidOut, refusal: &str) -> bool {
    let ran = |m: Method| out.attempts.iter().any(|a| a.method == m && a.skipped.is_none());
    let starved = out
        .attempts
        .iter()
        .any(|a| a.method == Method::Freeform && a.skipped.as_ref().is_some_and(|k| k.reason == refusal));
    let primitives_failed = out.verdict == Verdict::Fail && out.body.is_none();
    method == Method::Auto
        && out.method != Method::Freeform
        && !ran(Method::Freeform)
        && (primitives_failed || (starved && out.verdict != Verdict::Pass))
}

/// Does `again` explain the part better than `first`: a solid, and a better verdict or the same with
/// more of the part within the tolerance?
fn wins(again: &ToSolidOut, first: &ToSolidOut) -> bool {
    let within = |o: &ToSolidOut| o.deviation.as_ref().map_or(0.0, |d| d.within);
    let (a, f) = (rank(again.verdict), rank(first.verdict));
    again.body.is_some() && (a > f || (a == f && within(again) > within(first)))
}

/// Every object id of the document.
fn object_ids(s: &Session) -> std::collections::BTreeSet<u64> {
    s.doc.all_ids().into_iter().collect()
}

/// The objects added to the document since `before` was taken.
fn added_since(s: &Session, before: &std::collections::BTreeSet<u64>) -> Vec<ObjectId> {
    s.doc.all_ids().into_iter().filter(|x| !before.contains(x)).map(ObjectId).collect()
}

/// Removes the objects of an attempt that lost.
fn remove_all(s: &mut Session, ids: &[ObjectId]) {
    for x in ids {
        s.doc.remove(*x);
    }
}

/// The part's one freeform wrap after a failed primitives result (whose objects are `first`): the
/// better of the two is kept (its attempts after the other's), every object of the other attempt is
/// removed, and the solid kept has the part's name.
fn freeform_fallback(
    s: &mut Session,
    part_in: &ToSolidIn,
    set: &Settings,
    (id, first): (ObjectId, &[ObjectId]),
    mut out: ToSolidOut,
) -> CmdResult<ToSolidOut> {
    let watch = Stopwatch::start();
    let before = object_ids(s);
    let mut again = model_part(s, &ToSolidIn { method: Method::Freeform, ..part_in.clone() }, set, id)?;
    let second = added_since(s, &before);
    let name = part_in.name.clone().unwrap_or_else(|| set.name.clone());
    if !wins(&again, &out) {
        remove_all(s, &second);
        again.attempts.iter_mut().for_each(|a| a.returned = false);
        out.attempts.append(&mut again.attempts);
        rename_body(s, &out, &name);
        return Ok(out);
    }
    remove_all(s, first);
    rename_body(s, &again, &name);
    let step = PrepStep {
        step: "freeform_fallback".into(),
        reason: "the primitives did not model this part, so it was wrapped in patches as its one wrap".into(),
        numbers: json!({"primitive_issues": out.issues.iter().map(|x| x.code.clone()).collect::<Vec<_>>(), "ms": watch.ms()}),
    };
    // the attempts that came before stay in the list (none of them is the returned one now)
    let mut tried = std::mem::take(&mut out.attempts);
    tried.iter_mut().for_each(|a| a.returned = false);
    tried.append(&mut again.attempts);
    again.attempts = tried;
    again.prep.preparation.insert(0, step);
    Ok(again)
}

/// Gives the solid of `out` the name `name`.
fn rename_body(s: &mut Session, out: &ToSolidOut, name: &str) {
    if let Some(b) = &out.body
        && s.doc.body(b.id).is_some_and(|o| o.name != name)
    {
        s.doc.rename(b.id, name);
    }
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
        #[cfg(test)]
        fail_hook(part_in)?;
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
        co_equal: true,
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

pub(super) fn failure(set: &Settings, id: ObjectId, why: &str) -> ToSolidOut {
    let report = ScanSummary {
        triangles_before: 0,
        triangles: 0,
        holes_before: 0,
        holes_filled: 0,
        holes_left: 0,
        holes_inward: 0,
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

#[cfg(test)]
#[path = "pipeline_prepare_tests.rs"]
mod tests;
