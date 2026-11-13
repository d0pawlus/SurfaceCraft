//! Preparation of messy scans for `scan.to_solid` (P9-O9), part one: a scan that holds several parts
//! is split, and every part that counts is modelled on its own. A scan that already works takes
//! exactly the path it took before: no preparation, an empty `preparation` list.
//!
//! What counts: a piece of the cleaned scan with at least `min_share` of the triangles or the area
//! is a part (`scan.split`); a part with at least `part_share` (default 10 %) of the area of the
//! largest piece is *co-equal*. Pieces between the two are minor, pieces below `min_share` are
//! debris; neither is modelled, and when their area is more than [`UNMODELLED_REVIEW_SHARE`] of the
//! scan the verdict is at most `review` and an issue says so.
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
    let area = |k: &usize| a.parts[*k].area;
    let largest = a.parts.iter().map(|p| p.area).fold(0.0, f64::max);
    let distinct = a.distinct();
    let cavities: Vec<usize> = (0..a.parts.len()).filter(|k| !distinct.contains(k)).collect();
    let (mut equal, minor): (Vec<usize>, Vec<usize>) =
        distinct.into_iter().partition(|k| area(k) >= part_share * largest);
    equal.sort_by(|x, y| area(y).total_cmp(&area(x)).then(x.cmp(y)));
    let take = requested.unwrap_or(MAX_COMPONENTS).min(MAX_COMPONENTS);
    let rest = equal.split_off(take.min(equal.len()));
    let (not_requested, beyond_cap) = if requested.is_some() { (rest, Vec::new()) } else { (Vec::new(), rest) };
    Plan { chosen: equal, not_requested, beyond_cap, minor, cavities }
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
        let part = scan_split::add_part(s, &base, k, &a.parts[k], &mesh, total_tris);
        let allow = i.rebuild && n < REBUILD_PARTS;
        done.push(run_part(s, i, set, (&part.name, part.id, a.parts[k].area), n + 1, allow)?);
    }
    let all_area: f64 = a.parts.iter().chain(&a.rest).map(|p| p.area).sum();
    let area_of = |ks: &[usize]| ks.iter().map(|k| a.parts[*k].area).sum::<f64>() / all_area.max(1e-12);
    let rest_area = a.rest.iter().map(|p| p.area).sum::<f64>() / all_area.max(1e-12);
    let account = Account {
        minor: (plan.minor.len() + a.rest.len(), area_of(&plan.minor) + rest_area),
        cavities: (plan.cavities.len(), area_of(&plan.cavities)),
        beyond_cap: (plan.beyond_cap.len(), area_of(&plan.beyond_cap)),
        not_requested: (plan.not_requested.len(), area_of(&plan.not_requested)),
        modelled: plan.chosen.len(),
        debris_pieces: a.rest.len(),
        ms: watch.ms(),
    };
    Ok(Some(combine(set, report.clone(), done, &account)))
}

/// What was left unmodelled, as `(pieces, share of the area)`.
struct Account {
    minor: (usize, f64),
    cavities: (usize, f64),
    beyond_cap: (usize, f64),
    not_requested: (usize, f64),
    modelled: usize,
    debris_pieces: usize,
    ms: u64,
}

/// Models one part on its own (`(name, mesh, area)`); a cancelled run is an error, any other failure a failed part.
fn run_part(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    part: (&str, ObjectId, f64),
    n: usize,
    allow_rebuild: bool,
) -> CmdResult<(ComponentOut, ToSolidOut)> {
    let (name, id, area) = part;
    let part_in = ToSolidIn {
        scan: ObjRef::Id(id.0),
        clean: false,
        name: Some(format!("{}-{n}", set.name)),
        components: None,
        rebuild: allow_rebuild,
        ..i.clone()
    };
    let triangles = s.doc.mesh(id).map_or(0, |m| m.mesh.triangle_count());
    let mut out = model_part(s, &part_in, set, id)?;
    if i.method == Method::Auto && out.verdict == Verdict::Fail && out.body.is_none() {
        // the primitives found no closed solid in this part: model it as freeform instead
        let watch = Stopwatch::start();
        let again = model_part(s, &ToSolidIn { method: Method::Freeform, ..part_in.clone() }, set, id)?;
        if again.body.is_some() {
            let step = PrepStep {
                step: "freeform_fallback".into(),
                reason: "the primitives built no solid for this part, so it was wrapped in patches".into(),
                numbers: json!({"primitive_issues": out.issues.iter().map(|x| x.code.clone()).collect::<Vec<_>>(), "ms": watch.ms()}),
            };
            out = again;
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
    };
    Ok((comp, out))
}

/// One run of the single-scan pipeline on a part; an error other than a cancellation becomes a failed result.
fn model_part(s: &mut Session, part_in: &ToSolidIn, set: &Settings, id: ObjectId) -> CmdResult<ToSolidOut> {
    let run =
        settings(s, part_in).and_then(|pset| prepare(s, part_in, pset.scan).and_then(|r| solve(s, part_in, pset, r)));
    match run {
        Err(e) if is_cancelled(&e) => Err(e),
        Err(e) => Ok(failure(set, id, message_of(&e))),
        Ok(o) => Ok(o),
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
        prep: PrepOut::default(),
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
fn collect_issues(parts: &[(ComponentOut, ToSolidOut)], acct: &Account) -> (Vec<Issue>, bool) {
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
    if acct.minor.1 > UNMODELLED_REVIEW_SHARE {
        let msg = format!(
            "{} pieces ({} of them debris), {:.1} % of the surface of the scan, were not modelled (each is below the share of the largest part that counts as a part)",
            acct.minor.0,
            acct.debris_pieces,
            pct(acct.minor.1)
        );
        issues.push(issue("unmodelled_pieces", msg));
        cap = true;
    }
    if acct.cavities.1 > UNMODELLED_REVIEW_SHARE {
        let msg = format!(
            "{} inner shells (cavity skins of hollow parts, {:.1} % of the surface) were not modelled: the solids are the outer shells",
            acct.cavities.0,
            pct(acct.cavities.1)
        );
        issues.push(issue("nested_shell_skipped", msg));
        cap = true;
    }
    if acct.beyond_cap.0 > 0 {
        let msg = format!(
            "{} more parts ({:.1} % of the surface) were not modelled: at most {MAX_COMPONENTS} are, the largest first; run `scan.split` and model them by hand",
            acct.beyond_cap.0,
            pct(acct.beyond_cap.1)
        );
        issues.push(issue("parts_beyond_limit", msg));
        cap = true;
    }
    (issues, cap)
}

/// The next steps of a split result: when every part passed, one export of all the solids; else the
/// steps of each part, each labelled with its part.
fn next_steps_of(parts: &[(ComponentOut, ToSolidOut)]) -> Vec<NextStep> {
    if parts.iter().all(|(c, _)| c.verdict == Verdict::Pass) {
        let ids: Vec<u64> = parts.iter().filter_map(|(c, _)| c.body.as_ref().map(|b| b.id.0)).collect();
        let why = "every part passed: export all the solids";
        return vec![NextStep { command: "export.step".into(), params: json!({"objects": ids}), why: why.into() }];
    }
    let mut next = Vec::new();
    for (n, (c, o)) in parts.iter().enumerate() {
        for st in o.next_steps.iter().filter(|_| c.verdict != Verdict::Pass).take(3) {
            let why = format!("part {} ({}): {}", n + 1, c.name, st.why);
            next.push(NextStep { why, ..st.clone() });
        }
    }
    next
}

/// One result for the whole scan: the worst co-equal part's verdict (at most `review` with unmodelled
/// surface), the largest part's body, the aggregate deviation, every body in `bodies`.
fn combine(set: &Settings, report: ScanSummary, parts: Vec<(ComponentOut, ToSolidOut)>, acct: &Account) -> ToSolidOut {
    let (issues, capped) = collect_issues(&parts, acct);
    let mut verdict = parts.iter().fold(Verdict::Pass, |v, c| worst(v, c.0.verdict));
    if capped && verdict == Verdict::Pass {
        verdict = Verdict::Review;
    }
    let deviation = aggregate(&parts);
    let next = next_steps_of(&parts);
    let largest = parts.iter().enumerate().max_by(|a, b| a.1.0.area.total_cmp(&b.1.0.area)).map_or(0, |(k, _)| k);
    let bodies: Vec<BodyOut> = parts.iter().filter_map(|(c, _)| c.body.clone()).collect();
    let step = PrepStep {
        step: "split".into(),
        reason: format!("the scan holds {} co-equal parts, modelled one by one", acct.modelled),
        numbers: json!({
            "modelled": acct.modelled, "minor_and_debris_pieces": acct.minor.0, "minor_and_debris_area_share": acct.minor.1,
            "debris_pieces": acct.debris_pieces, "cavity_shells": acct.cavities.0, "cavity_area_share": acct.cavities.1,
            "beyond_limit": acct.beyond_cap.0, "beyond_limit_area_share": acct.beyond_cap.1,
            "not_requested": acct.not_requested.0, "not_requested_area_share": acct.not_requested.1,
            "limit": MAX_COMPONENTS, "ms": acct.ms,
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
mod tests {
    use super::*;
    use surfacecraft_geom::{Mesh, Point3, synth};

    /// `n` equal closed balls far apart.
    fn balls(n: usize) -> Mesh {
        let mut all = Mesh::default();
        for k in 0..n {
            let mut m = synth::sphere(Point3::new(30.0 * k as f64, 0.0, 0.0), 5.0, 16, 8);
            m.weld(1e-9);
            all.append(&m);
        }
        all
    }

    #[test]
    fn only_the_cap_leaves_co_equal_parts_beyond_the_limit_and_a_request_does_not() {
        let a = scan_split::analyse(&balls(26), MIN_SHARE, false);
        let capped = plan_of(&a, PART_SHARE, None);
        assert_eq!((capped.chosen.len(), capped.beyond_cap.len(), capped.not_requested.len()), (24, 2, 0));
        let asked = plan_of(&a, PART_SHARE, Some(1));
        assert_eq!((asked.chosen.len(), asked.beyond_cap.len(), asked.not_requested.len()), (1, 0, 25));
        let acct = Account {
            minor: (0, 0.0),
            cavities: (0, 0.0),
            beyond_cap: (0, 0.0),
            not_requested: (25, 0.96),
            modelled: 1,
            debris_pieces: 0,
            ms: 0,
        };
        let (issues, capped_verdict) = collect_issues(&[], &acct);
        assert!(issues.is_empty() && !capped_verdict, "a requested subset is no issue");
        let over = Account { beyond_cap: (2, 0.07), ..acct };
        let (issues, capped_verdict) = collect_issues(&[], &over);
        assert!(
            issues.iter().any(|i| i.code == "parts_beyond_limit" && i.message.contains("2 more parts"))
                && capped_verdict
        );
    }

    #[test]
    fn a_piece_below_a_tenth_of_the_largest_is_minor_and_the_components_word_is_checked() {
        let mut m = balls(1);
        let mut small = synth::sphere(Point3::new(60.0, 0.0, 0.0), 1.2, 16, 8);
        small.weld(1e-9);
        m.append(&small);
        let a = scan_split::analyse(&m, MIN_SHARE, false);
        let p = plan_of(&a, PART_SHARE, None);
        assert_eq!((p.chosen.len(), p.minor.len()), (1, 1));
        assert_eq!(Components::requested(&Some(Components::Word("largest".into()))).unwrap(), Some(1));
        assert_eq!(Components::requested(&None).unwrap(), None);
        assert!(Components::requested(&Some(Components::Word("some".into()))).is_err());
        assert!(Components::requested(&Some(Components::Count(0))).is_err());
    }
}
