//! The result of a split `scan.to_solid` run (P9-O9): one verdict, the issues and next steps of the
//! whole scan, built from the result of each modelled part and from what was left out.
//!
//! The verdict is the worst co-equal part's. Small parts never lower it below `review`: a small part
//! that does not pass counts, with the pieces left out and the parts beyond the cap, as surface that
//! was not modelled to a pass, and when that is more than [`UNMODELLED_REVIEW_SHARE`] of the scan the
//! verdict is at most `review`. The output stays bounded whatever the number of parts: the issues of
//! small parts are summed up in one issue, their next steps are at most [`MAX_SMALL_PART_STEPS`].
use super::parts::LeftOutReason;
use super::prepare::{ComponentOut, PrepOut, PrepStep};
use super::{DeviationSummary, Issue, NextStep, ScanSummary, Settings, ToSolidOut, Verdict, issue};
use crate::cmd::body::BodyOut;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use surfacecraft_doc::ObjectId;

/// Surface not modelled to a pass above this share of the scan caps the verdict at `review`.
pub(super) const UNMODELLED_REVIEW_SHARE: f64 = 0.02;
/// Next steps taken from small parts that did not pass, at most.
const MAX_SMALL_PART_STEPS: usize = 3;
/// Small parts named in the issue about those that did not pass, at most.
const MAX_NAMED_PARTS: usize = 5;
/// Pieces left out listed one by one in the result (the largest), at most.
pub(super) const MAX_LISTED_PIECES: usize = 16;

/// A piece of the scan that was not modelled.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct PieceOut {
    /// Why it was not modelled.
    pub reason: LeftOutReason,
    /// Its triangles.
    pub triangles: usize,
    /// Its surface area (mm2).
    pub area: f64,
    /// Its share of the surface of the scan (0..1).
    pub share: f64,
    /// Lower corner of its bounding box.
    pub bbox_min: [f64; 3],
    /// Upper corner of its bounding box.
    pub bbox_max: [f64; 3],
}

/// What was left unmodelled, as `(pieces, share of the area)`.
pub(super) struct Tally {
    /// Pieces that are no part (every reason).
    pub left_out: (usize, f64),
    /// The same by reason.
    pub by_reason: Vec<(LeftOutReason, usize, f64)>,
    pub cavities: (usize, f64),
    pub beyond_cap: (usize, f64),
    pub not_requested: (usize, f64),
    pub modelled: usize,
    /// How many of the modelled parts are co-equal.
    pub co_equal: usize,
    pub debris_pieces: usize,
    /// The largest pieces left out, one by one.
    pub listed: Vec<PieceOut>,
    /// Total surface area of the scan (mm2).
    pub total_area: f64,
    pub ms: u64,
}

fn rank(v: Verdict) -> u8 {
    match v {
        Verdict::Pass => 2,
        Verdict::Review => 1,
        Verdict::Fail => 0,
    }
}

fn worst(a: Verdict, b: Verdict) -> Verdict {
    if rank(a) <= rank(b) { a } else { b }
}

/// Small parts that did not pass: how many and their share of the surface of the scan.
fn small_not_passed(parts: &[(ComponentOut, ToSolidOut)], tally: &Tally) -> (usize, f64) {
    let missed: Vec<&ComponentOut> =
        parts.iter().map(|p| &p.0).filter(|c| !c.co_equal && c.verdict != Verdict::Pass).collect();
    (missed.len(), missed.iter().map(|c| c.area).sum::<f64>() / tally.total_area.max(1e-12))
}

/// The issues of the co-equal parts, each named by its part when there are several.
fn co_equal_issues(parts: &[(ComponentOut, ToSolidOut)]) -> Vec<Issue> {
    let several = parts.len() > 1;
    let mut issues = Vec::new();
    for (n, (c, _)) in parts.iter().enumerate().filter(|(_, (c, _))| c.co_equal) {
        for x in &c.issues {
            let message =
                if several { format!("part {} ({}): {}", n + 1, c.name, x.message) } else { x.message.clone() };
            issues.push(Issue { code: x.code.clone(), message });
        }
    }
    issues
}

/// "3 open, 2 debris": the reasons of the pieces left out.
fn reasons_text(tally: &Tally) -> String {
    let words: Vec<String> = tally.by_reason.iter().map(|(r, n, _)| format!("{n} {}", r.word())).collect();
    words.join(", ")
}

fn left_out_issue(tally: &Tally) -> Issue {
    let msg = format!(
        "{} pieces ({} of them debris), {:.1} % of the surface of the scan, were not modelled ({}): a piece is modelled when it is co-equal (at least `part_share` of the largest part) or a closed, consistently oriented solid of at least {} triangles and {} tolerances across; lower `part_share` to model them with the rest, or run `scan.split` and model a piece on its own with `scan.to_solid`",
        tally.left_out.0,
        tally.debris_pieces,
        100.0 * tally.left_out.1,
        reasons_text(tally),
        super::parts::MIN_PART_TRIANGLES,
        super::parts::MIN_PART_SPAN,
    );
    issue("unmodelled_pieces", msg)
}

fn small_parts_issue(parts: &[(ComponentOut, ToSolidOut)], (count, share): (usize, f64)) -> Issue {
    let named: Vec<String> = parts
        .iter()
        .filter(|(c, _)| !c.co_equal && c.verdict != Verdict::Pass)
        .take(MAX_NAMED_PARTS)
        .map(|(c, _)| format!("{} ({:?})", c.name, c.verdict).to_lowercase())
        .collect();
    let msg = format!(
        "{count} small parts, {:.1} % of the surface of the scan, were modelled without a pass (among them {}); each has its verdict and issues in `components`",
        100.0 * share,
        named.join(", ")
    );
    issue("parts_not_passed", msg)
}

/// The issues of the parts (named by part when there are several) and of what was not modelled to
/// a pass; whether the verdict must come down to `review` for them.
pub(super) fn collect_issues(parts: &[(ComponentOut, ToSolidOut)], tally: &Tally) -> (Vec<Issue>, bool) {
    let mut issues = co_equal_issues(parts);
    let missed = small_not_passed(parts, tally);
    let unmodelled = tally.left_out.1 + missed.1 + tally.beyond_cap.1;
    let over = unmodelled > UNMODELLED_REVIEW_SHARE;
    if over && tally.left_out.0 > 0 {
        issues.push(left_out_issue(tally));
    }
    if over && missed.0 > 0 {
        issues.push(small_parts_issue(parts, missed));
    }
    let mut cap = over;
    if tally.cavities.1 > UNMODELLED_REVIEW_SHARE {
        let msg = format!(
            "{} inner shells (cavity skins of hollow parts, {:.1} % of the surface) were not modelled: the solids are the outer shells; `scan.split` makes a mesh of each shell if a cavity has to be modelled on its own",
            tally.cavities.0,
            100.0 * tally.cavities.1
        );
        issues.push(issue("nested_shell_skipped", msg));
        cap = true;
    }
    if tally.beyond_cap.0 > 0 {
        let msg = format!(
            "{} more parts ({:.1} % of the surface) were not modelled: at most {} are per call, the co-equal ones and then the largest; run `scan.split` and model the others on their own (`scan.to_solid` on each mesh), or raise `part_share` to leave the small ones out",
            tally.beyond_cap.0,
            100.0 * tally.beyond_cap.1,
            super::parts::MAX_COMPONENTS
        );
        issues.push(issue("parts_beyond_limit", msg));
        cap = true;
    }
    (issues, cap)
}

/// The steps of the parts that did not pass, each labelled with its part: all co-equal ones (three
/// steps each), and the first [`MAX_SMALL_PART_STEPS`] steps of small parts.
fn steps_of_parts_not_passed(parts: &[(ComponentOut, ToSolidOut)]) -> Vec<NextStep> {
    let mut next = Vec::new();
    let mut small = 0;
    for (n, (c, o)) in parts.iter().enumerate().filter(|(_, (c, _))| c.verdict != Verdict::Pass) {
        let take = if c.co_equal { 3 } else { MAX_SMALL_PART_STEPS.saturating_sub(small).min(1) };
        small += usize::from(!c.co_equal && take > 0);
        for st in o.next_steps.iter().take(take) {
            let why = format!("part {} ({}): {}", n + 1, c.name, st.why);
            next.push(NextStep { why, ..st.clone() });
        }
    }
    next
}

/// The next steps of a split result: every part passed and nothing left out: one export of all the
/// solids; else the steps of the parts that did not pass, an export of the solids that did, and (when
/// pieces were left out) the split that makes them meshes of their own.
pub(super) fn next_steps_of(parts: &[(ComponentOut, ToSolidOut)], scan: ObjectId, left_out: bool) -> Vec<NextStep> {
    let passed: Vec<u64> = parts
        .iter()
        .filter(|(c, _)| c.verdict == Verdict::Pass)
        .filter_map(|(c, _)| c.body.as_ref().map(|b| b.id.0))
        .collect();
    let export =
        |ids: &[u64], why: String| NextStep { command: "export.step".into(), params: json!({"objects": ids}), why };
    let mut next = Vec::new();
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
        next.extend(steps_of_parts_not_passed(parts));
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

/// The deviation over all modelled parts: shares and RMS weighted by the area of the part, the largest maximum.
pub(super) fn aggregate(parts: &[(ComponentOut, ToSolidOut)]) -> Option<DeviationSummary> {
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

/// The `split` step of `preparation`: what was modelled and what was not, with the figures.
fn split_step(parts: &[(ComponentOut, ToSolidOut)], tally: &Tally) -> PrepStep {
    let missed = small_not_passed(parts, tally);
    let by_reason: serde_json::Map<String, serde_json::Value> =
        tally.by_reason.iter().map(|(r, n, a)| (r.word().to_string(), json!({"pieces": n, "area_share": a}))).collect();
    let reason = format!(
        "the scan holds {} parts, modelled one by one ({} co-equal, {} small)",
        tally.modelled,
        tally.co_equal,
        tally.modelled.saturating_sub(tally.co_equal)
    );
    let numbers = json!({
        "modelled": tally.modelled, "co_equal": tally.co_equal, "small_parts": tally.modelled.saturating_sub(tally.co_equal),
        "small_parts_not_passed": missed.0, "small_parts_not_passed_area_share": missed.1,
        "minor_and_debris_pieces": tally.left_out.0, "minor_and_debris_area_share": tally.left_out.1,
        "left_out_by_reason": by_reason, "debris_pieces": tally.debris_pieces,
        "cavity_shells": tally.cavities.0, "cavity_area_share": tally.cavities.1,
        "beyond_limit": tally.beyond_cap.0, "beyond_limit_area_share": tally.beyond_cap.1,
        "not_requested": tally.not_requested.0, "not_requested_area_share": tally.not_requested.1,
        "unmodelled_area_share": tally.left_out.1 + missed.1 + tally.beyond_cap.1,
        "limit": super::parts::MAX_COMPONENTS, "ms": tally.ms,
    });
    PrepStep { step: "split".into(), reason, numbers }
}

/// One result for the whole scan: the worst co-equal part's verdict (at most `review` with surface not
/// modelled to a pass), the largest part's body, the aggregate deviation, every body in `bodies`.
pub(super) fn combine(
    set: &Settings,
    report: ScanSummary,
    parts: Vec<(ComponentOut, ToSolidOut)>,
    tally: Tally,
) -> ToSolidOut {
    let (issues, capped) = collect_issues(&parts, &tally);
    let mut verdict = parts.iter().filter(|c| c.0.co_equal).fold(Verdict::Pass, |v, c| worst(v, c.0.verdict));
    if capped && verdict == Verdict::Pass {
        verdict = Verdict::Review;
    }
    let deviation = aggregate(&parts);
    let next = next_steps_of(&parts, set.scan, capped);
    let largest = parts.iter().enumerate().max_by(|a, b| a.1.0.area.total_cmp(&b.1.0.area)).map_or(0, |(k, _)| k);
    let bodies: Vec<BodyOut> = parts.iter().filter_map(|(c, _)| c.body.clone()).collect();
    let step = split_step(&parts, &tally);
    let comps: Vec<ComponentOut> = parts.iter().map(|p| p.0.clone()).collect();
    let fallback = || super::prepare::failure(set, set.scan, "no part");
    let mut out = parts.into_iter().nth(largest).map_or_else(fallback, |p| p.1);
    out.verdict = verdict;
    out.scan = set.scan;
    out.scan_report = report;
    out.deviation = deviation;
    out.issues = issues;
    out.next_steps = next;
    out.prep = PrepOut { preparation: vec![step], components: comps, bodies, pieces_left_out: tally.listed };
    out
}
