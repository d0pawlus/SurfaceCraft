//! Tests of `pipeline_auto.rs`.
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
        attempts: Vec::new(),
        prep: super::super::PrepOut::default(),
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
        s.doc.add_mesh(&format!("m-{}", method_name(m)), synth::cuboid(Point3::origin(), Vec3::new(1.0, 1.0, 1.0), 1));
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
    let (s, report, called) =
        run_fake(&plan(0.9), outcomes(result(Verdict::Fail, SOLID, 1.0, 0.5), result(Verdict::Fail, SOLID, 1.0, 0.1)));
    let report = report.expect("a report");
    assert_eq!(called, ["primitives", "freeform"]);
    assert!(has_mesh(&s, "m-primitives") && !has_mesh(&s, "m-freeform"), "the losing attempt left its mark");
    assert_eq!(s.doc.meshes.len(), 1);
    assert_eq!(report.attempts.iter().map(|a| a.returned).collect::<Vec<_>>(), [true, false]);
}

#[test]
fn when_the_second_attempt_wins_only_its_document_remains() {
    let (s, report, _) =
        run_fake(&plan(0.9), outcomes(result(Verdict::Fail, SOLID, 1.0, 0.1), result(Verdict::Pass, SOLID, 1.0, 1.0)));
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
    let (_, report, called) =
        run_fake(&plan(0.9), outcomes(result(Verdict::Pass, SOLID, 1.0, 1.0), result(Verdict::Pass, SOLID, 1.0, 1.0)));
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

/// Only a freeform second method is an extra attempt: a primitives second method makes no wrap, so
/// it runs when the budget is spent and leaves the budget alone for a later part's retry.
#[test]
fn a_primitives_second_method_neither_needs_nor_takes_the_budget() {
    // Arrange: a freeform-first plan (share 0.3), the budget spent by others, then a fresh budget
    let spent = plan(0.3);
    assert!(spent.budget.draw() && spent.budget.draw());
    let fresh = plan(0.3);

    for (p, left) in [(&spent, 0usize), (&fresh, 2)] {
        // Act
        let (_, report, called) =
            run_fake(p, outcomes(result(Verdict::Pass, SOLID, 1.0, 1.0), result(Verdict::Fail, SOLID, 1.0, 0.5)));

        // Assert
        let report = report.expect("a report");
        assert_eq!(called, ["freeform", "primitives"], "the cost-free method runs");
        assert!(report.attempts[1].skipped.is_none());
        let drawn = (0..3).take_while(|_| p.budget.draw()).count();
        assert_eq!(drawn, left, "the budget is untouched by the primitives attempt");
    }
}

/// A freeform second method (primitives first) does draw: with one extra left it takes it.
#[test]
fn a_freeform_second_method_draws_one_extra_attempt() {
    let p = plan(0.9);
    let (_, report, called) =
        run_fake(&p, outcomes(result(Verdict::Fail, SOLID, 1.0, 0.5), result(Verdict::Pass, SOLID, 1.0, 1.0)));
    assert!(report.is_ok());
    assert_eq!(called, ["primitives", "freeform"]);
    assert!(p.budget.draw() && !p.budget.draw(), "one of the two was drawn");
}
