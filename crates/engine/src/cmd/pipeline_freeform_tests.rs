//! Tests of `pipeline_freeform.rs`.
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
        budget: super::super::budget::CallBudget::default(),
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

/// The retry is an extra attempt of the call's budget: with the budget spent by others (the
/// second method of `auto`, a rebuild-and-wrap-again) no second wrap is made, and a retry that is
/// made uses one of the two.
#[test]
fn the_retry_draws_from_the_budget_of_the_call() {
    // Arrange
    let (mut s, set) = session();
    let first = first_attempt(&mut s, &set, 300);
    assert!(set.budget.draw() && set.budget.draw(), "the budget is spent by others");

    // Act
    let (kept, retry) = retry_larger(&mut s, &set, first.clone(), |s, cap| wrap_with(s, &set, Some(cap)));

    // Assert
    let note = retry.expect("the withheld retry is recorded");
    assert_eq!(note.reason, RetryReason::BudgetSpent, "no extra attempt without budget");
    assert!(!note.kept && note.patches_to.is_none());
    assert_eq!(kept.patches, first.patches);
    let (mut s, set) = session();
    let first = first_attempt(&mut s, &set, 300);
    let (_, retry) = retry_larger(&mut s, &set, first, |s, cap| wrap_with(s, &set, Some(cap)));
    assert!(retry.is_some());
    assert!(set.budget.draw() && !set.budget.draw(), "the retry used one of the two");
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
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
        findings: Vec::new(),
    }
}

/// A call cancelled after the first network does not build the second one.
#[test]
fn a_cancelled_call_does_not_build_the_second_network() {
    let (mut s, set) = session();
    let mut calls = 0;
    let out = freeform_with(&mut s, &set, scan_summary(), |s, cap| {
        calls += 1;
        let mut w = wrap_with(s, &set, Some(cap.unwrap_or(300)))?;
        if cap.is_none() {
            w.capped = true;
            s.cancel.cancel();
        }
        Ok(w)
    });
    assert_eq!(calls, 1);
    assert!(out.as_ref().is_err_and(crate::cmd::is_cancelled), "the cancellation ends the call");
    assert!(set.budget.draw() && set.budget.draw(), "a second wrap that never ran spent nothing of the budget");
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
        noise_used: None,
        noise_source: None,
        noise_spread: None,
        noise_above_tolerance: false,
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

/// Two parts in turn, the first fine and the second capped, then a third that is capped too: the one
/// retry of the call goes to the second and is made once; the third makes one wrap.
#[test]
fn the_one_retry_of_a_call_goes_to_the_first_part_that_qualifies() {
    let (mut s, set) = session();
    let mut done: Vec<(super::super::ComponentOut, ToSolidOut)> = Vec::new();
    let mut calls = Vec::new();
    for capped in [false, true, true] {
        let allow = super::super::prepare::retry_allowed(true, &done);
        let set = Settings { allow_retry: allow, name: set.name.clone(), budget: set.budget.clone(), ..set };
        let mut n = 0usize;
        let out = freeform_with(&mut s, &set, empty_report(), |s, cap| {
            n += 1;
            let mut w = wrap_with(s, &set, Some(cap.unwrap_or(300)))?;
            w.capped = capped;
            w.deviation.within = if capped { 0.5 } else { 1.0 };
            Ok(w)
        })
        .unwrap();
        calls.push(n);
        let comp = super::super::prepare::component_for_tests(&out);
        done.push((comp, out));
    }
    assert_eq!(calls, vec![1, 2, 1], "wraps per part: the retry is made for the second only");
}

/// A split call: the first part spends both extra attempts of the call (a rebuild-and-wrap-again
/// and the second method of `auto`); the second part qualifies for the larger-cap retry and is
/// refused, which its component shows as `budget_spent`, not as "no retry needed"; the part that
/// needs none shows no note.
#[test]
fn a_part_refused_the_retry_by_the_spent_budget_says_so_in_its_component() {
    // Arrange
    let (mut s, set) = session();
    let mut done: Vec<(super::super::ComponentOut, ToSolidOut)> = Vec::new();
    let mut calls = Vec::new();
    for (k, capped) in [(0, false), (1, true), (2, false), (3, true)] {
        if k == 0 {
            assert!(set.budget.draw() && set.budget.draw(), "part 1 uses both extras");
        }
        let allow = super::super::prepare::retry_allowed(true, &done);
        let set = Settings { allow_retry: allow, name: set.name.clone(), budget: set.budget.clone(), ..set };
        let mut n = 0usize;

        // Act
        let out = freeform_with(&mut s, &set, empty_report(), |s, cap| {
            n += 1;
            let mut w = wrap_with(s, &set, Some(cap.unwrap_or(300)))?;
            w.capped = capped;
            w.deviation.within = if capped { 0.5 } else { 1.0 };
            Ok(w)
        })
        .unwrap();
        calls.push(n);
        done.push((super::super::prepare::component_for_tests(&out), out));
    }

    // Assert: one wrap per part, the refusal is on the parts that qualified
    assert_eq!(calls, vec![1, 1, 1, 1]);
    let reasons: Vec<Option<RetryReason>> = done.iter().map(|(c, _)| c.retry.as_ref().map(|r| r.reason)).collect();
    assert_eq!(reasons, vec![None, Some(RetryReason::BudgetSpent), None, Some(RetryReason::BudgetSpent)]);
    let note = done[1].0.retry.as_ref().unwrap();
    assert!(!note.kept && note.patches_to.is_none() && note.cap_to > note.cap_from, "{note:?}");
    assert!(done[1].1.next_steps.iter().any(|st| st.command == "surface.wrap"), "a larger cap is still advised");
}
