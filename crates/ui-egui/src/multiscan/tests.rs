//! The wizard's state machine, driven with canned outcomes: no window, no engine.
use super::report::{self, GlobalScan};
use super::*;
use serde_json::{Value, json};

const IDENTITY: [[f64; 4]; 4] =
    [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]];

fn shifted(x: f64) -> [[f64; 4]; 4] {
    let mut m = IDENTITY;
    m[0][3] = x;
    m
}

fn ids(n: u64) -> Vec<ObjectId> {
    (1..=n).map(ObjectId).collect()
}

/// The undo labels of a document with `n` steps.
fn history(n: usize) -> Vec<String> {
    (0..n).map(|k| format!("step {k}")).collect()
}

fn pair_outcome(verdict: &str, applied: bool) -> Value {
    json!({
        "matrix": shifted(5.0), "applied": applied, "verdict": verdict, "confident": verdict == "unique",
        "overlap_moving": 0.8, "overlap_fixed": 0.7, "confidence": 0.9, "note": "a sentence",
        "alternatives": if verdict == "ambiguous" {
            json!([{"transform": shifted(9.0), "overlap_moving": 0.6, "overlap_fixed": 0.5, "confidence": 0.3, "rms": 0.1, "score": 1.0, "method": "frames"}])
        } else {
            json!([])
        },
    })
}

/// A global outcome: `scans` says for each one whether it is `(placed, held_by_loop)`.
fn global_outcome(verdict: &str, applied: bool, scans: &[(bool, bool)]) -> Value {
    let rows: Vec<Value> = scans
        .iter()
        .enumerate()
        .map(|(k, (placed, held))| {
            json!({"index": k, "fixed": k == 0, "placed": placed, "held_by_loop": held, "matrix": shifted(k as f64)})
        })
        .collect();
    json!({
        "verdict": verdict, "issues": ["weakly_held"], "applied": applied, "scans": rows,
        "edges": [{"fixed": 0, "moving": 1, "status": "used", "rms": 0.02, "overlap_moving": 0.6, "strong_evidence": true}],
        "loop": {"cycles": 1, "before": {"worst_mm": 0.4, "mean_mm": 0.2}, "after": {"worst_mm": 0.1, "mean_mm": 0.05}},
        "used_edges": 3, "rejected_edges": 0, "moved": if applied { json!([1, 2]) } else { json!([]) }, "note": "n",
    })
}

fn merge_outcome(refused: bool, codes: &[&str]) -> Value {
    let issues: Vec<Value> = codes
        .iter()
        .map(|c| json!({"code": c, "severity": "refusal", "scans": [0, 1], "message": format!("{c} message")}))
        .collect();
    json!({
        "id": if refused { Value::Null } else { json!(40) }, "refused": refused, "tolerance": 0.2,
        "issues": issues, "triangles": 1200, "seam": {"open_edges": 0}, "after": {"pieces": 1}, "note": "n",
    })
}

/// Takes the one command the effects ask for and feeds back its outcome.
fn answer(w: &mut Wizard, fx: Effects, outcome: Value, labels: usize) -> Effects {
    let request = fx.run.expect("a command was asked for");
    w.on_result(request.purpose, Ok(outcome), &history(labels))
}

/// A wizard on three scans that has been through the coarse step with both poses accepted.
fn after_pairs(accept_all: bool) -> Wizard {
    let mut w = Wizard::new(&history(2), ids(3));
    let fx = w.act(Action::Next, &history(2));
    let fx = answer(&mut w, fx, pair_outcome("unique", false), 2);
    assert_eq!(fx, Effects::default());
    let fx = w.act(Action::AcceptPair, &history(2));
    let fx = answer(&mut w, Effects { run: fx.run, ..Effects::default() }, pair_outcome("unique", true), 3);
    let fx = answer(&mut w, fx, pair_outcome("unique", false), 3);
    assert_eq!(fx, Effects::default());
    let fx = if accept_all { w.act(Action::AcceptPair, &history(3)) } else { w.act(Action::RejectPair, &history(3)) };
    if accept_all {
        answer(&mut w, fx, pair_outcome("unique", true), 4);
    }
    w
}

#[test]
fn two_scans_are_needed_before_the_wizard_goes_on() {
    let mut w = Wizard::new(&history(0), Vec::new());
    assert!(!w.can_next());
    w.act(Action::ToggleScan(ObjectId(7)), &history(0));
    assert!(!w.can_next(), "one scan is no pair");
    w.act(Action::ToggleScan(ObjectId(8)), &history(0));
    assert!(w.can_next());
    w.act(Action::ToggleScan(ObjectId(7)), &history(0));
    assert_eq!(w.scans(), [ObjectId(8)], "a second click takes the scan off");
    assert_eq!(w.act(Action::Next, &history(0)), Effects::default(), "next does nothing without two scans");
}

#[test]
fn the_pairs_chain_each_scan_onto_the_one_before_it() {
    let mut w = Wizard::new(&history(0), ids(3));

    let fx = w.act(Action::Next, &history(0));

    assert_eq!(w.step(), Step::Pairs);
    let request = fx.run.expect("the first pair is proposed at once");
    assert_eq!(request.command, "align.pair_coarse");
    assert_eq!(request.params, json!({"object": 2, "target": 1, "apply": false}));
    assert_eq!(request.purpose, Purpose::ProposePair(0));
    let pairs: Vec<(u64, u64)> = w.pairs().iter().map(|p| (p.object.0, p.target.0)).collect();
    assert_eq!(pairs, [(2, 1), (3, 2)]);
}

#[test]
fn accepting_a_pair_applies_the_shown_pose_and_proposes_the_next_pair() {
    let mut w = Wizard::new(&history(2), ids(3));
    let fx = w.act(Action::Next, &history(2));
    answer(&mut w, fx, pair_outcome("ambiguous", false), 2);
    w.act(Action::ChoosePose(1), &history(2));
    assert_eq!(w.preview(), vec![(ObjectId(2), shifted(9.0))], "the chosen alternative is shown");

    let fx = w.act(Action::AcceptPair, &history(2));

    let request = fx.run.expect("the pose is applied");
    assert_eq!(request.params, json!({"object": 2, "target": 1, "apply": true, "choose": 1}));
    assert!(w.preview().is_empty(), "nothing is previewed while the command runs");
    let fx = answer(&mut w, Effects { run: Some(request), ..Effects::default() }, pair_outcome("ambiguous", true), 3);
    assert_eq!(w.undo_steps(), 1, "the applied pose is one undo step");
    assert!(matches!(w.pairs()[0].state, PairState::Accepted(_)));
    assert_eq!(fx.run.map(|r| r.purpose), Some(Purpose::ProposePair(1)), "the next pair follows");
}

#[test]
fn a_pair_with_no_match_cannot_be_accepted_only_rejected() {
    let mut w = Wizard::new(&history(0), ids(2));
    let fx = w.act(Action::Next, &history(0));
    answer(&mut w, fx, pair_outcome("no_match", false), 0);

    assert_eq!(w.act(Action::AcceptPair, &history(0)), Effects::default());
    let fx = w.act(Action::RejectPair, &history(0));

    assert_eq!(fx, Effects::default(), "it was the last pair");
    assert!(w.pairs_resolved());
    assert!(w.can_next());
}

#[test]
fn global_starts_from_the_current_poses_only_when_every_pair_was_accepted() {
    let mut accepted = after_pairs(true);
    let mut partly = after_pairs(false);

    let fx_accepted = accepted.act(Action::Next, &history(4));
    let fx_partly = partly.act(Action::Next, &history(3));

    assert_eq!(fx_accepted.run.map(|r| r.params["start"].clone()), Some(json!("current")));
    assert_eq!(fx_partly.run.map(|r| r.params["start"].clone()), Some(json!("search")));
}

#[test]
fn global_applies_with_the_weakly_held_scans_the_user_accepted() {
    let mut w = after_pairs(true);
    let fx = w.act(Action::Next, &history(4));
    let rows = [(true, true), (true, false), (true, true)];
    answer(&mut w, fx, global_outcome("partial", false, &rows), 4);
    assert_eq!(w.preview().len(), 1, "the weakly held scan stays where it is until it is accepted");
    w.act(Action::ToggleWeak(0), &history(4));
    assert!(w.accepted_weak().is_empty(), "the fixed scan is never weak");
    w.act(Action::ToggleWeak(1), &history(4));
    assert_eq!(w.preview().len(), 2);

    let fx = w.act(Action::ApplyGlobal, &history(4));

    let request = fx.run.expect("the poses are applied");
    assert_eq!(request.params["apply"], true);
    assert_eq!(request.params["accept_weak"], json!([2]));
    assert_eq!(request.params["fixed"], 1);
    answer(&mut w, Effects { run: Some(request), ..Effects::default() }, global_outcome("partial", true, &rows), 5);
    assert!(matches!(w.global(), GlobalState::Applied(_)));
    assert_eq!(w.undo_steps(), 3);
}

#[test]
fn a_global_alignment_that_connected_nothing_cannot_be_applied() {
    let mut w = after_pairs(true);
    let fx = w.act(Action::Next, &history(4));
    answer(&mut w, fx, global_outcome("failed", false, &[(true, true), (false, false), (false, false)]), 4);

    assert_eq!(w.act(Action::ApplyGlobal, &history(4)), Effects::default());
    assert!(w.can_next(), "the step can still be skipped");
}

#[test]
fn an_apply_that_moves_nothing_makes_no_undo_step_and_says_so() {
    let mut w = after_pairs(true);
    let fx = w.act(Action::Next, &history(4));
    let rows = [(true, true), (true, false), (true, false)];
    answer(&mut w, fx, global_outcome("partial", false, &rows), 4);
    let fx = w.act(Action::ApplyGlobal, &history(4));

    answer(&mut w, fx, global_outcome("partial", false, &rows), 4);

    assert_eq!(w.undo_steps(), 2, "only the two accepted pairs");
    assert_eq!(w.problem(), Some(&Problem::NothingMoved));
    assert!(matches!(w.global(), GlobalState::Proposed(_)));
}

fn at_merge() -> Wizard {
    let mut w = after_pairs(true);
    let fx = w.act(Action::Next, &history(4));
    answer(&mut w, fx, global_outcome("aligned", false, &[(true, true); 3]), 4);
    let fx = w.act(Action::ApplyGlobal, &history(4));
    answer(&mut w, fx, global_outcome("aligned", true, &[(true, true); 3]), 5);
    assert_eq!(w.act(Action::Next, &history(5)), Effects::default());
    assert_eq!(w.step(), Step::Merge);
    w
}

#[test]
fn a_misalignment_refusal_offers_to_merge_anyway_and_the_retry_says_warn() {
    let mut w = at_merge();
    let fx = w.act(Action::Merge, &history(5));
    assert_eq!(fx.run.as_ref().map(|r| r.params["alignment"].clone()), Some(json!("refuse")));

    answer(&mut w, fx, merge_outcome(true, &[report::MISALIGNED]), 5);

    let MergeState::Refused(r) = w.merge() else { panic!("{:?}", w.merge()) };
    assert!(r.warn_would_help());
    assert_eq!(w.undo_steps(), 3, "a refusal makes no undo step");
    w.act(Action::SetWarn(true), &history(5));
    let fx = w.act(Action::Merge, &history(5));
    assert_eq!(fx.run.as_ref().map(|r| r.params["alignment"].clone()), Some(json!("warn")));
}

#[test]
fn a_refusal_that_warn_does_not_lift_does_not_offer_it() {
    let mut w = at_merge();
    let fx = w.act(Action::Merge, &history(5));

    answer(&mut w, fx, merge_outcome(true, &["inverted_scan"]), 5);

    let MergeState::Refused(r) = w.merge() else { panic!("{:?}", w.merge()) };
    assert!(!r.warn_would_help());
    let mixed = report::merge_report(&merge_outcome(true, &[report::MISALIGNED, "opposed_normals"])).unwrap();
    assert!(!mixed.warn_would_help(), "one refusal warn cannot lift is enough");
    let fine = report::merge_report(&merge_outcome(false, &[])).unwrap();
    assert!(!fine.warn_would_help());
}

#[test]
fn a_merge_that_succeeds_is_one_undo_step_and_finish_selects_it() {
    let mut w = at_merge();
    let fx = w.act(Action::Merge, &history(5));

    answer(&mut w, fx, merge_outcome(false, &[]), 6);

    assert!(matches!(w.merge(), MergeState::Done(_)));
    assert_eq!(w.undo_steps(), 4);
    assert_eq!(w.act(Action::Merge, &history(6)), Effects::default(), "the mesh is made once");
    assert_eq!(w.act(Action::Finish, &history(6)), Effects { close: true, select: Some(40), ..Effects::default() });
}

#[test]
fn a_tolerance_that_is_not_a_positive_number_is_refused_without_running() {
    let mut w = at_merge();
    for text in ["abc", "-1", "0", "inf", "NaN"] {
        w.act(Action::SetTolerance(text.into()), &history(5));
        let fx = w.act(Action::Merge, &history(5));
        assert_eq!(fx, Effects::default(), "{text}");
        assert_eq!(w.problem(), Some(&Problem::BadTolerance), "{text}");
    }
    w.act(Action::SetTolerance(" 0.25 ".into()), &history(5));
    w.act(Action::SetName(" whole ".into()), &history(5));
    let params = w.act(Action::Merge, &history(5)).run.map(|r| r.params);
    assert_eq!(params.as_ref().map(|p| p["tolerance"].clone()), Some(json!(0.25)));
    assert_eq!(params.map(|p| p["name"].clone()), Some(json!("whole")));
}

#[test]
fn going_back_undoes_exactly_the_steps_of_the_step_left() {
    let mut w = at_merge();
    let fx = w.act(Action::Merge, &history(5));
    answer(&mut w, fx, merge_outcome(false, &[]), 6);

    let from_merge = w.act(Action::Back, &history(6));
    assert_eq!((w.step(), from_merge.undo), (Step::Global, 1), "the merge");
    let from_global = w.act(Action::Back, &history(5));
    assert_eq!((w.step(), from_global.undo), (Step::Pairs, 1), "the global poses");
    assert!(matches!(w.global(), GlobalState::Idle));
    let from_pairs = w.act(Action::Back, &history(4));
    assert_eq!((w.step(), from_pairs.undo), (Step::Pick, 2), "both accepted poses");
    assert!(w.pairs().is_empty());
    assert_eq!(w.undo_steps(), 0);
}

#[test]
fn going_back_from_a_step_that_changed_nothing_undoes_nothing() {
    let mut w = Wizard::new(&history(3), ids(2));
    let fx = w.act(Action::Next, &history(3));
    answer(&mut w, fx, pair_outcome("unique", false), 3);

    let back = w.act(Action::Back, &history(3));

    assert_eq!((w.step(), back.undo, back.close), (Step::Pick, 0, false));
}

#[test]
fn cancel_undoes_everything_the_wizard_did_at_any_step() {
    let mut at_pairs = after_pairs(true);
    let fx = at_pairs.act(Action::Cancel, &history(4));
    assert_eq!(fx, Effects { undo: 2, close: true, ..Effects::default() });

    let mut merged = at_merge();
    let fx = merged.act(Action::Merge, &history(5));
    answer(&mut merged, fx, merge_outcome(false, &[]), 6);
    let fx = merged.act(Action::Cancel, &history(6));
    assert_eq!(fx, Effects { undo: 4, close: true, ..Effects::default() });

    let mut untouched = Wizard::new(&history(3), ids(2));
    assert_eq!(untouched.act(Action::Cancel, &history(3)), Effects { close: true, ..Effects::default() });
}

#[test]
fn history_changed_behind_the_wizard_blocks_undo_and_leave_still_closes() {
    let mut w = after_pairs(true);
    let other_history = history(3);

    let cancel = w.act(Action::Cancel, &other_history);
    assert_eq!(cancel, Effects::default(), "nothing is undone and the wizard stays");
    assert_eq!(w.problem(), Some(&Problem::Stale));
    let back = w.act(Action::Back, &other_history);
    assert_eq!((back.undo, w.step()), (0, Step::Pairs));
    assert_eq!(w.problem(), Some(&Problem::Stale));

    assert_eq!(w.act(Action::Leave, &other_history), Effects { close: true, ..Effects::default() });
}

#[test]
fn a_command_that_changed_the_history_unexpectedly_is_noticed() {
    let mut w = Wizard::new(&history(2), ids(2));
    let fx = w.act(Action::Next, &history(2));
    answer(&mut w, fx, pair_outcome("unique", false), 2);
    let fx = w.act(Action::AcceptPair, &history(2));

    answer(&mut w, fx, pair_outcome("unique", true), 5);

    assert_eq!(w.problem(), Some(&Problem::Stale));
    assert_eq!(w.undo_steps(), 0, "what cannot be told apart from the user's own steps is not undone");
}

#[test]
fn a_failed_command_keeps_the_wizard_where_it_is_with_the_message() {
    let mut w = Wizard::new(&history(0), ids(3));
    let fx = w.act(Action::Next, &history(0));
    let request = fx.run.expect("proposed");
    w.on_result(request.purpose, Err("no mesh 2".into()), &history(0));
    assert_eq!(w.pairs()[0].state, PairState::Failed("no mesh 2".into()));
    assert_eq!(w.step(), Step::Pairs);
    assert!(!w.can_next());

    let retry = w.act(Action::RetryPair, &history(0));
    assert_eq!(retry.run.map(|r| r.purpose), Some(Purpose::ProposePair(0)));
    answer(
        &mut w,
        Effects {
            run: Some(Request { command: PAIR_COMMAND, params: Value::Null, purpose: Purpose::ProposePair(0) }),
            ..Effects::default()
        },
        pair_outcome("unique", false),
        0,
    );
    let fx = w.act(Action::AcceptPair, &history(0));
    let request = fx.run.expect("applied");
    let after = w.on_result(request.purpose, Err("cancelled".into()), &history(0));

    assert_eq!(after, Effects::default());
    assert_eq!(w.problem(), Some(&Problem::Engine("cancelled".into())));
    assert!(matches!(w.pairs()[0].state, PairState::Proposed(_)), "the proposal is still there");
    assert_eq!(w.undo_steps(), 0);
}

#[test]
fn an_accept_that_moved_nothing_keeps_the_proposal() {
    let mut w = Wizard::new(&history(0), ids(2));
    let fx = w.act(Action::Next, &history(0));
    answer(&mut w, fx, pair_outcome("ambiguous", false), 0);
    let fx = w.act(Action::AcceptPair, &history(0));

    answer(&mut w, fx, pair_outcome("ambiguous", false), 0);

    assert_eq!(w.problem(), Some(&Problem::NothingMoved));
    assert!(matches!(w.pairs()[0].state, PairState::Proposed(_)));
}

#[test]
fn actions_wait_while_a_command_runs_and_a_stray_outcome_is_ignored() {
    let mut w = Wizard::new(&history(0), ids(2));
    let fx = w.act(Action::Next, &history(0));
    assert!(w.running().is_some());

    assert_eq!(w.act(Action::Cancel, &history(0)), Effects::default());
    assert_eq!(w.act(Action::Back, &history(0)), Effects::default());
    assert_eq!(w.on_result(Purpose::Merge, Ok(Value::Null), &history(0)), Effects::default());
    assert!(w.running().is_some(), "an outcome of another purpose does not end the run");
    answer(&mut w, fx, pair_outcome("unique", false), 0);
    assert!(w.running().is_none());
}

#[test]
fn an_outcome_the_wizard_cannot_read_is_a_problem_not_a_crash() {
    let mut w = Wizard::new(&history(0), ids(2));
    let fx = w.act(Action::Next, &history(0));
    answer(&mut w, fx, json!({"surprise": true}), 0);
    assert!(matches!(w.pairs()[0].state, PairState::Failed(_)));

    let mut m = at_merge();
    let fx = m.act(Action::Merge, &history(5));
    answer(&mut m, fx, json!([1, 2, 3]), 5);
    assert_eq!(m.problem(), Some(&Problem::Unreadable));
    assert_eq!(m.merge(), &MergeState::Idle);
}

#[test]
fn a_deleted_scan_is_noticed() {
    let w = Wizard::new(&history(0), ids(3));
    assert!(w.refers_only_to(|_| true));
    assert!(!w.refers_only_to(|id| id != ObjectId(2)));
}

#[test]
fn retrying_the_merge_undoes_it_and_reopens_the_settings() {
    let mut w = at_merge();
    let fx = w.act(Action::Merge, &history(5));
    answer(&mut w, fx, merge_outcome(false, &[]), 6);
    w.act(Action::SetWarn(true), &history(6));
    assert!(!w.options().warn, "the settings are fixed once the mesh exists");

    let fx = w.act(Action::RetryMerge, &history(6));

    assert_eq!((fx.undo, w.merge()), (1, &MergeState::Idle));
    w.act(Action::SetWarn(true), &history(5));
    assert!(w.options().warn);
}

#[test]
fn the_report_readers_take_what_is_there_and_nothing_more() {
    assert!(report::pair_proposal(&json!({})).is_none());
    assert!(report::global_report(&json!({})).is_none());
    assert!(report::merge_report(&json!({})).is_none());
    let scan = GlobalScan { index: 1, fixed: false, placed: true, held_by_loop: false, matrix: IDENTITY };
    assert!(scan.weakly_held());
    let r = report::global_report(&global_outcome("partial", false, &[(true, true), (true, false)])).unwrap();
    assert_eq!((r.loops.cycles, r.used_edges), (1, 3));
    assert!(r.loops.worst_after_mm < r.loops.worst_before_mm);
    assert_eq!(r.edges.len(), 1);
}

#[test]
fn every_step_has_its_texts_in_both_languages() {
    for step in Step::ALL {
        for lang in crate::i18n::Lang::ALL {
            let catalogue = crate::i18n::catalog(lang);
            assert!(catalogue.contains_key(step.title_key()), "{lang:?} {}", step.title_key());
            assert!(catalogue.contains_key(step.hint_key()), "{lang:?} {}", step.hint_key());
        }
    }
}
