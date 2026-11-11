#![cfg(not(target_arch = "wasm32"))]
//! Background jobs: a command run on a fork of the session, on another thread, gives what
//! `Session::execute` gives in place, and a cancelled or failed one leaves the session alone.
use serde_json::{Value, json};
use std::time::{Duration, Instant};
use surfacecraft_engine::{EngineError, Finished, Job, JobPoll, Session, cmd};

/// Longest a test waits for a job.
const PATIENCE: Duration = Duration::from_secs(300);

fn session_with(shape: Value) -> Session {
    let mut s = Session::new();
    s.execute("mesh.synth", shape).unwrap();
    s
}

fn bracket() -> Session {
    session_with(json!({"shape": "bracket", "name": "scan"}))
}

/// A noisy sphere of about 90 thousand triangles: its segmentation takes seconds.
fn slow_scan() -> Session {
    session_with(json!({"shape": "sphere", "resolution": 300, "noise": 0.05, "name": "scan"}))
}

fn finish(job: &Job) -> Finished {
    let started = Instant::now();
    loop {
        match job.poll() {
            JobPoll::Finished(done) => return *done,
            JobPoll::Lost => panic!("the job was lost"),
            JobPoll::Running => {
                assert!(started.elapsed() < PATIENCE, "the job did not finish");
                std::thread::sleep(Duration::from_millis(2));
            }
        }
    }
}

#[test]
fn a_job_gives_the_result_log_and_undo_step_of_a_run_in_place() {
    let params = json!({"mesh": "scan"});
    let mut in_place = bracket();
    let expected = in_place.execute("region.auto_segment", params.clone());
    assert!(expected.is_ok(), "{expected:?}");

    let shown = bracket();
    let job = Job::start(&shown, "region.auto_segment", params.clone()).expect("a native build has threads");
    assert_eq!((job.command(), job.params()), ("region.auto_segment", &params));
    let done = finish(&job);

    assert_eq!(done.result, expected);
    assert_eq!(done.session.log, in_place.log);
    assert_eq!(done.session.history.undo_labels(), in_place.history.undo_labels());
    assert_eq!(done.session.doc.summaries(), in_place.doc.summaries());
    assert_eq!(done.session.epoch(), in_place.epoch());
    // the session the job was started from is the state from before the command
    assert_eq!(shown.log.len(), 1);
    assert!(shown.doc.regions.is_empty());
    assert!(matches!(job.poll(), JobPoll::Lost), "the result is handed over once");
}

#[test]
fn a_failing_job_reports_the_error_and_rolls_its_fork_back() {
    let shown = bracket();
    let params = json!({"mesh": "no such scan"});
    let mut in_place = bracket();
    let expected = in_place.execute("region.auto_segment", params.clone());
    assert!(matches!(expected, Err(EngineError::InvalidParams(_))), "{expected:?}");

    let done = finish(&Job::start(&shown, "region.auto_segment", params).unwrap());

    assert_eq!(done.result, expected);
    assert_eq!(done.session.log, in_place.log);
    assert_eq!(done.session.doc.summaries(), shown.doc.summaries());
    assert_eq!(done.session.history.undo_labels(), ["mesh.synth"], "no undo step for the command");
}

#[test]
fn a_cancelled_segmentation_stops_early_and_leaves_no_trace_in_the_document() {
    let shown = slow_scan();
    let job = Job::start(&shown, "region.auto_segment", json!({"mesh": "scan"})).unwrap();
    job.cancel();
    let started = Instant::now();
    let done = finish(&job);

    assert_eq!(done.result, Err(EngineError::Failed(cmd::CANCELLED.into())));
    assert!(done.session.doc.regions.is_empty(), "a cancelled command is rolled back");
    assert_eq!(done.session.history.undo_labels(), ["mesh.synth"], "no undo step for the command");
    assert!(started.elapsed() < Duration::from_secs(20), "cancelling took {:?}", started.elapsed());
}

#[test]
fn a_fork_has_its_own_cancellation_flag() {
    let s = bracket();
    let fork = s.fork();
    fork.cancel.cancel();
    assert!(!s.cancel.is_cancelled());
    assert!(!fork.fork().cancel.is_cancelled());
}

#[test]
fn a_cancelled_command_is_noted_in_the_log_only() {
    let mut s = bracket();
    let before = s.doc.summaries();
    s.note_cancelled("region.auto_segment", json!({"mesh": "scan"}));
    let last = s.log.last().unwrap();
    assert_eq!((last.command.as_str(), last.ok), ("region.auto_segment", false));
    assert_eq!(last.outcome, json!({"kind": "failed", "message": "cancelled"}));
    assert_eq!(s.doc.summaries(), before);
    assert_eq!(s.history.undo_labels(), ["mesh.synth"]);
}
