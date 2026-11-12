#![cfg(not(target_arch = "wasm32"))]
//! A dead pool thread (in the browser a panic aborts the worker, and the job on it never ends)
//! must not leave a job running for ever. Alone in this file: the report is process-wide and
//! would make the jobs of other tests look lost.
use serde_json::json;
use std::sync::Arc;
use surfacecraft_engine::{Job, JobPoll, Session};
use surfacecraft_geom::par;

#[test]
fn a_job_is_lost_once_a_pool_thread_is_reported_dead() {
    let mut shown = Session::new();
    shown.execute("mesh.synth", json!({"shape": "sphere", "resolution": 300, "noise": 0.05, "name": "scan"})).unwrap();
    let job = Job::start(&shown, "region.auto_segment", Arc::new(json!({"mesh": "scan"}))).unwrap();
    assert!(matches!(job.poll(), JobPoll::Running));
    assert!(job.is_running());

    par::background_failed();

    assert!(matches!(job.poll(), JobPoll::Lost), "the job is given up, not waited for");
    assert!(!job.is_running());
    assert_eq!(shown.log.len(), 1, "the session the job was started from is untouched");
    // let the thread end early instead of segmenting to the end
    job.cancel();
}
