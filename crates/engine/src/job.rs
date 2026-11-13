//! A command running off the calling thread, for front ends that must stay responsive.
//!
//! [`Job::start`] forks the session ([`Session::fork`]) and executes the command on the fork on
//! another thread, exactly as [`Session::execute`] would here: the fork comes back with the same
//! log entry, undo step and result. The caller keeps its own session untouched meanwhile (to
//! draw from) and replaces it with the fork when the job is done; dropping the job instead
//! abandons the command, and the caller's session is the state from before it.
//!
//! The caller never waits: [`Job::poll`] only reads a flag and then takes the result. In a
//! browser the page's thread may not block at all, so this is the only way it may be used there.
use crate::{EngineError, Session};
use serde_json::Value;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, TryLockError};
use surfacecraft_geom::{Cancel, par};

/// What a finished job hands back.
#[derive(Debug)]
pub struct Finished {
    /// The fork the command ran on: the caller's session from now on.
    pub session: Session,
    /// What [`Session::execute`] returned.
    pub result: Result<Value, EngineError>,
}

/// The state of a job.
#[derive(Debug)]
pub enum JobPoll {
    /// The command is still running.
    Running,
    /// The command finished (with a result or an error); returned once.
    Finished(Box<Finished>),
    /// The job's thread ended without a result (a bug: it panicked outside the command), a
    /// thread of the browser's pool died while the job ran (a panic aborts in wasm), or the
    /// result was already taken. The caller's own session is still good.
    Lost,
}

#[derive(Debug, Default)]
struct Slot {
    /// Set when the job's thread is done, after the result is in place.
    done: AtomicBool,
    finished: Mutex<Option<Finished>>,
}

/// Sets the flag when the job's thread ends, however it ends.
struct Done(Arc<Slot>);

impl Drop for Done {
    fn drop(&mut self) {
        self.0.done.store(true, Ordering::Release);
    }
}

/// One command running on a fork of a session, on another thread.
#[derive(Debug)]
pub struct Job {
    command: String,
    params: Arc<Value>,
    slot: Arc<Slot>,
    cancel: Cancel,
    /// [`par::background_failures`] when the job started: if it moves, a thread of the pool
    /// died, and the job may be the one that ran on it.
    failures: usize,
}

impl Job {
    /// Starts command `id` with `params` on a fork of `session`. `None` when there is no thread
    /// to run it on (a browser without threads): the caller then executes it in place.
    ///
    /// The parameters are shared with the job, not copied here: the one copy `execute` needs is
    /// made on the job's thread.
    #[must_use]
    pub fn start(session: &Session, id: &str, params: Arc<Value>) -> Option<Job> {
        let (command, given) = (id.to_string(), params.clone());
        Self::start_with(session.fork(), id, params, move |s| s.execute(&command, Value::clone(&given)))
    }

    fn start_with<W>(mut fork: Session, id: &str, params: Arc<Value>, work: W) -> Option<Job>
    where
        W: FnOnce(&mut Session) -> Result<Value, EngineError> + Send + 'static,
    {
        let slot = Arc::new(Slot::default());
        let cancel = fork.cancel.clone();
        let done = Done(slot.clone());
        let failures = par::background_failures();
        let spawned = par::spawn_background(move || {
            let result = work(&mut fork);
            if let Ok(mut finished) = done.0.finished.lock() {
                *finished = Some(Finished { session: fork, result });
            }
            // `done` drops here, after the result is in place
        });
        spawned.then(|| Job { command: id.to_string(), params, slot, cancel, failures })
    }

    /// The command's id.
    #[must_use]
    pub fn command(&self) -> &str {
        &self.command
    }

    /// The parameters the command was started with.
    #[must_use]
    pub fn params(&self) -> &Arc<Value> {
        &self.params
    }

    /// Where the job is; never blocks.
    #[must_use]
    pub fn poll(&self) -> JobPoll {
        if !self.slot.done.load(Ordering::Acquire) {
            // a dead pool thread never sets the flag: do not wait for it for ever
            let orphaned = par::background_failures() != self.failures;
            return if orphaned { JobPoll::Lost } else { JobPoll::Running };
        }
        let taken = match self.slot.finished.try_lock() {
            Ok(mut finished) => finished.take(),
            Err(TryLockError::Poisoned(poisoned)) => poisoned.into_inner().take(),
            Err(TryLockError::WouldBlock) => return JobPoll::Running,
        };
        taken.map_or(JobPoll::Lost, |f| JobPoll::Finished(Box::new(f)))
    }

    /// Is the job's thread still at work? (Also after [`Job::cancel`], until the command ends.)
    #[must_use]
    pub fn is_running(&self) -> bool {
        !self.slot.done.load(Ordering::Acquire) && par::background_failures() == self.failures
    }

    /// Asks the command to stop. The commands whose long loops look at the flag
    /// (`region.auto_segment`, `region.resegment`, `align.icp`, `align.pair_coarse`, `mesh.smooth`) end within a
    /// moment with the error [`crate::cmd::CANCELLED`]; any other runs to its end, but writes no
    /// file once cancelled (`cmd::write_file`). The caller's own session is the state from
    /// before the command either way; it keeps the job to know when its thread is free again
    /// ([`Job::is_running`]).
    pub fn cancel(&self) {
        self.cancel.cancel();
    }
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod tests {
    use super::*;
    use std::time::{Duration, Instant};

    fn wait(job: &Job) -> JobPoll {
        let started = Instant::now();
        loop {
            match job.poll() {
                JobPoll::Running if started.elapsed() < Duration::from_secs(60) => {
                    std::thread::sleep(Duration::from_millis(2));
                }
                other => return other,
            }
        }
    }

    #[test]
    fn a_job_whose_thread_panics_is_lost_and_the_session_survives() {
        let mut session = Session::new();
        session.execute("mesh.synth", serde_json::json!({"shape": "sphere"})).unwrap();
        let before = session.doc.summaries();
        let job = Job::start_with(session.fork(), "test.panic", Arc::new(Value::Null), |_| {
            panic!("a bug outside the command")
        })
        .unwrap();
        assert!(matches!(wait(&job), JobPoll::Lost));
        assert_eq!(session.doc.summaries(), before);
        assert_eq!(session.log.len(), 1);
    }

    #[test]
    fn the_work_runs_on_another_thread() {
        let here = std::thread::current().id();
        let job = Job::start_with(Session::new(), "test.thread", Arc::new(Value::Null), move |_| {
            Ok(serde_json::json!(std::thread::current().id() != here))
        })
        .unwrap();
        let JobPoll::Finished(done) = wait(&job) else { panic!("the job did not finish") };
        assert_eq!(done.result, Ok(serde_json::json!(true)));
    }
}
