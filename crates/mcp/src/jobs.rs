//! The job queue: every call is a job on one worker, run in the order it was sent. A call with
//! `queued: true` returns its job id at once; `job_status`, `job_wait` and `job_cancel` read it.
use crate::MAX_OPEN_JOBS;
use serde_json::Value;
use std::collections::{BTreeMap, VecDeque};
use std::sync::{Condvar, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};

/// Where a job is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum State {
    Queued,
    Running,
    Done,
    Failed,
    Cancelled,
}

impl State {
    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::Queued => "queued",
            Self::Running => "running",
            Self::Done => "done",
            Self::Failed => "failed",
            Self::Cancelled => "cancelled",
        }
    }

    pub(crate) fn finished(self) -> bool {
        matches!(self, Self::Done | Self::Failed | Self::Cancelled)
    }
}

/// What a job runs: a tool of the listing, or one command (for the resources).
#[derive(Clone, Debug)]
pub(crate) enum Request {
    Tool { name: String, args: Value },
    Command { id: String, params: Value },
}

impl Request {
    pub(crate) fn label(&self) -> String {
        match self {
            Self::Tool { name, .. } => name.clone(),
            Self::Command { id, .. } => id.clone(),
        }
    }
}

/// One job: its request until it starts, then its state and, once finished, its tool result.
pub(crate) struct Job {
    pub(crate) label: String,
    pub(crate) request: Option<Request>,
    pub(crate) state: State,
    pub(crate) result: Value,
}

/// The job table: a FIFO of job ids and the jobs by id.
#[derive(Default)]
pub(crate) struct Table {
    pub(crate) next: u64,
    pub(crate) queue: VecDeque<u64>,
    pub(crate) jobs: BTreeMap<u64, Job>,
    pub(crate) closed: bool,
}

/// The table shared by the server and its worker.
#[derive(Default)]
pub(crate) struct Jobs {
    pub(crate) table: Mutex<Table>,
    pub(crate) changed: Condvar,
}

/// How long a read waits for a job to finish.
#[derive(Clone, Copy, Debug)]
pub(crate) enum Wait {
    /// Not at all: report the state now.
    Poll,
    /// Up to this long.
    For(Duration),
    /// Until finished.
    Forever,
}

/// A job as read: its state and, once finished, its tool result.
pub(crate) struct View {
    pub(crate) job: u64,
    pub(crate) label: String,
    pub(crate) state: State,
    pub(crate) result: Value,
}

impl Jobs {
    pub(crate) fn lock(&self) -> MutexGuard<'_, Table> {
        self.table.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Queues `request`. With `capped`, a call is refused while `MAX_OPEN_JOBS` jobs are open.
    pub(crate) fn submit(&self, request: Request, capped: bool) -> Result<u64, String> {
        let mut t = self.lock();
        if capped && t.jobs.len() >= MAX_OPEN_JOBS {
            return Err(format!("{MAX_OPEN_JOBS} jobs are open; read finished ones with job_wait or job_status first"));
        }
        t.next = t.next.saturating_add(1);
        let id = t.next;
        let label = request.label();
        t.jobs.insert(id, Job { label, request: Some(request), state: State::Queued, result: Value::Null });
        t.queue.push_back(id);
        self.changed.notify_all();
        Ok(id)
    }

    /// The next job to run, its request taken. With `wait`, blocks until there is one; `None`
    /// when there is none (or the server is gone).
    pub(crate) fn next(&self, wait: bool) -> Option<(u64, Request)> {
        let mut t = self.lock();
        loop {
            while let Some(id) = t.queue.pop_front() {
                let Some(job) = t.jobs.get_mut(&id) else { continue };
                if job.state != State::Queued {
                    continue; // cancelled before it started
                }
                let Some(request) = job.request.take() else { continue };
                job.state = State::Running;
                return Some((id, request));
            }
            if t.closed || !wait {
                return None;
            }
            t = self.changed.wait(t).unwrap_or_else(PoisonError::into_inner);
        }
    }

    /// Records the result of a running job and wakes the readers.
    pub(crate) fn finish(&self, id: u64, result: Value) {
        let mut t = self.lock();
        if let Some(job) = t.jobs.get_mut(&id) {
            let failed = result.get("isError").and_then(Value::as_bool).unwrap_or(false);
            job.state = if failed { State::Failed } else { State::Done };
            job.result = result;
        }
        self.changed.notify_all();
    }

    /// Cancels a job that has not started yet. Returns whether it was cancelled.
    pub(crate) fn cancel(&self, id: u64) -> bool {
        let mut t = self.lock();
        let cancelled = match t.jobs.get_mut(&id) {
            Some(job) if job.state == State::Queued => {
                job.state = State::Cancelled;
                job.request = None;
                true
            }
            _ => false,
        };
        self.changed.notify_all();
        cancelled
    }

    /// The job's view, waiting as `wait` says. A finished job is taken away: its result is
    /// read once. `None` for an unknown job (never queued, or already read).
    pub(crate) fn read(&self, id: u64, wait: Wait) -> Option<View> {
        let end = match wait {
            Wait::For(d) => Instant::now().checked_add(d),
            _ => None,
        };
        let mut t = self.lock();
        loop {
            let state = t.jobs.get(&id)?.state;
            if state.finished() {
                break;
            }
            match wait {
                Wait::Poll => break,
                Wait::Forever => t = self.changed.wait(t).unwrap_or_else(PoisonError::into_inner),
                Wait::For(_) => match end {
                    Some(end) if Instant::now() < end => {
                        t = match self.changed.wait_timeout(t, end.saturating_duration_since(Instant::now())) {
                            Ok((guard, _)) => guard,
                            Err(e) => e.into_inner().0,
                        };
                    }
                    _ => break,
                },
            }
        }
        let (state, label) = {
            let job = t.jobs.get(&id)?;
            (job.state, job.label.clone())
        };
        let result =
            if state.finished() { t.jobs.remove(&id).map_or(Value::Null, |job| job.result) } else { Value::Null };
        Some(View { job: id, label, state, result })
    }
}
