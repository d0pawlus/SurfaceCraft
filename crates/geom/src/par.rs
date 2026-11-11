//! Data parallelism (P2): rayon where the calling thread may fan work out, plain loops elsewhere.
//! Callers write one closure; the result order is the index order either way, so parallel and
//! sequential runs give identical results.
//!
//! Native targets always fan out. In the browser a thread may block only when it is a worker of
//! the thread pool (the main thread must not wait: `Atomics.wait` throws there), so work spreads
//! only when the caller already runs on the pool, which is where [`spawn_background`] puts heavy
//! work. A browser build without threads never has a pool and runs everything in place.
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// Stack of a background thread on native targets (the browser's workers get theirs from
/// the web app): solidcore recurses deeply and asks for 8 MiB.
#[cfg(not(target_arch = "wasm32"))]
const BACKGROUND_STACK_BYTES: usize = 8 * 1024 * 1024;

/// Set once the browser's thread pool runs (see [`enable_background`]).
static POOL_STARTED: AtomicBool = AtomicBool::new(false);

/// Times a background thread was reported dead (see [`background_failed`]).
static FAILURES: AtomicUsize = AtomicUsize::new(0);

/// May the calling thread hand work to other threads and wait for it?
fn fans_out() -> bool {
    #[cfg(not(target_arch = "wasm32"))]
    {
        true
    }
    #[cfg(target_arch = "wasm32")]
    {
        rayon::current_thread_index().is_some()
    }
}

/// Threads data-parallel work started on the calling thread spreads over (1 where it runs in
/// place: the browser's main thread, or a browser build without threads).
#[must_use]
pub fn threads() -> usize {
    if fans_out() { rayon::current_num_threads() } else { 1 }
}

/// Does work started here run on several threads? (Results never depend on it; it only tells
/// callers whether speculative work can pay off.)
#[must_use]
pub fn parallel() -> bool {
    threads() > 1
}

/// Tells this module that the browser's thread pool is running, so [`spawn_background`] may use
/// it. The web app calls it once the pool has started; native targets need no call.
pub fn enable_background() {
    POOL_STARTED.store(true, Ordering::Release);
}

/// Reports that a thread of the browser's pool died (its worker threw: a panic aborts in wasm).
/// The pool is not used from then on, [`background_available`] is false and work runs in place;
/// whoever waits for background work learns from [`background_failures`] that it may never end.
/// On native targets, where a panic unwinds and is caught, only the count moves.
pub fn background_failed() {
    POOL_STARTED.store(false, Ordering::Release);
    FAILURES.fetch_add(1, Ordering::AcqRel);
}

/// How many times [`background_failed`] was called. Work started while it had one value and
/// still unfinished when it has another must be given up.
#[must_use]
pub fn background_failures() -> usize {
    FAILURES.load(Ordering::Acquire)
}

/// Can [`spawn_background`] run work off the calling thread? Always on native targets; in the
/// browser only after [`enable_background`].
#[must_use]
pub fn background_available() -> bool {
    cfg!(not(target_arch = "wasm32")) || POOL_STARTED.load(Ordering::Acquire)
}

/// Runs `f` on another thread and returns at once; false (and `f` is dropped unrun) when there is
/// no thread to run it on. Native: a new thread with the stack solidcore needs. Browser: a worker
/// of the pool, from where the functions of this module fan out.
pub fn spawn_background<F: FnOnce() + Send + 'static>(f: F) -> bool {
    if !background_available() {
        return false;
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::thread::Builder::new().name("surfacecraft-job".into()).stack_size(BACKGROUND_STACK_BYTES).spawn(f).is_ok()
    }
    #[cfg(target_arch = "wasm32")]
    {
        rayon::spawn(f);
        true
    }
}

/// `f(0), f(1), …, f(n − 1)`, evaluated in parallel where the caller may fan out.
pub fn map<T, F>(n: usize, f: F) -> Vec<T>
where
    T: Send,
    F: Fn(usize) -> T + Sync + Send,
{
    if fans_out() {
        use rayon::prelude::*;
        (0..n).into_par_iter().map(f).collect()
    } else {
        (0..n).map(f).collect()
    }
}

/// `f` applied to every item of `items`, in parallel where the caller may fan out.
pub fn map_slice<S, T, F>(items: &[S], f: F) -> Vec<T>
where
    S: Sync,
    T: Send,
    F: Fn(&S) -> T + Sync + Send,
{
    if fans_out() {
        use rayon::prelude::*;
        items.par_iter().map(f).collect()
    } else {
        items.iter().map(f).collect()
    }
}

/// Runs both closures, in parallel where the caller may fan out; the results are those of
/// `(a(), b())`.
pub fn join<A, B, RA, RB>(a: A, b: B) -> (RA, RB)
where
    A: FnOnce() -> RA + Send,
    B: FnOnce() -> RB + Send,
    RA: Send,
    RB: Send,
{
    if fans_out() { rayon::join(a, b) } else { (a(), b()) }
}

/// Sorts in parallel where the caller may fan out (unstable; ties must not matter to the caller).
pub fn sort_unstable<T: Ord + Send>(v: &mut [T]) {
    if fans_out() {
        use rayon::prelude::*;
        v.par_sort_unstable();
    } else {
        v.sort_unstable();
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn order_is_kept() {
        let v = super::map(10_000, |i| i * 2);
        assert!(v.iter().enumerate().all(|(i, x)| *x == 2 * i));
        let w = super::map_slice(&v, |x| x + 1);
        assert_eq!(w[9_999], 19_999);
    }

    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn background_work_runs_on_another_thread() {
        let (tx, rx) = std::sync::mpsc::channel();
        let here = std::thread::current().id();
        assert!(super::background_available());
        assert!(super::spawn_background(move || {
            let _ = tx.send(std::thread::current().id());
        }));
        let there = rx.recv_timeout(std::time::Duration::from_secs(10)).unwrap();
        assert_ne!(here, there);
    }
}
