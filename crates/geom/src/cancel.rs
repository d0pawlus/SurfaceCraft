//! A cancellation flag shared between whoever started a long computation and the loops doing it.
use std::sync::Arc;
#[cfg(test)]
use std::sync::atomic::AtomicUsize;
use std::sync::atomic::{AtomicBool, Ordering};

/// A flag a long computation checks between steps. Clones share the flag; a new token is not
/// cancelled, and one that was cancelled stays so.
#[derive(Clone, Debug, Default)]
pub struct Cancel {
    flag: Arc<AtomicBool>,
    /// Looks left before a test token cancels itself (0: never).
    #[cfg(test)]
    countdown: Arc<AtomicUsize>,
}

impl Cancel {
    /// A token nobody has cancelled.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Asks the computation holding a clone of this token to stop.
    pub fn cancel(&self) {
        self.flag.store(true, Ordering::Release);
    }

    /// Was the computation asked to stop? (In this crate's tests each look also counts down a token
    /// made by `Cancel::after_looks`.)
    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        #[cfg(test)]
        self.count_down();
        self.flag.load(Ordering::Acquire)
    }

    /// One look at a test token: cancels it at its last one (a countdown that loses a race to another
    /// thread just counts that look later).
    #[cfg(test)]
    fn count_down(&self) {
        let left = self.countdown.load(Ordering::Acquire);
        if left > 0 && self.countdown.compare_exchange(left, left - 1, Ordering::AcqRel, Ordering::Acquire) == Ok(1) {
            self.flag.store(true, Ordering::Release);
        }
    }

    /// A token that cancels itself at its `n`-th look (`n` ≥ 1): for testing that a computation
    /// polls and stops wherever the flag is set.
    #[cfg(test)]
    #[must_use]
    pub fn after_looks(n: usize) -> Self {
        Cancel { flag: Arc::new(AtomicBool::new(false)), countdown: Arc::new(AtomicUsize::new(n)) }
    }

    /// Looks taken at a token from [`Cancel::after_looks`] that are still to come (0 once it fired).
    #[cfg(test)]
    #[must_use]
    pub fn looks_left(&self) -> usize {
        self.countdown.load(Ordering::Acquire)
    }
}

#[cfg(test)]
mod tests {
    use super::Cancel;

    #[test]
    fn clones_share_the_flag_and_a_new_token_is_clear() {
        let a = Cancel::new();
        let b = a.clone();
        assert!(!b.is_cancelled());
        a.cancel();
        assert!(b.is_cancelled());
        assert!(!Cancel::new().is_cancelled());
    }
}
