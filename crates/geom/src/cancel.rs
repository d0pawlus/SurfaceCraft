//! A cancellation flag shared between whoever started a long computation and the loops doing it.
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

/// A flag a long computation checks between steps. Clones share the flag; a new token is not
/// cancelled, and one that was cancelled stays so.
#[derive(Clone, Debug, Default)]
pub struct Cancel(Arc<AtomicBool>, Arc<AtomicUsize>);

impl Cancel {
    /// A token nobody has cancelled.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Asks the computation holding a clone of this token to stop.
    pub fn cancel(&self) {
        self.0.store(true, Ordering::Release);
    }

    /// Was the computation asked to stop? (Each look also counts down a token made by
    /// [`Cancel::after_looks`].)
    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        let left = self.1.load(Ordering::Acquire);
        // a plain token reads 0 and pays one load; a countdown that loses a race to another thread
        // just counts that look later
        if left > 0 && self.1.compare_exchange(left, left - 1, Ordering::AcqRel, Ordering::Acquire) == Ok(1) {
            self.0.store(true, Ordering::Release);
        }
        self.0.load(Ordering::Acquire)
    }

    /// A token that cancels itself at its `n`-th look (`n` ≥ 1): for testing that a computation
    /// polls and stops wherever the flag is set.
    #[cfg(test)]
    #[must_use]
    pub fn after_looks(n: usize) -> Self {
        Cancel(Arc::new(AtomicBool::new(false)), Arc::new(AtomicUsize::new(n)))
    }

    /// Looks taken at a token from [`Cancel::after_looks`] that are still to come (0 once it fired).
    #[cfg(test)]
    #[must_use]
    pub fn looks_left(&self) -> usize {
        self.1.load(Ordering::Acquire)
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
