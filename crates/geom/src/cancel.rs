//! A cancellation flag shared between whoever started a long computation and the loops doing it.
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// A flag a long computation checks between steps. Clones share the flag; a new token is not
/// cancelled, and one that was cancelled stays so.
#[derive(Clone, Debug, Default)]
pub struct Cancel(Arc<AtomicBool>);

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

    /// Was the computation asked to stop?
    #[must_use]
    pub fn is_cancelled(&self) -> bool {
        self.0.load(Ordering::Acquire)
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
