//! The budget of extra attempts of one `scan.to_solid` call.
//!
//! The first attempt of a call is the pipeline itself. Anything beyond it (the other method that
//! `auto` tries when the first does not pass, a retry of the freeform fit with a larger cap) is an
//! *extra attempt* and draws from one budget per call: at most [`MAX_EXTRA_ATTEMPTS`] of them, and
//! together at most [`EXTRA_TIME_FACTOR`] times as long as the first attempt took (they cost about
//! as much as it did). A code path that wants to run again asks [`Budget::draw`] and, when it has
//! run, reports the time with [`Budget::spend`].

/// Extra attempts one call may run in total.
pub(super) const MAX_EXTRA_ATTEMPTS: usize = 2;
/// The extra attempts together may take this many times the first attempt's duration.
pub(super) const EXTRA_TIME_FACTOR: f64 = 2.0;

/// What is left for extra attempts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Budget {
    attempts_left: usize,
    seconds_left: f64,
    /// Seconds the first attempt took: what one extra attempt is expected to take.
    first_seconds: f64,
}

impl Budget {
    /// The budget after a first attempt that took `first_seconds`.
    pub(super) fn after_first(first_seconds: f64) -> Self {
        let first = first_seconds.max(0.0);
        Self { attempts_left: MAX_EXTRA_ATTEMPTS, seconds_left: EXTRA_TIME_FACTOR * first, first_seconds: first }
    }

    /// May one more extra attempt start? It is counted as started: the caller must run it.
    pub(super) fn draw(&mut self) -> bool {
        if self.attempts_left == 0 || self.first_seconds > self.seconds_left {
            return false;
        }
        self.attempts_left -= 1;
        true
    }

    /// Records the time an extra attempt took.
    pub(super) fn spend(&mut self, seconds: f64) {
        self.seconds_left = (self.seconds_left - seconds.max(0.0)).max(0.0);
    }

    /// Why no extra attempt can start, for a skipped entry.
    pub(super) fn refusal(&self) -> String {
        if self.attempts_left == 0 {
            format!("the call has used its {MAX_EXTRA_ATTEMPTS} extra attempts")
        } else {
            format!(
                "the extra attempts of the call may take at most {EXTRA_TIME_FACTOR} times the first attempt, and that time is spent"
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One call never runs more than two extra attempts, whoever draws.
    #[test]
    fn at_most_two_extra_attempts_in_total() {
        let mut b = Budget::after_first(10.0);
        assert!(b.draw());
        b.spend(5.0);
        assert!(b.draw());
        b.spend(5.0);
        assert!(!b.draw(), "a third extra attempt was allowed");
        assert!(b.refusal().contains("extra attempts"));
    }

    /// The extra attempts together take at most twice the first attempt.
    #[test]
    fn the_time_is_bounded_by_the_first_attempt() {
        let mut b = Budget::after_first(10.0);
        assert!(b.draw());
        b.spend(20.0);
        assert!(!b.draw());
        assert!(b.refusal().contains("time"));
    }
}
