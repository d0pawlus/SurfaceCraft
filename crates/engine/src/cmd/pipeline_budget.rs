//! The budget of extra attempts of one `scan.to_solid` call.
//!
//! The first attempt of a call is the pipeline itself. Anything beyond it is an *extra attempt*: the
//! other method that `auto` tries when the first does not pass, and the freeform fit's retry with a
//! larger patch cap. All of them draw from one budget per call: at most [`MAX_EXTRA_ATTEMPTS`] in
//! total, and, once the first attempt's duration is known, together at most [`EXTRA_TIME_FACTOR`]
//! times as long as it (they cost about as much as it did). A code path that wants to run again
//! asks [`CallBudget::draw`]; the time of a whole extra attempt is reported with
//! [`CallBudget::spend`]. The budget is shared by clone (the settings of the call carry it).
use std::cell::RefCell;
use std::rc::Rc;

/// Extra attempts one call may run in total.
pub(super) const MAX_EXTRA_ATTEMPTS: usize = 2;
/// The extra attempts together may take this many times the first attempt's duration.
pub(super) const EXTRA_TIME_FACTOR: f64 = 2.0;

/// What is left for extra attempts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct Budget {
    attempts_left: usize,
    /// Seconds the extra attempts have taken so far.
    spent: f64,
    /// Seconds the first attempt took, once it has ended.
    first_seconds: Option<f64>,
}

impl Budget {
    /// A budget before the first attempt has ended: the attempts are limited, the time not yet.
    pub(super) fn new() -> Self {
        Self { attempts_left: MAX_EXTRA_ATTEMPTS, spent: 0.0, first_seconds: None }
    }

    /// The first attempt ended after `seconds`: the time limit is known from now on.
    pub(super) fn set_first(&mut self, seconds: f64) {
        self.first_seconds = Some(seconds.max(0.0));
    }

    fn seconds_left(&self) -> f64 {
        self.first_seconds.map_or(f64::INFINITY, |f| (EXTRA_TIME_FACTOR * f - self.spent).max(0.0))
    }

    /// May one more extra attempt start? It is counted as started: the caller must run it.
    pub(super) fn draw(&mut self) -> bool {
        let expected = self.first_seconds.unwrap_or(0.0);
        if self.attempts_left == 0 || expected > self.seconds_left() {
            return false;
        }
        self.attempts_left -= 1;
        true
    }

    /// Records the time an extra attempt took.
    pub(super) fn spend(&mut self, seconds: f64) {
        self.spent += seconds.max(0.0);
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

/// The budget of one call, shared by everything that runs in it.
#[derive(Clone, Debug)]
pub(super) struct CallBudget(Rc<RefCell<Budget>>);

impl Default for CallBudget {
    fn default() -> Self {
        Self(Rc::new(RefCell::new(Budget::new())))
    }
}

impl CallBudget {
    pub(super) fn set_first(&self, seconds: f64) {
        self.0.borrow_mut().set_first(seconds);
    }

    pub(super) fn draw(&self) -> bool {
        self.0.borrow_mut().draw()
    }

    pub(super) fn spend(&self, seconds: f64) {
        self.0.borrow_mut().spend(seconds);
    }

    pub(super) fn refusal(&self) -> String {
        self.0.borrow().refusal()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One call never runs more than two extra attempts, whoever draws.
    #[test]
    fn at_most_two_extra_attempts_in_total() {
        let mut b = Budget::new();
        b.set_first(10.0);
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
        let mut b = Budget::new();
        b.set_first(10.0);
        assert!(b.draw());
        b.spend(20.0);
        assert!(!b.draw());
        assert!(b.refusal().contains("time"));
    }

    /// The freeform retry (inside the first attempt, before its duration is known) and the second
    /// method draw from the same shared budget: together never more than two.
    #[test]
    fn the_freeform_retry_and_the_second_method_share_the_budget() {
        let call = CallBudget::default();
        let in_settings = call.clone();
        assert!(in_settings.draw(), "the retry inside the first attempt");
        call.set_first(3.0);
        assert!(call.draw(), "the second method");
        assert!(!in_settings.draw(), "a retry inside the second method would be a third extra attempt");
    }
}
