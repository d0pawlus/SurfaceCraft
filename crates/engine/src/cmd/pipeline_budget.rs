//! The budget of extra attempts of one `scan.to_solid` call.
//!
//! The first attempt of a call is the pipeline itself. Anything beyond it is an *extra attempt*: the
//! other method that `auto` tries when the first does not pass, and the freeform fit's retry with a
//! larger patch cap. All of them draw from one budget per call: at most [`MAX_EXTRA_ATTEMPTS`] in
//! total. There is no time bound: a wall-clock limit would make the result depend on how fast or
//! how loaded the machine is. (The seconds of each attempt are recorded in the output only.) A code
//! path that wants to run again asks [`CallBudget::draw`]; the budget is shared by clone (the
//! settings of the call carry it).
use std::cell::Cell;
use std::rc::Rc;

/// Extra attempts one call may run in total.
pub(super) const MAX_EXTRA_ATTEMPTS: usize = 2;

/// The budget of one call, shared by everything that runs in it.
#[derive(Clone, Debug)]
pub(super) struct CallBudget(Rc<Cell<usize>>);

impl Default for CallBudget {
    fn default() -> Self {
        Self(Rc::new(Cell::new(MAX_EXTRA_ATTEMPTS)))
    }
}

impl CallBudget {
    /// May one more extra attempt start? It is counted as started: the caller must run it.
    pub(super) fn draw(&self) -> bool {
        let left = self.0.get();
        if left == 0 {
            return false;
        }
        self.0.set(left - 1);
        true
    }

    /// Why no extra attempt can start, for a skipped entry.
    pub(super) fn refusal(&self) -> String {
        format!("the call has used its {MAX_EXTRA_ATTEMPTS} extra attempts")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// One call never runs more than two extra attempts, whoever draws.
    #[test]
    fn at_most_two_extra_attempts_in_total() {
        let b = CallBudget::default();
        assert!(b.draw() && b.draw());
        assert!(!b.draw(), "a third extra attempt was allowed");
        assert!(b.refusal().contains("extra attempts"));
    }

    /// The freeform retry (inside the first attempt) and the second method draw from the same
    /// shared budget: together never more than two; a clone is the same budget.
    #[test]
    fn the_freeform_retry_and_the_second_method_share_the_budget() {
        let call = CallBudget::default();
        let in_settings = call.clone();
        assert!(in_settings.draw(), "the retry inside the first attempt");
        assert!(call.draw(), "the second method");
        assert!(!in_settings.draw(), "a retry inside the second method would be a third extra attempt");
    }

    /// A new call has a new budget.
    #[test]
    fn every_call_starts_with_a_full_budget() {
        let first = CallBudget::default();
        assert!(first.draw() && first.draw() && !first.draw());
        assert!(CallBudget::default().draw());
    }
}
