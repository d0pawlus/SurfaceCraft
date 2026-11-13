//! The cost model of one solve and the bound on it.
use super::{Problem, Result, arg, couples};

/// Largest estimated cost of one solve, in seconds of the measuring machine (see [`solve_seconds`]).
/// The bound is a work estimate: a pure function of the number of unknowns and hard rows, so the
/// same call is accepted or refused identically on every machine. Measured at the bound (24 × 24,
/// 864 hard scalar rows, estimate 5.7 s): 3.99 s on the development laptop (aarch64, `ci` profile),
/// about 300 MB; the full table is in `vpatch/tests.rs` (`cost_of_coupled_solves`). A web build is
/// expected to be 2 to 3 times slower (not measured); a whole call is at most six rounds.
pub const MAX_SOLVE_SECONDS: f64 = 8.0;
/// Seconds per `c³` (dense products, Cholesky) at `c` unknowns.
const SECONDS_PER_CUBE: f64 = 5e-10;
/// Seconds per `h·c²` (QR reduction of `h` hard scalar rows over `c` unknowns).
const SECONDS_PER_ROW: f64 = 7e-10;
/// Seconds per `min(h, c)³` (SVD of the reduced rows).
const SECONDS_PER_SVD: f64 = 2e-9;

/// Estimated seconds of one solve over `unknowns` with `hard_rows` hard scalar rows, on the
/// machine and build (`ci` profile) the model was fitted on. Measured (aarch64 laptop, one thread,
/// coupled layout of G1/G2 sides): 16 × 16 net (768 unknowns) 0.8 to 1.3 s with 450 rows and 11 s
/// with 27 000 rows; 24 × 24 (1728 unknowns) 4 to 5 s with 700 rows, 26 s with 7 000 and 66 s with
/// 27 000. The model reproduces those within about 25 %; a web build runs it slower (not measured).
#[must_use]
pub fn solve_seconds(unknowns: usize, hard_rows: usize) -> f64 {
    let (c, h) = (unknowns as f64, hard_rows as f64);
    SECONDS_PER_CUBE * c * c * c + SECONDS_PER_ROW * h * c * c + SECONDS_PER_SVD * h.min(c).powi(3)
}

impl Problem {
    /// Do tangent or curvature rows tie the three coordinates together (3 unknowns per control point)?
    #[must_use]
    pub fn is_coupled(&self) -> bool {
        self.hard.iter().any(couples) || self.soft.iter().any(|(r, _)| couples(r))
    }

    /// Unknowns of the solve: three per control point when coupled, one otherwise.
    #[must_use]
    pub fn unknowns(&self) -> usize {
        self.control[0] * self.control[1] * if self.is_coupled() { 3 } else { 1 }
    }

    /// Hard scalar rows of the solve (a position is three rows when coupled).
    #[must_use]
    pub fn hard_scalar_rows(&self) -> usize {
        let coupled = self.is_coupled();
        self.hard.iter().map(|r| if coupled && !couples(r) { 3 } else { 1 }).sum()
    }

    /// [`solve_seconds`] of this problem.
    #[must_use]
    pub fn estimated_seconds(&self) -> f64 {
        solve_seconds(self.unknowns(), self.hard_scalar_rows())
    }
}

/// Refuses a problem whose solve is estimated above [`MAX_SOLVE_SECONDS`], before anything is allocated.
pub(super) fn check_cost(p: &Problem) -> Result<()> {
    let est = p.estimated_seconds();
    if est > MAX_SOLVE_SECONDS {
        return Err(arg(format!(
            "the solve of {} unknowns with {} hard rows would take about {est:.1} s (limit {MAX_SOLVE_SECONDS:.1} s): \
             use a smaller control net or fewer hard conditions",
            p.unknowns(),
            p.hard_scalar_rows()
        )));
    }
    Ok(())
}
