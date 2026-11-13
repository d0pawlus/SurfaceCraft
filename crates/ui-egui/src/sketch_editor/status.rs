//! The solver's status as the editor shows it: a catalogue key, a colour token and the numbers.
use crate::theme::Token;
use surfacecraft_sketch::{SolveReport, SolveStatus};

/// What the solver says, as a catalogue key, a theme token and the numbers to show.
#[derive(Clone, Debug, PartialEq)]
pub struct StatusView {
    /// The catalogue key of the status line (`{dof}` and `{n}` placeholders).
    pub key: &'static str,
    /// The token of its colour.
    pub token: Token,
    /// Remaining degrees of freedom.
    pub dof: usize,
    /// Constraints that contradict the others (indices).
    pub conflicting: Vec<usize>,
    /// Constraints that add nothing (indices; includes the conflicting).
    pub redundant: Vec<usize>,
}

/// The status of the last solve (`None`: the sketch was never solved).
#[must_use]
pub fn status_view(report: Option<&SolveReport>) -> StatusView {
    let Some(r) = report else {
        return StatusView {
            key: "sketch.status.unsolved",
            token: Token::TextMuted,
            dof: 0,
            conflicting: Vec::new(),
            redundant: Vec::new(),
        };
    };
    let (key, token) = match r.status {
        SolveStatus::FullyDefined => ("sketch.status.fully", Token::StatusOk),
        SolveStatus::UnderDefined => ("sketch.status.under", Token::StatusWarning),
        SolveStatus::OverDefined => ("sketch.status.over", Token::StatusError),
        SolveStatus::Failed => ("sketch.status.failed", Token::StatusError),
    };
    StatusView { key, token, dof: r.dof, conflicting: r.conflicting.clone(), redundant: r.redundant.clone() }
}
