//! SurfaceCraft engine (layer L4): the [`Session`] and its command registry.
//!
//! Every user action is a command (`area.verb`) with a JSON input and output and a JSON Schema
//! for each (spike §3.5). The desktop toolbar and palette, the web build, `surfacecraft-cli run`
//! scripts and the MCP server all call [`Session::execute`]; programmatic calls never open
//! dialogs. Undoable commands snapshot the document first and roll back when they fail.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod cmd;
pub mod display;
pub mod job;
pub mod replay;
pub mod session;

pub use cmd::{CmdError, CommandSpec, registry};
pub use display::DisplayItem;
pub use job::{Finished, Job, JobPoll};
pub use session::{LogEntry, ScriptReport, ScriptStep, Session};

/// An error from executing a command, as reported to every front end.
#[derive(Clone, Debug, PartialEq, thiserror::Error, serde::Serialize)]
#[serde(tag = "kind", content = "message", rename_all = "snake_case")]
pub enum EngineError {
    /// No command has this id.
    #[error("unknown command `{0}`")]
    UnknownCommand(String),
    /// The parameters do not match the command's input schema.
    #[error("invalid parameters: {0}")]
    InvalidParams(String),
    /// The command ran and failed; the document is unchanged.
    #[error("{0}")]
    Failed(String),
    /// The command is not available now (e.g. nothing to undo).
    #[error("not available: {0}")]
    Unavailable(String),
    /// A bug: the command panicked. The document was rolled back.
    #[error("internal error in `{0}` (rolled back)")]
    Internal(String),
}
