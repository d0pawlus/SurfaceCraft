//! SurfaceCraft file I/O (layer L3).
//!
//! Every reader treats its bytes as hostile (spike §3.2 item 7): no panics, no indexing on
//! input-derived values, counts checked against the data actually present before anything is
//! allocated, and hard limits from [`Limits`].
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod iges;
pub mod mesh;
pub mod step;

pub use mesh::{Limits, MeshFormat, Scan, read_mesh, read_points, read_scan, write_mesh};

/// Why a file could not be read or written.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum IoError {
    /// The data is not a valid file of the expected format.
    #[error("{format}: {message}")]
    Malformed {
        /// Format name.
        format: &'static str,
        /// What is wrong.
        message: String,
    },
    /// The file exceeds a configured limit.
    #[error("{0}")]
    Limit(String),
    /// The format is unknown or unsupported here.
    #[error("unsupported format: {0}")]
    Unsupported(String),
    /// A writer could not produce output.
    #[error("export failed: {0}")]
    Export(String),
}

pub(crate) fn bad(format: &'static str, message: impl Into<String>) -> IoError {
    IoError::Malformed { format, message: message.into() }
}
