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

pub mod dxf;
pub mod iges;
pub mod mesh;
pub mod step;
pub mod svg;

pub use mesh::{Limits, MeshFormat, Scan, read_mesh, read_points, read_scan, write_mesh};

/// A polyline of a 2-D drawing, in plane coordinates (millimetres).
#[derive(Clone, Debug, PartialEq)]
pub struct DrawPath {
    /// Points in order (a path with fewer than two points is not written).
    pub points: Vec<[f64; 2]>,
    /// Does the path close on itself?
    pub closed: bool,
}

/// One named layer of a 2-D drawing (a section level).
#[derive(Clone, Debug, PartialEq)]
pub struct DrawLayer {
    /// Layer name; characters the target format does not allow are replaced.
    pub name: String,
    /// The paths of the layer.
    pub paths: Vec<DrawPath>,
}

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
