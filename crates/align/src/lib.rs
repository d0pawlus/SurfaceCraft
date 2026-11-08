//! SurfaceCraft alignment (layer L1), design doc §6.1.
//!
//! - [`kabsch`]: the rigid motion that best maps point pairs (N-point alignment), by SVD.
//! - [`icp`]: fine alignment of a mesh to a target surface, point-to-plane, on a BVH.
//! - [`datum`]: 3-2-1 frames from planes, axes and points (primary / secondary / tertiary).
//! - [`principal`]: principal axes of a mesh (align to world axes).
//!
//! All of it is mesh-side computation (design doc §5), no B-rep geometry.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod datum;
pub mod icp;
pub mod kabsch;
pub mod principal;

pub use datum::{Datum, datum_321};
pub use icp::{IcpOptions, IcpReport, icp};
pub use kabsch::kabsch;
pub use principal::principal_frame;

/// Why an alignment could not be computed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum AlignError {
    /// Not enough input (points, pairs, overlap).
    #[error("not enough data: {0}")]
    NotEnough(String),
    /// The input is degenerate (collinear points, parallel planes…).
    #[error("degenerate input: {0}")]
    Degenerate(String),
}
