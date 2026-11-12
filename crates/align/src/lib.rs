//! SurfaceCraft alignment (layer L1), design doc §6.1.
//!
//! - [`kabsch`]: the rigid motion that best maps point pairs (N-point alignment), by SVD.
//! - [`icp`]: fine alignment of a mesh to a target surface, point-to-plane, on a BVH.
//! - [`datum`]: 3-2-1 frames from planes, axes and points (primary / secondary / tertiary).
//! - [`principal`]: principal axes of a mesh (align to world axes).
//! - [`auto`]: a pose from the scan alone (largest flat face, turned axis, open rims).
//! - [`settle`]: the pose that puts the mirror plane on a world plane.
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

pub mod auto;
pub mod datum;
mod faces;
mod frame;
pub mod icp;
pub mod kabsch;
pub mod principal;
mod rim;
pub mod settle;
pub mod symmetry;
mod turned;

pub use auto::{AutoMethod, AutoPose, BaseFace, CONFIDENCE_THRESHOLD, LevelMode, auto_pose};
pub use datum::{Datum, datum_321};
pub use frame::InPlane;
pub use icp::{IcpOptions, IcpReport, icp, icp_cancellable, rms_to_surface};
pub use kabsch::kabsch;
pub use principal::principal_frame;
pub use settle::{Settled, WorldPlane, settle_symmetry};
pub use symmetry::{SymmetryPlane, symmetry_plane};

/// Why an alignment could not be computed.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum AlignError {
    /// Not enough input (points, pairs, overlap).
    #[error("not enough data: {0}")]
    NotEnough(String),
    /// The input is degenerate (collinear points, parallel planes…).
    #[error("degenerate input: {0}")]
    Degenerate(String),
    /// The caller cancelled the computation.
    #[error("cancelled")]
    Cancelled,
}

/// Plane through points with the normal oriented along `orient` (PCA).
pub(crate) fn fit_plane_for_symmetry(
    pts: &[surfacecraft_geom::Point3],
    orient: &surfacecraft_geom::Vec3,
) -> Option<(surfacecraft_geom::Point3, surfacecraft_geom::Vec3)> {
    surfacecraft_fit::plane::fit_plane(pts, Some(orient)).map(|f| (f.point, f.normal))
}
