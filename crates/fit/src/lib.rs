//! SurfaceCraft fitting (layer L1): least-squares fits on mesh points (design doc §6.2).
//! Mesh-side computation; B-rep/NURBS construction goes through the kernel (design doc §5).
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod plane;

pub use plane::{PlaneFit, fit_plane};
