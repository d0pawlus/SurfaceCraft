//! SurfaceCraft deviation analysis (layer L2): how far scan points are from a reference, as
//! statistics and a colour map (design doc §7: "deviation as constant feedback").
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Bvh, KdTree, Mesh, Point3, Vec3};

pub mod faces;

mod distribution;
mod hotspot;
mod noise;
/// Colours of the deviation map, linear RGBA in 0..1. [`colorize`] paints the viewport from these
/// constants, and the UI legend resolves the same values through its theme tokens, so the legend
/// and the map cannot drift apart.
pub mod palette;
mod report;
mod sample;
mod stats;
#[cfg(test)]
mod tests;

pub use distribution::{Distribution, HISTOGRAM_BINS, against_mesh, against_primitive, distribution, hist_bin};
pub use hotspot::{Hotspot, HotspotReport, Sign, Volume, hotspots};
pub use noise::{NoiseEstimate, estimate_noise, noise_estimate};
pub use palette::colorize;
pub use report::{Band, Histogram, MAX_BANDS, MAX_BINS, Percentiles, Report, ReportOptions, report};
pub use sample::{
    Sample, counted_values, oriented_plane, primitive_normal, samples_mesh, samples_mesh_tagged, samples_primitive,
    stats_of,
};
pub use stats::{DeviationStats, stats};
