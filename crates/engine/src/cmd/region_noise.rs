//! The scanner noise `region.auto_segment` and `region.resegment` hand to the segmentation: given
//! by the caller, measured on the scan (never derived from the fits of the regions), or a
//! documented fallback.
use super::{CmdError, CmdResult};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_deviation::noise_estimate;
use surfacecraft_fit::segment::{SegmentOptions, fallback_noise};
use surfacecraft_geom::Mesh;

/// Where the noise used by the segmentation came from.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum NoiseSource {
    /// Measured on the scan (`deviation.noise`).
    Estimated,
    /// Passed in `options.noise`.
    Given,
    /// No reliable measurement: a quarter of the tolerance, with which only small regions are
    /// consolidated into freeform areas.
    Fallback,
}

/// The noise used and where it came from.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(super) struct NoiseUsed {
    pub sigma: f64,
    pub source: NoiseSource,
}

/// Options for segmenting `mesh` with the noise filled in, and the noise used; `None` for the
/// report when the freeform consolidation (the only user of the noise) is off.
pub(super) fn with_noise(mesh: &Mesh, options: &SegmentOptions) -> CmdResult<(SegmentOptions, Option<NoiseUsed>)> {
    if let Some(n) = options.noise
        && !(n.is_finite() && n > 0.0)
    {
        return Err(CmdError::Params("noise must be a positive number of millimetres".into()));
    }
    if !options.freeform {
        return Ok((options.clone(), None));
    }
    if let Some(sigma) = options.noise {
        return Ok((options.clone(), Some(NoiseUsed { sigma, source: NoiseSource::Given })));
    }
    // a coarse mesh of a solid has no smooth samples: the estimate is then inflated by its edges
    let e = noise_estimate(mesh);
    if e.smooth && e.sigma.is_finite() && e.sigma > 0.0 {
        let filled = SegmentOptions { noise: Some(e.sigma), ..options.clone() };
        return Ok((filled, Some(NoiseUsed { sigma: e.sigma, source: NoiseSource::Estimated })));
    }
    let sigma = fallback_noise(options.tolerance);
    Ok((options.clone(), Some(NoiseUsed { sigma, source: NoiseSource::Fallback })))
}
