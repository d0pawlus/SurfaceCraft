//! The outcome of a pair alignment and when it is trusted.
use super::score::Fit;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::Transform;

/// Smallest share (0..1) of either scan that must lie on the other for a match to count.
pub const MIN_OVERLAP: f64 = 0.3;
/// A second, different pose that explains the moving scan this much (share of its area) as well as
/// the best one makes the answer ambiguous.
pub const AMBIGUITY_MARGIN: f64 = 0.03;
/// Confidence from which the answer counts as certain.
pub const CONFIDENCE_THRESHOLD: f64 = 0.5;
/// Largest share of the moving scan that may lie where the fixed scan is but off it.
pub(super) const MAX_CONFLICT: f64 = 0.25;
/// Largest RMS of the matched part as a fraction of the tolerance.
pub(super) const MAX_RMS_FRACTION: f64 = 0.5;
/// Share gap (of the moving scan's area) that gives full confidence.
const FULL_CONFIDENCE_GAP: f64 = 2.0 * AMBIGUITY_MARGIN;

/// Whether the pose found can be trusted.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum PairVerdict {
    /// One pose clearly explains the overlap better than any other.
    Unique,
    /// Several different poses fit about equally well (a symmetric part, or an overlap too
    /// featureless to fix the pose): see the alternatives.
    Ambiguous,
    /// No pose brings enough of the two scans together: too little overlap, or different parts.
    NoMatch,
}

/// Which stage proposed the winning pose.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum CoarseMethod {
    /// A signed permutation of the two principal frames.
    PrincipalAxes,
    /// Local descriptor matches and RANSAC.
    Descriptors,
}

/// A pose and how well it fits.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct PairCandidate {
    /// Motion (moving → fixed).
    #[schemars(with = "[[f64; 4]; 4]")]
    pub transform: Transform,
    /// Share (0..1) of the moving scan's area within the tolerance of the fixed scan.
    pub overlap_moving: f64,
    /// RMS distance (mm) of the matched part.
    pub rms: f64,
    /// Matches minus weighted conflicts of the moving scan (about the overlap share when the fixed
    /// scan has no borders): higher is better.
    pub score: f64,
    /// The stage that proposed it.
    pub method: CoarseMethod,
}

/// Verdict and confidence from the facts of the best pose: `overlap` is the larger of the two
/// shares, `tau` is in mm, `runner_up` is the best [`Fit::score`] any different pose reaches (`None`
/// when no other pose survived).
pub(super) fn judge(best: &Fit, overlap: f64, tau: f64, runner_up: Option<f64>) -> (PairVerdict, f64) {
    if !(overlap >= MIN_OVERLAP) || !(best.rms <= MAX_RMS_FRACTION * tau) || !(best.conflict <= MAX_CONFLICT) {
        return (PairVerdict::NoMatch, 0.0);
    }
    let gap = runner_up.map_or(1.0, |r| best.score() - r);
    let confidence = (gap / FULL_CONFIDENCE_GAP).clamp(0.0, 1.0);
    let verdict = if confidence >= CONFIDENCE_THRESHOLD { PairVerdict::Unique } else { PairVerdict::Ambiguous };
    (verdict, confidence)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fit(share: f64, conflict: f64, rms: f64) -> Fit {
        Fit { share, conflict, rms, msac: 0.0 }
    }

    #[test]
    fn verdicts_follow_overlap_fit_and_the_runner_up() {
        let best = fit(0.99, 0.01, 0.05);
        let s = best.score();
        assert_eq!(judge(&best, 0.99, 0.3, None).0, PairVerdict::Unique);
        assert_eq!(judge(&best, 0.99, 0.3, Some(s - 0.10)).0, PairVerdict::Unique);
        assert_eq!(judge(&best, 0.99, 0.3, Some(s - 0.01)).0, PairVerdict::Ambiguous);
        assert_eq!(judge(&best, 0.99, 0.3, Some(s)).1, 0.0);
        assert_eq!(judge(&fit(0.1, 0.0, 0.05), 0.2, 0.3, None).0, PairVerdict::NoMatch);
        assert_eq!(judge(&fit(0.9, 0.0, 0.2), 0.9, 0.3, None).0, PairVerdict::NoMatch);
        assert_eq!(judge(&fit(0.5, 0.4, 0.05), 0.5, 0.3, None).0, PairVerdict::NoMatch);
        assert_eq!(judge(&fit(f64::NAN, 0.0, 0.0), f64::NAN, 0.3, None).0, PairVerdict::NoMatch);
    }
}
