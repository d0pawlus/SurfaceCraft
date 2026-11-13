//! The outcome of a pair alignment and when it is trusted.
use super::score::Fit;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_geom::Transform;

/// Smallest share (0..1) of either scan that must lie on the other for a match to count.
pub const MIN_OVERLAP: f64 = 0.3;
/// A second, different pose that lacks fewer matches than this and has fewer conflicts than the
/// best one by less than this (both as shares, 0..1, of the moving scan's area) makes the answer
/// ambiguous.
pub const AMBIGUITY_MARGIN: f64 = 0.03;
/// Confidence from which the answer counts as certain.
pub const CONFIDENCE_THRESHOLD: f64 = 0.5;
/// Largest share of the moving scan that may lie where the fixed scan is but off it.
pub(super) const MAX_CONFLICT: f64 = 0.25;
/// Stability ([`super::score::stability`]) below which the matched part can slide or turn freely
/// and nothing is tried: `ambiguous`. Measured: such parts read 0.00001 to 0.0004 (cylinder
/// sector, long cylinder, a patch strictly inside a flat face, on Linux, macOS and Windows), and
/// the least pinned part that must be found 0.0116 (a real scan): a factor 2.5 above the highest
/// free one, which does not matter (a free part between the two would be probed and found
/// ambiguous anyway), and 11 below the lowest legitimate one, which does (it must not be caught).
pub(super) const STABILITY_FLOOR: f64 = 0.001;
/// Poses with a stability up to this are probed for slides (see `probe`); above it they are pinned
/// as far as can matter. Measured legitimate poses: 0.0116 and up, of which only the weakest few
/// fall below (3.4 times the lowest); every pose above is taken as pinned, the weakest of those
/// measured being a plate with a bump and partial scans at 0.05.
pub(super) const WEAK_BELOW: f64 = 0.04;
/// Largest RMS of the matched part as a fraction of the tolerance.
pub(super) const MAX_RMS_FRACTION: f64 = 0.5;
/// Clearance (shares of the moving scan's area) that gives full confidence.
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
    /// Share (0..1) of the fixed scan's area within the tolerance of the moving scan at this pose.
    pub overlap_fixed: f64,
    /// Confidence of this pose taken as the answer: 0 for an alternative (it is near-tied with the best).
    pub confidence: f64,
    /// RMS distance (mm) of the matched part.
    pub rms: f64,
    /// Matches minus weighted conflicts of the moving scan (about the overlap share when the fixed
    /// scan has no borders): higher is better.
    pub score: f64,
    /// The stage that proposed it.
    pub method: CoarseMethod,
}

/// Verdict and confidence from the facts of the best pose: `overlap` is the larger of the two
/// shares, `tau` is in mm, `clearance` is the smallest [`super::score::clearance`] to any other
/// distinct pose (`None` when all others are the same pose), `refined` the number of candidates
/// that could be refined: with fewer than two, nothing was compared and the answer is never certain;
/// `stable` below [`STABILITY_FLOOR`] means the part leaves the pose free to slide: ambiguous.
pub(super) fn judge(
    best: &Fit,
    overlap: f64,
    tau: f64,
    clearance: Option<f64>,
    refined: usize,
    stable: f64,
) -> (PairVerdict, f64) {
    if !(overlap >= MIN_OVERLAP) || !(best.rms <= MAX_RMS_FRACTION * tau) || !(best.conflict <= MAX_CONFLICT) {
        return (PairVerdict::NoMatch, 0.0);
    }
    if clearance.is_none() && refined < 2 {
        return (PairVerdict::Ambiguous, 0.0);
    }
    let gap = clearance.unwrap_or(1.0);
    let confidence = (gap / FULL_CONFIDENCE_GAP).clamp(0.0, 1.0);
    if !(stable >= STABILITY_FLOOR) {
        return (PairVerdict::Ambiguous, confidence.min(0.25));
    }
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
    fn verdicts_follow_overlap_fit_and_the_clearance() {
        let best = fit(0.99, 0.01, 0.05);
        assert_eq!(judge(&best, 0.99, 0.3, None, 8, 0.1).0, PairVerdict::Unique);
        assert_eq!(judge(&best, 0.99, 0.3, Some(0.10), 8, 0.1).0, PairVerdict::Unique);
        assert_eq!(judge(&best, 0.99, 0.3, Some(0.01), 8, 0.1).0, PairVerdict::Ambiguous);
        assert_eq!(judge(&best, 0.99, 0.3, Some(0.0), 8, 0.1).1, 0.0);
        assert_eq!(judge(&fit(0.1, 0.0, 0.05), 0.2, 0.3, None, 8, 0.1).0, PairVerdict::NoMatch);
        assert_eq!(judge(&fit(0.9, 0.0, 0.2), 0.9, 0.3, None, 8, 0.1).0, PairVerdict::NoMatch);
        assert_eq!(judge(&fit(0.5, 0.4, 0.05), 0.5, 0.3, None, 8, 0.1).0, PairVerdict::NoMatch);
        assert_eq!(judge(&fit(f64::NAN, 0.0, 0.0), f64::NAN, 0.3, None, 8, 0.1).0, PairVerdict::NoMatch);
    }

    /// A part that can slide is ambiguous however clear the other candidates are.
    #[test]
    fn a_part_that_can_slide_is_ambiguous() {
        let best = fit(0.99, 0.01, 0.05);
        assert_eq!(judge(&best, 0.99, 0.3, None, 8, 0.0004).0, PairVerdict::Ambiguous);
        assert!(judge(&best, 0.99, 0.3, None, 8, 0.0004).1 <= 0.25);
        assert_eq!(judge(&best, 0.99, 0.3, None, 8, f64::NAN).0, PairVerdict::Ambiguous);
    }

    /// One refined candidate means nothing was compared: never certain.
    #[test]
    fn a_single_refined_candidate_is_never_unique() {
        let best = fit(0.99, 0.01, 0.05);
        assert_eq!(judge(&best, 0.99, 0.3, None, 1, 0.1), (PairVerdict::Ambiguous, 0.0));
        assert_eq!(judge(&best, 0.99, 0.3, None, 5, 0.1).0, PairVerdict::Unique, "five refined, all the same pose");
    }
}
