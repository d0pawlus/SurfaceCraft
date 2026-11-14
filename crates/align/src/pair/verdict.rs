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
/// Largest share (0..1) of each scan's area that may lie where the other scan is, but off it, in a
/// pose that is called `unique`: the smaller of the two shares (a conflict of both scans at once).
/// Where two scans really coincide, one side may have surface the other lacks (a hidden face, a
/// part only one scan saw), never both; where the pose merely lays a lumpy or box-like surface on
/// other regions of itself, each scan's features stand where the other has bare surface.
/// Measured on the fixtures of `pair_coarse_wrong_unique` (the confusion table of the pull request):
/// the smaller share is at most 0.008 in every right pose and at least 0.050 in every wrong one.
pub const MAX_MUTUAL_CONFLICT: f64 = 0.025;
/// Stability ([`super::score::stability`]) below which a pose is called `ambiguous` whatever the
/// probes found: the matched part can slide or turn freely. A safeguard, not the decision (every
/// pose is probed). Measured on ambiguous parts: 0.00001 to 0.0004 (cylinder sector, long cylinder,
/// a patch strictly inside a flat face; macOS, Linux, Windows) and, on garbage poses of a patch
/// that nothing holds, up to 0.04; the lowest legitimate pose measured is 0.0116 (a real scan),
/// 11 times above the floor, so the floor never catches one.
pub(super) const STABILITY_FLOOR: f64 = 0.001;
/// Directions beyond the three weakest are probed when their eigenvalue is below this (the three
/// weakest are probed whatever they read). Extra evidence only: no verdict hangs on it.
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
    /// The pose the caller gave (`PairOptions::given`, used by `align.global` for scans placed by hand).
    Given,
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

/// What the probes of a pose found, for the verdict.
#[derive(Clone, Copy)]
pub(super) struct ProbeFacts {
    /// Displaced starts tried.
    pub attempted: usize,
    /// Of those, the ones whose refinement ran.
    pub completed: usize,
    /// Of those, the ones that settled elsewhere as well as the pose.
    pub slides: usize,
}

/// What the verdict rests on.
pub(super) struct Facts<'a> {
    /// The best pose's fit on the moving scan.
    pub best: &'a Fit,
    /// The larger of the two shares of a scan on the other.
    pub overlap: f64,
    /// Tolerance (mm).
    pub tau: f64,
    /// Smallest [`super::score::clearance`] to any other distinct pose (`None` when all others are
    /// the same pose).
    pub clearance: Option<f64>,
    /// Candidates that could be refined; with fewer than two nothing was compared and the answer is
    /// never certain.
    pub refined: usize,
    /// Stability of the pose; below [`STABILITY_FLOOR`] the part leaves the pose free to slide.
    pub stable: f64,
    /// The smaller of the two scans' conflicts (see [`MAX_MUTUAL_CONFLICT`]).
    pub mutual_conflict: f64,
    /// What the probes found.
    pub probes: ProbeFacts,
}

/// Verdict and confidence from the facts of the best pose.
pub(super) fn judge(f: &Facts) -> (PairVerdict, f64) {
    let best = f.best;
    if !(f.overlap >= MIN_OVERLAP) || !(best.rms <= MAX_RMS_FRACTION * f.tau) || !(best.conflict <= MAX_CONFLICT) {
        return (PairVerdict::NoMatch, 0.0);
    }
    if f.clearance.is_none() && f.refined < 2 {
        return (PairVerdict::Ambiguous, 0.0);
    }
    let gap = f.clearance.unwrap_or(1.0);
    let confidence = (gap / FULL_CONFIDENCE_GAP).clamp(0.0, 1.0);
    // a pose that slides, or that was not tested through (no direction, or a start whose ICP could not
    // run), is not certain; nor is one that both scans contradict
    let probes = &f.probes;
    if probes.slides > 0 || probes.attempted == 0 || probes.completed < probes.attempted {
        return (PairVerdict::Ambiguous, confidence.min(0.25));
    }
    if !(f.stable >= STABILITY_FLOOR) || !(f.mutual_conflict <= MAX_MUTUAL_CONFLICT) {
        return (PairVerdict::Ambiguous, confidence.min(0.25));
    }
    let verdict = if confidence >= CONFIDENCE_THRESHOLD { PairVerdict::Unique } else { PairVerdict::Ambiguous };
    (verdict, confidence)
}

#[cfg(test)]
mod tests {
    use super::*;

    const OK: ProbeFacts = ProbeFacts { attempted: 6, completed: 6, slides: 0 };

    fn fit(share: f64, conflict: f64, rms: f64) -> Fit {
        Fit { share, conflict, rms, msac: 0.0 }
    }

    /// The facts of a pose that is right in every respect, with the given parts replaced.
    fn judged(
        best: &Fit,
        overlap: f64,
        clearance: Option<f64>,
        refined: usize,
        stable: f64,
        probes: ProbeFacts,
    ) -> (PairVerdict, f64) {
        judge(&Facts { best, overlap, tau: 0.3, clearance, refined, stable, mutual_conflict: 0.0, probes })
    }

    #[test]
    fn verdicts_follow_overlap_fit_and_the_clearance() {
        let best = fit(0.99, 0.01, 0.05);
        assert_eq!(judged(&best, 0.99, None, 8, 0.1, OK).0, PairVerdict::Unique);
        assert_eq!(judged(&best, 0.99, Some(0.10), 8, 0.1, OK).0, PairVerdict::Unique);
        assert_eq!(judged(&best, 0.99, Some(0.01), 8, 0.1, OK).0, PairVerdict::Ambiguous);
        assert_eq!(judged(&best, 0.99, Some(0.0), 8, 0.1, OK).1, 0.0);
        assert_eq!(judged(&fit(0.1, 0.0, 0.05), 0.2, None, 8, 0.1, OK).0, PairVerdict::NoMatch);
        assert_eq!(judged(&fit(0.9, 0.0, 0.2), 0.9, None, 8, 0.1, OK).0, PairVerdict::NoMatch);
        assert_eq!(judged(&fit(0.5, 0.4, 0.05), 0.5, None, 8, 0.1, OK).0, PairVerdict::NoMatch);
        assert_eq!(judged(&fit(f64::NAN, 0.0, 0.0), f64::NAN, None, 8, 0.1, OK).0, PairVerdict::NoMatch);
    }

    /// A part that can slide is ambiguous however clear the other candidates are.
    #[test]
    fn a_part_that_can_slide_is_ambiguous() {
        let best = fit(0.99, 0.01, 0.05);
        assert_eq!(judged(&best, 0.99, None, 8, 0.0004, OK).0, PairVerdict::Ambiguous);
        assert!(judged(&best, 0.99, None, 8, 0.0004, OK).1 <= 0.25);
        assert_eq!(judged(&best, 0.99, None, 8, f64::NAN, OK).0, PairVerdict::Ambiguous);
    }

    /// One refined candidate means nothing was compared: never certain.
    #[test]
    fn a_single_refined_candidate_is_never_unique() {
        let best = fit(0.99, 0.01, 0.05);
        assert_eq!(judged(&best, 0.99, None, 1, 0.1, OK), (PairVerdict::Ambiguous, 0.0));
        assert_eq!(judged(&best, 0.99, None, 5, 0.1, OK).0, PairVerdict::Unique, "five refined, all the same pose");
    }

    /// Finding 1: a pose whose displaced starts could not all be refined (or none was tried), or that
    /// has slides, is not certain, however clear the candidates are.
    #[test]
    fn untested_or_sliding_poses_are_never_unique() {
        let best = fit(0.99, 0.01, 0.05);
        let failed = ProbeFacts { attempted: 6, completed: 5, slides: 0 };
        let none = ProbeFacts { attempted: 0, completed: 0, slides: 0 };
        let slides = ProbeFacts { attempted: 6, completed: 6, slides: 2 };
        for (name, p) in [("failed", failed), ("none", none), ("slides", slides)] {
            let (verdict, confidence) = judged(&best, 0.99, None, 8, 0.1, p);
            assert_eq!(verdict, PairVerdict::Ambiguous, "{name}");
            assert!(confidence <= 0.25, "{name}: {confidence}");
        }
        assert_eq!(judged(&best, 0.99, None, 8, 0.1, OK).0, PairVerdict::Unique);
    }

    /// A pose both scans contradict is never certain, however clear it stands among the others; a
    /// conflict of one scan alone does not count.
    #[test]
    fn a_pose_that_both_scans_contradict_is_never_unique() {
        let best = fit(0.8, 0.1, 0.05);
        let facts = |mutual_conflict| Facts {
            best: &best,
            overlap: 0.8,
            tau: 0.3,
            clearance: Some(0.5),
            refined: 8,
            stable: 0.1,
            mutual_conflict,
            probes: OK,
        };
        assert_eq!(judge(&facts(0.0)).0, PairVerdict::Unique);
        assert_eq!(judge(&facts(MAX_MUTUAL_CONFLICT)).0, PairVerdict::Unique);
        let (verdict, confidence) = judge(&facts(2.0 * MAX_MUTUAL_CONFLICT));
        assert_eq!(verdict, PairVerdict::Ambiguous);
        assert!(confidence <= 0.25);
        assert_eq!(judge(&facts(f64::NAN)).0, PairVerdict::Ambiguous);
    }
}
