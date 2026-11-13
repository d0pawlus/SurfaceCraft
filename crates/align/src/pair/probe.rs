//! The direct test for a pose that is only weakly pinned: displace it along each loose direction,
//! in both senses, refine from there and see where it ends.
//!
//! The stability of a pose (the smallest eigenvalue of its point-to-plane system) says how loose
//! it is, but constraint strength alone cannot tell a patch that slides from one that is held by
//! a few slivers of neighbouring faces, or by a gentle curve: both read a few thousandths. What
//! tells them apart is what happens when the pose is pushed: a held pose is pulled back (or the
//! displaced one fits clearly worse), a sliding one stays where it was put and fits as well.
//! The outcome is a large effect (the pose returns, or it does not), so it does not depend on
//! rounding.
use super::score::{clearance, fit_of, pose_gap, stability};
use super::verdict::{STABILITY_FLOOR, WEAK_BELOW};
use super::{AMBIGUITY_MARGIN, Cancel, Cand, Ranked, SAME_POSE_AFTER, Scene};
use crate::AlignError;
use surfacecraft_geom::Transform;

/// The displaced start moves a matched point by at most this fraction of the extent of the moving
/// scan: far beyond the tolerance (0.0015 to 0.02 of the diagonal) and the noise, so a start that
/// ends up back is pulled back, not left where noise would leave it; and large enough that a slide
/// into neighbouring features costs more than the ambiguity margin.
const PROBE_STEP: f64 = 0.2;

/// What the probes found.
pub(super) struct Probe {
    /// Stability of the pose.
    pub stability: f64,
    /// Displaced starts that ended in a different pose that fits as well: slides.
    pub slides: Vec<Ranked>,
    /// Displaced starts run (twice the number of loose directions, at most 12).
    pub tested: usize,
    /// Smallest clearance of the best pose over the displaced starts that ended elsewhere (the
    /// ambiguity margin is the bound it has to stay above); `None` when all came back.
    pub margin: Option<f64>,
    /// Largest distance (mm) a displaced start ended from the pose, among those that came back.
    pub returned_gap: f64,
}

impl Scene<'_> {
    /// Tests `best` (already polished) for slides; see the module docs. Poses with a stability
    /// above `WEAK_BELOW` are taken as pinned, those below `STABILITY_FLOOR` as free, without a
    /// test.
    pub(super) fn probe(&self, best: &Ranked, tau: f64, cancel: &Cancel) -> Result<Probe, AlignError> {
        let extent = surfacecraft_geom::Aabb::from_points(&self.mov.pts).diagonal();
        let st = stability(&self.fixed, &self.fix, &self.mov.pts, &best.cand.t, tau, WEAK_BELOW, PROBE_STEP * extent);
        let mut probe = Probe { stability: st.min, slides: Vec::new(), tested: 0, margin: None, returned_gap: 0.0 };
        if st.min < STABILITY_FLOOR {
            return Ok(probe);
        }
        let probes = super::score::corners(&self.mov.pts);
        for twist in &st.weak {
            for sign in [1.0, -1.0] {
                probe.tested += 1;
                let start = Cand { t: twist.displace(&best.cand.t, sign), method: best.cand.method };
                let Some(moved) = self.refine(&start, cancel)? else { continue };
                let t = self.converge(&moved.t, tau, cancel)?;
                let fit = fit_of(&self.fixed, &self.mov.pts, &t, tau, true);
                let gap = pose_gap(&best.cand.t, &t, &probes);
                if gap <= SAME_POSE_AFTER * self.diag {
                    probe.returned_gap = probe.returned_gap.max(gap);
                    continue;
                }
                let clear = clearance(&best.fit, &fit);
                probe.margin = Some(probe.margin.map_or(clear, |m| m.min(clear)));
                if clear < AMBIGUITY_MARGIN && !self.known(&probe.slides, &t, &probes) {
                    probe.slides.push(Ranked { cand: Cand { t, method: best.cand.method }, fit });
                }
            }
        }
        Ok(probe)
    }

    /// Is `t` the pose of one of `found`?
    fn known(&self, found: &[Ranked], t: &Transform, probes: &[surfacecraft_geom::Point3]) -> bool {
        found.iter().any(|r| pose_gap(&r.cand.t, t, probes) <= SAME_POSE_AFTER * self.diag)
    }
}
