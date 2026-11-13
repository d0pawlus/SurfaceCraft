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
use super::verdict::WEAK_BELOW;
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
    /// Tests `best` (already polished) for slides; see the module docs. Every pose is tested, along
    /// its three weakest directions and any further one below `WEAK_BELOW`: no verdict depends on
    /// how loose the pose reads.
    pub(super) fn probe(&self, best: &Ranked, tau: f64, cancel: &Cancel) -> Result<Probe, AlignError> {
        let extent = surfacecraft_geom::Aabb::from_points(&self.mov.pts).diagonal();
        let st = stability(&self.fixed, &self.fix, &self.mov.pts, &best.cand.t, tau, WEAK_BELOW, PROBE_STEP * extent);
        let mut probe = Probe { stability: st.min, slides: Vec::new(), tested: 0, margin: None, returned_gap: 0.0 };
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pair::{CoarseMethod, PairOptions, PairVerdict, solve};
    use surfacecraft_geom::{Mesh, Point3, Rng, Vec3, synth};

    fn noisy(mut m: Mesh, seed: u64) -> Mesh {
        synth::add_noise(&mut m, 0.03, &mut Rng::new(seed));
        m
    }

    /// The pose "as built" (identity) as the only candidate, solved with the probes.
    fn alone(moving: &Mesh, fixed: &Mesh) -> crate::pair::PairResult {
        let scene = Scene::new(moving, fixed, &Cancel::new()).unwrap();
        let only = Cand { t: Transform::identity(), method: CoarseMethod::PrincipalAxes };
        solve(&scene, vec![only], &PairOptions::default(), &Cancel::new()).unwrap()
    }

    /// A patch strictly inside a plane slides: shifts and a turn are all free. With one candidate
    /// there are no candidate alternatives, so every alternative is a slide found by the probe.
    #[test]
    fn a_plane_patch_on_a_plane_slides() {
        let fixed = noisy(synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 60.0, 40.0, 24, 16), 1);
        let patch = noisy(synth::plane(Point3::new(20.0, 10.0, 0.0), Vec3::x(), Vec3::y(), 22.0, 16.0, 11, 8), 2);
        let r = alone(&patch, &fixed);
        assert!(r.slides >= 3, "{} slides of {} starts", r.slides, r.probed);
        assert_eq!(r.verdict, PairVerdict::Ambiguous);
        assert!(!r.alternatives.is_empty());
    }

    /// A band of a cylinder slides along the axis and around it.
    #[test]
    fn a_cylinder_band_slides_along_and_around_the_axis() {
        let fixed = noisy(synth::cylinder(Point3::origin(), Vec3::z(), 10.0, 60.0, 48, 24), 1);
        let band = noisy(synth::cylinder(Point3::new(0.0, 0.0, 20.0), Vec3::z(), 10.0, 20.0, 40, 10), 2);
        let r = alone(&band, &fixed);
        assert!(r.slides >= 2, "{} slides of {} starts", r.slides, r.probed);
        assert_eq!(r.verdict, PairVerdict::Ambiguous);
    }

    /// A pinned shape: every displaced start comes back (or fits clearly worse), no slide.
    #[test]
    fn a_pinned_shape_gives_no_slide() {
        let fixed = noisy(synth::bracket(), 1);
        let moving = noisy(synth::bracket(), 2);
        let scene = Scene::new(&moving, &fixed, &Cancel::new()).unwrap();
        let tau = 0.3;
        let best = scene.rank(vec![Cand { t: Transform::identity(), method: CoarseMethod::PrincipalAxes }], tau);
        let probe = scene.probe(&best[0], tau, &Cancel::new()).unwrap();
        assert_eq!(probe.tested, 6, "the three weakest directions, both senses");
        assert!(probe.slides.is_empty(), "{} slides", probe.slides.len());
        assert!(probe.margin.is_none_or(|m| m >= 2.0 * AMBIGUITY_MARGIN), "{:?}", probe.margin);
    }
}
