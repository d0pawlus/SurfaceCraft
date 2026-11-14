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
use super::score::{Fit, fit_of, pose_gap, stability};
use super::verdict::WEAK_BELOW;
use super::{Cancel, Cand, Ranked, SAME_POSE_AFTER, Scene};
use crate::AlignError;
use surfacecraft_geom::{Point3, Transform};

/// The displaced start moves a matched point by at most this fraction of the extent of the moving
/// scan: far beyond the tolerance (0.0015 to 0.02 of the diagonal) and the noise, so a start that
/// ends up back is pulled back, not left where noise would leave it; and large enough that a slide
/// into neighbouring features costs more than the ambiguity margin.
pub(super) const PROBE_STEP: f64 = 0.2;
/// The first refinement of a displaced start trims at 1.5 times the displacement (so that the start
/// finds its pairs again), at most this fraction of the diagonal of the fixed scan (beyond it the
/// pairs are noise).
const MAX_FIRST_CUT: f64 = 0.25;
/// A start that settles elsewhere clears the pose only by clashing with the fixed scan: its
/// conflicts (surface where the fixed scan is, but off it, as a share of the moving scan's area)
/// exceed the pose's by more than this many times the spread of that difference among the starts
/// that came back (`NOISE_FACTOR` times, 8), and by at least `MIN_SLIDE`, or it has diverged
/// (`COLLAPSE`); otherwise it is a slide. Running off the border of
/// the fixed scan (a patch that slides to the end of a cylinder) costs matches but is no clash, and
/// is no evidence against the pose. Measured (the gate `gate_m2_loose_poses_are_decided_by_pushing_them`
/// prints it for every case on a coarse and a fine mesh): the spread among the starts that came back
/// is 0 (they return to the same conflicts), every start counted as a slide clashes by 0.0000, and
/// the closest legitimate start that settled elsewhere (the slender bar: its block meets the bar)
/// clashes by 0.062: 3.1 times the least conflict that clears a pose, and the slides are an
/// unbounded factor below it.
const NOISE_FACTOR: f64 = 8.0;
/// A start that settled elsewhere and kept less than this much less of the fit (it lost more than
/// 0.3 of the moving scan's matches) has diverged: it clears the pose by what it lost. Measured: the
/// slides lose at most 0.024 (running off the end of a finite cylinder, 12 times below), a start that
/// flew off a real scan (scan 6 of the corpus) 0.98 (3.3 times above).
const COLLAPSE: f64 = 0.3;
/// The least conflict (share of the moving scan's area) that clears a pose, whatever the spread.
pub(super) const MIN_SLIDE: f64 = 0.02;

/// What the probes found.
pub(super) struct Probe {
    /// Stability of the pose.
    pub stability: f64,
    /// Displaced starts that ended in a different pose that fits as well: slides.
    pub slides: Vec<Ranked>,
    /// Displaced starts tried (twice the number of loose directions, at most 12).
    pub attempted: usize,
    /// Of those, the starts whose refinement ran. A start whose ICP finds no pairs says nothing about
    /// the pose (it may have left the trimming distance); a pose with failed starts is never
    /// `unique`.
    pub completed: usize,
    /// Spread of the conflicts (shares of the moving scan's area) among the starts that came back.
    pub noise: f64,
    /// Clearance below which a start that settled elsewhere is a slide.
    pub threshold: f64,
    /// Smallest clearance of the pose over the starts that settled elsewhere; `None` when all
    /// came back.
    pub margin: Option<f64>,
    /// Largest clearance among the starts counted as slides (0 when none).
    pub slide_clearance: f64,
    /// Largest distance (mm) a start ended from the pose, among those that came back.
    pub returned_gap: f64,
}

/// A displaced start that was refined.
struct Outcome {
    t: Transform,
    fit: Fit,
    gap: f64,
}

impl Scene<'_> {
    /// Tests `best` (already polished) for slides; see the module docs. Every pose is tested, along
    /// its three weakest directions and any further one below `WEAK_BELOW`: no verdict depends on
    /// how loose the pose reads. `step` is the displacement, a fraction of the extent of the moving
    /// scan.
    pub(super) fn probe_at(&self, best: &Ranked, tau: f64, cancel: &Cancel, step: f64) -> Result<Probe, AlignError> {
        let extent = surfacecraft_geom::Aabb::from_points(&self.mov.pts).diagonal();
        let shift = step * extent;
        let st = stability(&self.fixed, &self.fix, &self.mov.pts, &best.cand.t, tau, WEAK_BELOW, shift);
        let first_cut = (1.5 * shift).max(super::REFINE_CUTS[0] * self.diag).min(MAX_FIRST_CUT * self.diag);
        let cuts = [first_cut, super::REFINE_CUTS[1] * self.diag];
        let mut outcomes = Vec::new();
        let mut attempted = 0;
        let corners = super::score::corners(&self.mov.pts);
        for twist in &st.weak {
            for sign in [1.0, -1.0] {
                attempted += 1;
                let start = Cand { t: twist.displace(&best.cand.t, sign), method: best.cand.method };
                let Some(moved) = self.refine_cuts(&start, cuts, cancel)? else { continue };
                let t = self.converge(&moved.t, tau, cancel)?;
                let fit = fit_of(&self.fixed, &self.mov.pts, &t, tau, true);
                outcomes.push(Outcome { gap: pose_gap(&best.cand.t, &t, &corners), t, fit });
            }
        }
        Ok(self.judge_outcomes(best, st.min, attempted, outcomes, &corners))
    }

    /// Sorts the outcomes into those that came back and those that settled elsewhere, measures the
    /// spread of the first and decides which of the second are slides.
    fn judge_outcomes(
        &self,
        best: &Ranked,
        stab: f64,
        attempted: usize,
        outcomes: Vec<Outcome>,
        corners: &[Point3],
    ) -> Probe {
        let back = |o: &Outcome| o.gap <= SAME_POSE_AFTER * self.diag;
        let deviation = |f: &Fit| (f.conflict - best.fit.conflict).abs();
        let noise = outcomes.iter().filter(|o| back(o)).map(|o| deviation(&o.fit)).fold(0.0, f64::max);
        let threshold = MIN_SLIDE.max(NOISE_FACTOR * noise);
        let returned_gap = outcomes.iter().filter(|o| back(o)).map(|o| o.gap).fold(0.0, f64::max);
        let mut probe = Probe {
            stability: stab,
            slides: Vec::new(),
            attempted,
            completed: outcomes.len(),
            noise,
            threshold,
            margin: None,
            slide_clearance: 0.0,
            returned_gap,
        };
        for o in outcomes.iter().filter(|o| !back(o)) {
            let loss = best.fit.share - o.fit.share;
            let clear = (o.fit.conflict - best.fit.conflict).max(if loss > COLLAPSE { loss } else { 0.0 });
            probe.margin = Some(probe.margin.map_or(clear, |m| m.min(clear)));
            let known = probe.slides.iter().any(|r| pose_gap(&r.cand.t, &o.t, corners) <= SAME_POSE_AFTER * self.diag);
            if clear < threshold {
                probe.slide_clearance = probe.slide_clearance.max(clear);
            }
            if clear < threshold && !known {
                probe.slides.push(Ranked { cand: Cand { t: o.t, method: best.cand.method }, fit: o.fit, slide: true });
            }
        }
        probe
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
        let probe = scene.probe_at(&best[0], tau, &Cancel::new(), PROBE_STEP).unwrap();
        assert_eq!(probe.attempted, 6, "the three weakest directions, both senses");
        assert!(probe.slides.is_empty(), "{} slides", probe.slides.len());
        assert!(probe.margin.is_none_or(|m| m >= 2.0 * probe.threshold), "{:?}", probe.margin);
    }
}
