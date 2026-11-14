//! The noise of a scan, as far as the tolerance may rest on it.
use surfacecraft_fit::noise::noise_estimate;
use surfacecraft_geom::Mesh;

/// The noise estimate of one scan (see `deviation.noise`) and whether it can be trusted.
/// Estimated once per scan: `align.global` hands the estimates of its scans to every pair.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScanNoise {
    /// Estimated sigma, mm.
    pub sigma: f64,
    /// False when too few smooth samples remained: the estimate then includes the curvature and
    /// the edges of a coarse mesh and is inflated (a tube of a ring at 3.5 mm triangles reads 0.12
    /// for a true 0.05).
    pub smooth: bool,
}

impl ScanNoise {
    /// Estimates the noise of `mesh` (builds a k-d tree over its vertices).
    #[must_use]
    pub fn of(mesh: &Mesh) -> Self {
        let e = noise_estimate(mesh);
        ScanNoise { sigma: e.sigma, smooth: e.smooth }
    }

    fn trusted(&self) -> Option<f64> {
        (self.smooth && self.sigma.is_finite() && self.sigma >= 0.0).then_some(self.sigma)
    }
}

/// The noise of the pair: the two estimates combined as a root of squares, or `None` when either is
/// not trustworthy. An inflated estimate must not set the cap of the tolerance (a coarse mesh of a
/// tube reads 0.12 for a true 0.05, a cap of 0.5 mm instead of 0.2), and the lowest tolerance is no
/// answer either: on the same meshes it is below the chord error between two tessellations of one
/// surface, and the true pose stops fitting. With no estimate the tolerance has no noise cap, and
/// the conflicts of both scans carry the verdict.
pub(super) fn pair_noise(moving: &ScanNoise, fixed: &ScanNoise) -> Option<f64> {
    Some(moving.trusted()?.hypot(fixed.trusted()?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Point3, Rng, Vec3, synth};

    fn noisy_plane(sigma: f64) -> Mesh {
        let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 40.0, 40.0, 40, 40);
        synth::add_noise(&mut m, sigma, &mut Rng::new(3));
        m
    }

    #[test]
    fn an_estimate_that_is_not_smooth_gives_no_noise_for_the_pair() {
        let good = ScanNoise { sigma: 0.05, smooth: true };
        let coarse = ScanNoise { sigma: 0.12, smooth: false };
        let nan = ScanNoise { sigma: f64::NAN, smooth: true };
        assert_eq!(pair_noise(&good, &good), Some(0.05_f64.hypot(0.05)));
        assert_eq!(pair_noise(&good, &coarse), None);
        assert_eq!(pair_noise(&coarse, &good), None);
        assert_eq!(pair_noise(&good, &nan), None);
    }

    #[test]
    fn the_estimate_of_a_plane_with_noise_is_smooth_and_near_the_noise() {
        let e = ScanNoise::of(&noisy_plane(0.05));
        assert!(e.smooth && (0.03..0.08).contains(&e.sigma), "{e:?}");
    }
}
