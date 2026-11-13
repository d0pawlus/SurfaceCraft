//! Deviation statistics: the mean, the spread and the extremes of the signed deviations against a tolerance.

use super::distribution::*;
use super::*;

/// Deviation statistics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct DeviationStats {
    /// Points measured (facing the reference, see [`Sample`]).
    pub count: usize,
    /// Points left out because their normal points away from the reference normal.
    #[serde(default)]
    pub not_facing: usize,
    /// Smallest signed deviation (mm; 0 without points).
    pub min: f64,
    /// Largest signed deviation (mm; 0 without points).
    pub max: f64,
    /// Largest |deviation| (mm).
    pub max_abs: f64,
    /// Mean signed deviation (mm).
    pub mean: f64,
    /// RMS deviation (mm).
    pub rms: f64,
    /// Tolerance used for the colour map and the in-tolerance share (mm).
    pub tolerance: f64,
    /// Share of points within ±tolerance (0..1).
    pub within: f64,
    /// Ten histogram bins of signed deviation over [−3·tol, 3·tol] (outside counts in the end bins).
    pub histogram: [usize; 10],
}

/// Statistics of signed deviations.
#[must_use]
pub fn stats(values: &[f64], tolerance: f64) -> DeviationStats {
    let tol = if tolerance.is_finite() && tolerance > 0.0 { tolerance } else { 0.1 };
    let vals: Vec<f64> = values.iter().copied().filter(|v| v.is_finite()).collect();
    let n = vals.len();
    let mut h = [0usize; 10];
    let (mut sum, mut sq, mut max_abs, mut within) = (0.0, 0.0, 0.0f64, 0usize);
    let (mut min, mut max) = (f64::INFINITY, f64::NEG_INFINITY);
    for &d in &vals {
        sum += d;
        sq += d * d;
        min = min.min(d);
        max = max.max(d);
        max_abs = max_abs.max(d.abs());
        if d.abs() <= tol {
            within += 1;
        }
        if let Some(b) = h.get_mut(hist_bin(d, tol)) {
            *b += 1;
        }
    }
    let nf = n.max(1) as f64;
    let (min, max) = if n == 0 { (0.0, 0.0) } else { (min, max) };
    DeviationStats {
        count: n,
        not_facing: 0,
        min,
        max,
        max_abs,
        mean: sum / nf,
        rms: (sq / nf).sqrt(),
        tolerance: tol,
        within: within as f64 / nf,
        histogram: h,
    }
}
