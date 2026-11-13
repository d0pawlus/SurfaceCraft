//! Distributions of the deviation over bands and the measures of scan points against a reference.

use super::report::*;
use super::sample::*;
use super::*;

/// Bins of the colour map's histogram: equal bins over −3·tol … +3·tol (the legend's range).
pub const HISTOGRAM_BINS: usize = 10;
/// Half-width of the histogram, in tolerances.
const HISTOGRAM_RANGE_TOLERANCES: f64 = 3.0;

/// The histogram bin of signed deviation `d` at tolerance `tol`. Values outside the range fall in
/// the end bins.
#[must_use]
pub fn hist_bin(d: f64, tol: f64) -> usize {
    let half = HISTOGRAM_BINS as f64 / 2.0;
    let scaled = (d / (HISTOGRAM_RANGE_TOLERANCES * tol) + 1.0) * half;
    scaled.floor().clamp(0.0, HISTOGRAM_BINS as f64 - 1.0) as usize
}

/// The histogram and the cumulative tolerance bands of the counted points.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Distribution {
    /// Counted points.
    pub count: usize,
    /// Histogram over −3·tol … +3·tol: the bins of [`DeviationStats::histogram`].
    pub histogram: Histogram,
    /// Share of the counted points with |deviation| within each tolerance.
    pub bands: Vec<Band>,
}

/// The distribution of the counted samples at tolerance `tol`, with the share within each band
/// tolerance. One pass over the samples and no sorting, so it is cheap next to the measurement.
#[must_use]
pub fn distribution(samples: &[Sample], tol: f64, band_tolerances: &[f64]) -> Distribution {
    let mut counts = vec![0usize; HISTOGRAM_BINS];
    let mut within = vec![0usize; band_tolerances.len()];
    let mut count = 0usize;
    for s in samples.iter().filter(|s| s.counted()) {
        count += 1;
        if let Some(c) = counts.get_mut(hist_bin(s.d, tol)) {
            *c += 1;
        }
        let magnitude = s.d.abs();
        for (w, &t) in within.iter_mut().zip(band_tolerances) {
            if magnitude <= t {
                *w += 1;
            }
        }
    }
    let n = count.max(1) as f64;
    let range = HISTOGRAM_RANGE_TOLERANCES * tol;
    let bands = band_tolerances
        .iter()
        .zip(within)
        .map(|(&tolerance, w)| Band { tolerance, count: w, share: w as f64 / n })
        .collect();
    Distribution { count, histogram: Histogram { lo: -range, hi: range, counts }, bands }
}

/// Signed deviation of each point from a primitive.
#[must_use]
pub fn against_primitive(points: &[Point3], prim: &Primitive) -> Vec<f64> {
    points.iter().map(|p| prim.distance(p)).collect()
}

/// Signed deviation of each point from a mesh surface (sign from the triangle normal; NaN when
/// farther than `max_distance`).
#[must_use]
pub fn against_mesh(points: &[Point3], target: &Bvh, max_distance: f64) -> Vec<f64> {
    surfacecraft_geom::par::map_slice(points, |p| match target.closest(p, max_distance) {
        Some(h) => {
            let sign = target.triangle_normal(h.triangle).map_or(1.0, |n| (p - h.point).dot(&n).signum());
            h.distance * if sign == 0.0 { 1.0 } else { sign }
        }
        None => f64::NAN,
    })
}
