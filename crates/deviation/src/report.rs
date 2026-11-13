//! Reports: histograms, bands and percentiles of the deviation, and the report built from samples.

use super::sample::*;
use super::*;

/// Largest number of histogram bins [`report`] accepts.
pub const MAX_BINS: usize = 1000;
/// Largest number of tolerance bands [`report`] accepts.
pub const MAX_BANDS: usize = 64;

/// Percentiles of the signed deviation (mm), linearly interpolated between sorted values.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Percentiles {
    /// 1st percentile.
    pub p1: f64,
    /// 5th percentile.
    pub p5: f64,
    /// Median.
    pub p50: f64,
    /// 95th percentile.
    pub p95: f64,
    /// 99th percentile.
    pub p99: f64,
}

/// Histogram of the signed deviation: equal bins over `lo..hi`.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Histogram {
    /// Lower edge of the first bin (mm).
    pub lo: f64,
    /// Upper edge of the last bin (mm).
    pub hi: f64,
    /// Counted points per bin. Values outside `lo..hi` fall in the end bins, so the counts sum to
    /// the number of counted points.
    pub counts: Vec<usize>,
}

/// The share of counted points whose |deviation| is within one tolerance.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Band {
    /// Tolerance (mm).
    pub tolerance: f64,
    /// Counted points with |deviation| ≤ tolerance.
    pub count: usize,
    /// Their share of the counted points (0..1).
    pub share: f64,
}

/// Statistics of the signed deviation of a set of samples.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Report {
    /// Counted points (measured and facing the reference).
    pub count: usize,
    /// Points left out because their normal points away from the reference normal.
    pub not_facing: usize,
    /// Points not measured: outside the region, or farther than the reach of the reference.
    pub unmeasured: usize,
    /// Smallest signed deviation (mm; 0 without counted points).
    pub min: f64,
    /// Largest signed deviation (mm; 0 without counted points).
    pub max: f64,
    /// Mean signed deviation (mm).
    pub mean: f64,
    /// RMS of the signed deviation (mm).
    pub rms: f64,
    /// Percentiles of the signed deviation.
    pub percentiles: Percentiles,
    /// Histogram of the signed deviation.
    pub histogram: Histogram,
    /// Share within each requested tolerance of |deviation|.
    pub bands: Vec<Band>,
}

/// Options of [`report`].
#[derive(Clone, Debug, PartialEq)]
pub struct ReportOptions {
    /// Histogram bins, 1 to [`MAX_BINS`].
    pub bins: usize,
    /// Histogram range (mm, finite, lower < upper). Default: ±p99 of |deviation|, or ±1 mm when
    /// every deviation is zero.
    pub range: Option<[f64; 2]>,
    /// Tolerances of the bands (mm, positive, at most [`MAX_BANDS`]).
    pub bands: Vec<f64>,
}

/// Linear interpolation of the `p`-th percentile (0..=100) of sorted values; 0 when empty.
fn percentile(sorted: &[f64], p: f64) -> f64 {
    let Some(last) = sorted.len().checked_sub(1) else { return 0.0 };
    let rank = (p / 100.0).clamp(0.0, 1.0) * last as f64;
    let lo = rank.floor();
    let hi = rank.ceil();
    let a = sorted.get(lo as usize).copied().unwrap_or(0.0);
    let b = sorted.get(hi as usize).copied().unwrap_or(0.0);
    a + (b - a) * (rank - lo)
}

/// Statistics of the signed deviation of the counted samples: extremes, mean, RMS, percentiles, a
/// histogram and the share within each tolerance band.
///
/// # Errors
/// Returns a message when the options are out of range (bins, range, bands).
pub fn report(samples: &[Sample], opts: &ReportOptions) -> Result<Report, String> {
    if opts.bins == 0 || opts.bins > MAX_BINS {
        return Err(format!("bins must be between 1 and {MAX_BINS}"));
    }
    if opts.bands.len() > MAX_BANDS {
        return Err(format!("at most {MAX_BANDS} bands"));
    }
    if let Some(t) = opts.bands.iter().find(|t| !(t.is_finite() && **t > 0.0)) {
        return Err(format!("band tolerance {t} must be positive"));
    }
    if let Some([lo, hi]) = opts.range
        && !(lo.is_finite() && hi.is_finite() && lo < hi)
    {
        return Err(format!("range {lo} to {hi} must be finite with lower < upper"));
    }
    let mut vals: Vec<f64> = samples.iter().filter(|s| s.counted()).map(|s| s.d).collect();
    vals.sort_by(f64::total_cmp);
    let count = vals.len();
    let mut abs: Vec<f64> = vals.iter().map(|d| d.abs()).collect();
    abs.sort_by(f64::total_cmp);
    let (min, max, mean, rms) = if count == 0 {
        (0.0, 0.0, 0.0, 0.0)
    } else {
        let n = count as f64;
        let sum: f64 = vals.iter().sum();
        let sq: f64 = vals.iter().map(|d| d * d).sum();
        let min = vals.first().copied().unwrap_or(0.0);
        let max = vals.last().copied().unwrap_or(0.0);
        (min, max, sum / n, (sq / n).sqrt())
    };
    let (lo, hi) = match opts.range {
        Some([lo, hi]) => (lo, hi),
        None => {
            let p99 = percentile(&abs, 99.0);
            let half = if p99 > 0.0 { p99 } else { abs.last().copied().unwrap_or(0.0) };
            let half = if half > 0.0 { half } else { 1.0 };
            (-half, half)
        }
    };
    let width = (hi - lo) / opts.bins as f64;
    let last = (opts.bins - 1) as f64;
    let mut counts = vec![0usize; opts.bins];
    for d in &vals {
        let k = ((d - lo) / width).floor();
        let idx = if k.is_nan() { 0 } else { k.clamp(0.0, last) as usize };
        if let Some(c) = counts.get_mut(idx) {
            *c += 1;
        }
    }
    let bands = opts
        .bands
        .iter()
        .map(|&tolerance| {
            let n = abs.partition_point(|&a| a <= tolerance);
            Band { tolerance, count: n, share: if count > 0 { n as f64 / count as f64 } else { 0.0 } }
        })
        .collect();
    Ok(Report {
        count,
        not_facing: samples.iter().filter(|s| s.not_facing()).count(),
        unmeasured: samples.iter().filter(|s| !s.d.is_finite()).count(),
        min,
        max,
        mean,
        rms,
        percentiles: Percentiles {
            p1: percentile(&vals, 1.0),
            p5: percentile(&vals, 5.0),
            p50: percentile(&vals, 50.0),
            p95: percentile(&vals, 95.0),
            p99: percentile(&vals, 99.0),
        },
        histogram: Histogram { lo, hi, counts },
        bands,
    })
}
