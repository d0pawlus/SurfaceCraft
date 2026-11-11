//! Where a region's data begins and ends along its wall or round. Hole filling leaves big
//! triangles whose vertices lie along a wall's line far from the wall, so the extent is that of the
//! dense part of the data.
use super::profile::{Corner, Wall, d2, tangent};
use std::f64::consts::TAU;

/// A wall's points count when they lie this near its line (mm); a round's when they lie within the
/// larger of `ON_CIRCLE_MIN` and `ON_CIRCLE_FRACTION` of its radius of the circle.
const ON_LINE: f64 = 0.6;
const ON_CIRCLE_MIN: f64 = 0.6;
const ON_CIRCLE_FRACTION: f64 = 0.25;
/// Bin width of the density test of a wall's points (mm), the share of the typical bin count a bin
/// needs to be dense, the most non-dense bins a run may bridge, the fewest points for which the test
/// is made and the most bins.
const DENSITY_BIN: f64 = 0.5;
const DENSE_FRACTION: f64 = 0.3;
const MAX_GAP_BINS: usize = 3;
const MIN_POINTS_FOR_DENSITY: usize = 20;
const MAX_BINS: usize = 4096;
/// Shortest span in which a round's angles are binned (mm).
const MIN_RADIUS: f64 = 0.05;

fn angle_of(p: [f64; 2], c: [f64; 2]) -> f64 {
    (p[1] - c[1]).atan2(p[0] - c[0])
}

/// The extent of the dense part of sorted values. The values are binned; a bin with less than
/// `DENSE_FRACTION` of the typical count of the occupied bins holds stray points (the big triangles
/// that hole filling leaves put vertices along a wall's line, far from the wall), and the longest
/// longest run of dense bins gives the extent.
pub(super) fn dense_extent(sorted: &[f64], bin: f64) -> Option<(f64, f64)> {
    let (first, last) = (*sorted.first()?, *sorted.last()?);
    if sorted.len() < MIN_POINTS_FOR_DENSITY {
        return Some((first, last));
    }
    let bin = bin.max((last - first) / MAX_BINS as f64).max(1e-9);
    let index = |v: f64| ((v - first) / bin) as usize;
    let mut count = vec![0usize; index(last) + 1];
    for &v in sorted {
        if let Some(c) = count.get_mut(index(v)) {
            *c += 1;
        }
    }
    let mut occupied: Vec<usize> = count.iter().copied().filter(|&c| c > 0).collect();
    occupied.sort_unstable();
    // the typical count of a dense bin: the upper quartile of the occupied bins (stray points make
    // many bins with a few points each, which would drag a median down)
    let typical = occupied.get(occupied.len() * 3 / 4).copied()?;
    let need = ((DENSE_FRACTION * typical as f64).ceil() as usize).max(1);
    // runs of dense bins, joined across at most MAX_GAP_BINS bins that are not dense
    let mut runs: Vec<(usize, usize)> = Vec::new();
    for i in (0..count.len()).filter(|&i| count.get(i).is_some_and(|&c| c >= need)) {
        match runs.last_mut() {
            Some(r) if i - r.1 - 1 <= MAX_GAP_BINS => r.1 = i,
            _ => runs.push((i, i)),
        }
    }
    let (s, e) = runs.into_iter().max_by_key(|r| r.1 - r.0)?;
    let lo = sorted.iter().copied().find(|&v| index(v) >= s)?;
    let hi = sorted.iter().rev().copied().find(|&v| index(v) <= e)?;
    Some((lo, hi))
}

pub(super) fn wall_ends(w: &Wall) -> ([f64; 2], [f64; 2]) {
    let t = tangent(w.n);
    let on_line = |p: &&[f64; 2]| (w.n[0] * p[0] + w.n[1] * p[1] - w.c).abs() <= ON_LINE;
    let mut s: Vec<f64> = w.pts.iter().filter(on_line).map(|p| p[0] * t[0] + p[1] * t[1]).collect();
    s.sort_by(f64::total_cmp);
    let (lo, hi) = dense_extent(&s, DENSITY_BIN).unwrap_or((0.0, 0.0));
    if hi - lo > 11.0 {
        let mut h = std::collections::BTreeMap::new();
        for v in &s {
            *h.entry(v.floor() as i64).or_insert(0usize) += 1;
        }
        eprintln!("DBG hist n {:?} c {:.2}: {:?}; run {lo:.2}..{hi:.2}", w.n, w.c, h);
        let off: Vec<_> =
            w.pts.iter().filter(|p| (w.n[0] * p[0] + w.n[1] * p[1] - w.c).abs() > ON_LINE).count().to_string().into();
        eprintln!("DBG off-line points: {off:?}");
    }
    let at = |s: f64| [w.n[0] * w.c + t[0] * s, w.n[1] * w.c + t[1] * s];
    (at(lo), at(hi))
}

pub(super) fn on_circle(k: &Corner, a: f64) -> [f64; 2] {
    [k.center[0] + k.r * a.cos(), k.center[1] + k.r * a.sin()]
}

/// The ends of an arc in travel order (counter-clockwise for a round of the void, clockwise for a
/// round of the material): the arc covers the circle but for the largest gap in its points.
pub(super) fn arc_ends(k: &Corner) -> ([f64; 2], [f64; 2]) {
    let band = ON_CIRCLE_MIN.max(ON_CIRCLE_FRACTION * k.r);
    let mut ang: Vec<f64> =
        k.pts.iter().filter(|p| (d2(**p, k.center) - k.r).abs() <= band).map(|p| angle_of(*p, k.center)).collect();
    ang.sort_by(f64::total_cmp);
    let n = ang.len();
    let mut best = (f64::NEG_INFINITY, 0usize);
    for i in 0..n {
        let (a, b) = (ang.get(i).copied().unwrap_or(0.0), ang.get((i + 1) % n).copied().unwrap_or(0.0));
        let gap = if i + 1 < n { b - a } else { b + TAU - a };
        if gap > best.0 {
            best = (gap, i);
        }
    }
    // the angles in counter-clockwise order from the end of the gap, unwrapped
    let run: Vec<f64> = (0..n)
        .filter_map(|j| {
            let idx = (best.1 + 1 + j) % n;
            ang.get(idx).map(|a| if idx <= best.1 { a + TAU } else { *a })
        })
        .collect();
    let (after, before) = dense_extent(&run, DENSITY_BIN / k.r.max(MIN_RADIUS)).unwrap_or((0.0, 0.0));
    if k.concave { (on_circle(k, after), on_circle(k, before)) } else { (on_circle(k, before), on_circle(k, after)) }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stray_points_along_a_wall_do_not_move_its_ends() {
        let mut s: Vec<f64> = (0..=200).map(|k| 0.05 * k as f64).collect();
        s.extend([14.0, 14.1, 17.0]);
        let (lo, hi) = dense_extent(&s, DENSITY_BIN).unwrap();
        assert!(lo.abs() < 1e-9 && (9.4..=10.0).contains(&hi), "{lo} {hi}");
        // a short list is taken as it is
        assert_eq!(dense_extent(&[1.0, 2.0, 30.0], DENSITY_BIN), Some((1.0, 30.0)));
        assert_eq!(dense_extent(&[], DENSITY_BIN), None);
    }
}
