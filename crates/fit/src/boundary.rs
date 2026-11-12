//! The outline of an open region as a smooth, corner-aware, *enclosing* polyline (P8-H9), the curve
//! an unstructured layout is conformed to (see [`crate::retopo::conform_to_outline`]).
//!
//! The boundary of a region grown over a scan is a zigzag of a few triangle edges, and the
//! lattice of a field-aligned layout cannot follow it. Each boundary loop of the mesh is
//! 1. resampled by arc length (step = the scan's mean edge length) and smoothed (binomial passes,
//!    Gaussian width [`SMOOTH_SPACINGS`] spacings);
//! 2. searched for *corners*: places where the chords to the points [`CORNER_SPACINGS`] spacings
//!    ahead and behind turn by more than [`CORNER_ANGLE_DEG`]; corners are kept sharp, so the
//!    arcs between them are smoothed with the corners pinned;
//! 3. pushed outward in the surface (along `tangent × normal`, per arc) by the largest distance
//!    any raw boundary vertex of that arc lies beyond the smoothed curve, plus a margin, so the
//!    outline contains every vertex of the region (within the cap [`MAX_OFFSET_SPACINGS`]).
//!
//! The loops run with the region on their left (the winding of the triangles). Everything depends
//! on the mesh and its mean edge length only, so computing the outline again later (after a layout
//! split) gives the same curve.
use std::collections::{BTreeMap, BTreeSet};
use surfacecraft_geom::{KdTree, Mesh, Point3, Vec3, normalize};

/// Width (σ) of the smoothing, in scan spacings.
const SMOOTH_SPACINGS: f64 = 3.0;
/// Distance to the chord points used to find corners, in scan spacings.
const CORNER_SPACINGS: f64 = 10.0;
/// A corner turns the chords by at least this angle.
const CORNER_ANGLE_DEG: f64 = 55.0;
/// Margin added to the enclosing offset, in scan spacings.
const MARGIN_SPACINGS: f64 = 1.0;
/// The enclosing offset is at most this many scan spacings.
const MAX_OFFSET_SPACINGS: f64 = 10.0;
/// Half-width of the stretch a lone tooth pushes outward, in scan spacings.
const DILATE_SPACINGS: f64 = 4.0;
/// Loops shorter than this many spacings are noise (tiny holes) and are ignored.
const MIN_LOOP_SPACINGS: f64 = 16.0;
/// Smallest number of points of a resampled loop.
const MIN_POINTS: usize = 12;

/// One closed boundary loop.
#[derive(Clone, Debug, Default)]
pub struct OutlineLoop {
    /// Points of the closed polyline, region on the left of the travel direction.
    pub points: Vec<Point3>,
    /// Indices of the corner points.
    pub corners: Vec<usize>,
    /// Unit outward direction (in the surface, away from the region) at every point.
    pub outward: Vec<Vec3>,
}

/// The outline of a region: its boundary loops (outer loop and holes).
#[derive(Clone, Debug, Default)]
pub struct Outline {
    /// Loops, longest first.
    pub loops: Vec<OutlineLoop>,
    /// The slack added to the enclosing offset everywhere (mm): a layout that already follows the
    /// raw boundary lies this far inside the outline.
    pub slack: f64,
}

/// The closest point of an outline.
#[derive(Clone, Copy, Debug)]
pub struct OutlineHit {
    /// Loop index.
    pub loop_index: usize,
    /// Arc length of the hit from the start of the loop.
    pub arc: f64,
    /// The point.
    pub point: Point3,
    /// Distance from the query.
    pub distance: f64,
    /// Outward direction at the hit.
    pub outward: Vec3,
}

impl OutlineLoop {
    /// Are the parallel vectors consistent (a point and an outward direction each, corners in range)?
    /// Loops of [`region_outline`] always are; a hand-built one may not be.
    #[must_use]
    pub fn is_valid(&self) -> bool {
        self.points.len() >= 2
            && self.points.len() == self.outward.len()
            && self.corners.iter().all(|&c| c < self.points.len())
    }

    fn segment_count(&self) -> usize {
        self.points.len()
    }

    /// Cumulative arc length at every point and the total length.
    #[must_use]
    pub fn arc_lengths(&self) -> (Vec<f64>, f64) {
        let mut s = Vec::with_capacity(self.points.len());
        let mut acc = 0.0;
        for k in 0..self.points.len() {
            s.push(acc);
            let (a, b) = (self.points[k], self.points[(k + 1) % self.points.len()]);
            acc += (b - a).norm();
        }
        (s, acc)
    }

    /// The point at arc length `s` (wrapped) and its outward direction.
    #[must_use]
    pub fn at(&self, s: f64) -> Option<(Point3, Vec3)> {
        if !self.is_valid() {
            return None;
        }
        let (arcs, total) = self.arc_lengths();
        let n = self.points.len();
        if n < 2 || total <= 0.0 {
            return None;
        }
        let s = s.rem_euclid(total);
        let k = arcs.partition_point(|&a| a <= s).saturating_sub(1).min(n - 1);
        let start = *arcs.get(k)?;
        let end = if k + 1 < n { *arcs.get(k + 1)? } else { total };
        let t = ((s - start) / (end - start).max(1e-12)).clamp(0.0, 1.0);
        let next = (k + 1) % n;
        let (a, b) = (*self.points.get(k)?, *self.points.get(next)?);
        let (oa, ob) = (*self.outward.get(k)?, *self.outward.get(next)?);
        Some((a + (b - a) * t, normalize(oa * (1.0 - t) + ob * t).unwrap_or(oa)))
    }

    /// The closest point of the loop to `q`.
    #[must_use]
    pub fn closest(&self, q: &Point3) -> Option<(f64, Point3, f64, Vec3)> {
        if !self.is_valid() {
            return None;
        }
        let (arcs, total) = self.arc_lengths();
        let n = self.segment_count();
        let mut best: Option<(f64, f64, Point3, Vec3)> = None;
        for k in 0..n {
            let (a, b) = (self.points[k], self.points[(k + 1) % n]);
            let e = b - a;
            let l2 = e.norm_squared();
            let t = if l2 > 1e-24 { ((q - a).dot(&e) / l2).clamp(0.0, 1.0) } else { 0.0 };
            let p = a + e * t;
            let d = (q - p).norm();
            if best.is_none_or(|(bd, ..)| d < bd) {
                let (oa, ob) = (self.outward[k], self.outward[(k + 1) % n]);
                let s = arcs[k] + e.norm() * t;
                best = Some((d, s.rem_euclid(total.max(1e-12)), p, normalize(oa * (1.0 - t) + ob * t).unwrap_or(oa)));
            }
        }
        best.map(|(d, s, p, o)| (s, p, d, o))
    }
}

impl Outline {
    /// The closest point over all loops.
    #[must_use]
    pub fn closest(&self, q: &Point3) -> Option<OutlineHit> {
        let mut best: Option<OutlineHit> = None;
        for (i, l) in self.loops.iter().enumerate() {
            if let Some((arc, point, distance, outward)) = l.closest(q)
                && best.is_none_or(|b| distance < b.distance)
            {
                best = Some(OutlineHit { loop_index: i, arc, point, distance, outward });
            }
        }
        best
    }
}

/// Boundary loops of the mesh as vertex-id chains, region on the left.
fn boundary_loops(mesh: &Mesh) -> Vec<Vec<u32>> {
    let mut directed: BTreeSet<(u32, u32)> = BTreeSet::new();
    for t in &mesh.triangles {
        for k in 0..3 {
            directed.insert((t[k], t[(k + 1) % 3]));
        }
    }
    let mut next: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            next.entry(a).or_default().push(b);
        }
    }
    let mut loops = Vec::new();
    let starts: Vec<u32> = next.keys().copied().collect();
    for s in starts {
        while let Some(first) = next.get_mut(&s).and_then(|v| if v.is_empty() { None } else { Some(v.remove(0)) }) {
            let mut lp = vec![s];
            let mut cur = first;
            while cur != s && lp.len() <= mesh.positions.len() {
                lp.push(cur);
                match next.get_mut(&cur).and_then(|v| if v.is_empty() { None } else { Some(v.remove(0)) }) {
                    Some(n) => cur = n,
                    None => break,
                }
            }
            if cur == s && lp.len() >= 3 {
                loops.push(lp);
            }
        }
    }
    loops
}

/// `count` points at equal arc length around the closed polyline `raw`.
fn resample(raw: &[Point3], count: usize) -> Vec<Point3> {
    let n = raw.len();
    let mut arcs = vec![0.0];
    for k in 0..n {
        let l = (raw[(k + 1) % n] - raw[k]).norm();
        arcs.push(arcs[k] + l);
    }
    let total = arcs[n].max(1e-12);
    let mut out = Vec::with_capacity(count);
    let mut seg = 0;
    for i in 0..count {
        let s = total * i as f64 / count as f64;
        while seg + 1 < n && arcs[seg + 1] < s {
            seg += 1;
        }
        let (a, b) = (raw[seg], raw[(seg + 1) % n]);
        let l = (arcs[seg + 1] - arcs[seg]).max(1e-12);
        out.push(a + (b - a) * ((s - arcs[seg]) / l).clamp(0.0, 1.0));
    }
    out
}

/// Binomial smoothing; points in `pinned` do not move.
fn smooth(points: &[Point3], passes: usize, pinned: &BTreeSet<usize>) -> Vec<Point3> {
    let n = points.len();
    let mut cur = points.to_vec();
    for _ in 0..passes {
        let prev = cur.clone();
        for k in 0..n {
            if !pinned.contains(&k) {
                cur[k] = Point3::from(
                    (prev[(k + n - 1) % n].coords + prev[k].coords * 2.0 + prev[(k + 1) % n].coords) * 0.25,
                );
            }
        }
    }
    cur
}

/// Corner indices: local maxima of the chord turning angle above the threshold.
fn find_corners(points: &[Point3], window: usize) -> Vec<usize> {
    let n = points.len();
    let w = window.max(2).min((n / 4).max(2));
    let turn: Vec<f64> = (0..n)
        .map(|k| {
            let (a, b, c) = (points[(k + n - w) % n], points[k], points[(k + w) % n]);
            match (normalize(b - a), normalize(c - b)) {
                (Some(u), Some(v)) => u.dot(&v).clamp(-1.0, 1.0).acos().to_degrees(),
                _ => 0.0,
            }
        })
        .collect();
    (0..n)
        .filter(|&k| {
            turn[k] >= CORNER_ANGLE_DEG
                && (1..=w).all(|d| turn[k] > turn[(k + d) % n] || (turn[k] == turn[(k + d) % n] && d % 2 == 0))
                && (1..=w).all(|d| turn[k] >= turn[(k + n - d) % n])
        })
        .collect()
}

fn tangent(points: &[Point3], k: usize) -> Vec3 {
    let n = points.len();
    normalize(points[(k + 1) % n] - points[(k + n - 1) % n]).unwrap_or_else(Vec3::x)
}

/// Smoothed polyline of a resampled loop with its corners: corners keep a lightly smoothed
/// position and the arcs between them are smoothed around them.
fn smooth_with_corners(sampled: &[Point3], passes: usize, spacing: f64) -> (Vec<Point3>, Vec<usize>) {
    let smoothed = smooth(sampled, passes, &BTreeSet::new());
    let smoothed_step =
        (0..smoothed.len()).map(|k| (smoothed[(k + 1) % smoothed.len()] - smoothed[k]).norm()).sum::<f64>()
            / smoothed.len() as f64;
    let window = (CORNER_SPACINGS * spacing / smoothed_step.max(1e-9)).round() as usize;
    let corners = find_corners(&smoothed, window);
    let light = smooth(sampled, 3, &BTreeSet::new());
    let pinned: BTreeSet<usize> = corners.iter().copied().collect();
    let mut base = smooth(sampled, passes, &pinned);
    for &c in &corners {
        base[c] = light[c];
    }
    (smooth(&base, passes, &pinned), corners)
}

/// The curve pushed outward by `delta` per point (corners along the bisector of their sides).
fn offset_loop(base: &[Point3], corners: Vec<usize>, outward: &[Vec3], delta: &[f64]) -> OutlineLoop {
    let n = base.len();
    let (mut points, mut dirs) = (Vec::with_capacity(n), Vec::with_capacity(n));
    for k in 0..n {
        if corners.contains(&k) {
            let (before, after) = (outward[(k + n - 1) % n], outward[(k + 1) % n]);
            let d = normalize(before + after).unwrap_or(before);
            let reach = delta[(k + n - 1) % n].max(delta[k]).max(delta[(k + 1) % n]);
            points.push(base[k] + d * (reach / d.dot(&before).max(0.35)));
            dirs.push(d);
        } else {
            points.push(base[k] + outward[k] * delta[k]);
            dirs.push(outward[k]);
        }
    }
    OutlineLoop { points, corners, outward: dirs }
}

/// The outline of one boundary loop, or `None` for loops too short to matter.
fn outline_loop(mesh: &Mesh, normals: &[Vec3], tree: &KdTree, ids: &[u32], spacing: f64) -> Option<OutlineLoop> {
    let raw: Vec<Point3> = ids.iter().filter_map(|&v| mesh.positions.get(v as usize).copied()).collect();
    let length: f64 = (0..raw.len()).map(|k| (raw[(k + 1) % raw.len()] - raw[k]).norm()).sum();
    if raw.len() < 3 || length < MIN_LOOP_SPACINGS * spacing {
        return None;
    }
    let count = ((length / spacing).ceil() as usize).max(MIN_POINTS);
    let step = length / count as f64;
    let passes = (2.0 * (SMOOTH_SPACINGS * spacing / step).powi(2)).round().max(1.0) as usize;
    let (base, corners) = smooth_with_corners(&resample(&raw, count), passes, spacing);
    let normal_at = |p: &Point3| tree.nearest(p).and_then(|(i, _)| normals.get(i)).copied().unwrap_or_else(Vec3::z);
    let outward: Vec<Vec3> = (0..base.len())
        .map(|k| normalize(tangent(&base, k).cross(&normal_at(&base[k]))).unwrap_or_else(Vec3::y))
        .collect();
    let probe = OutlineLoop { points: base.clone(), corners: corners.clone(), outward: outward.clone() };
    Some(offset_loop(&base, corners, &outward, &enclosing_offsets(&probe, &raw, spacing)))
}

/// Per point of `base`, how far outward it must move so that the curve contains every raw
/// boundary vertex near it: the largest exceedance of the raw vertices projecting onto a
/// neighbourhood of the point (a dilation, so a lone tooth moves a stretch of the curve),
/// smoothed, plus the margin.
fn enclosing_offsets(base: &OutlineLoop, raw: &[Point3], spacing: f64) -> Vec<f64> {
    let n = base.points.len();
    let (arcs, total) = base.arc_lengths();
    let mut exceed = vec![0.0f64; n];
    for r in raw {
        if let Some((s, p, _, out)) = base.closest(r) {
            let k = arcs.partition_point(|&a| a <= s).saturating_sub(1).min(n - 1);
            exceed[k] = exceed[k].max((r - p).dot(&out));
        }
    }
    let per_index = (total / n as f64).max(1e-9);
    let radius = ((DILATE_SPACINGS * spacing / per_index).ceil() as usize).max(1);
    let dilated: Vec<f64> = (0..n)
        .map(|k| (0..=2 * radius).map(|d| exceed[(k + 2 * n + d - radius.min(n)) % n]).fold(0.0, f64::max))
        .collect();
    let passes = (2.0 * (radius as f64 / 3.0).powi(2)).round().max(1.0) as usize;
    let mut cur = dilated;
    for _ in 0..passes {
        let prev = cur.clone();
        for k in 0..n {
            cur[k] = 0.25 * prev[(k + n - 1) % n] + 0.5 * prev[k] + 0.25 * prev[(k + 1) % n];
        }
    }
    let cap = MAX_OFFSET_SPACINGS * spacing;
    cur.iter().map(|d| (d + MARGIN_SPACINGS * spacing).min(cap)).collect()
}

/// The outline of `mesh` (its boundary loops); `spacing` is the mean edge length of the scan.
/// A closed mesh has no outline.
#[must_use]
pub fn region_outline(mesh: &Mesh, spacing: f64) -> Outline {
    if !(spacing.is_finite() && spacing > 0.0) {
        return Outline::default();
    }
    let normals = mesh.vertex_normals();
    let tree = KdTree::build(&mesh.positions);
    let mut loops: Vec<OutlineLoop> =
        boundary_loops(mesh).iter().filter_map(|ids| outline_loop(mesh, &normals, &tree, ids, spacing)).collect();
    loops.sort_by(|a, b| b.arc_lengths().1.total_cmp(&a.arc_lengths().1));
    Outline { loops, slack: MARGIN_SPACINGS * spacing }
}

/// Mean edge length of the mesh (the scan's sampling distance).
fn mean_edge(m: &Mesh) -> f64 {
    let (mut sum, mut n) = (0.0, 0usize);
    for t in &m.triangles {
        for k in 0..3 {
            if let (Some(a), Some(b)) = (m.positions.get(t[k] as usize), m.positions.get(t[(k + 1) % 3] as usize)) {
                sum += (b - a).norm();
                n += 1;
            }
        }
    }
    sum / n.max(1) as f64
}

/// The outline of `mesh` after welding and cleaning it (the same for every caller, so a layout
/// can be re-snapped later), with the cleaned mesh. A closed mesh has no outline.
#[must_use]
pub fn cleaned_outline(mesh: &Mesh) -> (Outline, Mesh) {
    let mut clean = mesh.clone();
    clean.weld(1e-9);
    clean.remove_degenerate(0.0);
    surfacecraft_geom::edit::compact(&mut clean);
    if clean.triangle_count() < 4 {
        return (Outline::default(), clean);
    }
    let outline = region_outline(&clean, mean_edge(&clean));
    (outline, clean)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    #[test]
    fn a_square_plate_gets_four_corners_and_an_outline_just_outside() {
        let m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 30.0, 30.0, 60, 60);
        let spacing = 0.5;
        let outline = region_outline(&m, spacing);
        assert_eq!(outline.loops.len(), 1);
        let lp = &outline.loops[0];
        assert_eq!(lp.corners.len(), 4, "corners {:?}", lp.corners);
        // the plate spans [-15, 15]²: the outline lies outside it by the margin, the region on its left
        for p in &lp.points {
            let outside = (-p.x).max(p.x - 30.0).max(-p.y).max(p.y - 30.0);
            assert!(outside > 0.0 && outside < 2.0 * spacing, "{p:?}");
        }
        let (_, total) = lp.arc_lengths();
        assert!(total > 120.0 && total < 130.0, "{total}");
    }
}

#[cfg(test)]
mod malformed {
    use super::*;

    /// A loop built by hand with parallel vectors of different lengths answers `None`, never panics.
    #[test]
    fn a_hand_built_loop_with_mismatched_vectors_is_refused() {
        let lp = OutlineLoop { points: vec![Point3::origin()], outward: Vec::new(), corners: Vec::new() };
        assert!(lp.closest(&Point3::new(1.0, 0.0, 0.0)).is_none());
        assert!(lp.at(0.5).is_none());
        let two = OutlineLoop {
            points: vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            outward: vec![Vec3::y()],
            corners: vec![7],
        };
        assert!(two.closest(&Point3::new(0.5, 1.0, 0.0)).is_none());
        let outline = Outline { loops: vec![lp, two], slack: 0.0 };
        assert!(outline.closest(&Point3::origin()).is_none());
    }
}
