//! Patches of the feature network for regions of any number of corners (P8-H9). Mesh-side
//! topology and orchestration only: the Bézier operations (curve split, curve and interior fits,
//! evaluation) are those of [`crate::features`].
//!
//! A region with four corners is one patch, as before. A triangular, pentagonal or hexagonal
//! region cannot be covered by four-sided patches without splitting its boundary curves (an odd
//! number of sides cannot be quadrangulated by interior edges alone), and a boundary curve is
//! shared with the region on its other side, so the split has to happen on both sides. The rule
//! is therefore global and deterministic: **when any region is not four-sided, every region is
//! split Catmull–Clark style**. Each feature curve is cut at its midpoint (de Casteljau, once per
//! curve, so both sides use the same two halves), each region gets a centre point and one *spoke*
//! curve from the centre to every side midpoint (shared by the two patches next to it), and a
//! region with n corners becomes n four-sided patches `corner, side midpoint, centre, previous
//! side midpoint`. The network stays watertight by construction. The region's scan points are
//! partitioned by the angle around the centre between neighbouring spokes before the interior
//! of each patch is fitted.
//!
//! With [`NgonMode::Patch`] nothing is split: the four-sided regions stay Bézier patches and every
//! other region with one outline is handed over as an [`NgonRegion`] (its sides are the same shared
//! feature curves, sampled densely, plus start parameters from a flattening of the region), for
//! the caller to cover with one trimmed patch.
use crate::features::{NetN, bezier_point, eval_net, fit_curve, fit_interior, split_bezier};
use crate::mls::ScanSurface;
use crate::param::flatten;
use std::collections::BTreeSet;
use std::f64::consts::TAU;
use surfacecraft_geom::{KdTree, Mesh, Point3, Vec3, any_perpendicular};

/// What `feature_network_with` does with regions that are not four-sided.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum NgonMode {
    /// Every region is split into four-sided patches when any region is not four-sided.
    #[default]
    Split,
    /// Four-sided regions are Bézier patches; the other regions are returned as [`NgonRegion`].
    Patch,
}

/// One side of an [`NgonRegion`]: a shared feature curve, evaluated densely.
#[derive(Clone, Debug)]
pub struct NgonSide {
    /// Points along the curve from the side's first corner to the next corner (both ends exact).
    pub points: Vec<[f64; 3]>,
    /// Parameters of `points` in the flattening of the region (`None` when there is none).
    pub uv: Option<Vec<[f64; 2]>>,
}

/// A region that is not four-sided, to be covered by one patch.
#[derive(Clone, Debug)]
pub struct NgonRegion {
    /// Corner ids (indices into the network's `corner_points`) in loop order.
    pub corners: Vec<u32>,
    /// Side `k` runs from corner `k` to corner `k + 1`.
    pub sides: Vec<NgonSide>,
    /// The vertices of the region.
    pub data: Vec<[f64; 3]>,
    /// Parameters of `data` in the flattening; `Some` exactly when every side has `uv`.
    pub data_uv: Option<Vec<[f64; 2]>>,
    /// Triangles of the region (ids into the input mesh).
    pub triangles: Vec<u32>,
}

/// Points per side of an [`NgonRegion`].
const SIDE_SAMPLES: usize = 320;
/// Relaxation steps of the flattening.
const ARAP_ITERATIONS: usize = 10;

/// A smooth region bounded by feature curves.
pub(crate) struct RegionRec {
    /// Corner ids, in the order the boundary loop visits them.
    pub corners: Vec<u32>,
    /// Feature curve of side `k` (from corner `k` to corner `k + 1`).
    pub sides: Vec<usize>,
    /// Is side `k` read in the stored direction of its curve?
    pub forward: Vec<bool>,
    /// Vertices of the region.
    pub points: Vec<Vec3>,
    /// Sum of the vertex normals along the boundary (outward side).
    pub mean_normal: Vec3,
    /// Triangles of the region (ids into the input mesh).
    pub triangles: Vec<u32>,
    /// Vertices (ids into the input mesh) of the boundary run that is corner `k`.
    pub corner_verts: Vec<Vec<u32>>,
    /// Boundary vertices (ids into the input mesh) strictly between corner `k` and corner `k + 1`,
    /// in loop order.
    pub side_verts: Vec<Vec<u32>>,
    /// Ids (into the input mesh) of `points`, one each.
    pub point_ids: Vec<u32>,
}

impl RegionRec {
    /// Do the parallel vectors agree (one curve and one direction per corner, curves in range)?
    /// `feature_network` builds them together; a hand-built record may not.
    fn is_consistent(&self, curve_count: usize) -> bool {
        let n = self.corners.len();
        n >= 3 && self.sides.len() == n && self.forward.len() == n && self.sides.iter().all(|&c| c < curve_count)
    }
}

/// The patches of a set of regions.
#[derive(Default)]
pub(crate) struct Patches {
    pub nets: Vec<NetN>,
    pub corners: Vec<[u32; 4]>,
    pub region_triangles: Vec<Vec<u32>>,
    /// Points of the corner ids beyond the clusters (side midpoints, region centres), in id order.
    pub extra_points: Vec<[f64; 3]>,
}

/// Points sampled along a spoke and nearest scan points averaged at each.
const SPOKE_SAMPLES: usize = 12;
const SUPPORT_SPACINGS: f64 = 3.5;

fn a3(v: Vec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn reversed(c: &[Vec3]) -> Vec<Vec3> {
    c.iter().rev().copied().collect()
}

/// Control points of side `k` read from corner `k` to corner `k + 1`.
fn side_from_start(rec: &RegionRec, k: usize, curves: &[Vec<Vec3>]) -> Vec<Vec3> {
    let c = curves.get(rec.sides[k]).cloned().unwrap_or_default();
    if rec.forward[k] { c } else { reversed(&c) }
}

/// Fills the boundary of a net from its four sides, fits the interior to `points` and orients
/// the patch along `mean_normal`; corners `[c0, c1, c2, c3]` go with sides c0→c1, c1→c2, c3→c2, c0→c3.
fn finish_net(sides: [&[Vec3]; 4], corners: [u32; 4], points: &[Vec3], mean_normal: &Vec3) -> Option<(NetN, [u32; 4])> {
    let n = sides[0].len().saturating_sub(1);
    if n < 2 || sides.iter().any(|s| s.len() != n + 1) {
        return None;
    }
    let mut net: NetN = vec![vec![[0.0; 3]; n + 1]; n + 1];
    for k in 0..=n {
        net[k][0] = a3(sides[0][k]);
        net[n][k] = a3(sides[1][k]);
        net[k][n] = a3(sides[2][k]);
        net[0][k] = a3(sides[3][k]);
    }
    fit_interior(&mut net, points);
    let h = 1e-3;
    let du = eval_net(&net, 0.5 + h, 0.5) - eval_net(&net, 0.5 - h, 0.5);
    let dv = eval_net(&net, 0.5, 0.5 + h) - eval_net(&net, 0.5, 0.5 - h);
    if du.cross(&dv).dot(mean_normal) >= 0.0 {
        return Some((net, corners));
    }
    // swap u and v: transposes the net and the corner order c0, c3, c2, c1
    let mut t = net.clone();
    for i in 0..=n {
        for j in 0..=n {
            t[i][j] = net[j][i];
        }
    }
    Some((t, [corners[0], corners[3], corners[2], corners[1]]))
}

/// One patch per four-sided region.
pub(crate) fn quad_patches(recs: &[RegionRec], curves: &[Vec<Vec3>]) -> Patches {
    let mut out = Patches::default();
    for rec in recs.iter().filter(|r| r.corners.len() == 4 && r.is_consistent(curves.len())) {
        let s: Vec<Vec<Vec3>> = (0..4).map(|k| side_from_start(rec, k, curves)).collect();
        // c0→c1, c1→c2, c3→c2, c0→c3
        let (top, left) = (reversed(&s[2]), reversed(&s[3]));
        let c = [rec.corners[0], rec.corners[1], rec.corners[2], rec.corners[3]];
        let Some((net, corners)) = finish_net([&s[0], &s[1], &top, &left], c, &rec.points, &rec.mean_normal) else {
            continue;
        };
        out.nets.push(net);
        out.corners.push(corners);
        out.region_triangles.push(rec.triangles.clone());
    }
    out
}

/// `n + 1` points of the Bézier curve `c` at equal parameter steps, the end points exact.
fn sample_curve(c: &[Vec3], n: usize) -> Vec<Vec3> {
    let (Some(first), Some(last)) = (c.first(), c.last()) else { return Vec::new() };
    if c.len() < 2 || n == 0 {
        return Vec::new();
    }
    (0..=n)
        .map(|j| match j {
            0 => *first,
            _ if j == n => *last,
            _ => bezier_point(c, j as f64 / n as f64),
        })
        .collect()
}

/// Share of the total chord length reached at each point (`0` at the first, `1` at the last);
/// `None` when the polyline has no length.
fn fractions(pts: &[Vec3]) -> Option<Vec<f64>> {
    let mut acc = vec![0.0];
    for w in pts.windows(2) {
        acc.push(acc.last().copied()? + (w[1] - w[0]).norm());
    }
    let total = acc.last().copied()?;
    if !(total.is_finite() && total > 1e-12) {
        return None;
    }
    Some(acc.into_iter().map(|a| a / total).collect())
}

/// The value at fraction `g` of the polyline `values` whose points sit at the fractions `frac`.
fn interpolate(frac: &[f64], values: &[[f64; 2]], g: f64) -> Option<[f64; 2]> {
    let last = frac.len().checked_sub(1).filter(|&l| l >= 1)?;
    let i = frac.partition_point(|&f| f <= g).clamp(1, last);
    let (f0, f1) = (*frac.get(i - 1)?, *frac.get(i)?);
    let (a, b) = (values.get(i - 1)?, values.get(i)?);
    let t = if f1 - f0 > 1e-12 { ((g - f0) / (f1 - f0)).clamp(0.0, 1.0) } else { 0.0 };
    Some([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t])
}

/// Parameters of every side and of the vertices of a region.
type RegionUv = (Vec<Vec<[f64; 2]>>, Vec<[f64; 2]>);

/// Parameters of the dense sides and of the region's vertices from a flattening of the region, or
/// `None` when the flattening fails, folds, or the boundary does not match the loop of the
/// feature network. A corner is a cluster of vertices, not one vertex: its parameters are the mean
/// of the cluster's boundary vertices, and the samples of a side are assigned to the boundary
/// vertices between its corners in proportion to arc length.
fn region_uv(mesh: &Mesh, rec: &RegionRec, dense: &[Vec<Vec3>]) -> Option<RegionUv> {
    let n = rec.corners.len();
    if rec.corner_verts.len() != n || rec.side_verts.len() != n || dense.len() != n {
        return None;
    }
    let flat = flatten(mesh, &rec.triangles, ARAP_ITERATIONS).ok()?;
    if flat.flipped > 0 {
        return None;
    }
    let on_boundary: BTreeSet<u32> = flat.boundary.iter().copied().collect();
    let uv_of = |id: u32| flat.uv.get(id as usize).copied().flatten();
    let mut corner_uv: Vec<[f64; 2]> = Vec::with_capacity(n);
    for run in &rec.corner_verts {
        if run.is_empty() {
            return None;
        }
        let mut sum = [0.0; 2];
        for v in run {
            let q = if on_boundary.contains(v) { uv_of(*v)? } else { return None };
            sum[0] += q[0];
            sum[1] += q[1];
        }
        corner_uv.push([sum[0] / run.len() as f64, sum[1] / run.len() as f64]);
    }
    let mut sides = Vec::with_capacity(n);
    for (k, d) in dense.iter().enumerate() {
        let mut xyz = vec![*d.first()?];
        let mut uv = vec![*corner_uv.get(k)?];
        for &v in rec.side_verts.get(k)? {
            if !on_boundary.contains(&v) {
                return None;
            }
            xyz.push(mesh.positions.get(v as usize)?.coords);
            uv.push(uv_of(v)?);
        }
        xyz.push(*d.last()?);
        uv.push(*corner_uv.get((k + 1) % n)?);
        let frac_boundary = fractions(&xyz)?;
        let mut out: Vec<[f64; 2]> =
            fractions(d)?.iter().map(|&g| interpolate(&frac_boundary, &uv, g)).collect::<Option<_>>()?;
        // both corners exactly, so that consecutive sides meet
        *out.first_mut()? = *uv.first()?;
        *out.last_mut()? = *uv.last()?;
        sides.push(out);
    }
    let data: Vec<[f64; 2]> = rec.point_ids.iter().map(|&v| uv_of(v)).collect::<Option<_>>()?;
    if data.len() != rec.points.len() {
        return None;
    }
    Some((sides, data))
}

/// The regions that are not four-sided, with their sides sampled and, when the flattening of the
/// region is reliable, their parameters (for every side and the data, or for none).
pub(crate) fn ngon_regions(mesh: &Mesh, recs: &[RegionRec], curves: &[Vec<Vec3>]) -> Vec<NgonRegion> {
    recs.iter()
        .filter(|r| r.corners.len() != 4 && r.is_consistent(curves.len()))
        .map(|rec| {
            let dense: Vec<Vec<Vec3>> =
                (0..rec.corners.len()).map(|k| sample_curve(&side_from_start(rec, k, curves), SIDE_SAMPLES)).collect();
            let (side_uv, data_uv) = match region_uv(mesh, rec, &dense) {
                Some((s, d)) => (Some(s), Some(d)),
                None => (None, None),
            };
            let sides = dense
                .iter()
                .enumerate()
                .map(|(k, d)| NgonSide {
                    points: d.iter().map(|p| a3(*p)).collect(),
                    uv: side_uv.as_ref().and_then(|s| s.get(k)).cloned(),
                })
                .collect();
            NgonRegion {
                corners: rec.corners.clone(),
                sides,
                data: rec.points.iter().map(|p| a3(*p)).collect(),
                data_uv,
                triangles: rec.triangles.clone(),
            }
        })
        .collect()
}

/// The region's points as a surface: a moving-least-squares quadric over about `SUPPORT_SPACINGS`
/// point spacings (a mean of neighbours would sag by the curvature times the squared spacing).
struct Surface {
    scan: ScanSurface,
    radius: f64,
    hint: Vec3,
}

impl Surface {
    fn new(points: &[Point3], hint: Vec3) -> Self {
        let tree = KdTree::build(points);
        let step = points.len().div_ceil(300).max(1);
        let gaps: Vec<f64> =
            points.iter().step_by(step).filter_map(|p| tree.knn(p, 2).get(1).map(|x| x.1.sqrt())).collect();
        let spacing = gaps.iter().sum::<f64>() / gaps.len().max(1) as f64;
        Self { scan: ScanSurface::new(points), radius: SUPPORT_SPACINGS * spacing.max(1e-6), hint }
    }

    /// `p` moved onto the surface (unchanged when the support is too thin).
    fn snap(&self, p: &Vec3) -> Vec3 {
        self.scan.project(&Point3::from(*p), self.radius, &self.hint).map_or(*p, |x| x.0.coords)
    }
}

/// The curve from the centre to a side midpoint, following the surface of the region.
fn spoke(surface: &Surface, centre: &Vec3, mid: &Vec3, degree: usize) -> Vec<Vec3> {
    let path: Vec<Vec3> = (0..=SPOKE_SAMPLES)
        .map(|k| match k {
            0 => *centre,
            SPOKE_SAMPLES => *mid,
            _ => surface.snap(&(centre + (mid - centre) * (k as f64 / SPOKE_SAMPLES as f64))),
        })
        .collect();
    fit_curve(&path, degree)
}

/// Angle of `p` around `centre` in the plane with normal `axis`, measured from `e1`.
fn angle_around(p: &Vec3, centre: &Vec3, e1: &Vec3, e2: &Vec3) -> f64 {
    let d = p - centre;
    d.dot(e2).atan2(d.dot(e1)).rem_euclid(TAU)
}

/// The corner whose patch holds the point at angle `theta`, given the spoke angles.
fn sector_corner(theta: f64, spoke_angles: &[(f64, usize)]) -> usize {
    let n = spoke_angles.len();
    let pos = spoke_angles.iter().rposition(|&(a, _)| a <= theta).unwrap_or(n - 1);
    let (i, j) = (spoke_angles[pos].1, spoke_angles[(pos + 1) % n].1);
    if j == (i + 1) % n { j } else { i }
}

/// Cut curves shared by all regions, and the ids of the points the split adds.
struct Split<'a> {
    curves: &'a [Vec<Vec3>],
    /// First and second half of every curve (de Casteljau at its midpoint).
    halves: Vec<(Vec<Vec3>, Vec<Vec3>)>,
    first_new_id: u32,
    degree: usize,
}

impl Split<'_> {
    fn mid_id(&self, curve: usize) -> u32 {
        self.first_new_id + curve as u32
    }

    fn centre_id(&self, region: usize) -> u32 {
        self.first_new_id + self.curves.len() as u32 + region as u32
    }

    /// Control points of the first (`first`) or second half of side `side`, read from corner `side`
    /// (first half) or toward corner `side + 1` (second half).
    fn half(&self, rec: &RegionRec, side: usize, first: bool) -> Vec<Vec3> {
        let Some((lo, hi)) = rec.sides.get(side).and_then(|&c| self.halves.get(c)) else { return Vec::new() };
        match (first, rec.forward.get(side).copied().unwrap_or(true)) {
            (true, true) => lo.clone(),
            (true, false) => reversed(hi),
            (false, true) => hi.clone(),
            (false, false) => reversed(lo),
        }
    }

    /// The patches of region `r` (one per corner) and the centre point.
    fn region(&self, r: usize, rec: &RegionRec) -> (Vec<(NetN, [u32; 4])>, Vec3) {
        let n = rec.corners.len();
        let pts: Vec<Point3> = rec.points.iter().map(|p| Point3::from(*p)).collect();
        let axis = rec.mean_normal.try_normalize(1e-12).unwrap_or_else(Vec3::z);
        let surface = Surface::new(&pts, axis);
        let centroid = rec.points.iter().sum::<Vec3>() / rec.points.len().max(1) as f64;
        let centre = surface.snap(&centroid);
        let mids: Vec<Vec3> = (0..n)
            .map(|k| {
                rec.sides.get(k).and_then(|&c| self.halves.get(c)).and_then(|h| h.0.last().copied()).unwrap_or(centre)
            })
            .collect();
        let spokes: Vec<Vec<Vec3>> = mids.iter().map(|m| spoke(&surface, &centre, m, self.degree)).collect();
        let assigned = partition(rec, &centre, &mids, &axis);
        let patches = (0..n)
            .filter_map(|k| {
                let prev = (k + n - 1) % n;
                let (bottom, left) = (self.half(rec, k, true), reversed(&self.half(rec, prev, false)));
                let (right, top) = (reversed(&spokes[k]), reversed(&spokes[prev]));
                let ids = [rec.corners[k], self.mid_id(rec.sides[k]), self.centre_id(r), self.mid_id(rec.sides[prev])];
                finish_net([&bottom, &right, &top, &left], ids, &assigned[k], &rec.mean_normal)
            })
            .collect();
        (patches, centre)
    }
}

/// The region's points split by the angle around the centre between neighbouring spokes: the
/// points of the patch at corner `k`.
fn partition(rec: &RegionRec, centre: &Vec3, mids: &[Vec3], axis: &Vec3) -> Vec<Vec<Vec3>> {
    let e1 = any_perpendicular(axis);
    let e2 = axis.cross(&e1);
    let mut angles: Vec<(f64, usize)> =
        mids.iter().enumerate().map(|(k, m)| (angle_around(m, centre, &e1, &e2), k)).collect();
    angles.sort_by(|a, b| a.0.total_cmp(&b.0));
    let mut assigned: Vec<Vec<Vec3>> = vec![Vec::new(); mids.len()];
    for p in &rec.points {
        assigned[sector_corner(angle_around(p, centre, &e1, &e2), &angles)].push(*p);
    }
    assigned
}

/// Every region split into one patch per corner (see the module documentation). New point ids
/// start at `first_new_id`: one per curve (its midpoint), then one per region (its centre).
pub(crate) fn ngon_patches(recs: &[RegionRec], curves: &[Vec<Vec3>], first_new_id: u32, degree: usize) -> Patches {
    let halves = curves.iter().map(|c| split_bezier(c, 0.5)).collect();
    let split = Split { curves, halves, first_new_id, degree };
    let mut out = Patches::default();
    out.extra_points.extend(split.halves.iter().map(|h| h.0.last().copied().map_or([0.0; 3], a3)));
    for (r, rec) in recs.iter().enumerate().filter(|(_, r)| r.is_consistent(curves.len())) {
        let (patches, centre) = split.region(r, rec);
        out.extra_points.push(a3(centre));
        for (k, (net, corners)) in patches.into_iter().enumerate() {
            out.nets.push(net);
            out.corners.push(corners);
            out.region_triangles.push(if k == 0 { rec.triangles.clone() } else { Vec::new() });
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn planar_curve(a: Vec3, b: Vec3, degree: usize) -> Vec<Vec3> {
        (0..=degree).map(|k| a + (b - a) * (k as f64 / degree as f64)).collect()
    }

    /// A sector with no points keeps the Coons interior of its (planar) boundary.
    #[test]
    fn a_sector_without_points_gets_the_coons_interior() {
        let z = 100.0;
        let p = |x: f64, y: f64| Vec3::new(x, y, z);
        let (a, b, c, d) = (p(0.0, 0.0), p(10.0, 0.0), p(12.0, 9.0), p(-1.0, 8.0));
        let degree = 5;
        let (bottom, right, top, left) = (
            planar_curve(a, b, degree),
            planar_curve(b, c, degree),
            planar_curve(d, c, degree),
            planar_curve(a, d, degree),
        );
        for points in [Vec::new(), vec![p(1.0, 1.0); 3]] {
            let (net, _) = finish_net([&bottom, &right, &top, &left], [0, 1, 2, 3], &points, &Vec3::z()).unwrap();
            for row in &net {
                for q in row {
                    assert!((q[2] - z).abs() < 1e-9, "{q:?}");
                }
            }
            assert!((eval_net(&net, 0.5, 0.5).z - z).abs() < 1e-9);
        }
    }
}
