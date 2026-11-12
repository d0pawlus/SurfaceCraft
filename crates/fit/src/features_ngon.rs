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
use crate::features::{NetN, eval_net, fit_curve, fit_interior, split_bezier};
use crate::mls::ScanSurface;
use std::f64::consts::TAU;
use surfacecraft_geom::{KdTree, Point3, Vec3, any_perpendicular};

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
fn finish_net(sides: [&[Vec3]; 4], corners: [u32; 4], points: &[Vec3], mean_normal: &Vec3) -> (NetN, [u32; 4]) {
    let n = sides[0].len().saturating_sub(1);
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
        return (net, corners);
    }
    // swap u and v: transposes the net and the corner order c0, c3, c2, c1
    let mut t = net.clone();
    for i in 0..=n {
        for j in 0..=n {
            t[i][j] = net[j][i];
        }
    }
    (t, [corners[0], corners[3], corners[2], corners[1]])
}

/// One patch per four-sided region.
pub(crate) fn quad_patches(recs: &[RegionRec], curves: &[Vec<Vec3>]) -> Patches {
    let mut out = Patches::default();
    for rec in recs {
        let s: Vec<Vec<Vec3>> = (0..4).map(|k| side_from_start(rec, k, curves)).collect();
        // c0→c1, c1→c2, c3→c2, c0→c3
        let (top, left) = (reversed(&s[2]), reversed(&s[3]));
        let c = [rec.corners[0], rec.corners[1], rec.corners[2], rec.corners[3]];
        let (net, corners) = finish_net([&s[0], &s[1], &top, &left], c, &rec.points, &rec.mean_normal);
        out.nets.push(net);
        out.corners.push(corners);
        out.region_triangles.push(rec.triangles.clone());
    }
    out
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
            .map(|k| {
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
    for (r, rec) in recs.iter().enumerate() {
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
            let (net, _) = finish_net([&bottom, &right, &top, &left], [0, 1, 2, 3], &points, &Vec3::z());
            for row in &net {
                for q in row {
                    assert!((q[2] - z).abs() < 1e-9, "{q:?}");
                }
            }
            assert!((eval_net(&net, 0.5, 0.5).z - z).abs() < 1e-9);
        }
    }
}
