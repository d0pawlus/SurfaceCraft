//! Robust scoring of a pose: how much of the moving scan lies on the fixed surface, and how
//! tightly. Truncated, so surface that has no counterpart (partial overlap) costs a fixed amount
//! and does not drag a good pose down the way a mean or a median distance would.
use super::sample::Samples;
use surfacecraft_geom::{Bvh, Mat3, Mesh, Point3, Transform, Vec3};

/// Weight of a conflict against a match in [`Fit::score`].
pub(super) const CONFLICT_WEIGHT: f64 = 3.0;
/// A point is in conflict beyond this many tolerances from a surface that is there to be hit.
const CONFLICT_TAUS: f64 = 2.0;

/// Per triangle, bit `k` set when edge `k` (corner `k` to `k + 1`) is shared by no other triangle
/// (a border) or by more than one (non-manifold). Vertices are matched by position, so a mesh that
/// repeats its vertices per triangle (unwelded) has the borders of the welded one.
fn open_edges(mesh: &Mesh, diag: f64) -> Vec<u8> {
    let q = 1.0 / (1e-6 * diag).max(1e-300);
    let cell = |p: &Point3| [(p.x * q).round() as i64, (p.y * q).round() as i64, (p.z * q).round() as i64];
    let mut half: Vec<([i64; 3], [i64; 3], u32)> = Vec::with_capacity(mesh.triangle_count() * 3);
    for i in 0..mesh.triangle_count() {
        let Some(c) = mesh.triangle(i) else { continue };
        for k in 0..3 {
            let (a, b) = (cell(&c[k]), cell(&c[(k + 1) % 3]));
            if a != b {
                half.push((a.min(b), a.max(b), (i * 3 + k) as u32));
            }
        }
    }
    surfacecraft_geom::par::sort_unstable(&mut half);
    let mut open = vec![0u8; mesh.triangle_count()];
    let mut i = 0;
    while i < half.len() {
        let mut j = i + 1;
        while j < half.len() && half.get(j).map(|h| (h.0, h.1)) == half.get(i).map(|h| (h.0, h.1)) {
            j += 1;
        }
        if j - i != 2 {
            for h in half.get(i..j).unwrap_or(&[]) {
                if let Some(m) = open.get_mut((h.2 / 3) as usize) {
                    *m |= 1 << (h.2 % 3);
                }
            }
        }
        i = j;
    }
    open
}

/// The fixed scan as a target: its BVH and which triangle edges are open (the scan ends there).
pub(super) struct Target<'a> {
    pub mesh: &'a Mesh,
    pub bvh: Bvh,
    /// Per triangle, bit `k` set when edge `k` (corner `k` to `k + 1`) has no neighbour.
    open: Vec<u8>,
    /// Closest points within this of an open edge count as lying on the scan's border.
    border: f64,
}

impl<'a> Target<'a> {
    pub(super) fn new(mesh: &'a Mesh, diag: f64) -> Self {
        let open = open_edges(mesh, diag);
        Target { mesh, bvh: Bvh::build(mesh), open, border: 1e-7 * diag }
    }

    /// Does the closest point `q` of triangle `tri` lie on an open edge of the scan?
    fn on_border(&self, tri: usize, q: &Point3) -> bool {
        let (Some(&bits), Some(c)) = (self.open.get(tri), self.mesh.triangle(tri)) else { return false };
        (0..3).any(|k| {
            let (a, b) = (c[k], c[(k + 1) % 3]);
            bits >> k & 1 == 1 && dist_to_segment(q, &a, &b) <= self.border
        })
    }
}

fn dist_to_segment(p: &Point3, a: &Point3, b: &Point3) -> f64 {
    let ab = b - a;
    let len2 = ab.norm_squared();
    let t = if len2 > 0.0 { ((p - a).dot(&ab) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p - (a + ab * t)).norm()
}

/// How a pose fits a set of points to a surface at tolerance `tau`.
#[derive(Clone, Copy, Debug)]
pub(super) struct Fit {
    /// Share (0..1) of the points within `tau` of the surface.
    pub share: f64,
    /// Share (0..1) of the points that lie where the surface is, but off it (beyond 2 `tau`, and
    /// the closest point is not at the end of the scan). Surface beyond the end of the other scan
    /// has no counterpart and is neither a match nor a conflict.
    pub conflict: f64,
    /// RMS distance (mm) of the matched points.
    pub rms: f64,
    /// Mean of min(distance, tau)²: lower is better; a point with no counterpart counts `tau²`.
    pub msac: f64,
}

impl Fit {
    /// Matches minus weighted conflicts, scaled so that for a surface without borders a share
    /// difference is a score difference: higher is better.
    pub(super) fn score(&self) -> f64 {
        (self.share - CONFLICT_WEIGHT * self.conflict) / (1.0 + CONFLICT_WEIGHT)
    }
}

/// How far `best` stands clear of `other`, in shares of the moving scan's area: the larger of the
/// matches `other` lacks and the conflicts `other` has more. Negative when `other` is better in
/// both. This is the quantity [`super::AMBIGUITY_MARGIN`] bounds.
pub(super) fn clearance(best: &Fit, other: &Fit) -> f64 {
    (best.share - other.share).max(other.conflict - best.conflict)
}

/// Scores `pts` moved by `t` against `target`; conflicts are only counted when `conflicts`.
pub(super) fn fit_of(target: &Target, pts: &[Point3], t: &Transform, tau: f64, conflicts: bool) -> Fit {
    let reach = if conflicts { f64::INFINITY } else { tau };
    let dist: Vec<Option<(f64, bool)>> = surfacecraft_geom::par::map_slice(pts, |p| {
        let h = target.bvh.closest(&t.apply(p), reach)?;
        let conflict = conflicts && h.distance > CONFLICT_TAUS * tau && !target.on_border(h.triangle, &h.point);
        Some((h.distance, conflict))
    });
    let n = dist.len();
    if n == 0 {
        return Fit { share: 0.0, conflict: 0.0, rms: f64::INFINITY, msac: tau * tau };
    }
    let (mut inliers, mut bad, mut sq) = (0usize, 0usize, 0.0f64);
    for (d, conflict) in dist.iter().flatten() {
        if *d <= tau {
            inliers += 1;
            sq += d * d;
        }
        bad += usize::from(*conflict);
    }
    let outliers = (n - inliers) as f64;
    Fit {
        share: inliers as f64 / n as f64,
        conflict: bad as f64 / n as f64,
        rms: if inliers > 0 { (sq / inliers as f64).sqrt() } else { f64::INFINITY },
        msac: (sq + outliers * tau * tau) / n as f64,
    }
}

/// A direction in which the matched part may be loose: a small rigid motion (rotation vector
/// `omega` and shift, about `centre`) scaled so that its largest displacement of a matched point
/// is the step asked for.
#[derive(Clone, Copy, Debug)]
pub(super) struct Twist {
    centre: Point3,
    omega: Vec3,
    shift: Vec3,
}

impl Twist {
    /// `t` followed by this motion (`sign` +1 or -1 picks the sense).
    pub(super) fn displace(&self, t: &Transform, sign: f64) -> Transform {
        let w = self.omega * sign;
        let angle = w.norm();
        let rot =
            if angle > 1e-300 { Transform::rotation_axis_angle(&(w / angle), angle) } else { Transform::identity() };
        Transform::translation(self.centre.coords + self.shift * sign)
            .then_after(&rot)
            .then_after(&Transform::translation(-self.centre.coords))
            .then_after(t)
    }
}

/// How firmly the matched part pins a pose.
#[derive(Clone, Debug)]
pub(super) struct Stability {
    /// Smallest eigenvalue of the normalised point-to-plane system (see [`stability`]).
    pub min: f64,
    /// The weakest directions, weakest first (see [`stability`]).
    pub weak: Vec<Twist>,
}

/// Directions always returned: a rigid motion has at most three free directions on a surface that
/// slides (a plane patch: two shifts and a turn; a sphere patch: three turns), and any free
/// motion is a combination of free eigenvectors, so the three weakest cannot miss one.
const ALWAYS_WEAK: usize = 3;
/// Most directions returned.
const MAX_WEAK: usize = 6;

/// How firmly the matched part pins the pose (geometric stability, after Gelfand et al. 2003):
/// the eigenvalues of the point-to-plane normal equations of the matched points at `t`. The
/// smallest is 0 when the part can slide or turn freely. The eigenvectors of the three weakest
/// (and of any further one below `weak_below`, up to six) are returned as motions of largest
/// displacement `step` mm, to be tried: no verdict depends on where an eigenvalue lies.
///
/// Two things keep it comparable between scans: the surface normals are the plane-fitted ones of
/// the nearest fixed sample (a raw face normal of a fine noisy mesh tilts by about noise over edge
/// length, which would pin a sliding patch by tilt squared), and the rotation about each principal
/// axis of the matched points is scaled by that axis's own lever arm, so a pinned direction reads
/// alike for a compact part and a 20:1 bar.
pub(super) fn stability(
    target: &Target,
    fixed: &Samples,
    pts: &[Point3],
    t: &Transform,
    tau: f64,
    weak_below: f64,
    step: f64,
) -> Stability {
    let none = Stability { min: 0.0, weak: Vec::new() };
    let hits: Vec<Option<(Point3, Vec3)>> = surfacecraft_geom::par::map_slice(pts, |p| {
        let q = t.apply(p);
        let h = target.bvh.closest(&q, tau)?;
        let (j, _) = fixed.tree.nearest(&h.point)?;
        Some((q, *fixed.fit.get(j)?))
    });
    let used: Vec<&(Point3, Vec3)> = hits.iter().flatten().collect();
    let Some(frame) = Frame::of(&used) else { return none };
    let Some((eig, order)) = weakest_first(&frame.system(&used)) else { return none };
    let min = eig.eigenvalues[order[0]].max(0.0);
    let weak = frame.weak_twists(&used, &eig, &order, weak_below, step);
    Stability { min: if min.is_finite() { min } else { 0.0 }, weak }
}

/// The principal frame of the matched points: centroid, axes, and the lever arm of the rotation
/// about each axis (the RMS distance of the points from that axis).
struct Frame {
    centre: Vec3,
    axes: Vec<Vec3>,
    lever: Vec<f64>,
}

impl Frame {
    fn of(used: &[&(Point3, Vec3)]) -> Option<Frame> {
        if used.len() < 6 {
            return None;
        }
        let n = used.len() as f64;
        let centre = used.iter().map(|h| h.0.coords).sum::<Vec3>() / n;
        let cov =
            used.iter().fold(Mat3::zeros(), |m, h| m + (h.0.coords - centre) * (h.0.coords - centre).transpose()) / n;
        if cov.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let eig = cov.symmetric_eigen();
        let trace = cov.trace();
        let lever: Vec<f64> = (0..3).map(|i| (trace - eig.eigenvalues[i]).max(0.0).sqrt()).collect();
        if lever.iter().any(|l| !(*l > 0.0)) {
            return None;
        }
        let axes = (0..3).map(|i| eig.eigenvectors.column(i).into_owned()).collect();
        Some(Frame { centre, axes, lever })
    }

    /// The normal equations of the matched points: rotation about each axis scaled by its lever
    /// arm, then the shift along each axis; divided by the number of points.
    fn system(&self, used: &[&(Point3, Vec3)]) -> nalgebra::Matrix6<f64> {
        let mut a = nalgebra::Matrix6::<f64>::zeros();
        for (q, nrm) in used.iter().map(|h| (&h.0, &h.1)) {
            let r = (q.coords - self.centre).cross(nrm);
            let mut row = nalgebra::Vector6::zeros();
            for (i, e) in self.axes.iter().enumerate() {
                row[i] = r.dot(e) / self.lever[i];
                row[3 + i] = nrm.dot(e);
            }
            a += row * row.transpose();
        }
        a / used.len() as f64
    }

    /// The motions of the weakest eigenvectors (see [`stability`]), each scaled to move the matched
    /// point that moves most by `step`.
    fn weak_twists(
        &self,
        used: &[&(Point3, Vec3)],
        eig: &nalgebra::linalg::SymmetricEigen<f64, nalgebra::U6>,
        order: &[usize],
        weak_below: f64,
        step: f64,
    ) -> Vec<Twist> {
        let mut weak = Vec::new();
        for (rank, &k) in order.iter().take(MAX_WEAK).enumerate() {
            if rank >= ALWAYS_WEAK && eig.eigenvalues[k] >= weak_below {
                break;
            }
            let v = eig.eigenvectors.column(k);
            let (mut omega, mut shift) = (Vec3::zeros(), Vec3::zeros());
            for (i, e) in self.axes.iter().enumerate() {
                omega += e * (v[i] / self.lever[i]);
                shift += e * v[3 + i];
            }
            let reach =
                used.iter().map(|h| (omega.cross(&(h.0.coords - self.centre)) + shift).norm()).fold(0.0, f64::max);
            if reach > 0.0 && reach.is_finite() {
                let scale = step / reach;
                weak.push(Twist { centre: Point3::from(self.centre), omega: omega * scale, shift: shift * scale });
            }
        }
        weak
    }
}

/// The eigen decomposition of a symmetric matrix with the indices of its eigenvalues, smallest
/// first (ties by index, so the order never depends on the platform's rounding). `None` when the
/// matrix is not finite: the iteration of the decomposition does not end on such input.
fn weakest_first(
    a: &nalgebra::Matrix6<f64>,
) -> Option<(nalgebra::linalg::SymmetricEigen<f64, nalgebra::U6>, Vec<usize>)> {
    if a.iter().any(|v| !v.is_finite()) {
        return None;
    }
    let eig = a.symmetric_eigen();
    let mut order: Vec<usize> = (0..6).collect();
    order.sort_by(|&i, &j| eig.eigenvalues[i].total_cmp(&eig.eigenvalues[j]).then(i.cmp(&j)));
    Some((eig, order))
}

/// The eight corners of the bounding box of `pts`: probes for comparing two poses.
pub(super) fn corners(pts: &[Point3]) -> Vec<Point3> {
    let mut b = surfacecraft_geom::Aabb::empty();
    pts.iter().for_each(|p| b.grow(p));
    if b.is_empty() {
        return Vec::new();
    }
    (0..8u8)
        .map(|k| {
            Point3::new(
                if k & 1 == 0 { b.min.x } else { b.max.x },
                if k & 2 == 0 { b.min.y } else { b.max.y },
                if k & 4 == 0 { b.min.z } else { b.max.z },
            )
        })
        .collect()
}

/// Largest distance (mm) a probe point lies apart under the two poses.
pub(super) fn pose_gap(a: &Transform, b: &Transform, probes: &[Point3]) -> f64 {
    a.max_displacement(b, probes)
}

/// Median of `values` (0 for none).
pub(super) fn median(mut values: Vec<f64>) -> f64 {
    if values.is_empty() {
        return 0.0;
    }
    values.sort_by(f64::total_cmp);
    values.get(values.len() / 2).copied().unwrap_or(0.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Vec3, synth};

    fn plane_mesh() -> Mesh {
        synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 20.0, 20.0, 10, 10)
    }

    #[test]
    fn points_on_the_surface_score_zero_and_far_points_cost_tau_squared() {
        let mesh = plane_mesh();
        let bvh = Target::new(&mesh, 30.0);
        let on = [Point3::new(5.0, 5.0, 0.0), Point3::new(7.0, 3.0, 0.0)];
        let f = fit_of(&bvh, &on, &Transform::identity(), 0.5, true);
        assert!((f.share - 1.0).abs() < 1e-12 && f.msac < 1e-12);
        let mixed = [Point3::new(5.0, 5.0, 0.0), Point3::new(5.0, 5.0, 9.0)];
        let f = fit_of(&bvh, &mixed, &Transform::identity(), 0.5, true);
        assert!((f.share - 0.5).abs() < 1e-12);
        assert!((f.msac - 0.125).abs() < 1e-12, "{}", f.msac);
        // the point 9 mm above the middle of the plane hits its interior: a conflict
        assert!((f.conflict - 0.5).abs() < 1e-12);
    }

    /// A point beyond the edge of the other scan has no counterpart: no conflict.
    #[test]
    fn surface_beyond_the_end_of_the_other_scan_is_not_a_conflict() {
        let mesh = plane_mesh();
        let target = Target::new(&mesh, 30.0);
        let beyond = [Point3::new(25.0, 5.0, 0.0), Point3::new(5.0, -4.0, 0.0), Point3::new(25.0, 5.0, 3.0)];
        let f = fit_of(&target, &beyond, &Transform::identity(), 0.5, true);
        assert_eq!(f.conflict, 0.0);
        assert_eq!(f.share, 0.0);
    }

    /// Finding 4: a matrix that is not finite has no eigenvalue to report (and must not be handed to
    /// the iteration).
    #[test]
    fn a_non_finite_system_is_not_decomposed() {
        let mut a = nalgebra::Matrix6::<f64>::identity();
        assert!(weakest_first(&a).is_some());
        a[(2, 3)] = f64::NAN;
        assert!(weakest_first(&a).is_none());
        a[(2, 3)] = f64::INFINITY;
        assert!(weakest_first(&a).is_none());
    }

    #[test]
    fn empty_input_is_a_poor_fit_not_a_panic() {
        let empty = Mesh::default();
        let f = fit_of(&Target::new(&empty, 1.0), &[], &Transform::identity(), 1.0, true);
        assert!(f.share == 0.0 && f.msac == 1.0);
        assert!(corners(&[]).is_empty());
        assert_eq!(median(Vec::new()), 0.0);
    }
}
