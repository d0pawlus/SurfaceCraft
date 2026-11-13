//! Robust scoring of a pose: how much of the moving scan lies on the fixed surface, and how
//! tightly. Truncated, so surface that has no counterpart (partial overlap) costs a fixed amount
//! and does not drag a good pose down the way a mean or a median distance would.
use surfacecraft_geom::{Bvh, Mesh, Point3, Transform};

/// Weight of a conflict against a match in [`Fit::score`].
pub(super) const CONFLICT_WEIGHT: f64 = 3.0;
/// A point is in conflict beyond this many tolerances from a surface that is there to be hit.
const CONFLICT_TAUS: f64 = 2.0;

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
    /// A target without borders (conflicts are not judged against it).
    pub(super) fn plain(mesh: &'a Mesh) -> Self {
        Target { mesh, bvh: Bvh::build(mesh), open: Vec::new(), border: 0.0 }
    }

    pub(super) fn new(mesh: &'a Mesh, diag: f64) -> Self {
        let open = mesh
            .triangle_neighbours()
            .iter()
            .map(|n| n.iter().enumerate().fold(0u8, |m, (k, &x)| if x == u32::MAX { m | (1 << k) } else { m }))
            .collect();
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

    #[test]
    fn empty_input_is_a_poor_fit_not_a_panic() {
        let empty = Mesh::default();
        let f = fit_of(&Target::new(&empty, 1.0), &[], &Transform::identity(), 1.0, true);
        assert!(f.share == 0.0 && f.msac == 1.0);
        assert!(corners(&[]).is_empty());
        assert_eq!(median(Vec::new()), 0.0);
    }
}
