//! Area-uniform samples of a scan with smoothed normals: what the coarse stage works on.
//!
//! The samples are drawn by area from a fixed seed, so they depend on the surface and not on how it
//! is triangulated (two scans of one part with different vertices give the same kind of samples).
use surfacecraft_geom::{Cancel, KdTree, Mesh, Point3, Rng, Vec3};

/// Seed of the samples.
const SEED: u64 = 0x5EED_0F5A;
/// Normals are averaged over neighbours within this many sample spacings.
const NORMAL_RADIUS: f64 = 2.5;
/// Plane fits for `Samples::fit` reach this many sample spacings.
const FIT_RADIUS: f64 = 4.0;
/// Neighbours whose normal is within this cosine of the sample's own take part in the average
/// (so a sharp edge is not rounded off).
const NORMAL_AGREE_COS: f64 = 0.866;

/// Points on a scan with a unit normal each.
pub(super) struct Samples {
    /// Sample positions.
    pub pts: Vec<Point3>,
    /// Smoothed unit normals, one per sample (oriented like the triangles).
    pub nrm: Vec<Vec3>,
    /// Normals fitted to the positions of the neighbours (plane fit over the neighbours that lie on
    /// the same face): unlike the averaged face normals, a fine noisy mesh does not tilt them. Used
    /// where a tilt would count (the stability of a pose).
    pub fit: Vec<Vec3>,
    /// Spatial index of `pts`.
    pub tree: KdTree,
}

/// Number of samples for a scan of `area` at `spacing`, at least 64 and at most `cap`.
pub(super) fn count_for(area: f64, spacing: f64, cap: usize) -> usize {
    let n = (area / (spacing * spacing)).ceil();
    if n.is_finite() { (n as usize).clamp(64, cap.max(64)) } else { cap.max(64) }
}

/// `count` samples of `mesh` with smoothed normals. `stream` is the index of the deterministic
/// sample stream (the fixed scan and the moving scan draw from different ones); it is a plain
/// sequence number, not a secret and not cryptographic.
pub(super) fn sample(mesh: &Mesh, count: usize, spacing: f64, stream: u64, cancel: &Cancel) -> Option<Samples> {
    let mut rng = Rng::new(SEED ^ stream);
    let raw = mesh.sample_points(count, &mut rng);
    if raw.len() < 16 || cancel.is_cancelled() {
        return None;
    }
    let pts: Vec<Point3> = raw.iter().map(|(p, _)| *p).collect();
    let raw_n: Vec<Vec3> = raw.iter().map(|(_, n)| *n).collect();
    let tree = KdTree::build(&pts);
    let radius = NORMAL_RADIUS * spacing;
    let nrm = surfacecraft_geom::par::map(pts.len(), |i| {
        let (Some(p), Some(n0)) = (pts.get(i), raw_n.get(i)) else { return Vec3::z() };
        let mut sum = *n0;
        for j in tree.within(p, radius) {
            if j != i
                && let Some(nj) = raw_n.get(j)
                && nj.dot(n0) > NORMAL_AGREE_COS
            {
                sum += nj;
            }
        }
        surfacecraft_geom::normalize(sum).unwrap_or(*n0)
    });
    let fit = fitted_normals(&pts, &nrm, &tree, FIT_RADIUS * spacing);
    Some(Samples { pts, nrm, fit, tree })
}

/// The normal of the plane through the neighbours of each sample that lie on the sample's face
/// (their smoothed normal within `NORMAL_AGREE_COS` of its own), oriented like the smoothed normal;
/// the smoothed normal itself where there are too few neighbours.
fn fitted_normals(pts: &[Point3], nrm: &[Vec3], tree: &KdTree, radius: f64) -> Vec<Vec3> {
    surfacecraft_geom::par::map(pts.len(), |i| {
        let (Some(p), Some(n0)) = (pts.get(i), nrm.get(i)) else { return Vec3::z() };
        let near: Vec<Point3> = tree
            .within(p, radius)
            .into_iter()
            .filter(|&j| nrm.get(j).is_some_and(|nj| nj.dot(n0) > NORMAL_AGREE_COS))
            .filter_map(|j| pts.get(j).copied())
            .collect();
        if near.len() < 8 {
            return *n0;
        }
        let c = near.iter().map(|q| q.coords).sum::<Vec3>() / near.len() as f64;
        let cov =
            near.iter().fold(surfacecraft_geom::Mat3::zeros(), |m, q| m + (q.coords - c) * (q.coords - c).transpose());
        if cov.iter().any(|v| !v.is_finite()) {
            return *n0;
        }
        let eig = cov.symmetric_eigen();
        let k = (0..3).min_by(|&a, &b| eig.eigenvalues[a].total_cmp(&eig.eigenvalues[b])).unwrap_or(0);
        let n: Vec3 = eig.eigenvectors.column(k).into_owned();
        if n.dot(n0) < 0.0 { -n } else { n }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::synth;

    #[test]
    fn samples_of_a_box_keep_the_face_normals_at_the_edges() {
        let m = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 6);
        let s = sample(&m, 800, 1.5, 0, &Cancel::new()).unwrap();
        assert_eq!(s.pts.len(), 800);
        // every normal is (nearly) an axis: averaging did not mix the faces
        assert!(s.nrm.iter().all(|n| n.amax() > 0.99), "a normal was rounded off an edge");
    }

    #[test]
    fn same_surface_same_samples() {
        let m = synth::bracket();
        let a = sample(&m, 500, 2.0, 1, &Cancel::new()).unwrap();
        let b = sample(&m, 500, 2.0, 1, &Cancel::new()).unwrap();
        assert_eq!(a.pts, b.pts);
        assert!(sample(&Mesh::default(), 500, 2.0, 1, &Cancel::new()).is_none());
    }
}
