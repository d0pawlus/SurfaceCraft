//! Area-uniform samples of a scan with smoothed normals: what the coarse stage works on.
//!
//! The samples are drawn by area from a fixed seed, so they depend on the surface and not on how it
//! is triangulated (two scans of one part with different vertices give the same kind of samples).
use surfacecraft_geom::{Cancel, KdTree, Mesh, Point3, Rng, Vec3};

/// Seed of the samples.
const SEED: u64 = 0x5EED_0F5A;
/// Normals are averaged over neighbours within this many sample spacings.
const NORMAL_RADIUS: f64 = 2.5;
/// Neighbours whose normal is within this cosine of the sample's own take part in the average
/// (so a sharp edge is not rounded off).
const NORMAL_AGREE_COS: f64 = 0.866;

/// Points on a scan with a unit normal each.
pub(super) struct Samples {
    /// Sample positions.
    pub pts: Vec<Point3>,
    /// Smoothed unit normals, one per sample (oriented like the triangles).
    pub nrm: Vec<Vec3>,
    /// Spatial index of `pts`.
    pub tree: KdTree,
}

/// Number of samples for a scan of `area` at `spacing`, at least 64 and at most `cap`.
pub(super) fn count_for(area: f64, spacing: f64, cap: usize) -> usize {
    let n = (area / (spacing * spacing)).ceil();
    if n.is_finite() { (n as usize).clamp(64, cap.max(64)) } else { cap.max(64) }
}

/// `count` samples of `mesh` with smoothed normals.
pub(super) fn sample(mesh: &Mesh, count: usize, spacing: f64, salt: u64, cancel: &Cancel) -> Option<Samples> {
    let mut rng = Rng::new(SEED ^ salt);
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
    Some(Samples { pts, nrm, tree })
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
