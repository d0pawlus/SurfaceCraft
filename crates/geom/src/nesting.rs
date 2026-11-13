//! Is one piece of a scan inside another, and does a surface cross itself? Both are answered from
//! a few deterministic probes (ray parity, sampled triangle pairs), not exhaustively: they are
//! diagnostics that tell "inner and outer skin of one part" from "separate parts" and a clean
//! surface from one with self-intersections.
use crate::{Aabb, Bvh, Mesh, Point3, Vec3, tri};

/// Probe directions: irrational-looking, so that rays do not run along mesh edges of grid-like scans.
const DIRECTIONS: [[f64; 3]; 3] = [
    [0.577_350_3, 0.311_283_5, 0.754_866_2],
    [-0.412_393_1, 0.801_938_5, 0.434_122_7],
    [0.283_561_9, -0.527_714_6, 0.801_287_4],
];
/// Crossings counted along one ray before it is given up.
const MAX_CROSSINGS: usize = 512;
/// Sample points of the inner piece that are tested.
pub const NEST_SAMPLES: usize = 5;

/// Do the boxes overlap (touching counts)?
#[must_use]
pub fn overlap(a: &Aabb, b: &Aabb) -> bool {
    !a.is_empty() && !b.is_empty() && (0..3).all(|k| a.min[k] <= b.max[k] && b.min[k] <= a.max[k])
}

/// Does `outer` contain `inner` (within `slack`)?
#[must_use]
pub fn contains(outer: &Aabb, inner: &Aabb, slack: f64) -> bool {
    !outer.is_empty()
        && !inner.is_empty()
        && (0..3).all(|k| outer.min[k] - slack <= inner.min[k] && inner.max[k] <= outer.max[k] + slack)
}

/// Crossings of the ray `origin + t·dir` with the surface (odd = inside a closed surface).
fn crossings(bvh: &Bvh, origin: &Point3, dir: &Vec3, eps: f64) -> usize {
    let (mut o, mut n) = (*origin, 0);
    while n < MAX_CROSSINGS {
        let Some((_, t)) = bvh.ray_cast(&o, dir) else { break };
        n += 1;
        o += dir * (t + eps);
    }
    n
}

/// Is `p` inside the closed surface `outer`? Majority of the parity along three directions.
#[must_use]
pub fn point_inside(outer: &Bvh, p: &Point3, eps: f64) -> bool {
    let odd = DIRECTIONS
        .iter()
        .filter(|d| crossings(outer, p, &Vec3::new(d[0], d[1], d[2]).normalize(), eps) % 2 == 1)
        .count();
    odd >= 2
}

/// Share of up to [`NEST_SAMPLES`] vertices of `inner` (an even stride over them) that lie inside
/// the closed surface `outer`. 0 for a mesh with no vertices.
#[must_use]
pub fn inside_share(inner: &Mesh, outer: &Bvh, eps: f64) -> f64 {
    let n = inner.positions.len();
    if n == 0 {
        return 0.0;
    }
    let count = NEST_SAMPLES.min(n);
    let inside =
        (0..count).filter_map(|k| inner.positions.get(k * n / count)).filter(|p| point_inside(outer, p, eps)).count();
    inside as f64 / count as f64
}

/// Does triangle `[a, b, c]` cross triangle `[d, e, f]`? (An edge of one passes through the other.)
fn crosses(x: &[Point3; 3], y: &[Point3; 3]) -> bool {
    let hit = |s: &[Point3; 3], t: &[Point3; 3]| {
        (0..3).any(|k| {
            let (p, q) = (s[k], s[(k + 1) % 3]);
            tri::ray_hit(&p, &(q - p), &t[0], &t[1], &t[2]).is_some_and(|u| u <= 1.0)
        })
    };
    hit(x, y) || hit(y, x)
}

/// Result of [`sampled_self_intersections`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct IntersectionSample {
    /// Triangles tested.
    pub tested: usize,
    /// Of them, triangles that cross another triangle they share no vertex with.
    pub crossing: usize,
}

/// Tests an even stride of at most `max_samples` triangles of `mesh` against the rest (through
/// `bvh`, built over the same mesh). Triangles that share a vertex are not tested against each other.
#[must_use]
pub fn sampled_self_intersections(mesh: &Mesh, bvh: &Bvh, max_samples: usize) -> IntersectionSample {
    let n = mesh.triangle_count();
    let count = max_samples.min(n);
    let mut out = IntersectionSample::default();
    for k in 0..count {
        let ti = k * n / count.max(1);
        let (Some(x), Some(idx)) = (mesh.triangle(ti), mesh.triangles.get(ti)) else { continue };
        out.tested += 1;
        let bx = Aabb::from_points(x.iter());
        let near = bvh.candidates(&|b| overlap(b, &bx));
        let found = near.iter().any(|&o| {
            let (Some(y), Some(oi)) = (mesh.triangle(o as usize), mesh.triangles.get(o as usize)) else { return false };
            o as usize != ti && !idx.iter().any(|v| oi.contains(v)) && crosses(&x, &y)
        });
        out.crossing += usize::from(found);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth;

    #[test]
    fn a_small_sphere_inside_a_big_one_is_nested_and_a_far_one_is_not() {
        let big = synth::sphere(Point3::origin(), 10.0, 32, 16);
        let bvh = Bvh::build(&big);
        let inner = synth::sphere(Point3::new(1.0, 0.0, 0.0), 3.0, 16, 8);
        let far = synth::sphere(Point3::new(30.0, 0.0, 0.0), 3.0, 16, 8);
        assert!((inside_share(&inner, &bvh, 1e-7) - 1.0).abs() < 1e-12);
        assert_eq!(inside_share(&far, &bvh, 1e-7), 0.0);
        assert!(contains(&big.bbox(), &inner.bbox(), 0.0));
        assert!(!contains(&big.bbox(), &far.bbox(), 0.0));
    }

    #[test]
    fn crossing_planes_are_found_and_a_clean_sphere_has_none() {
        let sphere = synth::sphere(Point3::origin(), 10.0, 24, 12);
        let bvh = Bvh::build(&sphere);
        assert_eq!(sampled_self_intersections(&sphere, &bvh, 500).crossing, 0);
        let mut m = synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 4.0, 4.0, 2, 2);
        let sheet = synth::plane(Point3::new(1.0, 1.0, -1.0), Vec3::x(), Vec3::z(), 2.0, 2.0, 1, 1);
        m.append(&sheet);
        let bvh = Bvh::build(&m);
        assert!(sampled_self_intersections(&m, &bvh, 100).crossing > 0);
    }
}
