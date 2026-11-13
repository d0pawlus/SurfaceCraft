//! Moving the vertices of the voxel surface onto the scan so that the accuracy of the result is
//! that of the scan, not of the voxel size: Taubin smoothing of the stair-stepped surface (Taubin
//! 1995, "A signal processing approach to fair surface design"), then a few steps of a robust
//! moving-least-squares projection (Levin 2003 "Mesh-independent surface interpolation"; Alexa et
//! al. 2003, "Computing and rendering point set surfaces"): each vertex moves along its normal to
//! the height of a quadric fitted to the scan vertices around it, weighted by distance and, so that
//! a second wall or a crease does not pull, by height above the current position.
use crate::{KdTree, Point3, Vec3, any_perpendicular, normalize, par};
use nalgebra::{SMatrix, SVector};

/// Taubin shrink and inflate factors (lambda, mu) and the number of smoothing rounds.
const TAUBIN: (f64, f64, usize) = (0.5, -0.53, 5);
/// Least-squares unknowns of the height field `c0 + c1 x + c2 y + c3 x² + c4 xy + c5 y²`.
const TERMS: usize = 6;
/// Fewer scan vertices than this around a vertex: no quadric, a weighted mean only.
const MIN_QUADRIC_POINTS: usize = 10;
/// Weights below this are ignored.
const MIN_WEIGHT: f64 = 1e-3;
/// Samples of the scan used to measure its median edge length.
const EDGE_SAMPLES: usize = 20_000;

/// Neighbours of every vertex in compressed form.
pub(super) struct Rings {
    start: Vec<u32>,
    nbr: Vec<u32>,
}

impl Rings {
    pub(super) fn new(vertices: usize, triangles: &[[u32; 3]]) -> Self {
        let mut pairs: Vec<u64> = Vec::with_capacity(triangles.len() * 6);
        for t in triangles {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                pairs.push((u64::from(a) << 32) | u64::from(b));
                pairs.push((u64::from(b) << 32) | u64::from(a));
            }
        }
        par::sort_unstable(&mut pairs);
        pairs.dedup();
        let mut start = vec![0u32; vertices + 1];
        for p in &pairs {
            if let Some(s) = start.get_mut((p >> 32) as usize + 1) {
                *s += 1;
            }
        }
        for v in 0..vertices {
            start[v + 1] += start[v];
        }
        Self { start, nbr: pairs.iter().map(|p| (p & 0xffff_ffff) as u32).collect() }
    }

    fn of(&self, v: usize) -> &[u32] {
        match (self.start.get(v), self.start.get(v + 1)) {
            (Some(&a), Some(&b)) => self.nbr.get(a as usize..b as usize).unwrap_or(&[]),
            _ => &[],
        }
    }
}

fn smooth_once(pos: &[Point3], rings: &Rings, factor: f64) -> Vec<Point3> {
    par::map(pos.len(), |v| {
        let ring = rings.of(v);
        let Some(p) = pos.get(v) else { return Point3::origin() };
        if ring.is_empty() {
            return *p;
        }
        let mean = ring.iter().filter_map(|&w| pos.get(w as usize)).fold(Vec3::zeros(), |a, q| a + q.coords)
            / ring.len() as f64;
        p + (Point3::from(mean) - p) * factor
    })
}

/// Taubin smoothing: alternating shrink and inflate steps, which smooths without shrinking.
pub(super) fn taubin(pos: &[Point3], rings: &Rings) -> Vec<Point3> {
    let mut cur = pos.to_vec();
    for _ in 0..TAUBIN.2 {
        cur = smooth_once(&cur, rings, TAUBIN.0);
        cur = smooth_once(&cur, rings, TAUBIN.1);
    }
    cur
}

/// Median edge length of a sample of the triangles (0 for none).
pub(super) fn median_edge(pos: &[Point3], triangles: &[[u32; 3]]) -> f64 {
    let n = triangles.len();
    let count = EDGE_SAMPLES.min(n);
    let mut lengths: Vec<f64> = Vec::with_capacity(count * 3);
    for k in 0..count {
        let Some(t) = triangles.get(k * n / count.max(1)) else { continue };
        for e in 0..3 {
            if let (Some(a), Some(b)) = (pos.get(t[e] as usize), pos.get(t[(e + 1) % 3] as usize)) {
                lengths.push((a - b).norm());
            }
        }
    }
    if lengths.is_empty() {
        return 0.0;
    }
    let mid = lengths.len() / 2;
    *lengths.select_nth_unstable_by(mid, f64::total_cmp).1
}

/// How the projection weighs and limits the scan vertices.
#[derive(Clone, Copy)]
pub(super) struct Fit {
    /// Search radius (mm); the distance weights are Gaussian with sigma `radius / 2`.
    pub radius: f64,
    /// Height above the current position at which a scan vertex is down-weighted (mm).
    pub height: f64,
    /// Largest distance a vertex may move in total (mm).
    pub max_move: f64,
}

/// Height of the quadric fitted around `p` (frame `n`, `t1`, `t2`), and its gradient `(c1, c2)`.
fn quadric(p: &Point3, frame: &(Vec3, Vec3, Vec3), scan: &KdTree, fit: &Fit, robust: f64) -> Option<(f64, f64, f64)> {
    let (n, t1, t2) = frame;
    let rho = fit.radius;
    let sigma2 = (0.5 * rho) * (0.5 * rho);
    let mut ata = SMatrix::<f64, TERMS, TERMS>::zeros();
    let mut atb = SVector::<f64, TERMS>::zeros();
    let (mut sum_w, mut sum_wz, mut used) = (0.0, 0.0, 0usize);
    for idx in scan.within(p, rho) {
        let Some(q) = scan.point(idx) else { continue };
        let d = q - p;
        let (x, y, z) = (d.dot(t1), d.dot(t2), d.dot(n));
        let w = (-(x * x + y * y + z * z) / (2.0 * sigma2)).exp() * (-0.5 * (z / robust) * (z / robust)).exp();
        if w < MIN_WEIGHT {
            continue;
        }
        let (u, v) = (x / rho, y / rho);
        let row = SVector::<f64, TERMS>::from_column_slice(&[1.0, u, v, u * u, u * v, v * v]);
        ata += row * row.transpose() * w;
        atb += row * (w * z / rho);
        (sum_w, sum_wz, used) = (sum_w + w, sum_wz + w * z, used + 1);
    }
    if used < 3 || sum_w <= 0.0 {
        return None;
    }
    if used >= MIN_QUADRIC_POINTS
        && let Some(c) = ata.lu().solve(&atb)
        && c.iter().all(|x| x.is_finite())
    {
        return Some((c[0] * rho, c[1], c[2]));
    }
    Some((sum_wz / sum_w, 0.0, 0.0))
}

/// One vertex projected: three rounds of fit and move along the normal.
fn project_vertex(p: &Point3, n: &Vec3, scan: &KdTree, fit: &Fit, h: f64) -> Point3 {
    let Some(mut normal) = normalize(*n) else { return *p };
    let mut cur = *p;
    for robust in [fit.height.max(1.2 * h), fit.height.max(0.6 * h), fit.height] {
        let t1 = any_perpendicular(&normal);
        let t2 = normal.cross(&t1);
        let Some((height, c1, c2)) = quadric(&cur, &(normal, t1, t2), scan, fit, robust) else { return cur };
        cur += normal * height.clamp(-1.5 * h, 1.5 * h);
        normal = normalize(normal - t1 * c1 - t2 * c2).unwrap_or(normal);
        if (cur - p).norm() > fit.max_move {
            return *p;
        }
    }
    cur
}

/// Projects every vertex of the surface onto the scan (`scan` is a tree over the scan's vertices).
pub(super) fn project(pos: &[Point3], normals: &[Vec3], scan: &KdTree, fit: &Fit, h: f64) -> Vec<Point3> {
    par::map(pos.len(), |v| match (pos.get(v), normals.get(v)) {
        (Some(p), Some(n)) => project_vertex(p, n, scan, fit, h),
        _ => Point3::origin(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Rng, synth};

    #[test]
    fn projecting_onto_a_noisy_plane_recovers_the_plane() {
        let mut plane = synth::plane(Point3::new(-10.0, -10.0, 0.0), Vec3::x(), Vec3::y(), 20.0, 20.0, 60, 60);
        synth::add_noise(&mut plane, 0.03, &mut Rng::new(5));
        let tree = KdTree::build(&plane.positions);
        let start: Vec<Point3> = (0..50).map(|k| Point3::new(-2.0 + 0.08 * k as f64, 0.5, 0.2)).collect();
        let normals = vec![Vec3::z(); start.len()];
        let fit = Fit { radius: 1.2, height: 0.1, max_move: 0.5 };
        let out = project(&start, &normals, &tree, &fit, 0.3);
        let worst = out.iter().map(|p| p.z.abs()).fold(0.0, f64::max);
        assert!(worst < 0.03, "{worst}");
    }

    #[test]
    fn projection_does_not_shrink_a_sphere() {
        let mut sphere = synth::sphere(Point3::origin(), 10.0, 80, 40);
        sphere.weld(1e-9);
        let tree = KdTree::build(&sphere.positions);
        let start: Vec<Point3> =
            (0..40).map(|k| Point3::from(Vec3::new(1.0, 0.3 * k as f64 / 40.0, 0.2).normalize() * 10.15)).collect();
        let normals: Vec<Vec3> = start.iter().map(|p| p.coords.normalize()).collect();
        let fit = Fit { radius: 1.5, height: 0.1, max_move: 0.5 };
        let out = project(&start, &normals, &tree, &fit, 0.3);
        let worst = out.iter().map(|p| (p.coords.norm() - 10.0).abs()).fold(0.0, f64::max);
        assert!(worst < 0.01, "{worst}");
    }
}
