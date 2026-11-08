//! Scattered region points → a regular grid, the input the kernel's least-squares surface fit
//! takes (design doc §6.3 gap: solidcore fits regular grids only; decision for M3: resample
//! here — mesh-side computation — and let the kernel build the NURBS).
//!
//! The region is parameterised over its best-fit plane (it must be a height field over it,
//! true for one patch of a freeform region). Each grid node gets a height from a weighted
//! quadratic (moving least squares) fit of its nearest points.
use nalgebra::{Matrix6, Vector6};
use surfacecraft_geom::{KdTree, Point3, Vec3, any_perpendicular};

/// A grid over a region.
#[derive(Clone, Debug, PartialEq)]
pub struct HeightGrid {
    /// `grid[i][j]`, i along the plane's u axis.
    pub grid: Vec<Vec<[f64; 3]>>,
    /// Plane origin.
    pub origin: Point3,
    /// Plane u axis.
    pub u: Vec3,
    /// Plane v axis.
    pub v: Vec3,
    /// Plane normal (the height direction).
    pub n: Vec3,
    /// Largest |height residual| of the points to their own MLS fit (fold detector), mm.
    pub fold: f64,
}

/// Resamples `points` onto an `nu × nv` grid over their best-fit plane. `None` for fewer than
/// 10 points, a degenerate spread, or a region that folds over its plane.
#[must_use]
pub fn height_grid(points: &[Point3], normals: &[Vec3], nu: usize, nv: usize) -> Option<HeightGrid> {
    if points.len() < 10 {
        return None;
    }
    let orient: Vec3 = normals.iter().filter(|n| surfacecraft_geom::finite_vec(n)).sum();
    let pl = crate::plane::fit_plane(points, if orient.norm() > 0.0 { Some(&orient) } else { None })?;
    let n = pl.normal;
    let u = any_perpendicular(&n);
    let v = n.cross(&u);
    // the normals must all face the plane side: otherwise the patch folds over it
    if normals.len() == points.len() && normals.iter().filter(|m| m.dot(&n) < 0.05).count() * 50 > normals.len() {
        return None;
    }
    let o = pl.point;
    let flat: Vec<Point3> = points.iter().map(|p| Point3::new((p - o).dot(&u), (p - o).dot(&v), 0.0)).collect();
    let heights: Vec<f64> = points.iter().map(|p| (p - o).dot(&n)).collect();
    let (mut u0, mut u1, mut v0, mut v1) = (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for f in &flat {
        u0 = u0.min(f.x);
        u1 = u1.max(f.x);
        v0 = v0.min(f.y);
        v1 = v1.max(f.y);
    }
    if !(u1 - u0 > 1e-9 && v1 - v0 > 1e-9) {
        return None;
    }
    let tree = KdTree::build(&flat);
    let k = 24.min(points.len());
    let mls = |x: f64, y: f64| -> Option<f64> {
        let nb = tree.knn(&Point3::new(x, y, 0.0), k);
        let h2 = nb.last().map_or(1.0, |l| l.1).max(1e-18);
        let mut a = Matrix6::<f64>::zeros();
        let mut b = Vector6::<f64>::zeros();
        for (i, d2) in &nb {
            let (Some(f), Some(h)) = (flat.get(*i), heights.get(*i)) else { continue };
            let w = (-2.0 * d2 / h2).exp();
            let (dx, dy) = (f.x - x, f.y - y);
            let row = Vector6::new(1.0, dx, dy, dx * dx, dx * dy, dy * dy);
            a += row * row.transpose() * w;
            b += row * (h * w);
        }
        // a little ridge keeps sparse corners stable
        for d in 3..6 {
            a[(d, d)] += 1e-9 * (1.0 + a[(d, d)]);
        }
        a.lu().solve(&b).map(|c| c[0]).filter(|z| z.is_finite())
    };
    let mut grid = Vec::with_capacity(nu);
    for i in 0..nu {
        let x = u0 + (u1 - u0) * i as f64 / (nu.max(2) - 1) as f64;
        let mut row = Vec::with_capacity(nv);
        for j in 0..nv {
            let y = v0 + (v1 - v0) * j as f64 / (nv.max(2) - 1) as f64;
            let z = mls(x, y)?;
            let p = o + u * x + v * y + n * z;
            row.push([p.x, p.y, p.z]);
        }
        grid.push(row);
    }
    // fold detector: a folded patch has points far from the height field
    let stride = points.len().div_ceil(500).max(1);
    let mut fold: f64 = 0.0;
    for (f, h) in flat.iter().zip(&heights).step_by(stride) {
        if let Some(z) = mls(f.x, f.y) {
            fold = fold.max((z - h).abs());
        }
    }
    Some(HeightGrid { grid, origin: o, u, v, n, fold })
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::Rng;

    #[test]
    fn resamples_a_wavy_patch() {
        let mut rng = Rng::new(3);
        let f = |x: f64, y: f64| 2.0 * (x / 10.0).sin() * (y / 8.0).cos();
        let pts: Vec<Point3> = (0..3000)
            .map(|_| {
                let (x, y) = (rng.range(0.0, 50.0), rng.range(0.0, 30.0));
                Point3::new(x, y, f(x, y))
            })
            .collect();
        let g = height_grid(&pts, &[], 20, 20).unwrap();
        // interior nodes lie on the surface
        let mut max: f64 = 0.0;
        for r in &g.grid[2..18] {
            for p in &r[2..18] {
                max = max.max((p[2] - f(p[0], p[1])).abs());
            }
        }
        assert!(max < 0.02, "max {max}");
        assert!(height_grid(&pts[..5], &[], 4, 4).is_none());
    }
}
