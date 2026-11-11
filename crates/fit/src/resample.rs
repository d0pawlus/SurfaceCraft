//! Scattered region points → a regular grid, the input the kernel's least-squares surface fit
//! takes (design doc §6.3 gap: solidcore fits regular grids only; decision for M3: resample
//! here — mesh-side computation — and let the kernel build the NURBS).
//!
//! The region is parameterised over its best-fit plane (it must be a height field over it,
//! true for one patch of a freeform region). Each grid node gets a height from a weighted
//! quadratic (moving least squares) fit of its nearest points.
use nalgebra::{Matrix6, Vector6};
use surfacecraft_geom::{KdTree, Point3, Vec3, any_perpendicular};

/// A node whose nearest point is more than this fraction of its 24th-nearest distance is outside
/// the footprint of the points (inside, the nearest point is a small part of that radius).
const OUTSIDE_RATIO: f64 = 0.6;

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
    let field = HeightField::new(points, normals)?;
    let (o, u, v, n) = (field.origin, field.u, field.v, field.n);
    let [[u0, u1], [v0, v1]] = field.domain;
    let mls = |x: f64, y: f64| field.height_extended(x, y);
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
    for (f, h) in field.flat.iter().zip(&field.heights).step_by(stride) {
        if let Some(z) = mls(f.x, f.y) {
            fold = fold.max((z - h).abs());
        }
    }
    Some(HeightGrid { grid, origin: o, u, v, n, fold })
}

/// A region seen as a height field over its best-fit plane, with moving-least-squares heights.
#[derive(Clone, Debug)]
pub struct HeightField {
    /// Plane origin.
    pub origin: Point3,
    /// Plane u axis.
    pub u: Vec3,
    /// Plane v axis.
    pub v: Vec3,
    /// Plane normal.
    pub n: Vec3,
    /// `[[u0, u1], [v0, v1]]` covered by the points.
    pub domain: [[f64; 2]; 2],
    tree: KdTree,
    flat: Vec<Point3>,
    heights: Vec<f64>,
}

impl HeightField {
    /// The field of `points` (`None` for fewer than 10, a degenerate spread, or a fold).
    #[must_use]
    pub fn new(points: &[Point3], normals: &[Vec3]) -> Option<Self> {
        if points.len() < 10 {
            return None;
        }
        let orient: Vec3 = normals.iter().filter(|n| surfacecraft_geom::finite_vec(n)).sum();
        let pl = crate::plane::fit_plane(points, if orient.norm() > 0.0 { Some(&orient) } else { None })?;
        let n = pl.normal;
        // the normals must all face the plane side: otherwise the patch folds over it
        if normals.len() == points.len() && normals.iter().filter(|m| m.dot(&n) < 0.05).count() * 50 > normals.len() {
            return None;
        }
        let u = any_perpendicular(&n);
        let v = n.cross(&u);
        Self::in_frame(points, pl.point, u, v, n)
    }

    /// The field of `points` over a given plane frame.
    #[must_use]
    pub fn in_frame(points: &[Point3], origin: Point3, u: Vec3, v: Vec3, n: Vec3) -> Option<Self> {
        if points.len() < 10 {
            return None;
        }
        let flat: Vec<Point3> =
            points.iter().map(|p| Point3::new((p - origin).dot(&u), (p - origin).dot(&v), 0.0)).collect();
        let heights: Vec<f64> = points.iter().map(|p| (p - origin).dot(&n)).collect();
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
        Some(Self { origin, u, v, n, domain: [[u0, u1], [v0, v1]], tree: KdTree::build(&flat), flat, heights })
    }

    /// Height above the plane at plane coordinates `(x, y)` (weighted quadratic of the 24 nearest).
    #[must_use]
    pub fn height(&self, x: f64, y: f64) -> Option<f64> {
        self.local_fit(x, y).map(|c| c[0])
    }

    /// Like [`Self::height`] inside the footprint of the points, but a node far outside it (a
    /// corner of the bounding rectangle of a round or L-shaped region) is continued linearly from
    /// the nearest point instead of by the quadratic, which swings wildly away from the data and
    /// would pull the least-squares surface off the region.
    #[must_use]
    pub fn height_extended(&self, x: f64, y: f64) -> Option<f64> {
        let q = Point3::new(x, y, 0.0);
        let near = self.tree.knn(&q, 1);
        let far = self.tree.knn(&q, 24.min(self.flat.len()));
        let (Some(&(i, d1)), Some(&(_, d24))) = (near.first(), far.last()) else { return self.height(x, y) };
        if d1 <= OUTSIDE_RATIO * OUTSIDE_RATIO * d24 {
            return self.height(x, y);
        }
        let a = self.flat.get(i)?;
        let c = self.local_fit(a.x, a.y)?;
        Some(c[0] + c[1] * (x - a.x) + c[2] * (y - a.y))
    }

    /// Coefficients `[h, ∂h/∂x, ∂h/∂y, …]` of the weighted quadratic of the 24 nearest points,
    /// centred at `(x, y)`.
    fn local_fit(&self, x: f64, y: f64) -> Option<Vector6<f64>> {
        let k = 24.min(self.flat.len());
        let nb = self.tree.knn(&Point3::new(x, y, 0.0), k);
        let h2 = nb.last().map_or(1.0, |l| l.1).max(1e-18);
        let mut a = Matrix6::<f64>::zeros();
        let mut b = Vector6::<f64>::zeros();
        for (i, d2) in &nb {
            let (Some(f), Some(h)) = (self.flat.get(*i), self.heights.get(*i)) else { continue };
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
        a.lu().solve(&b).filter(|c| c.iter().all(|v| v.is_finite()))
    }

    /// The surface point at plane coordinates `(x, y)`.
    #[must_use]
    pub fn point(&self, x: f64, y: f64) -> Option<Point3> {
        self.height(x, y).map(|z| self.origin + self.u * x + self.v * y + self.n * z)
    }

    /// An `nu × nv` grid of surface points over `[[x0, x1], [y0, y1]]`.
    #[must_use]
    pub fn grid(&self, d: [[f64; 2]; 2], nu: usize, nv: usize) -> Option<Vec<Vec<[f64; 3]>>> {
        let mut g = Vec::with_capacity(nu);
        for i in 0..nu {
            let x = d[0][0] + (d[0][1] - d[0][0]) * i as f64 / (nu.max(2) - 1) as f64;
            let mut row = Vec::with_capacity(nv);
            for j in 0..nv {
                let y = d[1][0] + (d[1][1] - d[1][0]) * j as f64 / (nv.max(2) - 1) as f64;
                let p = self.point(x, y)?;
                row.push([p.x, p.y, p.z]);
            }
            g.push(row);
        }
        Some(g)
    }
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

    /// A round footprint: the corners of its bounding rectangle are continued from the nearest
    /// data, not extrapolated by the quadratic.
    #[test]
    fn corners_outside_the_footprint_stay_near_the_data() {
        let mut rng = Rng::new(9);
        let f = |x: f64, y: f64| 2.0 * (x / 4.0).sin() * (y / 4.0).cos();
        let pts: Vec<Point3> = (0..4000)
            .filter_map(|_| {
                let (x, y) = (rng.range(-10.0, 10.0), rng.range(-10.0, 10.0));
                (x.hypot(y) <= 10.0).then(|| Point3::new(x, y, f(x, y)))
            })
            .collect();
        let field = HeightField::new(&pts, &[]).unwrap();
        let [[x0, x1], [y0, y1]] = field.domain;
        let mut worst: f64 = 0.0;
        for (x, y) in [(x0, y0), (x1, y0), (x0, y1), (x1, y1)] {
            // plane frame: heights are along `n`; compare the nodes' 3-D points with the analytic surface
            let p = field.origin + field.u * x + field.v * y + field.n * field.height_extended(x, y).unwrap();
            worst = worst.max((p.z - f(p.x, p.y)).abs());
        }
        assert!(worst < 1.5, "corner nodes off the surface by {worst}");
        // inside the footprint nothing changes
        assert_eq!(field.height_extended(0.0, 0.0), field.height(0.0, 0.0));
    }
}
