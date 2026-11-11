//! Scattered region points → a regular grid, the input the kernel's least-squares surface fit
//! takes (design doc §6.3 gap: solidcore fits regular grids only; decision for M3: resample
//! here — mesh-side computation — and let the kernel build the NURBS).
//!
//! The region is parameterised over its best-fit plane (it must be a height field over it,
//! true for one patch of a freeform region). Each grid node gets a height from a weighted
//! quadratic (moving least squares) fit of its nearest points.
use nalgebra::{Matrix6, Vector6};
use surfacecraft_geom::{KdTree, Point3, Vec3, any_perpendicular};

/// The raster cell of the footprint is this many point spacings wide, at most this many cells
/// across, and the spacing is measured on at most this many points.
const FOOTPRINT_CELL_SPACINGS: f64 = 2.0;
const MAX_FOOTPRINT_CELLS: usize = 512;
const SPACING_SAMPLES: usize = 2000;
/// Local point radii over which a node outside the footprint is continued linearly.
const EXTENSION_RADII: f64 = 3.0;

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

/// Cells of the raster over the points' bounding rectangle that lie outside their footprint: the
/// empty cells reachable from the border through empty cells (an empty cell enclosed by occupied
/// ones is a hole of the region, inside the footprint).
#[derive(Clone, Debug)]
struct Footprint {
    x0: f64,
    y0: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    outside: Vec<bool>,
}

impl Footprint {
    fn new(flat: &[Point3], tree: &KdTree, domain: [[f64; 2]; 2]) -> Self {
        let [[x0, x1], [y0, y1]] = domain;
        // cell = a few point spacings (mean distance to the nearest other point of a sample)
        let stride = flat.len().div_ceil(SPACING_SAMPLES).max(1);
        let gaps: Vec<f64> =
            flat.iter().step_by(stride).filter_map(|p| tree.knn(p, 2).get(1).map(|n| n.1.sqrt())).collect();
        let spacing = gaps.iter().sum::<f64>() / gaps.len().max(1) as f64;
        let cell =
            (FOOTPRINT_CELL_SPACINGS * spacing).max((x1 - x0).max(y1 - y0) / MAX_FOOTPRINT_CELLS as f64).max(1e-12);
        let nx = (((x1 - x0) / cell).floor() as usize + 1).min(MAX_FOOTPRINT_CELLS + 1);
        let ny = (((y1 - y0) / cell).floor() as usize + 1).min(MAX_FOOTPRINT_CELLS + 1);
        let mut fp = Self { x0, y0, cell, nx, ny, outside: vec![false; nx * ny] };
        let mut occupied = vec![false; nx * ny];
        for p in flat {
            let k = fp.index(p.x, p.y);
            if let Some(o) = occupied.get_mut(k) {
                *o = true;
            }
        }
        // flood the empty cells from the border
        let mut stack: Vec<usize> = (0..nx * ny)
            .filter(|&k| (k % nx == 0 || k % nx == nx - 1 || k / nx == 0 || k / nx == ny - 1) && !occupied[k])
            .collect();
        for &k in &stack {
            fp.outside[k] = true;
        }
        while let Some(k) = stack.pop() {
            let (i, j) = (k % nx, k / nx);
            let around = [
                (i > 0).then(|| k - 1),
                (i + 1 < nx).then(|| k + 1),
                (j > 0).then(|| k - nx),
                (j + 1 < ny).then(|| k + nx),
            ];
            for n in around.into_iter().flatten() {
                if !occupied[n] && !fp.outside[n] {
                    fp.outside[n] = true;
                    stack.push(n);
                }
            }
        }
        fp
    }

    fn index(&self, x: f64, y: f64) -> usize {
        let i = (((x - self.x0) / self.cell).floor().max(0.0) as usize).min(self.nx - 1);
        let j = (((y - self.y0) / self.cell).floor().max(0.0) as usize).min(self.ny - 1);
        j * self.nx + i
    }

    fn is_outside(&self, x: f64, y: f64) -> bool {
        self.outside.get(self.index(x, y)).copied().unwrap_or(false)
    }
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
    footprint: Footprint,
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
        let tree = KdTree::build(&flat);
        let footprint = Footprint::new(&flat, &tree, [[u0, u1], [v0, v1]]);
        Some(Self { origin, u, v, n, domain: [[u0, u1], [v0, v1]], tree, flat, heights, footprint })
    }

    /// Height above the plane at plane coordinates `(x, y)` (weighted quadratic of the 24 nearest).
    #[must_use]
    pub fn height(&self, x: f64, y: f64) -> Option<f64> {
        self.local_fit(x, y).map(|c| c[0])
    }

    /// Like [`Self::height`] for a node inside the footprint of the points (holes of the region
    /// are inside it), but a node outside it (a corner of the bounding rectangle of a round or
    /// L-shaped region) is continued linearly from the nearest point, over at most
    /// [`EXTENSION_RADII`] local point radii and flat beyond: the quadratic swings wildly away
    /// from the data and would pull the least-squares surface off the region. The footprint is
    /// the set of cells of a coarse raster that the points occupy plus the empty cells enclosed
    /// by them.
    #[must_use]
    pub fn height_extended(&self, x: f64, y: f64) -> Option<f64> {
        if !self.footprint.is_outside(x, y) {
            return self.height(x, y);
        }
        let k = 24.min(self.flat.len());
        let nearest = self.tree.knn(&Point3::new(x, y, 0.0), 1).first().map(|n| n.0)?;
        let a = self.flat.get(nearest)?;
        let c = self.local_fit(a.x, a.y)?;
        let radius = self.tree.knn(&Point3::new(a.x, a.y, 0.0), k).last().map_or(0.0, |l| l.1.sqrt());
        let (dx, dy) = (x - a.x, y - a.y);
        let reach = dx.hypot(dy);
        let used = if reach > EXTENSION_RADII * radius && reach > 0.0 { EXTENSION_RADII * radius / reach } else { 1.0 };
        Some(c[0] + used * (c[1] * dx + c[2] * dy))
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

    /// Points of the disc `|p| <= 10` of `f` on a jittered lattice of 0.2 mm, thinned towards -x
    /// (a 2:1 density gradient), with a round hole of radius `hole` at `(3, 0)` when `hole > 0`.
    fn disc(f: impl Fn(f64, f64) -> f64, hole: f64, seed: u64) -> Vec<Point3> {
        let mut rng = Rng::new(seed);
        let mut pts = Vec::new();
        for i in 0..100 {
            for j in 0..100 {
                let (x, y) = (
                    -10.0 + 0.2 * f64::from(i) + 0.15 * rng.uniform(),
                    -10.0 + 0.2 * f64::from(j) + 0.15 * rng.uniform(),
                );
                let share = 0.5 + 0.05 * (x + 10.0);
                let thinned = f64::from((i * 7 + j * 13) % 10) >= 10.0 * share;
                let in_hole = hole > 0.0 && (x - 3.0).hypot(y) <= hole;
                if x.hypot(y) <= 10.0 && !in_hole && !thinned {
                    pts.push(Point3::new(x, y, f(x, y)));
                }
            }
        }
        pts
    }

    /// 3-D point of the node `(x, y)` with height `z`, as the grid builds it.
    fn lift(field: &HeightField, x: f64, y: f64, z: f64) -> Point3 {
        field.origin + field.u * x + field.v * y + field.n * z
    }

    /// The corners of the bounding rectangle of a round footprint are continued from the nearest
    /// data: within 0.4 mm of the smooth surface they continue, where the plain quadratic of the
    /// 24 nearest points is off by more than 1 mm.
    #[test]
    fn corners_outside_the_footprint_stay_near_the_data() {
        let f = |x: f64, y: f64| 2.0 * (x / 4.0).sin() * (y / 4.0).cos();
        let field = HeightField::new(&disc(f, 0.0, 9), &[]).unwrap();
        let [[x0, x1], [y0, y1]] = field.domain;
        let (mut ext, mut plain): (f64, f64) = (0.0, 0.0);
        for (x, y) in [(x0, y0), (x1, y0), (x0, y1), (x1, y1)] {
            let (pe, pq) = (
                lift(&field, x, y, field.height_extended(x, y).unwrap()),
                lift(&field, x, y, field.height(x, y).unwrap()),
            );
            ext = ext.max((pe.z - f(pe.x, pe.y)).abs());
            plain = plain.max((pq.z - f(pq.x, pq.y)).abs());
        }
        eprintln!("corner nodes: extended error {ext:.3} mm, quadratic error {plain:.3} mm");
        assert!(ext < 0.4, "extended corner nodes off by {ext}");
        assert!(plain > 1.0, "the old quadratic extrapolation was off by only {plain}");
    }

    /// Inside the footprint nothing changed, bit for bit: a disc with an interior hole and a 2:1
    /// density gradient, every node whose position lies in the disc (hole nodes included).
    #[test]
    fn nodes_inside_the_footprint_are_identical_to_the_plain_fit() {
        let f = |x: f64, y: f64| 2.0 * (x / 4.0).sin() * (y / 4.0).cos();
        let field = HeightField::new(&disc(f, 3.0, 4), &[]).unwrap();
        let (mut inside, mut hole_nodes) = (0, 0);
        for i in 0..=60 {
            for j in 0..=60 {
                let (wx, wy) = (-9.0 + 18.0 * f64::from(i) / 60.0, -9.0 + 18.0 * f64::from(j) / 60.0);
                if wx.hypot(wy) > 9.0 {
                    continue;
                }
                inside += 1;
                hole_nodes += usize::from((wx - 3.0).hypot(wy) < 3.0);
                // the node in the plane frame of the field
                let d = Point3::new(wx, wy, f(wx, wy)) - field.origin;
                let (x, y) = (d.dot(&field.u), d.dot(&field.v));
                let (a, b) = (field.height_extended(x, y).unwrap(), field.height(x, y).unwrap());
                assert_eq!(a.to_bits(), b.to_bits(), "node ({x:.2}, {y:.2}): {a} vs {b}");
            }
        }
        assert!(inside > 2000 && hole_nodes > 200, "{inside} nodes, {hole_nodes} in the hole");
    }

    /// The linear continuation stops growing at a few point radii, so a node far away is not
    /// thrown off by an ever longer extrapolation of the gradient.
    #[test]
    fn the_extension_is_bounded() {
        let f = |x: f64, _y: f64| 0.8 * x;
        let field = HeightField::new(&disc(f, 0.0, 2), &[]).unwrap();
        let near = field.height_extended(14.0, 0.0).unwrap();
        let far = field.height_extended(500.0, 0.0).unwrap();
        assert!((far - near).abs() < 8.0, "{near} -> {far}: the gradient was extrapolated 500 mm");
    }
}
