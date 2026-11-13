//! A uniform grid over the points, for the neighbour searches of the ball pivoting.

use super::*;

// ------------------------------------------------------------------ uniform grid

/// Static uniform grid over the points (counting-sorted, dense).
pub(super) struct Grid {
    pub(super) min: Point3,
    pub(super) cell: f64,
    pub(super) dims: [usize; 3],
    pub(super) start: Vec<u32>,
    pub(super) items: Vec<u32>,
}

impl Grid {
    pub(super) fn build(pts: &[Point3], cell: f64) -> Self {
        let bb = Aabb::from_points(pts);
        let (min, ext) = if bb.is_empty() { (Point3::origin(), Vec3::zeros()) } else { (bb.min, bb.max - bb.min) };
        let mut cell = if cell.is_finite() && cell > 0.0 { cell } else { 1.0 };
        let cap = (pts.len() * 2).max(1 << 20) as f64;
        let dims = loop {
            let d = [0, 1, 2].map(|a| ((ext[a] / cell).floor() as usize).saturating_add(1));
            if (d[0] as f64) * (d[1] as f64) * (d[2] as f64) <= cap {
                break d;
            }
            cell *= 1.5;
        };
        let total = dims[0] * dims[1] * dims[2];
        let mut g = Grid { min, cell, dims, start: vec![0; total + 1], items: vec![0; pts.len()] };
        let ids: Vec<usize> = pts.iter().map(|p| g.cell_index(p)).collect();
        for &c in &ids {
            if let Some(s) = g.start.get_mut(c + 1) {
                *s += 1;
            }
        }
        for c in 0..total {
            g.start[c + 1] += g.start[c];
        }
        let mut fill: Vec<u32> = g.start.clone();
        for (i, &c) in ids.iter().enumerate() {
            if let Some(f) = fill.get_mut(c) {
                if let Some(slot) = g.items.get_mut(*f as usize) {
                    *slot = i as u32;
                }
                *f += 1;
            }
        }
        g
    }

    fn axis_cell(&self, v: f64, a: usize) -> usize {
        let last = self.dims[a] - 1;
        let c = ((v - self.min[a]) / self.cell).floor();
        if c <= 0.0 { 0 } else { (c as usize).min(last) }
    }

    fn cell_index(&self, p: &Point3) -> usize {
        let (x, y, z) = (self.axis_cell(p.x, 0), self.axis_cell(p.y, 1), self.axis_cell(p.z, 2));
        (z * self.dims[1] + y) * self.dims[0] + x
    }

    /// Calls `f` on every point in the cells overlapping the ball `(c, r)` (a superset of the
    /// points inside it); `f` returns true to stop.
    pub(super) fn visit(&self, c: &Point3, r: f64, mut f: impl FnMut(u32) -> bool) {
        let lo = [0, 1, 2].map(|a| self.axis_cell(c[a] - r, a));
        let hi = [0, 1, 2].map(|a| self.axis_cell(c[a] + r, a));
        for z in lo[2]..=hi[2] {
            for y in lo[1]..=hi[1] {
                let row = (z * self.dims[1] + y) * self.dims[0];
                let (Some(&s), Some(&e)) = (self.start.get(row + lo[0]), self.start.get(row + hi[0] + 1)) else {
                    continue;
                };
                for &i in self.items.get(s as usize..e as usize).unwrap_or(&[]) {
                    if f(i) {
                        return;
                    }
                }
            }
        }
    }
}
