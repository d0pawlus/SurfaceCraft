//! Cutting a scan into chunks without holding it: a histogram of triangle centroids on a coarse
//! grid, then a kd-style split of the grid into boxes of at most one chunk of triangles each
//! (pure, no file access). The histogram is at most [`MAX_GRID_CELLS`] counters, so the plan costs
//! a fixed few megabytes whatever the size of the scan.
use super::StoreError;
use crate::{Aabb, Point3};
use std::collections::HashMap;

/// Most cells of the partition grid (4 bytes of histogram and 4 bytes of leaf id each).
pub const MAX_GRID_CELLS: usize = 1 << 21;
/// Cells along the longest axis before the grid is coarsened to fit [`MAX_GRID_CELLS`].
const GRID_AXIS_CELLS: f64 = 512.0;
/// Growth of the cell size while the grid is too big.
const GRID_COARSEN: f64 = 1.26;

/// A uniform grid of cubic cells over a bounding box.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CellGrid {
    /// Corner of cell `[0, 0, 0]`.
    pub origin: Point3,
    /// Cell edge, mm.
    pub h: f64,
    /// Cells along each axis.
    pub dims: [usize; 3],
}

impl CellGrid {
    /// A grid over `bbox` with at most [`MAX_GRID_CELLS`] cells.
    pub fn new(bbox: &Aabb) -> Result<Self, StoreError> {
        let ext = bbox.size();
        let longest = ext.x.max(ext.y).max(ext.z);
        if bbox.is_empty() || !longest.is_finite() {
            return Err(StoreError::Input("the bounding box of the input is empty or not finite".into()));
        }
        let mut h = if longest > 0.0 { longest / GRID_AXIS_CELLS } else { 1.0 };
        loop {
            let dims = [0, 1, 2].map(|i| ((ext[i] / h).ceil() as usize).max(1));
            let cells = dims[0].saturating_mul(dims[1]).saturating_mul(dims[2]);
            if cells <= MAX_GRID_CELLS {
                return Ok(Self { origin: bbox.min, h, dims });
            }
            h *= GRID_COARSEN;
        }
    }

    /// Number of cells.
    #[must_use]
    pub fn cell_count(&self) -> usize {
        self.dims[0] * self.dims[1] * self.dims[2]
    }

    /// The cell holding `p` (points outside, or NaN, are clamped into the grid).
    #[must_use]
    pub fn cell_of(&self, p: &Point3) -> usize {
        let c = [0, 1, 2].map(|i| {
            let k = ((p[i] - self.origin[i]) / self.h).floor();
            (k.max(0.0) as usize).min(self.dims[i] - 1)
        });
        (c[2] * self.dims[1] + c[1]) * self.dims[0] + c[0]
    }
}

/// A leaf of the partition: a box of grid cells whose triangles form one chunk.
#[derive(Clone, Debug, PartialEq)]
pub struct Partition {
    /// The grid.
    pub grid: CellGrid,
    /// Leaf id of every cell (`u32::MAX` for empty cells and for cells that are cut by input order).
    pub cell_leaf: Vec<u32>,
    /// Triangles of every leaf, in leaf order.
    pub leaf_counts: Vec<u32>,
    /// A single cell with more triangles than a chunk holds is cut by input order into consecutive
    /// leaves: cell → first leaf.
    pub heavy: HashMap<u32, u32>,
    /// Chunk capacity.
    pub capacity: u32,
}

struct Region {
    lo: [usize; 3],
    hi: [usize; 3],
    total: u64,
}

impl Partition {
    /// Splits the grid by the histogram `counts` (one per cell) into leaves of at most `capacity`
    /// triangles. Leaves are numbered in depth-first order, lower half first.
    #[must_use]
    pub fn plan(grid: CellGrid, counts: &[u32], capacity: u32) -> Self {
        let mut p = Partition {
            grid,
            cell_leaf: vec![u32::MAX; counts.len()],
            leaf_counts: Vec::new(),
            heavy: HashMap::new(),
            capacity,
        };
        let total: u64 = counts.iter().map(|&c| u64::from(c)).sum();
        let mut stack = vec![Region { lo: [0; 3], hi: grid.dims, total }];
        while let Some(r) = stack.pop() {
            if r.total == 0 {
                continue;
            }
            let ext = [0, 1, 2].map(|i| r.hi[i] - r.lo[i]);
            if r.total <= u64::from(capacity) || ext == [1, 1, 1] {
                p.emit(&r, counts);
            } else if let Some((lower, upper)) = split(&r, ext, counts, &grid) {
                stack.push(upper);
                stack.push(lower);
            } else {
                p.emit(&r, counts);
            }
        }
        p
    }

    fn emit(&mut self, r: &Region, counts: &[u32]) {
        let id = self.leaf_counts.len() as u32;
        let single = (0..3).all(|i| r.hi[i] - r.lo[i] == 1);
        if single && r.total > u64::from(self.capacity) {
            let cell = cell_index(&self.grid, r.lo);
            self.heavy.insert(cell as u32, id);
            let cap = u64::from(self.capacity);
            let mut left = r.total;
            while left > 0 {
                self.leaf_counts.push(left.min(cap) as u32);
                left -= left.min(cap);
            }
            return;
        }
        self.leaf_counts.push(r.total as u32);
        for_cells(&self.grid, r, |cell| {
            if counts.get(cell).copied().unwrap_or(0) > 0
                && let Some(slot) = self.cell_leaf.get_mut(cell)
            {
                *slot = id;
            }
        });
    }

    /// The leaf that receives the next triangle whose centroid is in `cell`; `seen` counts, per
    /// heavy cell, the triangles routed so far.
    pub fn leaf_for(&self, cell: usize, seen: &mut HashMap<u32, u64>) -> Option<u32> {
        if let Some(&first) = self.heavy.get(&(cell as u32)) {
            let n = seen.entry(cell as u32).or_insert(0);
            let leaf = first.checked_add((*n / u64::from(self.capacity)) as u32)?;
            *n += 1;
            return Some(leaf);
        }
        self.cell_leaf.get(cell).copied().filter(|&l| l != u32::MAX)
    }
}

fn cell_index(g: &CellGrid, c: [usize; 3]) -> usize {
    (c[2] * g.dims[1] + c[1]) * g.dims[0] + c[0]
}

fn for_cells(g: &CellGrid, r: &Region, mut f: impl FnMut(usize)) {
    for z in r.lo[2]..r.hi[2] {
        for y in r.lo[1]..r.hi[1] {
            for x in r.lo[0]..r.hi[0] {
                f(cell_index(g, [x, y, z]));
            }
        }
    }
}

/// Cuts a region across its longest axis at the plane that balances the two halves best.
fn split(r: &Region, ext: [usize; 3], counts: &[u32], g: &CellGrid) -> Option<(Region, Region)> {
    let axis = (0..3).max_by_key(|&i| ext[i])?;
    if ext[axis] < 2 {
        return None;
    }
    let mut slab = vec![0u64; ext[axis]];
    for z in r.lo[2]..r.hi[2] {
        for y in r.lo[1]..r.hi[1] {
            for x in r.lo[0]..r.hi[0] {
                let k = [x, y, z][axis] - r.lo[axis];
                let c = u64::from(counts.get(cell_index(g, [x, y, z])).copied().unwrap_or(0));
                if let Some(s) = slab.get_mut(k) {
                    *s += c;
                }
            }
        }
    }
    let mut best = (1usize, u64::MAX, 0u64);
    let mut prefix = 0u64;
    for (s, &c) in slab.iter().enumerate().take(ext[axis] - 1) {
        prefix += c;
        let imbalance = (2 * prefix).abs_diff(r.total);
        if imbalance < best.1 {
            best = (s + 1, imbalance, prefix);
        }
    }
    let (cut, _, lower_total) = best;
    let mut lower = Region { lo: r.lo, hi: r.hi, total: lower_total };
    let mut upper = Region { lo: r.lo, hi: r.hi, total: r.total - lower_total };
    lower.hi[axis] = r.lo[axis] + cut;
    upper.lo[axis] = r.lo[axis] + cut;
    Some((lower, upper))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Vec3;

    fn grid(n: usize) -> CellGrid {
        CellGrid { origin: Point3::origin(), h: 1.0, dims: [n, n, 1] }
    }

    #[test]
    fn leaves_hold_at_most_capacity_and_cover_every_triangle() {
        let g = grid(16);
        let counts: Vec<u32> = (0..g.cell_count()).map(|i| ((i * 7919) % 23) as u32 * 5).collect();
        let total: u64 = counts.iter().map(|&c| u64::from(c)).sum();
        let p = Partition::plan(g, &counts, 200);
        assert!(p.leaf_counts.iter().all(|&c| c <= 200 && c > 0), "{:?}", p.leaf_counts);
        assert_eq!(p.leaf_counts.iter().map(|&c| u64::from(c)).sum::<u64>(), total);
        // every non-empty cell routes to a leaf
        let mut seen = HashMap::new();
        for (cell, &c) in counts.iter().enumerate() {
            assert_eq!(p.leaf_for(cell, &mut seen).is_some(), c > 0, "cell {cell}");
        }
        // balanced halves: no chunk is a sliver of the capacity unless the data forces it
        assert!(p.leaf_counts.len() as u64 <= 3 * total / 200 + 3);
    }

    #[test]
    fn a_cell_over_capacity_is_cut_by_input_order() {
        let g = grid(2);
        let counts = vec![0, 1000, 3, 0];
        let p = Partition::plan(g, &counts, 300);
        assert_eq!(p.heavy.len(), 1);
        let mut seen = HashMap::new();
        let mut got = vec![0u32; p.leaf_counts.len()];
        for _ in 0..1000 {
            got[p.leaf_for(1, &mut seen).unwrap() as usize] += 1;
        }
        got[p.leaf_for(2, &mut seen).unwrap() as usize] += 1;
        got[p.leaf_for(2, &mut seen).unwrap() as usize] += 1;
        got[p.leaf_for(2, &mut seen).unwrap() as usize] += 1;
        assert_eq!(got, p.leaf_counts);
        assert!(p.leaf_counts.iter().all(|&c| c <= 300));
    }

    #[test]
    fn the_grid_fits_its_budget_and_clamps_points() {
        let b = Aabb::from_points(&[Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 90.0, 80.0)]);
        let g = CellGrid::new(&b).unwrap();
        assert!(g.cell_count() <= MAX_GRID_CELLS && g.dims.iter().all(|&d| d >= 1));
        assert_eq!(g.cell_of(&Point3::new(-5.0, f64::NAN, 1e9)), cell_index(&g, [0, 0, g.dims[2] - 1]));
        let flat = Aabb::from_points(&[Point3::origin(), Point3::from(Vec3::new(10.0, 10.0, 0.0))]);
        assert_eq!(CellGrid::new(&flat).unwrap().dims[2], 1);
        assert!(CellGrid::new(&Aabb::empty()).is_err());
        let point = Aabb::from_points(&[Point3::new(1.0, 1.0, 1.0)]);
        assert_eq!(CellGrid::new(&point).unwrap().dims, [1, 1, 1]);
    }
}
