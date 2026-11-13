//! The solid of a scan as a sparse voxel set: for every (x, y) column of the grid, the runs of
//! consecutive inside voxels along z. A solid occupies few runs however fine the grid is (a
//! ball of 100 mm at 0.3 mm voxels has 335 000 columns with one run each, where a dense grid has 37
//! million voxels), and morphology on the runs is separable: erosion and dilation by a cube are a
//! shrink or grow of every run along z and an intersection or union of the runs of the neighbouring
//! columns along x and y.
use crate::par;

/// Half-open range of voxels `[start, end)` along z.
pub type Run = [u32; 2];

/// Sparse voxel set in compressed-row form: `runs[start[c] .. start[c + 1]]` are the runs of
/// column `c = j · nx + i`, ascending and separated by at least one outside voxel.
#[derive(Clone, Debug, Default)]
pub struct Occupancy {
    /// Columns along x.
    pub nx: usize,
    /// Columns along y.
    pub ny: usize,
    /// Voxels along z (runs never reach beyond).
    pub nz: u32,
    start: Vec<u32>,
    runs: Vec<Run>,
}

/// Merges ascending-by-start runs that overlap or touch.
fn merge_sorted(list: &mut Vec<Run>) {
    let mut out: Vec<Run> = Vec::with_capacity(list.len());
    for r in list.drain(..) {
        match out.last_mut() {
            Some(last) if r[0] <= last[1] => last[1] = last[1].max(r[1]),
            _ => out.push(r),
        }
    }
    *list = out;
}

/// `a ∩ b` of two run lists, appended to `out`.
fn intersect(a: &[Run], b: &[Run], out: &mut Vec<Run>) {
    let (mut i, mut j) = (0, 0);
    while let (Some(x), Some(y)) = (a.get(i), b.get(j)) {
        let (lo, hi) = (x[0].max(y[0]), x[1].min(y[1]));
        if lo < hi {
            out.push([lo, hi]);
        }
        if x[1] < y[1] {
            i += 1;
        } else {
            j += 1;
        }
    }
}

/// `a ∖ b` of two run lists, appended to `out`.
fn subtract(a: &[Run], b: &[Run], out: &mut Vec<Run>) {
    for r in a {
        let mut lo = r[0];
        for s in b.iter().filter(|s| s[1] > r[0] && s[0] < r[1]) {
            if s[0] > lo {
                out.push([lo, s[0]]);
            }
            lo = lo.max(s[1]);
        }
        if lo < r[1] {
            out.push([lo, r[1]]);
        }
    }
}

/// Column data in flat buffers: the columns that have runs, the end of each one's runs, the runs.
#[derive(Clone, Debug, Default)]
pub struct Flat {
    cols: Vec<u32>,
    ends: Vec<u32>,
    runs: Vec<Run>,
}

impl Flat {
    /// Adds column `col` (after every earlier one): `list` is sorted, merged and clipped to `nz`.
    pub fn push(&mut self, col: u32, list: &mut Vec<Run>, nz: u32) {
        list.iter_mut().for_each(|r| r[1] = r[1].min(nz));
        list.retain(|r| r[0] < r[1]);
        list.sort_unstable();
        merge_sorted(list);
        if list.is_empty() {
            return;
        }
        self.runs.extend_from_slice(list);
        self.cols.push(col);
        self.ends.push(u32::try_from(self.runs.len()).unwrap_or(u32::MAX));
    }

    fn append(&mut self, other: Flat) {
        let offset = u32::try_from(self.runs.len()).unwrap_or(u32::MAX);
        self.cols.extend(other.cols);
        self.ends.extend(other.ends.into_iter().map(|e| e.saturating_add(offset)));
        self.runs.extend(other.runs);
    }

    /// Bytes held.
    #[must_use]
    pub fn bytes(&self) -> usize {
        self.cols.len() * 8 + self.runs.len() * 8
    }

    /// Runs held.
    #[must_use]
    pub fn run_count(&self) -> usize {
        self.runs.len()
    }
}

impl Occupancy {
    /// Builds the set from the runs of every column, given as `(column, runs)` ascending by column;
    /// runs are merged and clipped to `nz`.
    #[must_use]
    pub fn from_columns(nx: usize, ny: usize, nz: u32, columns: Vec<(u32, Vec<Run>)>) -> Self {
        let mut flat = Flat::default();
        for (c, mut list) in columns {
            flat.push(c, &mut list, nz);
        }
        Self::from_flat(nx, ny, nz, flat)
    }

    /// Builds the set from flat column data (ascending columns).
    #[must_use]
    pub fn from_flat(nx: usize, ny: usize, nz: u32, flat: Flat) -> Self {
        let n = nx.saturating_mul(ny);
        let mut start = vec![0u32; n + 1];
        let mut prev = 0u32;
        for (c, end) in flat.cols.iter().zip(&flat.ends) {
            if let Some(s) = start.get_mut(*c as usize + 1) {
                *s = end.saturating_sub(prev);
            }
            prev = *end;
        }
        for k in 0..n {
            start[k + 1] = start[k + 1].saturating_add(start[k]);
        }
        Self { nx, ny, nz, start, runs: flat.runs }
    }

    /// The runs of column `(i, j)` (none outside the grid).
    #[must_use]
    pub fn column(&self, i: usize, j: usize) -> &[Run] {
        if i >= self.nx || j >= self.ny {
            return &[];
        }
        let c = j * self.nx + i;
        match (self.start.get(c), self.start.get(c + 1)) {
            (Some(&a), Some(&b)) => self.runs.get(a as usize..b as usize).unwrap_or(&[]),
            _ => &[],
        }
    }

    /// Is voxel `(i, j, k)` inside?
    #[must_use]
    pub fn contains(&self, i: usize, j: usize, k: u32) -> bool {
        let col = self.column(i, j);
        let n = col.partition_point(|r| r[1] <= k);
        col.get(n).is_some_and(|r| r[0] <= k)
    }

    /// Number of runs.
    #[must_use]
    pub fn run_count(&self) -> usize {
        self.runs.len()
    }

    /// Number of inside voxels.
    #[must_use]
    pub fn voxel_count(&self) -> u64 {
        self.runs.iter().map(|r| u64::from(r[1] - r[0])).sum()
    }

    /// Bytes held (the index of all columns plus the runs).
    #[must_use]
    pub fn bytes(&self) -> usize {
        self.start.len() * 4 + self.runs.len() * 8
    }

    /// A new set whose column `(i, j)` is what `f` writes for it, given this set. Rows run in
    /// parallel; the result does not depend on the thread count.
    ///
    /// `bound` is an upper bound of the runs of the result, known before anything is built (a grow
    /// along x or y writes at most `2k + 1` times the runs it reads, along z at most as many): when it
    /// passes `budget` nothing is built and the bound is the error, the same for any thread count.
    fn transform<F>(&self, budget: usize, bound: usize, f: F) -> Result<Self, usize>
    where
        F: Fn(&Occupancy, usize, usize, &mut Vec<Run>) + Sync + Send,
    {
        if bound > budget {
            return Err(bound);
        }
        let rows: Vec<Flat> = par::map(self.ny, |j| {
            let mut row = Flat::default();
            let mut list = Vec::new();
            for i in 0..self.nx {
                list.clear();
                f(self, i, j, &mut list);
                row.push((j * self.nx + i) as u32, &mut list, self.nz);
            }
            row
        });
        let mut all = Flat::default();
        for row in rows {
            all.append(row);
        }
        if all.runs.len() > budget {
            return Err(all.runs.len());
        }
        Ok(Self::from_flat(self.nx, self.ny, self.nz, all))
    }

    /// Erosion (`grow = false`) or dilation (`grow = true`) by a cube of half-size `k` along z.
    fn along_z(&self, k: u32, grow: bool, budget: usize) -> Result<Self, usize> {
        self.transform(budget, self.runs.len(), |o, i, j, out| {
            for r in o.column(i, j) {
                if grow {
                    out.push([r[0].saturating_sub(k), r[1].saturating_add(k)]);
                } else if r[1] - r[0] > k.saturating_mul(2) {
                    out.push([r[0].saturating_add(k), r[1] - k]);
                }
            }
        })
    }

    /// Erosion or dilation along x (`axis = 0`) or y (`axis = 1`).
    fn along_xy(&self, k: usize, axis: usize, grow: bool, budget: usize) -> Result<Self, usize> {
        let bound = self.runs.len().saturating_mul(k.saturating_mul(2).saturating_add(1));
        self.transform(budget, bound, |o, i, j, out| {
            let at = |d: isize| -> Option<(usize, usize)> {
                let (a, b) = if axis == 0 { (i as isize + d, j as isize) } else { (i as isize, j as isize + d) };
                (a >= 0 && b >= 0).then_some((a as usize, b as usize))
            };
            if grow {
                for d in -(k as isize)..=(k as isize) {
                    if let Some((a, b)) = at(d) {
                        out.extend_from_slice(o.column(a, b));
                    }
                }
                out.sort_unstable();
                merge_sorted(out);
                return;
            }
            let Some((a, b)) = at(-(k as isize)) else { return };
            let mut acc: Vec<Run> = o.column(a, b).to_vec();
            let mut tmp = Vec::new();
            for d in -(k as isize) + 1..=(k as isize) {
                let Some((a, b)) = at(d) else { return };
                tmp.clear();
                intersect(&acc, o.column(a, b), &mut tmp);
                std::mem::swap(&mut acc, &mut tmp);
            }
            out.extend(acc);
        })
    }

    /// Erosion by a cube of half-size `k` voxels (separable: z, x, y). `Err(runs)` when a step would
    /// hold more than `budget` runs.
    pub fn erode(&self, k: u32, budget: usize) -> Result<Self, usize> {
        if k == 0 {
            return Ok(self.clone());
        }
        self.along_z(k, false, budget)?.along_xy(k as usize, 0, false, budget)?.along_xy(k as usize, 1, false, budget)
    }

    /// Dilation by a cube of half-size `k` voxels (see [`Occupancy::erode`] for the budget).
    pub fn dilate(&self, k: u32, budget: usize) -> Result<Self, usize> {
        if k == 0 {
            return Ok(self.clone());
        }
        self.along_z(k, true, budget)?.along_xy(k as usize, 0, true, budget)?.along_xy(k as usize, 1, true, budget)
    }

    /// Closing then opening by a cube of half-size `k`: the solid loses every tunnel, gap, bridge
    /// and spur at most `2k` voxels wide and keeps what is at least `2k + 1` wide.
    pub fn close_open(&self, k: u32, budget: usize) -> Result<Self, usize> {
        self.dilate(k, budget)?.erode(k, budget)?.erode(k, budget)?.dilate(k, budget)
    }

    /// One pass that fills a voxel wherever two inside voxels touch only along an edge (a 2 × 2
    /// block of voxels in a checkerboard): the surface of such a set is not a manifold. Returns the
    /// new set and the number of voxels added.
    fn fill_diagonals_once(&self, budget: usize) -> Result<(Self, u64), usize> {
        // additions are few: the bound is the runs of the set (checked again once built)
        let grown = self.transform(budget, self.runs.len(), |o, i, j, out| {
            out.extend_from_slice(o.column(i, j));
            let own = o.column(i, j).to_vec();
            // along z: a neighbour column's run ends where an own run starts
            for (di, dj) in [(-1i64, 0i64), (1, 0), (0, -1), (0, 1)] {
                let (a, b) = (i as i64 + di, j as i64 + dj);
                if a < 0 || b < 0 {
                    continue;
                }
                let nb = o.column(a as usize, b as usize);
                for r in &own {
                    if r[0] > 0 && nb.iter().any(|s| s[1] == r[0]) {
                        out.push([r[0] - 1, r[0]]);
                    }
                }
            }
            // within a layer: this column and the diagonal one are outside, the two between inside
            for (di, dj) in [(-1i64, -1i64), (1, -1), (-1, 1), (1, 1)] {
                let (ri, rj) = (i as i64 + di, j as i64 + dj);
                if ri < 0 || rj < 0 || (rj, ri) < (j as i64, i as i64) {
                    continue;
                }
                let get = |a: i64, b: i64| if a < 0 || b < 0 { &[][..] } else { o.column(a as usize, b as usize) };
                let (p, q, r) = (get(i as i64 + di, j as i64), get(i as i64, j as i64 + dj), get(ri, rj));
                let (mut both, mut outside) = (Vec::new(), own.clone());
                intersect(p, q, &mut both);
                outside.extend_from_slice(r);
                outside.sort_unstable();
                merge_sorted(&mut outside);
                subtract(&both, &outside, out);
            }
        })?;
        let added = grown.voxel_count().saturating_sub(self.voxel_count());
        Ok((grown, added))
    }

    /// Repeats [`Occupancy::fill_diagonals_once`] until nothing is added (at most `rounds` times);
    /// returns the set, the voxels added and whether it settled.
    pub fn fill_diagonals(&self, rounds: usize, budget: usize) -> Result<(Self, u64, bool), usize> {
        let (mut cur, mut total) = (self.clone(), 0u64);
        for _ in 0..rounds {
            let (next, added) = cur.fill_diagonals_once(budget)?;
            total += added;
            cur = next;
            if added == 0 {
                return Ok((cur, total, true));
            }
        }
        Ok((cur, total, false))
    }

    /// The runs of both columns that differ: `(from, to, upper)` voxel ranges where exactly one of
    /// the two columns is inside, `upper` telling that it is the second.
    #[must_use]
    pub fn differences(a: &[Run], b: &[Run]) -> Vec<(u32, u32, bool)> {
        let mut only_a = Vec::new();
        let mut only_b = Vec::new();
        subtract(a, b, &mut only_a);
        subtract(b, a, &mut only_b);
        let mut out: Vec<(u32, u32, bool)> = only_a.iter().map(|r| (r[0], r[1], false)).collect();
        out.extend(only_b.iter().map(|r| (r[0], r[1], true)));
        out.sort_unstable();
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const BIG: usize = usize::MAX / 2;

    fn block(n: usize, lo: usize, hi: usize) -> Occupancy {
        let cols =
            (lo..hi).flat_map(|j| (lo..hi).map(move |i| ((j * n + i) as u32, vec![[lo as u32, hi as u32]]))).collect();
        Occupancy::from_columns(n, n, n as u32, cols)
    }

    #[test]
    fn closing_and_opening_keep_a_thick_block_and_remove_a_thin_one() {
        let thick = block(24, 6, 18);
        assert_eq!(thick.close_open(1, BIG).unwrap().voxel_count(), thick.voxel_count());
        let thin = block(24, 6, 8);
        assert_eq!(thin.close_open(1, BIG).unwrap().voxel_count(), 0);
        let three = block(24, 6, 9);
        assert_eq!(three.close_open(1, BIG).unwrap().voxel_count(), three.voxel_count());
    }

    #[test]
    fn dilation_grows_a_cube_by_k_on_every_side() {
        let b = block(24, 8, 12);
        assert_eq!(b.dilate(2, BIG).unwrap().voxel_count(), 8 * 8 * 8);
        assert_eq!(b.dilate(2, BIG).unwrap().erode(2, BIG).unwrap().voxel_count(), b.voxel_count());
    }

    #[test]
    fn a_run_budget_stops_a_solid_whose_runs_multiply_and_reports_the_count() {
        // a stack of thin layers, every second voxel: runs along z in every column
        let cols: Vec<(u32, Vec<Run>)> = (4..20usize)
            .flat_map(|j| {
                (4..20usize).map(move |i| ((j * 24 + i) as u32, (0..8u32).map(|l| [2 * l, 2 * l + 1]).collect()))
            })
            .collect();
        let o = Occupancy::from_columns(24, 24, 24, cols);
        assert_eq!(o.run_count(), 16 * 16 * 8);
        let need = o.dilate(1, BIG).unwrap().run_count();
        assert!(need >= 16 * 16, "{need}");
        let r = o.close_open(1, 100);
        assert!(matches!(r, Err(n) if n > 100), "{r:?}");
    }

    #[test]
    fn a_grow_whose_bound_passes_the_budget_is_refused_with_the_bound_before_any_row_is_built() {
        // eight layers per column, three voxels apart: dilation by one keeps them apart
        let cols: Vec<(u32, Vec<Run>)> = (4..20usize)
            .flat_map(|j| {
                (4..20usize).map(move |i| ((j * 24 + i) as u32, (0..4u32).map(|l| [4 * l + 1, 4 * l + 2]).collect()))
            })
            .collect();
        let o = Occupancy::from_columns(24, 24, 24, cols);
        let runs = o.run_count();
        assert_eq!(runs, 16 * 16 * 4);
        // the real result of the x-dilation has as many runs as the input (neighbours are identical and
        // merge), so building it would pass a budget of 1.5 x; the bound is 3 x and refuses first
        let budget = runs + runs / 2;
        // the built result: the layers of the neighbours coincide and merge, two columns are added at the ends
        assert_eq!(o.along_xy(1, 0, true, BIG).unwrap().run_count(), runs + 2 * 16 * 4);
        assert_eq!(o.along_xy(1, 0, true, budget).err(), Some(3 * runs));
        assert_eq!(o.along_z(1, true, runs - 1).err(), Some(runs));
    }

    #[test]
    fn a_tunnel_of_one_voxel_is_closed() {
        let mut cols = Vec::new();
        for j in 4..20usize {
            for i in 4..20usize {
                if (i, j) != (12, 12) {
                    cols.push(((j * 24 + i) as u32, vec![[4u32, 20u32]]));
                }
            }
        }
        let o = Occupancy::from_columns(24, 24, 24, cols);
        assert!(!o.contains(12, 12, 10));
        assert!(o.close_open(1, BIG).unwrap().contains(12, 12, 10));
    }

    #[test]
    fn diagonal_contacts_are_filled() {
        let cols = vec![(2 * 8 + 2, vec![[2u32, 4u32]]), (3 * 8 + 3, vec![[2u32, 4u32]])];
        let o = Occupancy::from_columns(8, 8, 8, cols);
        let (filled, added, settled) = o.fill_diagonals(4, BIG).unwrap();
        assert!(settled && added > 0);
        let both = filled.contains(3, 2, 2) || filled.contains(2, 3, 2);
        assert!(both);
    }
}
