//! Measurement coverage of a fitted patch (P8-H11): which parts of the rectangle of its parameter
//! domain have scan data under them, which are holes of the scan that the patch bridges, and
//! which lie outside the outline of the data (the patch only extrapolates there).
//!
//! The data are triangles projected into the plane frame of the fit. They are rasterised on a fine
//! raster (cells about half a triangle edge wide, at most [`MAX_RASTER`] across); an empty cell
//! reachable from the border through empty cells is outside the data, any other empty cell is a
//! hole. The shares are counted on the fine raster; the coarse grid handed out (at most
//! [`MAX_CELLS`] across) takes the majority class of the fine cells under each cell.

/// Most cells across the coarse grid.
pub const MAX_CELLS: usize = 32;
/// Most cells across the fine raster.
const MAX_RASTER: usize = 512;
/// Width of a fine cell as a share of the mean triangle edge.
const CELL_EDGE_SHARE: f64 = 0.5;
/// Triangles whose edges are measured for the mean edge.
const EDGE_SAMPLES: usize = 4000;

/// What lies under a cell of the parameter domain.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cell {
    /// Scan data under it: the patch is measured here.
    Measured,
    /// Inside the outline of the data but without any: a hole the patch bridges.
    Hole,
    /// Outside the outline of the data: the patch extrapolates.
    Extrapolated,
}

impl Cell {
    /// One letter: `M`, `H` or `E`.
    #[must_use]
    pub fn letter(self) -> char {
        match self {
            Self::Measured => 'M',
            Self::Hole => 'H',
            Self::Extrapolated => 'E',
        }
    }
}

/// Shares of the domain's area per class; they sum to one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Shares {
    /// Measured.
    pub measured: f64,
    /// Hole.
    pub hole: f64,
    /// Extrapolated.
    pub extrapolated: f64,
}

/// The coverage of one domain rectangle.
#[derive(Clone, Debug, PartialEq)]
pub struct Coverage {
    /// Cells along the first axis (u).
    pub cols: usize,
    /// Cells along the second axis (v).
    pub rows: usize,
    /// Row-major, row 0 at the lowest v, column 0 at the lowest u.
    pub cells: Vec<Cell>,
    /// Area shares.
    pub shares: Shares,
}

impl Coverage {
    /// The grid as one string of letters per row (row 0 first).
    #[must_use]
    pub fn rows_as_text(&self) -> Vec<String> {
        self.cells.chunks(self.cols.max(1)).map(|r| r.iter().map(|c| c.letter()).collect()).collect()
    }
}

/// The fine raster: cell size, extent and which cells hold data.
struct Raster {
    x0: f64,
    y0: f64,
    cell: f64,
    nx: usize,
    ny: usize,
    data: Vec<bool>,
}

impl Raster {
    fn mark(&mut self, i: usize, j: usize) {
        if i < self.nx
            && let Some(c) = self.data.get_mut(j * self.nx + i)
        {
            *c = true;
        }
    }

    fn mark_point(&mut self, p: [f64; 2]) {
        let at = |v: f64, o: f64| ((v - o) / self.cell).floor().max(0.0) as usize;
        let (i, j) = (at(p[0], self.x0).min(self.nx - 1), at(p[1], self.y0).min(self.ny - 1));
        self.mark(i, j);
    }

    /// Marks the cells whose centre is inside `t` (scanline), and the cells of its corners.
    fn fill(&mut self, t: &[[f64; 2]; 3]) {
        for p in t {
            self.mark_point(*p);
        }
        let (ymin, ymax) = (
            t.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
            t.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
        );
        let first = ((ymin - self.y0) / self.cell - 0.5).ceil().max(0.0) as usize;
        let last = (((ymax - self.y0) / self.cell - 0.5).floor().max(-1.0) + 1.0) as usize;
        for j in first..last.min(self.ny) {
            let yc = self.y0 + (j as f64 + 0.5) * self.cell;
            let xs: Vec<f64> = (0..3)
                .filter_map(|k| {
                    let (a, b) = (t.get(k)?, t.get((k + 1) % 3)?);
                    ((a[1] <= yc) != (b[1] <= yc)).then(|| a[0] + (yc - a[1]) / (b[1] - a[1]) * (b[0] - a[0]))
                })
                .collect();
            let (lo, hi) = (
                xs.iter().copied().fold(f64::INFINITY, f64::min),
                xs.iter().copied().fold(f64::NEG_INFINITY, f64::max),
            );
            if !(lo.is_finite() && hi.is_finite()) {
                continue;
            }
            let i0 = ((lo - self.x0) / self.cell - 0.5).ceil().max(0.0) as usize;
            let i1 = ((hi - self.x0) / self.cell - 0.5).floor();
            if i1 < 0.0 {
                continue;
            }
            for i in i0..=(i1 as usize).min(self.nx - 1) {
                self.mark(i, j);
            }
        }
    }

    /// Empty cells connected to the border through empty cells.
    fn outside(&self) -> Vec<bool> {
        let (nx, ny) = (self.nx, self.ny);
        let mut out = vec![false; nx * ny];
        let empty = |k: usize| self.data.get(k).is_some_and(|d| !d);
        let mut stack: Vec<usize> = (0..nx * ny)
            .filter(|&k| (k % nx == 0 || k % nx == nx - 1 || k / nx == 0 || k / nx == ny - 1) && empty(k))
            .collect();
        for &k in &stack {
            out[k] = true;
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
                if empty(n) && !out[n] {
                    out[n] = true;
                    stack.push(n);
                }
            }
        }
        out
    }
}

/// Mean edge length of the triangles (an evenly strided sample); `None` when there is none.
fn mean_edge(tris: &[[[f64; 2]; 3]]) -> Option<f64> {
    let stride = tris.len().div_ceil(EDGE_SAMPLES).max(1);
    let edges: Vec<f64> = tris
        .iter()
        .step_by(stride)
        .flat_map(|t| (0..3).filter_map(move |k| Some((t.get(k)?, t.get((k + 1) % 3)?))))
        .map(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]))
        .filter(|e| e.is_finite())
        .collect();
    (!edges.is_empty()).then(|| edges.iter().sum::<f64>() / edges.len() as f64)
}

/// The class of the coarse cell `(ci, cj)`: the majority of the fine cells under it.
fn majority(classes: &[Cell], nx: usize, ny: usize, (cols, rows): (usize, usize), (ci, cj): (usize, usize)) -> Cell {
    let span =
        |c: usize, n: usize, total: usize| (c * total / n, ((c + 1) * total / n).max(c * total / n + 1).min(total));
    let ((i0, i1), (j0, j1)) = (span(ci, cols, nx), span(cj, rows, ny));
    let mut count = [0usize; 3];
    for j in j0..j1 {
        for i in i0..i1 {
            match classes.get(j * nx + i) {
                Some(Cell::Measured) => count[0] += 1,
                Some(Cell::Hole) => count[1] += 1,
                Some(Cell::Extrapolated) => count[2] += 1,
                None => {}
            }
        }
    }
    let best = count.iter().copied().max().unwrap_or(0);
    if count[0] == best {
        Cell::Measured
    } else if count[1] == best {
        Cell::Hole
    } else {
        Cell::Extrapolated
    }
}

/// The coverage of the rectangle `domain = [[x0, x1], [y0, y1]]` by `tris` (2-D triangles in the
/// plane frame of the fit). `None` for no usable triangle or a degenerate rectangle.
#[must_use]
pub fn coverage(tris: &[[[f64; 2]; 3]], domain: [[f64; 2]; 2]) -> Option<Coverage> {
    let [[x0, x1], [y0, y1]] = domain;
    let (w, h) = (x1 - x0, y1 - y0);
    if !(w.is_finite() && h.is_finite() && w > 0.0 && h > 0.0) {
        return None;
    }
    let cell = (CELL_EDGE_SHARE * mean_edge(tris)?).max(w.max(h) / MAX_RASTER as f64);
    let nx = ((w / cell).ceil() as usize).clamp(1, MAX_RASTER);
    let ny = ((h / cell).ceil() as usize).clamp(1, MAX_RASTER);
    let cell = (w / nx as f64).max(h / ny as f64);
    let mut raster = Raster { x0, y0, cell, nx, ny, data: vec![false; nx * ny] };
    for t in tris.iter().filter(|t| t.iter().all(|p| p[0].is_finite() && p[1].is_finite())) {
        raster.fill(t);
    }
    let outside = raster.outside();
    let classes: Vec<Cell> = raster
        .data
        .iter()
        .zip(&outside)
        .map(|(&d, &o)| {
            if d {
                Cell::Measured
            } else if o {
                Cell::Extrapolated
            } else {
                Cell::Hole
            }
        })
        .collect();
    let share = |c: Cell| classes.iter().filter(|&&k| k == c).count() as f64 / classes.len() as f64;
    let shares =
        Shares { measured: share(Cell::Measured), hole: share(Cell::Hole), extrapolated: share(Cell::Extrapolated) };
    let (cols, rows) = if w >= h {
        (MAX_CELLS.min(nx), ((MAX_CELLS as f64 * h / w).round() as usize).clamp(1, MAX_CELLS).min(ny))
    } else {
        (((MAX_CELLS as f64 * w / h).round() as usize).clamp(1, MAX_CELLS).min(nx), MAX_CELLS.min(ny))
    };
    let cells = (0..rows)
        .flat_map(|j| (0..cols).map(move |i| (i, j)))
        .map(|at| majority(&classes, nx, ny, (cols, rows), at))
        .collect();
    Some(Coverage { cols, rows, cells, shares })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two triangles per unit cell of a `n × n` lattice, skipping those whose centre `keep` refuses.
    fn lattice(n: usize, keep: impl Fn(f64, f64) -> bool) -> Vec<[[f64; 2]; 3]> {
        let mut out = Vec::new();
        for i in 0..n {
            for j in 0..n {
                let (x, y) = (i as f64, j as f64);
                if keep(x + 0.5, y + 0.5) {
                    out.push([[x, y], [x + 1.0, y], [x + 1.0, y + 1.0]]);
                    out.push([[x, y], [x + 1.0, y + 1.0], [x, y + 1.0]]);
                }
            }
        }
        out
    }

    #[test]
    fn full_data_is_all_measured() {
        let c = coverage(&lattice(20, |_, _| true), [[0.0, 20.0], [0.0, 20.0]]).unwrap();
        assert!((c.shares.measured - 1.0).abs() < 1e-9, "{:?}", c.shares);
        assert!(c.cells.iter().all(|&k| k == Cell::Measured));
    }

    #[test]
    fn interior_gap_is_a_hole_and_the_rest_is_extrapolated() {
        let keep = |x: f64, y: f64| (x - 30.0).hypot(y - 30.0) <= 27.0 && (x - 30.0).hypot(y - 30.0) >= 9.0;
        let c = coverage(&lattice(60, keep), [[0.0, 60.0], [0.0, 60.0]]).unwrap();
        let pi = std::f64::consts::PI;
        let (hole, outer) = (pi * 81.0 / 3600.0, (3600.0 - pi * 729.0) / 3600.0);
        assert!((c.shares.hole - hole).abs() < 0.01, "{:?} vs {hole}", c.shares);
        assert!((c.shares.extrapolated - outer).abs() < 0.01, "{:?} vs {outer}", c.shares);
        assert_eq!(c.cells.get((c.rows / 2) * c.cols + c.cols / 2), Some(&Cell::Hole));
        assert_eq!(c.cells.first(), Some(&Cell::Extrapolated));
    }

    #[test]
    fn degenerate_input_gives_none() {
        assert!(coverage(&[], [[0.0, 1.0], [0.0, 1.0]]).is_none());
        assert!(coverage(&lattice(3, |_, _| true), [[0.0, 0.0], [0.0, 1.0]]).is_none());
        assert!(coverage(&lattice(3, |_, _| true), [[f64::NAN, 1.0], [0.0, 1.0]]).is_none());
    }

    #[test]
    fn nan_triangles_are_ignored() {
        let mut t = lattice(5, |_, _| true);
        t.push([[f64::NAN, 0.0], [1.0, 0.0], [1.0, 1.0]]);
        assert!(coverage(&t, [[0.0, 5.0], [0.0, 5.0]]).is_some());
    }

    #[test]
    fn grid_is_at_most_32_across_and_keeps_the_aspect() {
        let c = coverage(&lattice(100, |_, _| true), [[0.0, 100.0], [0.0, 50.0]]).unwrap();
        assert_eq!((c.cols, c.rows), (32, 16));
        assert_eq!(c.rows_as_text().len(), 16);
    }
}
