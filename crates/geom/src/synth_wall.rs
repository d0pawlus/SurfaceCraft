//! A synthetic thin wall scanned from both sides, with tunnels through it: two curved sheets `gap`
//! apart, joined along one edge (the edge of the wall), open on the other three (one rim), and square
//! tubes through the wall at chosen cells. Each tube is one handle. One-cell tubes stand for the
//! tunnels a scanner makes where the two sides of a thin wall come closer than its resolution; wider
//! tubes are real holes of the part. Exact ground truth: every sheet vertex lies on its sheet.
use crate::{Mesh, Point3};

/// What to build.
#[derive(Clone, Debug, PartialEq)]
pub struct Wall {
    /// Side of the square panel (mm).
    pub size: f64,
    /// Grid cells along each side.
    pub cells: usize,
    /// Distance between the two sheets (mm).
    pub gap: f64,
    /// Sag of the panel at its corners (mm): the sheets are `z = bend · (x² + y²) / size²`.
    pub bend: f64,
    /// Tubes through the wall: `(i, j, k)` opens the `k × k` cells from cell `(i, j)` in both sheets and
    /// joins the two openings. Tubes that overlap, touch the edge of the panel or each other are skipped.
    pub tubes: Vec<(usize, usize, usize)>,
}

impl Wall {
    /// Height of the middle surface of the wall at `(x, y)`.
    #[must_use]
    pub fn height(&self, x: f64, y: f64) -> f64 {
        self.bend * (x * x + y * y) / (self.size * self.size).max(f64::MIN_POSITIVE)
    }

    /// The tubes that are built: inside the panel (at least one cell from its edge), not touching
    /// each other (at least one cell apart).
    #[must_use]
    pub fn built_tubes(&self) -> Vec<(usize, usize, usize)> {
        let n = self.cells;
        let mut taken = vec![false; (n + 2) * (n + 2)];
        let mut out = Vec::new();
        for &(i, j, k) in &self.tubes {
            if k == 0 || i < 1 || j < 1 || i + k + 1 > n || j + k + 1 > n {
                continue;
            }
            let near = |a: usize, b: usize| a..=b;
            let free =
                near(i - 1, i + k).all(|x| near(j - 1, j + k).all(|y| taken.get(y * (n + 2) + x) == Some(&false)));
            if !free {
                continue;
            }
            for y in j - 1..=j + k {
                for x in i - 1..=i + k {
                    if let Some(t) = taken.get_mut(y * (n + 2) + x) {
                        *t = true;
                    }
                }
            }
            out.push((i, j, k));
        }
        out
    }
}

/// Builds the wall (see the module).
#[must_use]
pub fn wall(w: &Wall) -> Mesh {
    let n = w.cells.max(2);
    let side = n + 1;
    let step = w.size / n as f64;
    let mut positions = Vec::with_capacity(2 * side * side);
    for sheet in [0.5, -0.5] {
        for j in 0..side {
            for i in 0..side {
                let (x, y) = (i as f64 * step, j as f64 * step);
                positions.push(Point3::new(x, y, w.height(x, y) + sheet * w.gap));
            }
        }
    }
    let a = |i: usize, j: usize| (j * side + i) as u32;
    let b = |i: usize, j: usize| (side * side + j * side + i) as u32;
    let tubes = w.built_tubes();
    let open =
        |i: usize, j: usize| tubes.iter().any(|&(ti, tj, k)| (ti..ti + k).contains(&i) && (tj..tj + k).contains(&j));
    let mut triangles = Vec::new();
    for j in 0..n {
        for i in 0..n {
            if open(i, j) {
                continue;
            }
            let (p, q, r, s) = (a(i, j), a(i + 1, j), a(i + 1, j + 1), a(i, j + 1));
            triangles.extend([[p, q, r], [p, r, s]]);
            let (p, q, r, s) = (b(i, j), b(i + 1, j), b(i + 1, j + 1), b(i, j + 1));
            triangles.extend([[p, r, q], [p, s, r]]);
        }
    }
    // the edge of the wall at x = 0 joins the sheets
    for j in 0..n {
        triangles.extend([[a(0, j), a(0, j + 1), b(0, j + 1)], [a(0, j), b(0, j + 1), b(0, j)]]);
    }
    for &(i, j, k) in &tubes {
        for (p, q) in perimeter(i, j, k) {
            triangles.extend([[a(p.0, p.1), a(q.0, q.1), b(q.0, q.1)], [a(p.0, p.1), b(q.0, q.1), b(p.0, p.1)]]);
        }
    }
    Mesh { positions, triangles }
}

/// The unit edges around the `k × k` cells from `(i, j)`, counter-clockwise seen from +z.
fn perimeter(i: usize, j: usize, k: usize) -> Vec<((usize, usize), (usize, usize))> {
    let mut e = Vec::with_capacity(4 * k);
    for s in 0..k {
        e.push(((i + s, j), (i + s + 1, j)));
        e.push(((i + k, j + s), (i + k, j + s + 1)));
        e.push(((i + k - s, j + k), (i + k - s - 1, j + k)));
        e.push(((i, j + k - s), (i, j + k - s - 1)));
    }
    e
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pieces::pieces;

    #[test]
    fn every_tube_is_one_handle_and_the_wall_has_one_rim() {
        let w = Wall {
            size: 40.0,
            cells: 40,
            gap: 0.6,
            bend: 3.0,
            tubes: vec![(5, 5, 1), (10, 10, 4), (5, 6, 1), (30, 30, 1)],
        };
        let m = wall(&w);
        assert!(m.validate().is_ok());
        let ps = pieces(&m);
        assert_eq!(ps.len(), 1);
        assert_eq!(ps[0].loops.len(), 1, "one rim");
        assert_eq!(ps[0].unoriented_edges, 0);
        assert_eq!(ps[0].non_manifold_edges, 0);
        assert_eq!(w.built_tubes().len(), 3, "the tube next to another is skipped");
        assert_eq!(ps[0].genus(), Some(3));
    }
}
