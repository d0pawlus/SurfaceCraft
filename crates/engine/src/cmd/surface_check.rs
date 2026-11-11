//! Deviation of the geometry `surface.fit` delivers (P8-H4): the trimmed faces, not the spline
//! they were cut from. Mesh-side distance computations on the faces' edges and tessellations.
use std::collections::HashMap;
use surfacecraft_geom::{Bvh, Mesh, Point3};
use surfacecraft_kernel::record::brep::{Brep, Curve};

/// Cells of the segment grid are this many median segment lengths wide (at least [`MIN_CELL`] mm),
/// and a query looks at most this many rings of cells out.
const CELL_SEGMENTS: f64 = 4.0;
const MIN_CELL: f64 = 0.05;
const MAX_RINGS: usize = 200;

/// The edges of a sheet as polylines (a trimmed face's edges are degree-1 splines).
fn edge_polylines(sheet: &Brep) -> Vec<Vec<Point3>> {
    sheet
        .edges
        .iter()
        .filter_map(|e| match &e.curve {
            Curve::Spline { points, .. } => Some(points.iter().map(|p| Point3::new(p[0], p[1], p[2])).collect()),
            Curve::Polyline { points } => Some(points.iter().map(|p| Point3::new(p[0], p[1], p[2])).collect()),
            _ => None,
        })
        .collect()
}

fn segment_distance(p: &Point3, a: &Point3, b: &Point3) -> f64 {
    let ab = b - a;
    let len2 = ab.norm_squared();
    let t = if len2 > 0.0 { ((p - a).dot(&ab) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p - (a + ab * t)).norm()
}

type Cell = (i64, i64, i64);

/// Segments hashed into a grid of cubic cells (a segment is listed in every cell its bounding
/// box touches), for exact nearest-segment queries.
struct SegmentGrid {
    cell: f64,
    segments: Vec<(Point3, Point3)>,
    cells: HashMap<Cell, Vec<usize>>,
}

impl SegmentGrid {
    fn new(segments: Vec<(Point3, Point3)>) -> Self {
        let mut lengths: Vec<f64> = segments.iter().map(|(a, b)| (b - a).norm()).collect();
        lengths.sort_by(f64::total_cmp);
        let median = lengths.get(lengths.len() / 2).copied().unwrap_or(1.0);
        let cell = (CELL_SEGMENTS * median).max(MIN_CELL);
        let mut grid = Self { cell, segments, cells: HashMap::new() };
        for (k, (a, b)) in grid.segments.clone().iter().enumerate() {
            let (lo, hi) = (
                grid.cell_of(&Point3::new(a.x.min(b.x), a.y.min(b.y), a.z.min(b.z))),
                grid.cell_of(&Point3::new(a.x.max(b.x), a.y.max(b.y), a.z.max(b.z))),
            );
            for i in lo.0..=hi.0 {
                for j in lo.1..=hi.1 {
                    for l in lo.2..=hi.2 {
                        grid.cells.entry((i, j, l)).or_default().push(k);
                    }
                }
            }
        }
        grid
    }

    fn cell_of(&self, p: &Point3) -> Cell {
        ((p.x / self.cell).floor() as i64, (p.y / self.cell).floor() as i64, (p.z / self.cell).floor() as i64)
    }

    /// Distance from `p` to the nearest segment: rings of cells grow until no unseen cell can
    /// hold a nearer one.
    fn nearest(&self, p: &Point3) -> f64 {
        let c = self.cell_of(p);
        let mut best = f64::INFINITY;
        for ring in 0..=MAX_RINGS {
            let r = ring as i64;
            for i in c.0 - r..=c.0 + r {
                for j in c.1 - r..=c.1 + r {
                    for l in c.2 - r..=c.2 + r {
                        let on_ring = (i - c.0).abs() == r || (j - c.1).abs() == r || (l - c.2).abs() == r;
                        if !on_ring {
                            continue;
                        }
                        for &k in self.cells.get(&(i, j, l)).into_iter().flatten() {
                            if let Some((a, b)) = self.segments.get(k) {
                                best = best.min(segment_distance(p, a, b));
                            }
                        }
                    }
                }
            }
            if best <= ring as f64 * self.cell {
                break;
            }
        }
        best
    }
}

/// Largest distance from a point of `boundary` to the edges of `sheet` (the trim loops of the
/// delivered face), mm: how far the face's outline lies from the region's boundary vertices.
/// `None` when the sheet has no edge polylines.
#[must_use]
pub(super) fn outline_deviation(boundary: &[Point3], sheet: &Brep) -> Option<f64> {
    let segments: Vec<(Point3, Point3)> =
        edge_polylines(sheet).iter().flat_map(|l| l.windows(2).map(|w| (w[0], w[1]))).collect();
    if segments.is_empty() {
        return None;
    }
    let grid = SegmentGrid::new(segments);
    let worst = boundary.iter().map(|p| grid.nearest(p)).fold(0.0, f64::max);
    worst.is_finite().then_some(worst)
}

/// Largest distance from a point of `pts` to the mesh (the tessellated delivered sheets).
#[must_use]
pub(super) fn delivered_deviation(pts: &[Point3], delivered: &Mesh) -> f64 {
    let bvh = Bvh::build(delivered);
    pts.iter().map(|p| bvh.closest(p, f64::INFINITY).map_or(f64::INFINITY, |h| h.distance)).fold(0.0, f64::max)
}
