//! The outline of a region: its boundary loops, smoothed and simplified to a stated tolerance,
//! ready to trim a fitted sheet (P8-H4). Mesh-side computation only: the loops are chains of
//! mesh vertices; turning them into a trimmed B-rep face is the kernel's.
//!
//! The boundary of a region of triangles is noisy (a region grown over a scan follows the
//! triangle edges, a zigzag of about one edge length). The loops are measured in the plane frame
//! of the height grid, smoothed by a few passes of a closed binomial filter, and simplified
//! (Ramer–Douglas–Peucker, `geom::section::simplify`) so that every dropped vertex stays within
//! `tolerance` mm of the kept chain. The longest loop is the outer one, the loops of the opposite
//! winding are holes; holes smaller than a few tolerances across are noise and are ignored.
use crate::resample::HeightGrid;
use std::collections::HashMap;
use surfacecraft_geom::edit::boundary_loops;
use surfacecraft_geom::section::simplify;
use surfacecraft_geom::{Mesh, Point3};

/// Passes of the closed `[1 2 1] / 4` filter applied to every boundary loop.
const SMOOTHING_PASSES: usize = 2;
/// A hole whose area is below `π (HOLE_RADIUS_IN_TOLERANCES · tolerance)²` is ignored as noise.
const HOLE_RADIUS_IN_TOLERANCES: f64 = 5.0;
/// Fewest corners of a loop worth keeping.
const MIN_CORNERS: usize = 3;

/// The trimming outline of a region.
#[derive(Clone, Debug, PartialEq)]
pub struct Outline {
    /// Corners of the outer loop (smoothed surface-side points, the first not repeated).
    pub outer: Vec<Point3>,
    /// Corners of each hole.
    pub holes: Vec<Vec<Point3>>,
    /// Holes ignored as smaller than the noise limit (loops that collapse under the
    /// simplification count here too).
    pub holes_ignored: usize,
    /// Boundary loops of other, disconnected parts of the region (only the largest is trimmed).
    pub islands_ignored: usize,
    /// Area enclosed by the outer loop minus the holes, in the plane frame, mm².
    pub area: f64,
}

fn plane_xy(g: &HeightGrid, p: &Point3) -> [f64; 2] {
    let d = p - g.origin;
    [d.dot(&g.u), d.dot(&g.v)]
}

fn area2(p: &[[f64; 2]]) -> f64 {
    let n = p.len();
    (0..n)
        .map(|i| {
            let (a, b) = (p[i], p[(i + 1) % n]);
            a[0] * b[1] - b[0] * a[1]
        })
        .sum()
}

/// Splits a loop that visits a vertex twice (a pinch point of the boundary) into simple loops.
fn split_pinches(chain: &[u32]) -> Vec<Vec<u32>> {
    let mut stack: Vec<u32> = Vec::new();
    let mut at: HashMap<u32, usize> = HashMap::new();
    let mut loops = Vec::new();
    for &v in chain {
        if let Some(&k) = at.get(&v) {
            let cycle: Vec<u32> = stack.split_off(k);
            for c in cycle.iter().skip(1) {
                at.remove(c);
            }
            if cycle.len() >= MIN_CORNERS {
                loops.push(cycle);
            }
            stack.push(v);
            at.insert(v, k);
        } else {
            at.insert(v, stack.len());
            stack.push(v);
        }
    }
    if stack.len() >= MIN_CORNERS {
        loops.push(stack);
    }
    loops
}

fn smooth(points: &[Point3]) -> Vec<Point3> {
    let n = points.len();
    let mut cur = points.to_vec();
    for _ in 0..SMOOTHING_PASSES {
        let next: Vec<Point3> = (0..n)
            .map(|i| Point3::from((cur[(i + n - 1) % n].coords + cur[i].coords * 2.0 + cur[(i + 1) % n].coords) / 4.0))
            .collect();
        cur = next;
    }
    cur
}

/// A smoothed loop reduced to its corners, with the signed area (plane frame, mm²) they
/// enclose. The corners are chosen in the plane frame; `None` when fewer than 3 remain.
fn reduce_loop(g: &HeightGrid, pts: &[Point3], tolerance: f64) -> Option<(Vec<Point3>, f64)> {
    let smoothed = smooth(pts);
    let flat: Vec<[f64; 2]> = smoothed.iter().map(|p| plane_xy(g, p)).collect();
    let kept = simplify(&flat, true, tolerance);
    // `simplify` keeps a subset of the input in order: walk both lists together
    let mut corners = Vec::with_capacity(kept.len());
    let mut from = 0;
    for k in &kept {
        let hit = flat.iter().enumerate().skip(from).find(|(_, f)| *f == k).map(|(i, _)| i)?;
        corners.push(*smoothed.get(hit)?);
        from = hit + 1;
    }
    if corners.len() < MIN_CORNERS {
        return None;
    }
    let area = area2(&kept) / 2.0;
    area.is_finite().then_some((corners, area))
}

/// The outline of the triangles `tris` of `mesh`, measured in the plane frame of `grid`.
/// `None` when the region has no boundary loop (a closed surface) or none survives simplification.
#[must_use]
pub fn region_outline(mesh: &Mesh, tris: &[u32], grid: &HeightGrid, tolerance: f64) -> Option<Outline> {
    if !(tolerance.is_finite() && tolerance > 0.0) {
        return None;
    }
    let sub = mesh.submesh(tris);
    let mut loops: Vec<(Vec<Point3>, f64)> = Vec::new();
    let mut collapsed = 0;
    for chain in boundary_loops(&sub) {
        for simple in split_pinches(&chain) {
            let pts: Vec<Point3> = simple.iter().filter_map(|&v| sub.positions.get(v as usize).copied()).collect();
            match reduce_loop(grid, &pts, tolerance) {
                Some(l) => loops.push(l),
                None => collapsed += 1,
            }
        }
    }
    let (outer_idx, _) = loops.iter().enumerate().max_by(|a, b| a.1.1.abs().total_cmp(&b.1.1.abs()))?;
    let (outer, outer_area) = loops.get(outer_idx)?.clone();
    let hole_min = std::f64::consts::PI * (HOLE_RADIUS_IN_TOLERANCES * tolerance).powi(2);
    let (mut holes, mut holes_ignored, mut islands_ignored) = (Vec::new(), collapsed, 0);
    let mut area = outer_area.abs();
    for (i, (pts, a)) in loops.into_iter().enumerate() {
        if i == outer_idx {
            continue;
        }
        if (a > 0.0) == (outer_area > 0.0) {
            islands_ignored += 1;
        } else if a.abs() < hole_min {
            holes_ignored += 1;
        } else {
            area -= a.abs();
            holes.push(pts);
        }
    }
    Some(Outline { outer, holes, holes_ignored, islands_ignored, area })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resample::height_grid;
    use surfacecraft_geom::Vec3;

    /// A flat grid mesh of `n × n` cells of 1 mm, without the cells for which `skip` is true.
    fn grid_mesh(n: usize, skip: impl Fn(usize, usize) -> bool) -> (Mesh, Vec<u32>) {
        let mut positions = Vec::new();
        for j in 0..=n {
            for i in 0..=n {
                positions.push(Point3::new(i as f64, j as f64, 0.0));
            }
        }
        let w = (n + 1) as u32;
        let mut triangles = Vec::new();
        for j in 0..n {
            for i in 0..n {
                if skip(i, j) {
                    continue;
                }
                let a = j as u32 * w + i as u32;
                triangles.push([a, a + 1, a + w + 1]);
                triangles.push([a, a + w + 1, a + w]);
            }
        }
        let ids = (0..triangles.len() as u32).collect();
        (Mesh { positions, triangles }, ids)
    }

    fn grid_of(mesh: &Mesh, tris: &[u32]) -> HeightGrid {
        let sub = mesh.submesh(tris);
        let normals = vec![Vec3::new(0.0, 0.0, 1.0); sub.positions.len()];
        height_grid(&sub.positions, &normals, 20, 20).unwrap()
    }

    #[test]
    fn square_with_a_hole_and_a_notch() {
        // 30 × 30 mm, a 6 × 6 hole in the middle and a 5 × 5 notch cut from a corner
        let (mesh, ids) =
            grid_mesh(30, |i, j| ((12..18).contains(&i) && (12..18).contains(&j)) || (i >= 25 && j >= 25));
        let g = grid_of(&mesh, &ids);
        let o = region_outline(&mesh, &ids, &g, 0.1).unwrap();
        assert_eq!(o.holes.len(), 1);
        assert_eq!((o.holes_ignored, o.islands_ignored), (0, 0));
        // collinear boundary vertices are dropped; the smoothing chamfers each corner (the 6 of
        // the L and the 4 of the hole) into a few vertices, far fewer than the 116 of the boundary
        assert!(o.outer.len() >= 6 && o.outer.len() <= 40, "{} corners", o.outer.len());
        assert!(o.holes[0].len() >= 4 && o.holes[0].len() <= 20, "{} corners", o.holes[0].len());
        // area: 900 - 25 notch - 36 hole, up to the corner rounding of the smoothing
        let exact = 900.0 - 25.0 - 36.0;
        assert!((o.area - exact).abs() / exact < 0.01, "area {} vs {exact}", o.area);
    }

    #[test]
    fn tiny_holes_and_islands_are_ignored() {
        let (mesh, ids) = grid_mesh(30, |i, j| i == 10 && j == 10);
        let g = grid_of(&mesh, &ids);
        // a one-cell hole (1 mm²) is noise at tolerance 0.5 (limit π (2.5)² ≈ 19.6 mm²)
        let o = region_outline(&mesh, &ids, &g, 0.5).unwrap();
        assert!(o.holes.is_empty(), "{} holes", o.holes.len());
        assert!(o.holes_ignored >= 1);
        // a region of two separate squares keeps the larger one
        let (mesh, ids) = grid_mesh(30, |i, j| !((i < 12 && j < 12) || (i >= 20 && j >= 20 && i < 26 && j < 26)));
        let g = grid_of(&mesh, &ids);
        let o = region_outline(&mesh, &ids, &g, 0.1).unwrap();
        assert_eq!(o.islands_ignored, 1);
        assert!((o.area - 144.0).abs() < 4.0, "area {}", o.area);
    }

    #[test]
    fn hostile_input_gives_none() {
        let (mesh, ids) = grid_mesh(10, |_, _| false);
        let g = grid_of(&mesh, &ids);
        assert!(region_outline(&mesh, &ids, &g, f64::NAN).is_none());
        assert!(region_outline(&mesh, &ids, &g, -1.0).is_none());
        assert!(region_outline(&mesh, &[], &g, 0.1).is_none());
        assert!(region_outline(&mesh, &[u32::MAX], &g, 0.1).is_none());
    }

    #[test]
    fn pinched_loops_are_split() {
        // a figure of eight: 0-1-2-0-3-4
        let parts = split_pinches(&[0, 1, 2, 0, 3, 4]);
        assert_eq!(parts.len(), 2);
        assert!(parts.iter().any(|p| p == &vec![0, 1, 2]));
        assert!(parts.iter().any(|p| p == &vec![0, 3, 4]));
    }
}
