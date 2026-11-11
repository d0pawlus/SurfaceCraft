//! The outline of a region: its boundary loops, simplified to a stated tolerance, ready to trim
//! a fitted sheet (P8-H4). Mesh-side computation only: the loops are chains of mesh vertices;
//! turning them into a trimmed B-rep face is the kernel's.
//!
//! The boundary of a region of triangles is noisy (a region grown over a scan follows the
//! triangle edges, a zigzag of about one edge length). Each loop is reduced to corners
//! (Ramer–Douglas–Peucker, in 3-D) after the most smoothing that keeps the *measured* distance of every raw boundary vertex
//! from the reduced loop within `tolerance`: two binomial passes, one, or none (then the corners
//! are raw boundary vertices and the bound holds by construction). The longest loop is the outer
//! one, the loops of the opposite winding are holes; holes smaller than a few tolerances across
//! are noise and are ignored. When a loop has more corners than its share of `max_corners`, the
//! simplification tolerance grows until it fits and the larger deviation is reported.
use crate::resample::HeightGrid;
use std::collections::HashMap;
use surfacecraft_geom::edit::boundary_loops;
use surfacecraft_geom::{Mesh, Point3};

/// Smoothing passes tried, most first (a pass is the closed `[1 2 1] / 4` filter).
const SMOOTHING_LADDER: [usize; 3] = [2, 1, 0];
/// Without smoothing, the simplification tolerance is halved this many times to meet the bound.
const TIGHTENINGS: usize = 3;
/// Growth of the simplification tolerance while a loop has too many corners.
const LOOSENING: f64 = 1.5;
/// The bound the reduction aims at is this share of the tolerance: the corners are then moved
/// onto the fitted surface, which shifts the outline a little in it.
const PROJECTION_MARGIN: f64 = 0.75;
/// A hole whose area is below `π (HOLE_RADIUS_IN_TOLERANCES · tolerance)²` is ignored as noise.
const HOLE_RADIUS_IN_TOLERANCES: f64 = 5.0;
/// Fewest corners of a loop worth keeping.
const MIN_CORNERS: usize = 3;

/// The trimming outline of a region.
#[derive(Clone, Debug, PartialEq)]
pub struct Outline {
    /// Corners of the outer loop (the first not repeated).
    pub outer: Vec<Point3>,
    /// Corners of each hole.
    pub holes: Vec<Vec<Point3>>,
    /// Raw boundary vertices of the loops kept (outer and holes): what the outline must hug.
    pub boundary: Vec<Point3>,
    /// Holes ignored as smaller than the noise limit (loops that collapse under the
    /// simplification count here too).
    pub holes_ignored: usize,
    /// Boundary loops of other, disconnected parts of the region (only the largest is trimmed).
    pub islands_ignored: usize,
    /// Area enclosed by the outer loop minus the holes, in the plane frame, mm².
    pub area: f64,
    /// Largest distance (mm) of a raw boundary vertex of a kept loop from the chord of the
    /// corners it lies between. At most the tolerance unless the corner limit forced more.
    pub deviation: f64,
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

fn smooth(points: &[Point3], passes: usize) -> Vec<Point3> {
    let n = points.len();
    let mut cur = points.to_vec();
    for _ in 0..passes {
        let next: Vec<Point3> = (0..n)
            .map(|i| Point3::from((cur[(i + n - 1) % n].coords + cur[i].coords * 2.0 + cur[(i + 1) % n].coords) / 4.0))
            .collect();
        cur = next;
    }
    cur
}

/// Distance from `p` to the segment `a b`.
fn segment_distance(p: &Point3, a: &Point3, b: &Point3) -> f64 {
    let ab = b - a;
    let len2 = ab.norm_squared();
    let t = if len2 > 0.0 { ((p - a).dot(&ab) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p - (a + ab * t)).norm()
}

/// One reduction of a loop: its corners (indices into the loop) and signed area.
struct Reduced {
    corners: Vec<usize>,
    points: Vec<Point3>,
    area: f64,
    deviation: f64,
}

/// Corner indices of the closed loop `points` after Ramer–Douglas–Peucker in 3-D: every dropped
/// vertex is within `tol` of the chord of its arc. (Done in 3-D, not in the plane of the grid:
/// a boundary running round a curved wall is a straight line seen from the plane's normal, and
/// its sag would be invisible there.) The loop is cut at its first vertex and the one farthest
/// from it; the work is an explicit stack, so a long loop cannot overflow it.
fn rdp_closed(points: &[Point3], tol: f64) -> Vec<usize> {
    let n = points.len();
    let Some(first) = points.first() else { return Vec::new() };
    let far = (1..n).max_by(|&a, &b| (points[a] - first).norm().total_cmp(&(points[b] - first).norm())).unwrap_or(0);
    let mut keep = vec![false; n + 1];
    for k in [0, far, n] {
        if let Some(f) = keep.get_mut(k) {
            *f = true;
        }
    }
    let at = |k: usize| points.get(k % n);
    let mut stack = vec![(0, far), (far, n)];
    while let Some((lo, hi)) = stack.pop() {
        let (Some(a), Some(b)) = (at(lo), at(hi)) else { continue };
        let worst = (lo + 1..hi)
            .filter_map(|k| at(k).map(|p| (k, segment_distance(p, a, b))))
            .max_by(|x, y| x.1.total_cmp(&y.1));
        if let Some((k, _)) = worst.filter(|w| w.1 > tol) {
            keep[k] = true;
            stack.push((lo, k));
            stack.push((k, hi));
        }
    }
    (0..n).filter(|&k| keep[k]).collect()
}

/// Reduces `raw` (smoothed `passes` times) with simplification tolerance `tol`.
fn reduce_once(g: &HeightGrid, raw: &[Point3], passes: usize, tol: f64) -> Option<Reduced> {
    let smoothed = smooth(raw, passes);
    let corners = rdp_closed(&smoothed, tol);
    if corners.len() < MIN_CORNERS {
        return None;
    }
    let flat: Vec<[f64; 2]> = corners.iter().filter_map(|&i| smoothed.get(i)).map(|p| plane_xy(g, p)).collect();
    let area = area2(&flat) / 2.0;
    let deviation = chord_deviation(raw, &smoothed, &corners);
    let points = corners.iter().filter_map(|&i| smoothed.get(i).copied()).collect();
    (area.is_finite() && deviation.is_finite()).then_some(Reduced { corners, points, area, deviation })
}

/// Largest distance of a raw vertex from the chord (of the smoothed loop's corners) over its arc.
fn chord_deviation(raw: &[Point3], smoothed: &[Point3], corners: &[usize]) -> f64 {
    let n = raw.len();
    let mut worst: f64 = 0.0;
    for (k, &c0) in corners.iter().enumerate() {
        let c1 = corners.get(k + 1).copied().unwrap_or_else(|| corners.first().copied().unwrap_or(0) + n);
        let (Some(a), Some(b)) = (smoothed.get(c0 % n), smoothed.get(c1 % n)) else { continue };
        for i in c0..=c1 {
            if let Some(p) = raw.get(i % n) {
                worst = worst.max(segment_distance(p, a, b));
            }
        }
    }
    worst
}

/// The loop reduced as far as the bound allows: the most smoothing, then the loosest tolerance
/// that keeps the deviation within `tolerance`; the corner budget may force a looser reduction.
fn reduce_loop(g: &HeightGrid, raw: &[Point3], tolerance: f64, budget: usize) -> Option<(Vec<Point3>, f64, f64)> {
    let mut best: Option<Reduced> = None;
    let aim = tolerance * PROJECTION_MARGIN;
    let mut ladder: Vec<(usize, f64)> = SMOOTHING_LADDER.iter().map(|&p| (p, aim)).collect();
    ladder.extend((1..=TIGHTENINGS).map(|k| (0, aim / 2f64.powi(k as i32))));
    for (passes, tol) in ladder {
        let Some(r) = reduce_once(g, raw, passes, tol) else { continue };
        let ok = r.deviation <= aim;
        best = Some(r);
        if ok {
            break;
        }
    }
    let mut tol = aim;
    let mut passes = 0;
    while best.as_ref().is_some_and(|r| r.corners.len() > budget.max(MIN_CORNERS)) && passes < 40 {
        tol *= LOOSENING;
        best = reduce_once(g, raw, 0, tol).or(best);
        passes += 1;
    }
    best.map(|r| (r.points, r.area, r.deviation))
}

/// A boundary loop reduced to corners.
struct Loop {
    corners: Vec<Point3>,
    raw: Vec<Point3>,
    area: f64,
    deviation: f64,
}

/// The outline of the triangles `tris` of `mesh`, measured in the plane frame of `grid`, with at
/// most about `max_corners` corners over all loops. `None` when the region has no boundary loop
/// (a closed surface) or none survives simplification.
#[must_use]
pub fn region_outline(
    mesh: &Mesh,
    tris: &[u32],
    grid: &HeightGrid,
    tolerance: f64,
    max_corners: usize,
) -> Option<Outline> {
    if !(tolerance.is_finite() && tolerance > 0.0) {
        return None;
    }
    let sub = mesh.submesh(tris);
    let chains: Vec<Vec<u32>> = boundary_loops(&sub).iter().flat_map(|c| split_pinches(c)).collect();
    // each loop may keep a share of the corners in proportion to its length
    let total: usize = chains.iter().map(Vec::len).sum::<usize>().max(1);
    let mut loops: Vec<Loop> = Vec::new();
    let mut collapsed = 0;
    for chain in &chains {
        let raw: Vec<Point3> = chain.iter().filter_map(|&v| sub.positions.get(v as usize).copied()).collect();
        let budget = (max_corners as f64 * raw.len() as f64 / total as f64).ceil() as usize;
        match reduce_loop(grid, &raw, tolerance, budget) {
            Some((corners, area, deviation)) => loops.push(Loop { corners, raw, area, deviation }),
            None => collapsed += 1,
        }
    }
    let (outer_idx, _) = loops.iter().enumerate().max_by(|a, b| a.1.area.abs().total_cmp(&b.1.area.abs()))?;
    let outer_area = loops.get(outer_idx)?.area;
    let hole_min = std::f64::consts::PI * (HOLE_RADIUS_IN_TOLERANCES * tolerance).powi(2);
    let (mut holes, mut holes_ignored, mut islands_ignored) = (Vec::new(), collapsed, 0);
    let (mut outer, mut boundary, mut deviation) = (Vec::new(), Vec::new(), 0.0f64);
    let mut area = outer_area.abs();
    for (i, l) in loops.into_iter().enumerate() {
        if i == outer_idx {
            outer = l.corners;
            boundary.extend(l.raw);
            deviation = deviation.max(l.deviation);
        } else if (l.area > 0.0) == (outer_area > 0.0) {
            islands_ignored += 1;
        } else if l.area.abs() < hole_min {
            holes_ignored += 1;
        } else {
            area -= l.area.abs();
            boundary.extend(l.raw);
            deviation = deviation.max(l.deviation);
            holes.push(l.corners);
        }
    }
    Some(Outline { outer, holes, boundary, holes_ignored, islands_ignored, area, deviation })
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
        let o = region_outline(&mesh, &ids, &g, 0.1, 4000).unwrap();
        assert_eq!(o.holes.len(), 1);
        assert_eq!((o.holes_ignored, o.islands_ignored), (0, 0));
        // straight runs reduce to their ends: no smoothing is allowed to chamfer a corner, since
        // that would pull the loop off the boundary vertices by more than the tolerance
        assert_eq!(o.outer.len(), 6, "{:?}", o.outer);
        assert_eq!(o.holes[0].len(), 4);
        assert!(o.deviation <= 0.1, "deviation {}", o.deviation);
        // area: 900 - 25 notch - 36 hole, up to the corner rounding of the smoothing
        let exact = 900.0 - 25.0 - 36.0;
        assert!((o.area - exact).abs() / exact < 1e-9, "area {} vs {exact}", o.area);
    }

    #[test]
    fn tiny_holes_and_islands_are_ignored() {
        let (mesh, ids) = grid_mesh(30, |i, j| i == 10 && j == 10);
        let g = grid_of(&mesh, &ids);
        // a one-cell hole (1 mm²) is noise at tolerance 0.5 (limit π (2.5)² ≈ 19.6 mm²)
        let o = region_outline(&mesh, &ids, &g, 0.5, 4000).unwrap();
        assert!(o.holes.is_empty(), "{} holes", o.holes.len());
        assert!(o.holes_ignored >= 1);
        // a region of two separate squares keeps the larger one
        let (mesh, ids) = grid_mesh(30, |i, j| !((i < 12 && j < 12) || (i >= 20 && j >= 20 && i < 26 && j < 26)));
        let g = grid_of(&mesh, &ids);
        let o = region_outline(&mesh, &ids, &g, 0.1, 4000).unwrap();
        assert_eq!(o.islands_ignored, 1);
        assert!((o.area - 144.0).abs() < 1e-9, "area {}", o.area);
    }

    #[test]
    fn hostile_input_gives_none() {
        let (mesh, ids) = grid_mesh(10, |_, _| false);
        let g = grid_of(&mesh, &ids);
        assert!(region_outline(&mesh, &ids, &g, f64::NAN, 4000).is_none());
        assert!(region_outline(&mesh, &ids, &g, -1.0, 4000).is_none());
        assert!(region_outline(&mesh, &[], &g, 0.1, 4000).is_none());
        assert!(region_outline(&mesh, &[u32::MAX], &g, 0.1, 4000).is_none());
    }

    #[test]
    fn pinched_loops_are_split() {
        // a figure of eight: 0-1-2-0-3-4
        let parts = split_pinches(&[0, 1, 2, 0, 3, 4]);
        assert_eq!(parts.len(), 2);
        assert!(parts.iter().any(|p| p == &vec![0, 1, 2]));
        assert!(parts.iter().any(|p| p == &vec![0, 3, 4]));
    }

    /// A disc cut from a grid mesh has a staircase boundary: the loop must hug its vertices
    /// within the tolerance whatever the smoothing did, and the corner budget is honoured by
    /// loosening (reported through `deviation`).
    #[test]
    fn staircase_boundary_is_hugged_and_budgeted() {
        let (mesh, ids) = grid_mesh(60, |i, j| ((i as f64 - 29.5).hypot(j as f64 - 29.5)) > 28.0);
        let g = grid_of(&mesh, &ids);
        let o = region_outline(&mesh, &ids, &g, 0.1, 4000).unwrap();
        assert!(o.deviation <= 0.1, "deviation {}", o.deviation);
        // independent check: every raw boundary vertex is within 0.1 mm of the corner polygon
        let n = o.outer.len();
        let worst = o
            .boundary
            .iter()
            .map(|p| {
                (0..n).map(|k| segment_distance(p, &o.outer[k], &o.outer[(k + 1) % n])).fold(f64::INFINITY, f64::min)
            })
            .fold(0.0, f64::max);
        assert!(worst <= 0.1 + 1e-9, "worst {worst}");
        assert!(n < o.boundary.len(), "{n} corners for {} boundary vertices", o.boundary.len());
        let tight = region_outline(&mesh, &ids, &g, 0.1, 24).unwrap();
        assert!(tight.outer.len() <= 24, "{} corners", tight.outer.len());
        assert!(tight.deviation > 0.1, "a corner limit that forces a looser outline says so: {}", tight.deviation);
    }

    /// The boundary of a patch of a pipe wall runs round the wall: seen along the patch's normal it
    /// is a straight line, but in space it bulges by the sagitta of the arc. The outline must be
    /// reduced in space, so that no boundary vertex is farther than the tolerance from it.
    #[test]
    fn boundary_round_a_curved_wall_is_reduced_in_space() {
        let (nu, nv) = (90usize, 30usize);
        let sweep = 100f64.to_radians();
        let mut positions = Vec::new();
        for j in 0..=nv {
            for i in 0..=nu {
                let th = sweep * i as f64 / nu as f64;
                positions.push(Point3::new(15.0 * th.cos(), 15.0 * th.sin(), 20.0 * j as f64 / nv as f64));
            }
        }
        let w = (nu + 1) as u32;
        let mut triangles = Vec::new();
        for j in 0..nv as u32 {
            for i in 0..nu as u32 {
                let a = j * w + i;
                triangles.push([a, a + 1, a + w + 1]);
                triangles.push([a, a + w + 1, a + w]);
            }
        }
        let mesh = Mesh { positions, triangles };
        let ids: Vec<u32> = (0..mesh.triangle_count() as u32).collect();
        let normals = mesh.vertex_normals();
        let (pts, nrm) = crate::segment::region_points(&mesh, &normals, &ids);
        let g = height_grid(&pts, &nrm, 20, 20).unwrap();
        let o = region_outline(&mesh, &ids, &g, 0.1, 4000).unwrap();
        let n = o.outer.len();
        let worst = o
            .boundary
            .iter()
            .map(|p| {
                (0..n).map(|k| segment_distance(p, &o.outer[k], &o.outer[(k + 1) % n])).fold(f64::INFINITY, f64::min)
            })
            .fold(0.0, f64::max);
        assert!(worst <= 0.1 + 1e-9, "a boundary vertex is {worst} mm from the outline, {n} corners");
        assert!(o.deviation <= 0.1);
    }
}
