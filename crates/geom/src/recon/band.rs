//! Ball pivoting on a band: the seam between pieces of surface that already exist (`crate::merge`).
//!
//! The pivoting of [`super::pivot`] is reused with three changes. The front is given (the edges of
//! the existing triangles that border the band), no triangle is seeded anywhere else, and every
//! triangle a pivot would build passes a last test of the caller (which points it may use, that
//! it lies on surface some scan saw, that it crosses nothing). Every directed edge is used once, so
//! the band is manifold along its edges and oriented like the triangles it starts from.
use super::grid::Grid;
use super::pivot::{Bpa, FrontEdge, ball_center};
use super::*;
use crate::{Cancel, tri};

/// Pivots between two looks at the cancellation flag.
const CANCEL_EVERY: usize = 512;

/// A directed edge `a → b` of the existing triangle `(a, b, c)`, from which the band grows.
pub(crate) type FrontTriangle = [u32; 3];

/// The points, their normals and the triangles that exist already.
pub(crate) struct BandInput<'a> {
    /// Points (every vertex of the existing triangles and nothing else).
    pub(crate) points: &'a [Point3],
    /// A unit normal per point (the side the surface faces).
    pub(crate) normals: &'a [Vec3],
    /// Triangles that exist (counter-clockwise seen from outside).
    pub(crate) triangles: &'a [[u32; 3]],
    /// The front: border edges of existing triangles from which the band grows.
    pub(crate) front: &'a [FrontTriangle],
}

/// Centre of the ball resting on the existing triangle `(a, b, c)` with radius `rho`; above the
/// middle of the edge `a → b` along the triangle normal when the triangle is wider than the ball.
fn start_center(bpa: &Bpa<'_>, [a, b, c]: FrontTriangle, rho: f64) -> Option<Point3> {
    let p = |i: u32| bpa.pts.get(i as usize).copied();
    let (pa, pb, pc) = (p(a)?, p(b)?, p(c)?);
    if let Some((center, _)) = ball_center(&pa, &pb, &pc, rho) {
        return Some(center);
    }
    let n = tri::normal(&pa, &pb, &pc)?;
    Some(Point3::from((pa.coords + pb.coords) * 0.5) + n * rho)
}

/// The front of one pass: the given edges and the open edges of the band built so far, each once
/// and only while it has no neighbour.
fn front_of(bpa: &Bpa<'_>, given: &[FrontTriangle], first_new: usize, rho: f64) -> VecDeque<FrontEdge> {
    let mut out = VecDeque::new();
    let built = bpa.tris.get(first_new..).unwrap_or(&[]);
    let rotations = built.iter().flat_map(|t| [[t[0], t[1], t[2]], [t[1], t[2], t[0]], [t[2], t[0], t[1]]]);
    for f in given.iter().copied().chain(rotations) {
        let [a, b, c] = f;
        if bpa.edges.contains_key(&super::pivot::key(b, a)) {
            continue;
        }
        if let Some(center) = start_center(bpa, f, rho) {
            out.push_back(FrontEdge { a, b, c, center });
        }
    }
    out
}

/// Grows the band from `input.front` with balls of the radii `radii` (in order), building only
/// triangles `accept` takes. Returns the new triangles, or `None` when `cancel` was set.
pub(crate) fn pivot_band(
    input: &BandInput<'_>,
    radii: &[f64],
    accept: &mut dyn FnMut([u32; 3]) -> bool,
    cancel: &Cancel,
) -> Option<Vec<[u32; 3]>> {
    let mut bpa = Bpa::new(input.points, input.normals);
    for &t in input.triangles {
        bpa.add_triangle(t);
    }
    let first_new = bpa.tris.len();
    bpa.accept = Some(accept);
    let mut pivots = 0usize;
    for &rho in radii.iter().filter(|r| r.is_finite() && **r > 0.0) {
        if cancel.is_cancelled() {
            return None;
        }
        let grid = Grid::build(input.points, 2.0 * rho);
        let mut front = front_of(&bpa, input.front, first_new, rho);
        while let Some(f) = front.pop_front() {
            pivots += 1;
            if pivots.is_multiple_of(CANCEL_EVERY) && cancel.is_cancelled() {
                return None;
            }
            bpa.pivot(&grid, rho, &f, &mut front);
        }
    }
    bpa.accept = None;
    Some(bpa.tris.split_off(first_new))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two strips of triangles in the plane z = 0 facing +z, a gap of `gap` between them, and the front:
    /// the facing border edges of both. Returns (points, triangles, front).
    fn strips(gap: f64) -> (Vec<Point3>, Vec<[u32; 3]>, Vec<FrontTriangle>) {
        let n = 10;
        let mut pts = Vec::new();
        for (y0, _) in [(0.0, 0), (1.0 + gap, 1)] {
            for row in 0..2 {
                for i in 0..=n {
                    pts.push(Point3::new(f64::from(i), y0 + f64::from(row), 0.0));
                }
            }
        }
        let at = |strip: u32, row: u32, i: u32| strip * 2 * (n + 1) + row * (n + 1) + i;
        let (mut tris, mut front) = (Vec::new(), Vec::new());
        for s in 0..2 {
            for i in 0..n {
                tris.push([at(s, 0, i), at(s, 0, i + 1), at(s, 1, i + 1)]);
                tris.push([at(s, 0, i), at(s, 1, i + 1), at(s, 1, i)]);
            }
        }
        for i in 0..n {
            front.push([at(0, 1, i + 1), at(0, 1, i), at(0, 0, i)]);
            front.push([at(1, 0, i), at(1, 0, i + 1), at(1, 1, i + 1)]);
        }
        (pts, tris, front)
    }

    fn run(
        pts: &[Point3],
        tris: &[[u32; 3]],
        front: &[FrontTriangle],
        accept: &mut dyn FnMut([u32; 3]) -> bool,
    ) -> Vec<[u32; 3]> {
        let nrm = vec![Vec3::z(); pts.len()];
        let input = BandInput { points: pts, normals: &nrm, triangles: tris, front };
        pivot_band(&input, &[1.0, 2.0], accept, &Cancel::new()).unwrap()
    }

    #[test]
    fn the_band_between_two_strips_is_closed_only_with_what_the_hook_accepts() {
        let (pts, tris, front) = strips(1.0);
        let all = run(&pts, &tris, &front, &mut |_| true);
        // a gap of 10 by 1 between rows of 11 points: 20 triangles close it
        assert_eq!(all.len(), 20, "{all:?}");
        assert!(run(&pts, &tris, &front, &mut |_| false).is_empty(), "a hook that refuses all builds nothing");
        let some = run(&pts, &tris, &front, &mut |t| t.iter().all(|&v| pts[v as usize].x <= 5.0));
        assert!(!some.is_empty() && some.len() < all.len());
        assert!(some.iter().flatten().all(|&v| pts[v as usize].x <= 5.0));
    }

    #[test]
    fn a_point_no_triangle_uses_is_never_a_corner() {
        let (mut pts, tris, front) = strips(1.0);
        let stray = pts.len() as u32;
        pts.push(Point3::new(4.5, 1.5, 0.0));
        let built = run(&pts, &tris, &front, &mut |_| true);
        assert!(!built.is_empty());
        assert!(built.iter().flatten().all(|&v| v != stray));
    }

    #[test]
    fn a_wall_standing_on_the_band_does_not_keep_the_ball_out() {
        let (mut pts, mut tris, front) = strips(1.0);
        // a wall over the middle of the gap, its normal along y (perpendicular to the band)
        let w = pts.len() as u32;
        pts.extend([Point3::new(2.0, 1.5, 0.3), Point3::new(8.0, 1.5, 0.3), Point3::new(5.0, 1.5, 3.0)]);
        tris.push([w, w + 1, w + 2]);
        let mut nrm = vec![Vec3::z(); pts.len()];
        for k in 0..3 {
            nrm[w as usize + k] = Vec3::y();
        }
        let input = BandInput { points: &pts, normals: &nrm, triangles: &tris, front: &front };
        let built = pivot_band(&input, &[1.0, 2.0], &mut |t| !t.iter().any(|&v| v >= w), &Cancel::new()).unwrap();
        assert_eq!(built.len(), 20, "{built:?}");
    }
}
