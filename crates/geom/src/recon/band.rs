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
    for &rho in radii.iter().filter(|r| r.is_finite() && **r > 0.0) {
        let grid = Grid::build(input.points, 2.0 * rho);
        let mut front = front_of(&bpa, input.front, first_new, rho);
        let mut pivots = 0usize;
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
