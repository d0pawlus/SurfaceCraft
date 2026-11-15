//! Closed pieces of a scan with a negative volume: turned inside out, inner skins (inside another
//! closed piece: the inside of a hollow part, a cavity) or debris. One BVH over the closed pieces is
//! built once; each negative piece casts rays from one of its vertices that count only the crossings
//! with the other pieces whose box contains that vertex (a closed surface whose box does not contain
//! a point is crossed an even number of times), so the cost grows with the pieces that can hold it,
//! not with all pieces times all triangles.
use crate::bvh::RayCast;
use crate::nesting::{DIRECTIONS, MAX_CROSSINGS};
use crate::{Aabb, Bvh, Cancel, Mesh, Point3, Vec3, pieces};

/// A closed piece smaller than this share of its scan's area is not judged for being inside out
/// (debris).
pub const INVERTED_MIN_SHARE: f64 = 0.01;
/// Rays of the inside test ignore crossings closer than this share of the scan's diagonal.
const NEST_EPS_SHARE: f64 = 1e-9;
/// Triangle tests of one ray cast at most; a cast that runs out leaves the piece not nested (judged
/// as if alone: it can refuse, never hide an inversion).
const MAX_RAY_TESTS: usize = 1 << 16;
/// Negative pieces handled between two looks at the cancellation flag.
const CANCEL_EVERY: usize = 64;

/// What the nesting test cost: BVH builds and ray casts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct Cost {
    pub(super) builds: usize,
    pub(super) casts: usize,
}

/// Signed volume of a piece, measured about the centre of its box.
fn volume(mesh: &Mesh, p: &pieces::Piece) -> f64 {
    let o = p.bbox.center();
    p.triangles
        .iter()
        .filter_map(|&t| mesh.triangle(t as usize))
        .map(|[a, b, c]| (a - o).dot(&(b - o).cross(&(c - o))))
        .sum()
}

/// The closed pieces of a mesh in one BVH, with the piece of every triangle in it.
struct Closed<'p> {
    pieces: Vec<&'p pieces::Piece>,
    bvh: Bvh,
    piece_of: Vec<usize>,
    eps: f64,
}

impl Closed<'_> {
    /// Does piece `k` lie inside the union of the other closed pieces (parity of the crossings along
    /// three directions, majority)? Counts its casts in `cost`.
    fn nested(&self, mesh: &Mesh, k: usize, cost: &mut Cost) -> bool {
        let probe = self.pieces.get(k).and_then(|p| p.triangles.first()).and_then(|&t| mesh.triangles.get(t as usize));
        let Some(p) = probe.and_then(|t| mesh.positions.get(t[0] as usize)).copied() else { return false };
        let holds: Vec<bool> = self.pieces.iter().enumerate().map(|(j, q)| j != k && contains(&q.bbox, &p)).collect();
        if !holds.iter().any(|h| *h) {
            return false;
        }
        let keep = |t: usize| self.piece_of.get(t).and_then(|&j| holds.get(j)).copied().unwrap_or(false);
        let odd = DIRECTIONS
            .iter()
            .filter(|d| {
                let dir = Vec3::new(d[0], d[1], d[2]).normalize();
                self.crossings(&p, &dir, &keep, cost).is_some_and(|n| n % 2 == 1)
            })
            .count();
        odd >= 2
    }

    /// Crossings of the ray from `p` along `dir` with the triangles `keep` passes; `None` when a cast
    /// ran out of its budget.
    fn crossings(&self, p: &Point3, dir: &Vec3, keep: &dyn Fn(usize) -> bool, cost: &mut Cost) -> Option<usize> {
        let (mut o, mut n) = (*p, 0);
        while n < MAX_CROSSINGS {
            cost.casts += 1;
            match self.bvh.ray_cast_bounded(&o, dir, f64::INFINITY, keep, MAX_RAY_TESTS) {
                RayCast::Hit(_, t) => {
                    n += 1;
                    o += dir * (t + self.eps);
                }
                RayCast::Miss => break,
                RayCast::Exhausted => return None,
            }
        }
        Some(n)
    }
}

/// Is `p` in the box `b` (touching counts)?
fn contains(b: &Aabb, p: &Point3) -> bool {
    !b.is_empty() && (0..3).all(|k| b.min[k] <= p[k] && p[k] <= b.max[k])
}

/// Closed pieces with a negative volume, sorted: (turned inside out, inner skins, debris). An inner
/// skin lies inside another closed piece of the scan (the inside of a hollow part, a cavity, faces
/// inward), whatever its size; debris is a piece below [`INVERTED_MIN_SHARE`] of the scan's area
/// that lies inside nothing, as long as all such pieces together stay below it (many small inverted
/// pieces are a scan turned inside out in parts: counted as turned inside out). Only the first
/// refuse the merge. `None` when cancelled.
pub(super) fn negative_pieces(mesh: &Mesh, cancel: &Cancel) -> Option<(usize, usize, usize)> {
    negative_pieces_costed(mesh, cancel).map(|(n, _)| n)
}

/// [`negative_pieces`] and what its nesting tests cost.
pub(super) fn negative_pieces_costed(mesh: &Mesh, cancel: &Cancel) -> Option<((usize, usize, usize), Cost)> {
    let all = pieces::pieces(mesh);
    let closed: Vec<&pieces::Piece> = all.iter().filter(|p| p.closed()).collect();
    let negative: Vec<usize> =
        (0..closed.len()).filter(|&k| closed.get(k).is_some_and(|p| volume(mesh, p) < 0.0)).collect();
    let mut cost = Cost::default();
    if negative.is_empty() {
        return Some(((0, 0, 0), cost));
    }
    let ids: Vec<u32> = closed.iter().flat_map(|p| p.triangles.iter().copied()).collect();
    let piece_of: Vec<usize> =
        closed.iter().enumerate().flat_map(|(j, p)| std::iter::repeat_n(j, p.triangles.len())).collect();
    cost.builds += 1;
    let eps = NEST_EPS_SHARE * mesh.bbox().diagonal().max(f64::MIN_POSITIVE);
    let set = Closed { bvh: Bvh::build(&mesh.submesh(&ids)), pieces: closed, piece_of, eps };
    let area = mesh.area();
    let (mut inverted, mut skins, mut debris, mut debris_area) = (0, 0, 0, 0.0);
    for (n, &k) in negative.iter().enumerate() {
        if n % CANCEL_EVERY == 0 && cancel.is_cancelled() {
            return None;
        }
        let Some(p) = set.pieces.get(k) else { continue };
        if set.nested(mesh, k, &mut cost) {
            skins += 1;
        } else if p.area < INVERTED_MIN_SHARE * area {
            debris += 1;
            debris_area += p.area;
        } else {
            inverted += 1;
        }
    }
    if debris_area >= INVERTED_MIN_SHARE * area {
        return Some(((inverted + debris, skins, 0), cost));
    }
    Some(((inverted, skins, debris), cost))
}
