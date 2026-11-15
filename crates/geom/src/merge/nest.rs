//! Closed pieces of a scan with a negative volume: turned inside out, inner skins (inside another
//! closed piece: the inside of a hollow part, a cavity) or debris. One BVH over the closed pieces'
//! triangles and one over their boxes are built once; each negative piece finds the pieces whose box
//! holds one of its vertices through the box BVH, then casts rays from that vertex that count only
//! the crossings with those pieces (a closed surface whose box does not contain a point is crossed
//! an even number of times). The cost grows with the pieces that can hold a piece, not with all
//! pieces times all triangles or all pieces times all pieces.
use crate::bvh::RayCast;
use crate::nesting::{DIRECTIONS, MAX_CROSSINGS};
use crate::{Aabb, Bvh, Cancel, Mesh, Point3, Vec3, pieces};

/// A closed piece smaller than this share of its scan's area is not judged for being inside out
/// (debris).
pub const INVERTED_MIN_SHARE: f64 = 0.01;
/// Rays of the inside test ignore crossings closer than this share of the scan's diagonal.
const NEST_EPS_SHARE: f64 = 1e-9;
/// Triangle tests of one ray cast at most (rejected ones count too: many shells along the ray can
/// exhaust it); a cast that runs out, or a ray that meets [`MAX_CROSSINGS`] surfaces, leaves the
/// piece unjudged: judged as if alone (it can refuse, never hide an inversion) and counted.
const MAX_RAY_TESTS: usize = 1 << 16;
/// Box tests and ray casts between two looks at the cancellation flag.
const CANCEL_WORK: usize = 4096;

/// What the nesting test cost: BVH builds, box tests and ray casts.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct Cost {
    pub(super) builds: usize,
    pub(super) boxes: usize,
    pub(super) casts: usize,
}

impl Cost {
    fn work(&self) -> usize {
        self.boxes + self.casts
    }
}

/// The closed pieces with a negative volume of a scan, sorted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct Negative {
    /// Turned inside out (refuse the merge).
    pub(super) inverted: usize,
    /// Inside another closed piece: an inner skin.
    pub(super) skins: usize,
    /// Small loose pieces below [`INVERTED_MIN_SHARE`] of the area all together.
    pub(super) debris: usize,
    /// Of all of them, pieces whose inside test ran out (judged as if alone).
    pub(super) unjudged: usize,
}

impl Negative {
    /// (turned inside out, inner skins, debris).
    pub(super) fn counts(&self) -> (usize, usize, usize) {
        (self.inverted, self.skins, self.debris)
    }
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

/// Is a piece inside the others: yes, no, or not known (a cast ran out).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Inside {
    Yes,
    No,
    Unknown,
}

/// The closed pieces of a mesh in one BVH, with the piece of every triangle in it, and their boxes
/// in another (one degenerate triangle per piece whose box is the piece's).
struct Closed<'p> {
    pieces: Vec<&'p pieces::Piece>,
    bvh: Bvh,
    piece_of: Vec<usize>,
    boxes: Bvh,
    eps: f64,
}

impl Closed<'_> {
    /// The pieces other than `k` whose box holds `p`, ascending.
    fn holding(&self, k: usize, p: &Point3, cost: &mut Cost) -> Vec<usize> {
        let tests = std::cell::Cell::new(0usize);
        let found = self.boxes.candidates(&|b| {
            tests.set(tests.get() + 1);
            contains(b, p)
        });
        cost.boxes += tests.get() + found.len();
        found
            .into_iter()
            .map(|j| j as usize)
            .filter(|&j| j != k && self.pieces.get(j).is_some_and(|q| contains(&q.bbox, p)))
            .collect()
    }

    /// Does piece `k` lie inside the union of the other closed pieces (parity of the crossings along
    /// three directions, majority)? Counts its work in `cost`.
    fn nested(&self, mesh: &Mesh, k: usize, cost: &mut Cost) -> Inside {
        let probe = self.pieces.get(k).and_then(|p| p.triangles.first()).and_then(|&t| mesh.triangles.get(t as usize));
        let Some(p) = probe.and_then(|t| mesh.positions.get(t[0] as usize)).copied() else { return Inside::No };
        let holders = self.holding(k, &p, cost);
        if holders.is_empty() {
            return Inside::No;
        }
        let keep = |t: usize| self.piece_of.get(t).is_some_and(|j| holders.binary_search(j).is_ok());
        let mut odd = 0;
        for d in &DIRECTIONS {
            let dir = Vec3::new(d[0], d[1], d[2]).normalize();
            match self.crossings(&p, &dir, &keep, cost) {
                Some(n) => odd += n % 2,
                None => return Inside::Unknown,
            }
        }
        if odd >= 2 { Inside::Yes } else { Inside::No }
    }

    /// Crossings of the ray from `p` along `dir` with the triangles `keep` passes; `None` when a cast
    /// ran out of its budget or the ray met [`MAX_CROSSINGS`] surfaces.
    fn crossings(&self, p: &Point3, dir: &Vec3, keep: &dyn Fn(usize) -> bool, cost: &mut Cost) -> Option<usize> {
        let (mut o, mut n) = (*p, 0);
        loop {
            if n >= MAX_CROSSINGS {
                return None;
            }
            cost.casts += 1;
            match self.bvh.ray_cast_bounded(&o, dir, f64::INFINITY, keep, MAX_RAY_TESTS) {
                RayCast::Hit(_, t) => {
                    n += 1;
                    o += dir * (t + self.eps);
                }
                RayCast::Miss => return Some(n),
                RayCast::Exhausted => return None,
            }
        }
    }
}

/// Is `p` in the box `b` (touching counts)?
fn contains(b: &Aabb, p: &Point3) -> bool {
    !b.is_empty() && (0..3).all(|k| b.min[k] <= p[k] && p[k] <= b.max[k])
}

/// The closed pieces of `mesh` with a negative volume, sorted (see [`Negative`]). An inner skin lies
/// inside another closed piece of the scan (the inside of a hollow part, a cavity, faces inward),
/// whatever its size; debris is a piece below [`INVERTED_MIN_SHARE`] of the scan's area that lies
/// inside nothing, as long as all such pieces together stay below it (many small inverted pieces
/// are a scan turned inside out in parts: counted as turned inside out). Only the first refuse the
/// merge. `None` when cancelled.
pub(super) fn negative_pieces(mesh: &Mesh, cancel: &Cancel) -> Option<Negative> {
    negative_pieces_costed(mesh, cancel).map(|(n, _)| n)
}

/// The two BVHs of the closed pieces (see [`Closed`]).
fn closed_set<'p>(mesh: &Mesh, closed: Vec<&'p pieces::Piece>) -> Closed<'p> {
    let ids: Vec<u32> = closed.iter().flat_map(|p| p.triangles.iter().copied()).collect();
    let piece_of: Vec<usize> =
        closed.iter().enumerate().flat_map(|(j, p)| std::iter::repeat_n(j, p.triangles.len())).collect();
    let boxes = Mesh {
        positions: closed.iter().flat_map(|p| [p.bbox.min, p.bbox.max]).collect(),
        triangles: (0..closed.len() as u32).map(|j| [2 * j, 2 * j + 1, 2 * j]).collect(),
    };
    let eps = NEST_EPS_SHARE * mesh.bbox().diagonal().max(f64::MIN_POSITIVE);
    Closed { bvh: Bvh::build(&mesh.submesh(&ids)), pieces: closed, piece_of, boxes: Bvh::build(&boxes), eps }
}

/// [`negative_pieces`] and what its nesting tests cost.
pub(super) fn negative_pieces_costed(mesh: &Mesh, cancel: &Cancel) -> Option<(Negative, Cost)> {
    let all = pieces::pieces(mesh);
    let closed: Vec<&pieces::Piece> = all.iter().filter(|p| p.closed()).collect();
    let negative: Vec<usize> =
        (0..closed.len()).filter(|&k| closed.get(k).is_some_and(|p| volume(mesh, p) < 0.0)).collect();
    let mut cost = Cost::default();
    if negative.is_empty() {
        return Some((Negative::default(), cost));
    }
    cost.builds += 1;
    let set = closed_set(mesh, closed);
    let area = mesh.area();
    let (mut out, mut debris_area, mut next_look) = (Negative::default(), 0.0, 0usize);
    for &k in &negative {
        // polled by the work done since the last look
        if cost.work() >= next_look {
            if cancel.is_cancelled() {
                return None;
            }
            next_look = cost.work() + CANCEL_WORK;
        }
        let Some(p) = set.pieces.get(k) else { continue };
        let inside = set.nested(mesh, k, &mut cost);
        out.unjudged += usize::from(inside == Inside::Unknown);
        if inside == Inside::Yes {
            out.skins += 1;
        } else if p.area < INVERTED_MIN_SHARE * area {
            out.debris += 1;
            debris_area += p.area;
        } else {
            out.inverted += 1;
        }
    }
    if debris_area >= INVERTED_MIN_SHARE * area {
        out.inverted += out.debris;
        out.debris = 0;
    }
    Some((out, cost))
}
