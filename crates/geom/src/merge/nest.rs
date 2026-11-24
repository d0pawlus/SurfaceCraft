//! Closed pieces of a scan with a negative volume: turned inside out, inner skins (inside another
//! closed piece: the inside of a hollow part, a cavity) or debris. One BVH over the closed pieces'
//! triangles and one over their boxes are built once; each negative piece finds the pieces whose box
//! holds one of its vertices through the box BVH, then casts rays from that vertex that count only
//! the crossings with those pieces (a closed surface whose box does not contain a point is crossed
//! an even number of times).
//!
//! The whole stage has one budget of work (nodes visited, boxes listed, triangles tested),
//! proportional to the triangles of the closed pieces: a pile of coincident pieces or many
//! concentric shells cannot make it quadratic. The pieces are judged largest first, and a piece whose
//! point no other piece's box holds is decided by that box listing alone (it cannot be an inner
//! skin), so the large pieces that decide the verdict are judged before a pile of small ones can
//! spend the budget. Pieces left unjudged (the budget ran
//! out, or no two directions agreed) are inner skins with a warning while their area stays below
//! [`INVERTED_MIN_SHARE`] of the scan's; above it the scan is refused, as for loose inverted pieces:
//! running out never accepts a material inside-out surface.
use crate::bvh::RayCast;
use crate::nesting::{DIRECTIONS, MAX_CROSSINGS};
use crate::{Aabb, Bvh, Cancel, Mesh, Point3, Vec3, pieces};

/// A closed piece smaller than this share of its scan's area is not judged for being inside out
/// (debris).
pub const INVERTED_MIN_SHARE: f64 = 0.01;
/// Rays of the inside test ignore crossings closer than this share of the scan's diagonal.
const NEST_EPS_SHARE: f64 = 1e-9;
/// Work of the whole stage at most, per triangle of the closed pieces ...
pub(super) const WORK_PER_TRIANGLE: usize = 64;
/// ... plus this.
pub(super) const WORK_BASE: usize = 1 << 16;
/// Work of one cast at most (a share of the budget, so that one pile cannot eat it all at once).
const MAX_CAST_WORK: usize = 1 << 14;
/// Work between two looks at the cancellation flag (plus one cast at most).
pub(super) const CANCEL_WORK: usize = 4096;

/// What the nesting test cost: BVH builds and work (nodes visited, boxes listed, triangles tested).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct Cost {
    pub(super) builds: usize,
    pub(super) work: usize,
    /// The budget of the stage.
    pub(super) budget: usize,
    /// Work at which the cancellation flag is looked at next.
    next_look: usize,
    /// Looks taken.
    pub(super) looks: usize,
}

impl Cost {
    fn left(&self) -> usize {
        self.budget.saturating_sub(self.work)
    }
}

/// The closed pieces with a negative volume of a scan, sorted.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) struct Negative {
    /// Turned inside out (refuse the merge).
    pub(super) inverted: usize,
    /// Inside another closed piece: an inner skin (the unjudged ones included).
    pub(super) skins: usize,
    /// Small loose pieces below [`INVERTED_MIN_SHARE`] of the area all together.
    pub(super) debris: usize,
    /// Pieces whose inside test could not be completed within the budget (no two directions
    /// agreed). Below [`INVERTED_MIN_SHARE`] of the area together they are counted among the inner
    /// skins (a warning); above it they refuse the scan (`unjudged_refuses`).
    pub(super) unjudged: usize,
    /// The unjudged pieces cover at least [`INVERTED_MIN_SHARE`] of the scan's area: refused.
    pub(super) unjudged_refuses: bool,
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

/// Is a piece inside the others: yes, no, or not known (no two directions judged alike).
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
    cancel: &'p Cancel,
    cancelled: std::cell::Cell<bool>,
}

impl Closed<'_> {
    /// Looks at the cancellation flag once [`CANCEL_WORK`] has been done since the last look; true
    /// when cancelled (remembered).
    fn polled(&self, cost: &mut Cost) -> bool {
        if cost.work >= cost.next_look {
            cost.next_look = cost.work.saturating_add(CANCEL_WORK);
            cost.looks += 1;
            if self.cancel.is_cancelled() {
                self.cancelled.set(true);
            }
        }
        self.cancelled.get()
    }

    /// The point of piece `k` its test starts from (its first triangle's first corner).
    fn probe(&self, mesh: &Mesh, k: usize) -> Option<Point3> {
        let t = self.pieces.get(k).and_then(|p| p.triangles.first()).and_then(|&t| mesh.triangles.get(t as usize))?;
        mesh.positions.get(t[0] as usize).copied()
    }

    /// The pieces other than `k` whose box holds `p`, ascending; `None` when the budget ran out.
    fn holding(&self, k: usize, p: &Point3, cost: &mut Cost) -> Option<Vec<usize>> {
        // capped like a cast: a listing of a pile of coincident boxes costs at most MAX_CAST_WORK
        let (found, work, out) = self.boxes.candidates_counted(&|b| contains(b, p), cost.left().min(MAX_CAST_WORK));
        cost.work += work;
        if out {
            return None;
        }
        Some(
            found
                .into_iter()
                .map(|j| j as usize)
                .filter(|&j| j != k && self.pieces.get(j).is_some_and(|q| contains(&q.bbox, p)))
                .collect(),
        )
    }

    /// Does piece `k` lie inside the union of the other closed pieces? The parity of the crossings
    /// along three directions; the verdict of at least two judged directions that agree, else
    /// unknown.
    fn nested(&self, mesh: &Mesh, k: usize, cost: &mut Cost) -> Inside {
        let Some(p) = self.probe(mesh, k) else { return Inside::No };
        if self.polled(cost) {
            return Inside::Unknown;
        }
        let Some(holders) = self.holding(k, &p, cost) else { return Inside::Unknown };
        if holders.is_empty() {
            return Inside::No;
        }
        let keep = |t: usize| self.piece_of.get(t).is_some_and(|j| holders.binary_search(j).is_ok());
        let (mut odd, mut even) = (0, 0);
        for d in &DIRECTIONS {
            let dir = Vec3::new(d[0], d[1], d[2]).normalize();
            match self.crossings(&p, &dir, &keep, cost) {
                Some(n) if n % 2 == 1 => odd += 1,
                Some(_) => even += 1,
                None => {}
            }
        }
        if odd >= 2 {
            Inside::Yes
        } else if even >= 2 {
            Inside::No
        } else {
            Inside::Unknown
        }
    }

    /// Crossings of the ray from `p` along `dir` with the triangles `keep` passes; `None` when the
    /// budget ran out or the ray met [`MAX_CROSSINGS`] surfaces.
    fn crossings(&self, p: &Point3, dir: &Vec3, keep: &dyn Fn(usize) -> bool, cost: &mut Cost) -> Option<usize> {
        let (mut o, mut n) = (*p, 0);
        loop {
            if n >= MAX_CROSSINGS || self.polled(cost) {
                return None;
            }
            let (cast, work) = self.bvh.ray_cast_counted(&o, dir, f64::INFINITY, keep, cost.left().min(MAX_CAST_WORK));
            cost.work += work;
            match cast {
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
fn closed_set<'p>(mesh: &Mesh, closed: Vec<&'p pieces::Piece>, cancel: &'p Cancel) -> Closed<'p> {
    let ids: Vec<u32> = closed.iter().flat_map(|p| p.triangles.iter().copied()).collect();
    let piece_of: Vec<usize> =
        closed.iter().enumerate().flat_map(|(j, p)| std::iter::repeat_n(j, p.triangles.len())).collect();
    let boxes = Mesh {
        positions: closed.iter().flat_map(|p| [p.bbox.min, p.bbox.max]).collect(),
        triangles: (0..closed.len() as u32).map(|j| [2 * j, 2 * j + 1, 2 * j]).collect(),
    };
    let eps = NEST_EPS_SHARE * mesh.bbox().diagonal().max(f64::MIN_POSITIVE);
    Closed {
        bvh: Bvh::build(&mesh.submesh(&ids)),
        pieces: closed,
        piece_of,
        boxes: Bvh::build(&boxes),
        eps,
        cancel,
        cancelled: std::cell::Cell::new(false),
    }
}

/// [`negative_pieces`] and what its nesting tests cost.
pub(super) fn negative_pieces_costed(mesh: &Mesh, cancel: &Cancel) -> Option<(Negative, Cost)> {
    let all = pieces::pieces(mesh);
    let closed: Vec<&pieces::Piece> = all.iter().filter(|p| p.closed()).collect();
    // largest first (ties by order): the pieces that decide the verdict are judged first
    let mut negative: Vec<usize> =
        (0..closed.len()).filter(|&k| closed.get(k).is_some_and(|p| volume(mesh, p) < 0.0)).collect();
    let area_of = |k: usize| closed.get(k).map_or(0.0, |p| p.area);
    negative.sort_by(|&a, &b| area_of(b).total_cmp(&area_of(a)).then(a.cmp(&b)));
    let triangles: usize = closed.iter().map(|p| p.triangles.len()).sum();
    let mut cost =
        Cost { budget: WORK_BASE.saturating_add(WORK_PER_TRIANGLE.saturating_mul(triangles)), ..Cost::default() };
    if negative.is_empty() {
        return Some((Negative::default(), cost));
    }
    cost.builds += 1;
    let set = closed_set(mesh, closed, cancel);
    let verdicts = judge(&set, mesh, &negative, &mut cost)?;
    Some((sort_out(&set, mesh.area(), &verdicts), cost))
}

/// The verdict of every negative piece (`(piece, verdict)`), largest first, within the budget: a
/// piece no other piece's box holds is decided by its box listing alone (no cast). `None` when
/// cancelled.
fn judge(set: &Closed<'_>, mesh: &Mesh, negative: &[usize], cost: &mut Cost) -> Option<Vec<(usize, Inside)>> {
    let mut verdicts: Vec<(usize, Inside)> = Vec::with_capacity(negative.len());
    for &k in negative {
        if set.polled(cost) {
            return None;
        }
        // once the budget is spent the rest are unjudged without any more work
        let inside = if cost.left() == 0 { Inside::Unknown } else { set.nested(mesh, k, cost) };
        if set.cancelled.get() {
            return None;
        }
        verdicts.push((k, inside));
    }
    Some(verdicts)
}

/// The verdicts sorted into turned inside out, inner skins, debris and unjudged (see [`Negative`]).
fn sort_out(set: &Closed<'_>, area: f64, verdicts: &[(usize, Inside)]) -> Negative {
    let (mut out, mut debris_area, mut unjudged_area) = (Negative::default(), 0.0, 0.0);
    for &(k, inside) in verdicts {
        let a = set.pieces.get(k).map_or(0.0, |p| p.area);
        match inside {
            Inside::Yes => out.skins += 1,
            Inside::Unknown => {
                out.unjudged += 1;
                unjudged_area += a;
            }
            Inside::No if a < INVERTED_MIN_SHARE * area => {
                out.debris += 1;
                debris_area += a;
            }
            Inside::No => out.inverted += 1,
        }
    }
    if debris_area >= INVERTED_MIN_SHARE * area {
        out.inverted += out.debris;
        out.debris = 0;
    }
    if unjudged_area >= INVERTED_MIN_SHARE * area {
        out.unjudged_refuses = true;
    } else {
        out.skins += out.unjudged;
    }
    out
}
