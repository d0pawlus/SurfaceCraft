//! The tests every triangle the merge adds passes (restored, recovered, built in the seam or a hole),
//! and the record of those added so far. The guard reads the positions and normals of the union as
//! they are when a triangle is tested (vertices added since it was made included).
use super::overlap::COS_FACING;
use super::prep::Scan;
use super::union::Union;
use crate::{Aabb, Bvh, Mesh, Point3, Vec3, nesting, tri};
use std::collections::{HashMap, HashSet};

/// A seam triangle may lie this share of its longest edge (or the tolerance, if larger) from the scans.
pub const SUPPORT_SLOPE: f64 = 0.1;
/// Triangles near a new one tested for a crossing at most (the rest is taken as crossing).
const MAX_NEAR: usize = 1 << 16;
/// A new triangle spanning more grid cells than this along an axis is kept in a list tested every
/// time, and a candidate that large is tested against every new triangle.
pub const MAX_CELL_SPAN: i64 = 8;
/// A corner closer to an edge than this barycentric share is not inside (touching is no cover).
const INSIDE_MARGIN: f64 = 1e-3;
/// A crossing point closer to an end of an edge than this share of it is a touch, not a crossing.
const TOUCH_SHARE: f64 = 1e-6;

/// The points of the union a test reads: positions and normals.
#[derive(Clone, Copy)]
pub(super) struct Pts<'p> {
    pub(super) pos: &'p [Point3],
    pub(super) nrm: &'p [Vec3],
    pub(super) scan: &'p [u32],
}

impl<'p> Pts<'p> {
    pub(super) fn of(u: &'p Union) -> Self {
        Pts { pos: &u.positions, nrm: &u.normals, scan: &u.scan_of }
    }

    /// The scan all corners of `t` come from, if they come from one.
    fn scan_of(&self, t: &[u32; 3]) -> Option<u32> {
        let s = *self.scan.get(t[0] as usize)?;
        t.iter().all(|&v| self.scan.get(v as usize) == Some(&s)).then_some(s)
    }

    fn corners(&self, t: &[u32; 3]) -> Option<[Point3; 3]> {
        Some([*self.pos.get(t[0] as usize)?, *self.pos.get(t[1] as usize)?, *self.pos.get(t[2] as usize)?])
    }
}

/// Which corners a seam triangle may have. Every vertex of an edge the removal cut may be used. A
/// vertex of a scan's own border (a rim) may be used where the seam runs into it, but a triangle
/// has at least one cut vertex, two rim vertices only when they are the two ends of one rim edge
/// (the seam closes that edge; it never bridges two rims), and a rim vertex only where its normal
/// faces like the triangle (a boss standing on a face, a sphere sitting in a hole: their rims are
/// not joined to the face around them). A vertex added after the rule was made (by the recovery: a
/// scan's own vertex next to the seam) counts as cut.
pub(super) struct Corners {
    cut: Vec<bool>,
    rims: HashSet<u64>,
}

fn undirected(a: u32, b: u32) -> u64 {
    super::union::directed(a.min(b), a.max(b))
}

impl Corners {
    pub(super) fn new(u: &Union, scans: &[Scan], front: &[[u32; 3]]) -> Self {
        let mut cut = vec![false; u.positions.len()];
        for v in front.iter().flat_map(|f| [f[0], f[1]]) {
            if let Some(b) = cut.get_mut(v as usize) {
                *b = true;
            }
        }
        let edges = u.directed_edges();
        let mut rims = HashSet::new();
        for (i, t) in u.triangles.iter().enumerate() {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                if !edges.contains(&super::union::directed(b, a)) && !u.seam_side(scans, i, k) {
                    rims.insert(undirected(a, b));
                }
            }
        }
        Corners { cut, rims }
    }

    /// May a seam triangle have the corners `t`?
    pub(super) fn allows(&self, p: Pts<'_>, t: [u32; 3]) -> bool {
        let rim: Vec<u32> = t.iter().copied().filter(|&v| !self.cut.get(v as usize).copied().unwrap_or(true)).collect();
        let joined = match rim.as_slice() {
            [] | [_] => true,
            [a, b] => self.rims.contains(&undirected(*a, *b)),
            _ => false,
        };
        let Some(c) = p.corners(&t) else { return false };
        let Some(n) = tri::normal(&c[0], &c[1], &c[2]) else { return false };
        joined && rim.iter().all(|&v| p.nrm.get(v as usize).is_some_and(|m| m.dot(&n) >= COS_FACING))
    }
}

/// What the crossing test found.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Clash {
    Clear,
    Crossing,
    /// The test would have needed more comparisons than allowed.
    Budget,
}

/// The guard (see the module).
pub(super) struct Guard<'a> {
    scans: &'a [Scan],
    tol: f64,
    /// BVH over the triangles of the union when the guard was made, and those triangles.
    kept: Bvh,
    kept_tris: Vec<[u32; 3]>,
    /// New triangles (each once; `None` when taken back), by cell of a uniform grid (indices), and
    /// those too large for it.
    added: Vec<Option<[u32; 3]>>,
    index_of: HashMap<[u32; 3], u32>,
    cells: HashMap<[i64; 3], Vec<u32>>,
    large: Vec<u32>,
    cell: f64,
    /// Comparisons of the last crossing test.
    compared: std::cell::Cell<usize>,
    /// The corner rule of the seam, once the seam is being closed.
    pub(super) corners: Option<Corners>,
    pub(super) unsupported: usize,
    /// Refusals because the crossing test would have compared more than `max_near`: triangles, and
    /// holes the fill left open for that reason (once per fill pass).
    pub(super) over_budget: usize,
    /// Comparisons one crossing test may make ([`MAX_NEAR`]; lowered in tests).
    pub(super) max_near: usize,
    /// Set when a try of `accept_all` was refused for the budget (cleared by its caller).
    pub(super) patch_over_budget: bool,
    /// Collections of the new triangles a crossing test made (for a test of the early refusal).
    #[cfg(test)]
    collections: std::cell::Cell<usize>,
    pub(super) crossing: usize,
    pub(super) bad_corners: usize,
}

/// `b` grown by `d` on every side.
fn grown(b: &Aabb, d: f64) -> Aabb {
    let g = Vec3::new(d, d, d);
    Aabb { min: b.min - g, max: b.max + g }
}

fn cell_of(p: &Point3, cell: f64) -> [i64; 3] {
    [p.x, p.y, p.z].map(|c| (c / cell).floor() as i64)
}

impl<'a> Guard<'a> {
    pub(super) fn new(scans: &'a [Scan], u: &Union, tol: f64, cell: f64) -> Self {
        let kept_tris = u.triangles.clone();
        let kept = Bvh::build_owned(Mesh { positions: u.positions.clone(), triangles: kept_tris.clone() });
        let cell = if cell.is_finite() && cell > 0.0 { cell } else { 1.0 };
        Guard {
            scans,
            tol,
            kept,
            kept_tris,
            added: Vec::new(),
            index_of: HashMap::new(),
            cells: HashMap::new(),
            large: Vec::new(),
            cell,
            compared: std::cell::Cell::new(0),
            corners: None,
            unsupported: 0,
            over_budget: 0,
            max_near: MAX_NEAR,
            patch_over_budget: false,
            #[cfg(test)]
            collections: std::cell::Cell::new(0),
            crossing: 0,
            bad_corners: 0,
        }
    }

    /// Does triangle `t` (corners `c`) face the way its corners' normals do, and does every sample of
    /// it (centre, middles of the edges) lie on surface of a scan that faces that way too? (The
    /// corners' normals, not the triangle's: a sliver's own normal says little.)
    fn supported(&self, p: Pts<'_>, t: [u32; 3], c: &[Point3; 3]) -> bool {
        let normals: Option<Vec<Vec3>> = t.iter().map(|&v| p.nrm.get(v as usize).copied()).collect();
        let sum = normals.map_or_else(Vec3::zeros, |n| n.iter().fold(Vec3::zeros(), |s, x| s + x));
        let (Some(n), Some(own)) = (crate::normalize(sum), tri::normal(&c[0], &c[1], &c[2])) else {
            return false;
        };
        if own.dot(&n) <= 0.0 {
            return false;
        }
        let longest = (c[1] - c[0]).norm().max((c[2] - c[1]).norm()).max((c[0] - c[2]).norm());
        let reach = self.tol.max(SUPPORT_SLOPE * longest);
        let mid = |i: usize, j: usize| Point3::from((c[i].coords + c[j].coords) * 0.5);
        let centre = Point3::from((c[0].coords + c[1].coords + c[2].coords) / 3.0);
        [centre, mid(0, 1), mid(1, 2), mid(2, 0)].iter().all(|q| {
            self.scans
                .iter()
                .any(|s| s.bbox.dist2(q) <= reach * reach && s.bvh.closest_facing(q, reach, &[n], COS_FACING).is_some())
        })
    }

    /// Does triangle `t` (corners `c`) cross or cover a kept or a new triangle? Triangles near it are
    /// found with its box grown by the tolerance (a parallel layer within the tolerance has a box of
    /// its own that the exact one does not meet). At most `max_near` ([`MAX_NEAR`]) triangles are
    /// compared; a test that would need more is [`Clash::Budget`] (refused, counted apart).
    ///
    /// With `own` (a restored or recovered triangle), a triangle of the same scan is tested for
    /// crossing only, not for covering: a scan's own layers (a thin wall, a double skin) are its data
    /// and come back as they were.
    fn crosses(&self, p: Pts<'_>, t: [u32; 3], c: &[Point3; 3], own: bool) -> Clash {
        let bx = grown(&Aabb::from_points(c.iter()), self.tol);
        let (near, more) = self.kept.overlapping_up_to(&bx, self.max_near, &|_| true);
        if more {
            return Clash::Budget;
        }
        let mine = if own { p.scan_of(&t) } else { None };
        let clash = |x: &[u32; 3]| {
            let same = mine.is_some() && p.scan_of(x) == mine;
            p.corners(x).is_some_and(|y| clashes((c, &t), (&y, x), if same { None } else { Some(self.tol) }))
        };
        if near.iter().filter_map(|&i| self.kept_tris.get(i as usize)).any(clash) {
            return Clash::Crossing;
        }
        // the new triangles near it: those of its cells (or all live ones, for a candidate too large
        // for the grid) and the large ones, each once, within the budget left; a large candidate
        // over the budget is refused before anything is collected
        let large = self.too_large(&bx);
        if large && near.len() + self.index_of.len() > self.max_near {
            self.compared.set(near.len() + self.index_of.len());
            return Clash::Budget;
        }
        #[cfg(test)]
        self.collections.set(self.collections.get() + 1);
        let mut ids: Vec<u32> =
            if large { self.index_of.values().copied().collect() } else { self.cells_of(&bx).copied().collect() };
        ids.extend(self.large.iter().copied());
        ids.sort_unstable();
        ids.dedup();
        self.compared.set(near.len() + ids.len());
        if near.len() + ids.len() > self.max_near {
            return Clash::Budget;
        }
        if ids.iter().filter_map(|&i| self.added.get(i as usize).copied().flatten()).any(|x| clash(&x)) {
            Clash::Crossing
        } else {
            Clash::Clear
        }
    }

    /// Counts a refusal of `crosses`; true when the triangle passed.
    fn passed(&mut self, clash: Clash) -> bool {
        match clash {
            Clash::Clear => true,
            Clash::Crossing => {
                self.crossing += 1;
                false
            }
            Clash::Budget => {
                self.over_budget += 1;
                false
            }
        }
    }

    /// The cells the box `bx` spans along each axis.
    fn span(&self, bx: &Aabb) -> ([i64; 3], [i64; 3]) {
        (cell_of(&bx.min, self.cell), cell_of(&bx.max, self.cell))
    }

    fn too_large(&self, bx: &Aabb) -> bool {
        let (lo, hi) = self.span(bx);
        (0..3).any(|a| hi[a].saturating_sub(lo[a]) > MAX_CELL_SPAN)
    }

    /// The new triangles (indices into `added`) in the cells the box `bx` touches (a box within
    /// [`MAX_CELL_SPAN`]).
    fn cells_of<'s>(&'s self, bx: &Aabb) -> impl Iterator<Item = &'s u32> + 's {
        let (lo, hi) = self.span(bx);
        let span = move |a: usize| lo[a]..=hi[a].min(lo[a].saturating_add(MAX_CELL_SPAN));
        span(0)
            .flat_map(move |x| span(1).flat_map(move |y| span(2).map(move |z| [x, y, z])))
            .filter_map(|k| self.cells.get(&k))
            .flatten()
    }

    /// The cells of a triangle with box `bx` (none when it is too large for the grid).
    fn keys(&self, bx: &Aabb) -> Vec<[i64; 3]> {
        if self.too_large(bx) {
            return Vec::new();
        }
        let (lo, hi) = self.span(bx);
        let mut out = Vec::new();
        for x in lo[0]..=hi[0] {
            for y in lo[1]..=hi[1] {
                for z in lo[2]..=hi[2] {
                    out.push([x, y, z]);
                }
            }
        }
        out
    }

    /// Records the new triangle `t` (once, with its cells or in the list of large ones).
    fn add(&mut self, t: [u32; 3], c: &[Point3; 3]) {
        let bx = Aabb::from_points(c.iter());
        let i = self.added.len() as u32;
        self.added.push(Some(t));
        self.index_of.insert(t, i);
        if self.too_large(&bx) {
            self.large.push(i);
        }
        for k in self.keys(&bx) {
            self.cells.entry(k).or_default().push(i);
        }
    }

    /// Takes back the record of the new triangle `t`.
    pub(super) fn forget(&mut self, p: Pts<'_>, t: [u32; 3]) {
        let Some(i) = self.index_of.remove(&t) else { return };
        if let Some(slot) = self.added.get_mut(i as usize) {
            *slot = None;
        }
        self.large.retain(|&x| x != i);
        let Some(c) = p.corners(&t) else { return };
        for k in self.keys(&Aabb::from_points(c.iter())) {
            if let Some(list) = self.cells.get_mut(&k) {
                list.retain(|&x| x != i);
            }
        }
    }

    /// Collections of new triangles the crossing tests made.
    #[cfg(test)]
    pub(super) fn collections(&self) -> usize {
        self.collections.get()
    }

    /// Comparisons the last crossing test made (or would have made).
    #[cfg(test)]
    pub(super) fn compared(&self) -> usize {
        self.compared.get()
    }

    /// Tests a restored or recovered triangle (a triangle of a scan, at exactly its place, so on
    /// scanned surface by construction) for crossings and covers only, and records it when it passes.
    pub(super) fn restorable(&mut self, p: Pts<'_>, t: [u32; 3]) -> bool {
        let Some(c) = p.corners(&t) else { return false };
        let clash = self.crosses(p, t, &c, true);
        if !self.passed(clash) {
            return false;
        }
        self.add(t, &c);
        true
    }

    /// Tests a triangle built for the seam or a hole (corner rule, support, crossings) and records it
    /// when it passes.
    pub(super) fn accept(&mut self, p: Pts<'_>, t: [u32; 3]) -> bool {
        let Some(c) = p.corners(&t) else { return false };
        if self.corners.as_ref().is_some_and(|r| !r.allows(p, t)) {
            self.bad_corners += 1;
            return false;
        }
        if !self.supported(p, t, &c) {
            self.unsupported += 1;
            return false;
        }
        let clash = self.crosses(p, t, &c, false);
        if !self.passed(clash) {
            return false;
        }
        self.add(t, &c);
        true
    }

    /// Tests the built triangles `patch` together and records them only when all pass; the counts of a
    /// refused try are not kept (it is one of several).
    pub(super) fn accept_all(&mut self, p: Pts<'_>, patch: &[[u32; 3]]) -> bool {
        let mark = (self.unsupported, self.crossing, self.bad_corners, self.over_budget);
        let mut added: Vec<[u32; 3]> = Vec::with_capacity(patch.len());
        for &t in patch {
            if !self.accept(p, t) {
                for a in added {
                    self.forget(p, a);
                }
                // a try refused for the budget is noted: the loop it was for may stay open unjudged
                self.patch_over_budget |= self.over_budget > mark.3;
                (self.unsupported, self.crossing, self.bad_corners, self.over_budget) = mark;
                return false;
            }
            added.push(t);
        }
        true
    }
}

/// Do triangles `x` and `y` (corners and vertices) clash: cross (an edge through the other, also for
/// two sharing one vertex: the edge opposite it), or lie over each other (a corner of one inside the
/// other within `slab` of its plane; not tested without a `slab`)?
fn clashes((cx, ix): (&[Point3; 3], &[u32; 3]), (cy, iy): (&[Point3; 3], &[u32; 3]), slab: Option<f64>) -> bool {
    let shared: Vec<u32> = ix.iter().copied().filter(|v| iy.contains(v)).collect();
    let over = slab.is_some_and(|slab| covers(cx, cy, iy, ix, slab) || covers(cy, cx, ix, iy, slab));
    match shared.as_slice() {
        [] => over || nesting::crosses(cx, cy),
        [v] => over || opposite_pierces((cx, ix), cy, *v) || opposite_pierces((cy, iy), cx, *v),
        _ => over,
    }
}

/// Does the edge of triangle `t` opposite its vertex `v` pass through triangle `other` (not at its ends)?
fn opposite_pierces((c, ids): (&[Point3; 3], &[u32; 3]), other: &[Point3; 3], v: u32) -> bool {
    let Some(k) = ids.iter().position(|&w| w == v) else { return false };
    let (p, q) = (c[(k + 1) % 3], c[(k + 2) % 3]);
    tri::ray_hit(&p, &(q - p), &other[0], &other[1], &other[2])
        .is_some_and(|s| s > TOUCH_SHARE && s < 1.0 - TOUCH_SHARE)
}

/// Does triangle `outer` (vertices `oi`) cover a corner of `inner` (vertices `ii`) that it does not
/// share: the corner lies inside it seen along its normal and within `slab` of its plane? Two
/// triangles lying over each other cross no edge when they are nearly in one plane; this tells.
fn covers(outer: &[Point3; 3], inner: &[Point3; 3], ii: &[u32; 3], oi: &[u32; 3], slab: f64) -> bool {
    let Some(n) = tri::normal(&outer[0], &outer[1], &outer[2]) else { return false };
    inner.iter().zip(ii).filter(|(_, v)| !oi.contains(v)).any(|(p, _)| {
        let h = (p - outer[0]).dot(&n);
        if h.abs() > slab {
            return false;
        }
        let w = super::prep::barycentric_open(outer, &(p - n * h));
        w.iter().all(|&x| x > INSIDE_MARGIN)
    })
}
