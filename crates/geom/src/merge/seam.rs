//! Closing the seam: ball pivoting on the band between the kept regions ([`crate::recon::band`]),
//! from the edges the removal cut and only through their vertices. A triangle is built only when
//! it lies on surface some scan saw (its centre and the middles of its edges within the tolerance of
//! facing surface of the scans as given: no surface is made up over an opening nobody scanned) and
//! crosses no triangle there already.
use super::overlap::COS_FACING;
use super::prep::Scan;
use super::union::{Origin, Union};
use crate::recon::band::{BandInput, pivot_band};
use crate::{Aabb, Bvh, Cancel, Mesh, Point3, Vec3, nesting};
use std::collections::HashMap;

/// Ball radii of the passes, in median lengths of the front edges.
pub const RADII_EDGES: [f64; 3] = [1.0, 2.0, 4.0];
/// A seam triangle may lie this share of its longest edge (or the tolerance, if larger) from the scans.
pub const SUPPORT_SLOPE: f64 = 0.1;
/// Rounds of taking back the seam triangles at a pinched vertex and filling the holes again.
const UNPINCH_ROUNDS: usize = 4;
/// Triangles near a new one tested for a crossing at most (the rest is taken as crossing).
const MAX_NEAR: usize = 1 << 16;

/// What the seam step did.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SeamReport {
    /// Border edges the removal cut (both sides of the seam).
    pub front_edges: usize,
    /// Their length (mm).
    pub front_length: f64,
    /// Median length of those edges (mm).
    pub front_median: f64,
    /// Ball radii used (mm).
    pub radii: Vec<f64>,
    /// Removed triangles restored because the union had all their corners at exactly their places.
    pub restored: usize,
    /// Triangles built to close the seam by pivoting.
    pub built: usize,
    /// Small holes left by the pivoting that were filled, and the triangles that took.
    pub filled_loops: usize,
    pub fill_triangles: usize,
    /// Seam triangles taken back because they met another fan at a vertex only (a pinch).
    pub unpinched: usize,
    /// Candidate triangles refused because no scan saw surface there.
    pub unsupported: usize,
    /// Candidate triangles refused because they crossed a triangle there already.
    pub crossing: usize,
    /// Triangles of small pieces dropped because they lay wholly on the rest of the result.
    pub islands_dropped: usize,
    /// Front edges left open (no triangle on their other side) after the seam step.
    pub open_edges: usize,
    /// Their length (mm).
    pub open_length: f64,
}

/// The tests every new triangle (restored or built) passes, and the new triangles so far.
pub(super) struct Guard<'a> {
    scans: &'a [Scan],
    tol: f64,
    /// BVH over the kept triangles of the union.
    kept: Bvh,
    kept_tris: Vec<[u32; 3]>,
    /// Normal of every union vertex (from its scan).
    normals: Vec<Vec3>,
    /// New triangles by cell of a uniform grid.
    cells: HashMap<[i64; 3], Vec<[u32; 3]>>,
    cell: f64,
    pub(super) unsupported: usize,
    pub(super) crossing: usize,
}

fn cell_of(p: &Point3, cell: f64) -> [i64; 3] {
    [p.x, p.y, p.z].map(|c| (c / cell).floor() as i64)
}

impl<'a> Guard<'a> {
    pub(super) fn new(scans: &'a [Scan], u: &Union, tol: f64, cell: f64) -> Self {
        let kept = Mesh { positions: u.positions.clone(), triangles: u.triangles.clone() };
        let cell = if cell.is_finite() && cell > 0.0 { cell } else { 1.0 };
        Guard {
            scans,
            tol,
            kept: Bvh::build(&kept),
            kept_tris: kept.triangles,
            normals: u.normals.clone(),
            cells: HashMap::new(),
            cell,
            unsupported: 0,
            crossing: 0,
        }
    }

    /// Does triangle `t` (corners `c`) face the way its corners' normals do, and does every sample of
    /// it (centre, middles of the edges) lie on surface of a scan that faces that way too? (The
    /// corners' normals, not the triangle's: a sliver's own normal says little.)
    fn supported(&self, t: [u32; 3], c: &[Point3; 3]) -> bool {
        let sum = t.iter().filter_map(|&v| self.normals.get(v as usize)).fold(Vec3::zeros(), |s, n| s + n);
        let (Some(n), Some(own)) = (crate::normalize(sum), crate::tri::normal(&c[0], &c[1], &c[2])) else {
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

    /// Does triangle `t` (corners `c`) cross a kept or a new triangle it shares no vertex with?
    fn crosses(&self, pts: &[Point3], t: [u32; 3], c: &[Point3; 3]) -> bool {
        let bx = Aabb::from_points(c.iter());
        let corners = |x: &[u32; 3]| -> Option<[Point3; 3]> {
            Some([*pts.get(x[0] as usize)?, *pts.get(x[1] as usize)?, *pts.get(x[2] as usize)?])
        };
        let shares = |x: &[u32; 3]| x.iter().any(|v| t.contains(v));
        let (near, more) = self.kept.overlapping_up_to(&bx, MAX_NEAR, &|_| true);
        if more {
            return true;
        }
        let slab = self.tol;
        let clash = |x: &[u32; 3]| {
            corners(x).is_some_and(|y| {
                if shares(x) {
                    return covers(c, &y, x, &t, slab) || covers(&y, c, &t, x, slab);
                }
                nesting::crosses(c, &y) || covers(c, &y, x, &t, slab) || covers(&y, c, &t, x, slab)
            })
        };
        near.iter().filter_map(|&i| self.kept_tris.get(i as usize)).any(clash) || self.cells_of(&bx).any(clash)
    }

    /// The new triangles in the cells the box `bx` touches.
    fn cells_of<'s>(&'s self, bx: &Aabb) -> impl Iterator<Item = &'s [u32; 3]> + 's {
        let (lo, hi) = (cell_of(&bx.min, self.cell), cell_of(&bx.max, self.cell));
        let span = move |a: usize| lo[a]..=hi[a].min(lo[a].saturating_add(8));
        span(0)
            .flat_map(move |x| span(1).flat_map(move |y| span(2).map(move |z| [x, y, z])))
            .filter_map(|k| self.cells.get(&k))
            .flatten()
    }

    /// Records the new triangle `t`.
    fn add(&mut self, t: [u32; 3], c: &[Point3; 3]) {
        let bx = Aabb::from_points(c.iter());
        let (lo, hi) = (cell_of(&bx.min, self.cell), cell_of(&bx.max, self.cell));
        for x in lo[0]..=hi[0].min(lo[0].saturating_add(8)) {
            for y in lo[1]..=hi[1].min(lo[1].saturating_add(8)) {
                for z in lo[2]..=hi[2].min(lo[2].saturating_add(8)) {
                    self.cells.entry([x, y, z]).or_default().push(t);
                }
            }
        }
    }

    /// Tests the new triangles `p` together (support, crossings among them and with what is there)
    /// and records them only when all pass.
    pub(super) fn accept_all(&mut self, pts: &[Point3], p: &[[u32; 3]]) -> bool {
        let mark = (self.unsupported, self.crossing);
        let mut added: Vec<[u32; 3]> = Vec::with_capacity(p.len());
        for &t in p {
            if !self.accept(pts, t) {
                for a in added {
                    self.forget(pts, a);
                }
                // a refused fan is one of several tries: only the counts of the pivoting are kept
                (self.unsupported, self.crossing) = mark;
                return false;
            }
            added.push(t);
        }
        true
    }

    /// Takes back the record of the new triangle `t`.
    pub(super) fn forget(&mut self, pts: &[Point3], t: [u32; 3]) {
        let c = t.map(|v| pts.get(v as usize).copied().unwrap_or_else(Point3::origin));
        let bx = Aabb::from_points(c.iter());
        let (lo, hi) = (cell_of(&bx.min, self.cell), cell_of(&bx.max, self.cell));
        for x in lo[0]..=hi[0].min(lo[0].saturating_add(8)) {
            for y in lo[1]..=hi[1].min(lo[1].saturating_add(8)) {
                for z in lo[2]..=hi[2].min(lo[2].saturating_add(8)) {
                    if let Some(list) = self.cells.get_mut(&[x, y, z]) {
                        list.retain(|x| *x != t);
                    }
                }
            }
        }
    }

    /// Records a restored triangle (a triangle of a scan, at exactly its place) without tests: later
    /// seam triangles are tested against it.
    pub(super) fn record(&mut self, pts: &[Point3], t: [u32; 3]) -> bool {
        let c = t.map(|v| pts.get(v as usize).copied().unwrap_or_else(Point3::origin));
        self.add(t, &c);
        true
    }

    /// Tests the new triangle `t` (support, crossings) and records it when it passes.
    pub(super) fn accept(&mut self, pts: &[Point3], t: [u32; 3]) -> bool {
        let c = t.map(|v| pts.get(v as usize).copied());
        let [Some(a), Some(b), Some(d)] = c else { return false };
        let c = [a, b, d];
        if !self.supported(t, &c) {
            self.unsupported += 1;
            return false;
        }
        if self.crosses(pts, t, &c) {
            self.crossing += 1;
            return false;
        }
        self.add(t, &c);
        true
    }
}

/// Does triangle `outer` (vertices `oi`) cover a corner of `inner` (vertices `ii`) that it does not
/// share: the corner lies inside it seen along its normal and within `slab` of its plane? Two
/// triangles lying over each other cross no edge when they are nearly in one plane; this tells.
fn covers(outer: &[Point3; 3], inner: &[Point3; 3], ii: &[u32; 3], oi: &[u32; 3], slab: f64) -> bool {
    let Some(n) = crate::tri::normal(&outer[0], &outer[1], &outer[2]) else { return false };
    inner.iter().zip(ii).filter(|(_, v)| !oi.contains(v)).any(|(p, _)| {
        let h = (p - outer[0]).dot(&n);
        if h.abs() > slab {
            return false;
        }
        let q = p - n * h;
        let w = super::prep::barycentric_open(outer, &q);
        w.iter().all(|&x| x > INSIDE_MARGIN)
    })
}

/// A corner closer to an edge than this barycentric share is not inside (touching is no cover).
const INSIDE_MARGIN: f64 = 1e-3;

/// Median of the lengths (0 for none).
fn median(mut l: Vec<f64>) -> f64 {
    if l.is_empty() {
        return 0.0;
    }
    let k = l.len() / 2;
    *l.select_nth_unstable_by(k, f64::total_cmp).1
}

fn edge_length(u: &Union, f: &[u32; 3]) -> f64 {
    match (u.positions.get(f[0] as usize), u.positions.get(f[1] as usize)) {
        (Some(p), Some(q)) => (p - q).norm(),
        _ => 0.0,
    }
}

/// Closes the seam of `u` with the guard `guard`; `None` when cancelled.
pub(super) fn close(
    u: &mut Union,
    scans: &[Scan],
    guard: &mut Guard<'_>,
    rep: &mut SeamReport,
    cancel: &Cancel,
) -> Option<()> {
    let front = u.front(scans);
    let lengths: Vec<f64> = front.iter().map(|f| edge_length(u, f)).collect();
    rep.front_edges = front.len();
    rep.front_length = lengths.iter().fold(0.0, |s, l| s + l);
    rep.front_median = median(lengths);
    rep.radii = RADII_EDGES.iter().map(|k| k * rep.front_median).collect();
    if front.is_empty() || !(rep.front_median > 0.0) {
        return Some(());
    }
    let rule = Corners::new(u, scans, &front);
    let pts = &u.positions;
    let mut accept = |t: [u32; 3]| rule.allows(u, t) && guard.accept(pts, t);
    let input = BandInput { points: &u.positions, normals: &u.normals, triangles: &u.triangles, front: &front };
    let built = pivot_band(&input, &rep.radii, &mut accept, cancel)?;
    rep.built = built.len();
    u.origin.extend(std::iter::repeat_n(Origin::Seam, built.len()));
    u.triangles.extend(built);
    for _ in 0..UNPINCH_ROUNDS {
        let (loops, tris) = super::fill::fill_small(u, scans, guard);
        rep.filled_loops += loops;
        rep.fill_triangles += tris;
        let undone = super::fill::unpinch(u, guard);
        rep.unpinched += undone;
        if undone == 0 {
            break;
        }
    }
    measure_open(u, scans, rep);
    Some(())
}

/// Which corners a seam triangle may have. Every vertex of an edge the removal cut may be used. A
/// vertex of a scan's own border (a rim) may be used where the seam runs into it, but a triangle
/// has at least one cut vertex, two rim vertices only when they are the two ends of one rim edge
/// (the seam closes that edge; it never bridges two rims), and a rim vertex only where its normal
/// faces like the triangle (a boss standing on a face, a sphere sitting in a hole: their rims are
/// not joined to the face around them).
struct Corners {
    cut: Vec<bool>,
    rims: std::collections::HashSet<u64>,
}

fn undirected(a: u32, b: u32) -> u64 {
    super::union::directed(a.min(b), a.max(b))
}

impl Corners {
    fn new(u: &Union, scans: &[Scan], front: &[[u32; 3]]) -> Self {
        let mut cut = vec![false; u.positions.len()];
        for f in front {
            for v in [f[0], f[1]] {
                if let Some(b) = cut.get_mut(v as usize) {
                    *b = true;
                }
            }
        }
        let edges = u.directed_edges();
        let mut rims = std::collections::HashSet::new();
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

    fn allows(&self, u: &Union, t: [u32; 3]) -> bool {
        let rim: Vec<u32> =
            t.iter().copied().filter(|&v| !self.cut.get(v as usize).copied().unwrap_or(false)).collect();
        let joined = match rim.as_slice() {
            [] | [_] => true,
            [a, b] => self.rims.contains(&undirected(*a, *b)),
            _ => false,
        };
        if !joined {
            return false;
        }
        let c = t.map(|v| u.positions.get(v as usize).copied().unwrap_or_else(Point3::origin));
        let Some(n) = crate::tri::normal(&c[0], &c[1], &c[2]) else { return false };
        rim.iter().all(|&v| u.normals.get(v as usize).is_some_and(|m| m.dot(&n) >= COS_FACING))
    }
}

/// The open seam edges of `u` and their length.
pub(super) fn measure_open(u: &Union, scans: &[Scan], rep: &mut SeamReport) {
    let open = u.front(scans);
    rep.open_edges = open.len();
    rep.open_length = open.iter().fold(0.0, |s, f| s + edge_length(u, f));
}
