//! Where the offset would fold, the offset distance is reduced locally.
//!
//! Offsetting a surface by `d` along its normals folds it wherever the radius of curvature on the
//! side of the offset is smaller than `d` (the offset surface of a sphere of radius `r` has
//! radius `r − d` and turns inside out for `d > r`; Farouki, "The approximation of non-degenerate
//! offset surfaces", CAGD 3(1), 1986). A fold shows in the triangles first: an offset triangle
//! shrinks to a fraction of the area of the original or turns its normal away from it. Every vertex
//! of such a triangle has its offset multiplied by [`SHRINK`], and the triangles around it are
//! tested again, until none is left. The reduction is tapered into the neighbourhood (the offset
//! distance changes by at most [`MAX_OFFSET_SLOPE`] times the distance between vertices), so that
//! the relaxed vertices do not fold their unrelaxed neighbours in turn. Only triangles around the vertices changed are re-tested, so the
//! cost stays near one pass over the mesh.
use super::topology::Csr;
use crate::{Aabb, Bvh, Mesh, Point3, Vec3, nesting, par};

/// A triangle whose offset copy keeps less than this share of its area is folded.
pub(super) const MIN_AREA_RATIO: f64 = 0.15;
/// A triangle whose offset copy has a normal with a smaller cosine to the original's is folded.
pub(super) const MIN_NORMAL_COS: f64 = 0.3;
/// Triangles thinner than this (twice the area over the longest edge squared; 0.87 for an
/// equilateral one) are left out of the test: their shape changes with any offset.
const MIN_QUALITY: f64 = 0.05;
/// Factor a flagged vertex's offset is multiplied by in every round.
const SHRINK: f64 = 0.8;
/// Largest change of the offset distance per distance between neighbouring vertices.
const MAX_OFFSET_SLOPE: f64 = 0.5;
/// The offset of a vertex is never reduced below this share of the thickness: a fold that cannot be
/// resolved above it is refused (below it the two sheets would touch).
pub(super) const MIN_SCALE: f64 = 0.1;
/// Crossing triangles whose vertices are within this many thicknesses of each other along the surface
/// count as a fold of the offset (the flanks of a crease); farther ones are left to the report.
const LOCAL_REACH: f64 = 2.0;
/// ...and so do those within this many mean edges (on a coarse mesh two triangles that share no
/// vertex are already an edge or two apart).
const LOCAL_RINGS: f64 = 3.0;
/// Rounds of the exact crossing test around the reduced vertices at most.
const MAX_CROSSING_ROUNDS: usize = 12;
/// Rounds at most; `SHRINK^ROUNDS` is the smallest share of the offset a vertex can keep.
const MAX_ROUNDS: usize = 60;

/// What the test reads.
pub(super) struct Field<'a> {
    pub positions: &'a [Point3],
    pub normals: &'a [Vec3],
    pub triangles: &'a [[u32; 3]],
    /// Offsets (mm along the normals) of the layers that move; `[]` entries of zero are left out.
    pub offsets: &'a [f64],
    /// Offsets of both layers `[a, b]` (one of them may be 0).
    pub layers: [f64; 2],
    /// Triangles around every vertex.
    pub around: &'a Csr,
    /// Neighbours of every vertex.
    pub adjacent: &'a Csr,
    /// The thickness (mm): the offsets are `scale` times it.
    pub thickness: f64,
    /// Mean edge length of the mesh (mm).
    pub edge: f64,
    /// Boundary edges (the strip along them joins the two layers).
    pub boundary: &'a [(u32, u32)],
}

impl Field<'_> {
    /// Do triangles `x` and `y` of the layers mesh (`n` vertices per layer) lie within [`LOCAL_REACH`]
    /// thicknesses of each other along the scan? Crossings farther apart along the surface are real
    /// geometry (a channel narrower than the wall), not a fold of the offset, and are left to the sample
    /// that reports them; the two flanks of a ridge are close along the surface and count.
    fn near_along_the_surface(&self, x: &[u32; 3], y: &[u32; 3], n: u32) -> bool {
        let origin = |v: u32| v % n.max(1);
        let reach = (LOCAL_REACH * self.thickness).max(LOCAL_RINGS * self.edge);
        let mut best: std::collections::HashMap<u32, f64> = x.iter().map(|&v| (origin(v), 0.0)).collect();
        let mut queue: std::collections::VecDeque<u32> = best.keys().copied().collect();
        while let Some(v) = queue.pop_front() {
            let (Some(&dv), Some(pv)) = (best.get(&v), self.positions.get(v as usize)) else { continue };
            for &w in self.adjacent.row(v as usize) {
                let Some(pw) = self.positions.get(w as usize) else { continue };
                let d = dv + (pw - pv).norm();
                if d <= reach && best.get(&w).is_none_or(|&old| d < old - 1e-12) {
                    best.insert(w, d);
                    queue.push_back(w);
                }
            }
        }
        y.iter().any(|&v| best.contains_key(&origin(v)))
    }

    fn corner(&self, v: u32, scale: &[f64], offset: f64) -> Option<Point3> {
        let p = self.positions.get(v as usize)?;
        let n = self.normals.get(v as usize)?;
        Some(p + n * (offset * scale.get(v as usize)?))
    }

    /// Does the offset copy of triangle `ti` fold?
    fn folded(&self, ti: u32, scale: &[f64]) -> bool {
        let Some(t) = self.triangles.get(ti as usize) else { return false };
        let (Some(a), Some(b), Some(c)) =
            (self.positions.get(t[0] as usize), self.positions.get(t[1] as usize), self.positions.get(t[2] as usize))
        else {
            return false;
        };
        let reference = (b - a).cross(&(c - a));
        let area = reference.norm();
        let longest2 =
            [(b - a).norm_squared(), (c - b).norm_squared(), (a - c).norm_squared()].into_iter().fold(0.0, f64::max);
        if !(area > MIN_QUALITY * longest2) {
            return false;
        }
        self.offsets.iter().any(|&o| {
            let moved = (self.corner(t[0], scale, o), self.corner(t[1], scale, o), self.corner(t[2], scale, o));
            let (Some(p), Some(q), Some(r)) = moved else { return true };
            let now = (q - p).cross(&(r - p));
            let size = now.norm();
            !(size.is_finite() && size >= MIN_AREA_RATIO * area && now.dot(&reference) >= MIN_NORMAL_COS * size * area)
        })
    }
}

/// The vertices of the triangles around `dirty` that fold, ascending.
fn flagged(field: &Field, scale: &[f64], dirty: &[u32]) -> Vec<u32> {
    let mut tris: Vec<u32> = dirty.iter().flat_map(|&v| field.around.row(v as usize).iter().copied()).collect();
    tris.sort_unstable();
    tris.dedup();
    let bad: Vec<bool> = par::map(tris.len(), |i| tris.get(i).is_some_and(|&ti| field.folded(ti, scale)));
    let mut out: Vec<u32> = Vec::new();
    for (&ti, _) in tris.iter().zip(&bad).filter(|&(_, &b)| b) {
        if let Some(t) = field.triangles.get(ti as usize) {
            out.extend_from_slice(t);
        }
    }
    out.sort_unstable();
    out.dedup();
    out
}

/// Lowers the scale of the neighbours of `changed` so that it differs by at most the slope bound
/// per edge; returns every vertex whose scale changed (`changed` included), ascending.
fn taper(field: &Field, scale: &mut [f64], changed: &[u32]) -> Vec<u32> {
    let per_mm = MAX_OFFSET_SLOPE / field.thickness;
    let mut queue: std::collections::VecDeque<u32> = changed.iter().copied().collect();
    let mut all: Vec<u32> = changed.to_vec();
    while let Some(i) = queue.pop_front() {
        let Some(&si) = scale.get(i as usize) else { continue };
        let Some(pi) = field.positions.get(i as usize) else { continue };
        for &j in field.adjacent.row(i as usize) {
            let Some(pj) = field.positions.get(j as usize) else { continue };
            let bound = si + per_mm * (pj - pi).norm();
            if let Some(sj) = scale.get_mut(j as usize)
                && bound < *sj - 1e-12
            {
                *sj = bound;
                queue.push_back(j);
                all.push(j);
            }
        }
    }
    all.sort_unstable();
    all.dedup();
    all
}

/// The outcome of the relaxation.
pub(super) struct Relaxed {
    /// The share of the offset every vertex keeps (1 where nothing folds and nothing was tapered).
    pub scale: Vec<f64>,
    /// Vertices of triangles that folded or crossed another in some round (the others were only tapered).
    pub folded: usize,
    /// Vertices that still lie on a folded or crossing triangle at the floor [`MIN_SCALE`] (sharp
    /// creases on the side of the offset: no wall of that thickness fits there).
    pub unresolved: usize,
}

/// What the rounds change.
struct State {
    scale: Vec<f64>,
    ever: Vec<bool>,
    frozen: Vec<bool>,
}

impl State {
    /// Splits `bad` into the vertices that can still be reduced (returned) and those already at the
    /// floor (frozen), and records that they all folded.
    fn partition(&mut self, bad: Vec<u32>) -> Vec<u32> {
        let mut movable = Vec::new();
        for v in bad {
            let at_floor = self.scale.get(v as usize).is_some_and(|&s| s <= MIN_SCALE * 1.000_001);
            if let (Some(f), Some(e)) = (self.frozen.get_mut(v as usize), self.ever.get_mut(v as usize)) {
                *e = true;
                if at_floor {
                    *f = true;
                } else {
                    movable.push(v);
                }
            }
        }
        movable
    }

    /// Reduces `movable` and tapers the reduction; returns the vertices changed.
    fn shrink(&mut self, field: &Field, movable: &[u32]) -> Vec<u32> {
        for &v in movable {
            if let Some(s) = self.scale.get_mut(v as usize) {
                *s = (*s * SHRINK).max(MIN_SCALE);
            }
        }
        taper(field, &mut self.scale, movable)
    }
}

/// Rounds of the per-triangle test from the vertices `dirty`; the number of vertices that still fold
/// when [`MAX_ROUNDS`] could not resolve them is the error.
fn fold_rounds(field: &Field, st: &mut State, mut dirty: Vec<u32>) -> Result<(), usize> {
    for _ in 0..MAX_ROUNDS {
        let movable = {
            let bad = flagged(field, &st.scale, &dirty);
            st.partition(bad)
        };
        if movable.is_empty() {
            return Ok(());
        }
        dirty = st.shrink(field, &movable);
    }
    let left = flagged(field, &st.scale, &dirty)
        .into_iter()
        .filter(|&v| !st.frozen.get(v as usize).copied().unwrap_or(true))
        .count();
    if left == 0 { Ok(()) } else { Err(left) }
}

/// The offset layers `[a, b]` as one mesh (the first `n` vertices and `T` triangles the layer at `a`).
fn layers_mesh(field: &Field, scale: &[f64]) -> Mesh {
    let n = field.positions.len() as u32;
    let mut positions = Vec::with_capacity(2 * field.positions.len());
    for &o in &field.layers {
        positions.extend((0..n).map(|v| field.corner(v, scale, o).unwrap_or_else(Point3::origin)));
    }
    let mut triangles = field.triangles.to_vec();
    triangles.extend(field.triangles.iter().map(|t| t.map(|v| v + n)));
    // the strip along the rim, one row: it can cut through a layer next to a bend of the rim
    for &(u, v) in field.boundary {
        triangles.push([u, v, v + n]);
        triangles.push([u, v + n, u + n]);
    }
    Mesh { positions, triangles }
}

/// The vertices (as vertices of the scan, ascending) of the triangles around a reduced vertex that cross
/// a triangle of either layer which shares none of their vertices. Exact, but only around the vertices
/// whose offset was reduced: the per-triangle test cannot see a crossing between triangles that are not
/// neighbours (two rings apart at a crease).
fn crossing_vertices(field: &Field, scale: &[f64]) -> Vec<u32> {
    let n = field.positions.len() as u32;
    let mesh = layers_mesh(field, scale);
    let bvh = Bvh::build(&mesh);
    let reduced: Vec<u32> = (0..n).filter(|&v| scale.get(v as usize).is_some_and(|&s| s < 1.0)).collect();
    let mut tris: Vec<u32> = reduced.iter().flat_map(|&v| field.around.row(v as usize).iter().copied()).collect();
    tris.sort_unstable();
    tris.dedup();
    let tcount = field.triangles.len() as u32;
    let strip: Vec<u32> = (0..2 * field.boundary.len() as u32).map(|k| 2 * tcount + k).collect();
    let origin = |v: u32| v % n.max(1);
    let hits: Vec<Vec<u32>> = par::map(tris.len() * 2 + strip.len(), |k| {
        let id = match k {
            k if k < tris.len() * 2 => tris.get(k / 2).copied().unwrap_or(0) + (k % 2) as u32 * tcount,
            k => strip.get(k - tris.len() * 2).copied().unwrap_or(0),
        };
        let (Some(x), Some(xi)) = (mesh.triangle(id as usize), mesh.triangles.get(id as usize)) else {
            return Vec::new();
        };
        let bx = Aabb::from_points(x.iter());
        let near = bvh.candidates(&|b: &Aabb| (0..3).all(|a| b.min[a] <= bx.max[a] && bx.min[a] <= b.max[a]));
        for o in near {
            let (Some(y), Some(yi)) = (mesh.triangle(o as usize), mesh.triangles.get(o as usize)) else { continue };
            let shares = xi.iter().any(|v| yi.iter().any(|w| origin(*v) == origin(*w)));
            if o != id && !shares && nesting::crosses(&x, &y) && field.near_along_the_surface(xi, yi, n) {
                return xi.iter().chain(yi.iter()).map(|&v| origin(v)).collect();
            }
        }
        Vec::new()
    });
    let mut out: Vec<u32> = hits.into_iter().flatten().collect();
    out.sort_unstable();
    out.dedup();
    out
}

/// Relaxes the offset, or returns the number of vertices that still fold when [`MAX_ROUNDS`] rounds
/// could not resolve the others. After the per-triangle rounds, the neighbourhood of every reduced
/// vertex is tested exactly for triangles that cross; their vertices count as folded and are reduced,
/// for at most [`MAX_CROSSING_ROUNDS`] rounds.
pub(super) fn relax(field: &Field) -> Result<Relaxed, usize> {
    let n = field.positions.len();
    let mut st = State { scale: vec![1.0; n], ever: vec![false; n], frozen: vec![false; n] };
    fold_rounds(field, &mut st, (0..n as u32).collect())?;
    for _ in 0..MAX_CROSSING_ROUNDS {
        let movable = {
            let bad = crossing_vertices(field, &st.scale);
            st.partition(bad)
        };
        if movable.is_empty() {
            break;
        }
        let changed = st.shrink(field, &movable);
        fold_rounds(field, &mut st, changed)?;
    }
    Ok(Relaxed {
        folded: st.ever.iter().filter(|&&e| e).count(),
        unresolved: st.frozen.iter().filter(|&&f| f).count(),
        scale: st.scale,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fold_that_survives_the_floor_is_counted_as_unresolved() {
        // Arrange: a triangle whose normals point up, up and down: with an offset this large it stands
        // on its edge at every scale down to the floor
        let positions = [Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)];
        let normals = [Vec3::z(), Vec3::z(), -Vec3::z()];
        let triangles = [[0u32, 1, 2]];
        let around = Csr::from_pairs(3, &[(0, 0), (1, 0), (2, 0)]);
        let adjacent = Csr::from_pairs(3, &[(0, 1), (0, 2), (1, 0), (1, 2), (2, 0), (2, 1)]);
        let offsets = [-1000.0];
        let field = Field {
            positions: &positions,
            normals: &normals,
            triangles: &triangles,
            offsets: &offsets,
            layers: [offsets[0], 0.0],
            around: &around,
            adjacent: &adjacent,
            thickness: 1000.0,
            edge: 1.0,
            boundary: &[],
        };

        // Act
        let result = relax(&field);

        // Assert: they stay at the floor and are counted
        let relaxed = result.expect("relaxed");
        assert_eq!(relaxed.unresolved, 3);
        assert!(relaxed.scale.iter().all(|&s| (s - MIN_SCALE).abs() < 1e-9));
    }
}
