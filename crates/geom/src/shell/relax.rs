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
use crate::{Point3, Vec3, par};

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
/// Rounds at most; `SHRINK^ROUNDS` is the smallest share of the offset a vertex can keep.
const MAX_ROUNDS: usize = 60;

/// What the test reads.
pub(super) struct Field<'a> {
    pub positions: &'a [Point3],
    pub normals: &'a [Vec3],
    pub triangles: &'a [[u32; 3]],
    /// Offsets (mm along the normals) of the layers that move; `[]` entries of zero are left out.
    pub offsets: &'a [f64],
    /// Triangles around every vertex.
    pub around: &'a Csr,
    /// Neighbours of every vertex.
    pub adjacent: &'a Csr,
    /// The thickness (mm): the offsets are `scale` times it.
    pub thickness: f64,
}

impl Field<'_> {
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

/// The share of the offset every vertex keeps (1 where nothing folds), or the number of vertices
/// that still fold when [`MAX_ROUNDS`] rounds could not resolve them.
pub(super) fn relax(field: &Field) -> Result<Vec<f64>, usize> {
    let n = field.positions.len();
    let mut scale = vec![1.0; n];
    let mut dirty: Vec<u32> = (0..n as u32).collect();
    for _ in 0..MAX_ROUNDS {
        let bad = flagged(field, &scale, &dirty);
        if bad.is_empty() {
            return Ok(scale);
        }
        for &v in &bad {
            if let Some(s) = scale.get_mut(v as usize) {
                *s *= SHRINK;
            }
        }
        dirty = taper(field, &mut scale, &bad);
    }
    let left = flagged(field, &scale, &dirty).len();
    if left == 0 { Ok(scale) } else { Err(left) }
}
