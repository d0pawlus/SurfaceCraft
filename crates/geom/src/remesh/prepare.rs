//! The scan made ready for the solid: duplicate triangles dropped, small holes filled, debris
//! dropped, open boundaries refused.
use super::{Params, RemeshError};
use crate::{Mesh, edit, pieces};
use std::collections::HashSet;

/// Vertices closer than this (mm) are one, so that a coincident second shell shares its triangles.
const WELD_MM: f64 = 1e-6;
/// A boundary loop of this many edges or fewer is a flap (the two free edges of a fin are not a hole):
/// rays through it lose nothing worth a refusal. Any longer loop that is left, a missing triangle
/// included, is refused.
const MAX_FLAP_EDGES: usize = 2;

/// The scan prepared for the solid.
pub(super) struct Prepared {
    pub work: Mesh,
    pub holes_filled: usize,
    pub debris_triangles: usize,
    pub duplicate_triangles: usize,
    pub genus: Vec<Option<usize>>,
    pub non_manifold: usize,
}

/// Width of a boundary loop: its perimeter over pi (the diameter of a round hole).
fn loop_width(mesh: &Mesh, lp: &[u32]) -> f64 {
    let perimeter: f64 = (0..lp.len())
        .filter_map(|k| {
            Some(
                (mesh.positions.get(*lp.get(k)? as usize)?
                    - mesh.positions.get(*lp.get((k + 1) % lp.len())? as usize)?)
                .norm(),
            )
        })
        .sum();
    perimeter / std::f64::consts::PI
}

/// Drops every triangle that repeats an earlier one with the same winding (the same three vertices in
/// the same cyclic order): a coincident second shell. A twin with the opposite winding is kept: with
/// its mate it is a closed double-faced sheet, and dropping one would leave an open one.
fn drop_duplicates(mesh: &mut Mesh) -> usize {
    let mut seen: HashSet<[u32; 3]> = HashSet::with_capacity(mesh.triangles.len());
    let before = mesh.triangles.len();
    mesh.triangles.retain(|t| {
        let first = (0..3).min_by_key(|k| t[*k]).unwrap_or(0);
        seen.insert([t[first], t[(first + 1) % 3], t[(first + 2) % 3]])
    });
    before - mesh.triangles.len()
}

pub(super) fn prepare(mesh: &Mesh, p: &Params) -> Result<Prepared, RemeshError> {
    mesh.validate().map_err(|e| RemeshError::Params(e.to_string()))?;
    let mut work = mesh.clone();
    work.weld(WELD_MM);
    work.remove_degenerate(0.0);
    let duplicate_triangles = drop_duplicates(&mut work);
    if work.triangle_count() < 4 {
        return Err(RemeshError::Empty);
    }
    let wide = p.max_hole;
    let accept = move |m: &Mesh, lp: &[u32]| loop_width(m, lp) <= wide;
    let holes_filled = edit::fill_holes_where(&mut work, p.max_hole_edges, &accept).filled;
    let debris_triangles = edit::remove_small_components(&mut work, p.min_component_fraction, 8);
    let ps = pieces::pieces(&work);
    let loops: Vec<_> = ps.iter().flat_map(|q| q.loops.iter()).filter(|l| l.edges > MAX_FLAP_EDGES).collect();
    if let Some(biggest) = loops.iter().max_by(|a, b| a.length.total_cmp(&b.length)) {
        return Err(RemeshError::Open {
            loops: loops.len(),
            largest_edges: biggest.edges,
            length: biggest.length,
            max_hole_edges: p.max_hole_edges,
            max_hole: p.max_hole,
        });
    }
    Ok(Prepared {
        work,
        holes_filled,
        debris_triangles,
        duplicate_triangles,
        genus: ps.iter().map(pieces::Piece::genus).collect(),
        non_manifold: ps.iter().map(|q| q.non_manifold_edges).sum(),
    })
}
