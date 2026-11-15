//! A mesh file read into a reduced mesh, streaming: the file is read in batches (several times)
//! and its triangles are clustered on a grid while they are read
//! (`surfacecraft_geom::lod::SoupClusterer`), so the memory it takes follows the reduced mesh and
//! not the file. This is how a scan above the triangle limit of an import (the browser's, above
//! all) is still imported: reduced to a stated number of triangles.
//!
//! The cell size comes from the area of the surface (a surface of area A sampled at cell size h
//! has about 2 A / h² triangles) and grows until the result is within the target; a first result
//! far below the target is tried once more with a smaller cell.
use super::{StreamOptions, declared, detect, for_each_batch};
use crate::mesh::MeshFormat;
use crate::{IoError, bad};
use std::io::Cursor;
use surfacecraft_geom::lod::SoupClusterer;
use surfacecraft_geom::{Aabb, Mesh, Point3, tri};

/// Most passes over the file with a growing cell.
const ATTEMPTS: usize = 12;
/// Factor on the cell size after a pass that ended above the target.
const GROW: f64 = 1.3;
/// A first result below this share of the target is tried again with a smaller cell.
const REFINE_BELOW: f64 = 0.6;
/// Margin on the smaller cell of that second try, so it is likely to stay within the target.
const REFINE_MARGIN: f64 = 1.08;

/// A mesh reduced while it was read.
#[derive(Clone, Debug, PartialEq)]
pub struct Reduced {
    /// The reduced mesh.
    pub mesh: Mesh,
    /// Triangles in the file.
    pub source_triangles: u64,
    /// Cell size of the clustering grid, in the units of the result; no vertex of the file is
    /// farther than `√3` times it from its vertex in the result.
    pub cell_size: f64,
}

/// Triangles the header of a file declares (binary STL: its count; PLY: its faces, which are
/// polygons, so the triangles can be more). `None` when the file does not say (ASCII STL, OBJ,
/// point lists) or cannot be read.
#[must_use]
pub fn declared_triangles(bytes: &[u8], hint: Option<MeshFormat>) -> Option<u64> {
    let format = detect(&mut Cursor::new(bytes), hint).ok()?;
    declared(bytes, format)
}

struct Survey {
    triangles: u64,
    bbox: Aabb,
    area: f64,
}

fn survey(bytes: &[u8], format: super::StreamFormat, opts: &StreamOptions) -> Result<Survey, IoError> {
    let mut bbox = Aabb::empty();
    let mut area = 0.0;
    let open = || Ok(Cursor::new(bytes));
    let triangles = for_each_batch(&open, format, opts, &mut |batch| {
        for [a, b, c] in batch {
            bbox.grow(a);
            bbox.grow(b);
            bbox.grow(c);
            area += tri::area(a, b, c);
        }
        Ok(())
    })?;
    Ok(Survey { triangles, bbox, area })
}

/// One pass: the clusterer of cell size `h`, or `None` when the result passed `target`.
fn pass(
    bytes: &[u8],
    format: super::StreamFormat,
    opts: &StreamOptions,
    origin: Point3,
    h: f64,
    target: usize,
) -> Result<Option<SoupClusterer>, IoError> {
    let mut c = SoupClusterer::new(origin, h, target);
    let open = || Ok(Cursor::new(bytes));
    let read = for_each_batch(&open, format, opts, &mut |batch| {
        if c.push(batch) { Ok(()) } else { Err(IoError::Limit("the reduction passed its target".into())) }
    });
    match read {
        Ok(_) => Ok(Some(c)),
        Err(_) if c.is_over() => Ok(None),
        Err(e) => Err(e),
    }
}

/// Reads the mesh file `bytes` reduced to at most `target` triangles. `Ok(None)` when the file
/// has no more than `target` triangles (read it the ordinary way); [`IoError::Unsupported`] when
/// the format cannot be streamed (OBJ, ASCII PLY, point lists). `opts.limits` bound the file, not
/// the result.
pub fn read_reduced(
    bytes: &[u8],
    hint: Option<MeshFormat>,
    opts: &StreamOptions,
    target: usize,
) -> Result<Option<Reduced>, IoError> {
    if !(opts.scale.is_finite() && opts.scale > 0.0) {
        return Err(bad("mesh", "scale must be positive"));
    }
    let format = detect(&mut Cursor::new(bytes), hint)?;
    let s = survey(bytes, format, opts)?;
    if s.triangles <= target as u64 || s.bbox.is_empty() {
        return Ok(None);
    }
    let diagonal = s.bbox.diagonal();
    let budget = target.max(4) as f64;
    let estimate = (2.0 * s.area / budget).sqrt();
    // a soup without area (every triangle degenerate) still gets a cell that can reduce it
    let floor = diagonal / budget;
    let mut h = if estimate.is_finite() && estimate > floor { estimate } else { floor.max(f64::MIN_POSITIVE) };
    let mut found = None;
    for _attempt in 0..ATTEMPTS {
        if let Some(c) = pass(bytes, format, opts, s.bbox.min, h, target)? {
            found = Some((c, h));
            break;
        }
        h *= GROW;
    }
    let Some((mut best, mut cell_size)) = found else {
        return Err(IoError::Limit(format!(
            "the file could not be reduced to {target} triangles; give a larger `reduce_to` or reduce it elsewhere"
        )));
    };
    let reached = best.triangle_count();
    if reached == 0 {
        return Err(bad("mesh", "no triangle is left after the reduction (the triangles of the file have no area)"));
    }
    if (reached as f64) < REFINE_BELOW * target as f64 {
        let finer = cell_size * (reached as f64 / target as f64).sqrt() * REFINE_MARGIN;
        if let Some(c) = pass(bytes, format, opts, s.bbox.min, finer, target)? {
            best = c;
            cell_size = finer;
        }
    }
    let mut mesh = best.finish();
    mesh.validate().map_err(|e| bad("mesh", e.to_string()))?;
    mesh.remove_degenerate(0.0);
    surfacecraft_geom::edit::compact(&mut mesh);
    Ok(Some(Reduced { mesh, source_triangles: s.triangles, cell_size }))
}
