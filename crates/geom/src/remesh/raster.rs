//! Scan conversion of a triangle soup to the sparse voxel solid of [`Occupancy`].
//!
//! Every column of the grid is a ray along +z through the triangles. Each crossing is an *enter*
//! (the ray goes against the outward normal) or an *exit*; walking a column's crossings in order,
//! the solid is the stretch from an enter to the next exit (Nooruddin and Turk 2003, "Simplification
//! and repair of polygonal models using volumetric techniques", scanline variant). Crossings that
//! do not alternate (a fin, a coincident second shell, an unclosed edge) are resolved locally: an
//! exit while outside is ignored and an enter while inside restarts the run, so one stray triangle
//! costs its own column, not the rest of the ray. Memory is eight bytes per crossing.
use super::occupancy::{Occupancy, Run};
use super::{Grid, RemeshError};
use crate::{Mesh, Point3, par};

/// Sub-voxel offset of every ray inside its column (grid units): a fixed pair of irrational-looking
/// numbers, so that rays do not run along the edges of axis-aligned meshes.
const JITTER: (f64, f64) = (0.071_283_9, 0.029_745_3);
/// Triangles rasterised per parallel task.
const CHUNK: usize = 2048;
/// Chunks per batch between limit checks (fixed, so the refusal does not depend on the threads).
const BATCH_CHUNKS: usize = 16;

/// What the scan conversion found.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RasterStats {
    /// Crossings of rays with triangles.
    pub crossings: usize,
    /// Columns with at least one crossing.
    pub columns: usize,
    /// Columns whose crossings did not alternate (stray exits, restarted or unclosed runs).
    pub irregular_columns: usize,
}

/// Order-preserving map of an `f32` to `u32`.
fn ordered(f: f32) -> u32 {
    let b = f.to_bits();
    if b >> 31 == 1 { !b } else { b | 0x8000_0000 }
}

/// Inverse of [`ordered`].
fn unordered(u: u32) -> f32 {
    f32::from_bits(if u >> 31 == 1 { u & 0x7fff_ffff } else { !u })
}

/// Edge function of the 2D edge `a → b` at `p`, computed from the lexicographically smaller end so
/// that the two triangles on either side of an edge get exactly opposite values.
fn edge(a: (f64, f64), b: (f64, f64), p: (f64, f64)) -> f64 {
    if a <= b {
        (b.0 - a.0) * (p.1 - a.1) - (b.1 - a.1) * (p.0 - a.0)
    } else {
        -((a.0 - b.0) * (p.1 - b.1) - (a.1 - b.1) * (p.0 - b.0))
    }
}

/// Position of `p` in grid units: voxel `(i, j, k)` has its centre at `(i, j, k)`.
fn to_grid(g: &Grid, p: &Point3) -> [f64; 3] {
    [(p.x - g.origin.x) / g.h - 0.5, (p.y - g.origin.y) / g.h - 0.5, (p.z - g.origin.z) / g.h - 0.5]
}

/// Appends the crossings of the columns under triangle `t`.
fn rasterise(g: &Grid, t: [[f64; 3]; 3], flip: bool, out: &mut Vec<u64>) {
    let xy = |v: &[f64; 3]| (v[0], v[1]);
    let (a, b, c) = (xy(&t[0]), xy(&t[1]), xy(&t[2]));
    let det = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    if !(det.is_finite() && det != 0.0) {
        return;
    }
    let range = |lo: f64, hi: f64, jit: f64, n: usize| -> Option<(usize, usize)> {
        let (first, last) = ((lo - jit).ceil().max(0.0), (hi - jit).floor().min(n as f64 - 1.0));
        (first <= last).then_some((first as usize, last as usize))
    };
    let (Some((i0, i1)), Some((j0, j1))) = (
        range(a.0.min(b.0).min(c.0), a.0.max(b.0).max(c.0), JITTER.0, g.nx),
        range(a.1.min(b.1).min(c.1), a.1.max(b.1).max(c.1), JITTER.1, g.ny),
    ) else {
        return;
    };
    let enter = (det < 0.0) != flip;
    for j in j0..=j1 {
        for i in i0..=i1 {
            let p = (i as f64 + JITTER.0, j as f64 + JITTER.1);
            let (e0, e1, e2) = (edge(a, b, p), edge(b, c, p), edge(c, a, p));
            let inside = if det > 0.0 { e0 > 0.0 && e1 > 0.0 && e2 > 0.0 } else { e0 < 0.0 && e1 < 0.0 && e2 < 0.0 };
            if !inside {
                continue;
            }
            let z = ((e1 * t[0][2] + e2 * t[1][2] + e0 * t[2][2]) / (e0 + e1 + e2)) as f32;
            let key = ((j * g.nx + i) as u64) << 33 | u64::from(ordered(z)) << 1 | u64::from(enter);
            out.push(key);
        }
    }
}

/// The crossings of all triangles, sorted by column, then z, then kind. Fails once their number
/// would pass `limit`, so memory stays bounded for any input.
fn crossings(mesh: &Mesh, g: &Grid, flip: bool, limit: usize) -> Result<Vec<u64>, RemeshError> {
    let chunks = mesh.triangle_count().div_ceil(CHUNK);
    let mut all: Vec<u64> = Vec::new();
    let batch = BATCH_CHUNKS;
    for first in (0..chunks).step_by(batch) {
        let part: Vec<Vec<u64>> = par::map(batch.min(chunks - first), |b| {
            let mut out = Vec::new();
            let lo = (first + b) * CHUNK;
            for ti in lo..(lo + CHUNK).min(mesh.triangle_count()) {
                if let Some([p, q, r]) = mesh.triangle(ti) {
                    rasterise(g, [to_grid(g, &p), to_grid(g, &q), to_grid(g, &r)], flip, &mut out);
                }
            }
            out
        });
        for p in part {
            all.extend(p);
        }
        if all.len() > limit {
            return Err(RemeshError::TooMuchWork {
                what: "ray crossings".into(),
                needed: all.len(),
                limit,
                advice: "use a larger voxel".into(),
            });
        }
    }
    par::sort_unstable(&mut all);
    Ok(all)
}

/// Runs of one column from its sorted crossings, and whether they alternated.
fn column_runs(list: &[u64]) -> (Vec<Run>, bool) {
    let (mut runs, mut regular, mut open): (Vec<Run>, bool, Option<f32>) = (Vec::new(), true, None);
    for key in list {
        let w = unordered(((key >> 1) & 0xffff_ffff) as u32);
        if key & 1 == 1 {
            regular &= open.is_none();
            open = Some(w);
        } else if let Some(s) = open.take() {
            let (k0, k1) = (s.ceil().max(0.0) as u32, w.ceil().max(0.0) as u32);
            if k1 > k0 {
                runs.push([k0, k1]);
            }
        } else {
            regular = false;
        }
    }
    (runs, regular && open.is_none())
}

/// The solid of `mesh` on `grid`. `flip` reads the triangles as inside out.
pub(super) fn solid(
    mesh: &Mesh,
    grid: &Grid,
    flip: bool,
    limit: usize,
) -> Result<(Occupancy, RasterStats), RemeshError> {
    let sorted = crossings(mesh, grid, flip, limit)?;
    let mut stats = RasterStats { crossings: sorted.len(), ..Default::default() };
    let mut columns: Vec<(u32, Vec<Run>)> = Vec::new();
    let mut start = 0;
    while let Some(first) = sorted.get(start) {
        let col = first >> 33;
        let len = sorted.get(start..).map_or(0, |r| r.iter().take_while(|k| *k >> 33 == col).count());
        let (runs, regular) = column_runs(sorted.get(start..start + len).unwrap_or(&[]));
        stats.columns += 1;
        stats.irregular_columns += usize::from(!regular);
        if !runs.is_empty() {
            columns.push((col as u32, runs));
        }
        start += len;
    }
    drop(sorted);
    Ok((Occupancy::from_columns(grid.nx, grid.ny, grid.nz as u32, columns), stats))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::synth;

    fn grid_for(mesh: &Mesh, h: f64) -> Grid {
        Grid::new(&mesh.bbox(), h, 3).unwrap()
    }

    #[test]
    fn ordered_floats_sort_like_floats() {
        let v = [-5.5f32, -0.1, 0.0, 0.3, 12.0];
        assert!(v.windows(2).all(|w| ordered(w[0]) < ordered(w[1])));
        assert!(v.iter().all(|&f| unordered(ordered(f)) == f));
    }

    #[test]
    fn a_sphere_fills_its_volume_and_has_no_irregular_columns() {
        let mut m = synth::sphere(Point3::origin(), 10.0, 64, 32);
        m.weld(1e-9);
        let g = grid_for(&m, 0.5);
        let (occ, stats) = solid(&m, &g, false, 10_000_000).unwrap();
        let exact = 4.0 / 3.0 * std::f64::consts::PI * 1000.0;
        let volume = occ.voxel_count() as f64 * 0.125;
        assert!((volume - exact).abs() / exact < 0.02, "{volume} vs {exact}");
        assert_eq!(stats.irregular_columns, 0);
    }

    #[test]
    fn an_inside_out_mesh_is_read_with_flip_and_a_stray_fin_costs_only_its_column() {
        let mut m = synth::sphere(Point3::origin(), 10.0, 64, 32);
        m.weld(1e-9);
        let g = grid_for(&m, 0.5);
        let (reference, _) = solid(&m, &g, false, 10_000_000).unwrap();
        let mut flipped = m.clone();
        flipped.triangles.iter_mut().for_each(|t| t.swap(1, 2));
        let (back, _) = solid(&flipped, &g, true, 10_000_000).unwrap();
        assert_eq!(back.voxel_count(), reference.voxel_count());
        let mut finned = m.clone();
        let n = finned.positions.len() as u32;
        finned.positions.extend([
            Point3::new(0.0, 0.0, 12.0),
            Point3::new(3.0, 0.0, 12.0),
            Point3::new(0.0, 3.0, 13.0),
        ]);
        finned.triangles.push([n, n + 1, n + 2]);
        let (with_fin, stats) = solid(&finned, &g, false, 10_000_000).unwrap();
        assert!(stats.irregular_columns > 0);
        let diff = with_fin.voxel_count().abs_diff(reference.voxel_count());
        assert!(diff < 40, "{diff}");
    }

    #[test]
    fn the_crossing_limit_refuses() {
        let m = synth::sphere(Point3::origin(), 10.0, 64, 32);
        let g = grid_for(&m, 0.5);
        assert!(matches!(solid(&m, &g, false, 100), Err(RemeshError::TooMuchWork { .. })));
    }
}
