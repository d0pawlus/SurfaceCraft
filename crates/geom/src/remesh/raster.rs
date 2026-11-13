//! Scan conversion of a triangle soup to the sparse voxel solid of [`Occupancy`].
//!
//! Every column of the grid is a ray along +z through the triangles. Each crossing is an *enter*
//! (the ray goes against the outward normal) or an *exit*; walking a column's crossings in order,
//! the solid is the stretch from an enter to the next exit (Nooruddin and Turk 2003, "Simplification
//! and repair of polygonal models using volumetric techniques", scanline variant). Crossings that
//! do not alternate (a fin, a coincident second shell, an unclosed edge) are resolved locally: an
//! exit while outside is ignored and an enter while inside restarts the run, so one stray triangle
//! costs its own column, not the rest of the ray. Memory is eight bytes per crossing.
use super::occupancy::{Flat, Occupancy, Run};
use super::{Grid, RemeshError};
use crate::{Mesh, Point3, par};
use std::sync::atomic::{AtomicUsize, Ordering};

/// Sub-voxel offset of every ray inside its column (grid units): a fixed pair of irrational-looking
/// numbers, so that rays do not run along the edges of axis-aligned meshes.
const JITTER: (f64, f64) = (0.071_283_9, 0.029_745_3);
/// Triangles rasterised per parallel task.
const CHUNK: usize = 2048;
/// Chunks per batch between limit checks (fixed, so the refusal does not depend on the threads).
const BATCH_CHUNKS: usize = 16;
/// Crossings never exceed the cells under the boxes of the triangles; a scan whose boxes hold more than
/// this many times the crossing limit is refused before any crossing is made.
const WORK_FACTOR: u64 = 4;
/// Two crossings this close (voxels) are the same surface seen twice (a coincident shell).
const COINCIDENT: f32 = 0.05;

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

/// The rows (`j0..=j1`) the triangle reaches, clipped to the grid.
fn row_range(g: &Grid, t: &[[f64; 3]; 3]) -> Option<(usize, usize)> {
    let ys = t.map(|v| v[1]);
    let (lo, hi) = (ys[0].min(ys[1]).min(ys[2]), ys[0].max(ys[1]).max(ys[2]));
    let (first, last) = ((lo - JITTER.1).ceil().max(0.0), (hi - JITTER.1).floor().min(g.ny as f64 - 1.0));
    (first <= last).then_some((first as usize, last as usize))
}

/// The columns `i0..=i1` the triangle covers on row `j`, from the exact x-interval of its projection
/// on the line of the rays of that row (not from its box: a long diagonal sliver has a box of L² cells
/// and L crossings), clipped to the grid.
fn row_columns(g: &Grid, t: &[[f64; 3]; 3], j: usize) -> Option<(usize, usize)> {
    let y = j as f64 + JITTER.1;
    let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
    for k in 0..3 {
        let (p, q) = (t[k], t[(k + 1) % 3]);
        if !((p[1] <= y && y <= q[1]) || (q[1] <= y && y <= p[1])) {
            continue;
        }
        if p[1] == q[1] {
            lo = lo.min(p[0].min(q[0]));
            hi = hi.max(p[0].max(q[0]));
        } else {
            let x = p[0] + (q[0] - p[0]) * (y - p[1]) / (q[1] - p[1]);
            lo = lo.min(x);
            hi = hi.max(x);
        }
    }
    let (first, last) = ((lo - JITTER.0).ceil().max(0.0), (hi - JITTER.0).floor().min(g.nx as f64 - 1.0));
    (first <= last).then_some((first as usize, last as usize))
}

/// Columns the rays of the rows reach under the triangle: an upper bound of its crossings.
fn span_cells(g: &Grid, t: &[[f64; 3]; 3]) -> u64 {
    let Some((j0, j1)) = row_range(g, t) else { return 0 };
    (j0..=j1).filter_map(|j| row_columns(g, t, j)).fold(0u64, |acc, (i0, i1)| acc.saturating_add((i1 - i0 + 1) as u64))
}

/// Appends the crossings of the columns under triangle `t`, row by row; stops when `made` (crossings
/// of all tasks so far, this one's included after each row) passes `limit`.
fn rasterise(g: &Grid, t: [[f64; 3]; 3], flip: bool, out: &mut Vec<u64>, budget: (&AtomicUsize, usize)) {
    let xy = |v: &[f64; 3]| (v[0], v[1]);
    let (a, b, c) = (xy(&t[0]), xy(&t[1]), xy(&t[2]));
    let det = (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0);
    if !(det.is_finite() && det != 0.0) {
        return;
    }
    let Some((j0, j1)) = row_range(g, &t) else { return };
    let enter = (det < 0.0) != flip;
    for j in j0..=j1 {
        let Some((i0, i1)) = row_columns(g, &t, j) else { continue };
        let before = out.len();
        for i in i0..=i1 {
            let p = (i as f64 + JITTER.0, j as f64 + JITTER.1);
            let (e0, e1, e2) = (edge(a, b, p), edge(b, c, p), edge(c, a, p));
            let inside = if det > 0.0 { e0 > 0.0 && e1 > 0.0 && e2 > 0.0 } else { e0 < 0.0 && e1 < 0.0 && e2 < 0.0 };
            if !inside {
                continue;
            }
            let z = ((e1 * t[0][2] + e2 * t[1][2] + e0 * t[2][2]) / (e0 + e1 + e2)) as f32;
            let col = (j as u64) * (g.nx as u64) + i as u64;
            out.push(col << 33 | u64::from(ordered(z)) << 1 | u64::from(enter));
        }
        let made = budget.0.fetch_add(out.len() - before, Ordering::Relaxed) + (out.len() - before);
        if made > budget.1 {
            return;
        }
    }
}

fn triangle_of(mesh: &Mesh, g: &Grid, ti: usize) -> Option<[[f64; 3]; 3]> {
    mesh.triangle(ti).map(|[p, q, r]| [to_grid(g, &p), to_grid(g, &q), to_grid(g, &r)])
}

/// Upper bound of the crossings: the columns under the exact row spans of all triangles.
fn crossing_bound(mesh: &Mesh, g: &Grid) -> u64 {
    let chunks = mesh.triangle_count().div_ceil(CHUNK);
    par::map(chunks, |c| {
        let lo = c * CHUNK;
        (lo..(lo + CHUNK).min(mesh.triangle_count()))
            .filter_map(|ti| triangle_of(mesh, g, ti))
            .fold(0u64, |acc, t| acc.saturating_add(span_cells(g, &t)))
    })
    .into_iter()
    .fold(0u64, u64::saturating_add)
}

/// The crossings of all triangles, sorted by column, then z, then kind. Refused before anything is
/// allocated when the row spans of the triangles could hold more than [`WORK_FACTOR`] times `limit`, and
/// stopped within a row of the moment `limit` is passed, so memory stays bounded for any input. The
/// refusals report the bound, which does not depend on the thread count.
fn crossings(mesh: &Mesh, g: &Grid, flip: bool, limit: usize) -> Result<Vec<u64>, RemeshError> {
    let bound = crossing_bound(mesh, g);
    let refuse = |what: &str| RemeshError::TooMuchWork {
        what: what.into(),
        needed: usize::try_from(bound).unwrap_or(usize::MAX),
        limit,
        advice: "use a larger voxel".into(),
    };
    if bound > (limit as u64).saturating_mul(WORK_FACTOR) {
        return Err(refuse("ray crossings (at most, counted from the rows of the triangles, before any is made)"));
    }
    let chunks = mesh.triangle_count().div_ceil(CHUNK);
    let (mut all, count): (Vec<u64>, AtomicUsize) = (Vec::new(), AtomicUsize::new(0));
    for first in (0..chunks).step_by(BATCH_CHUNKS) {
        let part: Vec<Vec<u64>> = par::map(BATCH_CHUNKS.min(chunks - first), |b| {
            let mut out = Vec::new();
            let lo = (first + b) * CHUNK;
            for ti in lo..(lo + CHUNK).min(mesh.triangle_count()) {
                if count.load(Ordering::Relaxed) > limit {
                    break;
                }
                if let Some(t) = triangle_of(mesh, g, ti) {
                    rasterise(g, t, flip, &mut out, (&count, limit));
                }
            }
            out
        });
        if count.load(Ordering::Relaxed) > limit {
            return Err(refuse("ray crossings (stopped while they were made)"));
        }
        for p in part {
            all.extend(p);
        }
    }
    par::sort_unstable(&mut all);
    Ok(all)
}

/// Runs of one column from its sorted crossings, and whether they alternated. A second enter or exit
/// at the same place as the one before (a coincident shell) is the same surface seen twice, not a
/// defect.
fn column_runs(list: &[u64]) -> (Vec<Run>, bool) {
    let (mut runs, mut regular, mut open): (Vec<Run>, bool, Option<f32>) = (Vec::new(), true, None);
    let mut last_exit = f32::NEG_INFINITY;
    for key in list {
        let w = unordered(((key >> 1) & 0xffff_ffff) as u32);
        if key & 1 == 1 {
            regular &= open.is_none_or(|s| (w - s).abs() <= COINCIDENT);
            open = Some(w);
        } else if let Some(s) = open.take() {
            let (k0, k1) = (s.ceil().max(0.0) as u32, w.ceil().max(0.0) as u32);
            if k1 > k0 {
                runs.push([k0, k1]);
            }
            last_exit = w;
        } else {
            regular &= (w - last_exit).abs() <= COINCIDENT;
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
    let mut flat = Flat::default();
    let mut start = 0;
    while let Some(first) = sorted.get(start) {
        let col = first >> 33;
        let len = sorted.get(start..).map_or(0, |r| r.iter().take_while(|k| *k >> 33 == col).count());
        let (mut runs, regular) = column_runs(sorted.get(start..start + len).unwrap_or(&[]));
        stats.columns += 1;
        stats.irregular_columns += usize::from(!regular);
        flat.push(col as u32, &mut runs, grid.nz as u32);
        start += len;
    }
    drop(sorted);
    Ok((Occupancy::from_flat(grid.nx, grid.ny, grid.nz as u32, flat), stats))
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
    fn many_large_triangles_are_refused_before_any_crossing_is_made() {
        let mut m = Mesh::default();
        for k in 0..4000u32 {
            let base = m.positions.len() as u32;
            let z = f64::from(k) * 1e-3;
            m.positions.extend([Point3::new(0.0, 0.0, z), Point3::new(100.0, 0.0, z), Point3::new(0.0, 100.0, z)]);
            m.triangles.push([base, base + 1, base + 2]);
        }
        let g = Grid::new(&m.bbox(), 0.1, 3).unwrap();
        match solid(&m, &g, false, 1000) {
            Err(RemeshError::TooMuchWork { what, needed, .. }) => {
                assert!(what.contains("before any is made"), "{what}");
                assert!(needed > 4000 * 1000, "{needed}");
            }
            other => panic!("{other:?}"),
        }
    }

    /// A flat square fanned into slivers that each run the whole width: boxes of L² cells, L crossings.
    #[test]
    fn a_fan_of_full_width_slivers_passes_and_a_dense_scan_passes_the_pre_check() {
        let n = 160usize;
        let mut m = Mesh::default();
        m.positions.push(Point3::new(0.0, 0.0, 0.0));
        for k in 0..=n {
            m.positions.push(Point3::new(300.0, 300.0 * k as f64 / n as f64, 0.0));
        }
        for k in 0..n as u32 {
            m.triangles.push([0, k + 1, k + 2]);
        }
        let g = Grid::new(&m.bbox(), 0.3, 3).unwrap();
        assert!(crossing_bound(&m, &g) < 3_000_000, "{}", crossing_bound(&m, &g));
        let (_, stats) = solid(&m, &g, false, 40_000_000).unwrap();
        assert!(stats.crossings > 400_000 && stats.crossings < 1_200_000, "{}", stats.crossings);
        let dense = synth::sphere(Point3::origin(), 150.0, 1000, 500);
        let g = Grid::new(&dense.bbox(), 0.3, 3).unwrap();
        assert!(dense.triangle_count() >= 998_000);
        assert!(crossing_bound(&dense, &g) <= 4 * 40_000_000);
    }

    #[test]
    fn a_limit_between_the_real_crossings_and_the_bound_stops_the_run_while_it_runs() {
        let mut m = synth::sphere(Point3::origin(), 10.0, 64, 32);
        m.weld(1e-9);
        let g = grid_for(&m, 0.5);
        let (_, stats) = solid(&m, &g, false, usize::MAX / 8).unwrap();
        let bound = crossing_bound(&m, &g);
        assert!(bound >= stats.crossings as u64);
        let limit = stats.crossings - 1;
        assert!(bound <= 4 * limit as u64, "the pre-check must pass for the runtime stop to be reached");
        match solid(&m, &g, false, limit) {
            Err(RemeshError::TooMuchWork { what, needed, .. }) => {
                assert!(what.contains("stopped while they were made"), "{what}");
                assert_eq!(needed as u64, bound);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn the_crossing_limit_refuses() {
        let m = synth::sphere(Point3::origin(), 10.0, 64, 32);
        let g = grid_for(&m, 0.5);
        assert!(matches!(solid(&m, &g, false, 100), Err(RemeshError::TooMuchWork { .. })));
    }
}
