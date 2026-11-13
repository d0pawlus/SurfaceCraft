//! The boundary of a sparse voxel solid as a quad mesh: naive surface nets (Gibson 1998, "Constrained
//! elastic surface nets"; Frisken's "Simple and efficient surface nets" 2002 for the construction):
//! one vertex per cell of the dual grid that holds both inside and outside voxel centres, one quad
//! for every pair of face-adjacent voxels that differ, joined to the four cells around the pair. The
//! quads are wound counter-clockwise seen from outside. On a set without checkerboard 2 × 2 blocks
//! (see `Occupancy::fill_diagonals`) every edge of the result has exactly two quads.
use super::occupancy::Occupancy;
use super::{Grid, RemeshError};
use crate::{Mesh, Point3, par};

/// A pair of face-adjacent voxels, the lower one at `(i, j, k)`, that differ.
#[derive(Clone, Copy)]
struct Pair {
    axis: u8,
    i: usize,
    j: usize,
    k: u32,
    /// The inside voxel is the upper one.
    inside_upper: bool,
}

fn key(ci: usize, cj: usize, ck: u32) -> u64 {
    ((ci as u64) << 42) | ((cj as u64) << 21) | u64::from(ck)
}

fn unkey(k: u64) -> (usize, usize, usize) {
    (((k >> 42) & 0x1f_ffff) as usize, ((k >> 21) & 0x1f_ffff) as usize, (k & 0x1f_ffff) as usize)
}

/// Calls `f` for every pair of face-adjacent voxels that differ, column by column.
fn for_each_pair(occ: &Occupancy, mut f: impl FnMut(Pair)) {
    for j in 0..occ.ny {
        for i in 0..occ.nx {
            let col = occ.column(i, j);
            if col.is_empty() && occ.column(i + 1, j).is_empty() && occ.column(i, j + 1).is_empty() {
                continue;
            }
            for r in col {
                f(Pair { axis: 2, i, j, k: r[0].saturating_sub(1), inside_upper: true });
                f(Pair { axis: 2, i, j, k: r[1] - 1, inside_upper: false });
            }
            for (axis, other) in [(0u8, occ.column(i + 1, j)), (1u8, occ.column(i, j + 1))] {
                for (a, b, upper) in Occupancy::differences(col, other) {
                    for k in a..b {
                        f(Pair { axis, i, j, k, inside_upper: upper });
                    }
                }
            }
        }
    }
}

/// The number of quads the solid's boundary has.
#[must_use]
pub fn quad_count(occ: &Occupancy) -> usize {
    let mut n = 0usize;
    for j in 0..occ.ny {
        for i in 0..occ.nx {
            let col = occ.column(i, j);
            n += 2 * col.len();
            for other in [occ.column(i + 1, j), occ.column(i, j + 1)] {
                n += Occupancy::differences(col, other).iter().map(|d| (d.1 - d.0) as usize).sum::<usize>();
            }
        }
    }
    n
}

/// The four cells around a pair, counter-clockwise seen from outside.
fn cells(p: &Pair) -> [u64; 4] {
    let (i, j, k) = (p.i, p.j, p.k);
    // the grid has an empty border of voxels, so a pair is never at index 0; saturate for safety
    let (i0, j0, k0) = (i.saturating_sub(1), j.saturating_sub(1), k.saturating_sub(1));
    let mut c = match p.axis {
        2 => [key(i0, j0, k), key(i, j0, k), key(i, j, k), key(i0, j, k)],
        0 => [key(i, j0, k0), key(i, j, k0), key(i, j, k), key(i, j0, k)],
        _ => [key(i0, j, k0), key(i0, j, k), key(i, j, k), key(i, j, k0)],
    };
    if p.inside_upper {
        c.reverse();
    }
    c
}

/// The surface of `occ` as quads over the vertices of the dual grid, positions in model space.
pub(super) fn quads(occ: &Occupancy, grid: &Grid, limit: usize) -> Result<(Vec<Point3>, Vec<[u32; 4]>), RemeshError> {
    let count = quad_count(occ);
    if count > limit {
        let advice = "use a larger voxel".to_string();
        return Err(RemeshError::TooMuchWork { what: "surface quads".into(), needed: count, limit, advice });
    }
    let mut all: Vec<u64> = Vec::with_capacity(count * 4);
    for_each_pair(occ, |p| all.extend(cells(&p)));
    par::sort_unstable(&mut all);
    all.dedup();
    let mut out: Vec<[u32; 4]> = Vec::with_capacity(count);
    for_each_pair(occ, |p| {
        let find = |c: u64| all.binary_search(&c).unwrap_or(0) as u32;
        out.push(cells(&p).map(find));
    });
    let h = grid.h;
    let positions = all
        .iter()
        .map(|&c| {
            let (ci, cj, ck) = unkey(c);
            Point3::new(
                grid.origin.x + (ci as f64 + 1.0) * h,
                grid.origin.y + (cj as f64 + 1.0) * h,
                grid.origin.z + (ck as f64 + 1.0) * h,
            )
        })
        .collect();
    Ok((positions, out))
}

/// Splits every quad along its shorter diagonal.
pub(super) fn triangulate(positions: &[Point3], quads: &[[u32; 4]]) -> Vec<[u32; 3]> {
    let mut tris = Vec::with_capacity(quads.len() * 2);
    for q in quads {
        let d = |a: u32, b: u32| match (positions.get(a as usize), positions.get(b as usize)) {
            (Some(p), Some(r)) => (p - r).norm_squared(),
            _ => 0.0,
        };
        if d(q[0], q[2]) <= d(q[1], q[3]) {
            tris.push([q[0], q[1], q[2]]);
            tris.push([q[0], q[2], q[3]]);
        } else {
            tris.push([q[0], q[1], q[3]]);
            tris.push([q[1], q[2], q[3]]);
        }
    }
    tris
}

/// Duplicates every vertex whose triangles fall into several fans (groups connected through shared
/// edges at that vertex), so that each vertex is a manifold point. Returns the mesh and the number
/// of vertices added.
#[must_use]
pub fn split_pinched_vertices(mesh: &Mesh) -> (Mesh, usize) {
    let n = mesh.positions.len();
    let mut incident: Vec<Vec<u32>> = vec![Vec::new(); n];
    for (ti, t) in mesh.triangles.iter().enumerate() {
        for &v in t {
            if let Some(list) = incident.get_mut(v as usize) {
                list.push(ti as u32);
            }
        }
    }
    let mut positions = mesh.positions.clone();
    let mut triangles = mesh.triangles.clone();
    let mut added = 0;
    for (v, tris) in incident.iter().enumerate() {
        let fans = fans_of(mesh, v as u32, tris);
        for fan in fans.iter().skip(1) {
            let new = positions.len() as u32;
            positions.push(mesh.positions.get(v).copied().unwrap_or_else(Point3::origin));
            added += 1;
            for &t in fan {
                if let Some(tri) = triangles.get_mut(t as usize) {
                    tri.iter_mut().filter(|x| **x == v as u32).for_each(|x| *x = new);
                }
            }
        }
    }
    (Mesh { positions, triangles }, added)
}

/// The triangles around `v` grouped into fans, in order of their lowest triangle index.
fn fans_of(mesh: &Mesh, v: u32, tris: &[u32]) -> Vec<Vec<u32>> {
    if tris.len() < 2 {
        return vec![tris.to_vec()];
    }
    let mut group: Vec<usize> = (0..tris.len()).collect();
    let find = |g: &mut Vec<usize>, mut x: usize| {
        while g[x] != x {
            g[x] = g[g[x]];
            x = g[x];
        }
        x
    };
    let mut by_neighbour: std::collections::BTreeMap<u32, usize> = std::collections::BTreeMap::new();
    for (local, &t) in tris.iter().enumerate() {
        for &w in mesh.triangles.get(t as usize).into_iter().flatten().filter(|w| **w != v) {
            if let Some(&other) = by_neighbour.get(&w) {
                let (a, b) = (find(&mut group, local), find(&mut group, other));
                if a != b {
                    group[b] = a;
                }
            } else {
                by_neighbour.insert(w, local);
            }
        }
    }
    let mut fans: std::collections::BTreeMap<usize, Vec<u32>> = std::collections::BTreeMap::new();
    for (local, &t) in tris.iter().enumerate() {
        let root = find(&mut group, local);
        fans.entry(root).or_default().push(t);
    }
    let mut out: Vec<Vec<u32>> = fans.into_values().collect();
    out.sort_by_key(|f| f.first().copied());
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(n: usize, lo: usize, hi: usize) -> (Occupancy, Grid) {
        let cols =
            (lo..hi).flat_map(|j| (lo..hi).map(move |i| ((j * n + i) as u32, vec![[lo as u32, hi as u32]]))).collect();
        let occ = Occupancy::from_columns(n, n, n as u32, cols);
        let grid = Grid { origin: Point3::new(-1.0, -1.0, -1.0), h: 1.0, nx: n, ny: n, nz: n };
        (occ, grid)
    }

    fn mesh_of(occ: &Occupancy, grid: &Grid) -> Mesh {
        let (positions, q) = quads(occ, grid, usize::MAX).unwrap();
        let triangles = triangulate(&positions, &q);
        Mesh { positions, triangles }
    }

    #[test]
    fn a_cube_of_voxels_is_a_closed_outward_manifold_of_genus_zero() {
        let (occ, grid) = cube(12, 3, 7);
        assert_eq!(quad_count(&occ), 6 * 16);
        let m = mesh_of(&occ, &grid);
        let pieces = crate::pieces::pieces(&m);
        assert_eq!(pieces.len(), 1);
        assert!(pieces[0].closed());
        assert_eq!(pieces[0].genus(), Some(0));
        let volume: f64 = (0..m.triangle_count())
            .filter_map(|i| m.triangle(i))
            .map(|[a, b, c]| a.coords.dot(&b.coords.cross(&c.coords)) / 6.0)
            .sum();
        assert!(volume > 0.0, "{volume}");
    }

    #[test]
    fn a_solid_in_the_first_column_and_voxel_does_not_panic() {
        let (occ, grid) = cube(8, 0, 3);
        let (positions, quads) = quads(&occ, &grid, usize::MAX).unwrap();
        assert!(!positions.is_empty() && !quads.is_empty());
    }

    #[test]
    fn two_cubes_touching_along_an_edge_are_two_fans_after_splitting() {
        let n = 12;
        let mut cols = Vec::new();
        for j in 2..5usize {
            for i in 2..5usize {
                cols.push(((j * n + i) as u32, vec![[2u32, 5u32]]));
            }
        }
        for j in 5..8usize {
            for i in 5..8usize {
                cols.push(((j * n + i) as u32, vec![[5u32, 8u32]]));
            }
        }
        cols.sort_by_key(|c| c.0);
        let occ = Occupancy::from_columns(n, n, n as u32, cols);
        let grid = Grid { origin: Point3::origin(), h: 1.0, nx: n, ny: n, nz: n };
        let m = mesh_of(&occ, &grid);
        let (split, added) = split_pinched_vertices(&m);
        assert!(added > 0);
        let pieces = crate::pieces::pieces(&split);
        assert_eq!(pieces.len(), 2);
        assert!(pieces.iter().all(|p| p.closed()));
    }
}
