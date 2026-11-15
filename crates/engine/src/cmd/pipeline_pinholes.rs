//! Pinholes of a segmented region (P9-O6b): the segmentation leaves a few triangles of no region inside a large
//! flat face (a handful of noisy triangles that fit no primitive). A patch over a pinhole is on the region's
//! primitive all the same, so the triangles that are holes in a region count as the region when a patch is
//! matched to it, and every sample of a patch can then be required to lie on the region.
//!
//! A pinhole is a connected set of triangles that belong to no region, closed all the way round (no edge on
//! the open border of the scan, none shared with a triangle of another region or with a loose one), whose
//! neighbours are all of one region and whose vertices all lie within `near` of that region's primitive.
use surfacecraft_fit::Primitive;
use surfacecraft_geom::{Mesh, Point3};

/// The triangles of `scan` that are pinholes, as `(triangle, region)`. `owner` is the region of each triangle
/// (`unowned` for none), `primitives` the primitive of each region.
pub(super) fn pinholes(
    scan: &Mesh,
    owner: &[u32],
    primitives: &[&Primitive],
    near: f64,
    unowned: u32,
) -> Vec<(u32, u32)> {
    let neighbours = scan.triangle_neighbours();
    let owner_of = |t: usize| owner.get(t).copied().unwrap_or(unowned);
    let mut seen = vec![false; scan.triangles.len()];
    let mut out = Vec::new();
    for start in 0..scan.triangles.len() {
        if owner_of(start) != unowned || seen.get(start).copied().unwrap_or(true) {
            continue;
        }
        let (component, enclosing) = flood(&neighbours, &owner_of, &mut seen, start, unowned);
        let Some(region) = enclosing else { continue };
        let Some(primitive) = usize::try_from(region).ok().and_then(|k| primitives.get(k)) else { continue };
        let close = component.iter().all(|&t| {
            scan.triangles.get(t).is_some_and(|tri| {
                tri.iter().all(|&v| {
                    scan.positions.get(v as usize).is_some_and(|p: &Point3| primitive.distance(p).abs() <= near)
                })
            })
        });
        if close {
            out.extend(component.into_iter().map(|t| (t as u32, region)));
        }
    }
    out
}

/// The triangles of no region connected to `start`, and the one region they are enclosed by (`None` when they
/// touch the open border of the scan, a non-manifold edge, or more than one region, or no region at all).
fn flood(
    neighbours: &[[u32; 3]],
    owner_of: &dyn Fn(usize) -> u32,
    seen: &mut [bool],
    start: usize,
    unowned: u32,
) -> (Vec<usize>, Option<u32>) {
    let (mut component, mut enclosing, mut open) = (vec![start], None, false);
    if let Some(s) = seen.get_mut(start) {
        *s = true;
    }
    let mut next = 0;
    while let Some(&t) = component.get(next) {
        next += 1;
        for &n in neighbours.get(t).map_or(&[u32::MAX; 3], |n| n) {
            if n == u32::MAX {
                open = true;
                continue;
            }
            let n = n as usize;
            if owner_of(n) == unowned {
                if !seen.get(n).copied().unwrap_or(true) {
                    if let Some(s) = seen.get_mut(n) {
                        *s = true;
                    }
                    component.push(n);
                }
            } else if enclosing.is_some_and(|r| r != owner_of(n)) {
                open = true;
            } else {
                enclosing = Some(owner_of(n));
            }
        }
    }
    (component, if open { None } else { enclosing })
}

#[cfg(test)]
mod tests {
    use super::*;

    const NONE: u32 = u32::MAX;

    /// A flat `n x n` grid of cells in z = 0, two triangles a cell (the cell `c` has triangles `2 c`, `2 c + 1`),
    /// with the vertices of the cells in `lifted` raised by `lift`.
    fn grid(n: usize, lift: f64, lifted: &[usize]) -> Mesh {
        let id = |i: usize, j: usize| (i * (n + 1) + j) as u32;
        let mut positions = Vec::new();
        for i in 0..=n {
            for j in 0..=n {
                let up = lifted.iter().any(|&c| {
                    let (ci, cj) = (c / n, c % n);
                    (ci..=ci + 1).contains(&i) && (cj..=cj + 1).contains(&j)
                });
                positions.push(Point3::new(i as f64, j as f64, if up { lift } else { 0.0 }));
            }
        }
        let mut triangles = Vec::new();
        for i in 0..n {
            for j in 0..n {
                triangles.push([id(i, j), id(i + 1, j), id(i + 1, j + 1)]);
                triangles.push([id(i, j), id(i + 1, j + 1), id(i, j + 1)]);
            }
        }
        Mesh { positions, triangles }
    }

    fn plane() -> Primitive {
        Primitive::Plane { point: [0.0; 3], normal: [0.0, 0.0, 1.0] }
    }

    fn owners(m: &Mesh, holes: &[usize]) -> Vec<u32> {
        (0..m.triangles.len()).map(|t| if holes.contains(&t) { NONE } else { 0 }).collect()
    }

    #[test]
    fn unassigned_triangles_enclosed_by_one_region_and_on_its_primitive_are_closed() {
        let m = grid(6, 0.0, &[]);
        let holes = [2 * (3 * 6 + 3), 2 * (3 * 6 + 3) + 1];
        let found = pinholes(&m, &owners(&m, &holes), &[&plane()], 0.1, NONE);
        assert_eq!(found, vec![(holes[0] as u32, 0), (holes[1] as u32, 0)]);
    }

    #[test]
    fn a_hole_off_the_primitive_or_on_the_open_border_or_between_two_regions_stays_open() {
        let m = grid(6, 0.0, &[]);
        let middle = [2 * (3 * 6 + 3)];
        // off the primitive: the cell's vertices are 0.5 mm up, the limit 0.1
        let up = grid(6, 0.5, &[3 * 6 + 3]);
        assert!(pinholes(&up, &owners(&up, &middle), &[&plane()], 0.1, NONE).is_empty());
        // on the border of the scan
        assert!(pinholes(&m, &owners(&m, &[0]), &[&plane()], 0.1, NONE).is_empty());
        // a hole whose neighbours belong to two regions
        let mut own = owners(&m, &middle);
        own[2 * (3 * 6 + 3) + 1] = 1;
        assert!(pinholes(&m, &own, &[&plane(), &plane()], 0.1, NONE).is_empty());
        // no region around it at all
        assert!(pinholes(&m, &vec![NONE; m.triangles.len()], &[&plane()], 0.1, NONE).is_empty());
    }

    #[test]
    fn hostile_indices_and_lengths_are_survived() {
        let mut m = grid(3, 0.0, &[]);
        m.triangles.push([0, 99, 1]);
        assert!(pinholes(&m, &[NONE], &[], 0.1, NONE).is_empty());
        assert!(pinholes(&m, &[], &[&plane()], f64::NAN, NONE).is_empty());
    }
}
