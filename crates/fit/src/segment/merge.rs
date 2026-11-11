//! Merging of adjacent regions that describe one surface.
use super::Segment;
use super::fits::fit_tris;
use crate::primitive::Primitive;
use surfacecraft_geom::{Mesh, Vec3};

/// Cheap pre-test for merging: two primitives of the same kind describing (nearly) the same
/// surface — normals/axes within 3°, positions and radii within the tolerance (radii also 3 %).
fn same_surface(a: &Primitive, b: &Primitive, tol: f64) -> bool {
    let v = |x: &[f64; 3]| Vec3::new(x[0], x[1], x[2]);
    let cos3 = 3f64.to_radians().cos();
    let close_r = |r1: f64, r2: f64| (r1 - r2).abs() <= tol.max(0.03 * r1.abs().max(r2.abs()));
    // distance of point `q` from the line through `p` along unit `d`
    let line_dist = |p: Vec3, d: Vec3, q: Vec3| (q - p - d * (q - p).dot(&d)).norm();
    match (a, b) {
        (Primitive::Plane { point: p1, normal: n1 }, Primitive::Plane { point: p2, normal: n2 }) => {
            let (n1, n2) = (v(n1), v(n2));
            n1.dot(&n2).abs() >= cos3 && (v(p2) - v(p1)).dot(&n1).abs() <= tol
        }
        (Primitive::Sphere { center: c1, radius: r1 }, Primitive::Sphere { center: c2, radius: r2 }) => {
            (v(c1) - v(c2)).norm() <= tol.max(0.03 * r1.abs()) && close_r(*r1, *r2)
        }
        (
            Primitive::Cylinder { point: p1, axis: a1, radius: r1 },
            Primitive::Cylinder { point: p2, axis: a2, radius: r2 },
        ) => {
            let (a1, a2) = (v(a1), v(a2));
            a1.dot(&a2).abs() >= cos3 && line_dist(v(p1), a1, v(p2)) <= tol.max(0.03 * r1.abs()) && close_r(*r1, *r2)
        }
        (
            Primitive::Cone { apex: x1, axis: a1, half_angle: h1 },
            Primitive::Cone { apex: x2, axis: a2, half_angle: h2 },
        ) => {
            v(a1).dot(&v(a2)) >= cos3
                && (h1 - h2).abs() <= 3f64.to_radians()
                && line_dist(v(x1), v(a1), v(x2)) <= 4.0 * tol
        }
        (
            Primitive::Torus { center: c1, axis: a1, major_radius: m1, minor_radius: n1 },
            Primitive::Torus { center: c2, axis: a2, major_radius: m2, minor_radius: n2 },
        ) => {
            v(a1).dot(&v(a2)).abs() >= cos3
                && (v(c1) - v(c2)).norm() <= tol.max(0.03 * m1.abs())
                && close_r(*m1, *m2)
                && close_r(*n1, *n2)
        }
        _ => false,
    }
}

/// Merges adjacent segments describing the same surface whose union still fits one primitive
/// about as well as the parts.
pub(super) fn merge_adjacent(
    mesh: &Mesh,
    normals: &[Vec3],
    nb: &[[u32; 3]],
    mut segs: Vec<Segment>,
    tol: f64,
) -> Vec<Segment> {
    // pairs whose union was already refused (identified by first triangle and size of both
    // parts): later passes must not repeat those fits
    let mut refused: std::collections::HashSet<(u32, u32, usize, usize)> = std::collections::HashSet::new();
    let key_of = |segs: &[Segment], a: usize, b: usize| {
        (
            segs[a].triangles.first().copied().unwrap_or(u32::MAX),
            segs[b].triangles.first().copied().unwrap_or(u32::MAX),
            segs[a].triangles.len(),
            segs[b].triangles.len(),
        )
    };
    for _pass in 0..64 {
        let mut owner = vec![u32::MAX; mesh.triangle_count()];
        for (i, s) in segs.iter().enumerate() {
            for &t in &s.triangles {
                owner[t as usize] = i as u32;
            }
        }
        // adjacent pairs of the same kind, by shared edge count (strongest first)
        let mut pairs: std::collections::HashMap<(u32, u32), usize> = std::collections::HashMap::new();
        for (t, nn) in nb.iter().enumerate() {
            let a = owner[t];
            if a == u32::MAX {
                continue;
            }
            for &o in nn {
                if o == u32::MAX {
                    continue;
                }
                let b = owner[o as usize];
                if b != u32::MAX && a < b {
                    *pairs.entry((a, b)).or_default() += 1;
                }
            }
        }
        let mut cand: Vec<((u32, u32), usize)> = pairs
            .into_iter()
            .filter(|((a, b), _)| match (segs.get(*a as usize), segs.get(*b as usize)) {
                (Some(x), Some(y)) => same_surface(&x.primitive, &y.primitive, tol),
                _ => false,
            })
            .collect();
        cand.sort_by(|x, y| y.1.cmp(&x.1).then(x.0.cmp(&y.0)));
        let mut busy = vec![false; segs.len()];
        let mut merged_any = false;
        for ((a, b), _) in cand {
            let (a, b) = (a as usize, b as usize);
            if busy[a] || busy[b] || refused.contains(&key_of(&segs, a, b)) {
                continue;
            }
            let mut tris = segs[a].triangles.clone();
            tris.extend_from_slice(&segs[b].triangles);
            let Some(f) = fit_tris(mesh, normals, &tris) else {
                refused.insert(key_of(&segs, a, b));
                continue;
            };
            // the union must fit about as well as its parts did (not merely within the growth
            // tolerance), so merging never trades accuracy for fewer regions
            let limit = (segs[a].rms.max(segs[b].rms) * 1.25).max(0.25 * tol).min(tol);
            if f.rms > limit || f.primitive.kind() != segs[a].primitive.kind() {
                refused.insert(key_of(&segs, a, b));
                continue;
            }
            tris.sort_unstable();
            segs[a] = Segment { triangles: tris, primitive: f.primitive, rms: f.rms };
            segs[b].triangles.clear();
            // one merge per region and pass; the merged region meets its other neighbours next pass
            busy[a] = true;
            busy[b] = true;
            merged_any = true;
        }
        segs.retain(|s| !s.triangles.is_empty());
        if !merged_any {
            break;
        }
    }
    segs
}
