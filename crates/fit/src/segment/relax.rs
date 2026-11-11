//! Boundary relaxation between neighbouring regions.
use super::fits::{refit_kind_points, rms_of};
use super::{Segment, region_points};
use crate::primitive::Primitive;
use surfacecraft_geom::{Mesh, Vec3};

/// Mean corner distance of triangle `t` to `prim`, plus a small normal-disagreement term (tie
/// break); `None` for a degenerate triangle.
fn boundary_cost(mesh: &Mesh, prim: &Primitive, t: usize, tol: f64) -> Option<f64> {
    let corners = mesh.triangle(t)?;
    let (n, c) = (mesh.face_normal(t)?, mesh.face_centroid(t)?);
    let d: f64 = corners.iter().map(|p| prim.distance(p).abs()).sum::<f64>() / 3.0;
    Some(d + 0.1 * tol * (1.0 - prim.normal(&c).dot(&n).abs()))
}

/// The neighbouring region whose primitive triangle `t` fits better than its own (`None`: it
/// stays where it is, or is not on a region boundary).
fn better_owner(mesh: &Mesh, nb: &[[u32; 3]], segs: &[Segment], owner: &[u32], t: usize, tol: f64) -> Option<u32> {
    let a = owner[t];
    if a == u32::MAX {
        return None;
    }
    let mut cands: Vec<u32> = nb[t].iter().filter(|&&o| o != u32::MAX).map(|&o| owner[o as usize]).collect();
    cands.retain(|&b| b != a && b != u32::MAX);
    cands.sort_unstable();
    cands.dedup();
    let mut best_cost = segs.get(a as usize).and_then(|s| boundary_cost(mesh, &s.primitive, t, tol))?;
    let mut best = a;
    for b in cands {
        let cost = segs.get(b as usize).and_then(|s| boundary_cost(mesh, &s.primitive, t, tol));
        if let Some(c) = cost
            && c < best_cost
        {
            best_cost = c;
            best = b;
        }
    }
    (best != a).then_some(best)
}

/// What the refit of a region that took or lost triangles came to.
enum Refit {
    /// Refitted within the tolerance.
    Fitted(Primitive, f64),
    /// The refit was rejected; the primitive stays, its RMS must describe the triangles it now
    /// has (`None` for an empty set).
    Kept(Option<f64>),
}

/// Refit of `seg` (keeping its kind) on its new triangles; the points are collected once.
fn refit_region(mesh: &Mesh, normals: &[Vec3], tris: &[u32], seg: &Segment, tol: f64) -> Refit {
    let (p, n) = region_points(mesh, normals, tris);
    match refit_kind_points(&p, &n, seg.primitive.kind()) {
        Some(f) if f.rms <= tol => Refit::Fitted(f.primitive, f.rms),
        _ => Refit::Kept((!p.is_empty()).then(|| rms_of(&p, &seg.primitive))),
    }
}

/// Boundary relaxation: region growing accepts anything within the tolerance, so a region bleeds
/// over a tangent boundary (a fillet over the neighbouring plane stays within 0.2 mm for about
/// 1.4 mm) and the strip biases its fit. Each pass moves every triangle that borders another
/// region to the neighbouring region whose primitive it fits best, then refits the regions that
/// changed. Deterministic (moves are decided on the previous state), a few passes, boundary-sized.
pub(super) fn relax_boundaries(
    mesh: &Mesh,
    normals: &[Vec3],
    nb: &[[u32; 3]],
    mut segs: Vec<Segment>,
    tol: f64,
) -> Vec<Segment> {
    const PASSES: usize = 8;
    if segs.len() < 2 {
        return segs;
    }
    let mut owner = vec![u32::MAX; mesh.triangle_count()];
    for (i, s) in segs.iter().enumerate() {
        for &t in &s.triangles {
            owner[t as usize] = i as u32;
        }
    }
    for _pass in 0..PASSES {
        // decided on the previous state, so every triangle is independent of the others
        let moves: Vec<(u32, u32)> = surfacecraft_geom::par::map(nb.len(), |t| {
            better_owner(mesh, nb, &segs, &owner, t, tol).map(|b| (t as u32, b))
        })
        .into_iter()
        .flatten()
        .collect();
        if moves.is_empty() {
            break;
        }
        let mut touched = vec![false; segs.len()];
        for &(t, b) in &moves {
            touched[owner[t as usize] as usize] = true;
            touched[b as usize] = true;
            owner[t as usize] = b;
        }
        let mut members: Vec<Vec<u32>> = vec![Vec::new(); segs.len()];
        for (t, &o) in owner.iter().enumerate() {
            if o != u32::MAX && touched[o as usize] {
                members[o as usize].push(t as u32);
            }
        }
        // regions keep their kind (a boundary move does not change it); independent refits run in parallel
        let jobs: Vec<(usize, Vec<u32>)> = members.into_iter().enumerate().filter(|(i, _)| touched[*i]).collect();
        let fits =
            surfacecraft_geom::par::map_slice(&jobs, |(i, tris)| refit_region(mesh, normals, tris, &segs[*i], tol));
        for ((i, tris), refit) in jobs.into_iter().zip(fits) {
            match refit {
                Refit::Fitted(primitive, rms) => {
                    segs[i].primitive = primitive;
                    segs[i].rms = rms;
                }
                Refit::Kept(Some(rms)) => segs[i].rms = rms,
                Refit::Kept(None) => {}
            }
            segs[i].triangles = tris;
        }
    }
    segs.retain(|s| !s.triangles.is_empty());
    segs
}
