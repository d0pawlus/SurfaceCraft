//! Segmentation of a mesh into regions that each follow one primitive ("auto segment").
//!
//! Fit-and-grow, in the spirit of Efficient RANSAC (Schnabel et al. 2007) but deterministic and
//! driven by mesh connectivity: seeds are taken from the flattest unassigned triangles; a seed
//! patch (its smooth neighbourhood) gets an automatic primitive fit; the region then grows
//! across neighbouring triangles whose corners lie within the tolerance of the primitive and
//! whose normal agrees with it, refitting as it doubles. Regions below `min_triangles` are
//! released. Everything is mesh-side (design doc §5).
use crate::auto::fit_auto;
use crate::primitive::{Primitive, PrimitiveKind};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use surfacecraft_geom::{Mesh, Point3, Vec3};

/// Segmentation settings.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(default)]
pub struct SegmentOptions {
    /// Distance tolerance of a triangle corner to its region's primitive (mm).
    pub tolerance: f64,
    /// Largest angle between a triangle normal and the primitive normal (degrees).
    pub max_angle_deg: f64,
    /// Smallest region kept (triangles).
    pub min_triangles: usize,
    /// Size of the seed patch (triangles).
    pub seed_size: usize,
}

impl Default for SegmentOptions {
    fn default() -> Self {
        Self { tolerance: 0.2, max_angle_deg: 20.0, min_triangles: 30, seed_size: 60 }
    }
}

/// One region found.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct Segment {
    /// Triangle indices, sorted.
    pub triangles: Vec<u32>,
    /// Primitive of the region.
    pub primitive: Primitive,
    /// RMS of the region's vertices to it (mm).
    pub rms: f64,
}

/// Vertices and their normals of a set of triangles.
#[must_use]
pub fn region_points(mesh: &Mesh, normals: &[Vec3], tris: &[u32]) -> (Vec<Point3>, Vec<Vec3>) {
    // sort + dedup the corner ids: cost proportional to the region, not to the whole mesh (seeds
    // call this thousands of times on scans of millions of vertices)
    let mut ids: Vec<u32> =
        tris.iter().filter_map(|&t| mesh.triangles.get(t as usize)).flat_map(|t| t.iter().copied()).collect();
    ids.sort_unstable();
    ids.dedup();
    let mut p = Vec::with_capacity(ids.len());
    let mut n = Vec::with_capacity(ids.len());
    for vi in ids {
        if let (Some(pp), Some(nn)) = (mesh.positions.get(vi as usize), normals.get(vi as usize)) {
            p.push(*pp);
            n.push(*nn);
        }
    }
    (p, n)
}

/// Grows a set of triangles from `seeds` across edges whose dihedral angle is below
/// `max_angle_deg` ("select smooth region").
#[must_use]
pub fn grow_smooth(mesh: &Mesh, seeds: &[u32], max_angle_deg: f64) -> Vec<u32> {
    let nb = mesh.triangle_neighbours();
    let cos = max_angle_deg.to_radians().cos();
    let mut inside = vec![false; mesh.triangle_count()];
    let mut q: VecDeque<u32> = VecDeque::new();
    for &s in seeds {
        if let Some(f) = inside.get_mut(s as usize)
            && !*f
        {
            *f = true;
            q.push_back(s);
        }
    }
    while let Some(t) = q.pop_front() {
        let Some(nt) = mesh.face_normal(t as usize) else { continue };
        for &o in nb.get(t as usize).into_iter().flatten() {
            if o == u32::MAX || inside.get(o as usize).copied().unwrap_or(true) {
                continue;
            }
            if mesh.face_normal(o as usize).is_some_and(|no| no.dot(&nt) >= cos) {
                if let Some(f) = inside.get_mut(o as usize) {
                    *f = true;
                }
                q.push_back(o);
            }
        }
    }
    inside.iter().enumerate().filter(|(_, f)| **f).map(|(i, _)| i as u32).collect()
}

fn accepts(mesh: &Mesh, prim: &Primitive, t: usize, tol: f64, cos: f64) -> bool {
    let Some(corners) = mesh.triangle(t) else { return false };
    if corners.iter().any(|c| prim.distance(c).abs() > tol) {
        return false;
    }
    let (Some(n), Some(c)) = (mesh.face_normal(t), mesh.face_centroid(t)) else { return false };
    prim.normal(&c).dot(&n).abs() >= cos
}

/// Points used for one fit at most (evenly strided); the final RMS is over every point.
const FIT_POINTS: usize = 4000;

/// Automatic fit of a set of triangles on at most [`FIT_POINTS`] of their vertices.
fn fit_tris(mesh: &Mesh, normals: &[Vec3], tris: &[u32]) -> Option<crate::primitive::PrimitiveFit> {
    let (p, n) = region_points(mesh, normals, tris);
    if p.len() <= FIT_POINTS {
        return fit_auto(&p, &n).map(|a| a.best);
    }
    let stride = p.len().div_ceil(FIT_POINTS);
    let ps: Vec<Point3> = p.iter().step_by(stride).copied().collect();
    let ns: Vec<Vec3> = n.iter().step_by(stride).copied().collect();
    let mut f = fit_auto(&ps, &ns)?.best;
    let sq: f64 = p.iter().map(|q| f.primitive.distance(q).powi(2)).sum();
    f.rms = (sq / p.len() as f64).sqrt();
    f.count = p.len();
    Some(f)
}

/// Segments `mesh` into primitive regions.
///
/// Robust to real scans (P1): seed patches are grown to a fixed *area* (the median triangle area
/// times `seed_size`), so dense and sparse parts of a scan get seeds of the same physical size;
/// fits use at most [`FIT_POINTS`] points; adjacent regions whose union is still one primitive of
/// the same kind within the tolerance are merged (a face cut by a hole or a seam comes out whole).
#[must_use]
pub fn segment(mesh: &Mesh, opts: &SegmentOptions) -> Vec<Segment> {
    let nt = mesh.triangle_count();
    if nt == 0 || mesh.validate().is_err() {
        return Vec::new();
    }
    let tol = if opts.tolerance.is_finite() && opts.tolerance > 0.0 { opts.tolerance } else { 0.2 };
    let cos = opts.max_angle_deg.clamp(1.0, 89.0).to_radians().cos();
    let nb = mesh.triangle_neighbours();
    let normals = mesh.vertex_normals();
    let fnorm: Vec<Option<Vec3>> = (0..nt).map(|t| mesh.face_normal(t)).collect();
    let areas: Vec<f64> = (0..nt).map(|t| mesh.face_area(t)).collect();
    let seed_tris = opts.seed_size.max(8);
    let seed_area = {
        let mut a: Vec<f64> = areas.iter().copied().filter(|x| *x > 0.0).collect();
        let k = a.len() / 2;
        let med = if a.is_empty() { 0.0 } else { *a.select_nth_unstable_by(k, f64::total_cmp).1 };
        med * seed_tris as f64
    };
    // flatness of each triangle: smallest = flattest neighbourhood (best seeds)
    let mut order: Vec<(f64, u32)> = (0..nt)
        .map(|t| {
            let n = fnorm[t].unwrap_or_else(Vec3::z);
            let worst = nb[t]
                .iter()
                .filter(|&&o| o != u32::MAX)
                .filter_map(|&o| fnorm.get(o as usize).copied().flatten())
                .map(|m| 1.0 - m.dot(&n))
                .fold(0.0, f64::max);
            (worst, t as u32)
        })
        .collect();
    order.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut owner = vec![u32::MAX; nt];
    let mut out: Vec<Segment> = Vec::new();
    let mut tried = vec![false; nt];
    // generation-stamped markers shared by every seed (no per-seed allocation of `nt` flags)
    let mut mark = vec![0u32; nt];
    let mut gen_id: u32 = 0;
    let mut next_gen = |mark: &mut Vec<u32>| {
        gen_id = gen_id.wrapping_add(1);
        if gen_id == 0 {
            mark.iter_mut().for_each(|m| *m = 0);
            gen_id = 1;
        }
        gen_id
    };
    let max_regions = 10_000;
    for &(_, seed) in &order {
        if out.len() >= max_regions {
            break;
        }
        let s = seed as usize;
        if owner[s] != u32::MAX || tried[s] {
            continue;
        }
        let g = next_gen(&mut mark);
        // seed patch: smooth unassigned neighbourhood, breadth first, up to the seed area
        let mut patch = vec![seed];
        let mut area = areas[s];
        mark[s] = g;
        let mut q = VecDeque::from([seed]);
        while let Some(t) = q.pop_front() {
            if (patch.len() >= seed_tris && area >= seed_area) || patch.len() >= 8 * seed_tris {
                break;
            }
            let Some(n0) = fnorm[t as usize] else { continue };
            for &o in &nb[t as usize] {
                let oi = o as usize;
                if o == u32::MAX || mark[oi] == g || owner[oi] != u32::MAX {
                    continue;
                }
                if fnorm[oi].is_some_and(|m| m.dot(&n0) > 0.94) {
                    mark[oi] = g;
                    patch.push(o);
                    area += areas[oi];
                    q.push_back(o);
                }
            }
        }
        for &t in &patch {
            tried[t as usize] = true;
        }
        if patch.len() < 8 {
            continue;
        }
        let Some(mut fit) = fit_tris(mesh, &normals, &patch) else { continue };
        if fit.rms > tol {
            continue;
        }
        // grow, refitting each time the region doubles
        let g = next_gen(&mut mark);
        let id = out.len() as u32;
        let mut region: Vec<u32> = Vec::new();
        let mut q: VecDeque<u32> = VecDeque::new();
        for &t in &patch {
            if accepts(mesh, &fit.primitive, t as usize, tol, cos) {
                mark[t as usize] = g;
                region.push(t);
                q.push_back(t);
            }
        }
        let mut next_refit = region.len().max(8) * 2;
        while let Some(t) = q.pop_front() {
            for &o in &nb[t as usize] {
                let oi = o as usize;
                if o == u32::MAX || mark[oi] == g || owner[oi] != u32::MAX {
                    continue;
                }
                if accepts(mesh, &fit.primitive, oi, tol, cos) {
                    mark[oi] = g;
                    region.push(o);
                    q.push_back(o);
                }
            }
            if region.len() >= next_refit {
                next_refit *= 2;
                if let Some(f) = fit_tris(mesh, &normals, &region)
                    && f.rms <= tol
                {
                    fit = f;
                }
            }
        }
        if region.len() < opts.min_triangles {
            continue;
        }
        if let Some(f) = fit_tris(mesh, &normals, &region)
            && f.rms <= tol
        {
            fit = f;
        }
        for &t in &region {
            owner[t as usize] = id;
        }
        region.sort_unstable();
        out.push(Segment { triangles: region, primitive: fit.primitive, rms: fit.rms });
    }
    let out = relax_boundaries(mesh, &normals, &nb, out, tol);
    merge_adjacent(mesh, &normals, &nb, out, tol)
}

/// Mean corner distance of triangle `t` to `prim`, plus a small normal-disagreement term (tie
/// break); `None` for a degenerate triangle.
fn boundary_cost(mesh: &Mesh, prim: &Primitive, t: usize, tol: f64) -> Option<f64> {
    let corners = mesh.triangle(t)?;
    let (n, c) = (mesh.face_normal(t)?, mesh.face_centroid(t)?);
    let d: f64 = corners.iter().map(|p| prim.distance(p).abs()).sum::<f64>() / 3.0;
    Some(d + 0.1 * tol * (1.0 - prim.normal(&c).dot(&n).abs()))
}

/// Boundary relaxation: region growing accepts anything within the tolerance, so a region bleeds
/// over a tangent boundary (a fillet over the neighbouring plane stays within 0.2 mm for about
/// 1.4 mm) and the strip biases its fit. Each pass moves every triangle that borders another
/// region to the neighbouring region whose primitive it fits best, then refits the regions that
/// changed. Deterministic (moves are decided on the previous state), a few passes, boundary-sized.
fn relax_boundaries(mesh: &Mesh, normals: &[Vec3], nb: &[[u32; 3]], mut segs: Vec<Segment>, tol: f64) -> Vec<Segment> {
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
        let mut moves: Vec<(u32, u32)> = Vec::new();
        for (t, nn) in nb.iter().enumerate() {
            let a = owner[t];
            if a == u32::MAX {
                continue;
            }
            let mut cands: Vec<u32> = nn.iter().filter(|&&o| o != u32::MAX).map(|&o| owner[o as usize]).collect();
            cands.retain(|&b| b != a && b != u32::MAX);
            if cands.is_empty() {
                continue;
            }
            cands.sort_unstable();
            cands.dedup();
            let Some(mut best_cost) = segs.get(a as usize).and_then(|s| boundary_cost(mesh, &s.primitive, t, tol))
            else {
                continue;
            };
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
            if best != a {
                moves.push((t as u32, best));
            }
        }
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
        for (i, tris) in members.into_iter().enumerate() {
            if !touched[i] {
                continue;
            }
            if let Some(f) = fit_tris(mesh, normals, &tris)
                && f.rms <= tol
            {
                segs[i].primitive = f.primitive;
                segs[i].rms = f.rms;
            }
            segs[i].triangles = tris;
        }
    }
    segs.retain(|s| !s.triangles.is_empty());
    segs
}

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
fn merge_adjacent(mesh: &Mesh, normals: &[Vec3], nb: &[[u32; 3]], mut segs: Vec<Segment>, tol: f64) -> Vec<Segment> {
    for _pass in 0..4 {
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
            if busy[a] || busy[b] {
                continue;
            }
            let mut tris = segs[a].triangles.clone();
            tris.extend_from_slice(&segs[b].triangles);
            let Some(f) = fit_tris(mesh, normals, &tris) else { continue };
            // the union must fit about as well as its parts did (not merely within the growth
            // tolerance), so merging never trades accuracy for fewer regions
            let limit = (segs[a].rms.max(segs[b].rms) * 1.25).max(0.25 * tol).min(tol);
            if f.rms > limit || f.primitive.kind() != segs[a].primitive.kind() {
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

// How many segments of each kind.
#[must_use]
pub fn kinds(segs: &[Segment]) -> Vec<(PrimitiveKind, usize)> {
    let mut v: Vec<(PrimitiveKind, usize)> = Vec::new();
    for s in segs {
        let k = s.primitive.kind();
        match v.iter_mut().find(|(kk, _)| *kk == k) {
            Some(e) => e.1 += 1,
            None => v.push((k, 1)),
        }
    }
    v.sort();
    v
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Rng, synth};

    /// Gate M2 (Region): the bracket (6 planar faces, a cylindrical boss, a spherical dome)
    /// with σ = 0.05 mm noise segments into the right kinds, each region within RMS 0.1 mm.
    #[test]
    fn gate_m2_auto_segment_bracket() {
        let mut m = synth::bracket();
        synth::add_noise(&mut m, 0.05, &mut Rng::new(4));
        let segs = segment(&m, &SegmentOptions::default());
        let k = kinds(&segs);
        eprintln!(
            "gate_m2 segment: {k:?}, rms {:?}",
            segs.iter().map(|s| (s.rms * 1e4).round() / 1e4).collect::<Vec<_>>()
        );
        assert_eq!(k, vec![(PrimitiveKind::Plane, 6), (PrimitiveKind::Sphere, 1), (PrimitiveKind::Cylinder, 1)]);
        assert!(segs.iter().all(|s| s.rms <= 0.1));
        // nearly every triangle is assigned
        let assigned: usize = segs.iter().map(|s| s.triangles.len()).sum();
        assert!(assigned as f64 >= 0.9 * m.triangle_count() as f64, "{assigned} of {}", m.triangle_count());
    }

    #[test]
    fn smooth_growth_stops_at_sharp_edges() {
        let m = synth::cuboid(Point3::origin(), Vec3::new(10.0, 10.0, 10.0), 4);
        let face = grow_smooth(&m, &[0], 10.0);
        assert_eq!(face.len(), 4 * 4 * 2);
        assert!(segment(&Mesh::default(), &SegmentOptions::default()).is_empty());
    }
}
