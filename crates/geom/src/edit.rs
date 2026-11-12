//! Mesh editing for real scans (P1): components, outliers, holes, decimation, smoothing.
//!
//! Everything here is mesh-side computation (design doc §5). Each function validates the mesh
//! first and does nothing on an invalid one, so later indexing stays in range.
//!
//! - small components: connectivity through shared vertices;
//! - outliers: spikes, i.e. vertices far off the plane of their one-ring compared with the ring's
//!   size (creases and corners stay well below the threshold);
//! - holes: boundary loops triangulated by ear clipping in the loop's mean plane; the smooth
//!   variant refines the patch (centroid splits + Delaunay flips) and fairs it with the
//!   surrounding ring held fixed (membrane, then thin-plate iterations);
//! - decimation: quadric error metric edge collapse (Garland & Heckbert 1997) with boundary and
//!   feature-edge constraint planes, link-condition and normal-flip checks;
//! - smoothing: Taubin λ|μ (1995), boundary and sharp-edge vertices optionally fixed.
use crate::{Mesh, Point3, Vec3};
use nalgebra::Matrix3;
use std::collections::HashMap;

pub use crate::decimate::{DecimateOptions, decimate};
pub(crate) use crate::decimate::{Guard, decimate_guarded};
#[cfg(test)]
pub(crate) use crate::holes::fill_holes_with;
pub use crate::holes::{FillReport, MAX_FILL_EDGES, boundary_loops, fill_holes};

/// Removes every triangle on an edge that three or more triangles share, until no such edge is
/// left; returns how many triangles were removed. What stays is a surface whose edges have one or
/// two triangles each, with small holes where the extra sheets met it (fill them afterwards).
pub fn remove_non_manifold(mesh: &mut Mesh) -> usize {
    let before = mesh.triangles.len();
    loop {
        let mut uses: HashMap<(u32, u32), u32> = HashMap::with_capacity(mesh.triangles.len() * 3 / 2);
        for t in &mesh.triangles {
            for k in 0..3 {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        let on_bad_edge = |t: &[u32; 3]| {
            (0..3).any(|k| {
                let (a, b) = (t[k], t[(k + 1) % 3]);
                uses.get(&(a.min(b), a.max(b))).is_some_and(|c| *c > 2)
            })
        };
        let kept = mesh.triangles.len();
        mesh.triangles.retain(|t| !on_bad_edge(t));
        if mesh.triangles.len() == kept {
            break;
        }
    }
    compact(mesh);
    before - mesh.triangles.len()
}

/// Drops vertices no triangle uses; returns how many were dropped.
pub fn compact(mesh: &mut Mesh) -> usize {
    if mesh.validate().is_err() {
        return 0;
    }
    let mut used = vec![u32::MAX; mesh.positions.len()];
    let mut out = Vec::with_capacity(mesh.positions.len());
    for t in &mut mesh.triangles {
        for v in t.iter_mut() {
            let i = *v as usize;
            if used[i] == u32::MAX {
                used[i] = out.len() as u32;
                out.push(mesh.positions[i]);
            }
            *v = used[i];
        }
    }
    let removed = mesh.positions.len() - out.len();
    mesh.positions = out;
    removed
}

/// Connected components (through shared vertices) as lists of triangle indices, largest first.
#[must_use]
pub fn components(mesh: &Mesh) -> Vec<Vec<u32>> {
    if mesh.validate().is_err() {
        return Vec::new();
    }
    // union-find over vertices
    let mut parent: Vec<u32> = (0..mesh.positions.len() as u32).collect();
    fn find(p: &mut [u32], mut x: u32) -> u32 {
        while p[x as usize] != x {
            let up = p[p[x as usize] as usize];
            p[x as usize] = up;
            x = up;
        }
        x
    }
    for t in &mesh.triangles {
        let a = find(&mut parent, t[0]);
        for &v in &t[1..] {
            let b = find(&mut parent, v);
            if a != b {
                parent[b as usize] = a;
            }
        }
    }
    let mut groups: HashMap<u32, Vec<u32>> = HashMap::new();
    for (i, t) in mesh.triangles.iter().enumerate() {
        let r = find(&mut parent, t[0]);
        groups.entry(r).or_default().push(i as u32);
    }
    let mut v: Vec<Vec<u32>> = groups.into_values().collect();
    v.sort_by(|a, b| b.len().cmp(&a.len()).then(a.first().cmp(&b.first())));
    v
}

/// Removes components with fewer than `max(min_triangles, min_fraction × largest)` triangles.
/// Returns the number of triangles removed.
pub fn remove_small_components(mesh: &mut Mesh, min_fraction: f64, min_triangles: usize) -> usize {
    let comps = components(mesh);
    let Some(largest) = comps.first().map(Vec::len) else { return 0 };
    let f = if min_fraction.is_finite() { min_fraction.clamp(0.0, 1.0) } else { 0.0 };
    let keep_at = ((largest as f64 * f).ceil() as usize).max(min_triangles).min(largest);
    let mut keep = vec![false; mesh.triangles.len()];
    for c in comps.iter().filter(|c| c.len() >= keep_at) {
        for &t in c {
            keep[t as usize] = true;
        }
    }
    let before = mesh.triangles.len();
    let mut k = keep.iter();
    mesh.triangles.retain(|_| k.next().copied().unwrap_or(false));
    compact(mesh);
    before - mesh.triangles.len()
}

/// Vertex → neighbouring vertices (one ring, unordered, no duplicates).
pub(crate) fn vertex_rings(mesh: &Mesh) -> Vec<Vec<u32>> {
    let mut ring: Vec<Vec<u32>> = vec![Vec::new(); mesh.positions.len()];
    for t in &mesh.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            ring[a as usize].push(b);
            ring[b as usize].push(a);
        }
    }
    for r in &mut ring {
        r.sort_unstable();
        r.dedup();
    }
    ring
}

/// A ring point this many times the median distance from the ring's middle may be a spike
/// itself.
const RING_STRAY_FACTOR: f64 = 2.0;
/// The points a plane is fitted to are spread enough when their second principal extent is at
/// least this share of the first (points nearly in a line give no plane).
const RING_MIN_SPREAD: f64 = 0.2;

/// Least-squares plane of `pts` as (centroid, unit normal, mean distance to the centroid, ratio
/// of the second to the first principal extent).
fn ring_plane(pts: &[Point3]) -> Option<(Vec3, Vec3, f64, f64)> {
    if pts.len() < 3 {
        return None;
    }
    let c = pts.iter().fold(Vec3::zeros(), |a, p| a + p.coords) / pts.len() as f64;
    let mut cov = Matrix3::zeros();
    let mut radius = 0.0;
    for p in pts {
        let d = p.coords - c;
        cov += d * d.transpose();
        radius += d.norm();
    }
    radius /= pts.len() as f64;
    if !(radius > 0.0) {
        return None;
    }
    let e = cov.symmetric_eigen();
    let mut order = [0usize, 1, 2];
    order.sort_by(|&a, &b| e.eigenvalues[a].total_cmp(&e.eigenvalues[b]));
    let [least, mid, most] = order;
    let spread = (e.eigenvalues[mid].max(0.0) / e.eigenvalues[most].max(f64::MIN_POSITIVE)).sqrt();
    Some((c, e.eigenvectors.column(least).into_owned(), radius, spread))
}

/// Is `p` a spike that another spike in its ring `pts` hides? The other spike is a ring point
/// far from the rest (more than [`RING_STRAY_FACTOR`] × the median distance from the ring's
/// middle, the median of each coordinate). With it left out the rest must still span a plane,
/// `p` must be farther from that plane than `ratio` × the whole ring's radius, and the point
/// left out must be off the plane as well (a far point that lies in the plane is no spike: rings
/// of thin triangles have those).
fn hidden_spike(pts: &[Point3], p: &Point3, ratio: f64, ring_radius: f64) -> bool {
    let median = |mut v: Vec<f64>| {
        v.sort_by(f64::total_cmp);
        v.get(v.len() / 2).copied().unwrap_or(0.0)
    };
    let mid = Point3::new(
        median(pts.iter().map(|q| q.x).collect()),
        median(pts.iter().map(|q| q.y).collect()),
        median(pts.iter().map(|q| q.z).collect()),
    );
    let reach = RING_STRAY_FACTOR * median(pts.iter().map(|q| (q - mid).norm()).collect());
    pts.iter().enumerate().filter(|(_, q)| (*q - mid).norm() > reach).any(|(j, q)| {
        let rest: Vec<Point3> = pts.iter().enumerate().filter(|(k, _)| *k != j).map(|(_, r)| *r).collect();
        let Some((c, n, radius, spread)) = ring_plane(&rest) else { return false };
        spread >= RING_MIN_SPREAD
            && (p.coords - c).dot(&n).abs() > ratio * ring_radius
            && (q.coords - c).dot(&n).abs() > ratio * radius
    })
}

/// Removes spikes: vertices farther from the least-squares plane of their one ring than
/// `ratio` × the ring's radius, and spikes that a spike next to them hides from that test (see
/// [`hidden_spike`]). Triangles touching them are removed (the holes
/// left behind can be filled with [`fill_holes`]). Returns the number of triangles removed.
pub fn remove_outliers(mesh: &mut Mesh, ratio: f64) -> usize {
    if mesh.validate().is_err() {
        return 0;
    }
    let ratio = if ratio.is_finite() && ratio > 0.0 { ratio } else { 1.5 };
    let ring = vertex_rings(mesh);
    let positions = &mesh.positions;
    let bad: Vec<bool> = crate::par::map(ring.len(), |i| {
        let Some(r) = ring.get(i) else { return false };
        let pts: Vec<Point3> = r.iter().filter_map(|&j| positions.get(j as usize).copied()).collect();
        let (Some((c, n, radius, _)), Some(p)) = (ring_plane(&pts), positions.get(i)) else { return false };
        (p.coords - c).dot(&n).abs() > ratio * radius || hidden_spike(&pts, p, ratio, radius)
    });
    let before = mesh.triangles.len();
    mesh.triangles.retain(|t| !t.iter().any(|&v| bad[v as usize]));
    compact(mesh);
    before - mesh.triangles.len()
}

/// Taubin smoothing; `fix_boundary` keeps boundary vertices, `feature_angle_deg` keeps vertices on
/// edges sharper than that (None = smooth everything). Returns the largest displacement (mm).
pub fn smooth(mesh: &mut Mesh, iterations: usize, fix_boundary: bool, feature_angle_deg: Option<f64>) -> f64 {
    smooth_cancellable(mesh, iterations, fix_boundary, feature_angle_deg, &crate::Cancel::new()).unwrap_or(0.0)
}

/// [`smooth`] that gives up when `cancel` is set (checked every iteration): `None` then, with the
/// mesh left part-way smoothed, for the caller to discard.
pub fn smooth_cancellable(
    mesh: &mut Mesh,
    iterations: usize,
    fix_boundary: bool,
    feature_angle_deg: Option<f64>,
    cancel: &crate::Cancel,
) -> Option<f64> {
    if mesh.validate().is_err() {
        return Some(0.0);
    }
    let ring = vertex_rings(mesh);
    let mut fixed = vec![false; mesh.positions.len()];
    let nb = mesh.triangle_neighbours();
    let cos = feature_angle_deg.map(|a| a.clamp(1.0, 179.0).to_radians().cos());
    for (ti, t) in mesh.triangles.iter().enumerate() {
        for k in 0..3 {
            let o = nb[ti][k];
            let edge = [t[k], t[(k + 1) % 3]];
            let mark = if o == u32::MAX {
                fix_boundary
            } else if let Some(c) = cos {
                match (mesh.face_normal(ti), mesh.face_normal(o as usize)) {
                    (Some(a), Some(b)) => a.dot(&b) < c,
                    _ => false,
                }
            } else {
                false
            };
            if mark {
                for v in edge {
                    fixed[v as usize] = true;
                }
            }
        }
    }
    let start = mesh.positions.clone();
    let step = |pos: &mut Vec<Point3>, f: f64| {
        let old = pos.clone();
        for (i, r) in ring.iter().enumerate() {
            if fixed[i] || r.is_empty() {
                continue;
            }
            let c = r.iter().fold(Vec3::zeros(), |a, &j| a + old[j as usize].coords) / r.len() as f64;
            pos[i] = old[i] + (c - old[i].coords) * f;
        }
    };
    for _ in 0..iterations.min(1000) {
        if cancel.is_cancelled() {
            return None;
        }
        step(&mut mesh.positions, 0.5);
        step(&mut mesh.positions, -0.53);
    }
    Some(mesh.positions.iter().zip(&start).map(|(a, b)| (a - b).norm()).fold(0.0, f64::max))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_fin_on_an_edge_is_removed_with_the_triangles_it_touches() {
        // two triangles sharing an edge, a third triangle (the fin) on the same edge, one apart
        let mut m = Mesh {
            positions: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.5, 0.5, 1.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            triangles: vec![[0, 1, 2], [1, 3, 2], [1, 2, 4], [1, 5, 3]],
        };
        assert_eq!(remove_non_manifold(&mut m), 3);
        assert_eq!(m.triangles.len(), 1);
        assert_eq!(remove_non_manifold(&mut m), 0);
    }
    use crate::{Rng, synth};

    #[test]
    fn small_components_and_compact() {
        let mut m = synth::sphere(Point3::origin(), 10.0, 32, 16);
        m.weld(1e-9);
        let n = m.triangle_count();
        let mut debris = synth::plane(Point3::new(50.0, 0.0, 0.0), Vec3::x(), Vec3::y(), 1.0, 1.0, 2, 2);
        debris.weld(1e-9);
        m.append(&debris);
        assert_eq!(components(&m).len(), 2);
        assert_eq!(remove_small_components(&mut m, 0.01, 10), 8);
        assert_eq!(m.triangle_count(), n);
        assert!(m.positions.iter().all(|p| p.x < 20.0));
    }

    #[test]
    fn spikes_go_creases_stay() {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 8);
        m.weld(1e-9);
        let n = m.triangle_count();
        // creases and corners are not outliers
        assert_eq!(remove_outliers(&mut m.clone(), 1.5), 0);
        // a spike in the middle of a face is
        let top = m
            .positions
            .iter()
            .position(|p| (p.z - 20.0).abs() < 1e-9 && (p.x - 10.0).abs() < 1e-9 && (p.y - 10.0).abs() < 1e-9);
        let i = top.unwrap();
        m.positions[i].z += 15.0;
        let removed = remove_outliers(&mut m, 1.5);
        assert!(removed > 0 && removed <= 8, "{removed}");
        assert_eq!(m.triangle_count(), n - removed);
        // the hole it leaves is one loop, filled flat back to a closed box
        let loops = boundary_loops(&m);
        assert_eq!(loops.len(), 1);
        let r = fill_holes(&mut m, 100, false);
        assert_eq!(r.filled, 1);
        assert!(boundary_loops(&m).is_empty());
        assert!((m.area() - 6.0 * 400.0).abs() < 1e-6, "{}", m.area());
    }

    /// Two spikes next to each other are in each other's ring; neither may hide the other.
    #[test]
    fn neighbouring_spikes_both_go() {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 8);
        m.weld(1e-9);
        for (x, lift) in [(10.0, 10.4), (12.5, 7.8)] {
            let i = m
                .positions
                .iter()
                .position(|p| (p.z - 20.0).abs() < 1e-9 && (p.x - x).abs() < 1e-9 && (p.y - 10.0).abs() < 1e-9)
                .unwrap();
            m.positions[i].z += lift;
        }
        assert!(remove_outliers(&mut m, 1.5) > 0);
        let high = m.positions.iter().filter(|p| p.z > 20.5).count();
        assert_eq!(high, 0, "{high} spike vertices are left");
        // creases and corners of the clean box are still not outliers
        let mut clean = synth::cuboid(Point3::origin(), Vec3::new(20.0, 20.0, 20.0), 8);
        clean.weld(1e-9);
        assert_eq!(remove_outliers(&mut clean, 1.5), 0);
    }

    #[test]
    fn smooth_fill_follows_the_sphere() {
        let r = 20.0;
        let mut m = synth::sphere(Point3::origin(), r, 64, 32);
        m.weld(1e-9);
        // cut a cap out near the equator
        let keep: Vec<u32> = (0..m.triangle_count() as u32)
            .filter(|&t| {
                let c = m.face_centroid(t as usize).unwrap();
                !(c.x > 15.0)
            })
            .collect();
        let mut cut = m.submesh(&keep);
        let loops = boundary_loops(&cut);
        assert_eq!(loops.len(), 1);
        let mut flat = cut.clone();
        // deterministic: the same input gives the same fill
        let (mut x, mut y) = (cut.clone(), cut.clone());
        fill_holes(&mut x, 1000, true);
        fill_holes(&mut y, 1000, true);
        assert_eq!(x, y);
        fill_holes(&mut flat, 1000, false);
        let rep = fill_holes(&mut cut, 1000, true);
        assert_eq!(rep.filled, 1);
        assert!(boundary_loops(&cut).is_empty());
        let err = |mm: &Mesh| {
            (0..mm.triangle_count())
                .filter_map(|t| mm.face_centroid(t))
                .map(|p| (p.coords.norm() - r).abs())
                .fold(0.0, f64::max)
        };
        // a flat cap misses the sphere by its sagitta (≈ 5 mm); the faired cap follows it
        assert!(err(&flat) > 3.0, "{}", err(&flat));
        // a thin plate over the hole is a paraboloid-like cap, well inside the flat error
        assert!(err(&cut) < 0.35 * err(&flat), "smooth fill error {} vs flat {}", err(&cut), err(&flat));
    }

    #[test]
    fn decimation_keeps_shape_and_creases() {
        let mut m = synth::cuboid(Point3::origin(), Vec3::new(20.0, 10.0, 5.0), 24);
        m.weld(1e-9);
        let before = m.triangle_count();
        let removed = decimate(&mut m, &DecimateOptions { target_triangles: 200, ..Default::default() });
        assert!(removed > 0);
        assert!(m.triangle_count() <= before / 4, "{}", m.triangle_count());
        // flat faces collapse freely; the box stays a box (every vertex on its surface, corners kept)
        for p in &m.positions {
            let d =
                [p.x, 20.0 - p.x, p.y, 10.0 - p.y, p.z, 5.0 - p.z].into_iter().map(f64::abs).fold(f64::MAX, f64::min);
            assert!(d < 1e-6, "{p}");
        }
        assert!((m.area() - 2.0 * (200.0 + 100.0 + 50.0)).abs() < 1e-6);
        assert!(boundary_loops(&m).is_empty());
        // a noisy sphere: 10 % of the triangles stay within 0.3 mm of the radius
        let mut s = synth::sphere(Point3::origin(), 20.0, 128, 64);
        s.weld(1e-9);
        let n = s.triangle_count();
        decimate(&mut s, &DecimateOptions { target_triangles: n / 10, ..Default::default() });
        assert!(s.triangle_count() <= n / 10 + 2);
        let err = s.positions.iter().map(|p| (p.coords.norm() - 20.0).abs()).fold(0.0, f64::max);
        assert!(err < 0.3, "{err}");
    }

    #[test]
    fn taubin_reduces_noise_without_shrinking() {
        let mut m = synth::sphere(Point3::origin(), 20.0, 96, 48);
        m.weld(1e-9);
        synth::add_noise(&mut m, 0.1, &mut Rng::new(3));
        let err = |mm: &Mesh| {
            let v: Vec<f64> = mm.positions.iter().map(|p| p.coords.norm() - 20.0).collect();
            let mean = v.iter().sum::<f64>() / v.len() as f64;
            let rms = (v.iter().map(|x| x * x).sum::<f64>() / v.len() as f64).sqrt();
            (mean, rms)
        };
        let (_, rms0) = err(&m);
        smooth(&mut m, 10, true, None);
        let (mean, rms1) = err(&m);
        assert!(rms1 < 0.5 * rms0, "{rms0} → {rms1}");
        assert!(mean.abs() < 0.05, "shrinkage {mean}");
    }

    #[test]
    fn invalid_meshes_are_left_alone() {
        let mut m = Mesh { positions: vec![Point3::origin()], triangles: vec![[0, 1, 7]] };
        assert_eq!(compact(&mut m), 0);
        assert!(components(&m).is_empty());
        assert_eq!(remove_outliers(&mut m, 1.5), 0);
        assert_eq!(fill_holes(&mut m, 10, true), FillReport::default());
        assert_eq!(decimate(&mut m, &DecimateOptions::default()), 0);
        assert_eq!(smooth(&mut m, 3, true, Some(30.0)), 0.0);
        assert_eq!(m.triangles, vec![[0, 1, 7]]);
    }
}
