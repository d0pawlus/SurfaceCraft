//! Working-mesh preparation, level chain, fallback layout and layout statistics.
use super::field::boundary_neighbours;
use super::{MAX_WORK_TRIANGLES, PolyMesh};
use std::collections::BTreeMap;
use surfacecraft_geom::edit::{DecimateOptions, compact, decimate};
use surfacecraft_geom::{Bvh, Mesh, synth};

pub(super) fn euler_of(m: &PolyMesh) -> i64 {
    let mut e = std::collections::BTreeSet::new();
    for f in &m.faces {
        for k in 0..f.len() {
            let (a, b) = (f[k], f[(k + 1) % f.len()]);
            e.insert((a.min(b), a.max(b)));
        }
    }
    m.vertices.len() as i64 - e.len() as i64 + m.faces.len() as i64
}

pub(super) fn euler_of_mesh(m: &Mesh) -> i64 {
    let mut e = std::collections::BTreeSet::new();
    for t in &m.triangles {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            e.insert((a.min(b), a.max(b)));
        }
    }
    let used: std::collections::BTreeSet<u32> = m.triangles.iter().flatten().copied().collect();
    used.len() as i64 - e.len() as i64 + m.triangles.len() as i64
}

pub(super) fn is_closed(m: &Mesh) -> bool {
    boundary_neighbours(m).iter().all(Vec::is_empty)
}

pub(super) fn irregular_fraction(m: &PolyMesh) -> f64 {
    let mut nb: Vec<std::collections::BTreeSet<u32>> = vec![Default::default(); m.vertices.len()];
    let mut boundary: BTreeMap<(u32, u32), u32> = BTreeMap::new();
    for f in &m.faces {
        for k in 0..f.len() {
            let (a, b) = (f[k], f[(k + 1) % f.len()]);
            nb[a as usize].insert(b);
            nb[b as usize].insert(a);
            *boundary.entry((a.min(b), a.max(b))).or_insert(0) += 1;
        }
    }
    let mut on_boundary = vec![false; m.vertices.len()];
    for ((a, b), c) in boundary {
        if c == 1 {
            on_boundary[a as usize] = true;
            on_boundary[b as usize] = true;
        }
    }
    let interior: Vec<usize> = (0..nb.len()).filter(|&i| !on_boundary[i] && !nb[i].is_empty()).collect();
    if interior.is_empty() {
        return 0.0;
    }
    interior.iter().filter(|&&i| nb[i].len() != 4).count() as f64 / interior.len() as f64
}

/// The working mesh: welded, cleaned, decimated to roughly 12 triangles per wanted face and
/// refined so no edge is longer than ρ/2 (the extraction needs several vertices per lattice
/// cell), its vertices on the original surface.
pub(super) fn working_mesh(mesh: &Mesh, rho: f64, target_faces: usize) -> Mesh {
    let mut m = mesh.clone();
    m.weld(1e-9);
    m.remove_degenerate(0.0);
    compact(&mut m);
    let want = (target_faces * 12).clamp(300, MAX_WORK_TRIANGLES);
    let mut changed = false;
    if m.triangle_count() > want {
        let opts = DecimateOptions {
            target_triangles: want,
            max_error: None,
            feature_angle_deg: 181.0,
            preserve_boundary: true,
        };
        decimate(&mut m, &opts);
        changed = true;
    }
    let before = m.triangle_count();
    let refined = synth::refine(&m, |_| rho / 2.0);
    if refined.triangle_count() != before && refined.triangle_count() <= MAX_WORK_TRIANGLES * 2 {
        m = refined;
        changed = true;
    }
    if changed {
        let bvh = Bvh::build(mesh);
        for p in &mut m.positions {
            if let Some(h) = bvh.closest(p, f64::INFINITY) {
                *p = h.point;
            }
        }
    }
    m
}

/// Coarse-to-fine chain of levels, finest first.
pub(super) fn build_levels(fine: &Mesh) -> Vec<Mesh> {
    let mut levels = vec![fine.clone()];
    while let Some(last) = levels.last() {
        let t = last.triangle_count() / 3;
        if last.triangle_count() < 700 || levels.len() >= 6 {
            break;
        }
        let mut c = last.clone();
        let opts =
            DecimateOptions { target_triangles: t, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
        decimate(&mut c, &opts);
        if c.triangle_count() >= last.triangle_count() || c.triangle_count() < 100 {
            break;
        }
        levels.push(c);
    }
    levels
}

/// Fallback layout: a coarse decimation whose every triangle becomes three quads.
pub(super) fn fallback(work: &Mesh, target_faces: usize, bvh: &Bvh) -> PolyMesh {
    let mut m = work.clone();
    let t = (target_faces / 3).max(8);
    if m.triangle_count() > t {
        let opts =
            DecimateOptions { target_triangles: t, max_error: None, feature_angle_deg: 181.0, preserve_boundary: true };
        decimate(&mut m, &opts);
    }
    compact(&mut m);
    let nrm = m.vertex_normals();
    let mut vertices = Vec::with_capacity(m.positions.len());
    let mut normals = Vec::with_capacity(m.positions.len());
    for (p, n) in m.positions.iter().zip(&nrm) {
        match bvh.closest(p, f64::INFINITY) {
            Some(h) => vertices.push(h.point),
            None => vertices.push(*p),
        }
        normals.push(*n);
    }
    PolyMesh { vertices, normals, faces: m.triangles.iter().map(|t| t.to_vec()).collect() }
}
