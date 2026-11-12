//! Decimation: quadric error metric edge collapse (Garland & Heckbert 1997) with boundary and
//! feature-edge constraint planes, link-condition and normal-flip checks, optionally guarded by
//! a measured deviation from the original surface. The collapse loop is in [`crate::collapse`];
//! with a guard a large mesh is first reduced in independent parts (see [`crate::partition`]).
use crate::collapse::{Collapser, Quadric, plane_quadric};
use crate::edit::compact;
use crate::{Bvh, Mesh, Point3, Vec3, tri};
use std::collections::HashMap;

/// Decimation settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DecimateOptions {
    /// Stop at this many triangles.
    pub target_triangles: usize,
    /// Never move the surface more than this (mm, approximate quadric distance); `None` = no limit.
    pub max_error: Option<f64>,
    /// Edges with a dihedral angle above this are kept sharp (degrees).
    pub feature_angle_deg: f64,
    /// Keep boundary edges in place.
    pub preserve_boundary: bool,
}

impl Default for DecimateOptions {
    fn default() -> Self {
        Self { target_triangles: 0, max_error: None, feature_angle_deg: 40.0, preserve_boundary: true }
    }
}

/// Quadric edge-collapse decimation; returns the number of triangles removed.
pub fn decimate(mesh: &mut Mesh, opts: &DecimateOptions) -> usize {
    decimate_guarded(mesh, opts, None)
}

/// Measured guard of a collapse: the reduction of [`crate::reduce`] checks every collapse against
/// the original surface instead of trusting the quadric alone (which overestimates the error of noisy
/// data and ignores the sagitta inside a triangle).
pub(crate) struct Guard<'a> {
    /// The original surface.
    pub bvh: &'a Bvh,
    /// Per-vertex error budget (mm), indexed by the vertices of the mesh before the call; a collapse
    /// is allowed only when the measured deviation is within the smaller budget of its two vertices,
    /// and the merged vertex keeps that smaller budget. Its content is undefined after the call.
    pub budget: &'a mut Vec<f64>,
}

/// With a guard, meshes with at least this many triangles are reduced in parts first.
const PARTS_FROM_TRIANGLES: usize = 100_000;

/// [`decimate`] with an optional measured [`Guard`]; without one it is [`decimate`].
pub(crate) fn decimate_guarded(mesh: &mut Mesh, opts: &DecimateOptions, mut guard: Option<Guard<'_>>) -> usize {
    if guard.as_ref().is_some_and(|g| g.budget.len() != mesh.positions.len()) {
        return 0;
    }
    if mesh.validate().is_err() || mesh.triangles.len() <= opts.target_triangles.max(4) {
        return 0;
    }
    let before = mesh.triangles.len();
    let mut tris = std::mem::take(&mut mesh.triangles);
    let mut pos = std::mem::take(&mut mesh.positions);
    let (mut q, mut locked) = setup(&tris, &pos, opts);
    let max_cost = opts.max_error.filter(|e| e.is_finite() && *e > 0.0).map(|e| e * e);
    if let Some(g) = guard.as_mut()
        && tris.len() >= PARTS_FROM_TRIANGLES
    {
        let whole = crate::partition::Whole { tris, pos, q, locked };
        let done = crate::partition::reduce_in_parts(whole, g, opts, max_cost);
        (tris, pos, q, locked) = (done.tris, done.pos, done.q, done.locked);
    }
    let mut c = Collapser::new(tris, pos, q, locked, guard, true);
    c.run(opts.target_triangles, max_cost);
    let (tris, pos, _) = c.finish();
    mesh.positions = pos;
    mesh.triangles = tris;
    compact(mesh);
    before - mesh.triangles.len()
}

/// The vertex quadrics (face planes, plus constraint planes along boundary and feature edges) and
/// the vertices that must not move (on an edge of three or more faces).
fn setup(tris: &[[u32; 3]], pos: &[Point3], opts: &DecimateOptions) -> (Vec<Quadric>, Vec<bool>) {
    let nv = pos.len();
    let mut q = vec![Quadric::zeros(); nv];
    let mut fnormal: Vec<Vec3> = Vec::with_capacity(tris.len());
    for t in tris {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        let n = crate::normalize(tri::cross(&a, &b, &c)).unwrap_or_else(Vec3::zeros);
        fnormal.push(n);
        let k = plane_quadric(&n, &a, 1.0);
        for &v in t {
            q[v as usize] += k;
        }
    }
    let cos_feat = opts.feature_angle_deg.clamp(1.0, 179.0).to_radians().cos();
    let mut edge_faces: HashMap<(u32, u32), Vec<u32>> = HashMap::with_capacity(tris.len() * 3 / 2);
    for (i, t) in tris.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            edge_faces.entry((a.min(b), a.max(b))).or_default().push(i as u32);
        }
    }
    let mut locked = vec![false; nv];
    let mut edge_list: Vec<(&(u32, u32), &Vec<u32>)> = edge_faces.iter().collect();
    edge_list.sort_unstable_by_key(|(k, _)| **k);
    for (&(a, b), fs) in edge_list {
        let (pa, pb) = (pos[a as usize], pos[b as usize]);
        let e = pb - pa;
        let len2 = e.norm_squared();
        let boundary = fs.len() == 1;
        let feature = fs.len() == 2 && fnormal[fs[0] as usize].dot(&fnormal[fs[1] as usize]) < cos_feat;
        if fs.len() > 2 {
            locked[a as usize] = true;
            locked[b as usize] = true;
            continue;
        }
        if (boundary && opts.preserve_boundary) || feature {
            for &f in fs {
                if let Some(side) = crate::normalize(e.cross(&fnormal[f as usize])) {
                    let k = plane_quadric(&side, &pa, 1000.0 * len2.max(1e-12));
                    q[a as usize] += k;
                    q[b as usize] += k;
                }
            }
        }
    }
    (q, locked)
}
