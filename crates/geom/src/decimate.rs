//! Decimation: quadric error metric edge collapse (Garland & Heckbert 1997) with boundary and
//! feature-edge constraint planes, link-condition and normal-flip checks, optionally guarded by
//! a measured deviation from the original surface.
use crate::edit::compact;
use crate::{Bvh, Mesh, Point3, Vec3, tri};
use nalgebra::{Matrix4, Vector4};
use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet};

/// A symmetric 4×4 quadric.
type Quadric = Matrix4<f64>;

fn plane_quadric(n: &Vec3, p: &Point3, w: f64) -> Quadric {
    let v = Vector4::new(n.x, n.y, n.z, -n.dot(&p.coords));
    v * v.transpose() * w
}

#[derive(Clone, Copy, PartialEq)]
struct Cand {
    cost: f64,
    a: u32,
    b: u32,
    stamp: (u32, u32),
    target: [f64; 3],
}
impl Eq for Cand {}
impl Ord for Cand {
    fn cmp(&self, o: &Self) -> Ordering {
        o.cost.total_cmp(&self.cost).then(o.a.cmp(&self.a)).then(o.b.cmp(&self.b))
    }
}
impl PartialOrd for Cand {
    fn partial_cmp(&self, o: &Self) -> Option<Ordering> {
        Some(self.cmp(o))
    }
}

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
    /// and the merged vertex keeps that smaller budget.
    pub budget: &'a mut Vec<f64>,
}

/// Collapses whose quadric error is below this fraction of the budget are not measured.
const GUARD_SKIP: f64 = 0.25;
/// With a guard, a collapse may not grow a vertex above this many neighbours (or above the valence
/// it already has). Flat regions collapse at zero cost, and without the cap they pile up in a few
/// hubs whose rings make every further collapse linear in the ring (minutes on a 100k-triangle block).
const GUARD_VALENCE: usize = 32;

/// [`decimate`] with an optional measured [`Guard`]; without one it is [`decimate`].
pub(crate) fn decimate_guarded(mesh: &mut Mesh, opts: &DecimateOptions, mut guard: Option<Guard<'_>>) -> usize {
    if guard.as_ref().is_some_and(|g| g.budget.len() != mesh.positions.len()) {
        return 0;
    }
    if mesh.validate().is_err() || mesh.triangles.len() <= opts.target_triangles.max(4) {
        return 0;
    }
    let nv = mesh.positions.len();
    let mut tris = std::mem::take(&mut mesh.triangles);
    let before = tris.len();
    let mut pos = std::mem::take(&mut mesh.positions);
    let mut alive_t = vec![true; tris.len()];
    let mut vfaces: Vec<Vec<u32>> = vec![Vec::new(); nv];
    for (i, t) in tris.iter().enumerate() {
        for &v in t {
            vfaces[v as usize].push(i as u32);
        }
    }
    // quadrics
    let mut q = vec![Quadric::zeros(); nv];
    let mut fnormal: Vec<Vec3> = Vec::with_capacity(tris.len());
    for t in &tris {
        let (a, b, c) = (pos[t[0] as usize], pos[t[1] as usize], pos[t[2] as usize]);
        let cr = tri::cross(&a, &b, &c);
        let n = crate::normalize(cr).unwrap_or_else(Vec3::zeros);
        fnormal.push(n);
        let k = plane_quadric(&n, &a, 1.0);
        for &v in t {
            q[v as usize] += k;
        }
    }
    // boundary and feature edges get constraint planes
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
                let fnn = fnormal[f as usize];
                if let Some(side) = crate::normalize(e.cross(&fnn)) {
                    let k = plane_quadric(&side, &pa, 1000.0 * len2.max(1e-12));
                    q[a as usize] += k;
                    q[b as usize] += k;
                }
            }
        }
    }
    drop(edge_faces);
    let mut stamp = vec![0u32; nv];
    let mut alive_v = vec![true; nv];
    let cost_of = |q: &Quadric, p: &Point3| {
        let v = Vector4::new(p.x, p.y, p.z, 1.0);
        (v.transpose() * q * v)[(0, 0)].max(0.0)
    };
    let best_target = |q: &Quadric, pa: &Point3, pb: &Point3| -> (Point3, f64) {
        let a = q.fixed_view::<3, 3>(0, 0).into_owned();
        let bvec = -q.fixed_view::<3, 1>(0, 3).into_owned();
        let mut cands = vec![*pa, *pb, Point3::from((pa.coords + pb.coords) / 2.0)];
        if a.determinant().abs() > 1e-10
            && let Some(x) = a.lu().solve(&bvec)
        {
            let p = Point3::from(x);
            // only when close to the edge (no runaway solutions in flat regions)
            let l = (pb - pa).norm();
            if crate::finite(&p) && (p - Point3::from((pa.coords + pb.coords) / 2.0)).norm() <= 2.0 * l {
                cands.push(p);
            }
        }
        cands.into_iter().map(|p| (p, cost_of(q, &p))).min_by(|x, y| x.1.total_cmp(&y.1)).unwrap_or((*pa, 0.0))
    };
    let mut heap: BinaryHeap<Cand> = BinaryHeap::with_capacity(tris.len() * 3 / 2);
    let push =
        |heap: &mut BinaryHeap<Cand>, a: u32, b: u32, pos: &[Point3], q: &[Quadric], stamp: &[u32], locked: &[bool]| {
            if locked[a as usize] || locked[b as usize] {
                return;
            }
            let qq = q[a as usize] + q[b as usize];
            let (p, c) = best_target(&qq, &pos[a as usize], &pos[b as usize]);
            heap.push(Cand { cost: c, a, b, stamp: (stamp[a as usize], stamp[b as usize]), target: [p.x, p.y, p.z] });
        };
    let mut seen_pair: HashSet<(u32, u32)> = HashSet::with_capacity(tris.len() * 3 / 2);
    for t in &tris {
        for k in 0..3 {
            let (a, b) = (t[k], t[(k + 1) % 3]);
            if seen_pair.insert((a.min(b), a.max(b))) {
                push(&mut heap, a.min(b), a.max(b), &pos, &q, &stamp, &locked);
            }
        }
    }
    let mut live = before;
    let max_cost = opts.max_error.filter(|e| e.is_finite() && *e > 0.0).map(|e| e * e);
    let target = opts.target_triangles;
    while live > target {
        let Some(c) = heap.pop() else { break };
        let (a, b) = (c.a, c.b);
        if !alive_v[a as usize] || !alive_v[b as usize] {
            continue;
        }
        if c.stamp != (stamp[a as usize], stamp[b as usize]) {
            continue;
        }
        if max_cost.is_some_and(|m| c.cost > m) {
            break;
        }
        // faces of a, b; shared faces are the ones removed
        let fa: Vec<u32> = vfaces[a as usize].iter().copied().filter(|&f| alive_t[f as usize]).collect();
        let fb: Vec<u32> = vfaces[b as usize].iter().copied().filter(|&f| alive_t[f as usize]).collect();
        let shared: Vec<u32> = fa.iter().copied().filter(|f| fb.contains(f)).collect();
        if shared.is_empty() || shared.len() > 2 {
            continue;
        }
        // link condition: common neighbours are exactly the opposite corners of shared faces
        let nbr = |fs: &[u32], v: u32| -> HashSet<u32> {
            fs.iter().flat_map(|&f| tris[f as usize]).filter(|&x| x != v).collect()
        };
        let (na, nb) = (nbr(&fa, a), nbr(&fb, b));
        let common = na.intersection(&nb).count();
        if common != shared.len() {
            continue;
        }
        if guard.is_some() && na.len() + nb.len() - common - 2 > GUARD_VALENCE.max(na.len()).max(nb.len()) {
            continue;
        }
        let p = Point3::new(c.target[0], c.target[1], c.target[2]);
        // no flipped or degenerate faces around the new vertex
        let mut ok = true;
        let mut fan: Vec<[Point3; 3]> = Vec::new();
        for &f in fa.iter().chain(fb.iter()) {
            if shared.contains(&f) {
                continue;
            }
            let t = tris[f as usize];
            let old = tri::cross(&pos[t[0] as usize], &pos[t[1] as usize], &pos[t[2] as usize]);
            let corners: Vec<Point3> = t.iter().map(|&v| if v == a || v == b { p } else { pos[v as usize] }).collect();
            let new = tri::cross(&corners[0], &corners[1], &corners[2]);
            fan.push([corners[0], corners[1], corners[2]]);
            if new.dot(&old) <= 0.0 || new.norm() < 1e-3 * old.norm() {
                ok = false;
                break;
            }
            if let (Some(n0), Some(n1)) = (crate::normalize(old), crate::normalize(new))
                && n0.dot(&n1) < 0.2
            {
                ok = false;
                break;
            }
        }
        if !ok {
            continue;
        }
        if let Some(g) = guard.as_ref() {
            let limit = g.budget[a as usize].min(g.budget[b as usize]);
            if c.cost > (GUARD_SKIP * limit).powi(2)
                && !collapse_within(g.bvh, limit, &p, &[pos[a as usize], pos[b as usize]], &fan)
            {
                continue;
            }
        }
        // collapse b into a
        for &f in &shared {
            alive_t[f as usize] = false;
            live -= 1;
        }
        for &f in &fb {
            if !alive_t[f as usize] {
                continue;
            }
            for v in tris[f as usize].iter_mut() {
                if *v == b {
                    *v = a;
                }
            }
            vfaces[a as usize].push(f);
        }
        vfaces[b as usize].clear();
        vfaces[a as usize].retain(|&f| alive_t[f as usize]);
        alive_v[b as usize] = false;
        if let Some(g) = guard.as_mut() {
            g.budget[a as usize] = g.budget[a as usize].min(g.budget[b as usize]);
        }
        pos[a as usize] = p;
        let qb = q[b as usize];
        q[a as usize] += qb;
        stamp[a as usize] = stamp[a as usize].wrapping_add(1);
        let mut nbs: Vec<u32> = vfaces[a as usize].iter().flat_map(|&f| tris[f as usize]).filter(|&x| x != a).collect();
        nbs.sort_unstable();
        nbs.dedup();
        for &o in &nbs {
            stamp[o as usize] = stamp[o as usize].wrapping_add(1);
        }
        // re-queue every edge around the neighbours (their stamps changed)
        seen_pair.clear();
        for &o in nbs.iter().chain(std::iter::once(&a)) {
            for &f in &vfaces[o as usize] {
                let t = tris[f as usize];
                for k in 0..3 {
                    let (x, y) = (t[k], t[(k + 1) % 3]);
                    let key = (x.min(y), x.max(y));
                    if seen_pair.insert(key) {
                        push(&mut heap, key.0, key.1, &pos, &q, &stamp, &locked);
                    }
                }
            }
        }
    }
    let mut k = alive_t.iter();
    tris.retain(|_| k.next().copied().unwrap_or(false));
    mesh.positions = pos;
    mesh.triangles = tris;
    compact(mesh);
    before - mesh.triangles.len()
}

/// Is the surface after a collapse within `limit` of the original? Measured: the new vertex and the
/// centroids of the new triangles (`fan`) against the original surface, and the two vertices
/// that vanish (`old`) against the new triangles.
fn collapse_within(original: &Bvh, limit: f64, p: &Point3, old: &[Point3], fan: &[[Point3; 3]]) -> bool {
    if original.closest(p, limit).is_none() {
        return false;
    }
    for t in fan {
        let centroid = Point3::from((t[0].coords + t[1].coords + t[2].coords) / 3.0);
        if original.closest(&centroid, limit).is_none() {
            return false;
        }
    }
    old.iter().all(|q| fan.iter().any(|t| (tri::closest_point(q, &t[0], &t[1], &t[2]) - q).norm() <= limit))
}
