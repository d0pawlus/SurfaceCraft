//! Extraction of a polygon mesh from the solved fields.
use super::MAX_POLY;
use super::PolyMesh;
use super::field::{Level, compat4, lattice_pair};
use std::collections::{BTreeMap, HashMap};
use surfacecraft_geom::{Bvh, KdTree, Point3, Vec3, any_perpendicular, normalize};

pub(super) struct Uf(Vec<u32>);

impl Uf {
    fn new(n: usize) -> Self {
        Self((0..n as u32).collect())
    }
    fn find(&mut self, mut x: u32) -> u32 {
        while self.0[x as usize] != x {
            let p = self.0[x as usize];
            self.0[x as usize] = self.0[p as usize];
            x = self.0[x as usize];
        }
        x
    }
    fn union(&mut self, a: u32, b: u32) {
        let (a, b) = (self.find(a), self.find(b));
        if a != b {
            self.0[a.max(b) as usize] = a.min(b);
        }
    }
}

/// Extracts a polygon mesh from the solved fields of the finest level.
pub(super) fn extract(l: &Level, o: &[Vec3], p: &[Vec3], rho: f64, bvh: &Bvh) -> Option<PolyMesh> {
    let (mut uf, steps) = lattice_unions(l, o, p, rho);
    let cl = Clusters::build(&mut uf, l, p);
    let mut cuf = cl.merge_close(rho);
    let edges = merged_edges(steps, &mut cuf, &cl.id);
    let merged = Merged::build(&cl, &mut cuf);
    let adj = pruned_adjacency(&edges);
    if adj.len() < 4 {
        return None;
    }
    // compact ids, positions on the working surface
    let ids: Vec<u32> = adj.keys().copied().collect();
    let index: HashMap<u32, u32> = ids.iter().enumerate().map(|(i, &c)| (c, i as u32)).collect();
    let (vertices, normals) = surface_vertices(&ids, &merged, bvh);
    let nbs = rotation_system(&ids, &adj, &index, &vertices, &normals);
    let mut faces = trace_faces(&nbs, &vertices, &normals);
    if faces.is_empty() {
        return None;
    }
    fill_small_holes(&mut faces);
    Some(compact_mesh(vertices, normals, faces))
}

/// Mesh vertices that share a lattice point (united in the returned union-find) and the edges
/// whose lattice points are one step apart.
fn lattice_unions(l: &Level, o: &[Vec3], p: &[Vec3], rho: f64) -> (Uf, Vec<(u32, u32)>) {
    let nv = l.v.len();
    let mut uf = Uf::new(nv);
    let mut steps: Vec<(u32, u32)> = Vec::new();
    for i in 0..nv {
        for &j in &l.nb[i] {
            if (j as usize) <= i {
                continue;
            }
            match lattice_offset(l, o, p, rho, (i, j as usize)) {
                Some(0) => {
                    uf.union(i as u32, j);
                }
                Some(1) => {
                    steps.push((i as u32, j));
                }
                _ => {}
            }
        }
    }
    (uf, steps)
}

/// The lattice distance (in steps) of the two ends of an edge, or `None` when their lattices do
/// not meet at the edge.
fn lattice_offset(l: &Level, o: &[Vec3], p: &[Vec3], rho: f64, (i, ju): (usize, usize)) -> Option<i32> {
    let (a, b) = compat4(&o[i], &l.n[i], &o[ju], &l.n[ju]);
    let (li, lj, di, dj) = lattice_pair(&l.v[i], &l.n[i], &p[i], &a, &l.v[ju], &l.n[ju], &p[ju], &b, rho);
    if (li - lj).norm() > 0.35 * rho {
        return None;
    }
    // vertex j's axes expressed in vertex i's: b is ±a or ±(n × a)
    let ti = l.n[i].cross(&a);
    let (ca, ct) = (b.dot(&a), b.dot(&ti));
    let mapped = if ca.abs() >= ct.abs() {
        let s = if ca >= 0.0 { 1 } else { -1 };
        (s * dj.0, s * dj.1)
    } else {
        let s = if ct >= 0.0 { 1 } else { -1 };
        (-s * dj.1, s * dj.0)
    };
    let k = (di.0 - mapped.0, di.1 - mapped.1);
    Some(k.0.abs() + k.1.abs())
}

/// The clusters of mesh vertices that share a lattice point, with their mean position, normal sum
/// and vertex count.
struct Clusters {
    /// Cluster id of every mesh vertex.
    id: Vec<u32>,
    /// Mean position of every cluster.
    pos: Vec<Vec3>,
    /// Sum of the vertex normals of every cluster.
    n: Vec<Vec3>,
    /// Number of mesh vertices of every cluster.
    members: Vec<u32>,
}

impl Clusters {
    /// Numbers the roots of `uf` in order of first appearance and accumulates every cluster.
    fn build(uf: &mut Uf, l: &Level, p: &[Vec3]) -> Self {
        let nv = l.v.len();
        let mut id_of_root: HashMap<u32, u32> = HashMap::new();
        let mut cluster = vec![0u32; nv];
        let mut count = 0u32;
        for i in 0..nv {
            let r = uf.find(i as u32);
            let id = *id_of_root.entry(r).or_insert_with(|| {
                count += 1;
                count - 1
            });
            cluster[i] = id;
        }
        let nc = count as usize;
        let mut cpos = vec![Vec3::zeros(); nc];
        let mut cn = vec![Vec3::zeros(); nc];
        let mut members = vec![0u32; nc];
        for i in 0..nv {
            let c = cluster[i] as usize;
            cpos[c] += p[i];
            cn[c] += l.n[i];
            members[c] += 1;
        }
        for c in 0..nc {
            cpos[c] /= f64::from(members[c].max(1));
        }
        Self { id: cluster, pos: cpos, n: cn, members }
    }

    /// Layout vertices of the same lattice point reached through separate clusters merge:
    /// clusters with a close position and an agreeing normal are united.
    fn merge_close(&self, rho: f64) -> Uf {
        let nc = self.pos.len();
        let ptree = KdTree::build(&self.pos.iter().map(|v| Point3::from(*v)).collect::<Vec<_>>());
        let mut cuf = Uf::new(nc);
        for c in 0..nc {
            for j in ptree.within(&Point3::from(self.pos[c]), 0.3 * rho) {
                if j != c && self.n[c].dot(&self.n[j]) > 0.0 {
                    cuf.union(c as u32, j as u32);
                }
            }
        }
        cuf
    }
}

/// The layout edges between merged clusters, as ordered pairs.
fn merged_edges(steps: Vec<(u32, u32)>, cuf: &mut Uf, cluster: &[u32]) -> std::collections::BTreeSet<(u32, u32)> {
    let mut edges: std::collections::BTreeSet<(u32, u32)> = std::collections::BTreeSet::new();
    for (i, j) in steps {
        let (a, b) = (cuf.find(cluster[i as usize]), cuf.find(cluster[j as usize]));
        if a != b {
            edges.insert((a.min(b), a.max(b)));
        }
    }
    edges
}

/// Position sums, normal sums and weights of the merged clusters (stored at their root).
struct Merged {
    /// Weighted position sum per root.
    pos: Vec<Vec3>,
    /// Normal sum per root.
    n: Vec<Vec3>,
    /// Weight (vertex count) per root.
    w: Vec<f64>,
}

impl Merged {
    /// Accumulates every cluster at the root of its merged cluster.
    fn build(cl: &Clusters, cuf: &mut Uf) -> Self {
        let nc = cl.pos.len();
        let mut mpos = vec![Vec3::zeros(); nc];
        let mut mn = vec![Vec3::zeros(); nc];
        let mut mw = vec![0.0f64; nc];
        for c in 0..nc {
            let r = cuf.find(c as u32) as usize;
            let w = f64::from(cl.members[c]);
            mpos[r] += cl.pos[c] * w;
            mn[r] += cl.n[c];
            mw[r] += w;
        }
        Self { pos: mpos, n: mn, w: mw }
    }
}

/// The adjacency lists of the layout graph; dangling vertices are pruned.
fn pruned_adjacency(edges: &std::collections::BTreeSet<(u32, u32)>) -> BTreeMap<u32, Vec<u32>> {
    let mut adj: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in edges {
        adj.entry(a).or_default().push(b);
        adj.entry(b).or_default().push(a);
    }
    loop {
        let weak: Vec<u32> = adj.iter().filter(|(_, v)| v.len() < 2).map(|(k, _)| *k).collect();
        if weak.is_empty() {
            break;
        }
        for k in weak {
            if let Some(nbs) = adj.remove(&k) {
                for b in nbs {
                    if let Some(l) = adj.get_mut(&b) {
                        l.retain(|&x| x != k);
                    }
                }
            }
        }
    }
    adj
}

/// Positions (moved onto the working surface) and normals of the layout vertices `ids`.
fn surface_vertices(ids: &[u32], m: &Merged, bvh: &Bvh) -> (Vec<Point3>, Vec<Vec3>) {
    let mut vertices = Vec::with_capacity(ids.len());
    let mut normals = Vec::with_capacity(ids.len());
    for &c in ids {
        let c = c as usize;
        let pos = Point3::from(m.pos[c] / m.w[c].max(1.0));
        let nrm = normalize(m.n[c]).unwrap_or_else(Vec3::z);
        match bvh.closest(&pos, f64::INFINITY) {
            Some(h) => {
                normals.push(nrm);
                vertices.push(h.point);
            }
            None => {
                normals.push(nrm);
                vertices.push(pos);
            }
        }
    }
    (vertices, normals)
}

/// The rotation system: the neighbours of every layout vertex, counter-clockwise about its normal.
fn rotation_system(
    ids: &[u32],
    adj: &BTreeMap<u32, Vec<u32>>,
    index: &HashMap<u32, u32>,
    vertices: &[Point3],
    normals: &[Vec3],
) -> Vec<Vec<u32>> {
    let mut nbs: Vec<Vec<u32>> = Vec::with_capacity(ids.len());
    for (k, &c) in ids.iter().enumerate() {
        let n = normals[k];
        let e1 = any_perpendicular(&n);
        let e2 = n.cross(&e1);
        let mut list: Vec<(f64, u32)> = adj[&c]
            .iter()
            .filter_map(|b| index.get(b).copied())
            .map(|b| {
                let d = vertices[b as usize] - vertices[k];
                ((d.dot(&e2)).atan2(d.dot(&e1)), b)
            })
            .collect();
        list.sort_by(|a, b| a.0.total_cmp(&b.0));
        nbs.push(list.into_iter().map(|x| x.1).collect());
    }
    nbs
}

/// The counter-clockwise faces (3 to `MAX_POLY` vertices) of the rotation system `nbs`.
fn trace_faces(nbs: &[Vec<u32>], vertices: &[Point3], normals: &[Vec3]) -> Vec<Vec<u32>> {
    let mut seen: Vec<Vec<bool>> = nbs.iter().map(|l| vec![false; l.len()]).collect();
    let mut faces: Vec<Vec<u32>> = Vec::new();
    for u0 in 0..nbs.len() {
        for k0 in 0..nbs[u0].len() {
            if seen[u0][k0] {
                continue;
            }
            let Some(walk) = walk_face(nbs, &mut seen, (u0, k0)) else {
                continue;
            };
            if !(3..=MAX_POLY).contains(&walk.len()) {
                continue;
            }
            if face_is_counter_clockwise(&walk, vertices, normals) {
                faces.push(walk);
            }
        }
    }
    faces
}

/// Walks the face that starts with the directed edge (vertex, neighbour slot) `start`; `None`
/// when the walk does not close up cleanly.
fn walk_face(nbs: &[Vec<u32>], seen: &mut [Vec<bool>], start: (usize, usize)) -> Option<Vec<u32>> {
    let (u0, k0) = start;
    let (mut u, mut k) = (u0, k0);
    let mut walk: Vec<u32> = Vec::new();
    loop {
        seen[u][k] = true;
        walk.push(u as u32);
        let v = nbs[u][k] as usize;
        let back = nbs[v].iter().position(|&x| x as usize == u)?;
        let deg = nbs[v].len();
        let nk = (back + deg - 1) % deg;
        u = v;
        k = nk;
        if (u, k) == (u0, k0) {
            return Some(walk);
        }
        if seen[u][k] || walk.len() > 4 * MAX_POLY {
            return None;
        }
    }
}

/// Does the polygon `walk` have a positive area about the sum of its vertex normals?
fn face_is_counter_clockwise(walk: &[u32], vertices: &[Point3], normals: &[Vec3]) -> bool {
    let p0 = vertices[walk[0] as usize];
    let nsum: Vec3 = walk.iter().map(|&w| normals[w as usize]).sum();
    let mut area = 0.0;
    for w in 1..walk.len() - 1 {
        let (a, b) = (vertices[walk[w] as usize] - p0, vertices[walk[w + 1] as usize] - p0);
        area += a.cross(&b).dot(&nsum);
    }
    area > 0.0
}

/// The mesh of `faces` without the vertices no face uses (faces renumbered).
fn compact_mesh(vertices: Vec<Point3>, normals: Vec<Vec3>, mut faces: Vec<Vec<u32>>) -> PolyMesh {
    let mut used = vec![false; vertices.len()];
    for f in &faces {
        for &w in f {
            used[w as usize] = true;
        }
    }
    let mut remap = vec![u32::MAX; vertices.len()];
    let (mut vs, mut ns) = (Vec::new(), Vec::new());
    for i in 0..vertices.len() {
        if used[i] {
            remap[i] = vs.len() as u32;
            vs.push(vertices[i]);
            ns.push(normals[i]);
        }
    }
    for f in &mut faces {
        for w in f.iter_mut() {
            *w = remap[*w as usize];
        }
    }
    PolyMesh { vertices: vs, normals: ns, faces }
}

/// Closes boundary loops of 3 to `MAX_POLY` edges with one polygon each.
pub(super) fn fill_small_holes(faces: &mut Vec<Vec<u32>>) {
    let mut next = unmatched_edges(faces);
    let starts: Vec<u32> = next.keys().copied().collect();
    for s in starts {
        while let Some(first) = next.get(&s).and_then(|v| v.first().copied()) {
            let (lp, closed) = walk_hole(&next, s, first);
            consume_walk(&mut next, &lp, s);
            if closed && lp.len() >= 3 {
                faces.push(lp);
            } else if !closed {
                // an open chain: drop its first edge to make progress
                if let Some(v) = next.get_mut(&s)
                    && !v.is_empty()
                {
                    v.remove(0);
                }
            }
        }
    }
}

/// The directed edges of `faces` without an opposite edge, reversed and grouped by start vertex
/// (a hole's edges run against them).
fn unmatched_edges(faces: &[Vec<u32>]) -> BTreeMap<u32, Vec<u32>> {
    let mut directed: std::collections::BTreeSet<(u32, u32)> = std::collections::BTreeSet::new();
    for f in faces.iter() {
        for k in 0..f.len() {
            directed.insert((f[k], f[(k + 1) % f.len()]));
        }
    }
    let mut next: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            next.entry(b).or_default().push(a);
        }
    }
    next
}

/// Walks the loop of `next` from `s` through `first`; the flag says whether it came back to `s`
/// within `MAX_POLY` vertices.
fn walk_hole(next: &BTreeMap<u32, Vec<u32>>, s: u32, first: u32) -> (Vec<u32>, bool) {
    let mut lp = vec![s];
    let mut cur = first;
    let mut closed = false;
    while lp.len() <= MAX_POLY {
        if cur == s {
            closed = true;
            break;
        }
        lp.push(cur);
        match next.get(&cur).and_then(|v| v.first().copied()) {
            Some(n) => cur = n,
            None => break,
        }
    }
    (lp, closed)
}

/// Removes the edges of the walk `lp` (and its closing edge back to `s`) from `next` so it is not
/// repeated.
fn consume_walk(next: &mut BTreeMap<u32, Vec<u32>>, lp: &[u32], s: u32) {
    let mut consumed = lp.to_vec();
    consumed.push(s);
    for w in consumed.windows(2) {
        if let Some(v) = next.get_mut(&w[0])
            && let Some(pos) = v.iter().position(|&x| x == w[1])
        {
            v.remove(pos);
        }
    }
}
