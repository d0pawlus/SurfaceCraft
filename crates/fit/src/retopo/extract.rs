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
    let nv = l.v.len();
    let mut uf = Uf::new(nv);
    let mut steps: Vec<(u32, u32)> = Vec::new();
    for i in 0..nv {
        for &j in &l.nb[i] {
            if (j as usize) <= i {
                continue;
            }
            let ju = j as usize;
            let (a, b) = compat4(&o[i], &l.n[i], &o[ju], &l.n[ju]);
            let (li, lj, di, dj) = lattice_pair(&l.v[i], &l.n[i], &p[i], &a, &l.v[ju], &l.n[ju], &p[ju], &b, rho);
            if (li - lj).norm() > 0.35 * rho {
                continue;
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
            match k.0.abs() + k.1.abs() {
                0 => {
                    uf.union(i as u32, j);
                }
                1 => {
                    steps.push((i as u32, j));
                }
                _ => {}
            }
        }
    }
    // clusters
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
    // layout vertices of the same lattice point reached through separate clusters merge
    let ptree = KdTree::build(&cpos.iter().map(|v| Point3::from(*v)).collect::<Vec<_>>());
    let mut cuf = Uf::new(nc);
    for c in 0..nc {
        for j in ptree.within(&Point3::from(cpos[c]), 0.3 * rho) {
            if j != c && cn[c].dot(&cn[j]) > 0.0 {
                cuf.union(c as u32, j as u32);
            }
        }
    }
    let mut edges: std::collections::BTreeSet<(u32, u32)> = std::collections::BTreeSet::new();
    for (i, j) in steps {
        let (a, b) = (cuf.find(cluster[i as usize]), cuf.find(cluster[j as usize]));
        if a != b {
            edges.insert((a.min(b), a.max(b)));
        }
    }
    // merged cluster positions and normals
    let mut mpos = vec![Vec3::zeros(); nc];
    let mut mn = vec![Vec3::zeros(); nc];
    let mut mw = vec![0.0f64; nc];
    for c in 0..nc {
        let r = cuf.find(c as u32) as usize;
        let w = f64::from(members[c]);
        mpos[r] += cpos[c] * w;
        mn[r] += cn[c];
        mw[r] += w;
    }
    // adjacency; dangling vertices are pruned
    let mut adj: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in &edges {
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
    if adj.len() < 4 {
        return None;
    }
    // compact ids, positions on the working surface
    let ids: Vec<u32> = adj.keys().copied().collect();
    let index: HashMap<u32, u32> = ids.iter().enumerate().map(|(i, &c)| (c, i as u32)).collect();
    let mut vertices = Vec::with_capacity(ids.len());
    let mut normals = Vec::with_capacity(ids.len());
    for &c in &ids {
        let c = c as usize;
        let pos = Point3::from(mpos[c] / mw[c].max(1.0));
        let nrm = normalize(mn[c]).unwrap_or_else(Vec3::z);
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
    // rotation system, counter-clockwise about the vertex normal
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
    let mut seen: Vec<Vec<bool>> = nbs.iter().map(|l| vec![false; l.len()]).collect();
    let mut faces: Vec<Vec<u32>> = Vec::new();
    for u0 in 0..ids.len() {
        for k0 in 0..nbs[u0].len() {
            if seen[u0][k0] {
                continue;
            }
            let (mut u, mut k) = (u0, k0);
            let mut walk: Vec<u32> = Vec::new();
            let mut ok = true;
            loop {
                seen[u][k] = true;
                walk.push(u as u32);
                let v = nbs[u][k] as usize;
                let Some(back) = nbs[v].iter().position(|&x| x as usize == u) else {
                    ok = false;
                    break;
                };
                let deg = nbs[v].len();
                let nk = (back + deg - 1) % deg;
                u = v;
                k = nk;
                if (u, k) == (u0, k0) {
                    break;
                }
                if seen[u][k] || walk.len() > 4 * MAX_POLY {
                    ok = false;
                    break;
                }
            }
            if !ok || walk.len() < 3 || walk.len() > MAX_POLY {
                continue;
            }
            let p0 = vertices[walk[0] as usize];
            let nsum: Vec3 = walk.iter().map(|&w| normals[w as usize]).sum();
            let mut area = 0.0;
            for w in 1..walk.len() - 1 {
                let (a, b) = (vertices[walk[w] as usize] - p0, vertices[walk[w + 1] as usize] - p0);
                area += a.cross(&b).dot(&nsum);
            }
            if area > 0.0 {
                faces.push(walk);
            }
        }
    }
    if faces.is_empty() {
        return None;
    }
    fill_small_holes(&mut faces);
    // drop vertices no face uses
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
    Some(PolyMesh { vertices: vs, normals: ns, faces })
}

/// Closes boundary loops of 3 to `MAX_POLY` edges with one polygon each.
pub(super) fn fill_small_holes(faces: &mut Vec<Vec<u32>>) {
    let mut directed: std::collections::BTreeSet<(u32, u32)> = std::collections::BTreeSet::new();
    for f in faces.iter() {
        for k in 0..f.len() {
            directed.insert((f[k], f[(k + 1) % f.len()]));
        }
    }
    // a hole's edges run against the unmatched directed edges
    let mut next: BTreeMap<u32, Vec<u32>> = BTreeMap::new();
    for &(a, b) in &directed {
        if !directed.contains(&(b, a)) {
            next.entry(b).or_default().push(a);
        }
    }
    let starts: Vec<u32> = next.keys().copied().collect();
    for s in starts {
        while let Some(first) = next.get(&s).and_then(|v| v.first().copied()) {
            // walk the loop from s
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
            // consume the edges of this walk so it is not repeated
            let mut consumed = lp.clone();
            consumed.push(s);
            for w in consumed.windows(2) {
                if let Some(v) = next.get_mut(&w[0])
                    && let Some(pos) = v.iter().position(|&x| x == w[1])
                {
                    v.remove(pos);
                }
            }
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
