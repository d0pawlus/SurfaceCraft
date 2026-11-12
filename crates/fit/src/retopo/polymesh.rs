//! Methods of [`PolyMesh`] and [`QuadMesh`]: relaxation, splitting and topology queries.
use super::{PolyMesh, QuadMesh};
use std::collections::{BTreeMap, HashMap};
use surfacecraft_geom::{Point3, Vec3, normalize};

impl PolyMesh {
    /// The faces as quads when every one is a quad.
    #[must_use]
    pub fn as_quads(&self) -> Option<QuadMesh> {
        let faces: Option<Vec<[u32; 4]>> = self.faces.iter().map(|f| <[u32; 4]>::try_from(f.as_slice()).ok()).collect();
        Some(QuadMesh { vertices: self.vertices.clone(), normals: self.normals.clone(), faces: faces? })
    }

    /// Tangential relaxation: every interior vertex moves halfway to the average of its edge
    /// neighbours (in its tangent plane) and is projected back onto the scan by `project`.
    /// Evens out quad sizes and untangles small folds of the extracted layout.
    pub fn relax(&mut self, iterations: usize, project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>) {
        let nv = self.vertices.len();
        if self.normals.len() != nv {
            return; // a vertex without a normal cannot be relaxed
        }
        let mut nb: Vec<std::collections::BTreeSet<u32>> = vec![Default::default(); nv];
        for f in &self.faces {
            for k in 0..f.len() {
                let (a, b) = (f[k], f[(k + 1) % f.len()]);
                if let Some(s) = nb.get_mut(a as usize) {
                    s.insert(b);
                }
                if let Some(s) = nb.get_mut(b as usize) {
                    s.insert(a);
                }
            }
        }
        let mut fixed = vec![false; nv];
        let mut uses: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for f in &self.faces {
            for k in 0..f.len() {
                let (a, b) = (f[k], f[(k + 1) % f.len()]);
                *uses.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        for ((a, b), c) in uses {
            if c == 1 {
                for v in [a, b] {
                    if let Some(x) = fixed.get_mut(v as usize) {
                        *x = true;
                    }
                }
            }
        }
        for _ in 0..iterations {
            let mut moved: Vec<(Point3, Vec3)> = Vec::with_capacity(nv);
            for i in 0..nv {
                let (p, n) = (self.vertices[i], self.normals[i]);
                if fixed[i] || nb[i].len() < 3 {
                    moved.push((p, n));
                    continue;
                }
                let avg: Vec3 =
                    nb[i].iter().filter_map(|&j| self.vertices.get(j as usize)).map(|q| q.coords).sum::<Vec3>()
                        / nb[i].len() as f64;
                let d = avg - p.coords;
                let target = Point3::from(p.coords + (d - n * d.dot(&n)) * 0.5);
                moved.push(project(&target, &n).unwrap_or((p, n)));
            }
            for (i, (p, n)) in moved.into_iter().enumerate() {
                self.vertices[i] = p;
                self.normals[i] = n;
            }
        }
    }

    /// One Catmull–Clark-style topological split of every face into quads (edge midpoints and a
    /// face centre); `project` moves each new point onto the scan and returns its normal.
    #[must_use]
    pub fn split_to_quads(&self, project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>) -> QuadMesh {
        let mut vertices = self.vertices.clone();
        let mut normals = self.normals.clone();
        let mut mid: HashMap<(u32, u32), u32> = HashMap::new();
        let mut faces: Vec<[u32; 4]> = Vec::with_capacity(self.faces.len() * 4);
        for f in &self.faces {
            let n = f.len();
            if n < 3 {
                continue;
            }
            let pts: Vec<(Point3, Vec3)> = f
                .iter()
                .map(|&i| {
                    (
                        vertices.get(i as usize).copied().unwrap_or_else(Point3::origin),
                        normals.get(i as usize).copied().unwrap_or_else(Vec3::z),
                    )
                })
                .collect();
            let centre: Vec3 = pts.iter().map(|p| p.0.coords).sum::<Vec3>() / n as f64;
            let nsum: Vec3 = pts.iter().map(|p| p.1).sum();
            let nrm = normalize(nsum).unwrap_or_else(Vec3::z);
            let (cp, cn) = project(&Point3::from(centre), &nrm).unwrap_or((Point3::from(centre), nrm));
            let c = vertices.len() as u32;
            vertices.push(cp);
            normals.push(cn);
            let mut edge_ids = Vec::with_capacity(n);
            for k in 0..n {
                let (a, b) = (f[k], f[(k + 1) % n]);
                let key = (a.min(b), a.max(b));
                let id = match mid.get(&key) {
                    Some(&id) => id,
                    None => {
                        let (pa, na) = (pts[k].0, pts[k].1);
                        let (pb, nb) = (pts[(k + 1) % n].0, pts[(k + 1) % n].1);
                        let mp = Point3::from((pa.coords + pb.coords) * 0.5);
                        let mn = normalize(na + nb).unwrap_or(na);
                        let (p, nn) = project(&mp, &mn).unwrap_or((mp, mn));
                        let id = vertices.len() as u32;
                        vertices.push(p);
                        normals.push(nn);
                        mid.insert(key, id);
                        id
                    }
                };
                edge_ids.push(id);
            }
            for k in 0..n {
                faces.push([f[k], edge_ids[k], c, edge_ids[(k + n - 1) % n]]);
            }
        }
        QuadMesh { vertices, normals, faces }
    }
}

impl QuadMesh {
    /// The same mesh as polygons.
    #[must_use]
    pub fn to_poly(&self) -> PolyMesh {
        PolyMesh {
            vertices: self.vertices.clone(),
            normals: self.normals.clone(),
            faces: self.faces.iter().map(|f| f.to_vec()).collect(),
        }
    }

    /// Splits every quad into four (see [`PolyMesh::split_to_quads`]).
    #[must_use]
    pub fn split(&self, project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>) -> QuadMesh {
        self.to_poly().split_to_quads(project)
    }

    /// Vertex valences.
    #[must_use]
    pub fn valences(&self) -> Vec<usize> {
        let mut nb: Vec<std::collections::BTreeSet<u32>> = vec![Default::default(); self.vertices.len()];
        for f in &self.faces {
            for k in 0..4 {
                let (a, b) = (f[k], f[(k + 1) % 4]);
                if let Some(s) = nb.get_mut(a as usize) {
                    s.insert(b);
                }
                if let Some(s) = nb.get_mut(b as usize) {
                    s.insert(a);
                }
            }
        }
        nb.iter().map(std::collections::BTreeSet::len).collect()
    }

    /// Tangential relaxation, see [`PolyMesh::relax`].
    pub fn relax(&mut self, iterations: usize, project: &dyn Fn(&Point3, &Vec3) -> Option<(Point3, Vec3)>) {
        let mut poly = self.to_poly();
        poly.relax(iterations, project);
        self.vertices = poly.vertices;
        self.normals = poly.normals;
    }

    /// Quads whose geometric normal points against the vertex normals (folded or inverted).
    #[must_use]
    pub fn folded(&self) -> usize {
        self.faces
            .iter()
            .filter(|f| {
                let p: Vec<Option<&Point3>> = f.iter().map(|&i| self.vertices.get(i as usize)).collect();
                let n: Vec3 = f.iter().filter_map(|&i| self.normals.get(i as usize)).sum();
                match (p[0], p[1], p[2], p[3]) {
                    (Some(a), Some(b), Some(c), Some(d)) => {
                        // Newell normal of the quad
                        let nn = (c - a).cross(&(d - b));
                        nn.dot(&n) <= 0.0
                    }
                    _ => true,
                }
            })
            .count()
    }

    /// Edges used by one face only (the free boundary) as vertex pairs.
    #[must_use]
    pub fn boundary_edges(&self) -> Vec<(u32, u32)> {
        let mut count: BTreeMap<(u32, u32), u32> = BTreeMap::new();
        for f in &self.faces {
            for k in 0..4 {
                let (a, b) = (f[k], f[(k + 1) % 4]);
                *count.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        count.into_iter().filter(|(_, c)| *c == 1).map(|(e, _)| e).collect()
    }

    /// Euler characteristic V − E + F.
    #[must_use]
    pub fn euler(&self) -> i64 {
        let mut edges = std::collections::BTreeSet::new();
        for f in &self.faces {
            for k in 0..4 {
                let (a, b) = (f[k], f[(k + 1) % 4]);
                edges.insert((a.min(b), a.max(b)));
            }
        }
        let used: std::collections::BTreeSet<u32> = self.faces.iter().flatten().copied().collect();
        used.len() as i64 - edges.len() as i64 + self.faces.len() as i64
    }
}
