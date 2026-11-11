//! What the viewport draws: every visible mesh (with its deviation map, or its regions in
//! colour), every visible body tessellated through the kernel, and every visible primitive's
//! patch. Derived data is cached, so it is rebuilt only after the geometry changes.
use crate::Session;
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::PrimitiveKind;
use surfacecraft_geom::{Mesh, lod::Lod};

/// Meshes with more triangles than this are drawn through a coarser level of detail (P2).
pub const LOD_BUDGET: usize = 2_000_000;

/// One drawable.
#[derive(Clone, Debug)]
pub struct DisplayItem {
    /// Object.
    pub id: ObjectId,
    /// Kind.
    pub kind: ObjectKind,
    /// Triangles to draw (a coarse level of detail for very large scans).
    pub mesh: Arc<Mesh>,
    /// Changes whenever the drawn geometry does (GPU buffers are re-uploaded on change).
    pub key: usize,
    /// Optional per-vertex colours of `mesh` (deviation maps, regions), RGBA.
    pub colors: Option<Arc<Vec<[f32; 4]>>>,
    /// Identity of `colors`: changes whenever their content does (only the colour buffer is
    /// rewritten on change). 0 without colours.
    pub colors_key: u64,
    /// The full-resolution mesh: picking and selections refer to its triangle ids.
    pub pick_mesh: Arc<Mesh>,
    /// Changes whenever `pick_mesh` does.
    pub pick_key: usize,
    /// How `mesh` derives from `pick_mesh` (`None` when drawn at full resolution).
    pub lod: Option<Arc<Lod>>,
    /// B-rep identities of a body's display mesh (faces and edges the viewport can pick).
    pub topo: Option<Arc<BodyTopo>>,
}

/// What of a body's B-rep the viewport can pick: the face of every display triangle (grouped
/// into per-face triangle ranges) and the edge polylines, indexed as in the body record, so the
/// indices go straight into the `edges` / `faces` parameters of feature commands.
#[derive(Clone, Debug, Default)]
pub struct BodyTopo {
    /// B-rep face of every triangle of the display mesh.
    pub tri_face: Vec<u32>,
    /// `(face, first triangle, one past the last)` for each run of triangles of one face.
    pub face_ranges: Vec<(u32, u32, u32)>,
    /// Edge polylines in the body's edge order.
    pub edges: Vec<Vec<surfacecraft_geom::Point3>>,
}

impl BodyTopo {
    /// Builds the per-face ranges from the per-triangle face list.
    #[must_use]
    pub fn new(tri_face: Vec<u32>, edges: Vec<Vec<surfacecraft_geom::Point3>>) -> Self {
        let mut face_ranges: Vec<(u32, u32, u32)> = Vec::new();
        for (i, &f) in tri_face.iter().enumerate() {
            let i = i as u32;
            match face_ranges.last_mut() {
                Some(r) if r.0 == f && r.2 == i => r.2 = i + 1,
                _ => face_ranges.push((f, i, i + 1)),
            }
        }
        Self { tri_face, face_ranges, edges }
    }

    /// The face of triangle `t`.
    #[must_use]
    pub fn face_of(&self, t: u32) -> Option<u32> {
        self.tri_face.get(t as usize).copied()
    }

    /// The display triangles of the given faces (ascending).
    #[must_use]
    pub fn triangles_of_faces(&self, faces: &[usize]) -> Vec<u32> {
        let mut out = Vec::new();
        for &(f, a, b) in &self.face_ranges {
            if faces.contains(&(f as usize)) {
                out.extend(a..b);
            }
        }
        out
    }
}

impl DisplayItem {
    /// An item drawn and picked at full resolution without colours.
    #[must_use]
    pub fn plain(id: ObjectId, kind: ObjectKind, mesh: Arc<Mesh>) -> Self {
        let key = Arc::as_ptr(&mesh) as usize;
        Self {
            id,
            kind,
            key,
            colors: None,
            colors_key: 0,
            pick_mesh: mesh.clone(),
            pick_key: key,
            lod: None,
            topo: None,
            mesh,
        }
    }

    /// Drawn triangles for a selection of full-resolution triangle ids.
    #[must_use]
    pub fn drawn_triangles(&self, tris: &[u32]) -> Vec<u32> {
        match &self.lod {
            Some(l) => l.map_triangles(tris),
            None => tris.to_vec(),
        }
    }
}

/// Caches of derived display data.
#[derive(Debug, Default)]
pub struct TessCache {
    entries: HashMap<ObjectId, (usize, Arc<Mesh>, Arc<BodyTopo>)>,
    region_colors: HashMap<ObjectId, (u64, Arc<Vec<[f32; 4]>>)>,
    quads: HashMap<ObjectId, (u64, Arc<Mesh>)>,
    /// Display level of detail per scan: (mesh pointer, budget, lod, lod mesh).
    lods: HashMap<ObjectId, (usize, usize, Arc<Lod>, Arc<Mesh>)>,
    /// Colours mapped onto the lod: (source colours, lod mesh pointer, mapped).
    lod_colors: HashMap<ObjectId, (Arc<Vec<[f32; 4]>>, usize, Arc<Vec<[f32; 4]>>)>,
    /// Identity of the colours last shown per mesh (the Arc is kept so its address cannot be
    /// reused while it is the reference).
    color_ident: HashMap<ObjectId, (Arc<Vec<[f32; 4]>>, u64)>,
    counter: u64,
    /// Triangle budget for the level of detail (0 = [`LOD_BUDGET`]).
    pub budget: usize,
}

/// Colour of a region by its type (unclassified regions cycle a neutral palette).
#[must_use]
pub fn region_color(kind: Option<PrimitiveKind>, index: usize) -> [f32; 4] {
    match kind {
        Some(PrimitiveKind::Plane) => [0.35, 0.6, 0.95, 1.0],
        Some(PrimitiveKind::Cylinder) => [0.95, 0.6, 0.25, 1.0],
        Some(PrimitiveKind::Cone) => [0.85, 0.4, 0.8, 1.0],
        Some(PrimitiveKind::Sphere) => [0.4, 0.85, 0.45, 1.0],
        Some(PrimitiveKind::Torus) => [0.95, 0.85, 0.3, 1.0],
        None => {
            const P: [[f32; 4]; 4] =
                [[0.7, 0.55, 0.55, 1.0], [0.55, 0.7, 0.6, 1.0], [0.6, 0.6, 0.75, 1.0], [0.75, 0.7, 0.5, 1.0]];
            P[index % P.len()]
        }
    }
}

const MESH_GREY: [f32; 4] = [0.62, 0.66, 0.72, 1.0];

impl Session {
    fn region_overlay(&mut self, mesh_id: ObjectId, mesh: &Mesh, mesh_key: usize) -> Option<Arc<Vec<[f32; 4]>>> {
        let regions: Vec<(usize, Arc<Vec<u32>>, Option<PrimitiveKind>)> = self
            .doc
            .regions_of(mesh_id)
            .filter(|r| r.visible)
            .enumerate()
            .map(|(k, r)| (k, r.triangles.clone(), r.kind))
            .collect();
        if regions.is_empty() {
            self.tess.region_colors.remove(&mesh_id);
            return None;
        }
        let mut key = mesh_key as u64;
        for (k, t, kind) in &regions {
            key = key.wrapping_mul(1_099_511_628_211) ^ (Arc::as_ptr(t) as u64) ^ ((*k as u64) << 48);
            key ^= kind.map_or(7, |k| k as u64 + 1);
        }
        if let Some((k, c)) = self.tess.region_colors.get(&mesh_id)
            && *k == key
        {
            return Some(c.clone());
        }
        let mut colors = vec![MESH_GREY; mesh.vertex_count()];
        for (k, tris, kind) in &regions {
            let c = region_color(*kind, *k);
            for &t in tris.iter() {
                for &vi in mesh.triangles.get(t as usize).into_iter().flatten() {
                    if let Some(x) = colors.get_mut(vi as usize) {
                        *x = c;
                    }
                }
            }
        }
        let c = Arc::new(colors);
        self.tess.region_colors.insert(mesh_id, (key, c.clone()));
        Some(c)
    }

    /// The level of detail of a large scan (cached by mesh pointer and budget).
    fn lod_of(&mut self, id: ObjectId, mesh: &Arc<Mesh>, budget: usize) -> Option<(Arc<Lod>, Arc<Mesh>)> {
        let ptr = Arc::as_ptr(mesh) as usize;
        if let Some((p, b, l, m)) = self.tess.lods.get(&id)
            && *p == ptr
            && *b == budget
        {
            return Some((l.clone(), m.clone()));
        }
        self.tess.lod_colors.remove(&id);
        let lod = Arc::new(surfacecraft_geom::lod::cluster(mesh, budget)?);
        let m = Arc::new(lod.mesh.clone());
        self.tess.lods.insert(id, (ptr, budget, lod.clone(), m.clone()));
        Some((lod, m))
    }

    fn lod_colors_of(&mut self, id: ObjectId, c: &Arc<Vec<[f32; 4]>>, l: &Lod, m: &Arc<Mesh>) -> Arc<Vec<[f32; 4]>> {
        let mp = Arc::as_ptr(m) as usize;
        if let Some((src, p, mapped)) = self.tess.lod_colors.get(&id)
            && Arc::ptr_eq(src, c)
            && *p == mp
        {
            return mapped.clone();
        }
        let mapped = Arc::new(l.map_colors(c));
        self.tess.lod_colors.insert(id, (c.clone(), mp, mapped.clone()));
        mapped
    }

    /// A number that changes whenever the colours of mesh `id` are a different array.
    fn color_stamp(&mut self, id: ObjectId, c: &Arc<Vec<[f32; 4]>>) -> u64 {
        if let Some((a, g)) = self.tess.color_ident.get(&id)
            && Arc::ptr_eq(a, c)
        {
            return *g;
        }
        self.tess.counter += 1;
        let g = self.tess.counter;
        self.tess.color_ident.insert(id, (c.clone(), g));
        g
    }

    /// Everything visible: meshes, bodies, primitives.
    pub fn display_items(&mut self) -> Vec<DisplayItem> {
        let meshes: Vec<(ObjectId, Arc<Mesh>)> =
            self.doc.meshes.iter().filter(|m| m.visible).map(|m| (m.id, m.mesh.clone())).collect();
        let mut out = Vec::new();
        for (id, mesh) in meshes {
            let pick_key = Arc::as_ptr(&mesh) as usize;
            let colors = match self.live_map(id) {
                Some(map) if map.colours.len() == mesh.vertex_count() => Some(map.colours.clone()),
                _ => self.region_overlay(id, &mesh, pick_key),
            };
            let budget = if self.tess.budget == 0 { LOD_BUDGET } else { self.tess.budget };
            let lod = if mesh.triangle_count() > budget { self.lod_of(id, &mesh, budget) } else { None };
            let (drawn, key, drawn_colors) = match &lod {
                Some((l, m)) => {
                    let c = colors.as_ref().map(|c| self.lod_colors_of(id, c, l, m));
                    (m.clone(), Arc::as_ptr(m) as usize, c)
                }
                None => {
                    self.tess.lods.remove(&id);
                    self.tess.lod_colors.remove(&id);
                    (mesh.clone(), pick_key, colors.clone())
                }
            };
            let colors_key = match &colors {
                Some(c) => self.color_stamp(id, c),
                None => {
                    self.tess.color_ident.remove(&id);
                    0
                }
            };
            out.push(DisplayItem {
                id,
                kind: ObjectKind::Mesh,
                key,
                mesh: drawn,
                colors: drawn_colors,
                colors_key,
                pick_mesh: mesh,
                pick_key,
                lod: lod.map(|(l, _)| l),
                topo: None,
            });
        }
        let live: Vec<ObjectId> = self.doc.bodies.iter().map(|b| b.id).collect();
        self.tess.entries.retain(|id, _| live.contains(id));
        for b in self.doc.bodies.iter().filter(|b| b.visible) {
            let key = Arc::as_ptr(&b.brep) as usize;
            let (mesh, topo) = match self.tess.entries.get(&b.id) {
                Some((k, m, t)) if *k == key => (m.clone(), t.clone()),
                _ => {
                    let tm = surfacecraft_kernel::tess::tessellate_topo(&b.brep, None).unwrap_or_default();
                    let topo = Arc::new(BodyTopo::new(tm.tri_face, tm.edges));
                    let m = Arc::new(tm.mesh);
                    self.tess.entries.insert(b.id, (key, m.clone(), topo.clone()));
                    (m, topo)
                }
            };
            let mut item = DisplayItem::plain(b.id, ObjectKind::Body, mesh);
            item.topo = Some(topo);
            out.push(item);
        }
        for o in self.doc.surfaces.iter().filter(|o| o.visible) {
            out.push(DisplayItem::plain(o.id, ObjectKind::Surface, o.display.clone()));
        }
        let quads: Vec<_> =
            self.doc.quads.iter().filter(|q| q.visible).map(|q| (q.id, q.vertices.clone(), q.clone())).collect();
        for (id, verts, q) in quads {
            let mut key = verts.len() as u64 ^ ((q.faces.len() as u64) << 40);
            for v in &verts {
                for c in v {
                    key = key.wrapping_mul(1_099_511_628_211) ^ c.to_bits();
                }
            }
            let mesh = match self.tess.quads.get(&id) {
                Some((k, m)) if *k == key => m.clone(),
                _ => {
                    let m = Arc::new(q.display_mesh());
                    self.tess.quads.insert(id, (key, m.clone()));
                    m
                }
            };
            out.push(DisplayItem::plain(id, ObjectKind::Quad, mesh));
        }
        for k in self.doc.sketches.iter().filter(|k| k.visible) {
            let m = Arc::new(sketch_ribbons(&k.sketch));
            if m.triangle_count() > 0 {
                out.push(DisplayItem::plain(k.id, ObjectKind::Sketch, m));
            }
        }
        for p in self.doc.primitives.iter().filter(|p| p.visible) {
            out.push(DisplayItem::plain(p.id, ObjectKind::Primitive, p.patch.clone()));
        }
        out
    }
}

/// Sketch entities drawn as thin ribbons in their plane (the viewport draws triangles).
fn sketch_ribbons(s: &surfacecraft_sketch::Sketch) -> Mesh {
    use surfacecraft_sketch::Entity;
    let pt = |i: usize| s.points.get(i).copied().unwrap_or([0.0, 0.0]);
    let mut polylines: Vec<Vec<[f64; 2]>> = Vec::new();
    for e in &s.entities {
        match e {
            Entity::Line { a, b } => polylines.push(vec![pt(*a), pt(*b)]),
            Entity::Arc { center, a, b } => {
                let (c, pa, pb) = (pt(*center), pt(*a), pt(*b));
                let r = (pa[0] - c[0]).hypot(pa[1] - c[1]);
                let a0 = (pa[1] - c[1]).atan2(pa[0] - c[0]);
                let sweep = ((pb[1] - c[1]).atan2(pb[0] - c[0]) - a0).rem_euclid(std::f64::consts::TAU);
                polylines.push(
                    (0..=24)
                        .map(|k| {
                            let t = a0 + sweep * f64::from(k) / 24.0;
                            [c[0] + r * t.cos(), c[1] + r * t.sin()]
                        })
                        .collect(),
                );
            }
            Entity::Circle { center, radius } => {
                let c = pt(*center);
                polylines.push(
                    (0..=48)
                        .map(|k| {
                            let t = std::f64::consts::TAU * f64::from(k) / 48.0;
                            [c[0] + radius * t.cos(), c[1] + radius * t.sin()]
                        })
                        .collect(),
                );
            }
        }
    }
    let size = s.points.iter().fold(1.0f64, |m, p| m.max(p[0].abs()).max(p[1].abs()));
    let w = size * 0.004;
    let mut m = Mesh::default();
    for pl in polylines {
        for seg in pl.windows(2) {
            let (a, b) = (seg[0], seg[1]);
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let n = dx.hypot(dy).max(1e-12);
            let (ox, oy) = (-dy / n * w, dx / n * w);
            let base = m.positions.len() as u32;
            for q in [[a[0] + ox, a[1] + oy], [b[0] + ox, b[1] + oy], [b[0] - ox, b[1] - oy], [a[0] - ox, a[1] - oy]] {
                m.positions.push(s.plane.to_3d(q));
            }
            m.triangles.push([base, base + 1, base + 2]);
            m.triangles.push([base, base + 2, base + 3]);
        }
    }
    m
}
