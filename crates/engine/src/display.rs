//! What the viewport draws: every visible mesh (with its deviation map, or its regions in
//! colour), every visible body tessellated through the kernel, and every visible primitive's
//! patch. Derived data is cached, so it is rebuilt only after the geometry changes.
use crate::Session;
use std::collections::{HashMap, HashSet};
use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::PrimitiveKind;
use surfacecraft_geom::{Mesh, lod::Lod};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_sketch::Sketch;

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

/// Caches of derived display data. Every entry holds the geometry it was derived from, so an
/// entry is reused only for that very geometry (an address alone could be handed to new geometry
/// once the old one is freed), and entries of objects no longer in the document are dropped.
#[derive(Clone, Debug, Default)]
pub struct TessCache {
    /// Display mesh and B-rep identities per body: (B-rep, mesh, topology).
    entries: HashMap<ObjectId, (Arc<Brep>, Arc<Mesh>, Arc<BodyTopo>)>,
    /// Region colours per mesh: (mesh, its visible regions, colours).
    region_colors: HashMap<ObjectId, RegionColors>,
    /// Display mesh per quad layout: (the layout's geometry, mesh).
    quads: HashMap<ObjectId, (QuadShape, Arc<Mesh>)>,
    /// Ribbons per sketch: (the sketch, ribbons).
    sketches: HashMap<ObjectId, (Sketch, Arc<Mesh>)>,
    /// Display level of detail per scan: (source mesh, budget, lod, lod mesh).
    lods: HashMap<ObjectId, (Arc<Mesh>, usize, Arc<Lod>, Arc<Mesh>)>,
    /// Colours mapped onto the lod: (source colours, lod mesh, mapped).
    lod_colors: HashMap<ObjectId, (Arc<Vec<[f32; 4]>>, Arc<Mesh>, Arc<Vec<[f32; 4]>>)>,
    /// Identity of the colours last shown per mesh.
    color_ident: HashMap<ObjectId, (Arc<Vec<[f32; 4]>>, u64)>,
    /// The display key of every drawn mesh, by its address (the entry holds the mesh, so the
    /// address stays its own while the entry lives).
    idents: HashMap<usize, (Arc<Mesh>, usize)>,
    /// Triangle budget for the level of detail (0 = [`LOD_BUDGET`]).
    pub budget: usize,
}

/// A mesh's region colours with what they were made from: (mesh, visible regions in order, colours).
type RegionColors = (Arc<Mesh>, Vec<(Arc<Vec<u32>>, Option<PrimitiveKind>)>, Arc<Vec<[f32; 4]>>);

/// What a quad layout's display mesh is made from: vertices, faces and the grid size.
type QuadShape = (Vec<[f64; 3]>, Vec<[u32; 4]>, usize, usize);

/// The source of display keys and colour stamps: one counter for every session (a preview copy
/// shares the cache of its session, so the two must never hand out the same number).
static NEXT_STAMP: AtomicUsize = AtomicUsize::new(1);

fn next_stamp() -> usize {
    NEXT_STAMP.fetch_add(1, Ordering::Relaxed)
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
    fn region_overlay(&mut self, mesh_id: ObjectId, mesh: &Arc<Mesh>) -> Option<Arc<Vec<[f32; 4]>>> {
        let regions: Vec<(Arc<Vec<u32>>, Option<PrimitiveKind>)> =
            self.doc.regions_of(mesh_id).filter(|r| r.visible).map(|r| (r.triangles.clone(), r.kind)).collect();
        if regions.is_empty() {
            self.tess.region_colors.remove(&mesh_id);
            return None;
        }
        if let Some((m, rs, c)) = self.tess.region_colors.get(&mesh_id)
            && Arc::ptr_eq(m, mesh)
            && rs.len() == regions.len()
            && rs.iter().zip(&regions).all(|((ta, ka), (tb, kb))| Arc::ptr_eq(ta, tb) && ka == kb)
        {
            return Some(c.clone());
        }
        let mut colors = vec![MESH_GREY; mesh.vertex_count()];
        for (k, (tris, kind)) in regions.iter().enumerate() {
            let c = region_color(*kind, k);
            for &t in tris.iter() {
                for &vi in mesh.triangles.get(t as usize).into_iter().flatten() {
                    if let Some(x) = colors.get_mut(vi as usize) {
                        *x = c;
                    }
                }
            }
        }
        let c = Arc::new(colors);
        self.tess.region_colors.insert(mesh_id, (mesh.clone(), regions, c.clone()));
        Some(c)
    }

    /// The level of detail of a large scan (cached by mesh and budget).
    fn lod_of(&mut self, id: ObjectId, mesh: &Arc<Mesh>, budget: usize) -> Option<(Arc<Lod>, Arc<Mesh>)> {
        if let Some((src, b, l, m)) = self.tess.lods.get(&id)
            && Arc::ptr_eq(src, mesh)
            && *b == budget
        {
            return Some((l.clone(), m.clone()));
        }
        self.tess.lod_colors.remove(&id);
        let lod = Arc::new(surfacecraft_geom::lod::cluster(mesh, budget)?);
        let m = Arc::new(lod.mesh.clone());
        self.tess.lods.insert(id, (mesh.clone(), budget, lod.clone(), m.clone()));
        Some((lod, m))
    }

    fn lod_colors_of(&mut self, id: ObjectId, c: &Arc<Vec<[f32; 4]>>, l: &Lod, m: &Arc<Mesh>) -> Arc<Vec<[f32; 4]>> {
        if let Some((src, lm, mapped)) = self.tess.lod_colors.get(&id)
            && Arc::ptr_eq(src, c)
            && Arc::ptr_eq(lm, m)
        {
            return mapped.clone();
        }
        let mapped = Arc::new(l.map_colors(c));
        self.tess.lod_colors.insert(id, (c.clone(), m.clone(), mapped.clone()));
        mapped
    }

    /// A number that changes whenever the colours of mesh `id` are a different array.
    fn color_stamp(&mut self, id: ObjectId, c: &Arc<Vec<[f32; 4]>>) -> u64 {
        if let Some((a, g)) = self.tess.color_ident.get(&id)
            && Arc::ptr_eq(a, c)
        {
            return *g;
        }
        let g = next_stamp() as u64;
        self.tess.color_ident.insert(id, (c.clone(), g));
        g
    }

    /// The display key of `mesh`: the same while it is the same mesh, never one used before.
    fn ident(&mut self, mesh: &Arc<Mesh>) -> usize {
        let ptr = Arc::as_ptr(mesh) as usize;
        if let Some((m, k)) = self.tess.idents.get(&ptr)
            && Arc::ptr_eq(m, mesh)
        {
            return *k;
        }
        let k = next_stamp();
        self.tess.idents.insert(ptr, (mesh.clone(), k));
        k
    }

    /// Drops what was derived from objects no longer in the document, and the keys of meshes
    /// nothing but the cache holds any more.
    fn prune_display_cache(&mut self) {
        let ids = |v: &mut dyn Iterator<Item = ObjectId>| v.collect::<HashSet<ObjectId>>();
        let meshes = ids(&mut self.doc.meshes.iter().map(|m| m.id));
        let bodies = ids(&mut self.doc.bodies.iter().map(|b| b.id));
        let quads = ids(&mut self.doc.quads.iter().map(|q| q.id));
        let sketches = ids(&mut self.doc.sketches.iter().map(|k| k.id));
        let t = &mut self.tess;
        t.entries.retain(|id, _| bodies.contains(id));
        t.region_colors.retain(|id, _| meshes.contains(id));
        t.lods.retain(|id, _| meshes.contains(id));
        t.lod_colors.retain(|id, _| meshes.contains(id));
        t.color_ident.retain(|id, _| meshes.contains(id));
        t.quads.retain(|id, _| quads.contains(id));
        t.sketches.retain(|id, _| sketches.contains(id));
        t.idents.retain(|_, (m, _)| Arc::strong_count(m) > 1);
    }

    /// Everything visible: meshes, bodies, primitives.
    pub fn display_items(&mut self) -> Vec<DisplayItem> {
        self.prune_display_cache();
        let mut out = self.mesh_items();
        self.body_items(&mut out);
        for o in self.doc.surfaces.iter().filter(|o| o.visible) {
            out.push(DisplayItem::plain(o.id, ObjectKind::Surface, o.display.clone()));
        }
        self.quad_items(&mut out);
        self.sketch_items(&mut out);
        for p in self.doc.primitives.iter().filter(|p| p.visible) {
            out.push(DisplayItem::plain(p.id, ObjectKind::Primitive, p.patch.clone()));
        }
        for d in &mut out {
            d.key = self.ident(&d.mesh);
            d.pick_key = self.ident(&d.pick_mesh);
        }
        out
    }

    fn mesh_items(&mut self) -> Vec<DisplayItem> {
        let meshes: Vec<(ObjectId, Arc<Mesh>)> =
            self.doc.meshes.iter().filter(|m| m.visible).map(|m| (m.id, m.mesh.clone())).collect();
        let mut out = Vec::new();
        for (id, mesh) in meshes {
            let colors = match self.live_map(id) {
                Some(map) if map.colours.len() == mesh.vertex_count() => Some(map.colours.clone()),
                _ => self.region_overlay(id, &mesh),
            };
            let budget = if self.tess.budget == 0 { LOD_BUDGET } else { self.tess.budget };
            let lod = if mesh.triangle_count() > budget { self.lod_of(id, &mesh, budget) } else { None };
            let (drawn, drawn_colors) = match &lod {
                Some((l, m)) => {
                    let c = colors.as_ref().map(|c| self.lod_colors_of(id, c, l, m));
                    (m.clone(), c)
                }
                None => {
                    self.tess.lods.remove(&id);
                    self.tess.lod_colors.remove(&id);
                    (mesh.clone(), colors.clone())
                }
            };
            let colors_key = match &colors {
                Some(c) => self.color_stamp(id, c),
                None => {
                    self.tess.color_ident.remove(&id);
                    0
                }
            };
            // keys are set from the meshes at the end of `display_items`
            out.push(DisplayItem {
                id,
                kind: ObjectKind::Mesh,
                key: 0,
                mesh: drawn,
                colors: drawn_colors,
                colors_key,
                pick_mesh: mesh,
                pick_key: 0,
                lod: lod.map(|(l, _)| l),
                topo: None,
            });
        }
        out
    }

    fn body_items(&mut self, out: &mut Vec<DisplayItem>) {
        let (doc, tess) = (&self.doc, &mut self.tess);
        for b in doc.bodies.iter().filter(|b| b.shown()) {
            let (mesh, topo) = match tess.entries.get(&b.id) {
                Some((brep, m, t)) if Arc::ptr_eq(brep, &b.brep) => (m.clone(), t.clone()),
                _ => {
                    let tm = surfacecraft_kernel::tess::tessellate_topo(&b.brep, None).unwrap_or_default();
                    let topo = Arc::new(BodyTopo::new(tm.tri_face, tm.edges));
                    let m = Arc::new(tm.mesh);
                    tess.entries.insert(b.id, (b.brep.clone(), m.clone(), topo.clone()));
                    (m, topo)
                }
            };
            let mut item = DisplayItem::plain(b.id, ObjectKind::Body, mesh);
            item.topo = Some(topo);
            out.push(item);
        }
    }

    fn quad_items(&mut self, out: &mut Vec<DisplayItem>) {
        let (doc, tess) = (&self.doc, &mut self.tess);
        for q in doc.quads.iter().filter(|q| q.visible) {
            let mesh = match tess.quads.get(&q.id) {
                Some(((v, f, nu, nv), m)) if *v == q.vertices && *f == q.faces && *nu == q.nu && *nv == q.nv => {
                    m.clone()
                }
                _ => {
                    let m = Arc::new(q.display_mesh());
                    tess.quads.insert(q.id, ((q.vertices.clone(), q.faces.clone(), q.nu, q.nv), m.clone()));
                    m
                }
            };
            out.push(DisplayItem::plain(q.id, ObjectKind::Quad, mesh));
        }
    }

    fn sketch_items(&mut self, out: &mut Vec<DisplayItem>) {
        let (doc, tess) = (&self.doc, &mut self.tess);
        for k in doc.sketches.iter().filter(|k| k.visible) {
            let mesh = match tess.sketches.get(&k.id) {
                Some((s, m)) if *s == k.sketch => m.clone(),
                _ => {
                    let m = Arc::new(sketch_ribbons(&k.sketch));
                    tess.sketches.insert(k.id, (k.sketch.clone(), m.clone()));
                    m
                }
            };
            if mesh.triangle_count() > 0 {
                out.push(DisplayItem::plain(k.id, ObjectKind::Sketch, mesh));
            }
        }
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

#[cfg(test)]
mod tests {
    use crate::Session;
    use serde_json::json;
    use surfacecraft_geom::{Point3, synth};

    #[test]
    fn derived_display_data_goes_with_its_object() {
        let mut s = Session::new();
        s.tess.budget = 500;
        let scan = s.doc.add_mesh("scan", synth::sphere(Point3::origin(), 10.0, 40, 20));
        s.execute("body.box", json!({"size": [20.0, 10.0, 5.0]})).unwrap();
        s.execute(
            "sketch.create",
            json!({"name": "sq", "plane": {"origin": [0.0, 0.0, 0.0], "x": [1.0, 0.0, 0.0], "y": [0.0, 1.0, 0.0]}}),
        )
        .unwrap();
        s.execute(
            "sketch.add",
            json!({"sketch": "sq", "entities": [{"polygon": {"points": [[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]]}}]}),
        )
        .unwrap();
        let shown = s.display_items();
        assert_eq!(shown.len(), 3);
        assert_eq!((s.tess.lods.len(), s.tess.entries.len(), s.tess.sketches.len()), (1, 1, 1));
        drop(shown);
        let ids: Vec<_> = s.doc.summaries().iter().map(|o| o.id).collect();
        for id in ids {
            assert!(s.doc.remove(id));
        }
        // undo history still holds the removed geometry: the display keys it had may stay, the
        // cache entries of objects no longer in the document may not
        s.history = surfacecraft_doc::History::default();
        assert!(s.display_items().is_empty());
        let t = &s.tess;
        assert!(t.lods.is_empty() && t.entries.is_empty() && t.sketches.is_empty(), "{t:?}");
        assert!(t.idents.is_empty(), "nothing holds the old meshes, so their keys are gone");
        assert!(!s.doc.summaries().iter().any(|o| o.id == scan));
    }
}
