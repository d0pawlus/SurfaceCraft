//! What the viewport draws: every visible mesh (with its deviation map, or its regions in
//! colour), every visible body tessellated through the kernel, and every visible primitive's
//! patch. Derived data is cached, so it is rebuilt only after the geometry changes.
use crate::Session;
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_fit::PrimitiveKind;
use surfacecraft_geom::Mesh;

/// One drawable.
#[derive(Clone, Debug)]
pub struct DisplayItem {
    /// Object.
    pub id: ObjectId,
    /// Kind.
    pub kind: ObjectKind,
    /// Triangles to draw.
    pub mesh: Arc<Mesh>,
    /// Changes whenever the geometry does (GPU buffers are re-uploaded on change).
    pub key: usize,
    /// Optional per-vertex colours (deviation maps, regions), RGBA.
    pub colors: Option<Arc<Vec<[f32; 4]>>>,
}

/// Caches of derived display data.
#[derive(Debug, Default)]
pub struct TessCache {
    entries: HashMap<ObjectId, (usize, Arc<Mesh>)>,
    region_colors: HashMap<ObjectId, (u64, Arc<Vec<[f32; 4]>>)>,
    quads: HashMap<ObjectId, (u64, Arc<Mesh>)>,
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

    /// Everything visible: meshes, bodies, primitives.
    pub fn display_items(&mut self) -> Vec<DisplayItem> {
        let meshes: Vec<(ObjectId, Arc<Mesh>)> =
            self.doc.meshes.iter().filter(|m| m.visible).map(|m| (m.id, m.mesh.clone())).collect();
        let mut out = Vec::new();
        for (id, mesh) in meshes {
            let key = Arc::as_ptr(&mesh) as usize;
            let colors = match self.overlays.get(&id) {
                Some(c) if c.len() == mesh.vertex_count() => Some(c.clone()),
                _ => self.region_overlay(id, &mesh, key),
            };
            out.push(DisplayItem { id, kind: ObjectKind::Mesh, key, mesh, colors });
        }
        let live: Vec<ObjectId> = self.doc.bodies.iter().map(|b| b.id).collect();
        self.tess.entries.retain(|id, _| live.contains(id));
        for b in self.doc.bodies.iter().filter(|b| b.visible) {
            let key = Arc::as_ptr(&b.brep) as usize;
            let mesh = match self.tess.entries.get(&b.id) {
                Some((k, m)) if *k == key => m.clone(),
                _ => {
                    let m = Arc::new(surfacecraft_kernel::tess::tessellate(&b.brep, None).unwrap_or_default());
                    self.tess.entries.insert(b.id, (key, m.clone()));
                    m
                }
            };
            out.push(DisplayItem {
                id: b.id,
                kind: ObjectKind::Body,
                key: Arc::as_ptr(&mesh) as usize,
                mesh,
                colors: None,
            });
        }
        for o in self.doc.surfaces.iter().filter(|o| o.visible) {
            out.push(DisplayItem {
                id: o.id,
                kind: ObjectKind::Surface,
                key: Arc::as_ptr(&o.display) as usize,
                mesh: o.display.clone(),
                colors: None,
            });
        }
        let quads: Vec<_> =
            self.doc.quads.iter().filter(|q| q.visible).map(|q| (q.id, q.vertices.clone(), q.clone())).collect();
        for (id, verts, q) in quads {
            let mut key = verts.len() as u64;
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
            out.push(DisplayItem { id, kind: ObjectKind::Quad, key: Arc::as_ptr(&mesh) as usize, mesh, colors: None });
        }
        for k in self.doc.sketches.iter().filter(|k| k.visible) {
            let m = Arc::new(sketch_ribbons(&k.sketch));
            if m.triangle_count() > 0 {
                out.push(DisplayItem {
                    id: k.id,
                    kind: ObjectKind::Sketch,
                    key: Arc::as_ptr(&m) as usize,
                    mesh: m,
                    colors: None,
                });
            }
        }
        for p in self.doc.primitives.iter().filter(|p| p.visible) {
            out.push(DisplayItem {
                id: p.id,
                kind: ObjectKind::Primitive,
                key: Arc::as_ptr(&p.patch) as usize,
                mesh: p.patch.clone(),
                colors: None,
            });
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
