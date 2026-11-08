//! What the viewport draws: every visible mesh, and every visible body tessellated through the
//! kernel (cached per record, so a body is meshed again only after it changes).
use crate::Session;
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
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
    /// Optional per-vertex colours (deviation maps), RGBA.
    pub colors: Option<Arc<Vec<[f32; 4]>>>,
}

/// Cache of body display meshes.
#[derive(Debug, Default)]
pub struct TessCache {
    entries: HashMap<ObjectId, (usize, Arc<Mesh>)>,
}

impl Session {
    /// Everything visible, meshes first.
    pub fn display_items(&mut self) -> Vec<DisplayItem> {
        let mut out: Vec<DisplayItem> = self
            .doc
            .meshes
            .iter()
            .filter(|m| m.visible)
            .map(|m| DisplayItem {
                id: m.id,
                kind: ObjectKind::Mesh,
                key: Arc::as_ptr(&m.mesh) as usize,
                mesh: m.mesh.clone(),
                colors: self.overlays.get(&m.id).cloned(),
            })
            .collect();
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
        out
    }
}
