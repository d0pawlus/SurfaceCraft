//! Objects of a document.
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_geom::{Aabb, Mesh};
use surfacecraft_kernel::record::brep::Brep;

/// A document-unique object id (shared by every kind of object).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize, JsonSchema)]
#[serde(transparent)]
pub struct ObjectId(pub u64);

impl std::fmt::Display for ObjectId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "#{}", self.0)
    }
}

/// What kind of object an id names.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ObjectKind {
    /// A triangle mesh (scan).
    Mesh,
    /// A B-rep body (solid or sheet).
    Body,
}

/// A scan or other triangle mesh.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MeshObject {
    /// Id.
    pub id: ObjectId,
    /// Display name.
    pub name: String,
    /// Geometry (shared between undo snapshots).
    #[serde(with = "crate::format::mesh_b64")]
    pub mesh: Arc<Mesh>,
    /// Shown in the viewport.
    #[serde(default = "yes")]
    pub visible: bool,
}

/// A B-rep body, held as the kernel's neutral record.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BodyObject {
    /// Id.
    pub id: ObjectId,
    /// Display name.
    pub name: String,
    /// Geometry.
    pub brep: Arc<Brep>,
    /// Shown in the viewport.
    #[serde(default = "yes")]
    pub visible: bool,
}

fn yes() -> bool {
    true
}

/// A one-line description of an object, for lists, the UI tree and MCP.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
pub struct ObjectSummary {
    /// Id.
    pub id: ObjectId,
    /// Kind.
    pub kind: ObjectKind,
    /// Name.
    pub name: String,
    /// Shown?
    pub visible: bool,
    /// Short facts (triangle count, face count…).
    pub detail: String,
}

/// The whole model.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Document {
    /// Meshes.
    #[serde(default)]
    pub meshes: Vec<MeshObject>,
    /// B-rep bodies.
    #[serde(default)]
    pub bodies: Vec<BodyObject>,
    /// Next free id.
    #[serde(default)]
    pub next_id: u64,
}

impl Document {
    /// An empty document.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Reserves a fresh id.
    pub fn fresh_id(&mut self) -> ObjectId {
        self.next_id += 1;
        ObjectId(self.next_id)
    }

    /// A name not used by any object, built from `base`.
    #[must_use]
    pub fn unique_name(&self, base: &str) -> String {
        let base = if base.trim().is_empty() { "object" } else { base.trim() };
        if !self.name_taken(base) {
            return base.to_string();
        }
        (2..).map(|i| format!("{base} {i}")).find(|n| !self.name_taken(n)).unwrap_or_else(|| base.to_string())
    }

    fn name_taken(&self, n: &str) -> bool {
        self.meshes.iter().any(|m| m.name == n) || self.bodies.iter().any(|b| b.name == n)
    }

    /// Adds a mesh and returns its id.
    pub fn add_mesh(&mut self, name: &str, mesh: Mesh) -> ObjectId {
        let id = self.fresh_id();
        let name = self.unique_name(name);
        self.meshes.push(MeshObject { id, name, mesh: Arc::new(mesh), visible: true });
        id
    }

    /// Adds a body and returns its id.
    pub fn add_body(&mut self, name: &str, brep: Brep) -> ObjectId {
        let id = self.fresh_id();
        let name = self.unique_name(name);
        self.bodies.push(BodyObject { id, name, brep: Arc::new(brep), visible: true });
        id
    }

    /// The mesh with `id`.
    #[must_use]
    pub fn mesh(&self, id: ObjectId) -> Option<&MeshObject> {
        self.meshes.iter().find(|m| m.id == id)
    }

    /// Mutable mesh with `id`.
    pub fn mesh_mut(&mut self, id: ObjectId) -> Option<&mut MeshObject> {
        self.meshes.iter_mut().find(|m| m.id == id)
    }

    /// The body with `id`.
    #[must_use]
    pub fn body(&self, id: ObjectId) -> Option<&BodyObject> {
        self.bodies.iter().find(|b| b.id == id)
    }

    /// Mutable body with `id`.
    pub fn body_mut(&mut self, id: ObjectId) -> Option<&mut BodyObject> {
        self.bodies.iter_mut().find(|b| b.id == id)
    }

    /// Finds an object by name.
    #[must_use]
    pub fn find_by_name(&self, name: &str) -> Option<(ObjectId, ObjectKind)> {
        self.meshes
            .iter()
            .find(|m| m.name == name)
            .map(|m| (m.id, ObjectKind::Mesh))
            .or_else(|| self.bodies.iter().find(|b| b.name == name).map(|b| (b.id, ObjectKind::Body)))
    }

    /// The kind of object `id` names.
    #[must_use]
    pub fn kind_of(&self, id: ObjectId) -> Option<ObjectKind> {
        if self.mesh(id).is_some() {
            Some(ObjectKind::Mesh)
        } else if self.body(id).is_some() {
            Some(ObjectKind::Body)
        } else {
            None
        }
    }

    /// Removes an object of any kind; true when something was removed.
    pub fn remove(&mut self, id: ObjectId) -> bool {
        let before = self.meshes.len() + self.bodies.len();
        self.meshes.retain(|m| m.id != id);
        self.bodies.retain(|b| b.id != id);
        before != self.meshes.len() + self.bodies.len()
    }

    /// Renames an object; false when it does not exist.
    pub fn rename(&mut self, id: ObjectId, name: &str) -> bool {
        let name = self.unique_name(name);
        if let Some(m) = self.mesh_mut(id) {
            m.name = name;
            true
        } else if let Some(b) = self.body_mut(id) {
            b.name = name;
            true
        } else {
            false
        }
    }

    /// Shows or hides an object.
    pub fn set_visible(&mut self, id: ObjectId, visible: bool) -> bool {
        if let Some(m) = self.mesh_mut(id) {
            m.visible = visible;
            true
        } else if let Some(b) = self.body_mut(id) {
            b.visible = visible;
            true
        } else {
            false
        }
    }

    /// Every object, in creation order.
    #[must_use]
    pub fn summaries(&self) -> Vec<ObjectSummary> {
        let mut v: Vec<ObjectSummary> = self
            .meshes
            .iter()
            .map(|m| ObjectSummary {
                id: m.id,
                kind: ObjectKind::Mesh,
                name: m.name.clone(),
                visible: m.visible,
                detail: format!("{} triangles, {} vertices", m.mesh.triangle_count(), m.mesh.vertex_count()),
            })
            .chain(self.bodies.iter().map(|b| ObjectSummary {
                id: b.id,
                kind: ObjectKind::Body,
                name: b.name.clone(),
                visible: b.visible,
                detail: format!("{} faces, {} edges", b.brep.faces.len(), b.brep.edges.len()),
            }))
            .collect();
        v.sort_by_key(|s| s.id);
        v
    }

    /// Bounding box of every visible mesh.
    #[must_use]
    pub fn bbox(&self) -> Aabb {
        let mut b = Aabb::empty();
        for m in self.meshes.iter().filter(|m| m.visible) {
            let mb = m.mesh.bbox();
            if !mb.is_empty() {
                b.grow(&mb.min);
                b.grow(&mb.max);
            }
        }
        for body in self.bodies.iter().filter(|b| b.visible) {
            for v in &body.brep.vertices {
                b.grow(&surfacecraft_geom::Point3::new(v.point[0], v.point[1], v.point[2]));
            }
        }
        b
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_names_and_removal() {
        let mut d = Document::new();
        let a = d.add_mesh("scan", Mesh::default());
        let b = d.add_mesh("scan", Mesh::default());
        assert_ne!(a, b);
        assert_eq!(d.mesh(b).unwrap().name, "scan 2");
        assert_eq!(d.find_by_name("scan 2"), Some((b, ObjectKind::Mesh)));
        assert!(d.remove(a));
        assert!(!d.remove(a));
        assert_eq!(d.summaries().len(), 1);
        assert!(d.rename(b, "part"));
        assert_eq!(d.kind_of(b), Some(ObjectKind::Mesh));
    }
}
