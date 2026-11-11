//! Working copies (non-destructive editing): an editable copy kept beside a mesh's original.
//!
//! While a copy exists, mesh edits act on it and the original stays as it was, so the copy can be
//! measured against the original, promoted to replace it, or discarded. A rigid transform moves
//! both, which keeps the distance between them unchanged.
use crate::object::{Document, MeshObject, ObjectId};
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_geom::{Mesh, Transform};

/// The editable copy of a mesh.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct WorkingCopy {
    /// Geometry (shared between undo snapshots, like the original).
    #[serde(with = "crate::format::mesh_b64")]
    pub mesh: Arc<Mesh>,
}

impl MeshObject {
    /// Is there a working copy?
    #[must_use]
    pub fn is_working(&self) -> bool {
        self.working.is_some()
    }

    /// The geometry edits act on: the working copy when there is one, else the original.
    #[must_use]
    pub fn editable(&self) -> &Arc<Mesh> {
        self.working.as_ref().map_or(&self.mesh, |w| &w.mesh)
    }

    /// Mutable access to the geometry edits act on (see [`MeshObject::editable`]).
    pub fn editable_mut(&mut self) -> &mut Arc<Mesh> {
        match self.working.as_mut() {
            Some(w) => &mut w.mesh,
            None => &mut self.mesh,
        }
    }

    /// Moves the original and the working copy (if any) by the rigid transform `t`.
    pub fn transform_rigid(&mut self, t: &Transform) {
        self.mesh = Arc::new(self.mesh.transformed(t));
        if let Some(w) = self.working.as_mut() {
            w.mesh = Arc::new(w.mesh.transformed(t));
        }
    }
}

impl Document {
    /// Starts a working copy of mesh `id`, a copy of its current geometry. False when there is no
    /// such mesh or it has a working copy already.
    pub fn start_working(&mut self, id: ObjectId) -> bool {
        match self.mesh_mut(id) {
            Some(obj) if obj.working.is_none() => {
                obj.working = Some(WorkingCopy { mesh: obj.mesh.clone() });
                true
            }
            _ => false,
        }
    }

    /// Drops the working copy of mesh `id` and returns it; `None` when there is none.
    pub fn discard_working(&mut self, id: ObjectId) -> Option<WorkingCopy> {
        self.mesh_mut(id)?.working.take()
    }

    /// Replaces the original of mesh `id` with its working copy. Regions and quad layouts of the
    /// original are dropped when the triangles changed (their indices refer to the old mesh); the
    /// number of regions dropped is returned. `None` when there is no working copy.
    pub fn promote_working(&mut self, id: ObjectId) -> Option<usize> {
        let obj = self.mesh_mut(id)?;
        let copy = obj.working.take()?.mesh;
        if obj.mesh.triangles == copy.triangles {
            obj.mesh = copy;
            return Some(0);
        }
        self.replace_mesh_topology(id, Arc::unwrap_or_clone(copy))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use surfacecraft_geom::{Point3, Vec3, synth};

    fn flat(n: usize) -> Mesh {
        synth::plane(Point3::origin(), Vec3::x(), Vec3::y(), 10.0, 10.0, n, n)
    }

    #[test]
    fn start_is_refused_twice_and_discard_returns_the_copy() {
        let mut d = Document::new();
        let id = d.add_mesh("sheet", flat(4));
        assert!(d.start_working(id));
        assert!(!d.start_working(id), "a second copy is refused");
        assert!(!d.start_working(ObjectId(999)), "no such mesh");
        assert!(d.mesh(id).unwrap().is_working());
        let copy = d.discard_working(id).unwrap();
        assert_eq!(*copy.mesh, flat(4));
        assert!(d.discard_working(id).is_none());
    }

    #[test]
    fn promote_keeps_regions_when_the_triangles_are_the_same() {
        let mut d = Document::new();
        let id = d.add_mesh("sheet", flat(4));
        d.add_region("patch", id, vec![0, 1]);
        assert!(d.start_working(id));
        let lifted = flat(4).transformed(&Transform::translation(Vec3::new(0.0, 0.0, 1.0)));
        *d.mesh_mut(id).unwrap().editable_mut() = Arc::new(lifted);
        assert_eq!(d.promote_working(id), Some(0));
        assert_eq!(d.regions.len(), 1);
        assert!(!d.mesh(id).unwrap().is_working());
        assert!(d.promote_working(id).is_none());
    }

    #[test]
    fn promote_drops_regions_when_the_triangles_change() {
        let mut d = Document::new();
        let id = d.add_mesh("sheet", flat(4));
        d.add_region("patch", id, vec![0, 1]);
        assert!(d.start_working(id));
        *d.mesh_mut(id).unwrap().editable_mut() = Arc::new(flat(2));
        assert_eq!(d.promote_working(id), Some(1));
        assert!(d.regions.is_empty());
        assert_eq!(d.mesh(id).unwrap().mesh.triangle_count(), flat(2).triangle_count());
    }

    #[test]
    fn rigid_transform_moves_both_geometries() {
        let mut d = Document::new();
        let id = d.add_mesh("sheet", flat(2));
        assert!(d.start_working(id));
        let t = Transform::translation(Vec3::new(0.0, 0.0, 3.0));
        d.mesh_mut(id).unwrap().transform_rigid(&t);
        let obj = d.mesh(id).unwrap();
        assert!((obj.mesh.positions[0].z - 3.0).abs() < 1e-12);
        assert!((obj.working.as_ref().unwrap().mesh.positions[0].z - 3.0).abs() < 1e-12);
    }
}
