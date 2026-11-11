//! The mesh object: a scan or other triangle mesh, with its editable working copy.
use super::{Object, ObjectId, ObjectKind, yes};
use crate::working::WorkingCopy;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_geom::Mesh;

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
    /// Editable copy (non-destructive editing); the original above stays as it was while it exists.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub working: Option<WorkingCopy>,
}

impl MeshObject {
    fn describe(&self) -> String {
        let copy = if self.working.is_some() { ", working copy" } else { "" };
        format!("{} triangles, {} vertices{copy}", self.mesh.triangle_count(), self.mesh.vertex_count())
    }
}
object_common!(MeshObject, ObjectKind::Mesh);
