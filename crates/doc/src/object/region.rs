//! The region object: triangles of one mesh, optionally classified by a fitted primitive.
use super::ObjectId;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_fit::PrimitiveKind;

/// A persistent region: triangles of one mesh, optionally classified by a fitted primitive.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RegionObject {
    /// Id.
    pub id: ObjectId,
    /// Display name.
    pub name: String,
    /// The mesh the triangles belong to.
    pub mesh: ObjectId,
    /// Triangle indices, sorted and unique.
    pub triangles: Arc<Vec<u32>>,
    /// Geometry type after classification.
    #[serde(default)]
    pub kind: Option<PrimitiveKind>,
    /// RMS of the classification fit (mm).
    #[serde(default)]
    pub rms: Option<f64>,
    /// An organic area found by segmentation: no primitive describes it (`kind` is `None`).
    #[serde(default)]
    pub freeform: bool,
    /// Shown in the viewport (as a colour on its mesh).
    #[serde(default = "super::yes")]
    pub visible: bool,
}

impl RegionObject {
    pub(super) fn describe(&self) -> String {
        let k = if self.freeform {
            "freeform".to_string()
        } else {
            self.kind.map_or_else(|| "unclassified".to_string(), |k| format!("{k:?}").to_lowercase())
        };
        format!("{} triangles of {}, {k}", self.triangles.len(), self.mesh)
    }
}
