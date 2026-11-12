//! The mesh object: a scan or other triangle mesh, with its editable working copy.
use super::{Object, ObjectId, ObjectKind, yes};
use crate::working::WorkingCopy;
use schemars::JsonSchema;
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
    /// The unit the file was in when it was imported. The geometry is millimetres whatever it is.
    #[serde(default)]
    pub source_units: LengthUnit,
}

/// Length unit of a scan file.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "lowercase")]
pub enum LengthUnit {
    // Millimetres: the unit of the engine, and of every file unless stated otherwise.
    #[default]
    Mm,
    // Centimetres.
    Cm,
    // Metres.
    M,
    // Inches.
    In,
    // Feet.
    Ft,
    // Micrometres.
    Um,
}

impl LengthUnit {
    /// Millimetres in one of this unit (exact for the decimal and inch factors).
    #[must_use]
    pub fn mm_per_unit(self) -> f64 {
        match self {
            Self::Mm => 1.0,
            Self::Cm => 10.0,
            Self::M => 1000.0,
            Self::In => 25.4,
            Self::Ft => 304.8,
            Self::Um => 0.001,
        }
    }

    /// The unit's name as the commands spell it (`mm`, `cm`, `m`, `in`, `ft`, `um`).
    #[must_use]
    pub fn symbol(self) -> &'static str {
        match self {
            Self::Mm => "mm",
            Self::Cm => "cm",
            Self::M => "m",
            Self::In => "in",
            Self::Ft => "ft",
            Self::Um => "um",
        }
    }

    /// Is this millimetres (the unit of the geometry)?
    #[must_use]
    pub fn is_mm(self) -> bool {
        self == Self::Mm
    }
}

impl MeshObject {
    fn describe(&self) -> String {
        let copy = if self.working.is_some() { ", working copy" } else { "" };
        format!("{} triangles, {} vertices{copy}", self.mesh.triangle_count(), self.mesh.vertex_count())
    }
}
object_common!(MeshObject, ObjectKind::Mesh);
