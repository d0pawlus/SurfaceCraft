//! SurfaceCraft document model (layer L2).
//!
//! A [`Document`] holds every object of a project. Heavy payloads (meshes, B-rep records) sit
//! behind `Arc`, so a snapshot for undo is a cheap clone and only what an operation changes is
//! copied. [`History`] keeps those snapshots. The project file is JSON ([`format`]); meshes
//! inside it are base64 little-endian arrays so a large scan stays compact.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

pub mod base64;
pub mod feature;
pub mod format;
pub mod history;
pub mod object;
mod object_names;
pub mod param_use;
pub mod params;
pub mod working;

pub use feature::{BoolOp, FeatureObject, FeatureOp, Mode, Side, SketchObject};
pub use history::History;
pub use object::{
    BodyObject, Document, LengthUnit, MeshObject, Object, ObjectId, ObjectKind, ObjectSummary, PrimitiveObject,
    QuadObject, RegionObject, SurfaceObject,
};
pub use params::{Param, ParamError, ParamTable, ParamUser};
pub use working::WorkingCopy;
