//! Sketches and parametric features: the history (design doc §6.4). Features are kept in
//! creation order; each names its inputs (a sketch, bodies) and the body it writes. Replaying
//! the list in order (engine) rebuilds every feature-made body, so editing an early feature or a
//! sketch updates everything after it.
use crate::ObjectId;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::sync::Arc;
use surfacecraft_kernel::naming::{EdgeSig, FaceSig};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_sketch::{Sketch, SolveReport};

/// How a sweep combines with an existing body.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Mode {
    /// A new body.
    #[default]
    New,
    /// United with the target body.
    Add,
    /// Subtracted from the target body.
    Cut,
}

/// Which side of a cutting plane a split keeps.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum Side {
    /// The side the plane normal points to.
    #[default]
    Front,
    /// The side the plane normal points away from.
    Back,
}

/// Boolean operations.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum BoolOp {
    /// Union.
    Union,
    /// Difference (target − tool).
    Subtract,
    /// Intersection.
    Intersect,
}

/// What a feature does. Body and sketch fields are object ids.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize, JsonSchema)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum FeatureOp {
    /// A body as it was when it entered the history (imported or made directly).
    Base {
        /// The body's geometry at that time.
        #[schemars(skip)]
        brep: Arc<Brep>,
    },
    /// Extrude the sketch's profiles along its normal (or `direction`).
    Extrude {
        /// Sketch.
        sketch: ObjectId,
        /// Distance (negative = opposite side).
        distance: f64,
        /// Draft angle, degrees.
        #[serde(default)]
        draft_deg: f64,
        /// New body, add to or cut from `target`.
        #[serde(default)]
        mode: Mode,
        /// Target body for add/cut.
        #[serde(default)]
        target: Option<ObjectId>,
    },
    /// Revolve the sketch's first profile about an axis.
    Revolve {
        /// Sketch.
        sketch: ObjectId,
        /// A point of the axis (model space).
        axis_origin: [f64; 3],
        /// Axis direction.
        axis_direction: [f64; 3],
        /// Angle, degrees.
        angle_deg: f64,
        /// New body, add or cut.
        #[serde(default)]
        mode: Mode,
        /// Target body for add/cut.
        #[serde(default)]
        target: Option<ObjectId>,
    },
    /// Round edges of a body.
    Fillet {
        /// Body.
        target: ObjectId,
        /// Edge indices (record order of the body as it is before this feature).
        edges: Vec<usize>,
        /// Radius.
        radius: f64,
        /// Geometric signatures of `edges` (persistent naming; filled in by the replay).
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        #[schemars(skip)]
        edge_refs: Vec<EdgeSig>,
    },
    /// Bevel edges of a body.
    Chamfer {
        /// Body.
        target: ObjectId,
        /// Edge indices.
        edges: Vec<usize>,
        /// Distance.
        distance: f64,
        /// Geometric signatures of `edges` (persistent naming; filled in by the replay).
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        #[schemars(skip)]
        edge_refs: Vec<EdgeSig>,
    },
    /// Hollow a body.
    Shell {
        /// Body.
        target: ObjectId,
        /// Faces removed (record indices).
        faces: Vec<usize>,
        /// Wall thickness (positive outward).
        thickness: f64,
        /// Geometric signatures of `faces` (persistent naming; filled in by the replay).
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        #[schemars(skip)]
        face_refs: Vec<FaceSig>,
    },
    /// Copies of a body, united.
    Pattern {
        /// Body.
        target: ObjectId,
        /// Number of instances (including the original).
        count: usize,
        /// Translation between instances.
        #[serde(default)]
        translate: [f64; 3],
        /// Rotation axis (through `rotate_origin`) between instances.
        #[serde(default)]
        rotate_axis: Option<[f64; 3]>,
        /// Point of the rotation axis.
        #[serde(default)]
        rotate_origin: [f64; 3],
        /// Rotation angle between instances, degrees.
        #[serde(default)]
        rotate_deg: f64,
    },
    /// A skin through the profiles of several sketches, capped into a solid.
    Loft {
        /// Section sketches in order (each contributes its outer closed profile).
        sketches: Vec<ObjectId>,
        /// Degree across the sections (1 = ruled); `None` = the highest the section count allows (≤ 3).
        #[serde(default)]
        degree: Option<u32>,
        /// New body, add to or cut from `target`.
        #[serde(default)]
        mode: Mode,
        /// Target body for add/cut.
        #[serde(default)]
        target: Option<ObjectId>,
    },
    /// The first closed profile of one sketch swept along the open path of another.
    Sweep {
        /// Sketch with the profile (placed at the start of the path).
        profile: ObjectId,
        /// Sketch with the open path (a chain of lines and arcs).
        path: ObjectId,
        /// The profile turns with the path (true) or only moves along it (false).
        #[serde(default = "crate::object::yes_fn")]
        follow: bool,
        /// New body, add to or cut from `target`.
        #[serde(default)]
        mode: Mode,
        /// Target body for add/cut.
        #[serde(default)]
        target: Option<ObjectId>,
    },
    /// Tilts faces of a body away from a pull direction about a neutral plane.
    Draft {
        /// Body.
        target: ObjectId,
        /// Faces drafted (record indices of the body before this feature).
        faces: Vec<usize>,
        /// Draft angle, degrees (positive leans the faces in going along `pull`).
        angle_deg: f64,
        /// A point of the neutral plane (it does not move).
        neutral_origin: [f64; 3],
        /// Normal of the neutral plane.
        neutral_normal: [f64; 3],
        /// Pull direction (default: the neutral normal).
        #[serde(default)]
        pull: Option<[f64; 3]>,
        /// Geometric signatures of `faces` (persistent naming; filled in by the replay).
        #[serde(default, skip_serializing_if = "Vec::is_empty")]
        #[schemars(skip)]
        face_refs: Vec<FaceSig>,
    },
    /// The body reflected in a plane, united with the original (or replacing it).
    Mirror {
        /// Body.
        target: ObjectId,
        /// A point of the mirror plane.
        plane_origin: [f64; 3],
        /// Normal of the mirror plane.
        plane_normal: [f64; 3],
        /// Unite the reflection with the original (true) or keep only the reflection.
        #[serde(default = "crate::object::yes_fn")]
        merge: bool,
    },
    /// Cuts the body by a plane and keeps one side.
    Split {
        /// Body.
        target: ObjectId,
        /// A point of the cutting plane.
        plane_origin: [f64; 3],
        /// Normal of the cutting plane.
        plane_normal: [f64; 3],
        /// The side kept.
        #[serde(default)]
        keep: Side,
    },
    /// Copies of a body spaced evenly along the open path of a sketch, united.
    PathPattern {
        /// Body.
        target: ObjectId,
        /// Sketch with the open path (a chain of lines and arcs).
        path: ObjectId,
        /// Instances including the original (at the start of the path).
        count: usize,
        /// Each copy turns with the path tangent.
        #[serde(default)]
        follow: bool,
    },
    /// Boolean of the target with a tool body (the tool is consumed: hidden).
    Boolean {
        /// Body changed.
        target: ObjectId,
        /// Tool body.
        tool: ObjectId,
        /// Operation.
        op: BoolOp,
    },
}

impl FeatureOp {
    /// Objects this feature reads (sketch and bodies), for the dependency graph.
    #[must_use]
    pub fn inputs(&self) -> Vec<ObjectId> {
        match self {
            FeatureOp::Base { .. } => Vec::new(),
            FeatureOp::Extrude { sketch, target, .. } | FeatureOp::Revolve { sketch, target, .. } => {
                std::iter::once(*sketch).chain(*target).collect()
            }
            FeatureOp::Loft { sketches, target, .. } => sketches.iter().copied().chain(*target).collect(),
            FeatureOp::Sweep { profile, path, target, .. } => [*profile, *path].into_iter().chain(*target).collect(),
            FeatureOp::PathPattern { target, path, .. } => vec![*target, *path],
            FeatureOp::Fillet { target, .. }
            | FeatureOp::Chamfer { target, .. }
            | FeatureOp::Shell { target, .. }
            | FeatureOp::Draft { target, .. }
            | FeatureOp::Mirror { target, .. }
            | FeatureOp::Split { target, .. }
            | FeatureOp::Pattern { target, .. } => vec![*target],
            FeatureOp::Boolean { target, tool, .. } => vec![*target, *tool],
        }
    }

    /// Short name of the operation.
    #[must_use]
    pub fn label(&self) -> &'static str {
        match self {
            FeatureOp::Base { .. } => "base",
            FeatureOp::Extrude { .. } => "extrude",
            FeatureOp::Revolve { .. } => "revolve",
            FeatureOp::Fillet { .. } => "fillet",
            FeatureOp::Chamfer { .. } => "chamfer",
            FeatureOp::Shell { .. } => "shell",
            FeatureOp::Pattern { .. } => "pattern",
            FeatureOp::Loft { .. } => "loft",
            FeatureOp::Sweep { .. } => "sweep",
            FeatureOp::Draft { .. } => "draft",
            FeatureOp::Mirror { .. } => "mirror",
            FeatureOp::Split { .. } => "split",
            FeatureOp::PathPattern { .. } => "path_pattern",
            FeatureOp::Boolean { .. } => "boolean",
        }
    }
}

/// A sketch object.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SketchObject {
    /// Id.
    pub id: ObjectId,
    /// Name.
    pub name: String,
    /// The sketch.
    pub sketch: Sketch,
    /// Result of the last solve.
    #[serde(default)]
    pub report: Option<SolveReport>,
    /// Why the sketch is unusable after a parameter change: a dimension expression that does not
    /// evaluate, or constraints that cannot be satisfied. Features reading it fail while it is set.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<String>,
    /// Shown?
    #[serde(default = "crate::object::yes_fn")]
    pub visible: bool,
}

/// A feature of the history.
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct FeatureObject {
    /// Id.
    pub id: ObjectId,
    /// Name.
    pub name: String,
    /// What it does.
    pub op: FeatureOp,
    /// Body it writes.
    pub body: ObjectId,
    /// Skipped in replay.
    #[serde(default)]
    pub suppressed: bool,
    /// Why the last replay of this feature failed.
    #[serde(default)]
    pub error: Option<String>,
    /// Shown in lists.
    #[serde(default = "crate::object::yes_fn")]
    pub visible: bool,
    /// Numeric fields of `op` that are given as an expression over parameters, by field name
    /// (`distance`, `translate.1` for a component of a vector): the text. The fields of `op`
    /// hold the values of the last evaluation.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub exprs: BTreeMap<String, String>,
}
