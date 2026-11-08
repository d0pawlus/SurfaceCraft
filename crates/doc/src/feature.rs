//! Sketches and parametric features: the history (design doc §6.4). Features are kept in
//! creation order; each names its inputs (a sketch, bodies) and the body it writes. Replaying
//! the list in order (engine) rebuilds every feature-made body, so editing an early feature or a
//! sketch updates everything after it.
use crate::ObjectId;
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
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
    },
    /// Bevel edges of a body.
    Chamfer {
        /// Body.
        target: ObjectId,
        /// Edge indices.
        edges: Vec<usize>,
        /// Distance.
        distance: f64,
    },
    /// Hollow a body.
    Shell {
        /// Body.
        target: ObjectId,
        /// Faces removed (record indices).
        faces: Vec<usize>,
        /// Wall thickness (positive outward).
        thickness: f64,
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
            FeatureOp::Fillet { target, .. }
            | FeatureOp::Chamfer { target, .. }
            | FeatureOp::Shell { target, .. }
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
}
