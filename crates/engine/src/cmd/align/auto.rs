//! `align.auto`: a pose proposed from the scan alone (design doc §6.1, P9-M1).
use super::{AlignOut, apply_to, out};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_align::{AutoMethod, BaseFace, InPlane, LevelMode, auto_pose};
use surfacecraft_doc::ObjectKind;

/// Automatic alignment parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AutoIn {
    /// Mesh to move.
    pub object: ObjRef,
    /// How the base is levelled: `largest_face` (default) puts it exactly on z = 0, `all_faces` is a compromise over all near-axis-aligned faces by area.
    #[serde(default)]
    pub mode: LevelMode,
    /// Move the mesh (default false: the command only proposes the pose).
    #[serde(default)]
    pub apply: bool,
    /// Other meshes moved by the same motion.
    #[serde(default)]
    pub also: Vec<ObjRef>,
}

/// The proposed (or applied) pose and what it is based on.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct AutoOut {
    /// The motion.
    #[serde(flatten)]
    pub align: AlignOut,
    /// The levelling mode used.
    pub mode: LevelMode,
    /// What the pose is based on.
    pub method: AutoMethod,
    /// Share (0..1) of the surface area within 10 degrees of the frame's axes: 1 for a box,
    /// near 0 for a blob.
    pub confidence: f64,
    /// Is `confidence` at least 0.5? When false the proposal is a guess to check by eye.
    pub confident: bool,
    /// The face the part lies on (none for the principal-axes fallback).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base: Option<BaseFace>,
    /// Axis of a turned part, in the scan's frame.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub axis: Option<[f64; 3]>,
    /// How the rotation about z was found.
    pub in_plane: InPlane,
    /// A sentence about the outcome.
    pub note: String,
}

fn note_of(method: AutoMethod, confident: bool) -> &'static str {
    match (method, confident) {
        (AutoMethod::PrincipalFallback, _) => "no dominant flat faces: principal axes, low confidence",
        (_, false) => "few faces agree with the frame: check the proposal by eye",
        (AutoMethod::PlanarFace, true) => "laid on the largest flat face",
        (AutoMethod::OpenRim, true) => "laid on the open rim that stands for the missing face",
        (AutoMethod::Turned, true) => "turned part: axis on z, end face on the table",
    }
}

pub(super) fn auto(s: &mut Session, i: AutoIn) -> CmdResult<AutoOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let mesh = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let pose = auto_pose(&mesh, i.mode)?;
    apply_to(s, id, &i.also, &pose.transform, i.apply)?;
    let confident = pose.is_confident();
    Ok(AutoOut {
        align: out(id, &pose.transform, None, i.apply),
        mode: pose.mode,
        method: pose.method,
        confidence: pose.confidence,
        confident,
        base: pose.base,
        axis: pose.axis,
        in_plane: pose.in_plane,
        note: note_of(pose.method, confident).to_string(),
    })
}
