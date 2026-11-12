//! `align.symmetry`: the part's mirror plane put on a world plane (design doc §6.2, P9-M1).
use super::{AlignOut, apply_to, out};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_align::{SymmetryPlane, WorldPlane, settle_symmetry};
use surfacecraft_doc::ObjectKind;

/// Symmetry settling parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SettleIn {
    /// Mesh to move.
    pub object: ObjRef,
    /// World plane the mirror plane is put on: `yz` (x = 0, default), `xz` or `xy`.
    #[serde(default)]
    pub plane: WorldPlane,
    /// Move the mesh (default false: the command only proposes the pose).
    #[serde(default)]
    pub apply: bool,
    /// Other meshes moved by the same motion.
    #[serde(default)]
    pub also: Vec<ObjRef>,
}

/// The proposed (or applied) pose and the mirror plane it is based on.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SettleOut {
    /// The motion.
    #[serde(flatten)]
    pub align: AlignOut,
    /// The world plane the mirror plane lands on.
    pub plane: WorldPlane,
    /// The mirror plane found, in the scan's frame (before the motion).
    pub symmetry_plane: SymmetryPlane,
    /// RMS distance (mm) between the mesh and its mirror image: near the scan noise for a
    /// symmetric part.
    pub residual_rms: f64,
}

pub(super) fn settle(s: &mut Session, i: SettleIn) -> CmdResult<SettleOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let mesh = s.doc.mesh(id).map(|m| m.mesh.clone()).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let settled = settle_symmetry(&mesh, i.plane)?;
    apply_to(s, id, &i.also, &settled.transform, i.apply)?;
    Ok(SettleOut {
        align: out(id, &settled.transform, None, i.apply),
        plane: i.plane,
        symmetry_plane: settled.plane,
        residual_rms: settled.residual_rms,
    })
}
