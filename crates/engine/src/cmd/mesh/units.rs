//! Units of a scan (P9-U1): the conversion of the factor an import applies, and `mesh.set_units`,
//! which converts a mesh already imported, with everything that depends on it.
use super::deps::{Dependents, scaled_mesh};
use super::size_check::{ScaleWarning, scale_warning};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{LengthUnit, ObjectId, ObjectKind};
use surfacecraft_geom::Vec3;

/// The single factor an import applies: the optional `scale` times the unit's millimetres. `None`
/// when neither is given, so that a plain import does exactly what it did before units existed.
pub(crate) fn import_scale(scale: Option<f64>, units: Option<LengthUnit>) -> Option<f64> {
    match units {
        None | Some(LengthUnit::Mm) => scale,
        Some(u) => Some(scale.unwrap_or(1.0) * u.mm_per_unit()),
    }
}

/// `mesh.set_units` parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SetUnitsIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// The unit the coordinates really are in (not millimetres).
    pub from: LengthUnit,
}

/// What `mesh.set_units` did.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SetUnitsOut {
    /// The mesh.
    pub id: ObjectId,
    /// The unit the coordinates were in.
    pub from: LengthUnit,
    /// Factor every coordinate was multiplied by (millimetres in one `from`).
    pub factor: f64,
    /// Regions of the mesh rescaled with it.
    pub regions: usize,
    /// Primitives fitted to those regions, rescaled with it.
    pub primitives: usize,
    /// Quad layouts over the mesh, rescaled with it.
    pub quads: usize,
    /// The size check after the conversion, when the size still looks wrong.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub scale_warning: Option<ScaleWarning>,
}

fn too_big() -> CmdError {
    CmdError::Params("the converted mesh would not be finite".into())
}

/// Refuses a mesh that is in millimetres already, or was converted before.
fn check_convertible(source: LengthUnit, from: LengthUnit) -> CmdResult<()> {
    if from.is_mm() {
        return Err(CmdError::Params(
            "the coordinates are millimetres already: give the unit they really are in, for example \"m\"".into(),
        ));
    }
    if !source.is_mm() {
        let msg =
            format!("this mesh was already converted from {}: undo that before converting again", source.symbol());
        return Err(CmdError::Params(msg));
    }
    Ok(())
}

/// `mesh.set_units`: the coordinates of a mesh are in `from`; they become millimetres about the
/// origin, and so do the regions, primitives and quad layouts over it. Fitted surfaces cannot be
/// rescaled, so their presence refuses the conversion.
pub(super) fn set_units(s: &mut Session, i: SetUnitsIn) -> CmdResult<SetUnitsOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let obj = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    check_convertible(obj.source_units, i.from)?;
    let factor = i.from.mm_per_unit();
    let origin = Vec3::new(0.0, 0.0, 0.0);
    let scaled = scaled_mesh(&obj.mesh, &origin, factor).ok_or_else(too_big)?;
    let working =
        obj.working.as_ref().map(|w| scaled_mesh(&w.mesh, &origin, factor).ok_or_else(too_big)).transpose()?;
    let deps = Dependents::of(&s.doc, id);
    deps.check(&s.doc)?;
    let done = deps.rescale(&mut s.doc, &origin, factor)?;
    let obj = s.doc.mesh_mut(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    obj.mesh = Arc::new(scaled);
    if let (Some(w), Some(m)) = (obj.working.as_mut(), working) {
        w.mesh = Arc::new(m);
    }
    obj.source_units = i.from;
    Ok(SetUnitsOut {
        id,
        from: i.from,
        factor,
        regions: done.regions,
        primitives: done.primitives,
        quads: done.quads,
        scale_warning: scale_warning(&obj.mesh),
    })
}
