//! Units of a scan: the factor an import applies, and `mesh.set_units`, which states the unit of a
//! mesh that was imported (or sized) in the wrong one. Run it straight after the import: it scales
//! the mesh about the origin, so an alignment or a transform made before it is scaled with it.
//! Regions, primitives and quad layouts over the mesh are rescaled; bodies, sketches and the feature
//! history are not, and while there are any `mesh.set_units` refuses unless `force` is given.
use super::deps::{Dependents, Rescaled, scaled_mesh};
use crate::Session;
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use surfacecraft_doc::{Document, LengthUnit, MeshObject, ObjectId, ObjectKind};
use surfacecraft_geom::Vec3;

/// The single factor an import applies: the optional `scale` times the unit's millimetres. `None`
/// when neither is given, so that a plain import does exactly what it did before units existed.
pub(crate) fn import_scale(scale: Option<f64>, units: Option<LengthUnit>) -> Option<f64> {
    match units {
        None | Some(LengthUnit::Mm) => scale,
        Some(u) => Some(scale.unwrap_or(1.0) * u.mm_per_unit()),
    }
}

/// What an import says about its units, for the result: `None` when there is nothing to say.
#[must_use]
pub(crate) fn import_note(units: Option<LengthUnit>, factor: f64) -> Option<String> {
    let unit = units.unwrap_or_default();
    if unit.is_mm() && factor == 1.0 {
        return None;
    }
    Some(format!("the file is in {} and its coordinates are multiplied by {factor} to millimetres", unit.symbol()))
}

/// Objects a unit change does not rescale, counted over the whole document: bodies, sketches and
/// the feature history.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize, JsonSchema)]
pub struct LeftAlone {
    /// Bodies.
    pub bodies: usize,
    /// Sketches.
    pub sketches: usize,
    /// Features of the history.
    pub features: usize,
}

impl LeftAlone {
    /// Nothing is left alone.
    #[must_use]
    pub fn is_empty(self) -> bool {
        self.bodies + self.sketches + self.features == 0
    }
}

/// What a unit change would leave at the old size in `doc`.
#[must_use]
pub(crate) fn left_alone(doc: &Document) -> LeftAlone {
    LeftAlone { bodies: doc.bodies.len(), sketches: doc.sketches.len(), features: doc.features.len() }
}

/// `mesh.set_units` parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SetUnitsIn {
    /// Mesh id or name.
    pub object: ObjRef,
    /// The unit the coordinates really are in; the unit the mesh has now acknowledges it.
    /// A conversion is refused after mesh.scale_from_length.
    pub from: LengthUnit,
    /// Convert although bodies, sketches or features would stay at the old size.
    #[serde(default)]
    pub force: bool,
}

/// What `mesh.set_units` did.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SetUnitsOut {
    /// The mesh.
    pub id: ObjectId,
    /// The unit the coordinates really are in.
    pub from: LengthUnit,
    /// Factor every coordinate was multiplied by (1 when the unit was only acknowledged).
    pub factor: f64,
    /// Regions of the mesh rescaled with it.
    pub regions: usize,
    /// Primitives fitted to those regions, rescaled with it.
    pub primitives: usize,
    /// Quad layouts over the mesh, rescaled with it.
    pub quads: usize,
    /// What was NOT rescaled (it stays at the old size).
    pub left_alone: LeftAlone,
    /// In words, what was done (not after `force`).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub units_note: Option<String>,
}

fn too_big() -> CmdError {
    CmdError::Params("the converted mesh would not be finite".into())
}

/// Refuses what cannot be converted: a mesh sized from a length, and a document with objects a
/// unit change would leave behind (unless `force`). An acknowledgement (`factor` 1) changes nothing,
/// so nothing refuses it.
fn check_settable(obj: &MeshObject, factor: f64, left: LeftAlone, force: bool) -> CmdResult<()> {
    if factor == 1.0 {
        return Ok(());
    }
    if obj.rescaled {
        return Err(CmdError::Params(
            "this mesh was sized by mesh.scale_from_length: its unit cannot be set; undo that first".into(),
        ));
    }
    if !left.is_empty() && !force {
        let msg = format!(
            "bodies, sketches or features ({} bodies, {} sketches, {} features) would stay at the old size: pass \"force\": true to convert anyway",
            left.bodies, left.sketches, left.features
        );
        return Err(CmdError::Params(msg));
    }
    Ok(())
}

/// Scales the mesh `id`, its working copy and its dependents by `factor` about the origin. A fitted
/// surface refuses it before anything changes.
fn rescale_mesh(s: &mut Session, id: ObjectId, factor: f64) -> CmdResult<Rescaled> {
    let origin = Vec3::new(0.0, 0.0, 0.0);
    let obj = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
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
    Ok(done)
}

/// `mesh.set_units`: the coordinates of a mesh are in `from`; they become millimetres. The factor
/// is taken from the unit the mesh has now (`source_units`), so a wrong choice can be corrected
/// later. The same unit only acknowledges it: nothing is scaled.
pub(super) fn set_units(s: &mut Session, i: SetUnitsIn) -> CmdResult<SetUnitsOut> {
    let id = i.object.resolve(s, ObjectKind::Mesh)?;
    let obj = s.doc.mesh(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    let factor = i.from.mm_per_unit() / obj.source_units.mm_per_unit();
    check_settable(obj, factor, left_alone(&s.doc), i.force)?;
    let done = if factor == 1.0 { Rescaled::default() } else { rescale_mesh(s, id, factor)? };
    let obj = s.doc.mesh_mut(id).ok_or_else(|| CmdError::Params(format!("no mesh {id}")))?;
    obj.source_units = i.from;
    obj.units_stated = true;
    // after `force` the caller has overridden the left-alone check: no note about what was done
    let units_note = (!i.force).then(|| {
        if factor == 1.0 {
            format!("the unit is {} and stays: nothing was rescaled", i.from.symbol())
        } else {
            format!("the coordinates were in {} and are now millimetres (multiplied by {factor})", i.from.symbol())
        }
    });
    Ok(SetUnitsOut {
        id,
        from: i.from,
        factor,
        regions: done.regions,
        primitives: done.primitives,
        quads: done.quads,
        left_alone: left_alone(&s.doc),
        units_note,
    })
}
