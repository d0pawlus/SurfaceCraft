//! `surface.*`: freeform NURBS surfaces fitted to regions (design doc §6.3, milestone M3).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind};

use super::surface_fit::fit_cmd;
pub use super::surface_fit::{FitIn, FitOut, GapOut, OutlineOut, PieceOut};

/// One surface.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SurfIn {
    /// Surface id or name.
    pub object: ObjRef,
}

/// Surface facts.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SurfOut {
    /// Id.
    pub id: ObjectId,
    /// Control net.
    pub control: [usize; 2],
    /// Degree.
    pub degree: usize,
    /// RMS (mm).
    pub rms: f64,
    /// Largest deviation (mm).
    pub max: f64,
    /// Faces / edges of the sheet.
    pub faces: usize,
    /// Edges of the sheet.
    pub edges: usize,
}

fn info(s: &mut Session, i: SurfIn) -> CmdResult<SurfOut> {
    let id = i.object.resolve(s, ObjectKind::Surface)?;
    let o = s.doc.surface(id).ok_or_else(|| CmdError::Params(format!("no surface {id}")))?;
    Ok(SurfOut {
        id,
        control: o.control,
        degree: o.degree,
        rms: o.rms,
        max: o.max,
        faces: o.sheet.faces.len(),
        edges: o.sheet.edges.len(),
    })
}

/// Thicken parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ThickenIn {
    /// Surface (or sheet body) to thicken.
    pub object: ObjRef,
    /// Wall thickness, mm (negative = against the normal).
    pub thickness: f64,
    /// Half the thickness on each side.
    #[serde(default)]
    pub symmetric: bool,
    /// Name of the solid.
    #[serde(default)]
    pub name: Option<String>,
}

/// The solid made.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ThickenOut {
    /// New body.
    pub id: ObjectId,
    /// Its volume, mm³.
    pub volume: Option<f64>,
    /// Faces.
    pub faces: usize,
}

fn thicken(s: &mut Session, i: ThickenIn) -> CmdResult<ThickenOut> {
    let (id, kind) = i.object.resolve_any(s)?;
    let sheet = match kind {
        ObjectKind::Surface => {
            let o = s.doc.surface(id).ok_or_else(|| CmdError::Params(format!("no surface {id}")))?;
            o.parent.clone().unwrap_or_else(|| o.sheet.clone())
        }
        ObjectKind::Body => {
            s.doc.body(id).map(|b| b.brep.clone()).ok_or_else(|| CmdError::Params(format!("no body {id}")))?
        }
        _ => return Err(CmdError::Params(format!("{id} is not a surface or sheet body"))),
    };
    let solid = surfacecraft_kernel::solid::thicken(&sheet, i.thickness, i.symmetric)?;
    let faces = solid.faces.len();
    let volume = surfacecraft_kernel::solid::volume(&solid).ok();
    let bid = s.doc.add_body(i.name.as_deref().unwrap_or("thickened"), solid);
    Ok(ThickenOut { id: bid, volume, faces })
}

/// `surface.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "surface.thicken",
        "Thicken",
        "Surface",
        true,
        "Thickens surface `object` by `thickness` (or `symmetric`ally) into a solid body.",
        ThickenIn,
        ThickenOut,
        thicken
    ),
    command!(
        "surface.fit",
        "Fit surface",
        "Surface",
        true,
        "Fits one NURBS patch to `region` (or `mesh`/`triangles`); the control net grows until max deviation ≤ `tolerance`.",
        FitIn,
        FitOut,
        fit_cmd
    ),
    command!(
        "surface.info",
        "Surface info",
        "Surface",
        false,
        "Control net, degree and deviation of `object`.",
        SurfIn,
        SurfOut,
        info
    ),
];
