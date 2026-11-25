//! `surface.sew`: separate sheets (patches of `surface.patch`, `surface.from_curves` with
//! `ngon: "patch"`, fitted surfaces, sheet bodies) sewn into one body along the edges that meet
//! within a tolerance; a solid where they close (P9-O8).
use super::{CmdError, CmdResult, CommandSpec, ObjRef};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use std::collections::HashSet;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, ObjectKind};
use surfacecraft_kernel::record::brep::Brep;

/// Sewing tolerance when none is given (mm): a tenth of the M3 surface bound.
const DEFAULT_TOLERANCE_MM: f64 = 0.02;

/// Most objects sewn in one call.
const MAX_OBJECTS: usize = 10_000;

/// Sew parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct SewIn {
    /// Surfaces and sheet bodies to sew into one body.
    pub objects: Vec<ObjRef>,
    /// Edges closer than this are joined, mm (default 0.02, at most 1).
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Name of the new body (default "sewn").
    #[serde(default)]
    pub name: Option<String>,
}

/// The sewn body.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SewOut {
    /// New body.
    pub id: ObjectId,
    /// The sheets close into a solid of positive volume.
    pub closed: bool,
    /// Edges still used by one face only (where the sheets did not meet within the tolerance).
    pub open_edges: usize,
    /// Faces of the body.
    pub faces: usize,
    /// Volume of the solid, mm³ (only when closed).
    pub volume: Option<f64>,
    /// Tolerance used, mm.
    pub tolerance: f64,
    /// Why sheets that close are not reported as a solid (no positive volume), if so.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

/// The sheet of each object, each object once; a solid, a mesh or any other kind is refused.
fn sheets(s: &Session, objects: &[ObjRef]) -> CmdResult<Vec<Arc<Brep>>> {
    if objects.is_empty() {
        return Err(CmdError::Params("no surfaces to sew".into()));
    }
    if objects.len() > MAX_OBJECTS {
        return Err(CmdError::Params(format!("{} objects to sew; at most {MAX_OBJECTS}", objects.len())));
    }
    let mut seen = HashSet::with_capacity(objects.len());
    let mut out = Vec::with_capacity(objects.len());
    for o in objects {
        let (id, kind) = o.resolve_any(s)?;
        if !seen.insert(id) {
            continue;
        }
        let sheet = match kind {
            ObjectKind::Surface => s.doc.surface(id).map(|x| x.sheet.clone()),
            ObjectKind::Body => match s.doc.body(id) {
                Some(b) if surfacecraft_io::step::is_sheet(&b.brep) => Some(b.brep.clone()),
                Some(_) => return Err(CmdError::Params(format!("{id} is a solid; only sheets are sewn"))),
                None => None,
            },
            _ => return Err(CmdError::Params(format!("{id} is not a surface or sheet body"))),
        };
        out.push(sheet.ok_or_else(|| CmdError::Params(format!("no object {id}")))?);
    }
    Ok(out)
}

fn sew_cmd(s: &mut Session, i: SewIn) -> CmdResult<SewOut> {
    let tolerance = i.tolerance.unwrap_or(DEFAULT_TOLERANCE_MM);
    let max = surfacecraft_kernel::sew::MAX_TOLERANCE_MM;
    if !(tolerance.is_finite() && tolerance > 0.0 && tolerance <= max) {
        return Err(CmdError::Params(format!("tolerance must be above 0 and at most {max} mm")));
    }
    let sheets = sheets(s, &i.objects)?;
    let refs: Vec<&Brep> = sheets.iter().map(|b| &**b).collect();
    let sewn = surfacecraft_kernel::sew::sew(&refs, tolerance)?;
    let faces = sewn.brep.faces.len();
    let id = s.doc.add_body(i.name.as_deref().unwrap_or("sewn"), sewn.brep);
    Ok(SewOut {
        id,
        closed: sewn.closed,
        open_edges: sewn.open_edges,
        faces,
        volume: sewn.volume,
        tolerance,
        note: sewn.note,
    })
}

/// The commands of this file.
pub static COMMANDS: &[CommandSpec] = &[command!(
    "surface.sew",
    "Sew surfaces",
    "Surface",
    true,
    "Sews surfaces and sheet bodies of `objects` into one body along edges closer than `tolerance`: a closed solid where they enclose a volume, else a sheet with its open edges counted.",
    SewIn,
    SewOut,
    sew_cmd
)];
