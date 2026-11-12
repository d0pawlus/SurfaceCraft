//! `export.*` / `import.*`: CAD exchange (STEP; IGES in M5).
use super::{CmdError, CmdResult, CommandSpec, ObjRef, read_file, write_file};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectKind, base64};
use surfacecraft_kernel::record::brep::Brep;

/// Export parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ExportIn {
    /// Surfaces and bodies to write (sheets and solids cannot be mixed in one file).
    pub objects: Vec<ObjRef>,
    /// Output path; omitted → the file comes back base64 in `data`.
    #[serde(default)]
    pub path: Option<String>,
    /// Product name in the file (default "surfacecraft").
    #[serde(default)]
    pub name: Option<String>,
}

/// Export result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ExportOut {
    /// Bodies written.
    pub bodies: usize,
    /// "sheets" or "solids".
    pub content: String,
    /// Bytes.
    pub bytes: usize,
    /// Path written.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// Base64 contents when no path was given.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub data: Option<String>,
}

fn collect(s: &Session, objects: &[ObjRef]) -> CmdResult<Vec<(String, Brep)>> {
    if objects.is_empty() {
        return Err(CmdError::Params("no objects to export".into()));
    }
    let mut out = Vec::new();
    for o in objects {
        let (id, kind) = o.resolve_any(s)?;
        let item = match kind {
            ObjectKind::Surface => s.doc.surface(id).map(|x| (x.name.clone(), (*x.sheet).clone())),
            ObjectKind::Body => s.doc.body(id).map(|x| (x.name.clone(), (*x.brep).clone())),
            _ => return Err(CmdError::Params(format!("{id} is not a surface or body"))),
        };
        out.push(item.ok_or_else(|| CmdError::Params(format!("no object {id}")))?);
    }
    Ok(out)
}

fn finish(s: &Session, text: String, path: Option<String>, bodies: usize, content: &str) -> CmdResult<ExportOut> {
    let bytes = text.len();
    match path {
        Some(p) => {
            write_file(s, &p, text.as_bytes())?;
            Ok(ExportOut { bodies, content: content.into(), bytes, path: Some(p), data: None })
        }
        None => Ok(ExportOut {
            bodies,
            content: content.into(),
            bytes,
            path: None,
            data: Some(base64::encode(text.as_bytes())),
        }),
    }
}

/// The STEP text of solid bodies: our compact writer (shared vertices, edge curves and control
/// points; a few KB per spline patch) for what it supports, the solidcore exporter (about 24 KB
/// per patch) for the rest. A record the compact writer rejects as invalid stays an error.
fn solids_text(name: &str, items: &[(String, Brep)]) -> CmdResult<String> {
    let bodies: Vec<&Brep> = items.iter().map(|(_, b)| b).collect();
    match surfacecraft_io::step::write_solids(name, &bodies) {
        Err(surfacecraft_io::IoError::Unsupported(_)) => {
            let refs: Vec<(&str, &Brep)> = items.iter().map(|(n, b)| (n.as_str(), b)).collect();
            Ok(surfacecraft_kernel::step::export_solids(&refs)?)
        }
        other => Ok(other?),
    }
}

fn export_step(s: &mut Session, i: ExportIn) -> CmdResult<ExportOut> {
    let items = collect(s, &i.objects)?;
    let name = i.name.unwrap_or_else(|| "surfacecraft".into());
    let sheets = items.iter().filter(|(_, b)| surfacecraft_io::step::is_sheet(b)).count();
    if sheets == items.len() {
        let refs: Vec<&Brep> = items.iter().map(|(_, b)| b).collect();
        let text = surfacecraft_io::step::write_sheets(&name, &refs)?;
        finish(s, text, i.path, items.len(), "sheets")
    } else if sheets == 0 {
        let text = solids_text(&name, &items)?;
        finish(s, text, i.path, items.len(), "solids")
    } else {
        Err(CmdError::Params("sheets and solids go to separate STEP files: export them in two calls".into()))
    }
}

/// Import parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ImportIn {
    /// File path.
    #[serde(default)]
    pub path: Option<String>,
    /// File contents, base64.
    #[serde(default)]
    pub data: Option<String>,
    /// Name of the bodies (default "imported").
    #[serde(default)]
    pub name: Option<String>,
}

/// Import result.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ImportOut {
    /// New body ids.
    pub bodies: Vec<ObjectId>,
    /// Faces per body.
    pub faces: Vec<usize>,
}

fn import_step(s: &mut Session, i: ImportIn) -> CmdResult<ImportOut> {
    let bytes = match (&i.path, &i.data) {
        (Some(p), None) => read_file(s, p)?,
        (None, Some(d)) => base64::decode(d).map_err(CmdError::Params)?,
        _ => return Err(CmdError::Params("give exactly one of `path` or `data`".into())),
    };
    let recs = surfacecraft_kernel::step::import(&bytes)?;
    if recs.is_empty() {
        return Err(CmdError::Failed("the file has no bodies".into()));
    }
    let name = i.name.unwrap_or_else(|| "imported".into());
    let mut bodies = Vec::new();
    let mut faces = Vec::new();
    for r in recs {
        faces.push(r.faces.len());
        bodies.push(s.doc.add_body(&name, r));
    }
    Ok(ImportOut { bodies, faces })
}

fn export_iges(s: &mut Session, i: ExportIn) -> CmdResult<ExportOut> {
    let items = collect(s, &i.objects)?;
    let name = i.name.unwrap_or_else(|| "surfacecraft".into());
    let refs: Vec<&Brep> = items.iter().map(|(_, b)| b).collect();
    let text = surfacecraft_io::iges::write_iges(&name, &refs)?;
    let sheets = items.iter().filter(|(_, b)| surfacecraft_io::step::is_sheet(b)).count();
    let content = if sheets == 0 {
        "solids"
    } else if sheets == items.len() {
        "sheets"
    } else {
        "mixed"
    };
    finish(s, text, i.path, items.len(), content)
}

/// Exchange commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "export.iges",
        "Export IGES…",
        "File",
        false,
        "Writes surfaces and bodies of `objects` as IGES 5.3 (186 solids, 514 shells) to `path` or base64 `data`.",
        ExportIn,
        ExportOut,
        export_iges
    ),
    command!(
        "export.step",
        "Export STEP…",
        "File",
        false,
        "Writes surfaces (open shells) or solid bodies of `objects` as STEP AP214 to `path` or base64 `data`.",
        ExportIn,
        ExportOut,
        export_step
    ),
    command!(
        "import.step",
        "Import STEP…",
        "File",
        true,
        "Reads every body of a STEP file (`path` or base64 `data`) as bodies.",
        ImportIn,
        ImportOut,
        import_step
    ),
];
