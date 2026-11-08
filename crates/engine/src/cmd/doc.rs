//! `doc.*` and `edit.*`: the document as a whole, objects, undo/redo.
use super::{CmdError, CmdResult, CommandSpec, NoParams, ObjRef, read_file, write_file};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use surfacecraft_doc::{ObjectId, ObjectSummary, format};

/// Result listing objects.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ListOut {
    /// Every object.
    pub objects: Vec<ObjectSummary>,
}

fn list(s: &mut Session, _: NoParams) -> CmdResult<ListOut> {
    Ok(ListOut { objects: s.doc.summaries() })
}

fn new(s: &mut Session, _: NoParams) -> CmdResult<ListOut> {
    s.doc = surfacecraft_doc::Document::new();
    s.history.clear();
    Ok(ListOut { objects: Vec::new() })
}

/// A file path.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct PathIn {
    /// File path (`.scproj`).
    pub path: String,
}

/// Result of save.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct SaveOut {
    /// Path written.
    pub path: String,
    /// Bytes written.
    pub bytes: usize,
}

fn save(s: &mut Session, i: PathIn) -> CmdResult<SaveOut> {
    let text = format::to_json(&s.doc).map_err(CmdError::Failed)?;
    write_file(s, &i.path, text.as_bytes())?;
    Ok(SaveOut { path: i.path, bytes: text.len() })
}

fn open(s: &mut Session, i: PathIn) -> CmdResult<ListOut> {
    let bytes = read_file(s, &i.path)?;
    let text = String::from_utf8(bytes).map_err(|_| CmdError::Failed("project file is not UTF-8".into()))?;
    s.doc = format::from_json(&text).map_err(CmdError::Failed)?;
    s.history.clear();
    Ok(ListOut { objects: s.doc.summaries() })
}

/// One object.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ObjIn {
    /// Object id or name.
    pub object: ObjRef,
}

/// The id an operation touched.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct IdOut {
    /// Object id.
    pub id: ObjectId,
}

fn delete(s: &mut Session, i: ObjIn) -> CmdResult<IdOut> {
    let (id, _) = i.object.resolve_any(s)?;
    s.doc.remove(id);
    Ok(IdOut { id })
}

/// Rename parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RenameIn {
    /// Object id or name.
    pub object: ObjRef,
    /// New name (made unique).
    pub name: String,
}

fn rename(s: &mut Session, i: RenameIn) -> CmdResult<IdOut> {
    let (id, _) = i.object.resolve_any(s)?;
    s.doc.rename(id, &i.name);
    Ok(IdOut { id })
}

/// Visibility parameters.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ShowIn {
    /// Object id or name.
    pub object: ObjRef,
    /// Show (true) or hide (false).
    pub visible: bool,
}

fn show(s: &mut Session, i: ShowIn) -> CmdResult<IdOut> {
    let (id, _) = i.object.resolve_any(s)?;
    s.doc.set_visible(id, i.visible);
    Ok(IdOut { id })
}

/// Result of undo/redo.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct StepOut {
    /// The command undone or redone.
    pub command: String,
}

fn undo(s: &mut Session, _: NoParams) -> CmdResult<StepOut> {
    let cur = s.doc.clone();
    let (label, prev) = s.history.undo(cur).ok_or_else(|| CmdError::Unavailable("nothing to undo".into()))?;
    s.doc = prev;
    Ok(StepOut { command: label })
}

fn redo(s: &mut Session, _: NoParams) -> CmdResult<StepOut> {
    let cur = s.doc.clone();
    let (label, next) = s.history.redo(cur).ok_or_else(|| CmdError::Unavailable("nothing to redo".into()))?;
    s.doc = next;
    Ok(StepOut { command: label })
}

/// `doc.*` and `edit.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!("doc.new", "New", "File", false, "Starts an empty document (clears history).", NoParams, ListOut, new),
    command!("doc.open", "Open…", "File", false, "Opens a project file `path`.", PathIn, ListOut, open),
    command!("doc.save", "Save…", "File", false, "Saves the project to `path`.", PathIn, SaveOut, save),
    command!(
        "doc.list",
        "Objects",
        "File",
        false,
        "Lists every object with id, kind and name.",
        NoParams,
        ListOut,
        list
    ),
    command!("doc.delete", "Delete", "Edit", true, "Deletes `object`.", ObjIn, IdOut, delete),
    command!("doc.rename", "Rename", "Edit", true, "Renames `object` to `name`.", RenameIn, IdOut, rename),
    command!("doc.show", "Show/hide", "View", true, "Shows or hides `object` (`visible`).", ShowIn, IdOut, show),
    command!("edit.undo", "Undo", "Edit", false, "Undoes the last change.", NoParams, StepOut, undo),
    command!("edit.redo", "Redo", "Edit", false, "Redoes the last undone change.", NoParams, StepOut, redo),
];
