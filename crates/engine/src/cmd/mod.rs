//! The command registry.
//!
//! A command is a [`CommandSpec`]: a dotted id (`area.verb`), a label and toolbar group for
//! the UI, a one-line summary, input and output JSON Schemas, and a `run` function. Commands
//! are declared with [`command!`](crate::command) from a typed input and output, so the schema
//! and the parser can never disagree.
use crate::Session;
use schemars::JsonSchema;
use serde::de::DeserializeOwned;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::sync::LazyLock;
use surfacecraft_doc::{ObjectId, ObjectKind};

pub mod align;
pub mod app;
pub mod body;
pub mod curves;
pub mod deviation;
pub mod doc;
pub mod exchange;
pub mod feature;
pub mod hybrid;
pub mod mesh;
pub mod plan;
pub mod primitive;
pub mod quad;
pub mod region;
pub mod sketch;
pub mod surface;

/// An error from a command's `run`.
#[derive(Clone, Debug, PartialEq)]
pub enum CmdError {
    /// Bad parameters.
    Params(String),
    /// The operation failed.
    Failed(String),
    /// Not available in the current state.
    Unavailable(String),
}

impl<E: std::fmt::Display> From<E> for CmdError {
    fn from(e: E) -> Self {
        CmdError::Failed(e.to_string())
    }
}

/// Shorthand.
pub type CmdResult<T> = Result<T, CmdError>;

/// A command.
#[derive(Clone, Copy)]
pub struct CommandSpec {
    /// `area.verb`.
    pub id: &'static str,
    /// Human label (toolbar, palette).
    pub label: &'static str,
    /// Toolbar group / menu.
    pub group: &'static str,
    /// One-line description of what it does and its parameters.
    pub summary: &'static str,
    /// Changes the document (snapshotted for undo, rolled back on failure).
    pub undoable: bool,
    /// JSON Schema (2020-12) of the parameters.
    pub input: fn() -> Value,
    /// JSON Schema of the result.
    pub output: fn() -> Value,
    /// Runs it.
    pub run: fn(&mut Session, Value) -> CmdResult<Value>,
}

impl std::fmt::Debug for CommandSpec {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("CommandSpec").field("id", &self.id).finish_non_exhaustive()
    }
}

/// Parses parameters; `null` stands for `{}`.
pub fn parse<T: DeserializeOwned>(v: Value) -> CmdResult<T> {
    let v = if v.is_null() { Value::Object(serde_json::Map::new()) } else { v };
    serde_json::from_value(v).map_err(|e| CmdError::Params(e.to_string()))
}

/// Serialises a result.
pub fn out<T: Serialize>(v: T) -> CmdResult<Value> {
    serde_json::to_value(v).map_err(|e| CmdError::Failed(e.to_string()))
}

/// The JSON Schema of `T`.
#[must_use]
pub fn schema<T: JsonSchema>() -> Value {
    serde_json::to_value(schemars::schema_for!(T)).unwrap_or(Value::Null)
}

/// Declares a [`CommandSpec`] from a typed handler `fn(&mut Session, In) -> CmdResult<Out>`.
#[macro_export]
macro_rules! command {
    ($id:literal, $label:literal, $group:literal, $undo:expr, $summary:literal, $in:ty, $out:ty, $f:path) => {
        $crate::cmd::CommandSpec {
            id: $id,
            label: $label,
            group: $group,
            summary: $summary,
            undoable: $undo,
            input: $crate::cmd::schema::<$in>,
            output: $crate::cmd::schema::<$out>,
            run: |s, v| {
                let input: $in = $crate::cmd::parse(v)?;
                let o: $out = $f(s, input)?;
                $crate::cmd::out(o)
            },
        }
    };
}

static REGISTRY: LazyLock<Vec<CommandSpec>> = LazyLock::new(|| {
    let mut v: Vec<CommandSpec> = Vec::new();
    v.extend_from_slice(align::COMMANDS);
    v.extend_from_slice(app::COMMANDS);
    v.extend_from_slice(body::COMMANDS);
    v.extend_from_slice(doc::COMMANDS);
    v.extend_from_slice(exchange::COMMANDS);
    v.extend_from_slice(feature::COMMANDS);
    v.extend_from_slice(curves::COMMANDS);
    v.extend_from_slice(hybrid::COMMANDS);
    v.extend_from_slice(deviation::COMMANDS);
    v.extend_from_slice(mesh::COMMANDS);
    v.extend_from_slice(primitive::COMMANDS);
    v.extend_from_slice(quad::COMMANDS);
    v.extend_from_slice(region::COMMANDS);
    v.extend_from_slice(sketch::COMMANDS);
    v.extend_from_slice(surface::COMMANDS);
    v.sort_by_key(|c| c.id);
    v
});

/// Every command, sorted by id.
#[must_use]
pub fn registry() -> &'static [CommandSpec] {
    &REGISTRY
}

/// The command with `id`.
#[must_use]
pub fn find(id: &str) -> Option<&'static CommandSpec> {
    registry().iter().find(|c| c.id == id)
}

/// Commands too slow to run on every parameter change (a front end skips their live preview).
const HEAVY: &[&str] = &[
    "align.icp",
    "body.from_regions",
    "body.plan_regions",
    "body.to_mesh",
    "deviation.compute",
    "deviation.noise",
    "export.iges",
    "export.step",
    "feature.replay",
    "import.step",
    "mesh.clean",
    "mesh.decimate",
    "mesh.export",
    "mesh.fill_holes",
    "mesh.import",
    "mesh.reconstruct",
    "mesh.remove_outliers",
    "mesh.smooth",
    "primitive.auto_constrain",
    "primitive.fit",
    "primitive.symmetry",
    "quad.retopo",
    "region.auto_segment",
    "region.grow",
    "sketch.fit_chains",
    "surface.auto",
    "surface.fit",
    "surface.thicken",
    "surface.trim_to_region",
];

/// Whether command `id` is too slow to preview live (see [`Session::preview`]).
#[must_use]
pub fn is_heavy(id: &str) -> bool {
    HEAVY.contains(&id)
}

/// No parameters.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct NoParams {}

/// A reference to an object: its numeric id or its name.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize, JsonSchema)]
#[serde(untagged)]
pub enum ObjRef {
    /// By id.
    Id(u64),
    /// By name.
    Name(String),
}

impl ObjRef {
    /// Resolves to an id of the expected kind.
    pub fn resolve(&self, s: &Session, kind: ObjectKind) -> CmdResult<ObjectId> {
        let found = match self {
            ObjRef::Id(i) => s.doc.kind_of(ObjectId(*i)).map(|k| (ObjectId(*i), k)),
            ObjRef::Name(n) => s.doc.find_by_name(n),
        };
        match found {
            Some((id, k)) if k == kind => Ok(id),
            Some((id, k)) => Err(CmdError::Params(format!("{id} is a {k:?}, not a {kind:?}"))),
            None => Err(CmdError::Params(format!("no object {self:?}"))),
        }
    }

    /// Resolves to an id of any kind.
    pub fn resolve_any(&self, s: &Session) -> CmdResult<(ObjectId, ObjectKind)> {
        match self {
            ObjRef::Id(i) => s.doc.kind_of(ObjectId(*i)).map(|k| (ObjectId(*i), k)),
            ObjRef::Name(n) => s.doc.find_by_name(n),
        }
        .ok_or_else(|| CmdError::Params(format!("no object {self:?}")))
    }
}

/// Reads a file, refusing when the session has no file access.
pub fn read_file(s: &Session, path: &str) -> CmdResult<Vec<u8>> {
    if !s.allow_files {
        return Err(CmdError::Unavailable("file paths are not available here; pass the data inline".into()));
    }
    std::fs::read(path).map_err(|e| CmdError::Failed(format!("cannot read {path}: {e}")))
}

/// Writes a file, refusing when the session has no file access.
pub fn write_file(s: &Session, path: &str, data: &[u8]) -> CmdResult<()> {
    if !s.allow_files {
        return Err(CmdError::Unavailable("file paths are not available here; ask for the data inline".into()));
    }
    std::fs::write(path, data).map_err(|e| CmdError::Failed(format!("cannot write {path}: {e}")))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_are_dotted_unique_and_schemas_build() {
        let reg = registry();
        assert!(!reg.is_empty());
        for w in reg.windows(2) {
            assert!(w[0].id < w[1].id, "duplicate or unsorted id {}", w[1].id);
        }
        for c in reg {
            let parts: Vec<&str> = c.id.split('.').collect();
            assert_eq!(parts.len(), 2, "{} is not area.verb", c.id);
            assert!(c.id.chars().all(|ch| ch.is_ascii_lowercase() || ch == '.' || ch == '_'), "{}", c.id);
            assert!((c.input)().is_object(), "{} input schema", c.id);
            assert!((c.output)().is_object(), "{} output schema", c.id);
            assert!(!c.summary.is_empty() && !c.label.is_empty());
        }
        for h in HEAVY {
            assert!(find(h).is_some(), "heavy list names unknown command {h}");
        }
    }
}
