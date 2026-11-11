//! The session: a document, its history and command execution.
use crate::EngineError;
use crate::cmd::{self, CmdError, CmdResult, CommandSpec};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::HashMap;
use std::sync::{Arc, Weak};
use surfacecraft_doc::{Document, History, ObjectId};
use surfacecraft_geom::Mesh;

/// One executed command, for the UI log and macro recording.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LogEntry {
    /// Command id.
    pub command: String,
    /// Parameters as given.
    pub params: Value,
    /// Success?
    pub ok: bool,
    /// Result or error message.
    pub outcome: Value,
}

/// One step of a script report.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScriptStep {
    /// Command id.
    pub command: String,
    /// Success?
    pub ok: bool,
    /// The command's output (on success).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    /// The error (on failure).
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<Value>,
}

/// What running a script did. Execution stops at the first failing step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScriptReport {
    /// Did every step succeed?
    pub ok: bool,
    /// Steps run, in order.
    pub steps: Vec<ScriptStep>,
}

/// A deviation map of one mesh: its colours and its signed deviation per vertex. The map is valid
/// only for the geometry it was computed on, so it is dropped once the object takes another mesh
/// (a move, an edit, a delete, or a new document).
#[derive(Clone, Debug)]
pub struct DeviationMap {
    /// The geometry the map was computed on. A weak handle: the map does not keep the mesh alive,
    /// and the allocation is not reused while the handle exists, so pointer identity is exact.
    geometry: Weak<Mesh>,
    /// Colour of each vertex (`colorize`), for the viewport.
    pub colours: Arc<Vec<[f32; 4]>>,
    /// Signed deviation of each vertex in mm (NaN where not counted), for `deviation.at`.
    pub values: Arc<Vec<f32>>,
}

/// A modelling session.
#[derive(Debug, Default)]
pub struct Session {
    /// The model.
    pub doc: Document,
    /// Undo/redo.
    pub history: History,
    /// Every command executed, oldest first (capped).
    pub log: Vec<LogEntry>,
    /// Whether commands may touch the file system (false on the web and in sandboxed MCP).
    pub allow_files: bool,
    /// Body display meshes.
    pub tess: crate::display::TessCache,
    /// Deviation maps by mesh id. Only valid ones are kept: see [`Session::live_map`].
    pub maps: HashMap<ObjectId, DeviationMap>,
    /// Moves on with every executed command (see [`Session::epoch`]).
    epoch: u64,
}

const LOG_CAP: usize = 1000;

/// Read-only commands a front end may run without a log entry, through [`Session::query`]: the
/// viewport readout asks on pointer moves, and logging those would evict the user's own commands.
/// Nothing else is run by `query`.
const QUERY_COMMANDS: &[&str] = &["deviation.at"];

impl Session {
    /// A new, empty session; file access allowed except on wasm.
    #[must_use]
    pub fn new() -> Self {
        Self { allow_files: !cfg!(target_arch = "wasm32"), ..Self::default() }
    }

    /// Every command, sorted by id.
    #[must_use]
    pub fn commands(&self) -> &'static [CommandSpec] {
        cmd::registry()
    }

    /// Executes command `id` with JSON `params` (`null` = no parameters).
    pub fn execute(&mut self, id: &str, params: Value) -> Result<Value, EngineError> {
        let spec = cmd::find(id).ok_or_else(|| EngineError::UnknownCommand(id.to_string()))?;
        let before = spec.undoable.then(|| self.doc.clone());
        let result = run_guarded(spec, self, params.clone());
        let mapped = match result {
            Ok(v) => {
                if let Some(b) = before {
                    self.history.push(spec.id, b);
                }
                Ok(v)
            }
            Err(e) => {
                if let Some(b) = before {
                    self.doc = b;
                }
                Err(e)
            }
        };
        // a command may have moved geometry: maps that no longer match their mesh go
        self.epoch = self.epoch.wrapping_add(1);
        self.prune_maps();
        let outcome = match &mapped {
            Ok(v) => v.clone(),
            Err(e) => serde_json::to_value(e).unwrap_or(Value::Null),
        };
        self.log.push(LogEntry { command: spec.id.to_string(), params, ok: mapped.is_ok(), outcome });
        if self.log.len() > LOG_CAP {
            self.log.remove(0);
        }
        mapped
    }

    /// Runs read-only command `id` on a shared session, without a log entry or a history step. Only
    /// the ids in `QUERY_COMMANDS` run here, and only if they are read commands with a read form,
    /// not undoable and not heavy; anything else is refused as unavailable. This is for the UI's
    /// per-frame reads and is not reachable from the MCP, the CLI or scripts.
    pub fn query(&self, id: &str, params: Value) -> Result<Value, EngineError> {
        if !QUERY_COMMANDS.contains(&id) {
            return Err(EngineError::Unavailable(format!("{id} is not a query")));
        }
        let spec = cmd::find(id).ok_or_else(|| EngineError::UnknownCommand(id.to_string()))?;
        let read = spec
            .read
            .filter(|_| !spec.undoable && !cmd::is_heavy(id))
            .ok_or_else(|| EngineError::Unavailable(format!("{id} has no read-only form")))?;
        run_read_guarded(spec, read, self, params)
    }

    /// Moves on with every executed command, so that a front end knows whether what it cached
    /// about the session (maps, selections of maps) may be out of date.
    #[must_use]
    pub fn epoch(&self) -> u64 {
        self.epoch
    }

    /// The deviation map of mesh `id` if it is still valid: the object still holds the geometry the
    /// map was computed on.
    #[must_use]
    pub fn live_map(&self, id: ObjectId) -> Option<&DeviationMap> {
        let map = self.maps.get(&id)?;
        let current = self.doc.mesh(id)?;
        std::ptr::eq(map.geometry.as_ptr(), Arc::as_ptr(&current.mesh)).then_some(map)
    }

    /// Stores the map of mesh `id` for the geometry the object holds now. False when there is no
    /// such mesh.
    pub fn insert_map(&mut self, id: ObjectId, colours: Vec<[f32; 4]>, values: Vec<f32>) -> bool {
        let Some(object) = self.doc.mesh(id) else { return false };
        let geometry = Arc::downgrade(&object.mesh);
        self.maps.insert(id, DeviationMap { geometry, colours: Arc::new(colours), values: Arc::new(values) });
        true
    }

    /// Removes the map of mesh `id`; true when there was one.
    pub fn drop_map(&mut self, id: ObjectId) -> bool {
        self.maps.remove(&id).is_some()
    }

    /// Drops every map whose geometry is no longer the object's.
    fn prune_maps(&mut self) {
        let stale: Vec<ObjectId> = self.maps.keys().copied().filter(|id| self.live_map(*id).is_none()).collect();
        for id in stale {
            self.maps.remove(&id);
        }
    }

    /// Runs command `id` on a copy of the document and returns the copy: the preview of what
    /// executing it would do. This session (document, history, log) is not touched; the copy
    /// has no file access and never opens a dialog. Commands that change nothing
    /// (`!undoable`) are refused, and so are [`cmd::is_heavy`] ones, which a front end must not
    /// run on its UI thread every time a parameter changes.
    pub fn preview(&self, id: &str, params: Value) -> Result<Session, EngineError> {
        let spec = cmd::find(id).ok_or_else(|| EngineError::UnknownCommand(id.to_string()))?;
        if !spec.undoable {
            return Err(EngineError::Unavailable(format!("{id} does not change the document")));
        }
        if cmd::is_heavy(id) {
            return Err(EngineError::Unavailable(format!("{id} is too heavy to preview")));
        }
        let mut copy = Session { doc: self.doc.clone(), allow_files: false, ..Session::default() };
        copy.execute(id, params)?;
        Ok(copy)
    }

    /// Runs `{"commands": [{"command": id, "params": {…}}, …]}`.
    pub fn run_script(&mut self, script: &Value) -> Result<ScriptReport, EngineError> {
        let list = script
            .get("commands")
            .and_then(Value::as_array)
            .ok_or_else(|| EngineError::InvalidParams("a script is {\"commands\": [...]}".into()))?;
        let mut steps = Vec::with_capacity(list.len());
        for (i, entry) in list.iter().enumerate() {
            let id = entry
                .get("command")
                .and_then(Value::as_str)
                .ok_or_else(|| EngineError::InvalidParams(format!("step {i}: missing \"command\"")))?;
            let params = entry.get("params").cloned().unwrap_or(Value::Null);
            match self.execute(id, params) {
                Ok(v) => steps.push(ScriptStep { command: id.into(), ok: true, result: Some(v), error: None }),
                Err(e) => {
                    steps.push(ScriptStep {
                        command: id.into(),
                        ok: false,
                        result: None,
                        error: Some(serde_json::to_value(&e).unwrap_or(Value::Null)),
                    });
                    return Ok(ScriptReport { ok: false, steps });
                }
            }
        }
        Ok(ScriptReport { ok: true, steps })
    }
}

fn map_err(e: CmdError) -> EngineError {
    match e {
        CmdError::Params(m) => EngineError::InvalidParams(m),
        CmdError::Failed(m) => EngineError::Failed(m),
        CmdError::Unavailable(m) => EngineError::Unavailable(m),
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn run_guarded(spec: &CommandSpec, s: &mut Session, params: Value) -> Result<Value, EngineError> {
    let run = spec.run;
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| run(s, params)));
    match r {
        Ok(v) => v.map_err(map_err),
        Err(_) => Err(EngineError::Internal(spec.id.to_string())),
    }
}

#[cfg(target_arch = "wasm32")]
fn run_guarded(spec: &CommandSpec, s: &mut Session, params: Value) -> Result<Value, EngineError> {
    (spec.run)(s, params).map_err(map_err)
}

#[cfg(not(target_arch = "wasm32"))]
fn run_read_guarded(
    spec: &CommandSpec,
    read: fn(&Session, Value) -> CmdResult<Value>,
    s: &Session,
    params: Value,
) -> Result<Value, EngineError> {
    let r = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| read(s, params)));
    match r {
        Ok(v) => v.map_err(map_err),
        Err(_) => Err(EngineError::Internal(spec.id.to_string())),
    }
}

#[cfg(target_arch = "wasm32")]
fn run_read_guarded(
    _spec: &CommandSpec,
    read: fn(&Session, Value) -> CmdResult<Value>,
    s: &Session,
    params: Value,
) -> Result<Value, EngineError> {
    read(s, params).map_err(map_err)
}
