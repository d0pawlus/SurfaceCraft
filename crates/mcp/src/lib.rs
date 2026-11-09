//! SurfaceCraft MCP server (layer L5), spike §3.5 item 17: an own JSON-RPC 2.0 implementation
//! (one message per line), no `rmcp`. Every engine command is a tool (`mesh.import` →
//! `mesh_import`); in compact mode one tool per command area instead (`mesh` with `verb` and
//! `params`). Generic tools: `list_commands`, `describe_command`, `execute`, `batch` (with
//! rollback), `undo`, `redo`, `inspect_design`, `screenshot` (when the host can render),
//! `set_tool_mode` (the listing, at run time), and the queue: `job_status`, `job_wait`,
//! `job_cancel`. A call with `queued: true` returns a job id at once.
//! Resources: `surfacecraft://design` (the object list) and `surfacecraft://commands`.
//!
//! The server is transport-free: [`Server::handle`] maps one request value to one response
//! value (or none, for notifications). The CLI wires it to stdio, or forwards to a running
//! desktop app (bridged mode). Every call runs on one worker thread, in the order it was sent.
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use serde_json::{Value, json};
use std::collections::{BTreeMap, VecDeque};
use std::sync::{Arc, Condvar, Mutex, MutexGuard, PoisonError};
use std::time::{Duration, Instant};
use surfacecraft_engine::cmd::find;
use surfacecraft_engine::{CommandSpec, EngineError, Session, registry};

/// Protocol versions spoken, newest first.
pub const PROTOCOLS: &[&str] = &["2025-06-18", "2025-03-26", "2024-11-05"];

/// Most jobs open at once: queued, running, or finished and not yet read.
pub const MAX_OPEN_JOBS: usize = 64;

/// The wait of `job_wait` without `timeout_ms`, and its longest allowed wait (milliseconds).
const DEFAULT_WAIT_MS: u64 = 60_000;
const MAX_WAIT_MS: u64 = 600_000;

/// Environment variables choosing the tool listing at start (see [`ToolConfig::resolve`]).
pub const ENV_TOOLS: &str = "SURFACECRAFT_MCP_TOOLS";
/// See [`ENV_TOOLS`].
pub const ENV_AREAS: &str = "SURFACECRAFT_MCP_AREAS";

/// Tools that are not commands, listed in both modes.
const GENERAL: &[&str] =
    &["list_commands", "describe_command", "execute", "batch", "undo", "redo", "inspect_design", "screenshot"];
/// Tools the server answers itself: they act on the listing and the queue, so never queue.
const OWN: &[&str] = &["set_tool_mode", "job_status", "job_wait", "job_cancel"];

/// Something that executes commands: a local session, or a bridge to a running app.
pub trait Backend {
    /// Executes command `id`.
    fn execute(&mut self, id: &str, params: Value) -> Result<Value, Value>;
    /// A PNG screenshot of the viewport, if the backend can render.
    fn screenshot(&mut self, _width: u32, _height: u32) -> Option<Vec<u8>> {
        None
    }
}

/// A backend running commands in a local session.
pub struct LocalBackend {
    /// The session.
    pub session: Session,
    /// Optional renderer: session → PNG.
    pub render: Option<Box<dyn FnMut(&mut Session, u32, u32) -> Option<Vec<u8>> + Send>>,
}

impl LocalBackend {
    /// A fresh session; `allow_files` controls whether path parameters are accepted.
    #[must_use]
    pub fn new(allow_files: bool) -> Self {
        let mut session = Session::new();
        session.allow_files = allow_files && session.allow_files;
        Self { session, render: None }
    }
}

fn err_value(e: &EngineError) -> Value {
    serde_json::to_value(e).unwrap_or_else(|_| json!({"kind": "failed", "message": e.to_string()}))
}

impl Backend for LocalBackend {
    fn execute(&mut self, id: &str, params: Value) -> Result<Value, Value> {
        self.session.execute(id, params).map_err(|e| err_value(&e))
    }
    fn screenshot(&mut self, w: u32, h: u32) -> Option<Vec<u8>> {
        let r = self.render.as_mut()?;
        r(&mut self.session, w, h)
    }
}

/// Which tools `tools/list` shows: one tool per command (full), or one per command area
/// (compact). `areas` limits either to some areas. The general tools are always listed.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct ToolConfig {
    /// One tool per area (`verb` and `params`) instead of one per command.
    pub compact: bool,
    /// Only these areas (sorted; `None`: every area).
    pub areas: Option<Vec<String>>,
}

impl ToolConfig {
    /// Reads `mode` (`full` or `compact`) and `areas` (comma-separated). Absent values give the
    /// defaults; an unknown mode or area is an error that lists what is valid.
    pub fn parse(mode: Option<&str>, areas: Option<&str>) -> Result<Self, String> {
        let compact = match mode.map(str::trim) {
            None | Some("" | "full") => false,
            Some("compact") => true,
            Some(other) => return Err(format!("unknown tool mode `{other}`: use full or compact")),
        };
        let names: Vec<String> =
            areas.unwrap_or("").split(',').map(str::trim).filter(|a| !a.is_empty()).map(str::to_string).collect();
        Ok(Self { compact, areas: check_areas(names)? })
    }

    /// The configuration from the arguments, else from [`ENV_TOOLS`] and [`ENV_AREAS`].
    pub fn resolve(mode: Option<&str>, areas: Option<&str>) -> Result<Self, String> {
        let env_mode = std::env::var(ENV_TOOLS).ok();
        let env_areas = std::env::var(ENV_AREAS).ok();
        Self::parse(mode.or(env_mode.as_deref()), areas.or(env_areas.as_deref()))
    }
}

/// The listed areas, checked against the registry (an empty list means every area).
fn check_areas(names: Vec<String>) -> Result<Option<Vec<String>>, String> {
    let known = areas();
    let mut out: Vec<String> = Vec::new();
    for n in names {
        if !known.contains(&n.as_str()) {
            return Err(format!("unknown area `{n}`; areas: {}", known.join(", ")));
        }
        if !out.contains(&n) {
            out.push(n);
        }
    }
    out.sort();
    Ok((!out.is_empty()).then_some(out))
}

/// Every command area (the first part of the command ids), sorted.
#[must_use]
pub fn areas() -> Vec<&'static str> {
    let mut v: Vec<&'static str> = registry().iter().map(|c| area_of(c.id)).collect();
    v.sort_unstable();
    v.dedup();
    v
}

fn area_of(id: &'static str) -> &'static str {
    id.split('.').next().unwrap_or(id)
}

/// The verb of a command id (the part after the area).
fn verb_of(id: &'static str) -> &'static str {
    id.split_once('.').map_or(id, |(_, verb)| verb)
}

/// The commands of one area, in id order.
fn area_commands(area: &str) -> Vec<&'static CommandSpec> {
    registry().iter().filter(|c| area_of(c.id) == area).collect()
}

/// Tool name of a command id.
#[must_use]
pub fn tool_name(id: &str) -> String {
    id.replace('.', "_")
}

/// Command id of a tool name (the first `_` is the area separator).
#[must_use]
pub fn command_id(tool: &str) -> Option<&'static str> {
    registry().iter().map(|c| c.id).find(|id| tool_name(id) == tool)
}

/// The area of a compact-mode tool (its name is the area).
fn area_tool(name: &str) -> Option<&'static str> {
    areas().into_iter().find(|a| *a == name)
}

/// Whether `name` is a tool the server knows, in either mode.
fn known_tool(name: &str) -> bool {
    GENERAL.contains(&name) || OWN.contains(&name) || command_id(name).is_some() || area_tool(name).is_some()
}

/// Where a job is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Queued,
    Running,
    Done,
    Failed,
    Cancelled,
}

impl State {
    fn name(self) -> &'static str {
        match self {
            Self::Queued => "queued",
            Self::Running => "running",
            Self::Done => "done",
            Self::Failed => "failed",
            Self::Cancelled => "cancelled",
        }
    }

    fn finished(self) -> bool {
        matches!(self, Self::Done | Self::Failed | Self::Cancelled)
    }
}

/// What a job runs: a tool of the listing, or one command (for the resources).
#[derive(Clone, Debug)]
enum Request {
    Tool { name: String, args: Value },
    Command { id: String, params: Value },
}

impl Request {
    fn label(&self) -> String {
        match self {
            Self::Tool { name, .. } => name.clone(),
            Self::Command { id, .. } => id.clone(),
        }
    }
}

/// One job: its request until it starts, then its state and, once finished, its tool result.
struct Job {
    label: String,
    request: Option<Request>,
    state: State,
    result: Value,
}

/// The job table: a FIFO of job ids and the jobs by id.
#[derive(Default)]
struct Table {
    next: u64,
    queue: VecDeque<u64>,
    jobs: BTreeMap<u64, Job>,
    closed: bool,
}

/// The table shared by the server and its worker.
#[derive(Default)]
struct Jobs {
    table: Mutex<Table>,
    changed: Condvar,
}

/// How long a read waits for a job to finish.
#[derive(Clone, Copy, Debug)]
enum Wait {
    /// Not at all: report the state now.
    Poll,
    /// Up to this long.
    For(Duration),
    /// Until finished.
    Forever,
}

/// A job as read: its state and, once finished, its tool result.
struct View {
    job: u64,
    label: String,
    state: State,
    result: Value,
}

impl Jobs {
    fn lock(&self) -> MutexGuard<'_, Table> {
        self.table.lock().unwrap_or_else(PoisonError::into_inner)
    }

    /// Queues `request`. With `capped`, a call is refused while `MAX_OPEN_JOBS` jobs are open.
    fn submit(&self, request: Request, capped: bool) -> Result<u64, String> {
        let mut t = self.lock();
        if capped && t.jobs.len() >= MAX_OPEN_JOBS {
            return Err(format!("{MAX_OPEN_JOBS} jobs are open; read finished ones with job_wait or job_status first"));
        }
        t.next = t.next.saturating_add(1);
        let id = t.next;
        let label = request.label();
        t.jobs.insert(id, Job { label, request: Some(request), state: State::Queued, result: Value::Null });
        t.queue.push_back(id);
        self.changed.notify_all();
        Ok(id)
    }

    /// The next job to run, its request taken. With `wait`, blocks until there is one; `None`
    /// when there is none (or the server is gone).
    fn next(&self, wait: bool) -> Option<(u64, Request)> {
        let mut t = self.lock();
        loop {
            while let Some(id) = t.queue.pop_front() {
                let Some(job) = t.jobs.get_mut(&id) else { continue };
                if job.state != State::Queued {
                    continue; // cancelled before it started
                }
                let Some(request) = job.request.take() else { continue };
                job.state = State::Running;
                return Some((id, request));
            }
            if t.closed || !wait {
                return None;
            }
            t = self.changed.wait(t).unwrap_or_else(PoisonError::into_inner);
        }
    }

    /// Records the result of a running job and wakes the readers.
    fn finish(&self, id: u64, result: Value) {
        let mut t = self.lock();
        if let Some(job) = t.jobs.get_mut(&id) {
            let failed = result.get("isError").and_then(Value::as_bool).unwrap_or(false);
            job.state = if failed { State::Failed } else { State::Done };
            job.result = result;
        }
        self.changed.notify_all();
    }

    /// Cancels a job that has not started yet. Returns whether it was cancelled.
    fn cancel(&self, id: u64) -> bool {
        let mut t = self.lock();
        let cancelled = match t.jobs.get_mut(&id) {
            Some(job) if job.state == State::Queued => {
                job.state = State::Cancelled;
                job.request = None;
                true
            }
            _ => false,
        };
        self.changed.notify_all();
        cancelled
    }

    /// The job's view, waiting as `wait` says. A finished job is taken away: its result is
    /// read once. `None` for an unknown job (never queued, or already read).
    fn read(&self, id: u64, wait: Wait) -> Option<View> {
        let end = match wait {
            Wait::For(d) => Instant::now().checked_add(d),
            _ => None,
        };
        let mut t = self.lock();
        loop {
            let state = t.jobs.get(&id)?.state;
            if state.finished() {
                break;
            }
            match wait {
                Wait::Poll => break,
                Wait::Forever => t = self.changed.wait(t).unwrap_or_else(PoisonError::into_inner),
                Wait::For(_) => match end {
                    Some(end) if Instant::now() < end => {
                        t = match self.changed.wait_timeout(t, end.saturating_duration_since(Instant::now())) {
                            Ok((guard, _)) => guard,
                            Err(e) => e.into_inner().0,
                        };
                    }
                    _ => break,
                },
            }
        }
        let (state, label) = {
            let job = t.jobs.get(&id)?;
            (job.state, job.label.clone())
        };
        let result =
            if state.finished() { t.jobs.remove(&id).map_or(Value::Null, |job| job.result) } else { Value::Null };
        Some(View { job: id, label, state, result })
    }
}

/// The worker: runs queued jobs one at a time, in order, until the server is dropped.
fn work<B: Backend>(backend: Arc<Mutex<B>>, jobs: Arc<Jobs>) {
    while let Some((id, request)) = jobs.next(true) {
        let result = run_request(&*backend, request);
        jobs.finish(id, result);
    }
}

/// Runs one request on the backend. A panic becomes an error result: the worker survives it.
fn run_request<B: Backend>(backend: &Mutex<B>, request: Request) -> Value {
    let outcome = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        let mut b = backend.lock().unwrap_or_else(PoisonError::into_inner);
        match request {
            Request::Tool { name, args } => dispatch_tool(&mut *b, &name, args),
            Request::Command { id, params } => run_command(&mut *b, &id, params),
        }
    }));
    outcome.unwrap_or_else(|_| tool_error("internal error: a command panicked"))
}

fn err_text(v: &Value) -> String {
    serde_json::to_string_pretty(v).unwrap_or_else(|_| v.to_string())
}

fn rpc_error(id: &Value, code: i64, message: &str) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message}})
}

fn rpc_ok(id: &Value, result: Value) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "result": result})
}

fn text_result(v: &Value, is_error: bool) -> Value {
    json!({"content": [{"type": "text", "text": err_text(v)}], "structuredContent": v, "isError": is_error})
}

/// A tool result reporting `message` as an error.
fn tool_error(message: &str) -> Value {
    text_result(&json!({"error": message}), true)
}

/// Base64 for PNG payloads.
fn b64(data: &[u8]) -> String {
    const A: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let n = (u32::from(c.first().copied().unwrap_or(0)) << 16)
            | (u32::from(c.get(1).copied().unwrap_or(0)) << 8)
            | u32::from(c.get(2).copied().unwrap_or(0));
        for k in 0..4 {
            if k <= c.len() {
                out.push(char::from(A.get(((n >> (18 - 6 * k)) & 63) as usize).copied().unwrap_or(b'A')));
            } else {
                out.push('=');
            }
        }
    }
    out
}

/// Runs a command by id. A batch with a failed step is an error result that still carries each
/// step's outcome.
fn run_command<B: Backend>(backend: &mut B, id: &str, params: Value) -> Value {
    match backend.execute(id, params) {
        Ok(v) => {
            let failed_batch = id == "app.batch" && v.get("ok").and_then(Value::as_bool) == Some(false);
            text_result(&v, failed_batch)
        }
        Err(e) => text_result(&e, true),
    }
}

/// Runs `commands` in order; on the first failure, every step that changed the document is undone.
fn batch<B: Backend>(backend: &mut B, steps: &[Value]) -> Value {
    let mut done = 0usize;
    let mut results = Vec::new();
    for (k, st) in steps.iter().enumerate() {
        let Some(cmd) = st.get("command").and_then(Value::as_str) else {
            return text_result(&json!({"error": format!("step {k}: missing command")}), true);
        };
        let params = st.get("params").cloned().unwrap_or(Value::Null);
        let undoable = find(cmd).is_some_and(|c| c.undoable);
        match backend.execute(cmd, params) {
            Ok(v) => {
                if undoable {
                    done += 1;
                }
                results.push(v);
            }
            Err(e) => {
                for _ in 0..done {
                    let _ = backend.execute("edit.undo", Value::Null);
                }
                return text_result(&json!({"failed_step": k, "error": e, "rolled_back": done}), true);
            }
        }
    }
    text_result(&json!({"results": results}), false)
}

fn screenshot<B: Backend>(backend: &mut B, args: &Value) -> Value {
    let dim = |k: &str, d: u32| args.get(k).and_then(Value::as_u64).map_or(d, |v| v.clamp(16, 4096) as u32);
    match backend.screenshot(dim("width", 800), dim("height", 600)) {
        Some(png) => {
            json!({"content": [{"type": "image", "data": b64(&png), "mimeType": "image/png"}], "isError": false})
        }
        None => tool_error("this host cannot render screenshots"),
    }
}

/// The verbs of an area, as the compact tool's `verb` takes them.
fn verbs(area: &str) -> Vec<&'static str> {
    area_commands(area).iter().map(|c| verb_of(c.id)).collect()
}

/// A compact-mode call: `verb` picks a command of `area`, `params` are its parameters.
fn area_command<B: Backend>(backend: &mut B, area: &str, args: &Value) -> Value {
    let Some(verb) = args.get("verb").and_then(Value::as_str) else {
        return tool_error(&format!("`{area}` needs `verb`; verbs: {}", verbs(area).join(", ")));
    };
    let id = format!("{area}.{verb}");
    if find(&id).is_none() {
        return tool_error(&format!("`{area}` has no verb `{verb}`; verbs: {}", verbs(area).join(", ")));
    }
    run_command(backend, &id, args.get("params").cloned().unwrap_or(Value::Null))
}

/// Runs a tool of the listing: a general tool, a command tool, or an area tool (compact mode).
fn dispatch_tool<B: Backend>(backend: &mut B, name: &str, args: Value) -> Value {
    let arg = |k: &str| args.get(k).cloned().unwrap_or(Value::Null);
    match name {
        "list_commands" => run_command(backend, "app.commands", json!({"filter": arg("filter")})),
        "describe_command" => run_command(backend, "app.describe", json!({"id": arg("id")})),
        "execute" => match args.get("command").and_then(Value::as_str) {
            Some(cmd) => run_command(backend, cmd, arg("params")),
            None => tool_error("missing `command`"),
        },
        "batch" => {
            let steps = args.get("commands").and_then(Value::as_array).cloned().unwrap_or_default();
            batch(backend, &steps)
        }
        "undo" => run_command(backend, "edit.undo", Value::Null),
        "redo" => run_command(backend, "edit.redo", Value::Null),
        "inspect_design" => run_command(backend, "doc.list", Value::Null),
        "screenshot" => screenshot(backend, &args),
        other => {
            if let Some(id) = command_id(other) {
                return run_command(backend, id, args);
            }
            if let Some(area) = area_tool(other) {
                return area_command(backend, area, &args);
            }
            tool_error(&format!("unknown tool `{other}`"))
        }
    }
}

/// The input schema with the `queued` flag added (the server takes it off before the call runs).
fn queueable(mut schema: Value) -> Value {
    if let Some(obj) = schema.as_object_mut() {
        let props = obj.entry("properties").or_insert_with(|| json!({}));
        if let Some(props) = props.as_object_mut() {
            props.insert(
                "queued".into(),
                json!({"type": "boolean", "description": "true: return a job id at once and run the call on the worker (see job_status, job_wait, job_cancel)."}),
            );
        }
    }
    schema
}

/// A general tool's listing entry.
fn general(name: &str, description: &str, schema: Value) -> Value {
    let schema = if OWN.contains(&name) { schema } else { queueable(schema) };
    json!({"name": name, "description": description, "inputSchema": schema})
}

/// The general tools: the MCP-level ones, listed in both modes.
fn general_tools() -> Vec<Value> {
    let obj = |props: Value, req: &[&str]| json!({"type": "object", "properties": props, "required": req});
    let job = || json!({"type": "integer", "minimum": 1, "description": "The job id a queued call returned."});
    vec![
        general(
            "list_commands",
            "List SurfaceCraft commands (optional `filter`). Start with guide_read for the task guides.",
            obj(json!({"filter": {"type": "string"}}), &[]),
        ),
        general(
            "describe_command",
            "Input and output JSON Schemas of command `id`.",
            obj(json!({"id": {"type": "string"}}), &["id"]),
        ),
        general(
            "execute",
            "Execute command `command` with `params`.",
            obj(json!({"command": {"type": "string"}, "params": {"type": "object"}}), &["command"]),
        ),
        general(
            "batch",
            "Execute `commands` ([{command, params}]) in order; on the first failure every step that changed the document is undone. For steps that keep their effects, use app_batch.",
            obj(json!({"commands": {"type": "array"}}), &["commands"]),
        ),
        general("undo", "Undo the last change.", obj(json!({}), &[])),
        general("redo", "Redo.", obj(json!({}), &[])),
        general("inspect_design", "Every object of the design with id, kind, name and facts.", obj(json!({}), &[])),
        general(
            "screenshot",
            "PNG of the viewport (when the host can render).",
            obj(json!({"width": {"type": "integer"}, "height": {"type": "integer"}}), &[]),
        ),
        general(
            "set_tool_mode",
            "Choose the tool listing from now on: `mode` full (one tool per command) or compact (one tool per area, with verb and params); `areas` limits either to some areas (null or [] for all). Keeps what is not given. Returns the new listing.",
            obj(
                json!({
                    "mode": {"type": "string", "enum": ["full", "compact"]},
                    "areas": {"type": ["array", "null"], "items": {"type": "string"}}
                }),
                &[],
            ),
        ),
        general(
            "job_status",
            "State of a queued job: queued, running, done, failed or cancelled. A finished job's result is returned once, then the job is gone.",
            obj(json!({"job": job()}), &["job"]),
        ),
        general(
            "job_wait",
            "Waits up to `timeout_ms` (default 60000, at most 600000) for a queued job to finish, then returns its state and result, as job_status does. A still running job is reported as running.",
            obj(json!({"job": job(), "timeout_ms": {"type": "integer", "minimum": 0}}), &["job"]),
        ),
        general(
            "job_cancel",
            "Cancels a queued job before it starts (best effort: a running or finished job is reported as it is).",
            obj(json!({"job": job()}), &["job"]),
        ),
    ]
}

/// One command's listing entry (full mode).
fn command_tool(c: &CommandSpec) -> Value {
    json!({
        "name": tool_name(c.id),
        "title": c.label,
        "description": format!("{} ({})", c.summary, c.id),
        "inputSchema": queueable((c.input)()),
        "outputSchema": (c.output)(),
        "annotations": {"readOnlyHint": !c.undoable},
    })
}

/// One area's listing entry (compact mode): `verb` and `params`.
fn area_tool_spec(area: &str) -> Value {
    let commands = area_commands(area);
    let lines: Vec<String> = commands.iter().map(|c| format!("- {}: {}", verb_of(c.id), c.summary)).collect();
    let verb_names = verbs(area);
    json!({
        "name": area,
        "title": format!("{area} commands"),
        "description": format!(
            "Commands of the `{area}` area. `verb` picks one, `params` are its parameters (describe_command gives the schema).\n{}",
            lines.join("\n")
        ),
        "inputSchema": queueable(json!({
            "type": "object",
            "properties": {
                "verb": {"type": "string", "enum": verb_names},
                "params": {"type": "object", "description": "Parameters of the command (see describe_command)."}
            },
            "required": ["verb"],
            "additionalProperties": false
        })),
    })
}

/// The MCP server state.
pub struct Server<B: Backend> {
    backend: Arc<Mutex<B>>,
    jobs: Arc<Jobs>,
    threaded: bool,
    config: ToolConfig,
    initialized: bool,
}

impl<B: Backend> Drop for Server<B> {
    fn drop(&mut self) {
        self.jobs.lock().closed = true;
        self.jobs.changed.notify_all();
    }
}

impl<B: Backend + Send + 'static> Server<B> {
    /// A server over `backend`, listing every command. Calls run on one worker thread, in the
    /// order they were sent; where no thread can be started, the calling thread runs them.
    pub fn new(backend: B) -> Self {
        let backend = Arc::new(Mutex::new(backend));
        let jobs = Arc::new(Jobs::default());
        let (worker_backend, worker_jobs) = (Arc::clone(&backend), Arc::clone(&jobs));
        let threaded = std::thread::Builder::new()
            .name("surfacecraft-mcp".into())
            .spawn(move || work(worker_backend, worker_jobs))
            .is_ok();
        Self { backend, jobs, threaded, config: ToolConfig::default(), initialized: false }
    }
}

impl<B: Backend> Server<B> {
    /// The same server, listing `config`.
    #[must_use]
    pub fn with_tools(mut self, config: ToolConfig) -> Self {
        self.config = config;
        self
    }

    /// Queues a call and waits for its result (calls queued before it run first).
    fn call_sync(&mut self, request: Request) -> Value {
        match self.jobs.submit(request, false) {
            Ok(id) => match self.await_job(id, Wait::Forever) {
                Some(view) => view.result,
                None => tool_error("the call was lost"),
            },
            Err(message) => tool_error(&message),
        }
    }

    /// Waits for job `id` and takes it away. Without a worker thread, this thread first runs the
    /// queue, in order.
    fn await_job(&mut self, id: u64, wait: Wait) -> Option<View> {
        let wait = if self.threaded {
            wait
        } else {
            while let Some((job, request)) = self.jobs.next(false) {
                let result = run_request(&*self.backend, request);
                self.jobs.finish(job, result);
            }
            Wait::Poll
        };
        self.jobs.read(id, wait)
    }

    /// Runs a `tools/call`. `queued` is taken off the arguments here.
    fn call(&mut self, name: &str, mut args: Value) -> Value {
        let queued = match args.get("queued") {
            None | Some(Value::Null) => false,
            Some(Value::Bool(q)) => *q,
            Some(_) => return tool_error("`queued` is true or false"),
        };
        if let Some(obj) = args.as_object_mut() {
            obj.remove("queued");
        }
        if !known_tool(name) {
            return tool_error(&format!("unknown tool `{name}`"));
        }
        if OWN.contains(&name) {
            if queued {
                return tool_error(&format!("`{name}` cannot be queued"));
            }
            return self.own_tool(name, &args);
        }
        let request = Request::Tool { name: name.to_string(), args };
        if !queued {
            return self.call_sync(request);
        }
        match self.jobs.submit(request, true) {
            Ok(job) => text_result(&json!({"job": job, "tool": name, "state": State::Queued.name()}), false),
            Err(message) => tool_error(&message),
        }
    }

    /// The tools the server answers itself: `set_tool_mode` and the job tools.
    fn own_tool(&mut self, name: &str, args: &Value) -> Value {
        if name == "set_tool_mode" {
            return self.set_mode(args);
        }
        let Some(job) = args.get("job").and_then(Value::as_u64) else {
            return tool_error("`job` is the id a queued call returned");
        };
        let view = match name {
            "job_wait" => {
                let ms = match args.get("timeout_ms") {
                    None | Some(Value::Null) => DEFAULT_WAIT_MS,
                    Some(v) => match v.as_u64() {
                        Some(ms) => ms.min(MAX_WAIT_MS),
                        None => return tool_error("`timeout_ms` is a whole number of milliseconds"),
                    },
                };
                self.await_job(job, Wait::For(Duration::from_millis(ms)))
            }
            "job_cancel" => {
                self.jobs.cancel(job);
                self.jobs.read(job, Wait::Poll)
            }
            _ => self.jobs.read(job, Wait::Poll),
        };
        match view {
            Some(v) => job_result(&v),
            None => tool_error(&format!("unknown job {job}: never queued, or its result was already read")),
        }
    }

    /// `set_tool_mode`: changes the listing; what is not given stays as it was.
    fn set_mode(&mut self, args: &Value) -> Value {
        let mut config = self.config.clone();
        match args.get("mode") {
            None | Some(Value::Null) => {}
            Some(Value::String(mode)) => match mode.as_str() {
                "full" => config.compact = false,
                "compact" => config.compact = true,
                _ => return tool_error("`mode` is \"full\" or \"compact\""),
            },
            Some(_) => return tool_error("`mode` is \"full\" or \"compact\""),
        }
        match args.get("areas") {
            None => {}
            Some(Value::Null) => config.areas = None,
            Some(Value::Array(list)) => {
                let mut names = Vec::with_capacity(list.len());
                for a in list {
                    match a.as_str() {
                        Some(name) => names.push(name.to_string()),
                        None => return tool_error("`areas` is a list of area names"),
                    }
                }
                match check_areas(names) {
                    Ok(areas) => config.areas = areas,
                    Err(message) => return tool_error(&message),
                }
            }
            Some(_) => return tool_error("`areas` is a list of area names, or null for all"),
        }
        self.config = config;
        let tools = self.tools_list();
        let names: Vec<&str> = tools["tools"]
            .as_array()
            .map(|list| list.iter().filter_map(|t| t["name"].as_str()).collect())
            .unwrap_or_default();
        text_result(
            &json!({
                "mode": if self.config.compact { "compact" } else { "full" },
                "areas": self.config.areas,
                "tools": names,
            }),
            false,
        )
    }

    /// The listing: the general tools, then one tool per command (full) or per area (compact),
    /// filtered by the configured areas.
    fn tools_list(&self) -> Value {
        let mut tools = general_tools();
        let wanted = |area: &str| self.config.areas.as_ref().is_none_or(|list| list.iter().any(|a| a == area));
        if self.config.compact {
            for area in areas().into_iter().filter(|a| wanted(a)) {
                tools.push(area_tool_spec(area));
            }
        } else {
            for c in registry().iter().filter(|c| wanted(area_of(c.id))) {
                tools.push(command_tool(c));
            }
        }
        json!({"tools": tools})
    }

    /// Handles one JSON-RPC message; `None` for notifications.
    pub fn handle(&mut self, msg: &Value) -> Option<Value> {
        let id = msg.get("id").cloned();
        let method = msg.get("method").and_then(Value::as_str);
        let Some(method) = method else {
            return id.map(|i| rpc_error(&i, -32600, "invalid request: no method"));
        };
        let Some(id) = id else {
            // notifications (initialized, cancelled…) need no answer
            if method == "notifications/initialized" {
                self.initialized = true;
            }
            return None;
        };
        let params = msg.get("params").cloned().unwrap_or(Value::Null);
        Some(match method {
            "initialize" => {
                let asked = params.get("protocolVersion").and_then(Value::as_str).unwrap_or(PROTOCOLS[0]);
                let version = PROTOCOLS.iter().find(|p| **p == asked).copied().unwrap_or(PROTOCOLS[0]);
                rpc_ok(
                    &id,
                    json!({
                        "protocolVersion": version,
                        "capabilities": {"tools": {"listChanged": false}, "resources": {"listChanged": false}},
                        "serverInfo": {"name": "surfacecraft", "version": env!("CARGO_PKG_VERSION")},
                        "instructions": "SurfaceCraft scan-to-CAD. Every UI action is a tool. Start with guide_read (a guide per task), then list_commands or inspect_design. Meshes: mesh_import (path or base64 data) or mesh_synth. Several steps that keep their effects: app_batch. A long call: queued true, then job_wait."
                    }),
                )
            }
            "ping" => rpc_ok(&id, json!({})),
            "tools/list" => rpc_ok(&id, self.tools_list()),
            "tools/call" => {
                let Some(name) = params.get("name").and_then(Value::as_str) else {
                    return Some(rpc_error(&id, -32602, "tools/call needs `name`"));
                };
                let name = name.to_string();
                let args = params.get("arguments").cloned().unwrap_or(Value::Null);
                rpc_ok(&id, self.call(&name, args))
            }
            "resources/list" => rpc_ok(
                &id,
                json!({"resources": [
                    {"uri": "surfacecraft://design", "name": "design", "mimeType": "application/json", "description": "Objects of the open design"},
                    {"uri": "surfacecraft://commands", "name": "commands", "mimeType": "application/json", "description": "Every command"}
                ]}),
            ),
            "resources/read" => {
                let uri = params.get("uri").and_then(Value::as_str).unwrap_or("");
                let request = match uri {
                    "surfacecraft://design" => Request::Command { id: "doc.list".into(), params: Value::Null },
                    "surfacecraft://commands" => Request::Command { id: "app.commands".into(), params: Value::Null },
                    _ => return Some(rpc_error(&id, -32602, "unknown resource")),
                };
                let v = self.call_sync(request).get("structuredContent").cloned().unwrap_or(Value::Null);
                rpc_ok(&id, json!({"contents": [{"uri": uri, "mimeType": "application/json", "text": v.to_string()}]}))
            }
            other => rpc_error(&id, -32601, &format!("method not found: {other}")),
        })
    }

    /// Handles one line of text (a JSON-RPC message or batch); returns the reply line, if any.
    pub fn handle_line(&mut self, line: &str) -> Option<String> {
        let line = line.trim();
        if line.is_empty() {
            return None;
        }
        let v: Value = match serde_json::from_str(line) {
            Ok(v) => v,
            Err(e) => return Some(rpc_error(&Value::Null, -32700, &format!("parse error: {e}")).to_string()),
        };
        match v {
            Value::Array(list) => {
                let out: Vec<Value> = list.iter().filter_map(|m| self.handle(m)).collect();
                (!out.is_empty()).then(|| Value::Array(out).to_string())
            }
            m => self.handle(&m).map(|r| r.to_string()),
        }
    }
}

/// A job's status as a tool result. Once finished it carries the job's own content (so an image
/// comes back as an image); the structured part names the job, its state and its result.
fn job_result(v: &View) -> Value {
    let structured = json!({
        "job": v.job,
        "tool": v.label,
        "state": v.state.name(),
        "result": v.result.get("structuredContent").cloned().unwrap_or(Value::Null),
    });
    let content = match v.result.get("content") {
        Some(c) if v.state.finished() => c.clone(),
        _ => json!([{"type": "text", "text": err_text(&structured)}]),
    };
    json!({"content": content, "structuredContent": structured, "isError": false})
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::mpsc::{Receiver, channel};

    fn server() -> Server<LocalBackend> {
        Server::new(LocalBackend::new(false))
    }

    /// One `tools/call` through the protocol.
    fn call<B: Backend + Send + 'static>(s: &mut Server<B>, name: &str, args: Value) -> Value {
        s.handle(
            &json!({"jsonrpc": "2.0", "id": 7, "method": "tools/call", "params": {"name": name, "arguments": args}}),
        )
        .expect("a reply")
    }

    /// The tool names listed now.
    fn names<B: Backend + Send + 'static>(s: &mut Server<B>) -> Vec<String> {
        let t = s.handle(&json!({"jsonrpc": "2.0", "id": 2, "method": "tools/list"})).expect("a reply");
        t["result"]["tools"]
            .as_array()
            .map(|list| list.iter().filter_map(|x| x["name"].as_str().map(str::to_string)).collect())
            .unwrap_or_default()
    }

    fn structured(r: &Value) -> &Value {
        &r["result"]["structuredContent"]
    }

    fn object_count(s: &mut Server<LocalBackend>) -> usize {
        structured(&call(s, "inspect_design", json!({})))["objects"].as_array().map_or(0, Vec::len)
    }

    #[test]
    fn handshake_and_tools_cover_every_command() {
        let mut s = server();
        let r = s
            .handle(&json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-03-26"}}))
            .unwrap();
        assert_eq!(r["result"]["protocolVersion"], "2025-03-26");
        assert!(s.handle(&json!({"jsonrpc": "2.0", "method": "notifications/initialized"})).is_none());
        let t = s.handle(&json!({"jsonrpc": "2.0", "id": 2, "method": "tools/list"})).unwrap();
        let names: Vec<String> =
            t["result"]["tools"].as_array().unwrap().iter().map(|x| x["name"].as_str().unwrap().to_string()).collect();
        // every command is a tool
        for c in registry() {
            assert!(names.contains(&tool_name(c.id)), "missing tool for {}", c.id);
            assert_eq!(command_id(&tool_name(c.id)), Some(c.id));
        }
        assert!(names.len() >= registry().len() + 8);
    }

    #[test]
    fn tools_execute_batch_rollback_and_errors() {
        let mut s = server();
        let call = |s: &mut Server<LocalBackend>, name: &str, args: Value| {
            s.handle(&json!({"jsonrpc": "2.0", "id": 7, "method": "tools/call", "params": {"name": name, "arguments": args}})).unwrap()
        };
        let r = call(&mut s, "mesh_synth", json!({"shape": "sphere", "resolution": 16}));
        assert_eq!(r["result"]["isError"], false, "{r}");
        let r = call(
            &mut s,
            "batch",
            json!({"commands": [
                {"command": "body.box", "params": {"size": [1, 1, 1]}},
                {"command": "body.fillet", "params": {"object": "box", "edges": [999], "size": 1}}
            ]}),
        );
        assert_eq!(r["result"]["isError"], true);
        assert_eq!(r["result"]["structuredContent"]["rolled_back"], 1);
        let d = call(&mut s, "inspect_design", json!({}));
        assert_eq!(d["result"]["structuredContent"]["objects"].as_array().unwrap().len(), 1);
        let r = call(&mut s, "mesh_import", json!({"path": "/etc/passwd"}));
        assert_eq!(r["result"]["isError"], true, "file access is off in this server");
        let r = call(&mut s, "screenshot", json!({}));
        assert_eq!(r["result"]["isError"], true);
        assert!(s.handle_line("{not json").unwrap().contains("-32700"));
        let r = s.handle(&json!({"jsonrpc": "2.0", "id": 3, "method": "nope"})).unwrap();
        assert_eq!(r["error"]["code"], -32601);
        let r = s.handle(&json!({"jsonrpc": "2.0", "id": 4, "method": "resources/read", "params": {"uri": "surfacecraft://design"}})).unwrap();
        assert!(r["result"]["contents"][0]["text"].as_str().unwrap().contains("objects"));
    }

    /// Hostile messages never panic and always get an answer when they carry an id.
    #[test]
    fn hostile_messages_never_panic() {
        let mut s = server();
        for line in [
            "",
            "[]",
            "null",
            "42",
            r#"{"id": 1}"#,
            r#"{"id": 1, "method": "tools/call"}"#,
            r#"{"id": 1, "method": "tools/call", "params": {"name": 5}}"#,
            r#"{"id": 1, "method": "tools/call", "params": {"name": "execute", "arguments": {"command": "no.such"}}}"#,
            r#"{"id": 1, "method": "tools/call", "params": {"name": "batch", "arguments": {"commands": [1, 2]}}}"#,
            r#"{"id": 1, "method": "tools/call", "params": {"name": "screenshot", "arguments": {"width": -5}}}"#,
            r#"[{"id": 1, "method": "ping"}, {"method": "x"}]"#,
        ] {
            let _ = s.handle_line(line);
        }
    }

    #[test]
    fn guide_read_lists_the_guides_and_returns_one() {
        let mut s = server();
        let list = call(&mut s, "guide_read", json!({}));
        assert_eq!(list["result"]["isError"], false, "{list}");
        let guides = structured(&list)["guides"].as_array().expect("guides");
        assert_eq!(guides.len(), 7);
        assert!(guides.iter().all(|g| g["sections"].as_array().is_some_and(|s| !s.is_empty())));
        let one = call(&mut s, "guide_read", json!({"topic": "build-solid"}));
        assert!(structured(&one)["guides"][0]["markdown"].as_str().is_some_and(|m| m.contains("body.from_regions")));
        let bad = call(&mut s, "guide_read", json!({"topic": "nope"}));
        assert_eq!(bad["result"]["isError"], true);
    }

    #[test]
    fn compact_mode_lists_one_tool_per_area_and_dispatches_verbs() {
        let mut s = Server::new(LocalBackend::new(false)).with_tools(ToolConfig { compact: true, areas: None });
        let n = names(&mut s);
        assert!(n.contains(&"mesh".to_string()) && n.contains(&"guide".to_string()), "{n:?}");
        assert!(!n.iter().any(|t| t == "mesh_import"));
        assert!(n.contains(&"execute".to_string()) && n.contains(&"job_wait".to_string()));
        let r = call(&mut s, "mesh", json!({"verb": "synth", "params": {"shape": "sphere", "size": [5.0]}}));
        assert_eq!(r["result"]["isError"], false, "{r}");
        let bad = call(&mut s, "mesh", json!({"verb": "nope"}));
        assert_eq!(bad["result"]["isError"], true);
        assert_eq!(object_count(&mut s), 1);
    }

    #[test]
    fn areas_filter_the_listing_and_set_tool_mode_changes_it_at_run_time() {
        let mut s = server();
        assert!(names(&mut s).contains(&"mesh_import".to_string()));
        let r = call(&mut s, "set_tool_mode", json!({"areas": ["guide", "app"]}));
        assert_eq!(r["result"]["isError"], false, "{r}");
        let n = names(&mut s);
        assert!(n.contains(&"guide_read".to_string()) && n.contains(&"app_batch".to_string()), "{n:?}");
        assert!(!n.iter().any(|t| t.starts_with("mesh_")));
        assert!(n.contains(&"list_commands".to_string()), "the general tools stay");
        let bad = call(&mut s, "set_tool_mode", json!({"areas": ["nope"]}));
        assert_eq!(bad["result"]["isError"], true);
        call(&mut s, "set_tool_mode", json!({"mode": "compact"}));
        let n = names(&mut s);
        assert!(n.contains(&"app".to_string()) && !n.contains(&"app_batch".to_string()), "{n:?}");
        call(&mut s, "set_tool_mode", json!({"mode": "full", "areas": null}));
        assert!(names(&mut s).contains(&"mesh_import".to_string()));
    }

    #[test]
    fn app_batch_runs_steps_as_undo_steps_and_stops_at_the_first_error() {
        let mut s = server();
        let r = call(
            &mut s,
            "app_batch",
            json!({"commands": [
                {"command": "body.box", "params": {"size": [1, 1, 1], "name": "a"}},
                {"command": "body.fillet", "params": {"body": "missing", "edges": [0], "size": 1}},
                {"command": "body.box", "params": {"size": [2, 2, 2], "name": "b"}}
            ]}),
        );
        assert_eq!(r["result"]["isError"], true, "a failed step is an error result: {r}");
        assert_eq!(structured(&r)["succeeded"], 1);
        assert_eq!(structured(&r)["not_run"], 1);
        assert_eq!(object_count(&mut s), 1);
        call(&mut s, "undo", json!({}));
        assert_eq!(object_count(&mut s), 0, "the first step was its own undo step");
    }

    /// A backend whose `body.box` named `first` waits for the test's go-ahead. It records what
    /// ran, in order.
    struct Gate {
        go: Receiver<()>,
        ran: Arc<Mutex<Vec<String>>>,
    }

    impl Backend for Gate {
        fn execute(&mut self, id: &str, params: Value) -> Result<Value, Value> {
            let who = format!("{id}:{}", params.get("name").and_then(Value::as_str).unwrap_or("-"));
            if who == "body.box:first" {
                let _ = self.go.recv_timeout(Duration::from_secs(20));
            }
            if let Ok(mut ran) = self.ran.lock() {
                ran.push(who);
            }
            Ok(json!({"ran": id}))
        }
    }

    #[test]
    fn queued_jobs_run_in_order_and_a_queued_job_can_be_cancelled_before_it_starts() {
        let (go, gate) = channel::<()>();
        let ran = Arc::new(Mutex::new(Vec::new()));
        let mut s = Server::new(Gate { go: gate, ran: Arc::clone(&ran) });
        let first = call(&mut s, "body_box", json!({"queued": true, "size": [1, 1, 1], "name": "first"}));
        assert_eq!(first["result"]["isError"], false, "{first}");
        let job1 = structured(&first)["job"].as_u64().expect("a job id");
        let second = call(&mut s, "body_box", json!({"queued": true, "size": [2, 2, 2], "name": "second"}));
        let job2 = structured(&second)["job"].as_u64().expect("a job id");
        let cancelled = call(&mut s, "job_cancel", json!({"job": job2}));
        assert_eq!(structured(&cancelled)["state"], "cancelled", "{cancelled}");
        // the gate is still closed: the first job cannot be done
        let status = call(&mut s, "job_status", json!({"job": job1}));
        assert_ne!(structured(&status)["state"], "done", "{status}");
        go.send(()).expect("release the gate");
        // a call sent now runs after the first job
        let third = call(&mut s, "body_box", json!({"size": [3, 3, 3], "name": "third"}));
        assert_eq!(third["result"]["isError"], false, "{third}");
        let waited = call(&mut s, "job_wait", json!({"job": job1, "timeout_ms": 10_000}));
        assert_eq!(structured(&waited)["state"], "done", "{waited}");
        assert_eq!(structured(&waited)["result"]["ran"], "body.box", "{waited}");
        let order = ran.lock().map(|r| r.to_vec()).unwrap_or_default();
        assert_eq!(order, vec!["body.box:first".to_string(), "body.box:third".to_string()]);
        // a finished job's result is read once
        let gone = call(&mut s, "job_status", json!({"job": job1}));
        assert_eq!(gone["result"]["isError"], true, "{gone}");
    }

    #[test]
    fn hostile_queue_and_mode_arguments_never_panic() {
        let mut s = server();
        for (name, args) in [
            ("set_tool_mode", json!({"mode": 5})),
            ("set_tool_mode", json!({"areas": 5})),
            ("set_tool_mode", json!({"areas": [1, {}]})),
            ("set_tool_mode", json!({"areas": ["mesh", "mesh", "x\u{0}"]})),
            ("set_tool_mode", json!({"queued": true})),
            ("job_wait", json!({"job": "x"})),
            ("job_wait", json!({"job": 1, "timeout_ms": -1})),
            ("job_wait", json!({"job": 99_999, "timeout_ms": 1e300})),
            ("job_status", json!({})),
            ("job_cancel", json!({"job": null})),
            ("job_status", json!({"job": 0})),
            ("body_box", json!({"queued": "yes"})),
            ("body_box", json!({"queued": true, "size": "x"})),
            ("mesh", json!({"verb": 3})),
            ("mesh", json!({"verb": "import", "params": 7})),
            ("app_batch", json!({"commands": 3})),
            ("app_batch", json!({"commands": [{"command": "app.batch"}]})),
            ("app_batch", json!({"commands": [{"command": "x", "extra": 1}]})),
            ("guide_read", json!({"topic": 4})),
            ("no_such_tool", json!({})),
        ] {
            let r = call(&mut s, name, args);
            assert!(r["result"]["isError"].is_boolean(), "{name}: {r}");
        }
    }
}

/// Bridged mode: commands go to a running desktop app over its localhost control channel.
#[cfg(not(target_arch = "wasm32"))]
pub mod bridge {
    use super::Backend;
    use serde_json::{Value, json};
    use std::io::{BufRead, BufReader, Write};
    use std::net::TcpStream;

    /// A connection to a running app.
    pub struct BridgeBackend {
        stream: TcpStream,
        reader: BufReader<TcpStream>,
        token: String,
        next: u64,
    }

    impl BridgeBackend {
        /// Connects using a discovery file `{port, token}`; `None` when no app answers.
        #[must_use]
        pub fn connect(discovery: &std::path::Path) -> Option<Self> {
            let info: Value = serde_json::from_str(&std::fs::read_to_string(discovery).ok()?).ok()?;
            let port = u16::try_from(info.get("port")?.as_u64()?).ok()?;
            let token = info.get("token")?.as_str()?.to_string();
            let addr = std::net::SocketAddr::from(([127, 0, 0, 1], port));
            let stream = TcpStream::connect_timeout(&addr, std::time::Duration::from_millis(500)).ok()?;
            let reader = BufReader::new(stream.try_clone().ok()?);
            let mut b = Self { stream, reader, token, next: 1 };
            // a live app answers a harmless command
            b.request("engine.execute", json!({"command": "app.commands", "params": {"filter": "app."}})).ok()?;
            Some(b)
        }

        fn request(&mut self, method: &str, params: Value) -> Result<Value, Value> {
            let id = self.next;
            self.next += 1;
            let msg = json!({"token": self.token, "id": id, "method": method, "params": params});
            writeln!(self.stream, "{msg}").map_err(|e| json!({"kind": "failed", "message": e.to_string()}))?;
            let mut line = String::new();
            self.reader.read_line(&mut line).map_err(|e| json!({"kind": "failed", "message": e.to_string()}))?;
            let v: Value =
                serde_json::from_str(&line).map_err(|e| json!({"kind": "failed", "message": e.to_string()}))?;
            match (v.get("result"), v.get("error")) {
                (Some(r), _) => Ok(r.clone()),
                (None, Some(e)) => Err(json!({"kind": "failed", "message": e})),
                _ => Err(json!({"kind": "failed", "message": "malformed reply"})),
            }
        }
    }

    impl Backend for BridgeBackend {
        fn execute(&mut self, id: &str, params: Value) -> Result<Value, Value> {
            let r = self.request("engine.execute", json!({"command": id, "params": params}))?;
            match (r.get("ok"), r.get("err")) {
                (Some(v), _) => Ok(v.clone()),
                (None, Some(e)) => Err(e.clone()),
                _ => Err(json!({"kind": "failed", "message": "malformed reply"})),
            }
        }
        fn screenshot(&mut self, w: u32, h: u32) -> Option<Vec<u8>> {
            let r = self.request("ui.screenshot", json!({"width": w, "height": h})).ok()?;
            let s = r.get("png")?.as_str()?;
            decode_b64(s)
        }
    }

    fn decode_b64(s: &str) -> Option<Vec<u8>> {
        let val = |c: u8| -> Option<u32> {
            Some(match c {
                b'A'..=b'Z' => u32::from(c - b'A'),
                b'a'..=b'z' => u32::from(c - b'a') + 26,
                b'0'..=b'9' => u32::from(c - b'0') + 52,
                b'+' => 62,
                b'/' => 63,
                _ => return None,
            })
        };
        let bytes: Vec<u8> = s.bytes().filter(|b| *b != b'=' && !b.is_ascii_whitespace()).collect();
        let mut out = Vec::with_capacity(bytes.len() * 3 / 4);
        for chunk in bytes.chunks(4) {
            let mut n = 0u32;
            for (k, &c) in chunk.iter().enumerate() {
                n |= val(c)? << (18 - 6 * k as u32);
            }
            let full = [(n >> 16) as u8, (n >> 8) as u8, n as u8];
            out.extend_from_slice(full.get(..chunk.len().saturating_sub(1))?);
        }
        Some(out)
    }
}
