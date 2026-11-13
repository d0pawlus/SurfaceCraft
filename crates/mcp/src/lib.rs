//! SurfaceCraft MCP server (layer L5), spike §3.5 item 17: an own JSON-RPC 2.0 implementation
//! (one message per line), no `rmcp`. Every engine command is callable as a tool (`mesh.import`
//! → `mesh_import`). The listing is compact by default: one tool per command area (`mesh` with
//! `verb` and `params`) plus the workflow commands in full ([`FEATURED`]); in full mode it is
//! one tool per command. Generic tools: `list_commands`, `describe_command`, `execute`, `batch` (with
//! rollback), `undo`, `redo`, `inspect_design`, `screenshot` (when the host can render),
//! `set_tool_mode` (the listing, at run time), and the queue: `job_status`, `job_wait`,
//! `job_cancel`. A call with `queued: true` returns a job id at once.
//! Resources: `surfacecraft://design` (the object list), `surfacecraft://commands` and one
//! `surfacecraft://guide/<id>` per guide. Prompts: `scan_to_solid`. Results are lean (see
//! [`lean`]): compact text, rounded floats, and errors that carry what the next call needs.
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

use jobs::{Jobs, Request, State, View, Wait};
use lean::{text_result, tool_error, tool_error_with};
use serde_json::{Value, json};
use std::sync::{Arc, Mutex, PoisonError};
use std::time::Duration;
use surfacecraft_engine::cmd::app::{guide_index, guide_markdown};
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
/// The workflow commands: listed with their full input schema in compact mode too, so the short
/// path (import, scan to solid, compare, export) needs no `describe_command`.
pub const FEATURED: &[&str] =
    &["mesh.import", "scan.qa", "scan.to_solid", "body.compare", "export.step", "guide.read", "app.batch"];
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

/// Which tools `tools/list` shows: one per command area plus the featured commands (compact,
/// the default), or one per command (full). `areas` limits either to some areas. The general
/// tools are always listed.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ToolConfig {
    /// One tool per area (`verb` and `params`) instead of one per command.
    pub compact: bool,
    /// Only these areas (sorted; `None`: every area).
    pub areas: Option<Vec<String>>,
}

impl Default for ToolConfig {
    /// Compact, every area: the listing an agent can afford to read.
    fn default() -> Self {
        Self { compact: true, areas: None }
    }
}

impl ToolConfig {
    /// The full listing: one tool per command, every area.
    #[must_use]
    pub fn full() -> Self {
        Self { compact: false, areas: None }
    }

    /// Reads `mode` (`compact` or `full`) and `areas` (comma-separated). Absent values give the
    /// defaults (compact, every area); an unknown mode or area is an error that lists what is valid.
    pub fn parse(mode: Option<&str>, areas: Option<&str>) -> Result<Self, String> {
        let compact = match mode.map(str::trim) {
            None | Some("" | "compact") => true,
            Some("full") => false,
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

fn rpc_error(id: &Value, code: i64, message: &str) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message}})
}

fn rpc_ok(id: &Value, result: Value) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "result": result})
}

/// Every command id.
fn command_ids() -> Vec<&'static str> {
    registry().iter().map(|c| c.id).collect()
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
        Err(e) => text_result(&lean::with_hints(id, &e, &command_ids()), true),
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
        let known = verbs(area);
        return tool_error_with(&format!("`{area}` has no verb `{verb}`; verbs: {}", known.join(", ")), verb, &known);
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
            unknown_tool(other)
        }
    }
}

/// The error for a tool name the server does not know, with the nearest names.
fn unknown_tool(name: &str) -> Value {
    let names: Vec<String> = registry().iter().map(|c| tool_name(c.id)).collect();
    let mut known: Vec<&str> = names.iter().map(String::as_str).collect();
    known.extend(areas());
    known.extend(GENERAL.iter().chain(OWN).copied());
    tool_error_with(&format!("unknown tool `{name}`"), name, &known)
}

/// The MCP server state.
pub struct Server<B: Backend> {
    backend: Arc<Mutex<B>>,
    jobs: Arc<Jobs>,
    threaded: bool,
    config: ToolConfig,
    files: Option<bool>,
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
        Self { backend, jobs, threaded, config: ToolConfig::default(), files: None, initialized: false }
    }
}

impl<B: Backend> Server<B> {
    /// The same server, listing `config`.
    #[must_use]
    pub fn with_tools(mut self, config: ToolConfig) -> Self {
        self.config = config;
        self
    }

    /// The same server, telling the agent whether path parameters work here (`None`, the
    /// default: not known, as when bridged to an app).
    #[must_use]
    pub fn with_files(mut self, allowed: Option<bool>) -> Self {
        self.files = allowed;
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
            return unknown_tool(name);
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

    /// The listing: the general tools, then the featured commands and one tool per area
    /// (compact) or one tool per command (full), filtered by the configured areas.
    fn tools_list(&self) -> Value {
        let mut tools = listing::general_tools();
        let wanted = |area: &str| self.config.areas.as_ref().is_none_or(|list| list.iter().any(|a| a == area));
        if self.config.compact {
            let featured = FEATURED.iter().filter_map(|id| find(id)).filter(|c| wanted(area_of(c.id)));
            tools.extend(featured.map(listing::featured_tool));
            tools.extend(areas().into_iter().filter(|a| wanted(a)).map(listing::area_tool_spec));
            tools = tools.into_iter().map(listing::with_short_queued).collect();
        } else {
            tools.extend(registry().iter().filter(|c| wanted(area_of(c.id))).map(|c| listing::command_tool(c, true)));
        }
        json!({"tools": tools})
    }

    /// A command's structured result, run in queue order (for the resources); `None` when the
    /// command failed.
    fn command_value(&mut self, id: &str, params: Value) -> Option<Value> {
        let result = self.call_sync(Request::Command { id: id.to_string(), params });
        let failed = result.get("isError").and_then(Value::as_bool).unwrap_or(true);
        if failed { None } else { result.get("structuredContent").cloned() }
    }

    /// `resources/list`: the design, the commands, and one resource per guide. Answered without
    /// the worker, so it never waits behind a running job.
    fn resources_list() -> Value {
        let mut list = vec![
            json!({"uri": "surfacecraft://design", "name": "design", "mimeType": "application/json", "description": "Objects of the open design"}),
            json!({"uri": "surfacecraft://commands", "name": "commands", "mimeType": "application/json", "description": "Every command"}),
        ];
        list.extend(guide_index().into_iter().map(|(id, title)| {
            json!({
                "uri": format!("{}{id}", prompts::GUIDE_URI),
                "name": format!("guide: {id}"),
                "mimeType": "text/markdown",
                "description": title,
            })
        }));
        json!({"resources": list})
    }

    /// `resources/read`: the contents of `uri`, or `None` for an unknown resource.
    fn resource_read(&mut self, uri: &str) -> Option<Value> {
        let (mime, text) = if let Some(topic) = uri.strip_prefix(prompts::GUIDE_URI) {
            ("text/markdown", guide_markdown(topic)?.to_string())
        } else {
            let id = match uri {
                "surfacecraft://design" => "doc.list",
                "surfacecraft://commands" => "app.commands",
                _ => return None,
            };
            let value = self.command_value(id, Value::Null)?;
            ("application/json", value.to_string())
        };
        Some(json!({"contents": [{"uri": uri, "mimeType": mime, "text": text}]}))
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
                        "capabilities": {
                            "tools": {"listChanged": false},
                            "resources": {"listChanged": false},
                            "prompts": {"listChanged": false}
                        },
                        "serverInfo": {"name": "surfacecraft", "version": env!("CARGO_PKG_VERSION")},
                        "instructions": prompts::instructions(self.files)
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
            "resources/list" => rpc_ok(&id, Self::resources_list()),
            "resources/read" => {
                let uri = params.get("uri").and_then(Value::as_str).unwrap_or("");
                match self.resource_read(uri) {
                    Some(contents) => rpc_ok(&id, contents),
                    None => rpc_error(&id, -32602, "unknown resource, or it cannot be read now"),
                }
            }
            "prompts/list" => rpc_ok(&id, prompts::list()),
            "prompts/get" => {
                let name = params.get("name").and_then(Value::as_str).unwrap_or("");
                match prompts::get(name, params.get("arguments").unwrap_or(&Value::Null)) {
                    Some(prompt) => rpc_ok(&id, prompt),
                    None => rpc_error(&id, -32602, "unknown prompt"),
                }
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
        _ => json!([{"type": "text", "text": structured.to_string()}]),
    };
    json!({"content": content, "structuredContent": structured, "isError": false})
}

#[cfg(test)]
mod tests;

#[cfg(not(target_arch = "wasm32"))]
pub mod bridge;
mod jobs;
pub mod lean;
mod listing;
pub mod prompts;
mod shorten;
