//! SurfaceCraft MCP server (layer L5), spike §3.5 item 17: an own JSON-RPC 2.0 implementation
//! (one message per line), no `rmcp`. Every engine command is a tool (`mesh.import` →
//! `mesh_import`), plus generic tools: `list_commands`, `describe_command`, `execute`, `batch`
//! (with rollback), `undo`, `redo`, `inspect_design`, `screenshot` (when the host can render).
//! Resources: `surfacecraft://design` (the object list) and `surfacecraft://commands`.
//!
//! The server is transport-free: [`Server::handle`] maps one request value to one response
//! value (or none, for notifications). The CLI wires it to stdio, or forwards to a running
//! desktop app (bridged mode).
#![deny(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::unimplemented,
    clippy::todo,
    clippy::unreachable
)]

use serde_json::{Value, json};
use surfacecraft_engine::{EngineError, Session, registry};

/// Protocol versions spoken, newest first.
pub const PROTOCOLS: &[&str] = &["2025-06-18", "2025-03-26", "2024-11-05"];

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
    pub render: Option<Box<dyn FnMut(&mut Session, u32, u32) -> Option<Vec<u8>>>>,
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

/// The MCP server state.
pub struct Server<B: Backend> {
    backend: B,
    initialized: bool,
}

fn rpc_error(id: &Value, code: i64, message: &str) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "error": {"code": code, "message": message}})
}

fn rpc_ok(id: &Value, result: Value) -> Value {
    json!({"jsonrpc": "2.0", "id": id, "result": result})
}

fn text_result(v: &Value, is_error: bool) -> Value {
    let text = serde_json::to_string_pretty(v).unwrap_or_else(|_| v.to_string());
    json!({"content": [{"type": "text", "text": text}], "structuredContent": v, "isError": is_error})
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

impl<B: Backend> Server<B> {
    /// A server over `backend`.
    pub fn new(backend: B) -> Self {
        Self { backend, initialized: false }
    }

    /// The backend.
    pub fn backend_mut(&mut self) -> &mut B {
        &mut self.backend
    }

    fn generic_tools() -> Vec<Value> {
        let obj = |props: Value, req: &[&str]| json!({"type": "object", "properties": props, "required": req});
        vec![
            json!({"name": "list_commands", "description": "List SurfaceCraft commands (optional `filter`).",
                "inputSchema": obj(json!({"filter": {"type": "string"}}), &[])}),
            json!({"name": "describe_command", "description": "Input and output JSON Schemas of command `id`.",
                "inputSchema": obj(json!({"id": {"type": "string"}}), &["id"])}),
            json!({"name": "execute", "description": "Execute command `command` with `params`.",
                "inputSchema": obj(json!({"command": {"type": "string"}, "params": {"type": "object"}}), &["command"])}),
            json!({"name": "batch", "description": "Execute `commands` ([{command, params}]) in order; on the first failure every step is undone.",
                "inputSchema": obj(json!({"commands": {"type": "array"}}), &["commands"])}),
            json!({"name": "undo", "description": "Undo the last change.", "inputSchema": obj(json!({}), &[])}),
            json!({"name": "redo", "description": "Redo.", "inputSchema": obj(json!({}), &[])}),
            json!({"name": "inspect_design", "description": "Every object of the design with id, kind, name and facts.",
                "inputSchema": obj(json!({}), &[])}),
            json!({"name": "screenshot", "description": "PNG of the viewport (when the host can render).",
                "inputSchema": obj(json!({"width": {"type": "integer"}, "height": {"type": "integer"}}), &[])}),
        ]
    }

    fn tools_list(&self) -> Value {
        let mut tools = Self::generic_tools();
        for c in registry() {
            tools.push(json!({
                "name": tool_name(c.id),
                "title": c.label,
                "description": format!("{} ({})", c.summary, c.id),
                "inputSchema": (c.input)(),
                "outputSchema": (c.output)(),
                "annotations": {"readOnlyHint": !c.undoable},
            }));
        }
        json!({"tools": tools})
    }

    fn run(&mut self, id: &str, params: Value) -> Value {
        match self.backend.execute(id, params) {
            Ok(v) => text_result(&v, false),
            Err(e) => text_result(&e, true),
        }
    }

    fn batch(&mut self, steps: &[Value]) -> Value {
        let mut done = 0usize;
        let mut results = Vec::new();
        for (k, st) in steps.iter().enumerate() {
            let Some(cmd) = st.get("command").and_then(Value::as_str) else {
                return text_result(&json!({"error": format!("step {k}: missing command")}), true);
            };
            let params = st.get("params").cloned().unwrap_or(Value::Null);
            let undoable = surfacecraft_engine::cmd::find(cmd).is_some_and(|c| c.undoable);
            match self.backend.execute(cmd, params) {
                Ok(v) => {
                    if undoable {
                        done += 1;
                    }
                    results.push(v);
                }
                Err(e) => {
                    for _ in 0..done {
                        let _ = self.backend.execute("edit.undo", Value::Null);
                    }
                    return text_result(&json!({"failed_step": k, "error": e, "rolled_back": done}), true);
                }
            }
        }
        text_result(&json!({"results": results}), false)
    }

    fn call(&mut self, name: &str, args: Value) -> Value {
        let a = |k: &str| args.get(k).cloned().unwrap_or(Value::Null);
        match name {
            "list_commands" => self.run("app.commands", json!({"filter": a("filter")})),
            "describe_command" => self.run("app.describe", json!({"id": a("id")})),
            "execute" => {
                let Some(cmd) = args.get("command").and_then(Value::as_str) else {
                    return text_result(&json!({"error": "missing `command`"}), true);
                };
                let cmd = cmd.to_string();
                self.run(&cmd, a("params"))
            }
            "batch" => {
                let steps = args.get("commands").and_then(Value::as_array).cloned().unwrap_or_default();
                self.batch(&steps)
            }
            "undo" => self.run("edit.undo", Value::Null),
            "redo" => self.run("edit.redo", Value::Null),
            "inspect_design" => self.run("doc.list", Value::Null),
            "screenshot" => {
                let dim = |k: &str, d: u32| args.get(k).and_then(Value::as_u64).map_or(d, |v| v.clamp(16, 4096) as u32);
                match self.backend.screenshot(dim("width", 800), dim("height", 600)) {
                    Some(png) => {
                        json!({"content": [{"type": "image", "data": b64(&png), "mimeType": "image/png"}], "isError": false})
                    }
                    None => text_result(&json!({"error": "this host cannot render screenshots"}), true),
                }
            }
            other => match command_id(other) {
                Some(id) => self.run(id, args),
                None => text_result(&json!({"error": format!("unknown tool `{other}`")}), true),
            },
        }
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
                        "instructions": "SurfaceCraft scan-to-CAD. Every UI action is a tool; start with list_commands or inspect_design. Meshes: mesh_import (path or base64 data) or mesh_synth."
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
                let r = match uri {
                    "surfacecraft://design" => self.backend.execute("doc.list", Value::Null),
                    "surfacecraft://commands" => self.backend.execute("app.commands", Value::Null),
                    _ => return Some(rpc_error(&id, -32602, "unknown resource")),
                };
                let v = r.unwrap_or_else(|e| e);
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

#[cfg(test)]
mod tests {
    use super::*;

    fn server() -> Server<LocalBackend> {
        Server::new(LocalBackend::new(false))
    }

    #[test]
    fn handshake_and_tools_cover_every_command() {
        let mut s = server();
        let r = s.handle(&json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-03-26"}})).unwrap();
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
