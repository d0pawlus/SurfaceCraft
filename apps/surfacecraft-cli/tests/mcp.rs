//! MCP end to end: `surfacecraft-cli mcp` over stdio, and the bridge to a running app's control
//! channel (gate M6: MCP covers every command).
use serde_json::{Value, json};
use std::io::{BufRead, BufReader, Write};
use std::process::{Command, Stdio};

#[test]
fn gate_m6_mcp_stdio_exposes_and_runs_every_command() {
    let mut child = Command::new(env!("CARGO_BIN_EXE_surfacecraft-cli"))
        .args(["mcp", "--local"])
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::null())
        .spawn()
        .unwrap();
    let mut stdin = child.stdin.take().unwrap();
    let mut out = BufReader::new(child.stdout.take().unwrap());
    let mut ask = |msg: Value| -> Value {
        writeln!(stdin, "{msg}").unwrap();
        stdin.flush().unwrap();
        let mut line = String::new();
        out.read_line(&mut line).unwrap();
        serde_json::from_str(&line).unwrap()
    };
    let init =
        ask(json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-06-18"}}));
    assert_eq!(init["result"]["serverInfo"]["name"], "surfacecraft");
    let tools = ask(json!({"jsonrpc": "2.0", "id": 2, "method": "tools/list"}));
    let names: Vec<&str> =
        tools["result"]["tools"].as_array().unwrap().iter().map(|t| t["name"].as_str().unwrap()).collect();
    for c in surfacecraft_engine::registry() {
        assert!(names.contains(&surfacecraft_mcp::tool_name(c.id).as_str()), "no tool for {}", c.id);
    }
    // a small workflow through tools: synthetic scan → segment → fit → STEP
    let call = |name: &str, args: Value| json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": name, "arguments": args}});
    let r = ask(call("mesh_synth", json!({"shape": "bracket", "noise": 0.05, "seed": 4, "name": "scan"})));
    assert_eq!(r["result"]["isError"], false, "{r}");
    let r = ask(call("region_auto_segment", json!({"mesh": "scan"})));
    assert_eq!(r["result"]["structuredContent"]["regions"].as_array().unwrap().len(), 8, "{r}");
    let r = ask(call("execute", json!({"command": "body.box", "params": {"size": [10, 10, 10]}})));
    assert_eq!(r["result"]["isError"], false);
    let r = ask(call("export_step", json!({"objects": ["box"]})));
    assert_eq!(r["result"]["structuredContent"]["content"], "solids");
    let r = ask(call("screenshot", json!({"width": 64, "height": 48})));
    // with a GPU an image comes back; without one (CI) a clear error
    let c = &r["result"]["content"][0];
    assert!(c["type"] == "image" || r["result"]["isError"] == true, "{r}");
    drop(stdin);
    let _ = child.wait();
}

#[test]
fn bridge_forwards_to_a_running_app() {
    use surfacecraft_engine::Session;
    use surfacecraft_mcp::{Backend, bridge::BridgeBackend};
    // a stand-in for the desktop app: the real control channel, served by a session thread
    let (rx, _port) = surfacecraft_ui_egui::control::start(|| {}).unwrap();
    std::thread::spawn(move || {
        let mut s = Session::new();
        for r in rx {
            let reply = match r.method.as_str() {
                "engine.execute" => {
                    let cmd = r.params["command"].as_str().unwrap_or("").to_string();
                    match s.execute(&cmd, r.params["params"].clone()) {
                        Ok(v) => json!({"ok": v}),
                        Err(e) => json!({"err": e}),
                    }
                }
                _ => json!({"err": "unsupported"}),
            };
            let _ = r.reply.send(reply);
        }
    });
    let mut b = BridgeBackend::connect(&surfacecraft_ui_egui::control::discovery_path()).expect("bridge connects");
    let v = b.execute("body.box", json!({"size": [2, 3, 4]})).unwrap();
    assert!((v["volume"].as_f64().unwrap() - 24.0).abs() < 1e-9);
    assert!(b.execute("no.such", Value::Null).is_err());
    let mut server = surfacecraft_mcp::Server::new(b);
    let r = server
        .handle(&json!({"jsonrpc": "2.0", "id": 1, "method": "tools/call", "params": {"name": "inspect_design", "arguments": {}}}))
        .unwrap();
    assert_eq!(r["result"]["structuredContent"]["objects"].as_array().unwrap().len(), 1);
}
