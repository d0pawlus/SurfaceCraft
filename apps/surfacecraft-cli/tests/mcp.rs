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

/// Starts `surfacecraft-cli mcp --local` with `args` and extra environment; returns the child and
/// its stdin and stdout.
fn start_mcp(
    args: &[&str],
    env: &[(&str, &str)],
) -> (std::process::Child, std::process::ChildStdin, BufReader<std::process::ChildStdout>) {
    let mut cmd = Command::new(env!("CARGO_BIN_EXE_surfacecraft-cli"));
    cmd.args(["mcp", "--local"]).args(args).env_remove("SURFACECRAFT_MCP_TOOLS").env_remove("SURFACECRAFT_MCP_AREAS");
    for (k, v) in env {
        cmd.env(k, v);
    }
    let mut child = cmd.stdin(Stdio::piped()).stdout(Stdio::piped()).stderr(Stdio::null()).spawn().unwrap();
    let stdin = child.stdin.take().unwrap();
    let stdout = BufReader::new(child.stdout.take().unwrap());
    (child, stdin, stdout)
}

/// One request/reply exchange with the server.
fn exchange(stdin: &mut std::process::ChildStdin, out: &mut BufReader<std::process::ChildStdout>, msg: Value) -> Value {
    writeln!(stdin, "{msg}").unwrap();
    stdin.flush().unwrap();
    let mut line = String::new();
    out.read_line(&mut line).unwrap();
    serde_json::from_str(&line).unwrap()
}

/// The tool names a `tools/list` reply carries.
fn listed(reply: &Value) -> Vec<String> {
    reply["result"]["tools"].as_array().unwrap().iter().map(|t| t["name"].as_str().unwrap().to_string()).collect()
}

#[test]
fn gate_p8_compact_mode_and_queued_jobs_over_stdio() {
    let list = json!({"jsonrpc": "2.0", "id": 2, "method": "tools/list"});
    // the listing comes from the arguments: one tool per area, only the areas asked for
    let (mut child, mut stdin, mut out) = start_mcp(&["--tools", "compact", "--areas", "mesh,app,guide"], &[]);
    let names = listed(&exchange(&mut stdin, &mut out, list.clone()));
    assert!(names.iter().any(|n| n == "mesh") && names.iter().any(|n| n == "guide"), "{names:?}");
    assert!(!names.iter().any(|n| n.starts_with("mesh_") || n == "body"), "{names:?}");
    assert!(names.iter().any(|n| n == "job_wait") && names.iter().any(|n| n == "set_tool_mode"));
    let call = |name: &str, args: Value| json!({"jsonrpc": "2.0", "id": 3, "method": "tools/call", "params": {"name": name, "arguments": args}});
    // a queued call answers at once with its job id; job_wait returns the result
    let queued = exchange(
        &mut stdin,
        &mut out,
        call(
            "mesh",
            json!({"verb": "synth", "params": {"shape": "bracket", "noise": 0.05, "seed": 4, "name": "scan"}, "queued": true}),
        ),
    );
    assert_eq!(queued["result"]["structuredContent"]["state"], "queued", "{queued}");
    let job = queued["result"]["structuredContent"]["job"].clone();
    let done = exchange(&mut stdin, &mut out, call("job_wait", json!({"job": job, "timeout_ms": 60_000})));
    assert_eq!(done["result"]["structuredContent"]["state"], "done", "{done}");
    // several steps in one call, each its own undo step
    let r = exchange(
        &mut stdin,
        &mut out,
        call(
            "app",
            json!({"verb": "batch", "params": {"commands": [{"command": "body.box", "params": {"size": [10, 10, 10]}}]}}),
        ),
    );
    assert_eq!(r["result"]["isError"], false, "{r}");
    // the listing changes at run time
    let r = exchange(&mut stdin, &mut out, call("set_tool_mode", json!({"mode": "full", "areas": null})));
    assert_eq!(r["result"]["isError"], false, "{r}");
    let names = listed(&exchange(&mut stdin, &mut out, list.clone()));
    assert!(names.iter().any(|n| n == "mesh_import") && names.iter().any(|n| n == "body_box"));
    drop(stdin);
    let _ = child.wait();

    // without arguments the environment chooses
    let (mut child, mut stdin, mut out) = start_mcp(&[], &[("SURFACECRAFT_MCP_TOOLS", "compact")]);
    let names = listed(&exchange(&mut stdin, &mut out, list));
    assert!(names.iter().any(|n| n == "mesh") && !names.iter().any(|n| n == "mesh_import"), "{names:?}");
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
