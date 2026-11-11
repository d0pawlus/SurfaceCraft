//! Tests of the server through its protocol.
use super::*;
use std::sync::mpsc::{Receiver, channel};

/// A server with the full listing (one tool per command).
fn server() -> Server<LocalBackend> {
    Server::new(LocalBackend::new(false)).with_tools(ToolConfig::full())
}

/// A server as an agent gets it: the default (compact) listing.
fn default_server() -> Server<LocalBackend> {
    Server::new(LocalBackend::new(false))
}

/// One `tools/call` through the protocol.
fn call<B: Backend + Send + 'static>(s: &mut Server<B>, name: &str, args: Value) -> Value {
    s.handle(&json!({"jsonrpc": "2.0", "id": 7, "method": "tools/call", "params": {"name": name, "arguments": args}}))
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
        .handle(
            &json!({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {"protocolVersion": "2025-03-26"}}),
        )
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
        s.handle(
            &json!({"jsonrpc": "2.0", "id": 7, "method": "tools/call", "params": {"name": name, "arguments": args}}),
        )
        .unwrap()
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
    let r = s
        .handle(
            &json!({"jsonrpc": "2.0", "id": 4, "method": "resources/read", "params": {"uri": "surfacecraft://design"}}),
        )
        .unwrap();
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
    assert_eq!(guides.len(), 8);
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
    assert!(n.contains(&"mesh_import".to_string()), "a featured command is listed in full");
    assert!(!n.iter().any(|t| t == "mesh_clean"));
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
    assert!(n.contains(&"app".to_string()) && !n.contains(&"app_describe".to_string()), "{n:?}");
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

/// One request through the protocol.
fn rpc<B: Backend + Send + 'static>(s: &mut Server<B>, method: &str, params: Value) -> Value {
    s.handle(&json!({"jsonrpc": "2.0", "id": 9, "method": method, "params": params})).expect("a reply")
}

/// Largest default `tools/list` reply an agent should have to read (bytes).
const MAX_DEFAULT_LISTING: usize = 48_000;

#[test]
fn the_default_listing_is_small_and_carries_the_workflow_tools_in_full() {
    // Arrange
    let mut s = default_server();

    // Act
    let reply = rpc(&mut s, "tools/list", Value::Null);

    // Assert
    let size = reply.to_string().len();
    assert!(size < MAX_DEFAULT_LISTING, "the default listing is {size} bytes");
    let tools = reply["result"]["tools"].as_array().expect("tools");
    let tool = |name: &str| tools.iter().find(|t| t["name"] == name);
    for id in FEATURED {
        let t = tool(&tool_name(id)).unwrap_or_else(|| panic!("{id} is not listed"));
        assert!(t["inputSchema"]["properties"].is_object(), "{id} has no input schema");
        assert!(t.get("outputSchema").is_none(), "{id} carries an output schema");
    }
    assert!(tool("scan_to_solid").is_some_and(|t| t["inputSchema"]["properties"]["scan"].is_object()));
    assert!(tool("region").is_some() && tool("region_merge").is_none(), "areas, not commands");
    // a command that is not listed is still callable by its tool name
    let r = call(&mut s, "mesh_synth", json!({"shape": "sphere", "resolution": 16}));
    assert_eq!(r["result"]["isError"], false, "{r}");
}

#[test]
fn errors_carry_what_the_next_call_needs() {
    // Arrange
    let mut s = default_server();

    // Act
    let params = call(&mut s, "scan_qa", json!({"objekt": 1}));
    let verb = call(&mut s, "region", json!({"verb": "merg"}));
    let command = call(&mut s, "execute", json!({"command": "scan.to_solids"}));
    let tool = call(&mut s, "scan_tosolid", json!({}));

    // Assert
    assert_eq!(params["result"]["isError"], true, "{params}");
    assert_eq!(structured(&params)["command"], "scan.qa");
    assert!(structured(&params)["input_schema"]["properties"]["object"].is_object(), "{params}");
    assert_eq!(structured(&verb)["did_you_mean"][0], "merge", "{verb}");
    assert_eq!(structured(&command)["did_you_mean"][0], "scan.to_solid", "{command}");
    assert_eq!(structured(&tool)["did_you_mean"][0], "scan_to_solid", "{tool}");
}

#[test]
fn results_are_compact_text_with_rounded_floats() {
    // Arrange
    let mut s = default_server();

    // Act
    let r = call(&mut s, "mesh_synth", json!({"shape": "sphere", "size": [5.0], "noise": 0.03, "resolution": 16}));

    // Assert
    let text = r["result"]["content"][0]["text"].as_str().expect("text");
    assert!(!text.contains('\n'), "the text is pretty-printed: {text}");
    let parsed: Value = serde_json::from_str(text).expect("the text is JSON");
    assert_eq!(&parsed, structured(&r));
    let area = structured(&r)["area"].as_f64().expect("area");
    assert!((area * 1e6 - (area * 1e6).round()).abs() < 1e-3, "area {area} has more than 6 decimals");
    assert!(structured(&r)["triangles"].is_u64(), "integers stay integers");
}

#[test]
fn prompts_and_guide_resources_answer() {
    // Arrange
    let mut s = default_server();

    // Act
    let init = rpc(&mut s, "initialize", json!({}));
    let prompts = rpc(&mut s, "prompts/list", Value::Null);
    let prompt = rpc(&mut s, "prompts/get", json!({"name": "scan_to_solid", "arguments": {"scan": "part.stl"}}));
    let no_prompt = rpc(&mut s, "prompts/get", json!({"name": "nope"}));
    let resources = rpc(&mut s, "resources/list", Value::Null);
    let guide = rpc(&mut s, "resources/read", json!({"uri": "surfacecraft://guide/scan-to-solid"}));
    let no_guide = rpc(&mut s, "resources/read", json!({"uri": "surfacecraft://guide/nope"}));

    // Assert
    assert!(init["result"]["capabilities"]["prompts"].is_object(), "{init}");
    assert!(init["result"]["instructions"].as_str().is_some_and(|i| i.contains("scan_to_solid")), "{init}");
    assert_eq!(prompts["result"]["prompts"][0]["name"], "scan_to_solid");
    let text = prompt["result"]["messages"][0]["content"]["text"].as_str().expect("prompt text");
    assert!(text.contains("part.stl") && text.contains("scan_to_solid"), "{text}");
    assert_eq!(no_prompt["error"]["code"], -32602);
    let uris: Vec<&str> = resources["result"]["resources"]
        .as_array()
        .expect("resources")
        .iter()
        .filter_map(|r| r["uri"].as_str())
        .collect();
    assert!(
        uris.contains(&"surfacecraft://design") && uris.contains(&"surfacecraft://guide/scan-to-solid"),
        "{uris:?}"
    );
    assert_eq!(guide["result"]["contents"][0]["mimeType"], "text/markdown");
    assert!(guide["result"]["contents"][0]["text"].as_str().is_some_and(|m| m.contains("scan.to_solid")), "{guide}");
    assert_eq!(no_guide["error"]["code"], -32602);
}

#[test]
fn the_instructions_follow_the_file_setting() {
    let mut off = default_server().with_files(Some(false));
    let mut on = default_server().with_files(Some(true));
    let text = |s: &mut Server<LocalBackend>| {
        rpc(s, "initialize", json!({}))["result"]["instructions"].as_str().unwrap_or_default().to_string()
    };
    assert!(text(&mut off).contains("--allow-files"));
    assert!(text(&mut on).contains("pass `path`"));
}

#[test]
fn hostile_prompt_and_resource_requests_never_panic() {
    let mut s = default_server();
    for (method, params) in [
        ("prompts/get", json!(null)),
        ("prompts/get", json!({"name": 5, "arguments": [1]})),
        ("prompts/get", json!({"name": "scan_to_solid", "arguments": {"scan": {"x": 1}, "tolerance": -1}})),
        ("prompts/list", json!([1, 2])),
        ("resources/read", json!({"uri": 7})),
        ("resources/read", json!({"uri": "surfacecraft://guide/"})),
        ("resources/read", json!({"uri": "surfacecraft://guide/../../etc/passwd"})),
        ("resources/list", json!("x")),
    ] {
        let r = rpc(&mut s, method, params);
        assert!(r.get("result").is_some() || r.get("error").is_some(), "{method}: {r}");
    }
}
