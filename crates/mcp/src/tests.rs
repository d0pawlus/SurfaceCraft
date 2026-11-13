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

/// Largest default `tools/list` reply an agent should have to read (bytes). The budget of the lean
/// transport, unchanged: the structural shortening brought the listing to about 37 400 bytes, and
/// the space below the budget is what lets commands gain parameters without hand trimming.
const MAX_DEFAULT_LISTING: usize = 48_000;
/// Smallest gap (bytes) between the default listing and its limit: when less is left, the listing
/// is shortened structurally before a new parameter has to be trimmed by hand.
const MIN_HEADROOM: usize = 2_000;

#[test]
fn the_default_listing_carries_the_workflow_tools_in_full() {
    // Arrange
    let mut s = default_server();

    // Act
    let reply = rpc(&mut s, "tools/list", Value::Null);

    // Assert
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

mod shortening {
    use super::*;
    use crate::shorten::{MAX_DESCRIPTION_CHARS, first_sentence, shorten_description, shorten_schema};

    fn listing_text(s: &mut Server<LocalBackend>) -> String {
        rpc(s, "tools/list", Value::Null).to_string()
    }

    fn listed_tools(s: &mut Server<LocalBackend>) -> Vec<Value> {
        rpc(s, "tools/list", Value::Null)["result"]["tools"].as_array().cloned().unwrap_or_default()
    }

    /// Descriptions of every schema of `v` that sits in a `properties` map.
    fn property_descriptions(v: &Value, out: &mut Vec<String>) {
        match v {
            Value::Object(map) => {
                if let Some(Value::Object(props)) = map.get("properties") {
                    for p in props.values() {
                        if let Some(d) = p.get("description").and_then(Value::as_str) {
                            out.push(d.to_string());
                        }
                    }
                }
                map.values().for_each(|x| property_descriptions(x, out));
            }
            Value::Array(list) => list.iter().for_each(|x| property_descriptions(x, out)),
            _ => {}
        }
    }

    #[test]
    fn first_sentence_of_a_plain_description() {
        assert_eq!(first_sentence("Tolerance in mm. Larger values merge more."), "Tolerance in mm.");
        assert_eq!(first_sentence("No period at all"), "No period at all");
        assert_eq!(first_sentence("Ends here."), "Ends here.");
        assert_eq!(first_sentence(""), "");
        assert_eq!(first_sentence("  padded.  Next"), "padded.");
    }

    #[test]
    fn first_sentence_ignores_abbreviations_decimals_and_code() {
        assert_eq!(
            first_sentence("Scale factor, e.g. 25.4 for inches. Default 1."),
            "Scale factor, e.g. 25.4 for inches."
        );
        assert_eq!(first_sentence("Ratio of 0.2 to 0.5 of the size. More."), "Ratio of 0.2 to 0.5 of the size.");
        assert_eq!(first_sentence("Use `a. b` for both. Then more."), "Use `a. b` for both.");
        assert_eq!(first_sentence("Units, i.e. mm or cm. Rest."), "Units, i.e. mm or cm.");
    }

    #[test]
    fn a_long_description_is_cut_on_a_word_with_an_ellipsis() {
        let long = "word ".repeat(60);
        let cut = shorten_description(&long, MAX_DESCRIPTION_CHARS);
        assert!(cut.chars().count() <= MAX_DESCRIPTION_CHARS, "{cut}");
        assert!(cut.ends_with("word…"), "{cut}");
        let short = "Short enough.";
        assert_eq!(shorten_description(short, MAX_DESCRIPTION_CHARS), short, "nothing cut, no ellipsis");
    }

    #[test]
    fn a_cut_never_lands_inside_code_or_a_number() {
        // The word boundary before the cap lies inside the code span: the cut backs out of it.
        let text = format!("{} `a b c d e f g h i j k l m n o p` end", "x".repeat(100));
        let cut = shorten_description(&text, MAX_DESCRIPTION_CHARS);
        assert_eq!(cut, format!("{}…", "x".repeat(100)), "{cut}");
        // The cap lands inside the number: the number goes as a whole.
        let text = format!("{} 0.25 tail", "y".repeat(116));
        let cut = shorten_description(&text, MAX_DESCRIPTION_CHARS);
        assert_eq!(cut, format!("{}…", "y".repeat(116)), "{cut}");
    }

    #[test]
    fn a_code_span_longer_than_the_cap_leaves_no_dangling_backtick() {
        let text = format!("`{}` selects the mode", "word ".repeat(40).trim_end());
        let cut = shorten_description(&text, MAX_DESCRIPTION_CHARS);
        assert!(!cut.contains('`'), "{cut}");
        assert!(cut.starts_with("word word") && cut.ends_with("word…"), "{cut}");
        assert!(cut.chars().count() <= MAX_DESCRIPTION_CHARS, "{cut}");
    }

    #[test]
    fn a_full_stop_before_lower_case_or_a_digit_is_not_a_sentence_end() {
        assert_eq!(first_sentence("Steps: 1. cut 2. fill. Then stop."), "Steps: 1. cut 2. fill.");
        assert_eq!(first_sentence("Units: mm, cm, etc. Default mm."), "Units: mm, cm, etc. Default mm.");
        assert_eq!(first_sentence("Region no. 5 of the scan. More."), "Region no. 5 of the scan.");
        assert_eq!(first_sentence("Size, approx. Twice the edge. More."), "Size, approx. Twice the edge.");
        assert_eq!(first_sentence("Angle ≥ 5°. `x` × 2 follows."), "Angle ≥ 5°.");
        // A command id after a full stop joins the sentences: more text is kept, none is lost.
        assert_eq!(first_sentence("Run first. scan.qa reports. Then fit."), "Run first. scan.qa reports.");
    }

    #[test]
    fn cutting_multi_byte_text_is_valid_utf8_at_every_cap() {
        let text = "zażółć gęślą ≥ 5° × 2 jaźń ".repeat(8) + "日本語のテキスト ".repeat(8).as_str();
        let whole = first_sentence(&text);
        for cap in 0..whole.chars().count() + 2 {
            let cut = shorten_description(&text, cap);
            assert!(cut.chars().count() <= cap.max(1), "cap {cap}: {cut}");
            assert!(cut.ends_with('…') || cut == whole, "cap {cap}: {cut}");
            let kept = cut.trim_end_matches('…');
            assert!(whole.starts_with(kept), "cap {cap}: {cut} is not a prefix");
            // A cut backs up by at most one word (the longest here has 8 characters) and its space.
            assert!(kept.chars().count() + 12 > cap.min(whole.chars().count()), "cap {cap}: {cut} lost text");
        }
        let no_space = "ę".repeat(500);
        assert!(shorten_description(&no_space, 120).chars().count() <= 120);
    }

    fn sample_schema() -> Value {
        json!({
            "type": "object",
            "description": "Top-level text. Kept as is.",
            "properties": {
                "unit": {
                    "type": "string",
                    "enum": ["mm", "cm", "m", "in", "ft", "um"],
                    "default": "mm",
                    "description": "Length unit of the file. One of the listed values. More detail follows here."
                },
                "nested": {
                    "type": "object",
                    "properties": {"mode": {"type": "string", "enum": ["adaptive", "uniform"], "description": "How to grid. Second."}},
                    "required": ["mode"]
                }
            },
            "required": ["unit"]
        })
    }

    #[test]
    fn shortening_keeps_enums_defaults_required_and_names() {
        let (short, cuts) = shorten_schema(&sample_schema());
        assert_eq!(cuts, 2);
        assert_eq!(short["properties"]["unit"]["enum"], json!(["mm", "cm", "m", "in", "ft", "um"]));
        assert_eq!(short["properties"]["unit"]["default"], "mm");
        assert_eq!(short["properties"]["unit"]["description"], "Length unit of the file.");
        assert_eq!(short["properties"]["nested"]["properties"]["mode"]["enum"], json!(["adaptive", "uniform"]));
        assert_eq!(short["properties"]["nested"]["properties"]["mode"]["description"], "How to grid.");
        assert_eq!(short["required"], json!(["unit"]));
        assert_eq!(short["properties"]["nested"]["required"], json!(["mode"]));
        assert_eq!(short["description"], "Top-level text. Kept as is.");
    }

    #[test]
    fn the_default_listing_keeps_enums_required_and_says_where_the_docs_are() {
        let mut s = default_server();
        let compact = listed_tools(&mut s);
        let mut full = server();
        let full = listed_tools(&mut full);
        for id in FEATURED {
            let name = tool_name(id);
            let (c, f) = (
                compact.iter().find(|t| t["name"] == name.as_str()).expect("compact tool"),
                full.iter().find(|t| t["name"] == name.as_str()).expect("full tool"),
            );
            assert_eq!(c["inputSchema"]["required"], f["inputSchema"]["required"], "{id}");
            let keys = |t: &Value| -> Vec<String> {
                t["inputSchema"]["properties"].as_object().map(|m| m.keys().cloned().collect()).unwrap_or_default()
            };
            assert_eq!(keys(c), keys(f), "{id}");
            for (prop, spec) in f["inputSchema"]["properties"].as_object().into_iter().flatten() {
                assert_eq!(c["inputSchema"]["properties"][prop]["enum"], spec["enum"], "{id}.{prop}");
                assert_eq!(c["inputSchema"]["properties"][prop]["default"], spec["default"], "{id}.{prop}");
            }
            let without_queued = |t: &Value| {
                let mut schema = t["inputSchema"].clone();
                if let Some(props) = schema["properties"].as_object_mut() {
                    props.remove("queued");
                }
                schema
            };
            let cut = without_queued(c) != without_queued(f);
            let description = c["description"].as_str().unwrap_or_default();
            if cut {
                assert!(
                    description.contains("describe_command"),
                    "{id} is shortened but does not say where the docs are"
                );
                assert!(description.contains(*id), "{id}: the note names the command id");
            }
        }
        assert!(compact.iter().any(|t| t["description"].as_str().is_some_and(|d| d.contains("describe_command"))));
    }

    #[test]
    fn describe_command_and_parameter_errors_stay_unshortened() {
        let mut s = default_server();
        let full_schema = (find("scan.to_solid").expect("command").input)();
        let mut long = Vec::new();
        property_descriptions(&full_schema, &mut long);
        let longest = long.iter().max_by_key(|d| d.chars().count()).cloned().unwrap_or_default();
        assert!(longest.chars().count() > MAX_DESCRIPTION_CHARS, "scan.to_solid has a long description to protect");

        let listed = listed_tools(&mut s);
        let tool = listed.iter().find(|t| t["name"] == "scan_to_solid").expect("listed");
        let mut shortened = Vec::new();
        property_descriptions(&tool["inputSchema"], &mut shortened);
        assert!(!shortened.contains(&longest), "the listing carries the long description");

        let described = call(&mut s, "describe_command", json!({"id": "scan.to_solid"}));
        assert!(
            structured(&described).to_string().contains(&serde_json::to_string(&longest).unwrap_or_default()[1..20])
        );
        let mut in_described = Vec::new();
        property_descriptions(&structured(&described)["input_schema"], &mut in_described);
        assert!(in_described.contains(&longest), "{described}");

        let err = call(&mut s, "scan_to_solid", json!({"nonsense": 1}));
        let mut in_error = Vec::new();
        property_descriptions(&structured(&err)["input_schema"], &mut in_error);
        assert!(in_error.contains(&longest), "the parameter error carries the full schema: {err}");
    }

    #[test]
    fn area_verb_lines_are_first_sentences_and_the_full_summary_is_one_call_away() {
        let mut s = default_server();
        let tools = listed_tools(&mut s);
        let area = tools.iter().find(|t| t["name"] == "scan").expect("scan area");
        let text = area["description"].as_str().unwrap_or_default();
        let spec = find("scan.qa").expect("command");
        let line = text.lines().find(|l| l.starts_with("- qa:")).expect("verb line");
        assert!(line.chars().count() <= "- qa: ".len() + crate::shorten::MAX_SUMMARY_CHARS, "{line}");
        assert!(text.contains("describe_command"), "{text}");
        let described = call(&mut s, "describe_command", json!({"id": "scan.qa"}));
        assert_eq!(structured(&described)["command"]["summary"], spec.summary, "{described}");
        assert!(tools.iter().all(|t| {
            t["inputSchema"]["properties"]["queued"]
                .get("description")
                .is_none_or(|d| d.as_str().is_some_and(|d| d.chars().count() < 80))
        }));
    }

    #[test]
    fn the_default_listing_has_headroom_and_is_deterministic() {
        let mut s = default_server();
        let first = listing_text(&mut s);
        assert_eq!(first, listing_text(&mut s), "same server, same bytes");
        assert_eq!(first, listing_text(&mut default_server()), "new server, same bytes");
        let size = first.len();
        assert!(size <= MAX_DEFAULT_LISTING, "the default listing is {size} bytes");
        assert!(
            MAX_DEFAULT_LISTING - size >= MIN_HEADROOM,
            "only {} bytes of headroom left (limit {MAX_DEFAULT_LISTING}, listing {size}): shorten descriptions \
             structurally in crates/mcp, do not hand-trim command docs",
            MAX_DEFAULT_LISTING - size
        );
    }

    #[test]
    fn the_full_listing_carries_every_command_unshortened() {
        let mut full = server();
        let tools = listed_tools(&mut full);
        for c in registry() {
            let name = tool_name(c.id);
            let tool = tools.iter().find(|t| t["name"] == name.as_str()).unwrap_or_else(|| panic!("{} missing", c.id));
            assert_eq!(tool["inputSchema"], crate::listing::queueable((c.input)()), "{}: input schema", c.id);
            assert_eq!(tool["description"], format!("{} ({})", c.summary, c.id), "{}: description", c.id);
            assert_eq!(tool["outputSchema"], (c.output)(), "{}: output schema", c.id);
        }
        let text = listing_text(&mut full);
        assert!(!text.contains("Parameter notes shortened"), "a full-mode tool says it was shortened");
        assert!(!text.contains(crate::listing::QUEUED_NOTE), "full mode carries the short `queued` text");
    }

    #[test]
    #[ignore = "prints the size table for the pull request"]
    fn print_the_size_table() {
        let mut full = server();
        let mut compact = default_server();
        let (f, c) = (listing_text(&mut full), listing_text(&mut compact));
        println!("TOTAL default={} full={}", c.len(), f.len());
        for t in listed_tools(&mut compact) {
            println!(
                "TOOL {} {} bytes, description {} bytes, schema {} bytes",
                t["name"],
                t.to_string().len(),
                t["description"].to_string().len(),
                t["inputSchema"].to_string().len()
            );
        }
        for id in FEATURED {
            let spec = find(id).expect("featured");
            let before = crate::listing::command_tool_unshortened(spec).to_string().len();
            let tools = listed_tools(&mut compact);
            let tool = tools.iter().find(|t| t["name"] == tool_name(id).as_str()).expect("listed");
            let mut after = Vec::new();
            property_descriptions(&tool["inputSchema"], &mut after);
            let mut original = Vec::new();
            property_descriptions(&(spec.input)(), &mut original);
            let cut = original.iter().filter(|d| !after.contains(d)).count();
            let longest = after.iter().map(|d| d.chars().count()).max().unwrap_or(0);
            let longest_before = original.iter().map(|d| d.chars().count()).max().unwrap_or(0);
            println!(
                "{id}: {before} -> {} bytes, {} properties, {cut} cut, longest {longest_before} -> {longest}, description {} chars",
                tool.to_string().len(),
                original.len(),
                tool["description"].as_str().map_or(0, |d| d.chars().count()),
            );
        }
    }
}
