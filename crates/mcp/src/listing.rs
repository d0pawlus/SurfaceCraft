//! `tools/list`: the entries of the general tools, of one command and of one area.
use crate::{OWN, area_commands, tool_name, verb_of, verbs};
use serde_json::{Value, json};
use surfacecraft_engine::CommandSpec;

/// The input schema with the `queued` flag added (the server takes it off before the call runs).
pub(crate) fn queueable(mut schema: Value) -> Value {
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
pub(crate) fn general(name: &str, description: &str, schema: Value) -> Value {
    let schema = if OWN.contains(&name) { schema } else { queueable(schema) };
    json!({"name": name, "description": description, "inputSchema": schema})
}

/// The general tools: the MCP-level ones, listed in both modes.
pub(crate) fn general_tools() -> Vec<Value> {
    let obj = |props: Value, req: &[&str]| json!({"type": "object", "properties": props, "required": req});
    let job = || json!({"type": "integer", "minimum": 1, "description": "The job id a queued call returned."});
    vec![
        general(
            "list_commands",
            "List SurfaceCraft commands (optional `filter`). The short path is mesh_import, scan_to_solid, export_step; guide_read has a guide per task.",
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
            "Choose the tool listing from now on: `mode` compact (one tool per area, with verb and params, plus the workflow commands; the default) or full (one tool per command); `areas` limits either to some areas (null or [] for all). Keeps what is not given. Returns the new listing; call tools/list again to read the schemas (no list-changed notification is sent).",
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

/// One command's listing entry. With `output`, it carries the output schema too (full mode);
/// a featured command of the compact listing does without it.
pub(crate) fn command_tool(c: &CommandSpec, output: bool) -> Value {
    let mut tool = json!({
        "name": tool_name(c.id),
        "title": c.label,
        "description": format!("{} ({})", c.summary, c.id),
        "inputSchema": queueable((c.input)()),
        "annotations": {"readOnlyHint": !c.undoable},
    });
    if output && let Some(obj) = tool.as_object_mut() {
        obj.insert("outputSchema".into(), (c.output)());
    }
    tool
}

/// One area's listing entry (compact mode): `verb` and `params`.
pub(crate) fn area_tool_spec(area: &str) -> Value {
    let commands = area_commands(area);
    let lines: Vec<String> = commands.iter().map(|c| format!("- {}: {}", verb_of(c.id), c.summary)).collect();
    let verb_names = verbs(area);
    json!({
        "name": area,
        "title": format!("{area} commands"),
        "description": format!(
            "Commands of the `{area}` area. `verb` picks one, `params` are its parameters (describe_command gives the schema; a wrong parameter returns it too).\n{}",
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
