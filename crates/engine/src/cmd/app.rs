//! `app.*`: introspection of the command set (used by the CLI, the palette and MCP), batches of
//! commands; `guide.*`: the task guides, embedded at compile time.
use super::{CmdError, CmdResult, CommandSpec, find, registry};
use crate::{Session, command};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::Value;

/// Parameters of `app.commands`.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ListIn {
    /// Only ids or labels containing this text (case-insensitive).
    #[serde(default)]
    pub filter: Option<String>,
}

/// One command in a listing.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct CommandInfo {
    /// Id.
    pub id: String,
    /// Label.
    pub label: String,
    /// Group.
    pub group: String,
    /// Summary.
    pub summary: String,
    /// Changes the document.
    pub undoable: bool,
}

/// Result of `app.commands`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct ListOut {
    /// Matching commands.
    pub commands: Vec<CommandInfo>,
}

fn info(c: &CommandSpec) -> CommandInfo {
    CommandInfo {
        id: c.id.into(),
        label: c.label.into(),
        group: c.group.into(),
        summary: c.summary.into(),
        undoable: c.undoable,
    }
}

fn list(_s: &mut Session, i: ListIn) -> CmdResult<ListOut> {
    let f = i.filter.map(|f| f.to_lowercase());
    let commands = registry()
        .iter()
        .filter(|c| f.as_ref().is_none_or(|f| c.id.contains(f.as_str()) || c.label.to_lowercase().contains(f.as_str())))
        .map(info)
        .collect();
    Ok(ListOut { commands })
}

/// Parameters of `app.describe`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct DescribeIn {
    /// Command id.
    pub id: String,
}

/// Result of `app.describe`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct DescribeOut {
    /// The command.
    pub command: CommandInfo,
    /// JSON Schema of its parameters.
    pub input_schema: Value,
    /// JSON Schema of its result.
    pub output_schema: Value,
}

fn describe(_s: &mut Session, i: DescribeIn) -> CmdResult<DescribeOut> {
    let c = find(&i.id).ok_or_else(|| CmdError::Params(format!("unknown command `{}`", i.id)))?;
    Ok(DescribeOut { command: info(c), input_schema: (c.input)(), output_schema: (c.output)() })
}

/// The guides, embedded at compile time (the web build and sandboxed hosts have no files).
const GUIDES: &[(&str, &str)] = &[
    ("align-scan", include_str!("../../../../docs/guides/align-scan.md")),
    ("extract-primitives", include_str!("../../../../docs/guides/extract-primitives.md")),
    ("fit-surfaces", include_str!("../../../../docs/guides/fit-surfaces.md")),
    ("build-solid", include_str!("../../../../docs/guides/build-solid.md")),
    ("sketches-features", include_str!("../../../../docs/guides/sketches-features.md")),
    ("check-deviation", include_str!("../../../../docs/guides/check-deviation.md")),
    ("export", include_str!("../../../../docs/guides/export.md")),
];

/// Parameters of `guide.read`.
#[derive(Clone, Debug, Default, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct GuideIn {
    /// Guide id (`align-scan`, `extract-primitives`, …); omitted: list the guides.
    #[serde(default)]
    pub topic: Option<String>,
}

/// One guide.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct GuideInfo {
    /// Id: the `topic` to ask for.
    pub id: String,
    /// Title (the first heading).
    pub title: String,
    /// Section headings, in order.
    pub sections: Vec<String>,
    /// The Markdown text (only when a `topic` was given).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub markdown: Option<String>,
}

/// Result of `guide.read`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct GuideOut {
    /// The guides listed, or the one asked for.
    pub guides: Vec<GuideInfo>,
}

/// Title and section headings of a guide (headings inside code fences are not sections).
fn guide_info(id: &str, markdown: &str) -> GuideInfo {
    let mut title = String::new();
    let mut sections = Vec::new();
    let mut fenced = false;
    for line in markdown.lines() {
        if line.trim_start().starts_with("```") {
            fenced = !fenced;
        } else if fenced {
            continue;
        } else if let Some(t) = line.strip_prefix("# ") {
            if title.is_empty() {
                title = t.trim().to_string();
            }
        } else if let Some(s) = line.strip_prefix("## ") {
            sections.push(s.trim().to_string());
        }
    }
    GuideInfo { id: id.to_string(), title, sections, markdown: None }
}

fn read_guide(_s: &mut Session, i: GuideIn) -> CmdResult<GuideOut> {
    let Some(topic) = i.topic else {
        return Ok(GuideOut { guides: GUIDES.iter().map(|(id, md)| guide_info(id, md)).collect() });
    };
    let Some((id, md)) = GUIDES.iter().find(|(id, _)| *id == topic.as_str()) else {
        let ids: Vec<&str> = GUIDES.iter().map(|(id, _)| *id).collect();
        return Err(CmdError::Params(format!("no guide `{topic}`; guides: {}", ids.join(", "))));
    };
    let mut info = guide_info(id, md);
    info.markdown = Some((*md).to_string());
    Ok(GuideOut { guides: vec![info] })
}

/// Most steps one `app.batch` call runs.
pub const MAX_BATCH_STEPS: usize = 256;

/// One step of `app.batch`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BatchStep {
    /// Command id, e.g. `body.box`.
    pub command: String,
    /// Its parameters (omitted: none).
    #[serde(default)]
    pub params: Value,
}

/// Parameters of `app.batch`.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct BatchIn {
    /// The steps, in order.
    pub commands: Vec<BatchStep>,
    /// Keep going after a failed step (default false: stop at the first failure).
    #[serde(default)]
    pub continue_on_error: bool,
}

/// The outcome of one step.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct BatchStepOut {
    /// Command id.
    pub command: String,
    /// Succeeded?
    pub ok: bool,
    /// The command's result (on success).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub result: Option<Value>,
    /// The error (on failure). A failed step changes nothing.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub error: Option<Value>,
}

/// Result of `app.batch`.
#[derive(Clone, Debug, Serialize, Deserialize, JsonSchema)]
pub struct BatchOut {
    /// Every step succeeded.
    pub ok: bool,
    /// Steps that succeeded.
    pub succeeded: usize,
    /// The steps run, in order.
    pub steps: Vec<BatchStepOut>,
    /// Steps not run because an earlier step failed.
    pub not_run: usize,
}

/// Runs the steps in order. Each step is its own undo step; steps before a failure stay applied.
/// The batch is checked first: an unknown command or a nested `app.batch` runs nothing.
fn batch(s: &mut Session, i: BatchIn) -> CmdResult<BatchOut> {
    if i.commands.len() > MAX_BATCH_STEPS {
        return Err(CmdError::Params(format!("a batch has at most {MAX_BATCH_STEPS} steps")));
    }
    for (k, st) in i.commands.iter().enumerate() {
        if st.command == "app.batch" {
            return Err(CmdError::Params(format!("step {k}: app.batch cannot run inside app.batch")));
        }
        if find(&st.command).is_none() {
            return Err(CmdError::Params(format!("step {k}: unknown command `{}`", st.command)));
        }
    }
    let total = i.commands.len();
    let mut steps = Vec::with_capacity(total);
    let mut ok = true;
    for st in i.commands {
        match s.execute(&st.command, st.params) {
            Ok(v) => steps.push(BatchStepOut { command: st.command, ok: true, result: Some(v), error: None }),
            Err(e) => {
                ok = false;
                let error = serde_json::to_value(&e).unwrap_or(Value::Null);
                steps.push(BatchStepOut { command: st.command, ok: false, result: None, error: Some(error) });
                if !i.continue_on_error {
                    break;
                }
            }
        }
    }
    let succeeded = steps.iter().filter(|x| x.ok).count();
    let not_run = total - steps.len();
    Ok(BatchOut { ok, succeeded, steps, not_run })
}

/// `app.*` and `guide.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
    command!(
        "app.batch",
        "Run commands",
        "App",
        false,
        "Runs `commands` ([{command, params}]) in order, each as its own undo step; stops at the first failure unless `continue_on_error`. Steps before a failure stay applied. Cannot contain app.batch.",
        BatchIn,
        BatchOut,
        batch
    ),
    command!(
        "guide.read",
        "Read guide",
        "App",
        false,
        "Lists the guides (id, title, section headings); with `topic` returns that guide as Markdown.",
        GuideIn,
        GuideOut,
        read_guide
    ),
    command!(
        "app.commands",
        "List commands",
        "App",
        false,
        "Lists commands (optional `filter`).",
        ListIn,
        ListOut,
        list
    ),
    command!(
        "app.describe",
        "Describe command",
        "App",
        false,
        "Input and output JSON Schemas of command `id`.",
        DescribeIn,
        DescribeOut,
        describe
    ),
];

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn object_count(s: &mut Session) -> usize {
        s.execute("doc.list", Value::Null).ok().and_then(|v| v["objects"].as_array().map(Vec::len)).unwrap_or(0)
    }

    #[test]
    fn guides_list_titles_and_sections_and_return_markdown() {
        let mut s = Session::new();
        let all = read_guide(&mut s, GuideIn::default()).expect("list");
        assert_eq!(all.guides.len(), GUIDES.len());
        for g in &all.guides {
            assert!(!g.title.is_empty() && !g.sections.is_empty(), "{} has no title or sections", g.id);
            assert!(g.markdown.is_none());
        }
        let one = read_guide(&mut s, GuideIn { topic: Some("align-scan".into()) }).expect("topic");
        assert!(one.guides[0].markdown.as_deref().is_some_and(|m| m.contains("align.icp")));
        assert!(read_guide(&mut s, GuideIn { topic: Some("nope".into()) }).is_err());
    }

    /// Every backticked `area.verb` in a guide is a registered command, so the guides cannot
    /// drift from the registry.
    #[test]
    fn guides_name_only_registered_commands() {
        let areas: Vec<&str> = registry().iter().filter_map(|c| c.id.split('.').next()).collect();
        for (id, md) in GUIDES {
            for span in md.split('`').skip(1).step_by(2) {
                let prefix = span.split('.').next().unwrap_or(span);
                if span.contains('.') && !span.contains(' ') && areas.contains(&prefix) {
                    assert!(find(span).is_some(), "{id}: `{span}` is not a command");
                }
            }
        }
    }

    #[test]
    fn batch_stops_at_the_first_failure_and_each_step_is_an_undo_step() {
        let mut s = Session::new();
        let commands = |go_on: bool| {
            json!({"commands": [
                {"command": "body.box", "params": {"size": [1, 1, 1], "name": "a"}},
                {"command": "body.fillet", "params": {"body": "missing", "edges": [0], "size": 1}},
                {"command": "body.box", "params": {"size": [2, 2, 2], "name": "b"}}
            ], "continue_on_error": go_on})
        };
        let out = s.execute("app.batch", commands(false)).expect("batch runs");
        assert_eq!(out["ok"], false);
        assert_eq!(out["succeeded"], 1);
        assert_eq!(out["steps"].as_array().map(Vec::len), Some(2));
        assert_eq!(out["not_run"], 1);
        assert_eq!(object_count(&mut s), 1, "the first step stays, the failed one changed nothing");
        s.execute("edit.undo", Value::Null).expect("undo");
        assert_eq!(object_count(&mut s), 0, "the first step was its own undo step");

        let out = s.execute("app.batch", commands(true)).expect("batch runs");
        assert_eq!(out["steps"].as_array().map(Vec::len), Some(3));
        assert_eq!(out["not_run"], 0);
        assert_eq!(object_count(&mut s), 2);
    }

    #[test]
    fn batch_refuses_nesting_and_unknown_commands_before_running_anything() {
        let mut s = Session::new();
        let nested = json!({"commands": [
            {"command": "body.box", "params": {"size": [1, 1, 1]}},
            {"command": "app.batch", "params": {"commands": []}}
        ]});
        assert!(matches!(s.execute("app.batch", nested), Err(crate::EngineError::InvalidParams(_))));
        let unknown =
            json!({"commands": [{"command": "body.box", "params": {"size": [1, 1, 1]}}, {"command": "no.such"}]});
        assert!(s.execute("app.batch", unknown).is_err());
        assert_eq!(object_count(&mut s), 0, "a refused batch runs no step");
        let many = json!({"commands": vec![json!({"command": "app.commands"}); MAX_BATCH_STEPS + 1]});
        assert!(s.execute("app.batch", many).is_err());
    }
}
