//! `app.*`: introspection of the command set (used by the CLI, the palette and MCP).
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

/// `app.*` commands.
pub static COMMANDS: &[CommandSpec] = &[
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
