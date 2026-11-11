---
name: surfacecraft-commands
description: Adding or changing a SurfaceCraft command, crate boundaries, the CLI and MCP.
---

# Commands

- A command lives in `crates/engine/src/cmd/<area>.rs` and is declared with `command!(id, label,
  group, undoable, summary, Input, Output, handler)`. `Input` derives `Deserialize + JsonSchema`
  with `#[serde(deny_unknown_fields)]`; `Output` derives `Serialize + JsonSchema`. The registry is
  sorted by id; ids are `area.verb` in lowercase.
- A handler never opens a dialog, never blocks on user input, and touches files only through
  `cmd::read_file` / `cmd::write_file` (refused when `Session::allow_files` is false, e.g. web, MCP
  sandbox). Anything that takes a path also accepts inline base64 data or returns it.
- Undoable commands are snapshotted by the engine and rolled back on error; do not snapshot yourself.
- Objects are referenced by `ObjRef` (numeric id or name).
- Front ends: the desktop/web UI calls `Session::execute` (toolbar, palette, menus); the CLI runs
  `{"commands": [...]}` scripts (`surfacecraft-cli run`) or single commands (`exec`); the MCP server
  (`surfacecraft-cli mcp`) exposes the same registry as tools. A new command needs no front-end code.
- Layers (checked by `cargo xtask layers`): L0 geom; L1 kernel, fit, align, sketch; L2 deviation,
  doc; L3 io; L4 engine; L5 ui-egui, mcp; apps. A crate depends only on lower layers (or earlier
  crates of its own layer, listed in `INTRA_LAYER_ORDER`). Only `kernel` uses solidcore.
- The one exception to "every command is logged" is `Session::query` (`crates/engine/src/session.rs`).
  It runs a read-only command on a shared `&Session` without a log entry or a history step, so that
  the desktop UI can poll a value on every frame (today only `deviation.at`, for the viewport
  readout). The ids it accepts are a private list in `session.rs`; a command joins it only if it is
  declared with `read_command!` (it has a `read` form), is not undoable and is not heavy. It is
  UI-only: the MCP server, the CLI and scripts reach commands through `execute`, which logs them,
  and replay of the log is unaffected because queries are never in it.
