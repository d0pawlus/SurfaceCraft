---
name: surfacecraft-agent-tools
description: Driving SurfaceCraft from an agent — MCP server, bridged mode, CLI scripts, the AI assistant.
---

# Agent tools

- `surfacecraft-cli mcp` speaks MCP (JSON-RPC 2.0, one message per line) on stdio. Every engine command is
  callable as a tool (`area.verb` → `area_verb`); generic tools: `list_commands`, `describe_command`,
  `execute`, `batch` (rolls back on failure), `undo`, `redo`, `inspect_design`, `screenshot`, `set_tool_mode`,
  `job_status`, `job_wait`, `job_cancel` (a call with `queued: true` returns a job id).
- The short path is three calls: `mesh_import`, `scan_to_solid`, `export_step`. `scan.to_solid` returns
  `verdict` (`pass`, `review`, `fail`), the deviation against the scan, `issues` and `next_steps` with ready
  parameters; a poor model is a verdict, not an error. Guide: `guide_read` topic `scan-to-solid`.
- `scan.to_solid` has two methods (`method`: `auto`, `primitives`, `freeform`). Organic parts, castings and
  body panels go through `freeform` (`surface.wrap`: a watertight patch network over the whole scan); judge
  scan-to-CAD work on such scans first, they are what the product is for.
- The default listing is compact: one tool per area (`verb`, `params`) plus the commands of `FEATURED`
  (`crates/mcp/src/lib.rs`) with their input schema. `--tools full` (or `SURFACECRAFT_MCP_TOOLS=full`) lists one
  tool per command with output schemas (about 380 KB). A command that an agent needs on the short path
  belongs in `FEATURED`; keep the default listing under the size the test `the_default_listing_is_small…` holds.
  The compact listing shortens prose structurally (`crates/mcp/src/shorten.rs`): property descriptions of the
  featured schemas and the verb lines of the area tools shrink to their first sentence (capped), enums,
  names and `required` stay, and `describe_command` / parameter errors carry the full text. Never hand-trim
  command docs to fit the budget; when the headroom test fails, shorten structurally in `crates/mcp`.
  Write a parameter's first sentence so that it stands alone: what it is, its unit, range and default
  (the first sentence, at most 120 characters, is all an agent sees before its first call).
- Results are lean (`crates/mcp/src/lean.rs`): compact JSON text, floats rounded to 6 decimals. A parameter
  error carries the command's `input_schema`; an unknown command, verb or tool carries `did_you_mean`.
- Prompt `scan_to_solid` (`prompts/get`), resources `surfacecraft://design`, `surfacecraft://commands` and
  `surfacecraft://guide/<id>`.
- When the desktop app runs, `mcp` bridges to it (localhost control channel, session token in
  `<temp>/surfacecraft-control.json`), so the agent changes what the user sees. `--local` forces a headless session;
  `--allow-files` lets path parameters touch the file system (off by default — pass base64 `data`).
- Scripts: `surfacecraft-cli run script.json` with `{"commands": [{"command": ..., "params": ...}]}`; the GUI log
  exports the same format ("Copy as script").
- The AI assistant (`surfacecraft-cli assist align|workflow "<goal>"`, GUI panel "Assistant") calls the program in
  `SURFACECRAFT_AI_COMMAND`: request JSON (`system`, `messages`, `tools`, `image_png_base64`) on stdin, response JSON
  (`text`, `calls: [{name, arguments}]`) on stdout. The choice of LLM service and key is the user's.
