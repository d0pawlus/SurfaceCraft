---
name: surfacecraft-agent-tools
description: Driving SurfaceCraft from an agent — MCP server, bridged mode, CLI scripts, the AI assistant.
---

# Agent tools

- `surfacecraft-cli mcp` speaks MCP (JSON-RPC 2.0, one message per line) on stdio. Every engine command is a tool
  (`area.verb` → `area_verb`) with its JSON Schemas; generic tools: `list_commands`, `describe_command`,
  `execute`, `batch` (rolls back on failure), `undo`, `redo`, `inspect_design`, `screenshot`.
- When the desktop app runs, `mcp` bridges to it (localhost control channel, session token in
  `<temp>/surfacecraft-control.json`), so the agent changes what the user sees. `--local` forces a headless session;
  `--allow-files` lets path parameters touch the file system (off by default — pass base64 `data`).
- Scripts: `surfacecraft-cli run script.json` with `{"commands": [{"command": ..., "params": ...}]}`; the GUI log
  exports the same format ("Copy as script").
- The AI assistant (`surfacecraft-cli assist align|workflow "<goal>"`, GUI panel "Assistant") calls the program in
  `SURFACECRAFT_AI_COMMAND`: request JSON (`system`, `messages`, `tools`, `image_png_base64`) on stdin, response JSON
  (`text`, `calls: [{name, arguments}]`) on stdout. The choice of LLM service and key is the user's.
