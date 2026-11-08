# SurfaceCraft: instructions for agents

SurfaceCraft is a clean-room, pure-Rust scan-to-CAD application (mesh alignment, primitive
extraction, freeform surface fitting, parametric feature tree) on the solidcore B-rep kernel, with
one command-driven core behind four front ends: desktop, web, CLI and MCP. This file is a router:
the hard rules, then the skill to read for your task. Design: `docs/superpowers/specs/`;
current state and next step: `docs/STATUS.md`.

## Hard rules (what enforces each)
- Independent: never copy code, icons, text, layouts or data from QuickSurface, Geomagic Design X
  or any closed product; their scans stay local (`SURFACECRAFT_LOCAL_CORPUS`, tests off by default):
  `skills/surfacecraft-ground-rules/SKILL.md`, `cargo xtask assets`.
- No GPL/LGPL/AGPL dependencies or code: `cargo xtask licences --check`.
- Every non-code file has a row in ATTRIBUTION.md: `cargo xtask assets`.
- Never crash: no unwrap/expect/panic/todo/unimplemented/unreachable/unsafe in non-test code;
  input (files, JSON, MCP parameters) is hostile: clippy `-D warnings` with the crate-root deny
  list, `hostile_params_never_panic`, `hostile_bytes_never_panic`.
- Layering (geom → kernel/fit/align/sketch → deviation/doc → io → engine → ui-egui/mcp → apps);
  only `crates/kernel` depends on solidcore; UI crates only from the UI layer: `cargo xtask layers`.
- Mesh computation stays here; B-rep/NURBS construction and evaluation go through solidcore
  (design doc §5); app-side geometry beyond that is listed in `xtask/app-geometry.txt` (shrink-only):
  `cargo xtask app-geometry`.
- Family code: solidcore first, then upstream, then adaptation with attribution (design doc §9);
  adaptations are listed in ATTRIBUTION.md "Adapted code" and in docs/STATUS.md debt.
- Everything is a command (`area.verb`, input and output JSON Schema) and programmatic calls never
  open dialogs: `skills/surfacecraft-commands/SKILL.md`.
- Application code is Rust only and builds for wasm: `cargo xtask wasm`.
- Thresholds of design doc §11.1 are never loosened to make a test pass.
- Code, comments, commit messages and docs in the code are in English.
- Before every commit run `cargo xtask quick <changed packages>` (fmt, assets, licences, layers,
  app-geometry, clippy and tests of those packages); the full `cargo xtask ci` runs on GitHub
  Actions for every pull request, and a pull request is merged only with all checks green.

## Working on X: read the skill first
| Working on | Read |
|---|---|
| Licences, assets, clean-room, test data | `skills/surfacecraft-ground-rules/SKILL.md` |
| Non-test Rust, hostile input | `skills/surfacecraft-crash-safety/SKILL.md` |
| A command, a crate boundary, the CLI, MCP | `skills/surfacecraft-commands/SKILL.md` |
| MCP, bridged mode, scripts, the AI assistant | `skills/surfacecraft-agent-tools/SKILL.md` |
| Writing or running tests, accuracy gates | `skills/surfacecraft-tests/SKILL.md` |
