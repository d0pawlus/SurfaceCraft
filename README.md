<h1 align="center">SurfaceCraft</h1>

<p align="center">
  <b>Scan to CAD: an open-source, clean-room reverse-engineering application, built in pure Rust.</b>
</p>

<p align="center">
  Turn a raw scanner mesh into clean, editable CAD: align it, extract primitives, fit freeform
  NURBS surfaces and rebuild a parametric feature tree, then export STEP or IGES.<br>
  Runs natively on Windows, macOS and Linux, and in the browser via WebAssembly.
</p>

<p align="center">
  <img alt="Written in Rust" src="https://img.shields.io/badge/written%20in-Rust-b7410e?style=flat-square&logo=rust&logoColor=white">
  <img alt="Runs on Windows, macOS, Linux and the web" src="https://img.shields.io/badge/runs%20on-Windows%20%C2%B7%20macOS%20%C2%B7%20Linux%20%C2%B7%20Web-d9622b?style=flat-square">
  <img alt="License: MIT OR Apache-2.0" src="https://img.shields.io/badge/license-MIT%20%2F%20Apache--2.0-b7410e?style=flat-square">
  <img alt="Agent-drivable over MCP" src="https://img.shields.io/badge/agents-MCP-d9622b?style=flat-square">
  <img alt="B-rep kernel: solidcore" src="https://img.shields.io/badge/kernel-solidcore-b7410e?style=flat-square">
</p>

<p align="center">
  <a href="#why-surfacecraft">Why SurfaceCraft</a> ·
  <a href="#the-pipeline">The pipeline</a> ·
  <a href="#quick-start">Quick start</a> ·
  <a href="#for-agents-mcp-cli-and-scripts">For agents</a> ·
  <a href="#measured-not-promised">Measured accuracy</a> ·
  <a href="#architecture">Architecture</a> ·
  <a href="#roadmap">Roadmap</a> ·
  <a href="#license-and-credits">License and credits</a>
</p>

## Why SurfaceCraft

- **The whole workflow in one place.** Mesh import and cleanup, alignment, region segmentation,
  primitive and freeform fitting, sketches with a constraint solver, solid features and CAD
  export. No round trips between a mesh tool and a CAD package.
- **A real B-rep kernel.** Surfaces and solids are built and evaluated by the open
  [solidcore](https://github.com/tangency-labs/solidcore) kernel, so what you export is a valid
  STEP/IGES body, not a triangle soup in disguise.
- **Measured, not promised.** Every accuracy claim below comes from a test with a fixed threshold.
  Thresholds are never loosened to make a test pass.
- **Agent-native.** Every action is a command (`area.verb`) with JSON Schema input and output.
  The desktop UI, the CLI, scripts, the MCP server and the built-in AI assistant all drive the
  same registry, so anything you can click, an agent can do.
- **Never crashes on bad input.** Files, JSON and MCP parameters are treated as hostile: no
  `unwrap`, `panic` or `unsafe` in application code, with fuzz tests on every importer and every
  command.
- **Everywhere, and yours.** One Rust codebase for desktop and the web. No subscription, no
  licence server, no telemetry.

## The pipeline

| Stage | What you get |
|---|---|
| **Import and clean** | STL, OBJ and PLY; hole filling, spike and debris removal, QEM decimation, Taubin smoothing, noise estimation. |
| **Align** | 3-2-1 datums, N-point (Kabsch), point-to-plane ICP, alignment to world axes. |
| **Segment and fit primitives** | Automatic region segmentation; plane, cylinder, cone, sphere and torus fits with automatic type detection; symmetry planes; persistent, editable regions. |
| **Freeform surfaces** | Fit Surface (one NURBS patch per region) and AutoSurface (quad layout to a continuous patch network), snap, crease, merge, thicken to a solid. |
| **Parametric modelling** | 2-D sketches with a constraint solver (fully/over-defined detection), sections from the mesh, extrude, revolve, fillet, chamfer, shell, pattern, trim, cut and booleans in a history DAG with undo, redo and suppress. Radius suggestions read from the scan. |
| **Check and export** | Colour deviation maps against the scan; STEP (surfaces and solids) and IGES. |

## Quick start

```sh
cargo run --release -p surfacecraft                                          # desktop app (egui + wgpu)
cargo run --release -p surfacecraft-cli -- run examples/scripts/m0-smoke.json  # run a command script headless
cargo run --release -p surfacecraft-cli -- commands                          # list every command
cargo run --release -p surfacecraft-cli -- describe mesh.import              # its input and output schema
cargo xtask ci                                                               # fmt, assets, licences, layers, clippy, wasm, tests
```

The [`examples/scripts`](examples/scripts) folder walks the whole pipeline, one script per stage:
alignment, primitives, Fit Surface, AutoSurface and parametric modelling.

### Web

```sh
cargo xtask web      # → apps/surfacecraft-web/dist (serve it with any static server)
```

You need the `wasm32-unknown-unknown` target and `wasm-bindgen` on your `PATH`. The browser build
renders with WebGPU, falling back to WebGL.

## For agents: MCP, CLI and scripts

```sh
cargo run --release -p surfacecraft-cli -- mcp                 # MCP over stdio
cargo run --release -p surfacecraft-cli -- exec region.auto_segment '{…}'  # one command, JSON in, JSON out
cargo run --release -p surfacecraft-cli -- assist align "put the part on the world axes"
```

- **MCP server.** Scan to STEP in three calls: `mesh_import`, `scan_to_solid`, `export_step`.
  `scan_to_solid` cleans, segments, fits, builds the solid and measures it against the scan; it
  answers with a verdict (`pass`, `review`, `fail`), the deviation and the next steps to take.
  Every registry command is callable as a tool. The default listing is compact (42 KB: one tool
  per command area plus the workflow commands); `--tools full` lists one tool per command.
  There are general tools to list and describe commands, run batches (rolled back on the first
  failure), undo and redo, inspect the design and take viewport screenshots. When the desktop
  app is running, the server **bridges to it**, so an agent works in the same session you are
  looking at; otherwise it runs headless (`--local`, files only with `--allow-files`).
  Add it to an MCP client as the command `surfacecraft-cli mcp --allow-files`.
- **Scripts.** A script is a list of commands:
  `{"commands": [{"command": "mesh.import", "params": {"path": "scan.stl"}}, …]}`.
  Programmatic calls never open dialogs.
- **AI assistant.** Built in, with `align` and `workflow` modes. Every step it takes is a normal
  command, so it shows up in history and can be undone.

## Measured, not promised

A few of the accuracy gates that run in CI (full table in [`docs/STATUS.md`](docs/STATUS.md)):

| Gate | Result |
|---|---|
| ICP, scan with σ = 0.02 mm noise | RMS 0.015 mm; 4° rotation recovered as 3.996° |
| Primitive fits, σ = 0.05 mm (plane, sphere, cylinder, cone, torus) | RMS 0.050 mm each (threshold 0.1) |
| Automatic primitive type, 5 kinds at σ = 0.05 mm | 5 / 5 correct |
| AutoSurface on a wave, σ = 0.02 mm | max deviation 0.089 mm (threshold 0.2); normal jump across patches ≤ 8e-5 rad |
| Sketch solver | DOF 0 and dimensions within 1e-9; redundant and conflicting constraints detected |
| STEP solid round-trip | volume and face count identical |
| Hostile input (random bytes into every importer, 20 bad values into every command) | 0 panics |

## Architecture

```
geom ─┬─ kernel (solidcore) ─┐
      ├─ fit                 │
      ├─ align               ├─ deviation, doc ── io ── engine ─┬─ ui-egui ─┬─ surfacecraft      (desktop)
      └─ sketch ─────────────┘                                   ├─ mcp ──────┼─ surfacecraft-cli  (CLI, MCP)
                                                                 └─ ai ───────┴─ surfacecraft-web  (wasm)
```

- **Layering is enforced** (`cargo xtask layers`): only `crates/kernel` talks to solidcore, and
  only the UI layer pulls in UI crates.
- **Mesh computation lives here; B-rep and NURBS go through solidcore.** App-side geometry beyond
  that is listed in [`xtask/app-geometry.txt`](xtask/app-geometry.txt), and the list may only shrink.
- **Everything is a command** with a JSON Schema, so the four front ends stay thin.
- Design document: [`docs/superpowers/specs/`](docs/superpowers/specs/2026-11-08-surfacecraft-scan-to-cad-rust-design.md).

## Roadmap

The MVP (milestones M0–M6: skeleton, alignment, primitives, Fit Surface, freeform, parametric,
web + MCP + AI) is complete. The next stages make it production-ready:

- **P1 — real scans** ✅ mesh editing, robust segmentation and noise estimation, a scan corpus.
- **P2 — performance:** parallel computation and level of detail for million-triangle scans.
- **P3 and beyond:** see [`ROADMAP.md`](ROADMAP.md) and the
  [production-readiness plan](docs/superpowers/plans/2026-11-09-production-readiness.md).

Current state, measurements and known debt: [`docs/STATUS.md`](docs/STATUS.md).

## Contributing

Read [`CONTRIBUTING.md`](CONTRIBUTING.md) and, if you work with an AI agent, [`AGENTS.md`](AGENTS.md)
and the skills in [`skills/`](skills). Run `cargo xtask ci` before every commit.

## License and credits

Licensed under either of [MIT](LICENSE-MIT) or [Apache-2.0](LICENSE-APACHE), at your option.

SurfaceCraft is an independent, clean-room implementation. It contains no code, icons, text,
layouts or data from any commercial reverse-engineering product, and depends on no GPL, LGPL or
AGPL code (`cargo xtask licences --check`). Third-party code and assets are credited in
[`ATTRIBUTION.md`](ATTRIBUTION.md), [`NOTICE`](NOTICE) and
[`THIRD-PARTY-LICENSES.txt`](THIRD-PARTY-LICENSES.txt); see also [`DISCLAIMER.md`](DISCLAIMER.md).
