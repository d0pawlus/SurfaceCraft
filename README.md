# SurfaceCraft

Clean-room, pure-Rust **scan-to-CAD**: turn a raw scanner mesh (STL/OBJ/PLY) into clean CAD
geometry — an aligned mesh, extracted primitives, fitted freeform surfaces and an editable
parametric feature tree, exported to STEP/IGES — in one application, on the open
[solidcore](https://github.com/tangency-labs/solidcore) B-rep kernel.

One command-driven core, four front ends: **desktop** (egui + wgpu), **web** (wasm), **CLI** and
**MCP** (agents). Every action is a command with JSON input/output schemas, so anything the UI
does can be scripted:

```bash
cargo run -p surfacecraft-cli -- run examples/scripts/m0-smoke.json
```

```bash
cargo run -p surfacecraft
```

- Design: [`docs/superpowers/specs/2026-11-08-surfacecraft-scan-to-cad-rust-design.md`](docs/superpowers/specs/2026-11-08-surfacecraft-scan-to-cad-rust-design.md)
- State, measurements, debt, next step: [`docs/STATUS.md`](docs/STATUS.md)
- Agents: [`AGENTS.md`](AGENTS.md)

Checks: `cargo xtask ci` (fmt, assets, licences, layers, app-geometry, clippy, wasm, tests).

Licence: MIT OR Apache-2.0. See NOTICE and ATTRIBUTION.md.
