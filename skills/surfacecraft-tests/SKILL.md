---
name: surfacecraft-tests
description: Writing and running SurfaceCraft tests and the accuracy gates of design doc §11.1.
---

# Tests

- `cargo xtask ci` runs everything CI runs: fmt, assets, licences, layers, app-geometry, clippy
  `-D warnings`, wasm check, tests. It runs on GitHub Actions for every pull request (Linux, plus
  Windows tests); merge only when every check is green.
- Locally, before each commit: `cargo xtask quick <changed packages>` (the static steps plus clippy,
  which type-checks; add `--tests` only when you need a test result to continue). Several parallel full builds of solidcore exhaust the RAM of a
  workstation; the pull request runs the whole suite.
- Checks only this workstation can run (local corpora, performance, GPU/GUI) go under "Do
  sprawdzenia lokalnie" in the pull request and run in batched integration passes
  (`docs/integration.md`).
- Documentation-only commits (`*.md`, STATUS, plans, merge resolutions of those) skip the local
  build entirely.
- On a workstation with `sccache` installed as the rustc wrapper (`~/.cargo/config.toml`,
  `build.rustc-wrapper`), every worktree reuses the compiled dependencies (solidcore once, not per
  worktree). Builds that are not incremental (the `ci` and `release` profiles) are cached; keep
  `CARGO_BUILD_JOBS` at 2 to 4 when several worktrees build at once.
- Unit tests sit next to the code; cross-crate behaviour in `crates/<c>/tests/`. A regression test
  goes into the area's existing test file.
- Accuracy gates (design doc §11.1, never loosened to make a test pass):
  - M1 alignment: known transform recovered on noise-free synthetic data within 1e-6 mm; ICP stops
    when the per-iteration change is below 1e-4 mm.
  - M2 primitives: RMS ≤ 0.1 mm on noisy synthetic data, correct automatic type.
  - M3/M4 surfaces: maximum deviation ≤ 0.2 mm.
  Gate tests are named `gate_m<N>_*` so they can be listed with `cargo test gate_`.
- Synthetic scans come from `surfacecraft_geom::synth` (exact ground truth). Local realism tests on
  third-party scans are `#[ignore]` and read `SURFACECRAFT_LOCAL_CORPUS`; never commit their output.
