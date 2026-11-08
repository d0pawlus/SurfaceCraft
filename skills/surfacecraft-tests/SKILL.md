---
name: surfacecraft-tests
description: Writing and running SurfaceCraft tests and the accuracy gates of design doc §11.1.
---

# Tests

- `cargo xtask ci` runs everything CI runs: fmt, assets, licences, layers, app-geometry, clippy
  `-D warnings`, wasm check, tests.
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
