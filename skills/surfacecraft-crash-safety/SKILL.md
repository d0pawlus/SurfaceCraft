---
name: surfacecraft-crash-safety
description: How non-test SurfaceCraft code avoids panics and treats input as hostile.
---

# Crash safety

- Every production crate root has
  `#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic, clippy::unimplemented, clippy::todo, clippy::unreachable)]`
  and the workspace forbids `unsafe_code`. Tests may unwrap (`clippy.toml`).
- Use `get()` / `get_mut()` instead of indexing with values that come from input. Check every
  count against the bytes actually present before allocating (see `crates/io/src/mesh/stl.rs`).
- Readers have hard limits (`surfacecraft_io::Limits`); recursion is bounded.
- Errors are `Result`s with a message a user can act on. Commands return `CmdError`, the engine
  maps them to `EngineError` and rolls the document back.
- `Session::execute` runs commands under `catch_unwind` on native targets as the last line of
  defence; a caught panic is a bug (`EngineError::Internal`).
- Tests: `hostile_params_never_panic` (every command × hostile JSON) and
  `hostile_bytes_never_panic` (fuzzed mesh files) must stay green.
