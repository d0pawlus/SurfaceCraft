# Batched integration passes

Owner's decision (2026-11-09): agents do not run local tests for every change. Each agent
type-checks its packages (`cargo xtask quick <packages>`), pushes, and lists what only this
workstation can check under **"Do sprawdzenia lokalnie"** in its pull request. GitHub Actions runs
the full `cargo xtask ci` (unit tests, gates, wasm) on every pull request.

An integration pass then checks several pull requests on one build:

1. **Collect.** Open pull requests with green CI whose "Do sprawdzenia lokalnie" is not "none".
2. **Combine.** A branch `integration/<date>` from `main`, merging each pull request branch
   (`git merge`, never rebase). A conflict is resolved there and reported to the pull request's
   author; the pull requests themselves stay unchanged.
3. **Build once.** One release build (sccache shares compiled dependencies with every worktree).
4. **Run** every listed check: local corpora (`SURFACECRAFT_LOCAL_CORPUS`, data never committed),
   `perf_scan_scale`, GPU/offscreen rendering, GUI scenarios with screenshots for the owner.
5. **Attribute.** A failure is bisected over the merged pull requests (`git bisect` on the
   integration branch, or re-running on each branch alone) and reported on that pull request with
   the command, the expected and the observed result.
6. **Record.** Aggregate results go to `docs/STATUS.md` (numbers only, no third-party data) in the
   integration branch's own commit; pull requests that passed are marked ready to merge for the
   owner.

The integration branch is not merged itself; `main` receives the pull requests, merged by the owner.

## Local builds by agents

Compiling is the slow part of an agent's work on this workstation, and several agents compiling at
once slow each other down far more than they gain (2026-11-11: eight at once, load average 40 to
70 on 8 cores, a four-minute build took hours). Rules for whoever dispatches agents:

1. **Compiler cache.** Each agent runs cargo with `RUSTC_WRAPPER=sccache` and
   `CARGO_INCREMENTAL=0` (sccache is installed on the workstation; it is set per command, not in
   the global Cargo configuration). Dependencies compiled once are then taken from the cache in
   every other worktree.
2. **Own target directory.** Each worktree has its own `CARGO_TARGET_DIR`
   (`../SurfaceCraft-wt/target-<name>`). One directory shared by several worktrees gave stale
   artifacts and ran another worktree's `xtask`.
3. **At most three agents compiling at a time.** The rest wait; look at the machine's load and at
   builds of other sessions before starting a batch.
4. **No local build for non-code changes.** A change that only touches documentation, locale
   files or other non-code files is pushed and checked by GitHub Actions (documentation-only pull
   requests skip the heavy steps there).

A copy-on-write clone of a warm target directory (`cp -c -R -p`, seconds on APFS) helps only when
the agent runs exactly the commands the directory was built with: Cargo compiles a dependency
again for a different set of packages or features. The compiler cache does not have that limit.
