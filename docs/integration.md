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

Compiling is the slow part of an agent's work on this workstation. One dependency, `solidcore`,
takes about three of the three and a half minutes of a first build, and several agents compiling
at once slow each other down far more than they gain (2026-11-11: eight at once, load average 40
to 70 on 8 cores, a four-minute build took hours). Rules for whoever dispatches agents:

1. **Start from a clone of the seed target directory.** `../SurfaceCraft-wt/target-seed` is
   built on `main` with the commands agents run (`cargo test --profile ci -p <package> --no-run`
   for each library package, `cargo xtask quick` for the same packages, `cargo xtask wasm`). A
   new worktree gets a copy-on-write clone of it, which takes about a second on APFS and no disk
   space until files change:
   `cp -c -R -p ../SurfaceCraft-wt/target-seed ../SurfaceCraft-wt/target-<name>`, then
   `CARGO_TARGET_DIR=../SurfaceCraft-wt/target-<name>` and `CARGO_INCREMENTAL=0` for every cargo
   call. Only the workspace crates are compiled again. Rebuild the seed when `Cargo.lock` or the
   toolchain changes (its `SEED_COMMIT` file names the commit it was built on).
2. **Own target directory.** Never one directory shared by several worktrees: that gave stale
   artifacts and ran another worktree's `xtask`.
3. **At most three agents compiling at a time.** The rest wait; look at the machine's load and at
   builds of other sessions before starting a batch.
4. **No local build for non-code changes.** A change that only touches documentation, locale
   files or other non-code files is pushed and checked by GitHub Actions (documentation-only pull
   requests skip the heavy steps there).
5. **Build one test binary, not all of them**, while working:
   `cargo test --profile ci -p <package> --test <file> <name>`.

Measured on 2026-11-12 (M1 Pro, another session's test running alongside):

| First build in a fresh worktree | Engine tests (`--no-run`) |
|---|---|
| Empty target directory | 212 s (`solidcore` alone 178 s) |
| Clone of the seed | 36 s, no dependency compiled |
| After editing `geom`, one engine test binary | 23 s |

`cargo xtask quick surfacecraft-engine` from the clone: 48 s; `surfacecraft-ui-egui` tests: 35 s.

Tried and dropped: a compiler cache (sccache). Its key includes the target directory's path, so
with one target directory per worktree it never hits (second build into another directory: 0
hits, 220 s).
