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
