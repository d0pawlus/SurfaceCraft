# MCP for agents: scan to model in one verified call (design and plan)

Goal: an AI agent turns a 3D scan into a CAD model through the MCP server without friction, quickly,
and with a measured answer to "is the model right?". The MCP server, the guides, the job queue and
the bridged mode exist (M6, P8-E1, P8-E2); this plan closes what an agent still trips over.

## What a trial run showed (2026-11-11, `surfacecraft-cli mcp --local`, synthetic scan)

| Finding | Number | Cost to the agent |
|---|---|---|
| `tools/list` in the default (full) mode | 381 KB, 130 tools; 220 KB of it output schemas | about 95 k tokens before the first call |
| `tools/list` in compact mode | 29 KB, 30 tools | every verb needs `describe_command` first |
| Scan to solid | 14 calls (qa, segment, one `primitive.fit` per region, plan, build, compare) | slow, and the agent has to pick regions by hand |
| `body.from_regions` on inputs it cannot honour | a one-face body, `skipped: 2`, `isError: false` | a wrong model unless the agent knows to run `body.compare` and read `unmeasured` |
| Results | every value twice (pretty text and `structuredContent`), floats with 16 digits | 2 to 3 times the tokens needed |
| A wrong parameter | an error text, no schema | one more round trip to `describe_command` |

## Design

### 1. `scan.to_solid` (engine, `crates/engine/src/cmd/pipeline.rs`)

One undoable command that runs the proven steps in order and judges the result. It calls the
existing handlers directly (one undo step, one log entry), so it adds no geometry code.

1. `scan.qa`; an invalid or empty mesh stops the run. Non-manifold edges do not: they are counted
   again after cleaning and reported as an issue.
2. `mesh.clean` with hole filling (`clean`, default true; `fill_holes_max_edges`, default 400).
3. Regions: `region.auto_segment`, or the scan's existing regions with `reuse_regions` (the agent
   fixes regions by hand and runs again; no cleaning then, as cleaning would drop the regions).
4. Selection: a region is modelled when it has at least `min_region_triangles` triangles
   (default: 1 % of the scan, at least 30) and its RMS is within max(0.1 mm, 2 sigma); the others are
   listed in `left_out` with the reason.
5. `primitive.fit` for each selected region, then `primitive.auto_constrain` (`constrain`,
   default true).
6. `body.from_regions`.
7. `body.compare` of the solid against the scan with `tolerance` (default 0.2 mm, the tolerance the
   hybrid gates hold a solid to).

Output: the ids made (`regions`, `primitives`, `body`), the step reports in short form, and

- `verdict`: `pass` (closed solid, no skipped input, coverage >= 0.9, share of points within
  `tolerance` >= `min_within`, default 0.99), `review` (a solid, with issues), `fail` (no solid, or a
  solid that covers less than 90 % of the scan points);
- `issues`: `{code, message}` with stable codes (`skipped_inputs`, `low_coverage`,
  `out_of_tolerance`, `left_out_regions`, `unassigned_triangles`, `no_solid`, `scan_defects`,
  `mostly_freeform`, `non_manifold_edges`);
- `next_steps`: `{command, params, why}`, ready to call (`export.step` on a pass; `surface.fit` for
  a large left-out region; `body.plan_regions` for skipped inputs; `body.refit_faces` for faces out
  of tolerance).

The command never fails because the model is poor: it returns `fail` or `review` and keeps the
regions and primitives so the agent can continue by hand. It fails (and rolls back) only on bad
parameters. No threshold of design doc 11.1 changes.

### 2. Lean MCP transport (`crates/mcp`)

- **Default listing = compact + featured.** One tool per area, plus the workflow commands with
  their full input schema: `mesh_import`, `scan_qa`, `scan_to_solid`, `body_compare`, `export_step`, `guide_read`, `app_batch`
  (42 KB in all).
  `--tools full` keeps the old listing. Any command stays callable by its tool name in either mode.
- **Self-correcting errors.** A parameter error carries the command's `input_schema`; an unknown
  command or verb carries `did_you_mean`.
- **Lean results.** Text content is compact JSON; floats are rounded to 6 decimals (6 significant
  digits below 1e-3) in text and `structuredContent`.
- **Prompts and guide resources.** `prompts/list`, `prompts/get` (`scan_to_solid`, argument
  `scan`); each guide is a resource `surfacecraft://guide/<id>`.
- **Instructions** describe the three-call path: import, `scan_to_solid`, `export_step`.
- The crate is split into `lib.rs`, `jobs.rs`, `lean.rs`, `prompts.rs`, `bridge.rs`, `tests.rs`
  (it was one 1273-line file).

### 3. Documentation

Guide `scan-to-solid` (the quick path and what to do on `review` or `fail`), the agent-tools skill,
README, `docs/STATUS.md`, ATTRIBUTION row for the guide.

## Non-goals

No new segmentation or fitting algorithm, no HTTP transport, no change to the control channel
(P8-H10), no automatic freeform surfacing inside `scan.to_solid` (it names the regions and the
command to run instead).

## Stages and gates

| Stage | Work | Gate |
|---|---|---|
| 1 | `scan.to_solid` | `crates/engine/tests/scan_to_solid.rs`: chamfered pin with a through hole (sigma = 0.03) gives `pass`, 5 faces, volume within 0.5 % of CAD, one undo step removes everything; a scan the planner cannot close gives `fail` or `review` with issues and next steps, never an error; `hostile_params_never_panic` covers it |
| 2 | Lean transport | `crates/mcp/src/tests.rs`: default listing under 48 KB and holds the featured tools; parameter error carries the schema; unknown verb carries `did_you_mean`; rounding keeps integers and strings; prompts and guide resources answer |
| 3 | End to end | `apps/surfacecraft-cli/tests/mcp.rs`: over stdio, a scan sent as base64 goes to a STEP file in 3 tool calls with `verdict: pass` |
| 4 | Docs | `cargo xtask quick` on the changed packages |

## Checks GitHub cannot run

None new: the gates use synthetic scans. The local corpus pass (`docs/integration.md`) should try
`scan.to_solid` on real scans and record how often it reaches `pass`.
