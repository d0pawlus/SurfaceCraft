# ScanRuler 0.10.1 — functional analysis (reference material #3 for SurfaceCraft)

> Source: the **public documentation and metadata only** of ScanRuler by CNC Kitchen (Stefan Hermann),
> <https://scanruler.com>, repository `CNCKitchen/scanruler` at commit
> `6f5dc739df3fe1599e18467fb5fb6aa983c18419` (2026-11-09, app 0.10.1, `scanruler-mcp` 0.6.2).
> Nature: analysis of **documented behaviour and functions**, read from the README, the manual, the plugin
> READMEs and design notes, the agent guides' headings, the changelog and the package manifests; the command
> catalogue `mcp/src/commands.json` was used only to enumerate command names and titles.
> **ScanRuler is AGPL-3.0-only. No implementation file (.ts/.tsx/.js/.mjs/.py, tests included) was opened,
> and nothing here is code, schema, UI text or pseudocode taken from it.** Everything is paraphrased; the
> algorithm names below are the standard published methods the docs name, not ScanRuler's implementation.
> Any adoption into SurfaceCraft must be a **clean-room reimplementation** from this functional description
> (design doc §9; `skills/surfacecraft-ground-rules/SKILL.md`).
> Complements [`2026-11-08-quicksurface-functional-analysis.md`](./2026-11-08-quicksurface-functional-analysis.md)
> and [`2026-11-08-geomagic-designx-functional-analysis.md`](./2026-11-08-geomagic-designx-functional-analysis.md).
> Markers: *observation* = from ScanRuler's docs; *note* = our reading of what it means for SurfaceCraft.

## 1. Product summary, licensing, deployment

*Observation.* ScanRuler is a **browser-only scan-inspection and scan-to-CAD tool**: fit analytic elements to a
3D scan and dimension between them (GOM Inspect / PC-DMIS style), paint deviation against a CAD part, map
wall thickness, measure flatbed images like a measuring microscope, model the scan into a solid STEP body,
and clean up or reduce the mesh. It started (0.1.0, August 2026) as a ball-bar / deviation / thickness /
STEP-datum tool and grew by ~40 releases in three months to 0.10.1 (2026-11-09); several releases a week,
with named outside contributors in the changelog.

- **Privacy model:** all processing is local in the browser; files never leave the machine unless the user
  connects an AI agent, in which case the docs warn explicitly (and require consent once) that what the agent
  reads goes to its model provider.
- **Licence:** AGPL-3.0-only, with a **commercial licence exception** sold by the single copyright holder
  (contributors sign a CLA so the owner can relicense). The commercial pitch targets embedding the fitting /
  alignment / deviation engine, the reverse-engineering or mesh tools in closed software, and hosting a
  modified version without publishing source. The docs stress that serving the app on any web server is
  "offering over a network" under the AGPL.
- **Third-party carve-outs:** Open CASCADE (LGPL-2.1 + exception) loaded as a separate, replaceable WASM file;
  their own AGPL STEP tessellator `meshstep`; OFL fonts; viridis colour tables (MIT). A dependency-licence
  check runs in CI; contributors are told not to transcribe algorithms from GPL/LGPL/AGPL projects.
- **Trademarks:** the ScanRuler name and the CNC Kitchen name/logo are not licensed; forks must rename.
- **Not everything is open:** scanruler.com carries an extra beta workspace that is not in the repository.
- **Deployment:** static Vite build served by a Cloudflare Worker (no server code); optional file store (R2)
  for short links `/s/<name>`; `?open=<url>&workspace=…` opens any CORS-readable file. **Docker:** nginx,
  unprivileged, port 8080, `/healthz`; a second compose service packages the MCP bridge so no Node is needed.

## 2. Architecture (as documented)

*Observation.*
- **Front end:** React 19, Vite 6, TypeScript, three.js + three-mesh-bvh, zustand stores. Heavy work in module
  **Web Workers** (a mesh worker plus plugin worker operations), some in WebAssembly: `manifold-3d` (mesh
  booleans), `meshoptimizer` (QEM simplification), `occt-wasm` (OCCT 8, Reverse engineer only, lazy-loaded,
  5–7 MB compressed / 22 MB unpacked, needs WASM tail calls), `meshstep` (STEP → mesh), `fflate` (zip).
- **Rule:** numerical code (fitting, deviation) must run without a browser — no DOM/scene objects — so the
  same maths runs in Node tests and in a **headless engine build** used by the MCP server.
- **Cancellable long work:** alignment is written so the worker can be stopped between passes; a stopped fit
  leaves nothing half-applied. An app-wide **activity strip** (READY / WORKING n% / ERROR) reports tracked
  worker tasks, showing a percentage only when progress is actually measured.
- **Plugin system:** workspaces such as Reverse engineer and Mesh Tools live in `plugins/<id>/` and reach the
  app only through declared extension points (workspace tab/panel/stage, undo-history sections, project-file
  sections, scan events, agent commands, settle/busy hooks, view toggles, guides, section layers, map
  legends, scene layers, solids lent between workspaces, CI browser checks). A CI script enforces that
  nothing outside `plugins/` imports a plugin; `?plugins=none` runs the bare app. A build must stand with
  any plugin or none.
- **Project file:** `.scanruler` = plain zip of the **original input files** + `project.json` holding only
  user decisions (elements, dimensions, alignments, calibration, features, display settings). Derived
  results (maps, edge chains, fits) are **recomputed on load**, so newer algorithms improve old projects.
  Compatibility is gated by a `schemaVersion`, not the app version. Undo history is not saved.
- **Units:** internal and API in millimetres; display in mm or inch through one units module, enforced by a
  CI check that bans hard-coded "mm" in UI text.
- **Agent bridge (`scanruler-mcp`, Node, `@modelcontextprotocol/sdk` + `ws`):** the agent starts the server
  over stdio; the open browser tab connects to it on `127.0.0.1:7317` with a pairing token (an HTTP probe
  first triggers the browser's local-network permission). Every command runs **in the visible tab, as one
  undo step**. Files travel as binary frames, not base64. Several server copies share one tab (first holds
  the port, others relay). Options: `--serve` a local build, `--engine/--headless` (no tab; only commands
  marked headless), `--tools compact` (about a dozen tools; the rest discovered via a list tool and run via
  a batch tool, for small local models), namespace filtering (`--tools=-flat,-thickness`), `--trace`,
  `--answer-within` (long calls return a queued job handle; a wait tool polls), daily self-update check.
  Server-own tools: status, open, wait, batch, write/read/list files (confined to the scan's folder, refusing
  agent-instruction files such as `AGENTS.md`), tools.
- **Command catalogue:** `area.verb` names, dots become underscores as MCP tool names, each with an input
  schema and a long behavioural description including "traps". Core app: **62 commands in 16 areas** —
  scan 10, flat (2D) 9, deviation 6, export 6, element 5, align 4, history 4, project 3, section 3, session 3,
  dimension 2, thickness 2, view 2, guide 1, report 1, workspace 1. Plugins add **~36 `reverse.*`**
  (parameters, features, sketch, body, section fit/stack, thread, network, notes, import/export) and
  **24 `mesh.*`**. With the server's 8 tools that is "over a hundred tools and ~50k tokens of schema".
- **Served guides:** `guide.read` returns task guides (measure, model, scan-to-cad, references) embedded in
  the build, so agent instructions always match the connected version; the shipped agent "skills" are only
  pointers to these guides.
- **Testing:** vitest unit + acceptance (real scans as optional local fixtures, skipped when absent — "never
  loosen a tolerance to pass"), MCP server tests, ~18 headless-Chrome end-to-end scripts that build their own
  fixtures with known answers, seeded fuzz of the RE kernel with blessed goldens on kernel upgrades, and a
  build benchmark over saved projects.

*Note.* This is strikingly close to SurfaceCraft's own shape (one command core, MCP bridge to a running app
with token on 127.0.0.1, undo per command, licence and boundary checks in CI, local-only corpus). Differences:
ScanRuler is browser-first with a JS bridge; SurfaceCraft is native-first Rust with wasm as a target.

## 3. Feature inventory by workflow

### 3.1 Import and formats
*Observation.* Scan: STL (asks for units: mm/cm/m/in, remembered), PLY, OBJ (+ `.mtl` + texture → colour),
3MF (own units, colours, slicer projects open as parts without modifiers/supports), STEP (tessellated finer
than a reference). Reference: STL/PLY/OBJ/3MF/STEP (AP203/214/242), tessellated by curvature only with a
part-scaled chord tolerance and an audit that flags cracked solids (sign of deviation needs a closed inside).
Flatbed: PNG/JPEG with the file's dpi as nominal scale. Drag-and-drop anywhere; open from URL.
Photogrammetry scans with arbitrary scale are detected (size heuristics, known file names) and flagged.

### 3.2 Mesh editing (Mesh Tools plugin)
*Observation.* Non-destructive **"edited copy"** model: the first edit forks a copy; every other workspace
keeps measuring the original until **Use for analysis** promotes the copy (then elements are refitted,
sections recut, maps remeasured). The copy can be coloured by its distance from the scan in use, with
added (unscanned) surface shown grey. Tools: remove islands, delete marked, flip normals, cut at plane (with
a preview of what closes), mirror, fill holes, smooth (keep-sharp-edges option), densify (split long CAD
triangles without moving the surface), shrink/expand (offset), boolean (with measured cylinders or modelled
bodies), shell/hollow, **wrap** (one closed skin, inside dropped), **trim to scan** (cut a modelled
replacement piece to fit a broken part), **reduce mesh** (QEM via meshoptimizer, with the tolerance
*measured both ways* and locally pinned until it holds for 99.9 % of each side; triangle-limit presets
quoted against Fusion / SOLIDWORKS import limits; export in the aligned pose). Suggested tool order is
encoded in the toolbar order. Roadmap: keep sharp edges through reduction.

### 3.3 Alignment
*Observation.*
- **3-2-1 datum alignment** as three plain-language steps (set on a plane → align with an axis → move to
  zero), slots accepting fitted elements or picked points, live preview of the pose before applying,
  the datum planes drawn as a "stage" around the part.
- **Auto-align:** proposes the pose from the scan alone by voting surface orientations, treating flat open
  rims as the missing face they stand for, laying the part on its largest flat face (or a turned part on
  its axis end) with its long side along X; result opens in the 3-2-1 editor for confirmation; ~1 s on
  2 M triangles. Two levelling modes (all faces by area vs. largest face exact).
- **Use symmetry:** settle the pose on the part's mirror plane.
- **Best fit to reference (Deviation):** rigid only, no scale (scale error is something to *see*); multi-start
  initialisation incl. all 24 principal-axis permutations; robust scoring; streamed intermediate poses;
  stoppable. Fallback **align by picking ≥3 point pairs** in a split view (collinear picks rejected), then ICP.
  **Local fine fit** on a hand-marked region (exclude fixtures, spray, supports, risers).
- **Scale part** to true size from one measured length vs. a caliper value (photogrammetry).
- Move/rotate by typed numbers is visible on the live site's Alignment panel (not detailed in the docs read);
  `align.clear` resets the alignment.

### 3.4 Primitive fitting and elements (3D Measure)
*Observation.* Click-to-fit **sphere, cylinder, cone, torus, plane, circle**: Gaussian (orthogonal LSQ) fit
with robust initial estimate, automatic region growth around the click that stops at edges and peels the
scan's rounded edge rims; multiple clicks for broken patches; per-element used-points cut-off (all/3σ/2σ/1σ).
Reports σ, form error, **Ø min / Ø max** brackets (what a caliper/pin gauge would find), cylinder length and
arc of wall. Manual marking with window / brush / lasso / straight-line brush, erase, invert, "through the
part", draft-restore after accidental Esc. **Constructions:** points (picked, typed, scan centroid, midpoint,
line–plane intersection), lines (two points, axis, plane–plane), planes (3 points, offset, midplane, typed,
**symmetry plane of the scan**), circles (3+ points, plane∩cylinder/sphere, typed). Constructions re-evaluate
when sources are refitted; delete cascades. **Extend** planes/cylinders for CAD hand-off (grips + fields,
"fit only inside the span"), **Assumed Ø** (design value carried beside the measurement, typo guard),
**Align element** to a designed parallel/perpendicular relation (warns past 2°). Every item reopens in its
editor (✎) keeping its identity, so downstream dimensions re-read instead of rebuilding.

### 3.5 Sections
*Observation.* Section plane taken across any element's direction or a coordinate plane, slid by offset,
tilted with a gizmo; frozen at creation; cut chains < 1 mm dropped. **Shadow** section: the part's projected
outer outline as lines/arcs/circles kept on the outside with a clearance ("play") and tolerance band, corner
rounding, area readout, optional extrusion to a solid block — for bins, foam cutouts and pockets. Sections
feed the 2D sheet; 2D fits on a section are drawn back in 3D and exported to STEP in the cut plane.

### 3.6 Dimensions
*Observation.* Created deliberately (no all-pairs explosion): point–point (with ΔXYZ; sphere gap/span),
point–axis, point–plane (signed), axis–axis, axis–plane, plane–plane, three angle types, diameter. Plane-based
distances also report the **surface-to-surface range** from each face's form error. Fits are treated as
finite: extrapolation warns, meaningless values refuse (planes > 3° from parallel). Optional nominal ± tolerance
with PASS/FAIL. A doc section explains why slice-based CAD measurements legitimately differ (projected
angle vs dihedral, mean vs min/max diameter).

### 3.7 GD&T
*Observation.* Flatness, cylindricity, sphericity, circularity; parallelism, perpendicularity, angularity
(basic angle); coaxiality, concentricity — to any measured datum, with optional limit and PASS/FAIL count.
Feature semantics follow ISO 1101 (a plane is its measured surface, a cylinder its axis). Since 0.10.1 form
tolerances are **minimum-zone** (previously spread about the best fit, which could fail good parts).

### 3.8 Deviation
*Observation.* Signed scan→reference distance (reference must be watertight; sign from pseudonormals at the
closest feature), interpolated hover reading and pinned readings, split side-by-side view, "reference must
face the same way" rule against thin walls, max search distance, range defaulting to the 95th percentile,
continuous or banded ramp, histogram + min/max/mean/RMS/σ, "within ±tol" share, nine colour maps (jet +
viridis family) with out-of-range caps. **Find volume errors / hotspots:** regions of extra or missing
material and patches over tolerance. **Animate the deformation** (FE-style exaggerated, spatially smoothed
motion). **Deviation from a fitted element** (no reference, no alignment, live). Export deviation as data.

### 3.9 Wall thickness
*Observation.* Self-referenced map, two methods (ray along inward normal with optional cone of rays; sphere
grown at the ray's midpoint), both-sided exits, grazing-surface rejection, unmeasured = grey, reversed ramp
(hot = thin), 2nd–95th percentile scale, "thinner than" share, histogram.

### 3.10 2D Measure (flatbed and section sheet)
*Observation.* Subpixel edge chains, three fit modes (click, box, picked points) for points, lines, circles,
arcs, splines; explicit scale states (none / nominal = **UNCALIBRATED** chip / calibrated, separate X and Y,
saved as scanner profiles); sheet alignment, rotate/mirror; counts and outline profiling (`flat.profile`
counts peaks); object detection on the image. Export SVG/DXF at 1:1 as true LINE/ARC/CIRCLE entities, CSV.

### 3.11 Reverse engineering to CAD (Reverse engineer plugin, OCCT)
*Observation.* Output is a **closed B-rep solid in STEP** (FreeCAD/Fusion open it as a body) plus STL/3MF for
printing; works without a scan as "a small parametric CAD". Sketch on a section / fitted / coordinate plane:
**Fit all chains** turns the slice into joined lines and arcs held to their points by the solver (detail
slider, fit tolerance, axis-snapping with inferred relations), infer corners, close; drawing tools with
snapping; constraints and dimensions with **named parameters and expressions**. Features: extrude/revolve
(new body / join / cut), fillet, chamfer, shell, hole, thread (ISO metric/UNC/UNF, read off the scan),
draft, mirror (symmetry plane default), patterns, loft, sweep, move faces, **snap faces onto the scan**,
Import STEP, move body, components (assembly-browser body list). Rollback bar, suppress by kind, **Play**
(animated history playback with captions). Surface route: **Fit surface** on a flood-filled or painted
region (degree-3 B-spline, control-net size, expand-by margin, smoothness) or exact **plane/cylinder with
Auto Constrain** (snap to parallel/perpendicular/coaxial within 0.5° / 1 mm, reporting σ before/after);
**automatic trim** of a surface set into a solid; **patch network** of tangent-continuous B-spline patches
with creases, drawn patch by patch over the scan. **Curvature colouring** to place region borders.
**Compare in Deviation** sends bodies as the reference. **Notes for the agent**: pins on the part saying
what is wrong, resolved by the agent. Their design note includes a frame-by-frame study of QuickSurface
demo videos as the surface-route benchmark, and says control-cage freeform and automatic surfacing are not
planned.

### 3.12 Measurement reports and export
*Observation.* Copy summary / copy report (plain text, PASS/FAIL), `report.get`; STEP AP214 of elements as
"solids & faces" (bounded faces, capped cylinder solids, balls, section curves, extruded shadows; names in
the CAD tree) or "construction surfaces"; scan as STL/3MF (colours kept) or PLY/XYZ cloud with normals, in
the aligned pose, never under the original's name. Pictures for agents: `view.render`, `section.render`,
`flat.render`, `scan.heightmap`.

### 3.13 Navigation, input, units, history, recovery
*Observation.* Orbit about the point under the cursor, standard views on number keys, fit-to-view,
**mouse schemes of other CAD tools**, **3Dconnexion SpaceMouse** (Chrome/Edge, only when the tab is focused),
touch. View bar per part (show, transparent, mesh, back faces), section view, crease shading (auto/always/
never, angle). Light/dark/system theme. Units mm/inch for display. `Enter` / **middle click** = the confirm
button of the moment, in every workspace. **Undo/redo** across workspaces, 64 steps, grouped drags, returns
to the workspace that made the edit, waits during computations, agent checkpoints/restore. **Recovery**:
browser checkpoint every few seconds, offered on next visit. **Guided hints**: an amber ring on the next
control, retiring after a workspace is used twice.

## 4. Accuracy, tolerance and performance claims

| Claim | Source |
|---|---|
| Ball-bar centre distance 148.639 mm vs GOM Inspect 148.64 mm (structured-light scan), matching point selection and σ | README "Accuracy"; manual "How the fits work" |
| Bracket scan vs nominal (1.43 M triangles): best fit converges at 0.072 mm RMS; randomly displaced scan returns within **0.9 µm** of the in-place fit | manual "The map"; README |
| Synthetic STEP cube with one face +0.2 mm, one −0.15 mm: read back with sign, 66.6 % within ±0.1 mm (four faces of six) | docs/development.md |
| Reference STEP chord tolerance ≈ 0.01 mm on a 100 mm part (≈ 1/10 of scanner resolution); 20 mm cube = 12 triangles vs 238 510 for size-adaptive defaults | manual "The reference straight from CAD" |
| Auto-align ~1 s on a 2 M-triangle scan | manual "Aligning the part" |
| Wall thickness (ray) ~150 000 vertices/s, < 5 s on 1.4 M triangles; bracket mean 16.3 mm ray, 7.7 mm their sphere, 3.3 mm max-inscribed-sphere (why they rejected the latter) | manual "Wall thickness" |
| Flatbed at 600 dpi = 42 µm/px; synthetic edge recovered better than 0.1 px | manual "2D Measure" |
| Reduce mesh: 215 k ball bar → ~4 600 triangles at ±0.05 mm in 0.5 s; 1.4 M block → ~20 500 at ±0.1 mm in 5 s, ~55 500 at ±0.05 mm in 10 s; tolerance = 99.9 % of each side | plugins/mesh-tools/README.md |
| Smooth: 1.4 M triangles in ~6 s; noisy 10 mm ball keeps radius within ~1 µm. Densify refuses above 20 M triangles | plugins/mesh-tools/README.md |
| Patch network: 30 patches over 0.5 M triangles fit and build in 5 s; patches 0.006–0.014 mm RMS from scan; tangent mismatch a few thousandths of a degree; re-read after edit < 1 s | plugins/reverse/README.md |
| Auto Constrain snaps within 0.5° and 1 mm | plugins/reverse/README.md |
| RE acceptance: Revopoint bracket (175 k vertices) → closed solid, median deviation within ±0.2 mm | plugins/reverse/docs/reverse-engineer-design.md |
| Agent RE runs: fan wheel 96.3 % within ±0.3 mm (RMS 0.117, 106 min, 284 tool calls); moulded housing 86.1 % within ±0.2 (0.173); casting 72.5 % within ±0.2, reported "not done" (0.387); freeform cover 68.9 % within ±0.4 (0.85) | README "Automatic reverse engineering" |
| Agent targets: 90 % in tolerance (analytic), 85 % (castings/mouldings), 75 % (freeform); tolerance ±0.02 (machined) to ±0.5 mm (sand cast), never finer than 3× scan noise | README; plugins/reverse/guides/scan-to-cad.md (headings) |
| Guard rails: plane∩cylinder refused past 5° lean; plane distances refused past 3°; element-align warns past 2°; Assumed Ø typo flag beyond 0.5 mm or 5 % | manual |
| OCCT WASM 5–7 MB compressed, 22 MB unpacked (design note: 4.7 MB brotli); Chrome 114 / Safari 17.2 / Firefox 121 | plugins/reverse/README.md; design note |
| MCP: >100 tools, ~50 k tokens of schema in full mode; compact ≈ a dozen tools; calls held ≤ 45 s then queued | mcp/README.md |

*Note.* Their honesty devices are worth copying in spirit: limits stated next to results ("the numbers compare
the model with the scan, not with the real part"), "not done" as an allowed agent outcome, and acceptance
fixtures that are generated with a known answer.

## 5. UX ideas worth learning from (behavioural)

- **Measurement carried beside design intent:** extension, assumed diameter and designed relation never
  change what was measured; the export uses the design value, reports show both.
- **Same identity on re-edit:** reopening an element/dimension in its creation box keeps id, colour and list
  position, so dependants re-read rather than rebuild; creation method can be switched on the way through.
- **Preview before apply** everywhere (3-2-1 pose, auto-align proposal, reduce result) with "how far it would
  move/rotate" readouts; Auto-align proposes, never applies.
- **One confirm gesture** (`Enter` / middle click) and **sticky tools** (the tool stays in hand until Esc).
- **Two-stage Esc** (first hands the camera back, second discards) plus a **Restore** for the discarded draft.
- **Explicit uncertainty states:** UNCALIBRATED chip, grey for "no reading", "reported as the guess it is",
  STEP tessellation tolerance shown as "the floor under the map".
- **Non-destructive mesh editing** with the distance-from-original map as the trust signal.
- **Recompute-on-load project files** that keep only decisions and original inputs.
- **Deformation animation** of a deviation map to see warp/twist direction at a glance.
- **Agent ergonomics:** guides served by the running build; compact tool mode for small models; batches and
  queued jobs; picture-returning tools; per-region notes the user pins for the agent; every agent step one
  undo step in the visible app; a consent warning about data leaving via the model provider.
- **Feature-history playback** with captions for explaining a model.
- **Guided hint ring** that retires itself after a workspace has been used twice.

## 6. Gap comparison with SurfaceCraft

Basis: `docs/STATUS.md` (M0–M6 and P1 closed, P2 in progress) and the design doc §2–§4, §11.1; SurfaceCraft's
engine registers ~80 commands in areas align, primitive, region, surface, quad, sketch, feature, body,
deviation, mesh, doc, export, import, edit, app.

### 6.1 What ScanRuler has that SurfaceCraft lacks
- **Metrology layer:** dimensions with nominal ± tolerance, GD&T (minimum zone), Ø min/max, surface-to-surface
  range, PASS/FAIL reports. (SurfaceCraft's scope is scan-to-CAD, not inspection — a scope decision.)
- **Wall-thickness map** and **2D flatbed measurement** (likewise outside current scope).
- **Deviation depth:** facing-aware signed map, histogram/bands/percentile range, hover/pinned readings,
  volume-error regions and over-tolerance hotspots, local fine fit on a marked region, point-pair fallback
  alignment, deformation animation, deviation against a single fitted element, STEP reference tessellated
  by curvature with an audit. SurfaceCraft has `deviation.compute/noise/clear` only.
- **Alignment helpers:** auto-align from surface orientation (incl. open-rim faces), align on symmetry plane,
  scale from one measured length, live pose preview in the 3-2-1 editor.
- **Element constructions and datum hand-off:** constructed points/lines/planes/circles with live
  re-evaluation, extend, assumed dimension, designed relation, STEP export of datums as faces/solids.
- **Sections as a product:** section stack (draft/taper/radius trends), shadow outline with clearance,
  2D sheet export to DXF/SVG/CSV; "fit all chains" with relation inference and axis snapping.
- **Mesh tools:** edited-copy model, reduce with *measured* tolerance, wrap, trim-to-scan repair piece,
  mesh booleans with bodies, shell/hollow, mirror, densify, offset, flip normals; STL/3MF/PLY/XYZ export in pose.
- **RE modelling:** parameters/expressions, holes, threads (incl. read from scan), draft, loft, sweep, move
  faces, snap faces onto scan, Import STEP as bodies, components, history playback, **automatic trim of a
  surface set into a solid**, **Auto Constrain** for fitted planes/cylinders, patch network with tangency,
  curvature colouring, body clearance/check/audit, **stable face/edge picks via fingerprints**.
- **Agent ergonomics:** served guides, compact/namespace tool modes, batch + queued jobs + trace, scan QA,
  per-face deviation report of bodies, user notes for the agent, multi-agent relay.
- **App UX:** recovery checkpoints, open-from-link, mm/inch display, SpaceMouse, CAD mouse schemes, touch,
  colour-map choice, activity strip with measured progress, guided hints.

### 6.2 What SurfaceCraft has that ScanRuler lacks
- **Pure-Rust, permissive (MIT/Apache) stack** with its own B-rep kernel (solidcore) — no LGPL OCCT, no AGPL;
  embeddable in closed products without a commercial exception.
- **Native desktop + CLI + MCP + web from one core**; the CLI/MCP run the full engine in-process (ScanRuler's
  headless engine exposes only the commands marked headless; most work needs a browser tab).
- **IGES export** (ScanRuler documents STEP/STL/3MF only).
- **Region as a first-class entity** with auto-segment/merge/split and **automatic primitive-type choice by
  residual** (ScanRuler's `scan.segment` finds flat regions; type is chosen by the user's click tool).
- **Quad-layout AutoSurface** with G1 Bézier patches and **thicken** of freeform sheets (ScanRuler explicitly
  does not plan automatic surfacing).
- **Fillet radius suggestion from the mesh** as a command; feature replay/suppress with exact-volume gates.
- **Hostile-input guarantees** (fuzzed importers, hostile parameters for every command, no panics) and
  §11.1 accuracy gates in CI on synthetic scans with known truth.
- A provider-agnostic **in-app AI assistant loop** (ScanRuler relies entirely on an external agent).

### 6.3 Candidate backlog items (functional; clean-room only)
All items below are **functional targets**. Implementation must be designed from this description and
public literature, never from ScanRuler source (AGPL).

1. **Deviation toolkit v2** — facing-aware signed map, percentile range + histogram + bands, hover/pin
   readings, over-tolerance hotspot regions and volume error, local fine fit on a marked region, point-pair
   fallback alignment, deviation against a single primitive. Directly serves design-doc goal "constant
   accuracy feedback".
2. **Surface-set trim to solid + Auto Constrain** — close a set of fitted planes/cylinders/freeform sheets into
   a solid by keeping cells lying on the scan; snap fitted primitives to parallel/perpendicular/coaxial within
   tolerances and report σ before/after. Both already listed as debt in STATUS.md.
3. **Section-sketch automation** — fit whole section chains into joined lines/arcs with inferred relations
   and axis snapping; section stacks reporting draft/taper/radius trends; DXF/SVG export of a section sheet.
4. **Mesh reduction with a measured tolerance + non-destructive edited copy** — verify reduction both ways and
   refine locally until a stated share holds; keep edits on a copy with a distance-from-original map until
   promoted. Extends P1's QEM decimation.
5. **Agent ergonomics and stable references** — served task guides via a `guide.read`-style command, compact
   and namespace-filtered MCP tool lists (SurfaceCraft is also at 104+ tools), batch + queued jobs, a scan QA
   command, per-face body-vs-scan reports, and fingerprint-based face/edge picks (fixes the positional-index
   debt in STATUS.md).

Further candidates: auto-align from surface orientation and align on symmetry plane; scale-from-measurement
for photogrammetry; recompute-on-load project format storing original inputs; recovery checkpoints;
parameters/expressions in features; thread feature; history playback; SpaceMouse and CAD mouse schemes;
units display toggle. **Scope decision for the user:** whether SurfaceCraft should grow an inspection layer
(dimensions, GD&T, wall thickness, 2D flatbed) — ScanRuler shows these are cheap once fitting and deviation
exist, but design doc §2 limits scope to four reconstruction functions.

## 7. Sources

Repository `CNCKitchen/scanruler`, commit `6f5dc739df3fe1599e18467fb5fb6aa983c18419`, fetched with
`gh api repos/CNCKitchen/scanruler/contents/<path>?ref=<sha>`. Read (documentation and metadata only):
- `README.md`, `COMMERCIAL.md`, `AGENTS.md`, `CHANGELOG.md` (version headings and the 0.10.x entries),
  `package.json`
- `docs/manual.md`, `docs/development.md`, `docs/docker.md`, `docs/activity-status.md`,
  `docs/virtual-caliper-plan.md` (introduction only)
- `mcp/README.md`, `mcp/package.json`, `mcp/src/commands.json` (command names and titles only; schemas and
  descriptions not reproduced)
- `plugins/README.md`, `plugins/mesh-tools/README.md`, `plugins/reverse/README.md`,
  `plugins/reverse/docs/reverse-engineer-design.md` (goal, kernel, acceptance, milestones, surface-route study)
- `plugins/reverse/skills/scan-reverse-engineering/SKILL.md`, `skills/scan-measuring/SKILL.md`
- Headings only: `plugins/reverse/guides/scan-to-cad.md`, `plugins/reverse/guides/model.md`,
  `src/commands/guides/measure.md`
- Fetched but not read: `CONTRIBUTING.md`, `plugins/mesh-tools/CHANGELOG.md`, `plugins/reverse/CHANGELOG.md`,
  `plugins/reverse/guides/model-reference.md`, `src/commands/guides/measure-reference.md`

Not opened: any `.ts`, `.tsx`, `.js`, `.mjs`, `.py` file, any test, `docs/images/*`, icons and brand assets.
