# Changelog

All notable changes to SurfaceCraft are listed here, newest release first. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the version numbers follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html). Before 1.0 a minor version may change
command parameters and defaults; such changes are listed under "Changed".

Every feature is a command (`area.verb`), so each entry below is available in the desktop app, the
web app, the CLI, scripts and the MCP server alike, unless it says otherwise. Numbers in brackets
are pull requests in <https://github.com/d0pawlus/SurfaceCraft>.

## [0.2.0] - 2026-11-13

Four days and 60 merged pull requests after 0.1.0. The release is about real scans: one command now takes
a scan to a checked solid, organic (freeform) parts are wrapped in patch networks instead of being
cut into primitive slivers, and messy, open, mis-scaled or multi-part scans are prepared before they
are modelled. The interface was rebuilt around the scan-to-CAD workflow, heavy commands no longer
freeze the window, and the browser build is multi-threaded.

### Highlights

- **Scan to solid in one call.** `scan.to_solid` runs quality check, cleaning, segmentation, fitting
  and solid building, then measures the result against the scan and returns a verdict (`pass`,
  `review`, `fail`) with coded issues and ready-to-run next steps (#75).
- **Organic parts.** `surface.wrap` builds an adaptive, optionally tangent-continuous patch network
  over a freeform scan; segmentation keeps a smooth organic area as one freeform region
  (#81, #82, #87, #96).
- **Messy scans.** Assemblies are split into parts, tangled closed scans are rebuilt, one-sided
  scans become shells of constant thickness, and a scan saved in metres or inches is no longer read
  as millimetres (#95, #99, #102).
- **New interface.** A ribbon with one tab per stage (Scan, Align, Segment, Shapes, Sketch, Solid,
  Surface, Inspect, Export), a guided tool panel, and a deviation panel with histogram and hotspots
  (#60, #68).
- **Responsive under load.** Heavy commands run in the background with a Cancel button, on desktop
  and in the browser; the web app uses threads where the browser allows it (#74).
- **Parametric design.** Named parameters and expressions drive sketch dimensions and feature
  values; a body can be refitted to a scan by adjusting its dimensions (#48, #91).

### Added

#### Scan to solid

- `scan.to_solid`: a single undoable command from scan to solid. A poor model is reported as a
  verdict, not as an error; `reuse_regions` models hand-corrected regions again (#75).
- Freeform path with preparation: a `preparation` report, per-part results and
  `components: all | largest | <n>` for scans that hold several parts (#99).
- `thickness`, `symmetric` and `keep_thickened` turn an open, one-sided scan into a shell solid of
  constant wall thickness. Deviation, coverage and verdict are always measured against the scan,
  never against the generated shell (#102).
- One automatic retry with a five times larger patch limit when the first patch network stopped at
  the limit (#96).
- Verdict codes that say what is wrong with the input: `inconsistent_orientation`,
  `scan_inside_out`, `tolerance_exceeds_part`, `suspicious_scale`, `open_surface` (#93, #95, #102).

#### Scan preparation and mesh editing

- Units: `mesh.import`, `mesh.reconstruct` and `mesh.import_stream` take `units` (`mm`, `cm`, `m`,
  `in`, `ft`, `um`) and convert to millimetres exactly. `mesh.set_units` corrects a mesh imported
  with the wrong unit. A scan whose size looks wrong and whose unit was never stated raises the
  `suspicious_scale` warning in import, `mesh.info`, `scan.qa` and `scan.to_solid` (#95).
- `mesh.scale_from_length`: uniform scale from two picked points and their true distance (#88).
- `scan.qa`: quality report before modelling: boundary and non-manifold edges, pieces and debris,
  holes with sizes, degenerate and duplicate triangles, estimated noise, and findings ordered by
  severity (#59).
- `scan.split`: separates a scan into its significant parts and classifies nested skins as cavities
  or inclusions; debris is counted and left out (#99).
- `mesh.remesh`: rebuilds a closed scan as a clean, manifold mesh on a voxel grid and projects it
  back onto the scan. It removes the small tunnels, bridges and fins that stop reduction, and keeps
  real holes of the part (#99).
- `mesh.thicken`: a closed, outward-oriented shell from an open mesh, with fold detection and a
  report of where the wall had to be made thinner (#102).
- `mesh.decimate` with `tolerance` and `share`: reduction to a measured, verified distance from the
  original surface instead of a triangle count (#47).
- Working copies: `mesh.working_begin`, `mesh.working_distance`, `mesh.working_promote` and
  `mesh.working_discard` let cleaning, smoothing and decimation act on a copy while the original,
  its regions and its deviation map stay untouched; the copy can be measured against the original
  before it replaces it (#62).
- Out-of-core import: `mesh.import_stream` streams a large STL or binary PLY into a chunk store on
  disk with a level-of-detail pyramid, without holding the mesh in memory; `mesh.store_info` reads
  it. Desktop and CLI only (#66).

#### Alignment

- `align.auto`: proposes a pose from the scan alone. The part is laid on its largest flat face it
  can stand on, with the longer side along x; turned parts get their axis on z. It reports its
  evidence and a confidence, and applies the pose only with `apply: true` (#88).
- `align.symmetry`: finds the mirror plane of a part and places it on a chosen coordinate plane (#88).
- `align.icp` on a marked part of the scan (`region` or `triangles`): only that part drives the fit
  while the whole object moves. Useful when one half of a part is deformed (#51).

#### Segmentation and regions

- Freeform regions: `region.auto_segment` and `region.resegment` merge the fragments of a smooth
  organic area into one region flagged `freeform`, while genuine planes, cylinders and other
  primitives stay. `freeform: false` gives the previous behaviour (#82).
- Region editing: `region.enlarge`, `region.shrink`, `region.separate`, `region.insert` (from
  triangles, a sphere or a box) and `region.resegment` (#53).

#### Primitives and solid building

- `primitive.auto_constrain` finds and enforces more relations between fitted primitives: tangency,
  mirror symmetry (given or detected planes) and equal spacing, with a report of how far each
  primitive moved in units of the scan noise (#55).
- The solid planner (`body.plan_regions`, `body.from_regions`) builds pockets with reflex corners,
  adjacent arcs, pockets with islands, steps and ribs, which it used to skip (#63).
- CAD refit: `body.refit_features` adjusts the dimensions of a feature history (sketch dimensions,
  extrude depths, fillet and chamfer sizes, shell thickness) to match a scan, within a stated
  maximum change. `body.refit_faces` does the same for bodies without history, face by face, with a
  stiffness per face and equal radii kept equal (#48).

#### Surfaces

- `surface.wrap` adaptive layout (`refine: adaptive`, the new default): patches are refined only
  where the scan is farther than the tolerance, instead of everywhere (#81).
- `surface.wrap` crease detection (`crease_deg`) and an experimental tangent-continuous construction
  (`continuity: g1`). The default stays `g0`, because `g1` needs more patches and time on real
  scans (#87).
- `surface.fit` trimming (`trim`): the fitted sheet is cut to the outline of its region, holes
  included. `split` divides a region that is not a height field into pieces that are (#65).
- `surface.fit` exclusion masks (`exclude_triangles`, `exclude_spheres`, `exclude_boxes`), a
  coverage map (measured, hole, extrapolated) and sample counters, so the report says what was
  actually measured (#85).
- `surface.from_curves` accepts regions with three, five, six or more corners (#84).
- Flipped-patch control in `surface.auto`: every patch is checked against the scan normal, flipped
  ones are repaired, and patches that could not be checked are counted and reported (#84).

#### Sketches and parametric history

- Named parameters and expressions: `param.set`, `param.delete`, `param.rename`, `param.list`,
  `param.eval`. A sketch dimension and every numeric field of a feature can be `{"expr": "..."}`;
  changing a parameter recomputes everything that uses it as one undo step (#91).
- `sketch.fit_chains`: fits whole section chains into connected lines and arcs and adds the
  relations the solver accepts (horizontal, vertical, tangent, concentric, equal radius and more),
  so a section becomes an editable, constrained sketch (#50).
- Sketch points can be dragged with the mouse; constraints move the rest of the sketch
  (`sketch.move_point`) (#86).
- Roll-back marker in the feature history (`feature.rollback`): features after the marker are
  parked, and a new feature lands in front of them (#86).

#### Inspection

- Deviation that knows which side it measures: points whose normal faces away from the reference
  are left out of the statistics. New `deviation.stats` (percentiles, histogram, share within
  tolerance bands), `deviation.hotspots` (connected areas beyond tolerance with area, sign and
  volume error) and deviation of a scan against a fitted primitive (#46).
- `body.compare`: body against scan per B-rep face, worst face first (#59).
- Deviation panel: histogram, tolerance bands, hotspot list that centres the view on a click, and
  the deviation under the pointer; Alt+click pins a reading (#68).
- `section.stack`: parallel sections through a mesh with area, perimeter, fitted circles, draft
  angle per side, and a profile class (`prismatic`, `tapered`, `varying`) (#52).

#### Import and export

- `export.section_sheet`: a section or a stack of sections as a 2-D drawing in DXF or SVG (#52).
- STEP export of open sheets writes parameter-space curves (PCURVEs) and seam curves, which
  receiving CAD systems need to trim faces reliably (#61).
- STEP export of closed multi-patch solids through a compact writer: about eight times smaller
  files (2 KB instead of 17 KB per patch) and about three times faster (#79).
- Closed patch networks of tens of thousands of patches can be built into a solid (tested to
  60 000) (#80).

#### Interface

- Ribbon with a tab per workflow stage, object browser on the left, tool panel on the right, a
  navigation bar in the viewport, a welcome card for an empty document, and a seven-step guide that
  marks the next step from the state of the document. Every command is also in the Tools menu and
  the search (Cmd/Ctrl+K) (#60).
- Tool forms: object fields offer only objects of the right kind, sketch entities and constraints
  are lists of fields instead of JSON text, and what went wrong is shown in words above OK and
  Cancel (#60).
- Names and plain hints for every command, parameter and choice in English and Polish, including
  Polish texts for engine messages; 116 own line icons (#60).
- Section view fills the cut face of a closed mesh (#67).
- Screen readers: the native window exposes its accessibility tree, and the 3-D view, the deviation
  histogram and the object rows have names and a sensible keyboard order (#92).

#### MCP server and automation

- Compact tool listing by default, with the workflow commands listed in full: 381 KB down to about
  37 KB. `--tools full` keeps the listing with one tool per command (#49, #75, #97).
- `guide.read` serves step-by-step guides (also as MCP resources); `app.batch` runs several commands
  in one call; `queued: true` returns a job id that `job_status`, `job_wait` and `job_cancel` follow
  (#49).
- Results are compact JSON; a parameter error carries the input schema of the command, and an
  unknown command or verb carries `did_you_mean`; prompt `scan_to_solid` (#75).

#### Releases

- Nightly desktop builds: every day at 04:00 Warsaw time from `main`, published as the pre-release
  `nightly` when `main` changed and its checks passed (#43).
- Release notes of an official release include its section of this changelog.

### Changed

- `surface.wrap` refines adaptively by default. `refine: uniform` gives the 0.1.0 behaviour (#81).
- `max_patches` of `surface.wrap` is a hard limit in every round. A round that would exceed it is
  built as the largest refinement that fits instead of being dropped (#96).
- `region.auto_segment` produces freeform regions by default (see above) (#82).
- The MCP server starts with the compact tool listing; clients that relied on one tool per command
  must pass `--tools full` (#75).
- Heavy commands started from the interface run in the background; other commands are refused until
  the running one ends or is cancelled. CLI, scripts and MCP are unchanged (#74).
- `mesh.decimate` refuses a target of zero triangles (it used to erase the scan) (#60).
- Deviation maps are dropped when the geometry they were computed on moves or changes, so no reading
  comes from stale geometry (#68).
- Official releases are made only by hand from a `v*` tag; a merge to `main` never publishes one (#43).
- solidcore updated to revision `4f2da85` (#54).

### Performance

- Segmentation grows and refits regions in parallel and gives exactly the output of the sequential
  algorithm on any number of threads (#64).
- Parallel and lower-memory vertex welding, binary STL parsing and hole filling. Hole filling no
  longer scans the whole mesh once per hole (18 s for 266 holes in one million triangles before) (#83).
- Brush and lasso selection search the mesh BVH instead of testing every triangle (#67).
- Sketches are redrawn from a cache that is refreshed only when the sketch changes (#86).
- The browser build runs geometry in parallel in web workers where the browser can share memory,
  and falls back to the single-threaded bundle elsewhere (#74).

### Fixed

- `mesh.clean` never adds non-manifold edges; hole filling could create them (1062 on one real
  scan) (#101).
- `mesh.clean` keeps sharp edges and corners when it fills a hole that crosses them, instead of
  rounding them into a chamfer, and removes spikes next to each other (#57).
- Bodies from `surface.wrap` are oriented consistently at every size, and reduction no longer folds
  triangles over the surface. A part smaller than the tolerance is reported as such instead of
  yielding a body without volume (#93).
- Splits of a quad layout are checked on the real geometry; a split that would damage a sound quad
  is rejected and reported (#84).
- Deviation statistics fit a narrow dock; three values had no visible number (#73).
- `scan.qa` and `body.compare` are reachable from the redesigned workflow (#69).
- The web app no longer shows a docking error when its first frame has no size (#44).
- The web app no longer freezes during automatic segmentation in browsers with thread support (#74).

### Known limitations

- Builds are not code-signed. Windows SmartScreen and macOS Gatekeeper warn on first start; check
  downloads against `SHA256SUMS.txt`.
- macOS builds are for Apple silicon only.
- Tangent-continuous patches (`continuity: g1`) are experimental and off by default.
- Out-of-core import stores large meshes on disk, but the modelling commands do not work on the
  store yet.
- On real scans `scan.to_solid` often answers `review` rather than `pass`: self-intersecting scans
  and large primitives with form error are not resolved automatically yet. The verdict lists the
  reason and the next step.

## [0.1.0] - 2026-11-09

First public release: mesh import (STL, OBJ, PLY, XYZ) and point-cloud reconstruction, alignment
(3-2-1, N-point, ICP, principal axes), automatic segmentation and primitive fitting, NURBS surface
fitting and freeform patch networks, sketches with a constraint solver, a parametric feature
history, hybrid solids from primitives and freeform patches, STEP and IGES export, the desktop and
web applications, the CLI, and the MCP server.

[0.2.0]: https://github.com/d0pawlus/SurfaceCraft/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/d0pawlus/SurfaceCraft/releases/tag/v0.1.0
