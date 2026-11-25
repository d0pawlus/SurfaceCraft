# Changelog

All notable changes to SurfaceCraft are listed here, newest release first. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the version numbers follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html). Before 1.0 a minor version may change
command parameters and defaults; such changes are listed under "Changed".

Every feature is a command (`area.verb`), so each entry below is available in the desktop app, the
web app, the CLI, scripts and the MCP server alike, unless it says otherwise. Numbers in brackets
are pull requests in <https://github.com/d0pawlus/SurfaceCraft>.

## [Unreleased]

### Changed

- `surface.patch`: a patch whose sides are all soft is fitted with a curvature-variation energy
  (third derivatives) that damps the fine texture of a scan without flattening the curvature of
  the shape. The energy is relaxed in steps only as far as the scan data need to come within
  `tolerance`, and the control net starts from the number of data points again (the 6 by 6 start
  of 0.3.0 is gone). Such a patch is closer to the scan than in 0.3.0 and its curvature is smooth
  enough to continue with G2: on the Artec corpus G2 holds its 5 % target at 9 of 12 sites with
  errors of 0.03 to 3.5 % (8 of 12 at 0.5 to 4.8 % in 0.3.0). [#167]
- Ray and segment tests against triangles (`geom::tri::ray_hit`, used by picking, ray casts and the
  crossing checks) decide that a ray is parallel to a triangle relative to their size, not by an
  absolute bound: hits on triangles smaller than about 2e-5 units (a part in metres) are now found
  (#158).
- `scan.to_solid`, freeform method (and `thickness`): a closed patch network whose body has no
  positive volume is not a closed solid, the same rule as for the primitives method: the issue
  `no_solid` is raised and the verdict is never `pass` (`review` when the network covers the scan,
  as for an open sheet); such a body also counts as not closed in the larger-cap retry, the rebuild
  and the comparison of `auto` (#158).

### Fixed

- `mesh.fill_holes` and `mesh.clean` (smooth fill), and so `scan.to_solid`: a smooth patch that
  would pass through the mesh or through itself (the thin-plate fairing continued a curved rim past
  itself) is faired again as a membrane, which stays within its loop's convex hull, when that one
  does not cross. On the square cap of a sphere the crossing cap made the wrapped solid read a
  negative `body.volume` (the kernel's healer turned the self-crossing solid inside out) (#158).

## [0.3.0] - 2026-11-16

Three days and twenty-one merged feature and fix pull requests after 0.2.0. The release is about harder real
scans: several scans of one part are brought into one pose without picked points, `scan.to_solid`
can put analytic faces into a freeform wrap, an n-sided opening gets one smooth trimmed patch, and
scans with small handles, self-crossing folds or thin shells get through to a solid more often.

### Highlights

- **Several scans, one pose.** `align.pair_coarse` aligns two overlapping scans and `align.global`
  2 to 24 scans of one part, with no picked points and a verdict that says when the pose is not
  unique (#108, #116, #117).
- **Hybrid solids from one call.** `scan.to_solid` with `method: hybrid` replaces the patches that
  lie on a plane, cylinder, cone or sphere by one analytic face each, inside the freeform wrap (#112).
- **One patch over an n-sided loop.** `surface.patch` fits a single trimmed patch with G0, G1 or G2
  per side against the neighbouring faces (#114).
- **Large scans at import.** `mesh.import` with `reduce_to` reduces a scan above the triangle limit
  while it is read, and the interface offers it when such a file is picked (#136).
- **Scan repair.** `mesh.close_handles` closes small tunnels and bridges, `mesh.clean` can remove
  self-crossing folds, thickened shells no longer cross themselves, and hole filling no longer
  closes an open sheet inside out (#107, #115, #120, #140).
- **Scans of many parts.** `scan.to_solid` models every small closed part with its own verdict and
  reports the pieces it leaves out with a reason (#127).

### Added

#### Alignment

- `align.pair_coarse`: brings a scan onto an overlapping scan or body without picked points, then
  refines with ICP. It returns the motion, the overlap of both scans, the residual, a verdict
  (`unique`, `ambiguous` with the near-tied `alternatives`, `no_match`) and a confidence. It
  proposes by default; `apply: true` moves the scan as one undo step only for a `unique` pose, an
  ambiguous one needs `choose` (#108).
- `align.global`: 2 to 24 scans of one part into one common pose. Pairs are solved by
  `align.pair_coarse`, pairs that disagree with the rest are rejected, and all poses are relaxed
  together. The report gives the pose of every scan, the loop closure error before and after, and a
  verdict (`aligned`, `partial`, `failed`); scans that cannot be connected are listed and never
  moved (#116).

#### Scan to solid

- `method: hybrid` of `scan.to_solid`: each group of patches that lies on a protected primitive
  region (plane, cylinder, cone, sphere) becomes one analytic face that shares its boundary edges
  with the neighbouring patches. A group that cannot be merged safely stays patches, with the reason
  in `left_as_patches`; the merged body replaces the wrap only when it is not worse
  (`hybrid_merge_rejected`, `hybrid_snap_skipped`) (#112).
- Option `close_handles` (with `handle_size`): wraps a copy of the scan with its small handles
  closed, measures the solid against the scan as it was and raises `scan_handles_closed` (#120).
- With `thickness`, self-crossing folds are removed after the holes are filled; the report
  `thickened.folds` lists what was removed and what was left (#115).

#### Scan preparation and mesh editing

- `mesh.import` option `reduce_to` (1000 to the platform limit): reduces a scan while it is
  imported. STL and binary PLY are streamed and clustered on a grid, so memory follows the reduced
  mesh, not the file; the result carries `reduced: {source_triangles, max_error}`. In the desktop
  and web apps a picked file that declares more than 5 million triangles opens a dialog first:
  reduce and import, import in full where the platform can hold it, or cancel (#136).
- `mesh.close_handles`: closes the small handles of a scan (tunnels, bridges and the sponge of tiny
  tunnels where the two sides of a thin wall run together) as a new mesh, keeping the rim and every
  vertex away from them. Candidates refused because cutting all of them at once would split the
  surface are taken back one by one (#120, #122).
- `mesh.clean` option `remove_folds` (off by default): removes small groups of triangles that face
  against the surface around them and pass through it, and fills the hole smoothly. Thin ribs, fins
  and sharp edges are left; a fold that cannot be filled is put back and reported (#115).

#### Surfaces

- `surface.patch`: one trimmed bicubic patch over a closed n-sided loop, with G0, G1 or G2 per side
  against a neighbouring face, optional interior points and curves, and scan data. It returns a
  sheet (#114).
- `surface.from_curves` option `ngon: split | patch`: `patch` covers a region of more than four
  sides with one trimmed patch instead of splitting it into quads. `split` stays the default (#114).
- `surface.wrap` option `coarse_start`: a first coarse mesh too big for `max_patches` is made
  coarser and the adaptive rounds spend the rest of the limit where the patches miss.
  `scan.to_solid` with `close_handles` uses it (#122).

#### Interface

- Auto-align is the first tool of the Align tab. Propose runs `align.auto` without applying it and
  draws the scan in the proposed pose, with rotation, translation and confidence; a proposal below
  the confidence threshold is marked as a guess. Apply moves the scan as one undo step (#132).
- The back side of a scan is drawn in its own colour (violet), so the inside of an open scan and
  flipped triangles can be told from the outside (#131).

### Changed

- An import refused for exceeding the triangle limit names `reduce_to` in its error (#136).
- A tool whose command only proposed (`applied: false`) stays open instead of closing (#132).
- `align.pair_coarse` answers `ambiguous` instead of `unique` when either scan contradicts the pose
  (surface of one scan in front of the other); the result carries `conflict_moving` and
  `conflict_fixed` (#117).
- `mesh.thicken` and `scan.to_solid` with `thickness` bound the offset by the local feature size and
  report `self_intersections_local`, `self_intersections` and `limited_vertices`. Only crossings
  between parts of the scan that are far apart along the surface raise `self_intersection` (#107).
- Point-cloud reconstruction rounds coordinates to a fine power-of-two lattice (about 1e-9 of the
  extent) first, so the result no longer depends on the unit the file was saved in or on the
  platform (#110).

- `scan.to_solid` on a scan of many parts models every piece that is a solid, not only those with
  `part_share` of the largest: a watertight, consistently oriented piece of positive volume with at
  least 100 triangles and 50 tolerances across is a *small part*, modelled with its own verdict
  (`components[].co_equal` tells the kinds apart). Co-equal is decided by area alone. A small part
  never fails the call; small parts that did not pass, pieces left out and parts beyond the cap keep
  it at `review` when together they hold more than 2 % of the surface (`unmodelled_pieces`, new
  `parts_not_passed`), and below that the parts that did not pass are still counted
  (`small_parts_not_passed`) and named in a next step. Pieces left out carry a reason (`debris`,
  `open`, `not_oriented`, `inside_out`); the 16 largest are listed in `pieces_left_out`.
  `minor_and_debris_*` now count the pieces left out and `debris_pieces` those with reason `debris`
  only. The cap of parts per call rises from 24 to 128, the cleaning protects small parts below its
  default by identity, and small parts are never rebuilt (#127).
- `surface.patch`: a patch whose sides are all soft now starts from a 6 by 6 control net and
  grows it only until the scan data are within `tolerance`; before, the start net came from the
  number of data points and followed the fine texture of a dense scan, which made its curvature
  useless as a G2 reference for a neighbouring patch. Such a patch is smoother and its `rms` is
  larger than before; pass a smaller `tolerance` for a closer fit (#139).
- `surface.patch` with a G2 side may take up to 18 rounds instead of 6: the curvature conditions
  are iterated on the same control net until they converge, and only then is the net refined (#139).

### Performance

- `body.compare` and the verdict step of `scan.to_solid` tessellate the body straight into the
  display mesh and build the search tree on it: a third of the memory per triangle on networks of
  10 000 to 60 000 patches (#104).
- Panning the view no longer uploads every mesh to the GPU again in each frame; pan is as smooth as
  orbit and zoom on large scans (#133).
- STEP re-import of a large open sheet written by SurfaceCraft is built from the topology the file
  states instead of being stitched by geometry, which was quadratic in the number of faces (#104).
- Smooth hole filling solves the three axes in parallel on a compressed operator: a large hole
  fills about 1.7 times faster with the same result (#156).
- Changing a parameter in a tool no longer tessellates unchanged bodies or reduces large scans
  again for the preview; display meshes of deleted objects are dropped, and sketches are not
  uploaded to the GPU again in every frame (#146).

### Fixed

- `align.pair_coarse` no longer returns a wrong pose as `unique` (28 of 219 synthetic pairs before,
  none after) (#117).
- Thickened shells no longer have crossings between neighbouring offset triangles, between the two
  sheets of a narrow bead or along the rim strip; what cannot be resolved is counted in
  `unresolved_vertices` instead of being returned as a crossing shell (#107).
- Importing or reconstructing a point cloud saved in metres or inches gives the same mesh as the
  same cloud in millimetres (#110).
- `surface.patch`, G2 to a neighbouring patch on real scans: the curvature condition is now a Newton
  step (it was a fixed-point step whose error was of first order in the speed of the
  parametrisation); a round that halves the miss of the side conditions is kept even when the scan
  deviation grows within the tolerance (the first round, which has no curvature condition yet, was
  kept before); a soft side next to a side with a reference face is no longer a corner conflict
  that relaxes the G1 and G2 conditions near the corner. On the Artec corpus G2 holds its 5 %
  target at 8 of 12 sites (2 of 11 before), every site where no feature of the part crosses the
  shared side. Where one does and the side misses its targets, a warning says so and advises to
  split the side at the feature or to ask for G1 there (#139).
- `scan.to_solid` on a split scan: a part whose `auto` freeform attempt was refused because the call's
  two extra attempts were spent, and whose primitives did not pass, is now wrapped once as its own
  wrap and the better result is kept (twelve small parts of the scan of many parts ended without a
  pass before; each passes now). The solid kept has the part's name and the losing attempt leaves no
  object behind (#127).
- Hole filling (`mesh.clean`, `mesh.fill_holes`, `scan.to_solid` with a large
  `fill_holes_max_edges`) no longer closes an open sheet inside out (one scanned from its hollow
  side, or whose normals are stored inward). When the patches would close a piece of the mesh with
  the scanned face pointing into the enclosed volume, every opening (a loop, or the loops of a
  ragged rim that touch at vertices, judged by their patches together) whose patches are a quarter
  of the piece's area or more stays open (`holes_inward`); smaller holes are filled, so a closed
  scan stored inside out with small holes is still reported as `scan_inside_out`. `scan.to_solid`
  raises the new issue `fill_inside_out` instead of failing with `scan_inside_out`, and
  `mesh.fill_holes` takes `allow_inward: true` to cap such a loop anyway (a mould of a dish)
  (#140).
- `scan.to_solid` with `method: hybrid` merges more than 64 groups of patches into analytic faces:
  groups are taken in batches of 64, up to 256 per merge (the groups past the first 64 stayed
  patches before); groups past 256 stay patches and say why (#142).
- A body or mesh could be drawn with the display mesh of geometry that undo or a delete had freed,
  when new geometry took the same place in memory; display caches now hold what they were made
  from and their keys are never reused (#146).

### Known limitations

- Builds are not code-signed. Windows SmartScreen and macOS Gatekeeper warn on first start; check
  downloads against `SHA256SUMS.txt`.
- macOS builds are for Apple silicon only.
- `method: hybrid` does not merge torus regions, sphere caps of 90 degrees or more, or bores in an
  open sheet, and takes 4 to 6 times as long as the freeform call.
- `surface.patch` and `ngon: patch` return sheets; no command yet sews trimmed patches into a solid.
- A scan with `close_handles` gets `review` at best, because the solid is built from a repaired copy.
- `reduce_to` clusters vertices: it is coarser than `mesh.decimate`, keeps no sharp edges and can
  leave non-manifold edges. ASCII PLY and OBJ files above the browser limit are still refused.
- `mesh.merge` (several aligned scans into one mesh) is not in this release.
- On real scans `scan.to_solid` often answers `review` rather than `pass`. The verdict lists the
  reason and the next step.

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

[0.3.0]: https://github.com/d0pawlus/SurfaceCraft/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/d0pawlus/SurfaceCraft/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/d0pawlus/SurfaceCraft/releases/tag/v0.1.0
