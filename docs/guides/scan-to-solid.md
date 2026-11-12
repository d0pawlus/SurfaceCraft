# Scan to solid in one call

`scan.to_solid` takes a raw scan to a closed solid and measures the solid against the scan. It
has two methods and chooses between them itself (`method: auto`):

- `primitives`: planes, cylinders, cones, spheres and tori fitted to regions; for machined and
  prismatic parts. A few analytic faces, exact dimensions.
- `freeform`: a watertight network of bicubic patches over the whole scan; for castings, body
  panels and organic shapes. Thousands of small faces that follow the scan.

Use the step-by-step guides when you need a feature history.

## The short path

- `mesh.import` with `path` (or base64 `data`), then `scan.to_solid` with `scan`, then
  `export.step` with the `objects` of the first next step.
- `tolerance` (mm, default 0.2) is what the solid is held to; `min_within` (default 0.99) is the
  share of the measured scan points that must lie within it.
- A scan of more than a million triangles: call with `queued: true` and read it with `job_wait`.

## What it does

`auto` segments the scan; when less than half of its triangles lie in regions a primitive fits,
the freeform method runs. Pass `method` to choose yourself.

Freeform (`surface.wrap` inside):

- Cleans the scan, then reduces it to a coarse mesh that stays within a budget of the scan.
- Every coarse triangle becomes three patches fitted to the scan; neighbours share their edges.
- The patches are measured against the scan; the budget is halved only around the scan points the
  patches miss (`refine: adaptive`, the default), until `min_within` of the scan is within
  `tolerance`, or 12000 patches. Smooth, well fitted areas keep large patches. Every round lays
  the patches on a new conforming triangle mesh, so neighbours always share their boundary curves
  (no T-junctions, no gaps). `levels` of the result lists patches and fit per round.
- A scan without open boundaries gives a solid; one with holes gives a sheet (`open_surface`).
- Patches meet in position (`continuity: g0`, what `scan.to_solid` uses). Tangent patches
  (`surface.wrap` with `continuity: g1`) are experimental: on 13 real scans they reached the
  tolerance on 7 where `g0` did on 9, and needed 56 % more patches and 2 to 3 times the time.

Primitives:

- Cleans the scan (`clean`, default true): debris, spikes, holes up to `fill_holes_max_edges`.
  Not with `reuse_regions`: cleaning changes the triangles and would drop the regions.
- Segments it, or takes the regions it already has with `reuse_regions`.
- Fits a primitive to every region with at least `min_region_triangles` triangles (default 1 %
  of the scan, at least 30) whose RMS is within max(0.1 mm, 2 sigma of the scan noise).
- Relates the primitives (`constrain`, default true): parallel, perpendicular, coaxial, equal radii.
- Builds the solid (`body.from_regions`) and compares it with the scan (`body.compare`).
- The whole run is one undo step.

## Reading the result

- `verdict` is `pass` (closed solid, nothing skipped, the points within tolerance), `review` (a
  solid with issues) or `fail` (no solid, or one that covers less than 90 % of the scan).
- `deviation` has `coverage` (share of the scan points on the solid), `within`, `rms`, `max_abs`
  and `mean`; `worst_faces` lists the faces with points outside the tolerance.
- `modelled` lists each region with its primitive, `role`, the `faces` it became and, when it was
  not built in, `skipped` with the reason. `left_out` lists the regions that were not modelled.
- `next_steps` are commands with ready `params`, most useful first.
- Only report the model as done on a `pass`. On a `review`, say what the issues are.

## Issues and what to do

- `no_solid`: the planner found no closed shape. Run `body.plan_regions` with the primitives and
  read each `role` and `reason`. A missing wall or floor is usually a region that was left out.
- `skipped_inputs`: a primitive could not be built in; `modelled[].skipped` says why.
- `low_coverage`: part of the scan has no face. Look at `left_out` and at `unassigned_triangles`.
- `out_of_tolerance`: `body.refit_faces` moves the faces onto the scan; `worst_faces` says where.
  It is offered only when the solid covers the scan.
- `left_out_regions` with an RMS reason: the region is freeform. `surface.fit` it and build with
  `body.from_regions`, passing the primitives of `modelled` and the surface in `surfaces`.
- `unassigned_triangles`: `region.grow` from a seed, or `region.resegment` with a looser tolerance.
- `mostly_freeform` (only with `method: primitives`): less than half of the scan lies in regions
  a primitive fits. Run again with `method: freeform`.
- `open_surface` (freeform): the network is a sheet because the scan has open boundaries. Fill
  the holes (`mesh.fill_holes`) and run again, or `surface.thicken` the sheet when the part is a
  shell.
- `out_of_tolerance` (freeform): the part needs more patches than the limit. When the first
  network stopped at `max_patches` (default 12000) with at least 25 % of the scan within the
  tolerance and the scan has at most 1.5 million triangles, the command wraps once more with five
  times the limit (at most 60000) and keeps the better network; `retry` reports both limits, both
  patch counts and shares and which one was kept. Beyond that, `surface.wrap` with a larger
  `max_patches` (up to 60000); the STEP file grows with the patch count.
- `non_manifold_edges`: edges shared by three or more triangles are left after cleaning; the
  regions and holes next to them are unreliable. `mesh.reconstruct` builds a new mesh from the
  points of the file.
- `scan_defects`: holes too large to fill; `mesh.fill_holes` with a larger `max_edges` (it drops
  the regions), then `scan.to_solid` again without `reuse_regions`.

## Second run

- Correct the regions (`region.merge`, `region.split`, `region.grow`, `region.insert`), then call
  `scan.to_solid` with `reuse_regions: true`.
- `edit.undo` takes a whole run back, the cleaning included.
