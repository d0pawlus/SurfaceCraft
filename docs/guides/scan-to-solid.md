# Scan to solid in one call

`scan.to_solid` takes a raw scan to a closed solid and measures the solid against the scan. It
suits parts made of planes, cylinders, cones, spheres and tori (machined and prismatic parts).
Use the step-by-step guides when the part is organic or when you need a feature history.

## The short path

- `mesh.import` with `path` (or base64 `data`), then `scan.to_solid` with `scan`, then
  `export.step` with the `objects` of the first next step.
- `tolerance` (mm, default 0.2) is what the solid is held to; `min_within` (default 0.99) is the
  share of the measured scan points that must lie within it.
- A scan of more than a million triangles: call with `queued: true` and read it with `job_wait`.

## What it does

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
- `mostly_freeform`: less than half of the scan lies in regions a primitive fits (a body panel,
  a casting, an organic shape). This command will not model it: follow guide `fit-surfaces`.
- `non_manifold_edges`: edges shared by three or more triangles are left after cleaning; the
  regions and holes next to them are unreliable. `mesh.reconstruct` builds a new mesh from the
  points of the file.
- `scan_defects`: holes too large to fill; `mesh.fill_holes` with a larger `max_edges` (it drops
  the regions), then `scan.to_solid` again without `reuse_regions`.

## Second run

- Correct the regions (`region.merge`, `region.split`, `region.grow`, `region.insert`), then call
  `scan.to_solid` with `reuse_regions: true`.
- `edit.undo` takes a whole run back, the cleaning included.
