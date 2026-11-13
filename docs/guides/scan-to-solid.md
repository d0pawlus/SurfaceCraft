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

## An open scan: a shell of constant thickness

A panel, a body or a casting scanned from one side has no back: the scan is open by nature, and the
patch network over it is a sheet (`open_surface`), not a solid. `surface.thicken` cannot help, the
kernel refuses sheets of many faces. Give the wall thickness instead:

- `scan.to_solid` with `thickness` (mm, no default, not 0, at most half the size of the scan). When
  the cleaned scan is open, the scan is thickened into a closed shell on the mesh side
  (`mesh.thicken`) and the shell is wrapped in patches (method `freeform`; `thickness` has no meaning
  with `primitives` or `reuse_regions`). The thickened mesh is removed afterwards unless
  `keep_thickened` is true; the scan stays; the whole call is one undo step.
- The sign says where the material is: positive puts it behind the scanned face (against the normals
  of the scan, the usual case for a scan whose normals point to the scanner), negative in front of
  it, `symmetric` half on each side. If the scan's normals point away from the scanner, use the
  negative sign (or flip the normals).
- The deviation, `coverage` and the verdict are measured against the scan, never against the
  thickened mesh. The back face has no scan points and is not counted. With `symmetric` the scan lies
  in the middle of the wall; it is measured against the face moved half the thickness along the
  scan's normals, so a symmetric result is judged on the shape of the face, not on the half wall.
- The rim is the largest boundary loop of the CLEANED scan (debris and spikes are gone by then) and
  stays open: holes are filled only up to one edge less than it, and up to `fill_holes_max_edges`
  (default 400). Every other loop that is left (a hole above the limit, or a window of the part) is
  stitched through the wall as a window: `thickened.windows` counts them, `rim_edges` is the rim, and
  the issue `windows_stitched` (verdict at most `review`) tells, with a next step that fills every
  hole but the rim.
- `thickened` in the result: `thickness`, `symmetric`, `clamped_vertices`, `largest_reduction`,
  `boundary_loops` and the thickened mesh when kept. A closed scan with `thickness` is an error:
  there is nothing to thicken.
- Where the surface curves more tightly than the wall is thick (a bead, a tight corner on the side
  of the offset), the offset is reduced locally (never below a tenth of the thickness) and the wall
  is thinner there: `clamped_vertices`, `folded_vertices` (the others were only tapered),
  `largest_reduction`, `median_reduction` and the sampled wall `min_wall` / `median_wall` say where
  and how much; more than 1 % of the vertices is the issue `thickness_reduced`. A sharp crease on the
  side of the offset cannot carry any wall: up to 0.1 % of the vertices may stay folded at the floor
  (`unresolved_vertices`), more is refused. The next step offers a thinner wall.
  A thickness whose offset folds triangles at more than 15 % of the vertices is refused (verdict `fail`, issue `no_solid`)
  with a thickness the surface can carry as the next step. Only local folds are detected
  (`self_intersection_checked` is always false): a part of the offset sheet that meets another part
  of the shell, such as a channel narrower than the wall or two sheets closer than the wall, is not.
- Before the offset the mesh is repaired: duplicate triangles are dropped; of the triangles on an
  edge shared by three or more the two that continue the surface best are kept and the others (fins)
  removed (`removed_non_manifold`); the orientation is made consistent per piece
  (`flipped_triangles`); places where boundary loops touch are split. The normals are averaged over a
  ball of half the wall (at least one edge), so noise finer than that does not fold it; at the rim
  the average is one-sided. The strip along the rim has rows of about one edge, at most 64 and at
  most 4 million triangles, so a wall much thicker than the rim edges has needle triangles there.
- `mesh.thicken` does the first half alone: a new mesh, closed and outward, from `object`,
  `thickness`, `symmetric` and `name`.

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
- `low_coverage`: part of the scan has no face (it lies farther than 5 mm from the solid). Look at
  `left_out` and at `unassigned_triangles`.
- `scan_inside_out`: more than 90 % of the scan points face away from the solid, so the normals of the scan point inward (the solid is turned outward by the kernel). Flip the normals of the scan and run again.
- `inconsistent_orientation`: at least half of the missing coverage is points that lie on the solid, but a share (`not_facing` in `deviation`) of its points
  face away from it, so the solid is inside out or has faces turned the wrong way. This is a defect
  of the solid, not a part of the scan it misses. It is a verdict of `fail`. A part not much
  larger than the tolerance (a scan in metres with a tolerance in millimetres) produced it on a
  real scan: see `tolerance_exceeds_part`.
- `tolerance_exceeds_part` (freeform): the tolerance is more than 5 % of the size of the part, so the
  coarse mesh is not resolved and the result cannot be trusted whatever the deviation says. Check the
  units of the scan (`mesh.scale_from_length`) or give a tolerance in the units of the scan.
- `out_of_tolerance`: `body.refit_faces` moves the faces onto the scan; `worst_faces` says where.
  It is offered only when the solid covers the scan.
- `left_out_regions` with an RMS reason: the region is freeform. `surface.fit` it and build with
  `body.from_regions`, passing the primitives of `modelled` and the surface in `surfaces`.
- `unassigned_triangles`: `region.grow` from a seed, or `region.resegment` with a looser tolerance.
- `mostly_freeform` (only with `method: primitives`): less than half of the scan lies in regions
  a primitive fits. Run again with `method: freeform`.
- `open_surface` (freeform): the network is a sheet because the scan has open boundaries. The
  first next step is `scan.to_solid` with a `thickness` (2 mm is a placeholder: measure the wall)
  for a part that is a shell scanned from one side; for holes in a scan of a solid, fill them
  (`mesh.fill_holes`) and run again.
- `thickness_reduced` (with `thickness`): the wall is thinner than asked at more than 1 % of the
  vertices because the surface curves more tightly than the thickness there.
- `out_of_tolerance` (freeform): the part needs more patches than the limit. When `max_patches` was
  not given, the first network stopped at the default limit (12000) with at least 25 % of the scan
  within the tolerance, the scan has at most 1.5 million triangles and `allow_retry` is not false,
  the command wraps once more with five times the limit (at most 60000). The second network
  replaces the first only if it is not open where the first is closed and puts at least 0.5
  percentage points more of the scan within the tolerance (or closes an open network at no loss).
  `retry` reports both limits, patches and shares, `kept`, a `reason` (`better`, `not_better`,
  `open_result`, `failed`) and the `error` of a failed second wrap. A `max_patches` you give is
  never exceeded. Beyond that, `surface.wrap` with a larger `max_patches` (up to 60000); the STEP
  file grows with the patch count.
- `non_manifold_edges`: edges shared by three or more triangles are left after cleaning; the
  regions and holes next to them are unreliable. `mesh.reconstruct` builds a new mesh from the
  points of the file.
- `scan_defects`: holes too large to fill; `mesh.fill_holes` with a larger `max_edges` (it drops
  the regions), then `scan.to_solid` again without `reuse_regions`.

## Second run

- Correct the regions (`region.merge`, `region.split`, `region.grow`, `region.insert`), then call
  `scan.to_solid` with `reuse_regions: true`.
- `edit.undo` takes a whole run back, the cleaning included.
