# Scan to solid in one call

`scan.to_solid` takes a raw scan to a closed solid and measures the solid against the scan. It
has three methods and chooses between them itself (`method: auto`):

- `primitives`: planes, cylinders, cones, spheres and tori fitted to regions; for machined and
  prismatic parts. A few analytic faces, exact dimensions.
- `freeform`: a watertight network of bicubic patches over the whole scan; for castings, body
  panels and organic shapes. Thousands of small faces that follow the scan.
- `hybrid`: one closed body in which the regions a primitive fits are exact analytic faces (plane,
  cylinder, cone, sphere, torus) and the rest of the scan is a network of patches; for parts that
  are partly machined and partly organic. The result reports `analytic` (faces on analytic
  surfaces, all faces, and their share of the surface area).

Use the step-by-step guides when you need a feature history.

## The short path

- `mesh.import` with `path` (or base64 `data`), then `scan.to_solid` with `scan`, then
  `export.step` with the `objects` of the first next step.
- `tolerance` (mm, default 0.2) is what the solid is held to; `min_within` (default 0.99) is the
  share of the measured scan points that must lie within it.
- A scan of more than a million triangles: call with `queued: true` and read it with `job_wait`.

## What it does

`auto` segments the scan (at your `tolerance`) and starts with the method the scan suggests: the
primitives when at least half of its triangles lie in regions a primitive fits, else freeform. When
that attempt does not end in `pass`, the other method runs too (the primitives after a freeform
attempt only if at least a quarter of the scan lies in primitive regions) and the better result is
returned: pass, then review, then fail; at the same verdict the larger share of all scan points
within the tolerance, then a closed solid over a sheet. `attempts` lists what was tried (method,
verdict, issue codes, seconds, `returned`); the attempt that lost leaves nothing in the document and
one `edit.undo` takes the whole run back. A winner that is no pass carries `other_method_tried`.
When the primitives cover at least a quarter of the scan (the same share as for the primitives
attempt after a freeform one) and the best result is no pass, `auto` also tries the hybrid method as
a further attempt. After a pass it does not (the hybrid attempt is slower and was not better on real
scans): the result carries a next step with the `scan.to_solid` call with `method: hybrid` and what
it would add. The hybrid body is returned when it is not worse: the verdict and the share of the scan within the tolerance
(shares within 0.5 percentage points are equal) decide first, then a closed solid over a sheet,
and only at a tie the larger share of the area on analytic faces; a hybrid body that explains the
scan less well is listed in `attempts` and not returned.
Pass `method` to choose yourself (one attempt), or `retry: false` to forbid the second one. It is
skipped on its own, by counts and never by a clock, when the cleaned mesh of the call (or of the
part) has more than 600 000 triangles or the first result has 30 000 patches or more: `attempts`
then holds an entry with `skipped.reason` and `skipped.call`, the call that runs that method. A
second method that is freeform, and the hybrid attempt, are also extra attempts of the call's
budget (see "Extra attempts of one call"); the primitives method makes no wrap and costs none.

The scan noise (`scan_report.noise_used`, `noise_source`, `noise_spread`) is what the segmentation
judges fits by. When it is above the tolerance, nothing is merged into freeform regions and a
result that is not a pass carries `noise_above_tolerance`: the first next step repeats the call with
a tolerance of three times the noise.

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

Hybrid (trim a stock; `surface.wrap`, `mesh.remesh` and the kernel inside):

- Fits a primitive to every region that qualifies, as the primitives method does, and plans them as
  `body.from_regions` does: which are global cuts (planes, full revolutions), which local tools
  (blind holes, pockets, slots), which additions (bosses, domes), which fillets.
- Makes a copy of the scan whose primitive regions lie a little (0.4 of the tolerance) beyond their
  primitive on the side the cut removes, or short of it for an addition; rebuilds that copy as a
  clean closed surface (`mesh.remesh`, voxel 0.75 of the tolerance) and wraps it in patches. This
  is the stock. At most 4000 patches (`max_patches` can only lower that): the kernel's booleans on
  a network are slow and fragile beyond it.
- The kernel trims the stock with the primitives (`hybrid::cut_stock`): the faces on the primitives
  are exactly the primitives, bounded by their intersections with the patches; the rest stays
  patches. A cut the kernel refuses, a partial cylinder, cone, sphere or torus (a fillet needs the
  sharp edge of two planes, which a network of patches has not) and anything else that is not
  applied stays patches, up to about 1.4 times the tolerance from the scan (the stock's own deviation plus the 0.4 offset; the verdict reports it as `out_of_tolerance` where it is over); `modelled[].skipped` says why.
- The kernel does at most `4000 / patches` operations (at least one) on the stock, each a boolean on the whole network; the cuts over that bound stay patches and `analytic.left_by_work_bound` counts them. A cancelled call stops between operations.
- Needs a scan that is closed after cleaning (the rebuild refuses an open one). It is judged
  against the scan by the same rules as the freeform method.

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
  with `primitives` or `reuse_regions`; it models one open shell: with several co-equal parts, or `components` / `part_share`, it is an error: `scan.split` first and thicken a part; `rebuild` does not apply). The thickened mesh is removed afterwards unless
  `keep_thickened` is true; the scan stays; the whole call is one undo step.
  A call with `thickness` is one open shell and one method: `method: auto` does not try a second method for it (`attempts` is empty), and its larger-cap retry draws from the call's budget of extra attempts like every other path (`retry.reason: budget_spent` when the call had spent it; with one shell the budget is always whole, so in practice the retry runs when it qualifies).
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
  with a thickness the surface can carry as the next step. An even sample of the thickened
  mesh is tested for triangles that cross one they share no vertex with (`self_intersections` of
  `thickened`, `self_intersection_samples` tested); a channel or gap narrower than the wall shows
  there, as the issue `self_intersection` (verdict at most `review`), with the sampled count and an extrapolation to the whole mesh. Only a sample is tested. Crossings near a relaxed vertex (the two flanks of a crease within two thicknesses along the surface) are not reported: they are tested exactly during the relaxation and the offset is reduced there until they are gone; the strip along the rim is not part of that test.
- Before the offset the mesh is repaired: duplicate triangles are dropped; of the triangles on an
  edge shared by three or more the two that continue the surface best are kept and the others (fins)
  removed (`removed_non_manifold`); the orientation is made consistent per piece
  (`flipped_triangles`); places where boundary loops touch are split. The normals are averaged over a
  ball of half the wall (at least one edge), so noise finer than that does not fold it; at the rim
  the average is one-sided. The strip along the rim has rows of about one edge, at most 64 and at
  most 4 million triangles, so a wall much thicker than the rim edges has needle triangles there.
- The thickened mesh has about twice the triangles of the scan, so the automatic larger-cap retry
  (never above 1.5 million triangles of the mesh that is wrapped) stops at about 750 000 scan
  triangles; above that, the next step offers `scan.to_solid` with an explicit `max_patches`.
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
- `noise_above_tolerance`: the scan is rougher than the tolerance; use the tolerance of the first
  next step (about 3 times the noise) or a smoother scan.
- `other_method_tried`: `auto` ran the other method as well and it did not do better; see `attempts`.
- Hybrid method, no body (verdict `fail`; `auto` then returns the other result, and the document is
  as it was before the attempt): `hybrid_no_primitives` (no region could be modelled as a
  primitive; use `freeform`), `hybrid_not_closed` (the scan cannot be rebuilt as a closed surface:
  an open scan or one with a thin wall; fill the holes with `mesh.fill_holes`, or use `freeform`),
  `hybrid_stock_failed` (the wrap over the rebuilt scan failed), `hybrid_stock_open` (the patch
  network is not closed), `hybrid_cut_failed` (the kernel could not apply any primitive to the
  stock, or the result was not one closed solid), `hybrid_no_analytic_faces` (no face of the body
  is on a primitive: nothing is gained over `freeform`), `hybrid_failed` (anything else, with the
  reason). `thickness` has no meaning with `method: hybrid`.
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
  `open_result`, `failed`, `budget_spent`) and the `error` of a failed second wrap. `budget_spent`: the retry was due but the call had used its extra attempts, so no second network was made (a call that needed no retry has no `retry`). A `max_patches` you give is
  never exceeded. Beyond that, `surface.wrap` with a larger `max_patches` (up to 60000); the STEP
  file grows with the patch count.
- `non_manifold_edges`: edges shared by three or more triangles are left after cleaning; the
  regions and holes next to them are unreliable. `mesh.reconstruct` builds a new mesh from the
  points of the file.
- `scan_defects`: holes too large to fill; `mesh.fill_holes` with a larger `max_edges` (it drops
  the regions), then `scan.to_solid` again without `reuse_regions`.

## Messy scans: parts, handles, shells

`scan.to_solid` looks at the cleaned scan before it models it and reports what it did in
`preparation` (`[{step, reason, numbers}]`, `numbers.ms` the time spent; empty when the scan was
modelled as it is).

- **Several parts in one file** (`split`). Pieces of the scan are sorted three ways. A piece with at
  least 1 % of the triangles or of the area is a *part*; a part with at least `part_share` (default
  10 %) of the area of the largest piece is *co-equal* and gets a solid of its own (the largest 24
  at most; `components` takes `all`, `largest` or a number). Parts below that are *minor* and
  pieces below 1 % are *debris*: they are not modelled. Exactly one co-equal part: that part is
  modelled by the usual path. Several: `components[]` has a verdict per part, `bodies` lists every
  solid, the top-level `body` and `patches` are the largest part's, `deviation` is the aggregate
  over the modelled parts (area-weighted share within tolerance, largest maximum) and `verdict`
  the worst co-equal part's.
- **What was not modelled is reported** (each issue says what to do: `part_share`, `scan.split`, `components`; the next steps include the split). More than 2 % of the surface in minor pieces and debris:
  issue `unmodelled_pieces` (count and area share) and the verdict is at most `review`. The inner
  skin of a hollow part is not modelled either: `nested_shell_skipped`, at most `review`. Co-equal
  parts past the 24 cap: `parts_beyond_limit` (count and area share), at most `review`. Parts you
  did not ask for with `components: largest` or a number are no issue and do not lower a pass;
  `preparation[0].numbers.not_requested` counts them. After a split with every part passed, one
  `export.step` names all the solids; otherwise each next step says which part it is for.
  `scan.split` does the separation alone (`min_share` above 0 and at most 1) and reports nesting
  (`cavity`, a hollow part's inner skin, or `inclusion`, a separate part inside another). It lists at
  most 64 parts; the significant pieces past that are not made into meshes but are counted in
  `parts_beyond_limit` with their `beyond_limit_triangles` and `beyond_limit_area`.
- **Topological noise** (`remesh`). A closed scan with many handles (genus 64 or more) or non-manifold
  edges (20 or more) is rebuilt with `mesh.remesh` before it is wrapped; a closed scan that wrapping
  does not pass on is rebuilt and wrapped again, and the result that is clearly better is kept (the
  rule of the larger-cap retry: a closed result is never replaced by an open one, the share within
  tolerance must gain more than 0.5 points; a tie keeps the plain result). `rebuild: false` turns it
  off. A rebuild repairs topology, so it is not tried for a closed result whose only issue is
  `out_of_tolerance` (that is the patch budget) unless the scan shows some topology evidence (genus
  16 or more, or 5 or more non-manifold edges). The rebuild is made for at most the 4 largest parts.
- **Extra attempts of one call** are counted in one place, a budget of 2 per call (a count of
  attempts, no clock, so the result does not depend on the machine's speed): the rebuild-and-wrap-again,
  the larger-cap retry (`allow_retry`: for the first part, in area order, whose first result is capped by
  the patch limit and short of the tolerance, and only in that part's first attempt) and the other method of
  `auto` all draw from it, whichever part asks first; when it is spent they are not run (an `auto` attempt
  that was not run is listed with its reason). A part's own wrap is not an extra attempt (a part the
  primitives could not close is wrapped as freeform once, as its one wrap; a scan with strong topology
  evidence is rebuilt and that wrap is made on the rebuilt scan). The worst case is therefore 26 wraps
  per call: one for each of the 24 parts and the 2 extra attempts, and at most 4 rebuilds. A part that
  was refused shows it: `components[].retry` with reason `budget_spent`, a `remesh` step in
  `components[].preparation` saying "not done: the call has used its 2 extra attempts", or the
  skipped entry in `components[].attempts`. For a split scan the top-level `attempts` and `retry`
  are those of the largest part; every part's own are in `components[]`.
  `mesh.remesh` makes a closed, manifold, outward oriented surface at a voxel of 1.5 x tolerance
  (finer for a thin wall, coarser for a very large area): tunnels, bridges, fins, coincident second
  shells and self-intersections up to two voxels wide are removed, genuine handles and walls of at
  least two voxels beyond that are kept, and the vertices are projected back onto the scan. It
  reports the deviation from the scan both ways (exact up to 100 000 vertices, else an even sample)
  and `removed_share`, the share of the scan farther from the result than the tolerance, which is
  what was removed (or lost), not error. It refuses an open scan (a hole wider than `max_hole`,
  default 5 mm, or a loop of more than two edges that was not filled (a missing triangle is one)), a resolution beyond its
  limits (16 million grid columns, 40 million ray crossings, 6 million quads; fewer on 32-bit
  targets), and a wall the voxel cannot resolve when `voxel` is given.
- A scan that already works is not touched: no preparation step, the same result as before. The
  piece analysis that decides this is O(n log n) on the cleaned scan (see the PR for the time on a
  million triangles).

## Second run

- Correct the regions (`region.merge`, `region.split`, `region.grow`, `region.insert`), then call
  `scan.to_solid` with `reuse_regions: true`.
- `edit.undo` takes a whole run back, the cleaning included.
