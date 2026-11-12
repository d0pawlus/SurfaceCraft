# Fit freeform surfaces

Fit NURBS patches to regions that are not planes, cylinders or other primitives, or build a
patch network over a whole scan.

## A whole scan

- `surface.wrap` with `mesh`: a watertight network of bicubic patches over the whole scan, for
  castings, body panels and organic shapes. `tolerance` (default 0.2 mm), `min_within` (default
  0.99) and `max_patches` (default 12000, at most 60000). The result is a solid when the scan is
  closed, else a sheet; `deviation` gives `within`, `rms`, `p99` and `max`.
- `refine` (default `adaptive`): the layout is refined only around the scan points the patches
  miss, so smooth areas keep large patches; `uniform` halves the budget of the whole scan each
  round (more patches for the same tolerance). `levels` lists the patches and the share within
  the tolerance of every round, `refine` the layout used.
- `max_patches` is a hard bound of every round, the first included (`cap_used` echoes it): no
  network has more patches. A round that would exceed it is not dropped: the largest refinement
  that fits is built (adaptive: the most deviating scan points first; uniform: the budget between
  the two rounds), within 90 % to 100 % of the cap when the search lands there (at most 4
  reductions of the scan, `probes` in `levels`), and `levels[].partial` says why a round was cut.
  The best round is the result, so a larger `max_patches` never gives a lower `deviation.within`.
  `capped` is true when the cap cut a round and the tolerance is still missed. A scan whose
  surface carries thousands of tiny handles cannot be reduced by edge collapse; its coarse mesh
  is then clustered on a grid, which keeps the cap at the price of a coarser network.
- It keeps sharp edges and thin walls (the patches meet G0). `scan.to_solid` with
  `method: freeform` runs it after cleaning and measures the result with `body.compare`.

## Regions

- Make the region first: `region.auto_segment`, `region.grow` or `region.create` (see
  `extract-primitives`).
- `surface.fit` with `region`, or with `mesh` and `triangles`. `tolerance` is the target maximum
  deviation in millimetres (default 0.2). `degree` defaults to 3. `control` `[u, v]` fixes the
  control net; without it the smallest net that meets `tolerance` is used.
- The result gives `within_tolerance`, `rms` and `max`. Read `warning` when it is present.
- By default the sheet is the whole rectangle of the fitted surface. `trim: true` cuts it to the
  outline of the region instead (outer loop and holes). The outline is smoothed and simplified
  to `outline_tolerance` (default 0.1 mm: no boundary vertex of the region farther than that from
  the trim edge), and holes smaller than five tolerances across are ignored; `outline` in the
  result says what was kept. If the outline cannot be used the sheet stays untrimmed and
  `trim_error` says why.
- A trimmed fit reports three separate numbers: `surface_deviation` (region points to the
  spline, the old `max`), `outline_deviation` (region boundary vertices to the trim edges, in the
  surface) and `delivered_deviation` (region points to the tessellated trimmed sheets).
  `within_tolerance` is true only when every one that applies is within its bound, the sheet is
  trimmed when `trim` was asked for, and no part of the region (`islands_ignored`, `skipped`) was
  left out.
- A region that wraps around (not a height field over one plane) is refused. `split: true`
  splits it into height-field pieces (`max_piece_angle`, default 45 degrees; `max_pieces`,
  default 16), fits each one and trims it to its part of the region. The result lists `pieces`
  with their own `max`, the `gaps` between neighbouring sheets and `gap_max`. This is a split,
  not a G1 network: the sheets meet within the gap but are not tangent-continuous.
- `surface.info` gives the control net, degree and deviation of one surface.
- Leave a part of the region out of the fit without changing the region or the mesh:
  `exclude_triangles` (mesh triangle indices), `exclude_spheres` (`center`, `radius`) and
  `exclude_boxes` (`min`, `max`). A triangle is inside a sphere or box when its centroid is. The
  excluded triangles do not pull the surface and their points are in none of the deviation
  numbers, so a boss, a scanner artefact or a neighbouring feature does not decide `max`. The
  trim outline still follows the whole region, so the sheet runs over the excluded part.
  `exclusion` in the result echoes the masks and says how many triangles and points they took
  out; indices outside the region are ignored and counted in `indices_ignored`. Bad masks (an
  index beyond the mesh, a negative radius, an inverted box, no points left) are errors.
- `samples` counts the points of the region: `eligible = measured + excluded + failed`.
  `measured` were used and checked, `excluded` were left out by a mask, `failed` could not be
  projected onto the spline or belong to a part that was left out (see `skipped`).
- `coverage` tells where the patch is measured. `grid` is up to 32 rows of up to 32 letters over
  the rectangle of the fit (`origin`, `u_axis`, `v_axis`, `u_range`, `v_range` place it; row 0 is
  the lowest v): `M` has scan data under it, `H` lies inside the outline of the data but has
  none (a hole the patch bridges, or a part you excluded), `E` lies outside the outline (the
  patch only extrapolates). `shares` gives the area share of each class (of all sheets, weighted,
  when the region was split); each piece also has its own `coverage` shares. Do not trust the
  deviation where the coverage is `H` or `E`.

## Curve networks

- `surface.from_curves` with `region` or `mesh`: cuts at sharp edges (dihedral angle above
  `angle`, default 35 degrees) and fits one Bezier patch to every four-sided smooth region,
  bounded by the shared feature curves. `degree` (2 to 8, default 5), `min_triangles` (default
  30) and `tolerance` apply.
- Regions with 3, 5, 6 or more corners are handled by splitting the whole network the same way:
  every feature curve is cut at its midpoint, every region gets a centre and one curve from it
  to each side midpoint, and a region with n corners becomes n four-sided patches (a network of
  four-sided regions only is not split). The patches share their boundary curves, so the
  network stays watertight. A crease of a scan that is resampled coarsely is blurred over two
  triangles; lower `angle` (for example to 20 degrees) when creases of about 55 degrees are
  missed.

## Quad layouts

- `quad.create` with `region` or `mesh` and `target_quads` (default 64): a structured layout,
  snapped to the scan.
- `quad.retopo` with `region` or `mesh`: a field-aligned quad-dominant layout over a closed or
  wrapped region. `target_quads` sets its density. With `conform_boundary: true` (experimental, off by default) the layout of an open region is
  conformed to the outline of the region: the outline is smoothed, its corners are kept, and it is
  pushed outward just far enough to contain every vertex of the region. The layout's free
  boundary is moved onto it, and a ring of faces closes the gap where the lattice stopped short
  (see `outline_conformed`, `outline_loops_conformed` and `outline_ring_faces` in the result).
  Every change is validated; if any quad of the final layout is improper (not a simple polygon of
  positive area), the plain layout without conforming is returned and `outline_conformed` is
  false with the reason. `quad.split` and `surface.auto` take the same parameter and then put
  split boundary vertices back onto the outline. Every split (conforming or not) is checked on the
  real geometry and rejected as a whole if it would break a proper quad (`split_rejected`,
  `splits_rejected`). Quads whose patch would run
  against the scan are turned around (`repaired_patches`); `flipped_patches` counts those that
  could not be repaired.
- `quad.snap` projects the cage onto the scan. `quad.split` and `quad.merge` change the density.
- `quad.crease` marks crease lines `u_lines` and `v_lines` (their indices), so patches meet
  without smoothing there. `remove` clears them.
- `surface.auto` with `object` (a layout) makes one patch per quad: `tolerance` (default 0.2
  mm), `name`, `make_solid` (also a solid when the network is closed, default true), and
  `max_patches`. Patches that run against the scan are repaired before the network is built
  (`repaired_patches`); if one cannot be, the command fails instead of returning a network with
  a flipped patch.

## Trim and thicken

- `surface.trim_to_region` with `object` (the surface) and `body` (a body with a face on it):
  the sheet of that face, trimmed to the region contour.
- `surface.thicken` with `object`, `thickness` (negative for the other side) and `symmetric`
  (half on each side) turns a surface into a solid body.

## Check

- A fit is good when `within_tolerance` is true. If it is false, read `warning`, then fit again
  with a larger `control` net.
- `deviation.compute` of the scan against the new surface gives the deviation over the whole
  region (see `check-deviation`).
