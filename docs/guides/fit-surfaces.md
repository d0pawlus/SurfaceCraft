# Fit freeform surfaces

Fit NURBS patches to regions that are not planes, cylinders or other primitives, or build a
patch network over a whole scan.

## Regions

- Make the region first: `region.auto_segment`, `region.grow` or `region.create` (see
  `extract-primitives`).
- `surface.fit` with `region`, or with `mesh` and `triangles`. `tolerance` is the target maximum
  deviation in millimetres (default 0.2). `degree` defaults to 3. `control` `[u, v]` fixes the
  control net; without it the smallest net that meets `tolerance` is used.
- The result gives `within_tolerance`, `rms` and `max`. Read `warning` when it is present.
- By default the sheet is the whole rectangle of the fitted surface. `trim: true` cuts it to the
  outline of the region instead (outer loop and holes). The outline is smoothed and simplified
  to `outline_tolerance` (default 0.1 mm), and holes smaller than five tolerances across are
  ignored; `outline` in the result says what was kept. If the outline cannot be used the sheet
  stays untrimmed and `warning` says why.
- A region that wraps around (not a height field over one plane) is refused. `split: true`
  splits it into height-field pieces (`max_piece_angle`, default 45 degrees; `max_pieces`,
  default 16), fits each one and trims it to its part of the region. The result lists `pieces`
  with their own `max`, the `gaps` between neighbouring sheets and `gap_max`. This is a split,
  not a G1 network: the sheets meet within the gap but are not tangent-continuous.
- `surface.info` gives the control net, degree and deviation of one surface.

## Curve networks

- `surface.from_curves` with `region` or `mesh`: cuts at sharp edges (dihedral angle above
  `angle`, default 35 degrees) and fits one Bezier patch to every four-sided smooth region,
  bounded by the shared feature curves. `degree` (2 to 8, default 5), `min_triangles` (default
  30) and `tolerance` apply.

## Quad layouts

- `quad.create` with `region` or `mesh` and `target_quads` (default 64): a structured layout,
  snapped to the scan.
- `quad.retopo` with `region` or `mesh`: a field-aligned quad-dominant layout over a closed or
  wrapped region. `target_quads` sets its density.
- `quad.snap` projects the cage onto the scan. `quad.split` and `quad.merge` change the density.
- `quad.crease` marks crease lines `u_lines` and `v_lines` (their indices), so patches meet
  without smoothing there. `remove` clears them.
- `surface.auto` with `object` (a layout) makes one patch per quad: `tolerance` (default 0.2
  mm), `name`, `make_solid` (also a solid when the network is closed, default true), and
  `max_patches`.

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
