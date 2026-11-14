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
  The best round is the result. A larger `max_patches` usually raises the share of points within the
  tolerance (the `within` field of the result's deviation), but that is not guaranteed.
  `capped` is true when the cap cut a round and the tolerance is still missed. A scan whose
  surface carries thousands of tiny handles cannot be reduced by edge collapse; its coarse mesh
  is then clustered on a grid, which keeps the cap at the price of a coarser network (`clustered` is
  true in the result). When the clustering would lose more than half of the surface, or leave fewer
  than a tenth of the cap, the command fails and asks for a larger `max_patches`.
- The scan tells creases from smooth edges: an edge is a crease when the scan normals on its two
  sides differ by more than `crease_deg` (default 35, at most 60; a thin wall's two faces are two
  sides, never averaged). With `continuity: g0` (the default) the patches meet in position only.
  `continuity: g1` is **experimental, opt-in**: the patches are then made tangent across the smooth
  edges and meet at the full angle across the creases. On a corpus of 13 real scans (tolerance
  0.2 mm, cap 12000 patches) `g0` reached 99 % of the points within tolerance on 9 scans, `g1` on 7
  (two more were capped at 95.6 % and 98.9 %); on the scans that stay above 99 % `g1` needed 56 %
  more patches in total (16 % to 145 % per scan) and 2 to 3 times the wall time; 15 % to 70 % of the
  patches fell back to the unconstrained construction (`g0_patches`, poorly shaped coarse
  triangles); the mean smooth-edge jump was 39.9 degrees with `g0` and 42.5 with `g1`, p99 175 to
  180 degrees in both (these statistics are polluted by inconsistent patch orientation and are to be
  re-measured after the orientation fix). On smooth synthetic parts `g1` brings the jump to under
  1 degree. `crease_edges` and `smooth_edges` count the coarse edges of each kind, `smooth_jump` is
  the normal jump left across the smooth patch edges (`mean_deg`, `p99_deg`, `max_deg`, and the
  samples with an undefined normal, `invalid_samples`/`invalid_edges`), `crease_angle_mean_deg` the
  angle kept across the creases, `g0_patches` the patches that could not be made tangent.
- `scan.to_solid` with `method: freeform` runs it after cleaning and measures the result with
  `body.compare`.

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
- `ngon` says what happens to regions that are not four-sided. `split` (the default) cuts each of
  them into four-sided patches around a centre, so the network stays watertight and a closed
  network also gives a solid. `patch` keeps the four-sided regions as Bezier patches and fits every
  other region with one outline as a single trimmed patch of its own, with `surface.patch`:
  its sides are its feature curves (soft: the patch follows the scan, and `side_gap_max_mm` in the
  result is the largest distance from them), its data are the region's vertices. Each trimmed
  patch is a separate surface object named `<name>_patch_<k>` and is listed in `trimmed_patches`.
  Such a network is reported open and no solid is made, because the trimmed patches are separate
  sheets (see `build-solid`). In `patch` mode regions with one or two corners are still skipped.

## Constrained patch

- `surface.patch` fills one closed outline with one trimmed bicubic patch. The outline is `sides`,
  in any order. A side is a body edge (`body`, `edge`) or a polyline (`points`, at least two
  points, two points being a straight segment). The sides are chained into one loop. A gap between
  two ends that is larger than `gap_tolerance` (default 0.01 mm) is an error that gives the gap and
  its position; smaller gaps are closed. A polyline of three or more points is densified along a
  smooth cubic interpolant through its points, so the patch is not pulled towards the chords; a turn
  of more than 45 degrees at a point divides the side there.
- Use it for one outline: a hole or a missing piece bounded by edges of a body or by curves traced
  on a scan, where the new surface has to meet its neighbours. For a whole network of feature
  curves use `surface.from_curves`.
- Side kinds. `soft` false (the default) makes a hard side: the patch has to meet it within the
  targets (gap 0.001 mm, angle 0.05 degrees, normal curvature 5 %). `soft: true` makes the side
  follow the scan: the patch comes as close to it as the data allow, and the gap is reported, not
  a target. Sides traced on a scan are soft, because a scanned edge is rounded and noisy, and a
  hard G0 on it makes the fit worse than the scan noise.
- Continuity per side, `continuity`: `g0` (the default, position only), `g1` (the same tangent
  plane along the side) or `g2` (also the same normal curvature across it).
- The reference face. G1 and G2 continue a neighbouring face. A body edge uses its only face (the
  face on the other side of the edge); if the edge has more faces, give `face: {body, face}`. A
  polyline has no face of its own, so G1 or G2 on a polyline needs `face`. Before the fit the side's
  points are moved onto the reference face; `snap_max_mm` reports how far.
- Corners. Where two G1 or G2 sides meet, their tangent planes must agree. `conflicts: "relax"`
  (the default) relaxes the tangent conditions over a short stretch of each side next to the corner
  and reports it (`corners`: the mismatch angle and the relaxed length). `conflicts: "fail"`
  refuses instead.
- Interior conditions: `points` (`hard: true` makes the patch pass through the point; otherwise
  `weight` sets how strongly it is pulled to the point) and `curves` (polylines the patch must
  contain, G0).
- Scan data: `region`, or `mesh` with `triangles`, gives the scan points, fitted softly. The
  parameters of the patch come from a flattening of the region (`parameters: "flattened"`) when
  every side point lies within `gap_tolerance` of a boundary vertex of the region, that is, when the
  sides are the region's own edge. Otherwise the plane of the outline is used (`"plane"`). On the
  plane route an outline whose projection crosses itself, or a reference face more than 75 degrees
  away from the plane, is refused.
- Net size: `control` sets the start net `[u, v]`, `max_control` (default 16, at most 24) bounds
  how far it grows.
- Cost bound. With g1 or g2 sides the solve is dense over three unknowns per control point. Its
  time and memory are estimated from the net and the number of hard conditions before it runs:
  one solve may take at most about 8 s (a work estimate, the same on every machine) and 512 MB
  (256 MB in the web build, which allows a coupled net of 20 x 20 but not 24 x 24). Measured on the
  development laptop: 16 x 16 takes 1 s and 72 MB, 24 x 24 with 700 hard rows 4 to 5 s and 280 MB,
  24 x 24 with many hard rows up to 66 s and 430 MB (refused). A start net or a set of hard points
  over the bound is an error that says so; a net that would have to grow past it stops at the net
  reached (`cost_limited`, and a warning). A call can be cancelled; it then adds nothing.
- Regions of more than 100 000 triangles (`max_flatten_triangles`, at most 400 000) are not
  flattened (about 3 s for 100 000 triangles, 47 s for 400 000): the plane of the outline is used,
  a warning says so, and if that fails too the error repeats the note.

Reading the report:

- `within_tolerance`: the largest distance of the scan points from the patch (`max`) is within
  `tolerance` (default 0.2 mm). `rms`, `p99` and `max` give the distribution; `points` counts the
  scan points used and `outside` those lying outside the outline (ignored).
- `within_targets`: every hard side meets its targets, the surface does not fold, the hard
  conditions do not contradict each other (`hard_residual_mm`), and no corner had to be relaxed.
  A fold is where the surface normal vanishes or turns by more than a right angle between
  neighbouring samples; a deep dome or more than half a turn of a cylinder is not a fold, a warning
  only notes that the patch is no graph over the outline's plane. Each side's `met` says whether it
  met its own targets. False is a result to read, not a failure: `sides` and `corners` say which condition
  missed and by how much.
- `sides`, one per side in loop order: `gap_max_mm` (the largest distance of the side from the
  patch), `angle_max_deg` (G1 and G2), `curvature_error_max` (G2, relative), `samples`, `soft`,
  `snap_max_mm`, and `folded`. A folded side means the patch turned back over the reference face:
  that is an error of the side, not a success.
- `control`, `rounds` (solve and refine passes), `dropped` (conditions found dependent and left
  out) and `warnings`.

Example 1: G1 on the edges of a hole. `tube` is a body whose five edges bound a five-sided hole
in a cylindrical sheet. Each edge belongs to one face, the cylinder, so no `face` is needed:

```json
{
  "command": "surface.patch",
  "params": {
    "sides": [
      {"body": "tube", "edge": 0, "continuity": "g1"},
      {"body": "tube", "edge": 1, "continuity": "g1"},
      {"body": "tube", "edge": 2, "continuity": "g1"},
      {"body": "tube", "edge": 3, "continuity": "g1"},
      {"body": "tube", "edge": 4, "continuity": "g1"}
    ],
    "tolerance": 0.2,
    "name": "hole_patch"
  }
}
```

Read `within_targets` and each side's `angle_max_deg` (the target is 0.05 degrees).

Example 2: polyline sides traced on a scan, with the region as data. The coordinates are
illustrative, on the surface z = 3·sin(x/12)·cos(y/9); real sides are points of the region's
boundary, so that the flattening applies:

```json
{
  "command": "surface.patch",
  "params": {
    "sides": [
      {"points": [[0, 0, 0], [10, 0, 2.24], [20, 0, 2.99]], "soft": true},
      {"points": [[20, 0, 2.99], [20, 12, 0.70], [20, 24, -2.66]], "soft": true},
      {"points": [[20, 24, -2.66], [10, 24, -1.99], [0, 24, 0]], "soft": true},
      {"points": [[0, 24, 0], [0, 0, 0]], "soft": true}
    ],
    "region": "wave_scan",
    "tolerance": 0.2,
    "name": "wave_patch"
  }
}
```

Here `within_tolerance` says whether the scan lies within 0.2 mm of the patch, and `sides` gives
the gap of each traced side (not a target, since the sides are soft). `parameters` tells which
parameters were used.

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
  (half on each side) turns a surface into a solid body. The kernel refuses a surface of many
  faces (a wrapped scan): for an open scan, thicken on the mesh side instead.
- `mesh.thicken` with `object` (an open mesh), `thickness` (mm; positive puts the material behind
  the scanned face, negative in front of it), `symmetric` and `name` makes a new closed mesh of
  constant thickness: every vertex is offset along a smoothed normal, the offset copy is turned
  around and the rim and the windows are stitched. Where the surface curves more tightly than the
  thickness the offset is reduced locally (`clamped_vertices`, `largest_reduction`); a thickness
  that folds most of the surface is refused. A sample of the result is tested for self-intersection
  (`self_intersections`): a channel narrower than the wall shows there. `surface.wrap` of the result is a solid. In one call:
  `scan.to_solid` with `thickness` (see `scan-to-solid`), which measures the solid against the scan.

## Check

- A fit is good when `within_tolerance` is true. If it is false, read `warning`, then fit again
  with a larger `control` net.
- `deviation.compute` of the scan against the new surface gives the deviation over the whole
  region (see `check-deviation`).
