# Align a scan

Bring a scan mesh into the model frame before any fitting. Each align command moves the mesh
(and the meshes in `also`) by one rigid motion. A motion is one undo step.

Over MCP every command is also a tool with the dot replaced by an underscore (`align.icp` is the
tool `align_icp`), or call `execute` with `command` and `params`.

## Import

- `mesh.import`: `path` (STL, OBJ, PLY or XYZ) or base64 `data` with `format`. Over MCP file
  paths are off unless the server runs with `--allow-files`, so send `data` inline.
- `scale` multiplies the coordinates on import (for an odd factor). `name` names the mesh.
- `mesh.info` reports triangles, bounding box, area and `source_units`. Check the size with it first.

### Units

STL, OBJ, PLY and XYZ files carry no length unit, and SurfaceCraft works in millimetres. Say which
unit the file is in with `units`: `mm`, `cm`, `m`, `in`, `ft` or `um`. Without `units` the file is
taken as millimetres, as before. The mesh is converted to millimetres on import (exact factors: cm
10, m 1000, in 25.4, ft 304.8, um 0.001), `scale` multiplies on top of it, and the object keeps the
unit in `source_units`. The import result carries `scale_factor` and, in words, `units_note`.
Normals, colours and per-vertex data go with the coordinates.

- `mesh.import` with `"units": "m"` for a scan saved in metres. `mesh.reconstruct` and
  `mesh.import_stream` take `units` too; the store records its unit (`mesh.store_info` shows
  `units`) and is written in millimetres.
- A size that looks like a unit mistake (largest side below 1 mm, or above 20 000 mm) comes back as
  `scale_warning` (code `suspicious_scale`) with the calls that fix it. It is raised only when the
  unit was NOT stated: an explicit `units`, including `mm`, and a size set by `mesh.scale_from_length`
  are taken as they are. The same warning is in `mesh.info`, `scan.qa` and `scan.to_solid`; in
  `scan.to_solid` it is one issue and a `pass` becomes `review`. The rule: the largest axis extent
  of the vertices; a mesh of 100 vertices or more has its most extreme vertices cut at each end
  (one per hundred), and a larger mesh is measured on 100 000 vertices taken evenly through the
  list. Below 1 mm: metres or inches; below 0.02 mm: definitely not millimetres; above 20 000 mm:
  micrometres or another unit.
- `mesh.set_units` states the unit of a mesh imported in the wrong one: `from` is the unit its
  coordinates really are in. The factor is taken from the unit the mesh has now, so `cm` can be
  corrected to `m` (factor 100), `m` to `mm` (factor 1/1000), and so on. `from` equal to the unit
  it has now only acknowledges it and rescales nothing; this also clears the warning. Regions,
  primitives and quad layouts over the mesh are rescaled with it, a fitted surface refuses the
  conversion, and a mesh sized by `mesh.scale_from_length` refuses it. One undo step.
- Run `mesh.set_units` straight after the import: it scales the mesh about the origin, so an
  alignment or a `mesh.transform` made before it is scaled too. While the document holds bodies,
  sketches or features (they are not rescaled, and they are not linked to a mesh, so every one of
  them counts) `mesh.set_units` refuses unless `force` is true; the result's `left_alone` counts
  what was left at the old size. `mesh.scale_from_length` reports `left_alone` the same way.
- Export (`mesh.export`, STEP, IGES) writes millimetres, as before.

## Scale (photogrammetry)

- `mesh.scale_from_length`: a scan of unknown size is scaled uniformly so that the distance
  between two points on it, `from` and `to`, becomes the true `length` in millimetres. The scaling
  is about `origin` (default: the mesh centroid); the result reports the `factor`. Coincident
  points, a length that is not a positive number and a factor outside 1e-6..1e6 are errors. What
  depends on the mesh is scaled with it: the RMS of its regions, the primitives fitted to those
  regions (positions, radii, RMS; directions and angles stay), and quad layouts over it (cage
  vertices, frame origin, domain). Fitted freeform surfaces cannot be rescaled: if one was fitted
  to a region of the mesh the command is refused, naming it, and nothing changes (delete it and
  fit again after scaling). Bodies and primitives or surfaces without a region link are not tied to
  a mesh and are left alone. One undo step.

## Coarse alignment

- `align.auto`: a pose from the scan alone, for a part with no alignment yet. It proposes by
  default (`apply` false); `apply: true` moves the mesh in one undo step. A proposal (any align
  command with `apply` false) records no history entry: undo depth and redo stay as they were. It
  is a heavy command: the GUI's live preview does not run it.
  - How it works: the triangle normals vote, weighted by area (each face takes the blend of its
    corners' vertex normals, so noise averages out); every dominant direction is refined to an
    exact planar face (trimmed plane fit, so spikes and debris do not count). An open flat rim, a
    planar boundary loop, stands for the face the scan did not capture and votes with the area
    it encloses.
  - The part is laid on its largest face that it can stand on (nothing of the part beyond that
    face): the plane becomes z = 0, the part above it, and the origin is the centre of that
    face. The longer side of the footprint lies along x (from the side faces, or from the
    footprint if the sides give nothing). A turned part (rotational symmetry: the normals lie on
    a great circle plus two poles, and the outer radius about that axis is the same, within 3 %,
    in each of 12 sectors of the circumference, measured on the vertices) gets its axis on z with its larger end face on z = 0; the
    rotation about the axis is free and the least rotation from the current pose is taken.
  - Ties are settled the same way every time. Faces within 3 % of the largest area are tied: an
    open rim wins, otherwise the pose needing the smaller rotation from the current one. Of the
    two directions of x, the one with more material at +x is taken: the surface centroid
    weighted by height above the base is compared with the base centre; when that is a tie in x
    the same test is made for y, and when it is a tie too, the smaller rotation wins. A part
    with an exact symmetry (a filleted block, a plain cylinder) therefore comes out in any of its
    equivalent poses.
  - An elliptical extrusion has normals all around too but is not a body of revolution: it goes
    down the planar path (an open rim or its largest face is the base, the long side of the
    footprint lies on x). A turned part with a narrow keyway or notch (narrower than one 30 degree
    sector) stays turned; a flat or cut wider than a sector, or deeper than 3 % of the radius, makes
    it prismatic: it is laid on its largest flat face.
  - A face counts as one the part can stand on when at most 2 % of the surface area lies beyond it
    (more than twice the slab width away on the wrong side); triangles crossing that distance are
    clipped, so the decision does not depend on how the surface is triangulated.
  - `mode`: `largest_face` (default) puts that face exactly on z = 0; `all_faces` rotates the
    frame by the area-weighted least-squares compromise over all faces within 10 degrees of an
    axis, so a part whose top is 1 degree off its base is tilted by a fraction of that degree
    instead of keeping the base exactly flat. The base centre stays on z = 0. `mode` has no effect
    on a turned part, whose axis comes from its end face.
  - The result carries the 4x4 `matrix`, `angle_degrees` and `translation_mm` (how far it moves
    the part), `method` (`planar_face`, `open_rim`, `turned` or `principal_fallback`), the `base`
    face (area, share of the surface, normal, centre, whether it is a rim, `overhang`), `axis`
    for a turned part, `in_plane` (how the rotation about z was found) and the `confidence`:
    the share of the surface area within 10 degrees of the frame's axes (of its axis and
    equator for a turned part). Below 0.5 `confident` is false and the proposal is a guess.
  - No dominant flat face (a smooth blob): the principal axes are proposed as
    `principal_fallback` at low confidence, never an error.
  - Limits: a mesh of spikes or heavy noise should be cleaned first (`mesh.clean`); two parallel
    faces at different heights compete as separate faces (only the larger counts); a footprint
    that is square within 1 % has no longer side, so x keeps the angle the side faces gave.
- `align.symmetry`: settles the part on its mirror plane. It fits the symmetry plane (as
  `primitive.symmetry` does) and moves the part so that plane becomes `plane`: `yz` (x = 0,
  default), `xz` or `xy`. The normal turns to the chosen axis by the shortest arc (the other axes
  stay as close as they can to where they were) and the part slides along that axis only. The
  result reports the `symmetry_plane` found and `residual_rms`, the RMS distance in millimetres
  between the mesh and its mirror image (near the scan noise for a symmetric part). `apply` false
  by default.

- `align.principal`: principal axes to X, Y and Z, centroid to the origin. Use it first when
  the part has no reference face.
- `align.center`: moves the bounding-box centre to the origin, nothing else.
- `align.datum`: 3-2-1 alignment. `primary` fixes +Z, `secondary` fixes +X, `tertiary`
  completes the origin. A datum is `{"plane": {"point": [x, y, z], "normal": [x, y, z]}}`,
  `{"axis": {"point": [..], "direction": [..]}}`, `{"point": {"point": [..]}}`, or
  `{"plane_from_triangles": [..]}`, a plane fitted to scan triangles (with `mesh` if not the
  object).
- `align.n_point`: `from` points on the scan and `to` points in the model, at least three pairs,
  least squares. Use it with points picked in the viewport.

## Fine alignment

- `align.icp`: point-to-plane ICP of `object` onto `target`, a mesh, body or primitive.
  `options` takes `max_iterations` and `tolerance` (mm per iteration). The result says whether
  it `converged` and how many `pairs` it used.

## Two scans of one part

- `align.pair_coarse`: brings the scan `object` onto the overlapping scan `target` (mesh or body)
  with no picked points, then refines with ICP. It tries the 24 signed permutations of the two
  principal frames (whole parts) and poses from local shape descriptors (FPFH) with RANSAC (partial
  overlap, down to about half shared), scores them on a subsample, refines the best few and reports
  the pose, `overlap_moving` and `overlap_fixed` (shares of each scan's area that lie on the other
  within `tolerance`), a `verdict` and a `confidence`.
- `verdict`: `unique` (one pose clearly fits better than any other: every other pose has at least
  3 % of the moving scan's area more in conflict or less on the target, and the pose survives being
  pushed), `ambiguous` (a symmetric part: the near-tied poses are in `alternatives`; or a part that
  can slide or turn freely, such as a flat or cylindrical patch) or `no_match` (too little overlap,
  or different parts).
- The push: every pose is displaced by 0.2 of the moving scan's extent along each of its three
  weakest directions (the eigenvectors of its point-to-plane system), in both senses, and refined
  again: 6 extra ICP runs, about 0.7 s on a scan of a million triangles. A start that settles in a
  different pose (more than 2 % of the diagonal away) that does not clash with the target by at
  least 2 % of the moving scan's area (`probe_threshold`; running off the end of a cylinder is no
  clash) is a slide (`slides`) and is listed as an alternative; a start that flew off (lost 30 %
  of the matches) or whose ICP found no pairs (`probes_failed`) says nothing: with any failed start,
  or none tried, the pose is never `unique`; if all come back the pose is held. No verdict depends on how loose
  the pose reads (`stability`: 0.2 for a bracket, 0.01 and up for real scans; below 0.001 the part
  is free and always `ambiguous`).
- It proposes by default (`apply` false); a pose that is not `unique` is applied only when
  `choose` names it (0 the best, 1 and up the alternatives; `choose` needs `apply`). The result then
  carries the numbers of the pose chosen. When `apply` was given and nothing moved, `refused` is true.
- Limits: the triangles of both scans must be consistently oriented (the descriptors use the
  normals); surface that ends at the border of the other scan is no conflict, but a part whose
  shared region is only flat or a cylinder-like surface is reported `ambiguous`; a scan that
  repeats its vertices per triangle gets the borders of the welded mesh (found by position).

## Many scans of one part

- `align.global`: `objects` (2 to 24 meshes, each once) are brought into the frame of `fixed`
  (default the first), with no picked points. Every pair is solved as `align.pair_coarse` does
  (for up to 9 scans all pairs; for more, the pairs of neighbours in the list first, cut at
  `max_pairs`, default 36: list scans in the order they were taken, or raise the cap). Only a
  `unique` pair with enough overlap is an edge; `ambiguous` and `no_match` pairs are not.
  `start: current` skips the search for scans that are already roughly in place: every pair starts
  from the current poses, and pairs whose boxes do not meet are skipped.
- The graph: the edges are weighted by overlap over squared residual; the spanning tree from the
  fixed scan (strongest first) chains the pair poses (`chained_matrix` of every scan); then all
  poses are relaxed together (Gauss-Newton on virtual point pairs: the points of one scan that lie
  on the other at the pair pose), so the error around a loop is spread over the loop. A pair pose
  that disagrees with the best consistent set of the others by more than 2.5 tolerances (a false
  `unique`) is `rejected` and left out.
- The report: per scan `placed`, `held_by_loop` and its `matrix`; per pair the verdict, overlaps,
  residual, the pair's own evidence (`stability`, `probed`, `probes_failed`, `slides`), whether it
  was used and its misfit before and after (`misfit_before`, `misfit_after`; also for a `rejected`
  pair, so the size of the contradiction is visible); `used_edges`, `rejected_edges` and
  `consensus_margin` (inliers of the winning set minus those of the best different one; 0 is a tie);
  `loop`: over the independent cycles, the largest edge misfit in each, worst and mean, before and
  after relaxation (before, the whole error sits on the edge that closes the cycle).
- The verdict is `aligned` only when every scan is placed and held by a closed loop of pairs
  (`held_by_loop`), every used pair agrees with the final poses within its tolerance, the vote that
  left pairs out was not a tie, and the left-out pairs are not more than 1.5 per used one. A scan
  that hangs on a single pair (two scans, a chain, a leaf) is `weakly_held`: the pair was `unique`,
  but `pair_coarse` is known to return a wrong pose as `unique` on near-symmetric or self-similar
  parts, and nothing else confirms this one. The verdict is then `partial`, and the note quotes the
  pair's confidence, overlap, stability and probe result: check such a scan by eye. `partial` also
  for scans not connected to the fixed one (`unplaced`, never moved), a loop that does not close
  (`loop_error_high`), `many_edges_rejected` and `consensus_unclear`; `failed` when nothing connected.
- It proposes by default; `apply: true` moves every placed scan in one undo step (also for a
  `partial` result: the note then names the weakly held scans), and refuses when the verdict is `failed`.
- Limits: a scan tied by a single wrong pair cannot be told from a right one (no loop checks
  it); a part with near symmetries gives pairs that are `unique` but wrong, and the consensus can
  only drop them while enough right pairs exist; the cap on pairs can leave a scan unconnected
  (`pairs_capped`).

## Parameters common to all align commands

- `object`: the mesh to move.
- `apply` (default true; false for `align.auto`, `align.symmetry`, `align.pair_coarse` and `align.global`, which propose first):
  false computes the motion without moving the mesh.
- `also`: other meshes moved by the same motion.

## Check

- The result reports the motion. `mesh.info` after an align command shows the new bounding box,
  to confirm the mesh is where you expect.
- `edit.undo` removes the last motion.
