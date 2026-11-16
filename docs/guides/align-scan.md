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
- The `tolerance` (default) is five times the median distance of the best pose's points, but never
  more than four times the `noise` of the pair (each scan's own estimate, combined): a tolerance made
  of the pose's own residual lets a pose that merely lies on other regions of a lumpy surface pass
  as a fit. A coarse mesh makes the estimate unreliable (`noise` 0): the cap is then 5 % of the mean
  edge length of the coarser scan (the chord error between two tessellations is a small part of an
  edge), and a pair of coarse scans is certified less often.
- `conflict_moving` and `conflict_fixed`: the share of each scan's area (3000 samples of each, however
  small the scan) that lies in the free space of the other: the other scan is there, but the point
  is in front of its surface and more than twice the tolerance away. Surface beyond the end of the
  other scan, or behind its surface (the underside of a box seen from the top), says nothing. At the
  right pose both are near 0 (at most 0.001 on the test fixtures); a pose where either exceeds 0.01
  is never `unique`: it is `ambiguous` with a confidence of at most 0.25 and the note says why
  (`no_match` stays for too little overlap, a poor fit or conflicts of the moving scan: a weak patch
  that lands anywhere is `ambiguous`, not `no_match`). A patch laid flat on a large scan where that has a boss over its
  footprint reads 0 on its own side and the boss shows on the other. Needs triangles oriented outward.
- `verdict`: `unique` (one pose clearly fits better than any other: every other pose has at least
  3 % of the moving scan's area more in conflict or less on the target, neither scan contradicts
  the other, and the pose survives being pushed), `ambiguous` (a symmetric part: the near-tied poses are in `alternatives`; or a part that
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
- The orientation of the triangles is checked, because the conflicts read the side a triangle faces
  (a scan turned inside out makes every conflict vanish): a pose is never `unique` (it is
  `ambiguous` with a confidence of at most 0.25, and `orientation_issue` and the note say why) when a
  scan's triangles are wound against each other or duplicated (more than 2 % of its shared edges), when
  a closed scan has a negative volume, or when the normals of the two scans mostly oppose where they
  meet. What cannot be seen: two OPEN scans that are both turned inside out, consistently. Each is
  wound consistently, neither is closed and their normals agree, so nothing flags them and the
  contradiction evidence is blind for both: check the orientation of your scans (a shaded view shows
  an inverted scan dark) before you trust `unique`.
- Limit, in plain words: two pieces of a part that has a symmetry cannot be told from the same piece
  scanned twice, so such a pair can come back `unique` at a wrong pose (the pieces are congruent and
  every surface fits). Give a rough start (place the scans by hand: `align.global` with `start`
  `current` takes it), or scan more views, so that `align.global` finds the pose contradicted by the
  other scans.
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
- The verdict is `aligned` only when every scan is placed and held by a closed loop of strong pairs
  (`held_by_loop`), every used pair agrees with the final poses within its tolerance, the last
  vote that left pairs out won clearly (by 2 pairs from 6 pairs up, by 1 below), and the left-out
  pairs are not more than 1.25 per used one. A pair is strong when its residual is below 0.3 % of
  the scan's diagonal (`rms_per_diagonal`, `strong_evidence`; confidence does not count, it read
  1.00 on wrong poses); a loop confirms its scans only when all its pairs are strong. A scan that
  hangs on a single pair (two scans, a chain, a leaf) or only in loops with a weak pair is
  `weakly_held`: the verdict is `partial`, `weak_pair` names the pair, and the note quotes its
  confidence, overlap, residual, stability and probe result. Also `partial`: scans not connected to
  the fixed one (`unplaced`, never moved), a loop that does not close (`loop_error_high`),
  `many_edges_rejected`, `consensus_unclear`; `failed` when nothing connected.
- Read `aligned` for what it is. **It means the pair poses agree with each other and with the final
  poses, not that they are right.** `align.pair_coarse` is known to return a wrong pose as
  `unique` on parts with a symmetry or a self-similar surface (a half turn onto another region
  that fits at noise level), and a triangle of scans that are all placed by the same symmetry
  closes without any misfit: nothing in the data tells it from a right one. The command is only as
  reliable as `align.pair_coarse`; on a part with a symmetry check the result by eye.
- It proposes by default. `apply: true` moves the scans held by a loop in one undo step; a weakly
  held scan moves only when `accept_weak` lists it (after you have read its pair's evidence in
  `edges`), the way `align.pair_coarse` moves a non-unique pose only when `choose` names it. The
  result lists `moved` and `left_weakly_held`; with nothing moved there is no undo step and the note
  says why. `apply` is refused when the verdict is `failed`.
- Limits: a scan tied by a single wrong pair cannot be told from a right one (no loop checks
  it); a part with near symmetries gives pairs that are `unique` but wrong, and the consensus can
  only drop them while enough right pairs exist; the cap on pairs can leave a scan unconnected
  (`pairs_capped`).

## Merge scans into one mesh

- `mesh.merge`: `objects` (2 to 64 meshes, each once) that are already aligned in one frame
  (`align.global` with `apply`) become one new mesh. The scans stay as they are; the new mesh is one
  undo step. It is a heavy command (cancellable, run in the background from the GUI).
- `tolerance`: surface of two scans closer than this is one surface. The default is 4 standard
  deviations of the noise of the two noisiest scans together (each scan's own noise estimate, in
  `scans[].noise`) plus 5 % of the median edge of the coarsest scan (two tessellations of one curved
  surface differ by part of an edge even without noise). `tolerance_source` says `noise` or `given`.
- Where scans overlap, one layer is kept, never two and never an average. The scan in which that
  surface lies farthest from its own border keeps it (distance along the edges; a closed scan has no
  border and keeps what it covers). The border of a scan is its least reliable part: the scanner sees
  the surface at a grazing angle there, and the edge is ragged. So the seam falls midway between the
  borders. Distances within one median edge count as equal and the scan listed first keeps the surface,
  so two scans that see the same surface with the same border (a boss seen whole by both) do not hand
  it back and forth. A triangle with a corner another scan keeps is removed. Triangles on the edge of
  a kept region that still lie wholly on another scan's kept surface are removed too.
- The seam is closed by ball pivoting from the edges the removal cut. A seam triangle is built only on
  surface some scan saw: its centre and the middles of its edges lie within the tolerance (or a tenth
  of its longest edge, if that is larger) of a scan, facing the same way. It also crosses or covers no
  triangle already there. So no surface is made up over an opening nobody scanned, and the rims of two
  separate pieces are never joined. Small holes the pivoting leaves (up to 64 edges, with no edge of a
  scan's own border) are filled by ear clipping or a fan through the same checks, the corner rule
  included. Where the band has a twisted corner (a hole of a few edges no triangle can fill facing the right way), the seam triangles
  around it are taken back once and the larger hole is filled. Seam triangles that meet another fan
  at one vertex only (a pinch) are taken back the same way. No vertex is added to the scans' vertices
  and kept vertices stay where they are, except that a seam vertex of a later scan moves onto a
  coinciding seam vertex of an earlier scan, by at most a quarter of the tolerance (and a tenth of
  the median edge), measured from its own position (welds never chain). A weld that would flip,
  fold or double a triangle is undone (`checks.welds_undone`). Removed triangles whose corners all exist in
  the result at exactly their places are put back: pieces cut from one scan share their tessellation,
  and there the seam is a re-weld (a restored triangle must cross and cover nothing either). Removed
  triangles whose centre the seam leaves without result surface within the tolerance come back on their
  own scan's vertices (`seam.recovered`).
- `alignment`: `"refuse"` (the default) or `"warn"`. With `"warn"` the alignment findings below
  (`scans_misaligned`) are warnings with the same numbers and the merge proceeds (one undo step);
  the other refusals still refuse.
- Refused, with the numbers and nothing made (`refused` true, no undo step):
  - `scans_misaligned`: more than 10 % of the surface two scans share (where one lies on facing surface
    of the other, up to 8 tolerances away) lies farther apart than the tolerance. Merging them would
    either stack two layers or average them into a surface neither scan saw. Align better
    (`align.icp`, `align.global`) or give a larger tolerance knowingly. Also refused: scans offset by
    more than 8 tolerances, which share no surface within that reach but lie over each other (flat
    scans too, whose boxes do not overlap within the reach). They are found along each sample's
    normal: the first surface of the other scan faces the same way (`pairs[].over`, median distance
    `pairs[].over_offset`), at least 5 % of the samples lie over it, fewer than 50 agree with it,
    and no chain of scans that share surface ties the two (pieces of a part that lie over each
    other, a step, are not a misalignment). Scans that are apart
    in space are no refusal: they give a union of pieces and the notice `scans_disjoint`.
  - `opposed_normals`: the normals of two scans oppose where they meet. One is turned inside out.
  - `inconsistent_winding`: triangles wound against the rest of their piece cover more than 0.1 % of a
    scan's area (a patch turned inside out). Fewer are left out and reported as
    `flipped_triangles_dropped`, so the result never has a flipped part.
  - `inverted_scan`: a closed piece of a scan has a negative volume. Inner skins (inside another
    closed piece, as the inside of a hollow part or a cavity, whatever their size) and debris (loose
    small pieces below 1 % of the scan's area all together) are not refused; they are reported as `inner_skins` with their counts.
- Reported, not refused: `layers_apart` (two scans that meet somewhere also lie over each other:
  at least 14 % of the points of one that relate to the other, on it or over it, lie over it beyond
  the reach, with nothing of their own scan in between; the issue gives the median offset and its
  spread, and says whether it is one offset, a shift as a whole, or not, a tilt or a curved surface;
  check the alignment), `overlap_unjudged` (samples whose search along the normal ran out of its budget in a
  pile of triangles), `overlap_too_small` (two scans that touch by fewer than 50 points either way:
  not judged), `seam_over_budget` (seam or restored triangles left out because the crossing
  test would have compared too many triangles), `non_manifold_input` (exact duplicate triangles and triangles on edges of
  three or more triangles are left out and counted as removed), `flipped_triangles_dropped`, `scans_disjoint` (groups of scans that share no surface: the result is a
  union of separate pieces), `scan_fully_covered` (a scan lies wholly on others and adds nothing),
  `seam_open` (cut edges no triangle could close), `removed_not_covered` (less than 99 % of the removed
  area within the tolerance of the result), `layers_stacked` (more than 0.5 % of the scans' area in the
  result lies on another scan's surface: two layers left; with their triangle count), `unwelded_vertices` (vertices of different
  scans closer than 0.01 median edges with no edge between them), `unoriented_edges` and
  `pinched_vertices`.
- The report:
  - per scan: triangles `kept`, `removed`, `restored`, `collapsed` (two corners welded into one),
    `dropped`, `overlap_share`, `noise` (absent when the tolerance was given);
  - per pair: the residual where one lies on the other (`shared`, `within`, `rms`, `p95`, `worst`,
    `beyond_share`), the points over the other beyond the reach (`over`, their median distance
    `over_offset` and spread `over_spread`, `one_offset`), `unjudged` points, and the shares of the
    points that relate the two that lie over the other (`over_share`) and agree (`agree_share`);
  - `seam`: cut edges, length, triangles created, refusals by reason (`refused_unsupported`,
    `refused_crossing`, `refused_corners`, `refused_budget`), recovered triangles and the specks taken back again, what is still uncovered
    after the recovery (`uncovered_after_seam`),
    open edges and their length;
  - `before` (the scans together) and `after`: pieces, boundary loops, genus, non-manifold and
    unoriented edges, pinched vertices;
  - `checks`: the removed triangles are sampled by area (up to 2 million points; every triangle at
    least once) and measured against the result (`removed_within_share`, `removed_worst`). Also the
    vertices that moved and by how much, the triangles the merge added that cross another (0 by
    construction), `unwelded_vertices` and `stacked_share` (the share of the scans' area in the result
    within the tolerance of facing surface of another scan: two layers);
  - `stages` in milliseconds.
- The result is what `scan.to_solid` (method `freeform`) wraps: four noisy views of the organic test
  blob merge into one piece that wraps no worse than one full scan of the blob (in a few noise draws a
  small loop stays open where all views meet in a point; it is reported).
- Limits:
  - `mesh.merge` expects scans that are already aligned (`align.global`); its checks of the
    alignment are a best-effort safety net, not a proof. Correct input is refused only in the cases
    marked "refused" below, each with its remedy; every alignment refusal (`scans_misaligned`) can be
    turned into a warning with `alignment: "warn"` (the merge then proceeds with the same numbers in
    the issue). What the checks do not see, see only as a warning, or refuse although correct:
    - a scan slid along a flat or cylindrical surface (distances along it do not change);
    - a displacement along the normals larger than the other scan's diagonal: it comes out as
      `scans_disjoint`, not as a misalignment;
    - a displaced pair that both overlap a third scan agreeing with each: it cannot be told from a
      real step of the part, and is not refused;
    - a slender part slid along its own axis: only its end caps lie over the other (a share of
      about its radius over its length), under the 14 % that `layers_apart` needs: not seen;
    - a scan displaced along a face parallel to the displacement, tilted, or seen over the other from
      one side only: reported as `layers_apart` (at least 14 % of the related points over the
      other), never refused;
    - a shelf over a floor seen by different scans is the same geometry: it warns although correct;
    - refused: a stepped part whose levels share nothing reads as one scan displaced over the other.
      Remedy: a scan that sees both levels (a close-up of the step), or `alignment: "warn"`;
    - refused: a hollow part whose outside is seen by one scan and whose bore by another that share
      nothing: the bore's points look across the bore at the outside wall, which faces the same way.
      Remedy: a scan that sees both, or `alignment: "warn"`;
    - refused: a feature 1 to 8 tolerances high that only one of two scans resolves over more than
      10 % of their shared points (a floor under a low shelf, an emboss) reads as `scans_misaligned`.
      Remedy: a larger tolerance, or `alignment: "warn"`;
    - refused: a correct pair whose noise has heavy tails, or a ragged or curled border in a narrow
      overlap, once more than 10 % of its shared points lie beyond the tolerance (the default
      tolerance covers Gaussian noise only). Remedy: a larger tolerance, or `alignment: "warn"`;
    - pairs that touch by too little for a verdict (fewer than 50 points either way) are reported as
      `overlap_too_small`, not judged.
  - Two OPEN scans that are both turned inside out, consistently, cannot be told from right ones (as
    with `align.pair_coarse`).
  - A wall thinner than the tolerance, seen from its two sides by two different scans only, reads as
    opposed normals and is refused.
  - Where a seam runs into the rim of a separate piece (the test bracket is a box, a boss and a half
    sphere whose rims touch without sharing vertices) part of it can stay open. It is reported as
    `seam_open`, never closed by bridging the rims.
  - A small patch scanned at a finer resolution inside a large scan loses to the large scan (it is
    nearer its own border everywhere).
  - Meshes with triangles far longer than their median edge (CAD exports, heavy decimation): where such
    a triangle was removed the gap can be wider than the pivoting reaches, and the seam stays partly
    open (`seam_open`, `removed_not_covered`). A uniform tessellation first helps.
  - The work is O(T log T) in the triangles of all scans (20 million at most natively, 2 million in the browser); the pair statistics
    use 20 000 points per scan.

## Parameters common to all align commands

- `object`: the mesh to move.
- `apply` (default true; false for `align.auto`, `align.symmetry`, `align.pair_coarse` and `align.global`, which propose first):
  false computes the motion without moving the mesh.
- `also`: other meshes moved by the same motion.

## Check

- The result reports the motion. `mesh.info` after an align command shows the new bounding box,
  to confirm the mesh is where you expect.
- `edit.undo` removes the last motion.
