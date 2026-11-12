# Align a scan

Bring a scan mesh into the model frame before any fitting. Each align command moves the mesh
(and the meshes in `also`) by one rigid motion. A motion is one undo step.

Over MCP every command is also a tool with the dot replaced by an underscore (`align.icp` is the
tool `align_icp`), or call `execute` with `command` and `params`.

## Import

- `mesh.import`: `path` (STL, OBJ, PLY or XYZ) or base64 `data` with `format`. Over MCP file
  paths are off unless the server runs with `--allow-files`, so send `data` inline.
- `scale` converts units (25.4 for inches to millimetres). `name` names the mesh.
- `mesh.info` reports triangles, bounding box and area. Check the units with it first.

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

## Parameters common to all align commands

- `object`: the mesh to move.
- `apply` (default true; false for `align.auto` and `align.symmetry`, which propose first):
  false computes the motion without moving the mesh.
- `also`: other meshes moved by the same motion.

## Check

- The result reports the motion. `mesh.info` after an align command shows the new bounding box,
  to confirm the mesh is where you expect.
- `edit.undo` removes the last motion.
