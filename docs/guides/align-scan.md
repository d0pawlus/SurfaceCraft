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

## Coarse alignment

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
- `apply` (default true): false computes the motion without moving the mesh.
- `also`: other meshes moved by the same motion.

## Check

- The result reports the motion. `mesh.info` after an align command shows the new bounding box,
  to confirm the mesh is where you expect.
- `edit.undo` removes the last motion.
