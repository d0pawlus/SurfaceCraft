# Extract primitives

Split a scan into regions that each follow one analytic surface (plane, sphere, cylinder, cone
or torus) and fit a primitive to each region. A region is a set of triangles of one mesh.

## Segment

- `region.auto_segment` with `mesh`: splits the mesh into regions, one per primitive. `options`
  takes `tolerance` (mm, default 0.2), `max_angle_deg` (default 20) and `min_triangles`
  (default 30). `replace` (default true) deletes the mesh's existing regions first.
- Organic surfaces: a bumpy surface is never one primitive for long, so fit-and-grow would end
  in thousands of slivers. With `options.freeform` (default true) regions that hold less than
  `freeform_min_share` of the scan's triangles (default 0.005, never fewer than 10 seed patches),
  or that are a curved primitive spanning less than a radius, are merged, together with the
  triangles between them, into one region per smooth area when at least five of them touch
  without a crease. Areas are split at creases sharper than `freeform_crease_deg` (default 40,
  between neighbouring face normals). Such a region has `freeform: true` and `kind: null`; fit
  a surface to it with `surface.fit`. Primitives with more triangles than the share and the
  ones that meet the area at creases (walls, fillets) stay as they were. `freeform: false`
  gives every fragment as a primitive region, as before.
- By hand: `region.grow` from seed `triangles` across edges sharper than `max_angle_deg`
  (default 15), or `region.create` from exact `triangles`.
- Correct the result with `region.split` (triangles into a new region, or into connected
  pieces) and `region.merge` (several regions into the first). `region.list` lists the regions,
  optionally of one `mesh`.

## Fit

- `primitive.fit` with `region`, or with `mesh` and `triangles`. `kind` is `auto` (default: the
  best residual), `plane`, `sphere`, `cylinder`, `cone` or `torus`. The result gives a new
  primitive id, `rms` and `max` in millimetres, and `count` of fitted points.
- `primitive.info` gives the parameters and fit quality of one primitive.

## Relate and trim

- `primitive.auto_constrain` finds parallel, perpendicular, coaxial and equal-radius relations
  between fitted primitives within its tolerances, and snaps them.
- `primitive.symmetry` on a `mesh` fits the plane that best mirrors the mesh onto itself: the
  mid-plane of a symmetric part.
- `primitive.trim_to_region` with `object` (the primitive) and `body`: the sheet of the body's
  face that lies on the primitive, bounded by its neighbours.

## Check

- Accept a fit when `rms` is small against your tolerance (for a clean scan, about 0.1 mm) and
  `max` stays near it. A large `max` usually means the region mixes two surfaces: split it with
  `region.split` and fit again.
- Each fit adds a new primitive object. Use `edit.undo` to drop a fit you do not want.
