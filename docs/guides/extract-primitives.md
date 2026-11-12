# Extract primitives

Split a scan into regions that each follow one analytic surface (plane, sphere, cylinder, cone
or torus) and fit a primitive to each region. A region is a set of triangles of one mesh.

## Segment

- `region.auto_segment` with `mesh`: splits the mesh into regions, one per primitive. `options`
  takes `tolerance` (mm, default 0.2), `max_angle_deg` (default 20) and `min_triangles`
  (default 30). `replace` (default true) deletes the mesh's existing regions first.
- Organic surfaces: a bumpy surface is never one primitive for long, so fit-and-grow would end
  in thousands of slivers. With `options.freeform` (default true) the small regions (less than
  `freeform_min_share` of the scan's triangles, default 0.005, never fewer than 10 seed patches)
  whose fit is not at the noise level of the scan, or that cover less than a radius of their
  curved primitive, are fragments. When at least five of them touch without a crease (face normals
  further apart than `freeform_crease_deg`, default 40), they are merged, with the triangles
  between them, into one region per smooth area. A region with a sound fit (large, or a plane or
  spanning a radius, RMS within 20 % of the scan's noise) stays a primitive whatever lies around
  it: a mounting face in a sculpted surface, a shallow cylindrical panel, small genuine features
  joined by tangent fillets. Such a merged region has `freeform: true` and `kind: null`; fit a
  surface to it with `surface.fit`. `freeform: false` gives every fragment as a primitive
  region, as before.
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
