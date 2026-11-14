# Build a solid from regions

Turn fitted primitives and freeform surfaces into one closed solid: a stock is trimmed by the
inputs. Each input needs its source region, because the region's points say which side is
material.

## Inputs

- Fit the primitives (`primitive.fit`) and the surfaces (`surface.fit`, or `surface.auto` for a
  layout). Keep their ids or names.
- `body.plan_regions` with `primitives` reports the role of each region before any cut: global
  cut, local tool part (blind hole, pocket, slot), fillet, or skipped, with the reason. Read it
  first. `square_planes_deg` (default 0.3, 0 disables) squares planes that lie close to a model
  axis.

## Build

- `body.from_regions` with `primitives` and `surfaces` (lists of ids or names), an optional
  `margin` (default 2 % of the points' diagonal plus 3 mm), `name` and `square_planes_deg`.
  The result is the solid, `faces_of` (for each input its faces in the solid, with `how` and
  `role`) and `skipped`, the number of inputs the kernel could not honour.
- `body.replace_face` swaps a planar face (its record index `face`) for a freeform `surface`
  that runs across the whole face.
- `body.extract_face` copies one face (`face`) as a sheet body.

## A solid with a constrained patch

- `surface.patch`, and `surface.from_curves` with `ngon: "patch"`, return sheets. A trimmed patch
  is not sewn into a body, and the kernel cannot sew separate sheets yet: it has no function that
  merges the records of several sheets, `nurbs::stitch` takes one record, and a record of unsewn
  sheets is refused when it is decoded. Measured on a 40 x 30 x 20 mm block with a planar cap and
  with a domed cap (2 mm), sewing failed at every tolerance from 1e-6 to 1e-2 mm, also for a cap
  whose gap was 1e-14 mm. A network with trimmed patches therefore stays a set of sheets, and
  `from_curves` with `ngon: "patch"` makes no solid. The default stays `split`, which makes a solid
  when the network is closed.
- A solid with one patch goes through the kernel's hybrid operations instead: the patch's surface
  replaces a planar face (`hybrid::replace_face`), or cuts the stock (`hybrid::cut_by_spline`).
  Measured with both, on the same block: a closed solid of six faces whose STEP round trip keeps
  the faces and the volume, and whose boolean cut by a 10 x 10 x 30 mm box succeeds. The volume
  error was exact for the flat cap, 5e-5 (relative) for a cap on curved top edges and 5e-4 for the
  domed cap.
- This route has no command yet: `hybrid::replace_face` and `hybrid::cut_by_spline` are kernel
  functions. `body.replace_face` is a different route: it fits a new surface to the points of the
  face's region (extended by `margin`), so it does not take the patch from `surface.patch`.

## Simple stock and tools

- `body.box` (`origin`, `size` as `[x, y, z]`), `body.cylinder` and `body.sphere` make
  primitive bodies for stock or for a boolean tool.
- `body.fillet` and `body.chamfer` round or bevel `edges` of a body with `size`.

## Parametric alternative

- When the design needs editable parameters, model it with features instead: see
  `sketches-features`.

## Check

- `skipped` should be 0. For each entry of `faces_of` with an empty `faces` list, read `how`.
- `body.to_mesh` tessellates the solid for a visual check.
- `deviation.compute` with the scan as `object` and the body as `against` measures the solid
  against the scan (see `check-deviation`).
