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
