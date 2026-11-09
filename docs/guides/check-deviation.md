# Check deviation

Measure how far a scan is from a reference (a primitive, a mesh, a body or a surface). The
result is statistics, plus a colour map on the scan.

## Measure

- `deviation.compute` with `object` (the scan mesh) and `against` (primitive, mesh, body or
  surface).
- `region` limits the measured vertices to one region of the scan.
- `tolerance` (mm, default 0.1) sets the colour map range, from minus to plus the tolerance.
- `max_distance` (mm, default 5) leaves out points farther from a mesh or body reference.
- `show` (default true) paints the colour map on the scan.
- The result has the statistics in `stats`. `warning` appears when the tolerance is below the
  estimated scan noise.

## Noise

- `deviation.noise` with `object` estimates the scan noise sigma. Choose `tolerance` above it:
  a tolerance below the noise colours the noise, not the part.

## Clear

- `deviation.clear` with `object` removes the colour map from the scan.

## Check

- Checking a body: `object` is the scan and `against` the body. The body is tessellated for the
  comparison.
- Checking a fitted surface or primitive: measure its own region (`region`). The maximum should
  be close to the `max` that the fit reported.
- Each new `deviation.compute` replaces the colour map of the scan; `deviation.clear` removes it.
