# Draft 19: the STEP writer writes cone labels STEP cannot hold, and its reader refuses them

Status: not filed (2026-11-24). Read from the code of the pin `c02c5c51c` and seen in SurfaceCraft's tests; no kernel-only
reproduction yet. Nothing was posted.

## What happens

`solidcore_step::write::topology::surface` writes a record `Surface::Cone { frame, radius_at_origin, half_angle }` as
`CONICAL_SURFACE` with `radius = radius_at_origin` and `semi_angle = |half_angle|`, as they are. Two labels the writer meets give a
file its own reader refuses ("cone with a negative radius or a semi angle outside (0, 90°)", `topology.rs`):

- **half angle 0**: `ops::blend` gives the round of a box edge as a cone of half angle 0 (SurfaceCraft's draft 02,
  `docs/upstream/02-blend-cylinder-as-cone.md`, still so on `c02c5c51c`). A filleted box exported and read back fails.
- **negative radius at the origin**: the writer's own `analytic_rounds` labels a rational face with the cone
  `face_surface::analytic_label` proves; that label's frame can sit past the apex (radius below 0 at its origin). Parts with a
  drill-point or a countersink cone exported and read back fail.

## Expected

The writer writes the same surface in a form STEP holds: a cone of half angle 0 as the `CYLINDRICAL_SURFACE` it is, a cone whose
radius at the frame's origin is negative from its apex (frame moved along the axis to where the radius is 0). A face with a NURBS form
has its parameters on the form, so the label's frame is free to move.

## Workaround in SurfaceCraft

`crates/kernel/src/step.rs` (`step_label`, `cones_from_their_apex`) relabels both before writing, including the rational faces the
writer would label (it runs `analytic_label` itself). To be removed when the writer does it.
