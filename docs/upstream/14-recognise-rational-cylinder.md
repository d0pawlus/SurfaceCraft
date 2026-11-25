# Draft 14: an exact rational cylinder face stays a spline after the rebuild at one placement

Update 2026-11-16 (P9-O8, pin `c02c5c51c`): the application's path moved from `step_brep::build_from_topology` to `solidcore_step::assembly`; on it every partial cylinder face of the merge comes back a spline after the rebuild (draft 18), so cylinders are now placed by the application with the kernel's full form, as cones and spheres were. `face_verdict` stays.

Status: not filed (2026-11-15). It does not reproduce on the current solidcore head (`f02d13e9c`). Kernel-only test
`crates/solidcore/tests/rational_cylinder_analytics.rs` on the branch `repro/surfacecraft-tess-recognise` of d0pawlus/solidcore (`971b413`):
`BSplineSurface::copy_and_add_analytics` (which calls `recognise_analytic`, `Cone::is_nurb_surface_cone`) on the exact two-patch rational
cylinder (r 2.4 mm, arc 0.2 rad, height 3.1) returns the cone/cylinder class at the origin, moved by 1e-6 in x, moved by (7.3, 1.1, 0), starting
at angle 0.3, with arcs 0.15, 0.25 and 0.3, r 5, height 8, and a quarter turn: ten of ten.

The failure seen in the application was on the path of the pin `4f2da85`: a one-face model through `step_brep::build_from_topology`, then
the rebuild. The crate `step-brep` is not in the head's workspace any more (the head has `solidcore-step`), so that path could not be
run kernel-only on the head, and `Body::from_record` does not add analytics (`add_analytics: false` in `solidcore-convert::brep::build`),
so a hand-made record rebuilt with it is a spline at every placement (a first, mistaken, repro: not a defect). Whether the application's
failing case still fails on the head depends on moving the application to a head that has the builder's successor; until then the
application keeps its own check (`face_verdict` refuses a new face that is not analytic, PR #142).

Not filed: the reproduction does not fail on the current head (tangency-labs AGENTS/CONTRIBUTING: issues come with a reproduction from real use).
