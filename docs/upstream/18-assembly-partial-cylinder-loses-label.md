# Draft 18: a partial cylinder face from `solidcore_step::assembly` comes back as a spline after a rebuild

Status: not filed (2026-11-16), and not yet reduced to a kernel-only reproduction. Seen on the pin `c02c5c51c` (solidcore head at the
start of P9-O8). Nothing was posted.

## What happened

The hybrid merge of SurfaceCraft (`crates/kernel/src/merge.rs`) replaces a group of freeform patches that lie on one cylinder by one
cylinder face. On the old pin `4f2da85` the face came from `step_brep::build_from_topology` (a one-face sheet of the group's boundary on
the cylinder), and the body rebuilt with `Body::from_record(.., true)` + `to_record` kept the face as `Surface::Cylinder`. `step-brep` is
gone at head; its successor for this job is `solidcore_step::assembly::Assembly` (`add_vertex`, `add_edge`, `add_face` with the
`Surface::Cylinder`, `finish(.., Kind::Model)`). The face it returns carries the cylinder label and a NURBS form of its own (step 4 of
`finish`: "turns round faces' frames so that each face's NURBS form covers it once"). Put into the body and rebuilt the same way, the face
comes back as a `RationalSpline`: in SurfaceCraft's `merge_faces` tests 6 of 6 cylinder cases (open sheet, closed bore used backward,
three sectors of a wall that goes round the axis, a group whose border is snapped onto the cylinder) give the face back as a spline, where
all planes keep their label.

## What is known

- The kernel keeps the label of a cone or sphere only on its own full form (the whole turn about the axis, the face a sub-box of it);
  SurfaceCraft already placed cones and spheres with that form and its own pcurves for this reason (owner exception of 2026-11-14,
  `docs/upstream/12-merge-onto-primitive.md`). Placing the cylinder the same way (form = the kernel's full turn,
  `solidcore::...::form::surface(label, [[0, 2π], v])`) keeps the label: the 6 tests pass. So the partial form the assembly gives is what
  loses the label.
- Related: draft 14 (an exact rational cylinder left a spline after the rebuild at one placement on the old pin; not reproduced
  kernel-only on head).

## What to ask (once reduced)

Either the assembly gives a round face the kernel's full form (so `Body::from_record` + `to_record` keep its label), or
`from_record` recognises a partial form of a cylinder as the cylinder. Reproduction to write first, kernel-only: one `Assembly` face on a
`Surface::Cylinder` bounded by two rulings and two arcs of 60 degrees, `finish`, then `Body::from_record(&record, true)` and
`to_record`; expected the face's surface `Cylinder`, seen (in the application) `RationalSpline`.
