# Draft 11: decoding a record of separate (unsewn) sheets fails, so separate sheets cannot be sewn

Update 2026-11-16 (P9-O8, pin `c02c5c51c`): the application sews separate sheets with `solidcore_ops::stitch::sew` on sheets decoded one by one (`surface.sew`); six one-face sheets of a block and of a domed block close into solids of the exact volume. Nothing to file.

Status: not filed (2026-11-15). It does not reproduce as a defect on the current head (`74300b3`). Kernel-only test
`crates/solidcore-ops/tests/record_sheets_repro.rs` (`unsewn_*`) on the branch `repro/surfacecraft-kernel-limits` of d0pawlus/solidcore: a record of
six separate sheets (a 40 x 30 x 20 mm block, bilinear faces; and bicubic Bezier faces with a flat and a 6 mm domed top), own edges and
vertices per sheet, one open shell each, validates; `Body::from_record(&record, false)` is refused with the NotYet error described below, which
the API documents (`solidcore-convert::to_kernel`: an untrusted record whose topology the healer changed has no record order); but
`Body::from_record(&record, true)` followed by `ops::stitch` at 1e-6 and at 1e-2 gives a closed solid of the exact volume (24 000 and 25 800 mm3).
So the route exists and the draft's claim "no route" and "sewing fails at every tolerance" is not true for kernel-only sheets; whatever failed in the
application is not reduced to a kernel case.

**solidcore commit observed:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`. Not yet checked on the
current head.
**Duplicate search:** not done yet. Search tangency-labs/solidcraft for "unsewn", "stitch",
"separate sheets" and "healer record order" before filing.
**Reproduction:** not written yet. The kernel-only test has to be written and verified first (see
"Steps to reproduce" and "Verification").

## What happened

Several sheet records that share their boundary curves, but are not one shell, cannot be combined
into one shell:

- `nurbs::stitch` on the sheet records fails at every tolerance tried, from 1e-6 to 1e-2 mm. This
  includes a flat cap whose shared edges differ by 1e-14 mm.
- Decoding the record of the unsewn sheets with `Body::from_record` is refused with
  `the healer changed the record's topology, so its record order is not known`. The record cannot be
  read back, so sewing cannot start from it.

The public API gives no route round this. `extract_face` takes one face out of a body.
`sheet_from_patches` and `sheet_from_bezier_network` build a sheet from patch grids and Bézier
networks only. `stitch` takes one record. No function merges the records of several sheets into one.

## Steps to reproduce (to be written and verified)

Not written yet, so this is a plan and not a result:

1. Build a 40 × 30 × 20 mm block as separate sheet records: four side sheets and a planar cap, each
   its own record, with shared boundary curves that agree to 1e-14 mm.
2. Call `nurbs::stitch` on the sheets at 1e-6 mm and at 1e-2 mm.
3. Call `Body::from_record(&record, false)` and `Body::from_record(&record, true)` on the unsewn
   record.

The reproduction must use the public solidcore API only, and it must fail on the current head before
the issue is filed.

## Expected

A record of sheets whose shared edges agree to 1e-14 mm either decodes, or sews into one shell at a
tolerance at least as large as the gap.

## Actual

As described above: the stitch fails at every tolerance, and the decode is refused.

## Context from the application

The application measured this with a block of 40 × 30 × 20 mm: a four-patch Bézier frame, and a cap
that was flat, domed by 2 mm, or on curved top edges. Sewing failed in all three cases. A closed solid
with six faces was obtained by a different route: `hybrid::replace_face` (flat and domed caps) and
`hybrid::cut_by_spline` (all three cap variants), with the cap's surface and no sewing. Its STEP round
trip kept the faces and the volume.

## Verification

- Not done: the kernel-only reproduction above, on the current head.
- Not done: the duplicate search.
- Observed by the application's measurement test on the pin above, not yet reduced to a solidcore test.
