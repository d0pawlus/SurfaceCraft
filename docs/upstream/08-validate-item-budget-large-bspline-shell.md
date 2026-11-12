# Draft 08: `Brep::validate` refuses a closed B-spline shell of about 40 000 bicubic patches with `item budget`

Status: draft, not filed. Filing needs the owner's consent. It is probably a question or documentation request rather
than a bug (see "Assessment"). If filed, open it in tangency-labs/solidcore (SurfaceCraft's drafts 04 to 07 name
tangency-labs/solidcraft; the kernel-paused notice in tangency-labs/solidcraft#3992 says new kernel issues get the
label `kernel-paused`).

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d` (the same text is at `b9c02da`).
**Duplicate search** (tangency-labs/solidcraft, read-only `gh search issues`): "item budget", "MAX_ITEMS",
"validate_built": nothing about a large B-spline shell. #3992 is the umbrella "kernel work is paused" notice and
`solidcore-record`'s own test `the_item_budget_bounds_input_records_and_not_the_kernels_own` cites it. Not searched:
tangency-labs/solidcore issues.

## What happened

SurfaceCraft's `surface.wrap` builds a watertight network of bicubic Bezier patches over a scan, up to 60 000 patches.
The record of such a network (one `Surface::Spline` face per patch, edges shared, one closed shell) was refused by
`Brep::validate` with `item budget` when the network passed about 40 000 patches. Items are counted per knot, control
point, curve, loop use and pcurve point; one bicubic patch of a closed network costs about 100 of them, and
`solidcore_record::brep::MAX_ITEMS` is 4 000 000.

Measured on a synthetic cube sphere (`6 n^2` bicubic patches, control points on a sphere, edges shared, closed):
`n = 81` (39 366 patches) validates, `n = 82` (40 344 patches) returns `item budget`. `Brep::validate_built()`
(every check but the budget) accepts all sizes tried (up to 80 736 patches), and `Body::from_record(&record, true)`
builds the body and `to_record` writes it again (volume read back within 2.1e-4 relative of the exact sphere at 12 150
patches).

## Steps to reproduce (public API only)

The record is assembled in `crates/kernel/tests/common/mod.rs` (`record_of`) with `solidcore-record` types only.

```rust
use solidcore_ops as ops;

let record = common::record_of(&common::cube_sphere(87, 50.0)); // 45 414 bicubic patches
assert!(record.validate_built().is_ok());
let (body, _) = ops::Body::from_record(&record, true).unwrap();
let (written, _) = body.to_record().unwrap();
written.validate().unwrap(); // Err: "portable B-rep: item budget"
```

Repro test: `record_of_a_large_closed_bspline_shell_validates` in `crates/kernel/tests/upstream_repro.rs`
(`#[ignore]`, fails on `4f2da85` with `45414 faces: Err("portable B-rep: item budget")`).

**Expected:** the record `Body::to_record` writes passes `Brep::validate`, or the documentation of `validate` and
`MAX_ITEMS` says that a closed bicubic shell is limited to about 40 000 faces and points at `validate_built` for
records a program builds itself.
**Actual:** `item budget` from `validate()`, with no hint in the message which count was exceeded or what the limit is.

## Assessment

The budget is deliberate: `validate` bounds hostile input and `validate_built` exists for records a program makes in
memory (#3992). From that reading the behaviour is by design and the report is a request for: (1) a message that names
the limit (`item budget: 4 000 123 of 4 000 000`), (2) a note in the docs of `Brep::validate` that a closed bicubic
network has a ceiling near 40 000 faces. A larger or configurable `MAX_ITEMS` is the alternative the owner may prefer to
ask for; scale on our side: STEP of such a body is about 24 KB per patch, so the input budget is not the only bound a
user meets.

## Our workaround

`crates/kernel/src/nurbs/network_check.rs`: a network we assemble ourselves is checked with `validate_built` and
bounded by our own `MAX_NETWORK_PATCHES` (120 000). `from_record` already uses `validate_built` internally, so nothing
else changes. Tests: `crates/kernel/tests/large_network.rs` (12 000, 16 000 and 60 000 patches, closed, volume within
1e-3 of the sphere).

## Verification

- Before the change `sheet_from_bezier_network_as_is` failed with `kernel operation `record validate` failed: portable
  B-rep: item budget` for 45 414, 60 000 and 80 736 patches and built 30 246 patches (run on the unmodified code).
- Not done: a real 1 M triangle scan (the `item budget` of the scan is inferred to be this error because the engine
  allows 60 000 patches and the budget is reached at about 40 000; the scan itself was not run).
- Not searched: tangency-labs/solidcore issues.
