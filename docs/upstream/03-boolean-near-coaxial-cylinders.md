# Draft 03: `boolean` (intersect) of two cylinders whose axes are 0.01 degrees apart fails with 1001

Status: draft, not filed. Filing needs the owner's consent. Open it in tangency-labs/solidcraft as
kernel-only work (template: Bug report, labels `bug`, `area:kernel`).

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.
**Duplicate search:** "intersect cylinders axes 1001" gives #1001 (FILLET: operation failed, unrelated number
match, an app report), "coaxial cylinders boolean" none. Related by theme, not a duplicate: #13650 (a boolean operand
that touches itself along a line fails with relate 1001).

## What happened

Intersecting two cylinders (radius 5, height 20, both based at the origin) where the second is turned 0.01 degrees
about x fails with `Status(1001)` (`SM_ERR`). The same pair turned 0.5 degrees intersects (volume 1551.9 mm3),
3 degrees gives 1459.4, 0 degrees gives 1570.8. Only a near-coincident axis fails.

## Steps to reproduce (public API only)

```rust
use solidcore_ops as ops;

let make = || {
    let b = ops::make_cylinder([0.0; 3], 5.0, 20.0).unwrap();
    let (rec, _) = b.to_record().unwrap();
    ops::Body::from_record(&rec, false).unwrap().0
};
let a = make();
let b = make();
let (s, c) = 0.01_f64.to_radians().sin_cos();
// rows: images of x, y, z; last row the translation
b.transform(&[[1.0, 0.0, 0.0, 0.0], [0.0, c, s, 0.0], [0.0, -s, c, 0.0], [0.0, 0.0, 0.0, 1.0]]).unwrap();
let lens = ops::boolean::boolean(a, b, ops::BooleanOp::Intersect).unwrap(); // Err(Status(1001))
```

Note: the SurfaceCraft test applies its matrix through a transposing wrapper (`solid::transform`); the matrix above
is the same turn written in the `Body::transform` convention. This exact snippet was not run; the verified
test is `near_coaxial_cylinders_intersect`.

**Expected:** a lens solid of volume between 1400 and pi*25*20, about 1570.8 mm3.
**Actual:** `Err(Status(1001))`.

Version: solidcore `4f2da85`, Windows 11.

## Verification

- `near_coaxial_cylinders_intersect` fails on `4f2da85` with `intersect must succeed: Status(1001)` (run before the rule).

## SurfaceCraft workaround

`engine::cmd::hybrid::snap_axes` snaps cylinder, cone and torus axes within 3 degrees of the first one.
