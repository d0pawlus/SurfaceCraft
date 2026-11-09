# Draft 02: `ops::blend` returns the round of a straight edge as a Cone of half angle 0

Status: draft, not filed. Filing needs the owner's consent. Open it in tangency-labs/solidcraft as
kernel-only work (template: Bug report, labels `bug`, `area:kernel`).

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.
**Duplicate search:** "fillet cone half angle", "blend cylinder cone half angle": no hit. Possibly this is
intended (SMLib's own form), in which case the issue is a request to document it or to normalise in the record
conversion; say so in the text.

## What happened

Rounding one straight edge of a box gives a record whose round is `Surface::Cone` with half angle 0, not
`Surface::Cylinder`. Anything that matches `Cylinder` misses it. The geometry is right; only the surface kind differs.

## Steps to reproduce (public API only)

```rust
use solidcore_ops as ops;
use solidcore_record::brep::Surface;

let b = ops::make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
let (rec, _) = b.to_record().unwrap();
let (b, _) = ops::Body::from_record(&rec, false).unwrap();
ops::blend(&b, ops::Blend::Round, &[0], &[2.0], &[], ops::Corners::Filled).unwrap();
let (rec, _) = b.to_record().unwrap();
let cylinders = rec.faces.iter().filter(|f| matches!(f.surface, Surface::Cylinder { .. })).count();
assert_eq!(cylinders, 1); // fails: 0; the faces are 6 planes and 1 Surface::Cone
```

**Expected:** one `Surface::Cylinder` (radius 2, axis along the edge).
**Actual:** six `Surface::Plane` and one `Surface::Cone` (`half_angle: 0.0`, radius at origin 2).

Version: solidcore `4f2da85`, Windows 11.

## Verification

- `fillet_of_box_edge_is_a_cylinder_face` fails on `4f2da85` with face kinds
  `["plane" x6, "cone"]` (run before the no-local-builds rule).

## SurfaceCraft workaround

`kernel::hybrid::faces_on_shape` accepts a Cone with `half_angle.abs() < 1e-9` as a cylinder.
