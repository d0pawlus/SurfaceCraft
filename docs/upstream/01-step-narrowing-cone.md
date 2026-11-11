# Draft 01: STEP round trip of a cone that narrows along +z returns one body per face

Status: draft, not filed. Filing needs the owner's consent. Per the upstream rule, solidcore is
developed inside tangency-labs/solidcraft at `solidcore/`; open it there as kernel-only work
(template: Bug report, labels `bug`, `area:io`). Re-verified on `b9c02da` earlier; the run on
`4f2da85` was done before the no-local-builds rule: see "Verification".

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d` (HEAD of tangency-labs/solidcore on 2026-11-09).
**Duplicate search** (`gh search issues --repo tangency-labs/solidcraft`): "cone STEP" gives only #6310
(Insert STEP: some faces did not import, a user file, no cone analysis). Not a duplicate; mention it as related.

## What happened

A frustum with radius 5 at the base and 1 at the top (height 10) is written with
`step_export::write` and read back with `step_brep::import`. A single solid must come back as one body;
it comes back as 2 bodies. Our notes say a cone that widens along +z and a sharp cone (5 to 0) survive;
those two cases were not re-run for this draft.

## Steps to reproduce (public API of solidcore crates only)

```rust
use solidcore::kernel::{Kernel, KernelOptions};
use solidcore_ops as ops;

let cone = ops::make_cone([0.0; 3], 5.0, 1.0, 10.0).unwrap();
let (rec, _) = cone.to_record().unwrap();

let mut k = Kernel::new(&KernelOptions::default());
let kb = solidcore_convert::brep::to_kernel_body(&mut k, &rec, true).unwrap();
let model = step_brep::brep_to_model::brep_to_model(&kb).unwrap();
let text = step_export::write(&model).unwrap();

let breps = step_brep::import(text.as_bytes(), &Default::default()).unwrap();
assert_eq!(breps.len(), 1, "got {} bodies", breps.len()); // fails: 2
```

**Expected:** one body, volume pi*10/3*(5^2 + 5*1 + 1^2), about 324.4 mm3.
**Actual:** `breps.len() == 2`. The conical face of negative half angle does not survive `step_brep::import`.

SolidCraft/SurfaceCraft version: solidcore `4f2da85`, Windows 11, Rust stable.

## Verification

- Run before the rule: `cargo test -p surfacecraft-kernel --profile ci --test upstream_repro -- --ignored`
  on `4f2da85`: `cone_narrowing_along_z_survives_step_round_trip` fails with "got 2 bodies".

## SurfaceCraft workaround (not part of the issue)

`kernel::hybrid::shape_solid` builds a cone that widens along +z from a 0.01 mm disc at the apex.
