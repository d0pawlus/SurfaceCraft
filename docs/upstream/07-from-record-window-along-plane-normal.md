# Draft 07: a plate turned about a general axis minus a prism along its plane normal does not read back (1007)

Status: draft, not filed. Filing needs the owner's consent. Open it in tangency-labs/solidcraft as
kernel-only work (template: Bug report, labels `bug`, `area:kernel`).

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.
**Duplicate found (probable, same symptom):** tangency-labs/solidcraft#13737 (record refused by `from_record`, status 1007).
Note the observation of this draft differs from SurfaceCraft's first note ("a tool whose direction equals a plane
normal exactly breaks the boolean"): the boolean here succeeds and the failure is the read back; and a tool
direction a hair off the normal is not a fix (below). That suggests the cause is the record of turned bodies in
general (drafts 04 to 06), not the exact equality. Hypothesis, not measured. Related: drafts 04 to 06. Recommended:
comment on #13737.

## What happened

The box 50 x 40 x 10 is turned by 1.38734577002826 rad about [0.26162006230833446, -0.20491162934367935,
-0.3720612130285178] and round-tripped through its record (it reads back). A 12 x 8 window (a prism made by
`ops::sweep::extrude`) with the direction `u_axis x v_axis` (unit) of the plate's top plane, read from that record,
centred on the plate and reaching 3 mm out of both faces, is subtracted. `boolean` and `to_record` succeed;
`from_record` of the result fails with `Status(1007)`. Axis-aligned plates and prisms work.

Sensitivity (scratch exploration on `4f2da85`, 60 pseudo-random turns x through/blind window, 120 cases, 22 of them
failing already on the plate itself, so 98 usable): direction exactly the normal: 6 fail; tilted by 1e-12: 65 fail;
by 1e-9: 7; by 1e-6: 3; by 1e-3: 6.

## Steps to reproduce (public API only)

```rust
use solidcore_ops as ops;
use solidcore_record::brep::Surface;

type V = [f64; 3];
fn cross(a: V, b: V) -> V { [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]] }
fn dot(a: V, b: V) -> f64 { a[0]*b[0] + a[1]*b[1] + a[2]*b[2] }
fn unit(a: V) -> V { let l = dot(a, a).sqrt(); [a[0]/l, a[1]/l, a[2]/l] }
fn add(a: V, b: V) -> V { [a[0]+b[0], a[1]+b[1], a[2]+b[2]] }
fn scale(a: V, s: f64) -> V { [a[0]*s, a[1]*s, a[2]*s] }
// `turn` is the rotation matrix of draft 04 (Body::transform convention: rows are the images of x, y, z).

let m = turn([0.26162006230833446, -0.20491162934367935, -0.3720612130285178], 1.38734577002826);
let plate = ops::make_box([0.0; 3], [50.0, 40.0, 10.0]).unwrap();
plate.transform(&m).unwrap();
let (rec, _) = plate.to_record().unwrap();
let turned_z = [m[2][0], m[2][1], m[2][2]];
let (origin, n) = rec.faces.iter()
    .filter_map(|f| match &f.surface {
        Surface::Plane { origin, u_axis, v_axis } => Some((*origin, unit(cross(*u_axis, *v_axis)))),
        _ => None,
    })
    .find(|(_, n)| dot(*n, turned_z) > 0.999).unwrap();
let (copy, _) = ops::Body::from_record(&rec, false).unwrap();            // reads back
let centre = add(add(scale([m[0][0], m[0][1], m[0][2]], 25.0), scale([m[1][0], m[1][1], m[1][2]], 20.0)),
                 scale(turned_z, 5.0));
let to_top = dot([origin[0]-centre[0], origin[1]-centre[1], origin[2]-centre[2]], n);
let e1 = unit(cross(n, [1.0, 0.0, 0.0]));
let e2 = cross(n, e1);
let base = add(centre, scale(n, -to_top - 3.0));
let pts: Vec<V> = [(-6.0, -4.0), (6.0, -4.0), (6.0, 4.0), (-6.0, 4.0)].iter()
    .map(|(a, b)| add(base, add(scale(e1, *a), scale(e2, *b)))).collect();
let window: Vec<ops::sweep::Curve> =
    (0..4).map(|i| ops::sweep::Curve::Line { start: pts[i], end: pts[(i + 1) % 4] }).collect();
let tool = ops::sweep::extrude(&window, n, 2.0 * to_top + 6.0, 0.0, Default::default(), true).unwrap();
let cut = ops::boolean::boolean(copy, tool, ops::BooleanOp::Subtract).unwrap().unwrap(); // succeeds
let (rec, _) = cut.to_record().unwrap();                                                 // succeeds
ops::Body::from_record(&rec, false).unwrap();                                            // Err(Status(1007))
```

**Expected:** the plate with the window reads back.
**Actual:** `Status(1007)`.

Version: solidcore `4f2da85`, Windows 11, Rust stable.

## Verification

- `window_along_a_plane_normal_reads_back` fails on `4f2da85` with `plate with window: from_record: Status(1007)`
  (run before the rule). The sensitivity table is from a deleted scratch test, not committed.

## SurfaceCraft workaround

`engine::cmd::plan` snaps tool directions within 0.5 degrees to the model axes (a model axis is not a fitted normal).
The kernel's "P3b" debt entry is reworded by this finding; STATUS.md is not edited by this change.
