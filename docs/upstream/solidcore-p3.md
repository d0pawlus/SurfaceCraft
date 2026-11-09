# solidcore problems found in P3 (draft issues)

Status: **drafts only.** Nothing has been filed at tangency-labs/solidcore. Filing needs the owner's
consent (AGENTS.md / plan P3b Z5). Per the solidcore contribution rule, each item goes up as an issue
with the reproduction first, before any PR.

Pin: `solidcore` rev `b9c02da746f48d24f1c2a32deb834276b1014d03` (workspace `Cargo.toml`).

Reproductions live in `crates/kernel/tests/upstream_repro.rs`. Each test asserts the expected result and
is `#[ignore]`d, so it fails on the pin. Run them with:

```
cargo test -p surfacecraft-kernel --profile ci --test upstream_repro -- --ignored
```

Once upstream fixes an item: bump the pin, remove that `#[ignore]`, and delete the workaround.

---

## 1. Cone narrowing along +z splits into several bodies after a STEP round trip

**Title:** STEP export/import of a cone with negative half angle (narrowing along z) returns one body per face

**Pin:** b9c02da

**Minimal reproduction** (`crates/kernel/tests/upstream_repro.rs`, `cone_narrowing_along_z_survives_step_round_trip`):

```rust
let cone = ops::make_cone([0.0; 3], 5.0, 1.0, 10.0).expect("make_cone");
let (rec, _) = cone.to_record().expect("to_record");
let text = step::export_solids(&[("cone", &rec)]).expect("export_solids");
let bodies = step::import(text.as_bytes()).expect("import");
assert_eq!(bodies.len(), 1, "a single solid must come back as one body, got {} bodies", bodies.len());
```

Here `step::export_solids` is `step_brep::brep_to_model` + `step_export::write`, and `step::import` is
`step_brep::import` + `solidcore_convert::brep::to_record`.

**Expected:** one body comes back, the same solid (volume π·10/3·(5² + 5·1 + 1²) ≈ 324.4 mm³ for this frustum).

**Actual (observed on the pin):** `got 2 bodies`, not 1 (`cargo test ... -- --ignored`, test panics at
`upstream_repro.rs:25`). A base radius of 5 and a top radius of 1 gives a cone that narrows along +z, which is a
negative half angle in the record. Our notes say a cone widening along +z and a sharp cone 5 → 0 round-trip;
those two cases were not re-run for this draft.

**Our workaround:** `kernel::hybrid::shape_solid` builds a cone that widens along +z from a 0.01 mm disc at
the apex (so the half angle is positive). The disc sits at the apex; the cone is then placed with
`frame_matrix`. Location: `crates/kernel/src/hybrid.rs`.

---

## 2. `ops::blend` returns a fillet cylinder as a `Cone` with half angle 0

**Title:** A blend of a straight edge is recorded as a Cone of half angle 0 instead of a Cylinder

**Pin:** b9c02da

**Minimal reproduction** (`fillet_of_box_edge_is_a_cylinder_face`):

```rust
let b = solid::make_box([0.0; 3], [20.0, 10.0, 5.0]).expect("make_box");
let f = solid::fillet(&b, &[0], 2.0).expect("fillet");   // solid::fillet -> ops::blend(Round)
let cylinders = f.faces.iter().filter(|face| matches!(face.surface, Surface::Cylinder { .. })).count();
assert_eq!(cylinders, 1);
```

**Expected:** the round is a `Surface::Cylinder` (radius 2, axis along the edge), as the blend of a
straight edge between two planes is a cylinder.

**Actual (observed on the pin):** the filleted box has six `Surface::Plane` faces and one `Surface::Cone`, and
no `Surface::Cylinder` (`face kinds were ["plane" x6, "cone"]`, test panics at `upstream_repro.rs:50`).
Our kernel code says this cone has half angle 0 (`kernel::hybrid::faces_on_shape`); the half angle and radius
were not re-read for this draft. The geometry is right; only the surface kind is not. Anything that matches
`Cylinder` misses it.

**Our workaround:** `kernel::hybrid::faces_on_shape` accepts a `Cone` with `half_angle.abs() < 1e-9` as a
cylinder when it compares with a `Shape::Cylinder` (comment: "a blend's cylinder comes back as a cone of
half angle zero (SMLib's own form)"). Location: `crates/kernel/src/hybrid.rs`.

---

## 3. Boolean of near-coaxial revolved surfaces fails with status 1001

**Title:** `boolean` (intersect) of two cylinders whose axes are 0.01 degrees apart fails with SM_ERR (1001)

**Pin:** b9c02da

**Minimal reproduction** (`near_coaxial_cylinders_intersect`):

```rust
let a = solid::make_cylinder([0.0; 3], 5.0, 20.0).expect("make_cylinder");
let c = 0.01_f64.to_radians().cos();
let s = 0.01_f64.to_radians().sin();
let turn = [[1.0, 0.0, 0.0, 0.0], [0.0, c, -s, 0.0], [0.0, s, c, 0.0], [0.0, 0.0, 0.0, 1.0]];
let tilted = solid::transform(&solid::make_cylinder([0.0; 3], 5.0, 20.0).expect("make_cylinder"), &turn)
    .expect("transform");
let (ba, _) = ops::Body::from_record(&a, false).expect("from_record a");
let (bb, _) = ops::Body::from_record(&tilted, false).expect("from_record b");
let out = ops::boolean(ba, bb, ops::BooleanOp::Intersect).expect("intersect must succeed");
let v = out.expect("the intersection is not empty").volume(1e-6).expect("volume");
assert!((1400.0..=std::f64::consts::PI * 25.0 * 20.0).contains(&v));
```

**Expected:** a solid lens of volume between 1400 mm³ and π·25·20 ≈ 1570.8 mm³.

**Actual (observed on the pin):** `ops::boolean` returns `Error::Status(1001)` (`SM_ERR`); the test panics with
`intersect must succeed: Status(1001)` at `upstream_repro.rs:72`. Probe results on the same pin, same radius
and height, cylinder 2 turned about x: 0.01° → 1001; 0.5° → intersects, volume 1551.9 mm³; 3° → 1459.4 mm³;
0° (coaxial) → 1570.8 mm³. So the failure needs a near-coincident axis, not any tilt.

**Our workaround:** `crates/engine/src/cmd/hybrid.rs`, `snap_axes`: before cutting, every cylinder,
cone and torus axis within 3° of the first one is snapped to it (and its point to the axis when the
offset is under 0.5 mm). It is kept as an auto-constraint (design doc §6.2) rather than a precondition;
the repro removes the need for the snap only once upstream intersects these surfaces.
