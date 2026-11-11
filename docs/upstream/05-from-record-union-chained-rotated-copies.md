# Draft 05: `union_all` of three chained rotated copies does not read back from its record (1007)

Status: draft, not filed. Filing needs the owner's consent. Open it in tangency-labs/solidcraft as
kernel-only work (template: Bug report, labels `bug`, `area:kernel`).

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.
**Duplicate found (probable, same symptom):** tangency-labs/solidcraft#13737 (record that `to_record` wrote is refused
by `from_record`, status 1007, plane axes past the squareness bound). Whether this case has the same cause is a
hypothesis; not measured. Related: draft 04. Recommended: comment on #13737. Searches "union_all rotated",
"pattern union rotated": no other hit.

## What happened

The application recipe for a circular pattern (copy; transform the copy by one step; copy again; `union_all`) with
three bars of 20 x 2 x 2 turned 60 degrees apart about z gives a union with the right volume (219.215 mm3) and a
`to_record` that succeeds, but `from_record` of that record fails with `Status(1007)`. Four bars at 60 degrees and four bars at
30 degrees fail the same way when chained. Turning each copy independently from the original, or uniting the
copies one after the other with `boolean`, reads back.

## Steps to reproduce (public API only)

```rust
use solidcore_ops as ops;

let (s, c) = 60.0_f64.to_radians().sin_cos();
let step = [[c, s, 0.0, 0.0], [-s, c, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]];
let base = ops::make_box([-10.0, -1.0, -1.0], [20.0, 2.0, 2.0]).unwrap();
let mut bodies = vec![base.copy().unwrap()];
let mut prev = base;
for _ in 1..3 {
    let next = prev.copy().unwrap();
    next.transform(&step).unwrap();
    bodies.push(next.copy().unwrap());
    prev = next;
}
let united = ops::boolean::union_all(bodies).unwrap();
assert!((united.volume(1e-6).unwrap() - 219.215).abs() < 0.01); // passes
let (record, _) = united.to_record().unwrap();                 // passes
ops::Body::from_record(&record, false).unwrap();               // Err(Status(1007))
```

**Expected:** the record reads back.
**Actual:** `Status(1007)`.

Version: solidcore `4f2da85`, Windows 11, Rust stable.

## Verification

- `union_of_three_chained_rotated_copies_reads_back` fails on `4f2da85` with `from_record: Status(1007)` (run before the rule).
- The variants (independent turns and sequential `boolean` read back; 4 bars at 60 and 30 degrees fail when chained)
  come from a scratch exploration on `4f2da85`, not from committed tests.

## SurfaceCraft effect

`feature.pattern` with rotation, `feature.path_pattern` follow: the body is made but has no measurable volume
(`crates/engine/tests/p5_features.rs`, comment on the 24-face assertion).
