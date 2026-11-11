# Draft 04: a box turned about a general axis does not read back from its own record (1007)

Status: draft, not filed. Filing needs the owner's consent. Open it in tangency-labs/solidcraft as
kernel-only work (template: Bug report, labels `bug`, `area:kernel`).

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.
**Duplicate found:** tangency-labs/solidcraft#13737 "An extruded rectangle's own record doesn't read back: a side wall's
plane axes are 1.2x past the reader's squareness bound" (open, by MarkusReichart). It reports the same
`Status(1007)` on `from_record` of a record that `to_record` wrote, with the cause read as the plane squareness bound
(`gamma(3)*|u|*|v|`). Our cases (drafts 04 to 07) are very likely the same root cause, but that is a hypothesis: we did not
measure `u.v` of the failing planes. Recommended: comment on #13737 with this case (one more way to hit it, and
a rate: 27 of 200 turns) instead of filing a new issue. The owner decides.
Other searches: "from_record 1007", "rotated box to_record": nothing else.

## What happened

`ops::make_box` turned by `Body::transform` with a pure rotation about a general axis gives a body whose
`to_record` succeeds, but `Body::from_record(&record, false)` fails with `Status(1007)` (`SM_ERR_INVALID_INPUT`);
`trusted = true` fails too. Of 200 pseudo-random rotations of the box 27 fail. The same box turned about z
in whole degrees (360 turns) or translated (100 positions) never fails.

## Steps to reproduce (public API only)

```rust
use solidcore_ops as ops;

fn turn(axis: [f64; 3], angle: f64) -> [[f64; 4]; 4] {
    let l = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let a = [axis[0] / l, axis[1] / l, axis[2] / l];
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        [t * a[0] * a[0] + c, t * a[0] * a[1] + s * a[2], t * a[0] * a[2] - s * a[1], 0.0],
        [t * a[0] * a[1] - s * a[2], t * a[1] * a[1] + c, t * a[1] * a[2] + s * a[0], 0.0],
        [t * a[0] * a[2] + s * a[1], t * a[1] * a[2] - s * a[0], t * a[2] * a[2] + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

let b = ops::make_box([0.0; 3], [50.0, 40.0, 10.0]).unwrap();
b.transform(&turn([-0.3902044387637371, 0.4445854841071728, 0.26358792709436607], 2.4184066553326042)).unwrap();
let (record, _) = b.to_record().unwrap();
ops::Body::from_record(&record, false).unwrap(); // Err(Status(1007))
```

**Expected:** the record reads back.
**Actual:** `Status(1007)`.

Version: solidcore `4f2da85`, Windows 11, Rust stable.

## Verification

- `turned_box_reads_back_from_its_record` fails on `4f2da85` with `turned box: from_record: Status(1007)` (run before the rule).
- The rates (27/200, 0/360, 0/100) come from a scratch exploration on `4f2da85`; the scratch test was deleted,
  the numbers are not part of the committed tests.
- Not done: reading `u.v` of the failing planes to confirm the #13737 cause.
