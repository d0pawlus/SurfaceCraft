# Draft 09: the healer of `Body::from_record(&record, false)` grows faster than linearly with the face count

Status: draft, not filed. Filing needs the owner's consent. If filed, open it in tangency-labs/solidcore (see draft 08
for the kernel-paused notice, tangency-labs/solidcraft#3992).

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.
**Duplicate search** (tangency-labs/solidcraft, read-only `gh search issues`): "healer slow", "healer performance":
nothing. Not searched: tangency-labs/solidcore issues.

## What happened

`ops::Body::from_record(&record, false)` runs the kernel's healer. On a closed network of bicubic B-spline faces
(one face per patch, edges shared, already valid) the time grows faster than linearly with the number of faces, so a
scan-fitted network of a few thousand patches takes tens of seconds, and every operation that decodes the body pays it.
`from_record(&record, true)` (no healer) on the same records takes 9 to 150 ms.

Measured on a synthetic cube sphere (`6 n^2` bicubic patches; control points on a sphere; closed; edges 2 per face),
`ci` profile of this workspace (`opt-level = 1`, dependencies optimised), one run each, the machine shared with other
builds, a single test process:

| faces | edges | `from_record(.., false)` | `from_record(.., true)` | time / faces (ms) |
|---|---|---|---|---|
| 600 | 1 200 | 1.45 s | 0.009 s | 2.4 |
| 1 014 | 2 028 | 2.73 s | 0.016 s | 2.7 |
| 2 166 | 4 332 | 8.85 s | 0.106 s | 4.1 |
| 4 056 | 8 112 | 30.0 s | 0.074 s | 7.4 |
| 8 214 | 16 428 | 91.4 s | 0.150 s | 11.1 |

The local exponent is about 1.0 between 600 and 1 014 faces, 1.95 between 2 166 and 4 056 and 1.6 between 4 056 and
8 214 (time ~ faces^e); four times the faces (1 014 to 4 056) took 11 times as long. SurfaceCraft also measured, on real
scan-fitted networks (not reproducible without the scans): 22 s at 4 152 patches and 65 s at 7 422. Run to run noise was
visible: a second run of the 1 014 and 4 056 face pair, concurrent with another test, gave 3.26 s and 24.5 s (x7.5).

## Steps to reproduce (public API only)

The record is assembled in `crates/kernel/tests/common/mod.rs` (`record_of`) with `solidcore-record` types only.

```rust
use solidcore_ops as ops;

let time = |n: usize| {
    let record = common::record_of(&common::cube_sphere(n, 50.0)); // 6 n^2 patches
    let start = std::time::Instant::now();
    ops::Body::from_record(&record, false).unwrap();
    start.elapsed().as_secs_f64()
};
let (small, large) = (time(13), time(26)); // 1 014 and 4 056 faces
assert!(large <= 6.0 * small); // linear would be 4x
```

Repro test: `healer_time_is_close_to_linear_in_the_face_count` in `crates/kernel/tests/upstream_repro.rs` (`#[ignore]`,
about 35 s; fails on `4f2da85` with `x11.0` alone and `x7.5` next to another test).

**Expected:** healing a valid, already consistent shell takes time near linear in its size (a pass over the entities
that finds nothing to do).
**Actual:** about faces^1.6 to faces^2.

Not measured: which healer stage takes the time (no profile), whether the cost depends on the topology (a cube sphere has
valence 3 and 4 vertices only), the release profile.

## The conversion without the healer is not linear either (large shells)

`from_record(&record, true)` on the same cube spheres (`ci` profile, shared machine, one run each):

| faces | `from_record(.., true)` | time / faces (us) |
|---|---|---|
| 16 224 | 0.59 s | 36 |
| 30 246 | 1.04 s | 34 |
| 60 000 | 2.01 s | 34 |
| 120 984 | 6.45 s | 53 |
| 240 000 | 21.5 s | 89 |

Linear up to about 60 000 faces, then growing: four times the faces (60 000 to 240 000) took 10.7 times as long (9.8 in a repeat). Through
SurfaceCraft's builder (adds our record check and the `to_record` that follows) the build is 3.4 s at 60 000 patches,
10.8 s at 120 984 and 37 s at 240 000; `solid::volume` of the body (a decode and a measurement) 9.5 s, 24.9 s and 111 s.
Read in the sources, not profiled: in `fill_shells` every face goes through `post_insert_faceuse`
(`solidcore/src/topology/brep/p10.rs`, about line 1556), which tests `shell.faces.contains(&f)` before pushing, a linear
scan per face, so the loop is quadratic in the faces of one shell (at 240 000 faces that is on the order of
3e10 comparisons). Repro test: `trusted_from_record_time_is_close_to_linear_in_the_face_count` in
`crates/kernel/tests/upstream_repro.rs` (`#[ignore]`, about 25 s). Expected: a set (or a flag on the face) instead of a
scan. Our limit follows from it: `MAX_NETWORK_PATCHES = 120 000`.

## Our workaround

`crates/kernel/src/solid.rs`: `decode` first converts a record with more than 2 000 faces as trusted (`from_record(rec,
true)`, no healer) and, when that conversion fails, retries with the healer (`from_record(rec, false)`); records up to
2 000 faces take the healer path first. It checks no provenance of the record. `crates/kernel/src/nurbs.rs` builds a
network of more than 2 000 patches without the healer, after its own checks (shared boundary curves and corners, every
edge in at most two faces, every shared edge traversed in opposite directions, a closed component enclosing a positive
volume). So a large body from a source other than that builder is converted without healing unless the trusted
conversion fails first, and the healer's cost stays what the table above shows if that fallback is taken.
