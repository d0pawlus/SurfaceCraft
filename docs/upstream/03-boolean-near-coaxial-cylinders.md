Status: to be filed; re-checked on head f02d13e on 2026-11-15, fails with upstream fixes off (status 1001) and on (NotSolid); no duplicate found (searches in "Notes for the owner" below). Filing needs the owner's consent.

# Draft 03: `boolean` (intersect) of two cylinders whose axes are 0.01 degrees apart

## Notes for the owner (not posted)

- Open in tangency-labs/solidcraft as kernel-only work: template "Bug report", labels `bug`, `area:kernel`.
- Title: Boolean (intersect) of two equal cylinders whose axes are 0.01° apart fails: kernel status 1001, or NotSolid with every `UpstreamFixes` flag on
- Pin: solidcore `f02d13e9ce38f45a8cd4b5a794fa09ae81d2221e` (tangency-labs/solidcore mirror, #17243), 2026-11-15, dev profile, Windows 11 x86_64. tangency-labs/solidcraft main is a different hash (21f5b7a6c at the time of writing).
- Measured: the test `near_coaxial_cylinders_intersect` in `W:\Claude\solidcore-head\crates\solidcore-ops\tests\surfacecraft_repro.rs`, with `SC_REPRO_FIXES` unset (status 1001) and `=1` (NotSolid). The test inlined below has its helpers (`ok`, `record_of`, `decode`, `to_kernel_matrix`, `fixes`) folded into one `read_back` and the matrix written in the `Body::transform` convention (the test's `turn`, transposed by `to_kernel_matrix`); the inlined copy has not been compiled separately. Compile and run it once before posting.
- I only had the two outcomes as summaries ("kernel status 1001", "the result is not a closed solid (NotSolid)"); paste the raw panic lines from the run if you want the exact text.
- The older numbers in the text (4f2da85, 2026-11-09) are marked as such: the 0.01° pair failed with `Status(1001)` there too, and 0.5°, 3° and 0° gave volumes 1551.9, 1459.4 and 1570.8 mm³. None of this was re-measured on head, so the text does not claim that only near-coincident axes fail (the old draft did).
- Duplicate searches (`gh issue list --repo tangency-labs/solidcraft --state all --search ...`, read only), no duplicate: "intersect cylinders near coaxial", "boolean cylinders axes 1001", "nearly parallel axes boolean", "coaxial cylinders boolean", "near-coincident cylinder axis intersect", "two cylinders turned slightly boolean intersect", "intersect NotSolid cylinders", "cylinder cylinder intersection almost same axis", "tilted cylinder 0.01 degrees". Hits read and judged related, not duplicates: #13656 (two coaxial cylinders a hair apart in radius, NotSolid or make_faceuse 1001; different input), #13802 (a tilted cylinder on a 4° edge, make_faceuse 1001), #1279 (cut with a cylinder coaxial with a hole, 0.1 mm inside its wall). The number 1001 in "#1001" is an unrelated issue.

--- text to post ---

## What happened

`ops::boolean::boolean(.., BooleanOp::Intersect)` of two cylinders of radius 5 and height 20, both based at the origin, the second turned 0.01° about x, returns `Err`:

- with every `UpstreamFixes` flag off (the default): kernel status 1001 (`SM_ERR`);
- with every flag on: the result is not a closed solid (`NotSolid`).

No lens solid is built either way.

## Steps to reproduce (public API only)

As a test in `crates/solidcore-ops/tests/`, on solidcore f02d13e:

```rust
use solidcore_ops as ops;

/// to_record, then from_record: the round trip every consumer of the record format makes.
fn read_back(b: &ops::Body) -> ops::Body {
    let (rec, _) = b.to_record().unwrap();
    ops::Body::from_record(&rec, false).unwrap().0
}

#[test]
fn near_coaxial_cylinders_intersect() {
    // with every flag on instead, first call
    // ops::session::set_thread_fixes(solidcore::kernel::UpstreamFixes::NAMES.iter().copied()).unwrap();
    let a = read_back(&ops::make_cylinder([0.0; 3], 5.0, 20.0).unwrap());
    let moving = read_back(&ops::make_cylinder([0.0; 3], 5.0, 20.0).unwrap());
    let (s, c) = 0.01_f64.to_radians().sin_cos();
    // Body::transform: rows 0 to 2 the images of x, y, z; row 3 the translation (p' = p·A + t)
    moving.transform(&[[1.0, 0.0, 0.0, 0.0], [0.0, c, s, 0.0], [0.0, -s, c, 0.0], [0.0, 0.0, 0.0, 1.0]]).unwrap();
    let b = read_back(&moving);
    let lens = ops::boolean::boolean(a, b, ops::BooleanOp::Intersect).unwrap(); // Err, see below
    let lens = lens.expect("the intersection came back empty (None)");
    let v = lens.volume(1e-6).unwrap();
    assert!((1400.0..=std::f64::consts::PI * 25.0 * 20.0).contains(&v), "lens volume {v}");
}
```

**Expected:** a lens solid whose volume lies between 1400 and π·5²·20 = 1570.80 mm³ (the cylinders are nearly the same body).
**Actual:** the `boolean` call returns `Err`: kernel status 1001 with the flags off, `NotSolid` ("the result is not a closed solid") with every flag on.

Version: solidcore f02d13e (dev profile), Windows 11 x86_64.

Earlier measurement, not repeated on head: on solidcore 4f2da85 (2026-11-09) the same pair also failed with `Status(1001)`, while the pair turned 0.5° intersected (volume 1551.9 mm³), 3° gave 1459.4 and 0° gave 1570.8.

## Open

I haven't traced where the intersection of the two near-coincident cylindrical faces goes wrong, and I don't suggest a fix. Related by theme, not the same input: #13656 (two coaxial cylinders a hair apart in radius, giving `NotSolid` or `make_faceuse` 1001).

--- end ---
