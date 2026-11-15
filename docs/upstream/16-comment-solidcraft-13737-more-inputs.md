Status: posted 2026-11-15 with the owner's consent: https://github.com/tangency-labs/solidcraft/issues/13737#issuecomment-6898964179 . Before posting: to be posted as a comment on tangency-labs/solidcraft#13737 (open, `needs-triage`, no comments as of 2026-11-15); measured on head f02d13e on 2026-11-15 (three inputs refused with fixes off and on, a fourth refused with fixes off and read back with all fixes on). Posting needs the owner's consent.

# Draft 16: comment on #13737, three more inputs that hit the squareness refusal

## Notes for the owner (not posted)

- #13737 ("An extruded rectangle's own record doesn't read back: a side wall's plane axes are 1.2x past the reader's squareness bound") is about an extrusion of a rectangle on a sloped sketch plane, with u·v measured at 3.886e-16 against the bound `gamma(3)·|u|·|v|` = 3.331e-16. Its open question: whether the bound is too strict for planes the kernel itself makes, or the sweep should make the walls square. This comment does not repeat any of that; it adds producers of the same refusal that involve no sketch plane.
- Measured: tests `turned_box_reads_back_from_its_record` (d), `union_of_three_chained_rotated_copies_reads_back` (e), `window_along_a_plane_normal_reads_back` (g), `shell_of_filleted_pocketed_plate_reads_back` (f) in `W:\Claude\solidcore-head\crates\solidcore-ops\tests\surfacecraft_repro.rs`, solidcore `f02d13e9ce38f45a8cd4b5a794fa09ae81d2221e`, dev profile, Windows x86_64, `SC_REPRO_FIXES` unset and `=1`. d, e, g: refused in both; f: refused unset, passes with `=1`. I did not measure u·v of the refused planes, nor bisect which flag lets f through; the comment says so.
- The bound is at `crates/solidcore/src/surface/from_parameters.rs:402` (`du.dot(dv).abs() > gamma(3) * a * b`), the message at line 405 (it was line 400 in the issue, at 513e6fcfc).
- The inlined code of d and e is the tests' code with helpers folded in; the inlined copy has not been compiled separately. g is described, not inlined (about 30 lines in the test). Compile d and e once before posting if you want to be sure of the snippets.
- No GitHub login is written into the closing line (a bare `@name` notifies); house style in that issue is "Measured and written with Claude Code (AI) on behalf of @<login>": add yours if you want it.
- Duplicate check: `gh issue list --repo tangency-labs/solidcraft --state all --search` for "axes are not square" and "skew parametrisation" finds only #13737 itself for the message; "turned box from_record" finds nothing relevant. Nothing is posted to #13737 yet.

--- text to post ---

Three more inputs reach the same refusal on solidcore f02d13e (dev profile, Windows x86_64), none from an extrusion of a sketch on a sloped plane. In each, `Body::to_record` succeeds and `Body::from_record(&record, false)` returns

```
Refused { operation: "surface::from_parameters::from_parameters", reason: "the plane's axes are not square: the kernel's plane cannot hold a skew parametrisation" }
```

the check at `surface/from_parameters.rs:402-405` with the bound `gamma(3)·|u|·|v|` this issue measures. All three refuse with every `UpstreamFixes` flag off and with every flag on. I did not measure u·v of the planes they refuse.

Rotation helper used below (`Body::transform` takes rows 0 to 2 as the images of x, y, z and row 3 as the translation, `p' = p·A + t`; the axis is normalised):

```rust
fn turn(axis: [f64; 3], angle: f64) -> [[f64; 4]; 4] {
    let l = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
    let a = axis.map(|x| x / l);
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    [
        [t * a[0] * a[0] + c, t * a[0] * a[1] + s * a[2], t * a[0] * a[2] - s * a[1], 0.0],
        [t * a[0] * a[1] - s * a[2], t * a[1] * a[1] + c, t * a[1] * a[2] + s * a[0], 0.0],
        [t * a[0] * a[2] + s * a[1], t * a[1] * a[2] - s * a[0], t * a[2] * a[2] + c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}
```

**1. A box turned about a general axis** (a rigid rotation of `make_box`, nothing else):

```rust
let b = ops::make_box([0.0; 3], [50.0, 40.0, 10.0]).unwrap();
b.transform(&turn([-0.3902044387637371, 0.4445854841071728, 0.26358792709436607], 2.4184066553326042)).unwrap();
let (rec, _) = b.to_record().unwrap();
assert!(ops::Body::from_record(&rec, false).is_ok()); // Err(Refused { .. }) as above
```

**2. The union of three chained rotated copies** of a bar, 60° about z each (copy the previous copy, `transform`, copy again; `union_all`). The union's volume is right (219.215 within 0.01), its `to_record` succeeds, `from_record` refuses:

```rust
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
let (rec, _) = united.to_record().unwrap();
assert!(ops::Body::from_record(&rec, false).is_ok()); // Err(Refused { .. }) as above
```

**3. A window subtracted along a plane normal.** The box 50 x 40 x 10 at the origin, turned by `turn([0.26162006230833446, -0.20491162934367935, -0.3720612130285178], 1.38734577002826)`; it reads back. Take from its record the `Surface::Plane` face whose unit normal `n = u_axis x v_axis` has a dot product above 0.999 with the turned z axis (row 2 of the matrix). The tool is `ops::sweep::extrude` of a 12 x 8 rectangle, centred on the plate's centre (the turned (25, 20, 5)), in the plane normal to `n`, direction `n`, length `2d + 6` where `d` is the distance from the centre to that face along `n` (it starts 3 mm below the plate and ends 3 mm above it). Subtract it from the plate read back from its record: the boolean and `to_record` succeed, `from_record` of the result refuses. (Its direction is exactly the normal of one of the body's planes.)

**A fourth input reads back once the fixes are on.** A 40 x 20 x 10 plate extruded from four lines whose corners carry round-off (`(2.787812552530594e-16, 7.107075885548428e-15)`, `(40, 1.3354580823781616e-14)`, `(40, 20.000000000000018)`, `(3.119522659134018e-16, 20.00000000000002)`), a through hole of radius 4 at (15, 8) subtracted, the two vertical edges at (0, 0) and (40, 20) rounded with radius 1.5, the top face opened by `ops::shell` with wall 1.0, every step through `to_record` / `from_record`. With the fixes off, `from_record` of the shell refuses with the same message as above. With every `UpstreamFixes` flag on, it reads back and has a volume. I did not bisect which flag does it.

So the refusal is reached from the planes of a rigidly turned box (1), of a boolean of rotated copies (2) and of a cut along a plane's own normal (3), not only from the walls of an extrusion; 1 is the smallest: one box, one rotation. These bear on the open question here (is the reader's bound too strict for planes the kernel makes, or should the producers make planes square). I haven't traced which line of the turn, the union or the cut gives the planes their skew.

--- end ---
