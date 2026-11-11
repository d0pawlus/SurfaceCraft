# Draft 06: shell of a filleted plate with a pocket does not read back from its record (1007)

Status: draft, not filed. Filing needs the owner's consent. Open it in tangency-labs/solidcraft as
kernel-only work (template: Bug report, labels `bug`, `area:kernel`).

**solidcore commit:** `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.
**Duplicate search:** "shell fillet record", "SM_ERR_INVALID_INPUT": hits #12485 and #11915 ("SHELL: operation failed",
app reports where the shell itself fails; different symptom), #13827 (shell/offset/thicken reconstruction plan; related
area, not a duplicate). Same symptom as #13737 (status 1007 from `from_record` of a record `to_record` wrote); the shared
cause is a hypothesis, not measured. Recommended: comment on #13737 or file separately, the owner decides.

## What happened

A 40 x 20 x 10 plate (outline from a constraint solver, so its corner coordinates carry round-off of 1e-14 to 1e-16),
a through hole of radius 4 at (15, 8), two vertical edges at opposite corners rounded with radius 1.5, the top face
opened with a wall of 1.0 (`ops::shell`, positive thickness). `shell` and `to_record` succeed; `from_record` of the
result fails with `Status(1007)`. With the outline coordinates exactly 40.0 and 20.0 the same chain reads back
(volume 2335.8 mm3), so the failure is sensitive to round-off at that level. In SurfaceCraft's parametric gate
it shows in 2 of 20 random edits (where the shell is not measurable afterwards).

## Steps to reproduce (public API only)

```rust
use solidcore_ops as ops;
use solidcore_record::brep::{Brep, Surface};

fn round_trip(b: &ops::Body) -> ops::Body {
    let (rec, _) = b.to_record().unwrap();
    ops::Body::from_record(&rec, false).unwrap().0
}
fn vertical_edge_at(rec: &Brep, x: f64, y: f64) -> usize {
    rec.edges.iter().position(|e| {
        let p = rec.vertices[e.vertices[0] as usize].point;
        let q = rec.vertices[e.vertices[1] as usize].point;
        (p[0] - q[0]).abs() < 1e-9 && (p[1] - q[1]).abs() < 1e-9 && (p[2] - q[2]).abs() > 1.0
            && (p[0] - x).abs() < 1e-6 && (p[1] - y).abs() < 1e-6
    }).unwrap()
}
fn top_face(rec: &Brep) -> usize {
    let mut best: Option<(usize, f64)> = None;
    for (i, f) in rec.faces.iter().enumerate() {
        let Surface::Plane { origin, u_axis: u, v_axis: v } = &f.surface else { continue };
        let n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]];
        let l = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        if (n[2] / l).abs() > 0.99 && best.map_or(true, |(_, z)| origin[2] > z) {
            best = Some((i, origin[2]));
        }
    }
    best.unwrap().0
}

let corner = [
    [2.787812552530594e-16, 7.107075885548428e-15, 0.0],
    [40.0, 1.3354580823781616e-14, 0.0],
    [40.0, 20.000000000000018, 0.0],
    [3.119522659134018e-16, 20.00000000000002, 0.0],
];
let outline: Vec<ops::sweep::Curve> =
    (0..4).map(|i| ops::sweep::Curve::Line { start: corner[i], end: corner[(i + 1) % 4] }).collect();
let z = [0.0, 0.0, 1.0];
let plate = ops::sweep::extrude(&outline, z, 10.0, 0.0, Default::default(), true).unwrap();
let hole = ops::sweep::Curve::Arc {
    center: [15.0, 8.0, 0.0], axis: z, reference: [1.0, 0.0, 0.0],
    radius: 4.0, start: 0.0, end: std::f64::consts::TAU,
};
let cutter = ops::sweep::extrude(&[hole], z, 12.0, 0.0, Default::default(), true).unwrap();
let pocketed = ops::boolean::boolean(round_trip(&plate), round_trip(&cutter), ops::BooleanOp::Subtract)
    .unwrap().unwrap();
let (rec, _) = pocketed.to_record().unwrap();
let (e0, e1) = (vertical_edge_at(&rec, 0.0, 0.0), vertical_edge_at(&rec, 40.0, 20.0));
let (body, _) = ops::Body::from_record(&rec, false).unwrap();
ops::blend(&body, ops::Blend::Round, &[e0, e1], &[1.5], &[], ops::Corners::Filled).unwrap();
let (rec, _) = body.to_record().unwrap();
let top = top_face(&rec);
let (body, _) = ops::Body::from_record(&rec, false).unwrap();
let (_, order) = body.to_record().unwrap();
ops::shell(&body, &order, &[top], 1.0).unwrap();   // succeeds
let (rec, _) = body.to_record().unwrap();          // succeeds
ops::Body::from_record(&rec, false).unwrap();      // Err(Status(1007))
```

**Expected:** the shell's record reads back and the body has a volume.
**Actual:** `Status(1007)`.

Version: solidcore `4f2da85`, Windows 11, Rust stable.

## Verification

- `shell_of_filleted_pocketed_plate_reads_back` fails on `4f2da85` with `shelled plate: from_record: Status(1007)`
  (run before the rule).
- Found on the way: the same shell with a negative thickness (inward) fails inside `ops::shell` with `Status(1001)` for
  13 of 120 random plates (scratch exploration on `4f2da85`, not committed); that is a separate symptom, not drafted.

## SurfaceCraft workaround

`crates/engine/tests/p5_parametric.rs` counts such steps as "unmeasured" instead of failing.
