Status: filed 2026-11-15 as tangency-labs/solidcraft#17437 (as a question, no `bug` label; with the owner's consent; the posted text is the part to post below, with the inlined tests run on solidcore `f02d13e` with upstream fixes off and on where there are any). Before filing: to be filed; re-checked on head f02d13e on 2026-11-15, fails with upstream fixes off and on; no duplicate found (searches in "Notes for the owner" below). Filing needs the owner's consent.

# Draft 02: the round of a box's straight edge is recorded as `Surface::Cone` with half angle 0

## Notes for the owner (not posted)

- Title: A round of a box's straight edge is recorded as `Surface::Cone` with half angle 0, not `Surface::Cylinder`
- Labels: `bug`, `area:kernel`.
- Open in tangency-labs/solidcraft as kernel-only work: template "Bug report" (`.github/ISSUE_TEMPLATE/bug.yml`: What happened / Steps to reproduce / version), labels `bug`, `area:kernel` (`needs-triage` is added by the repository). The text below keeps the form's three headings and #13737's measured / read / open split. It may be triaged as a design question rather than a bug; the title says only what the record holds.
- Pin: solidcore `f02d13e9ce38f45a8cd4b5a794fa09ae81d2221e` (tangency-labs/solidcore, "Boundary surface: three curves of one direction ... (#17243)"), 2026-11-15, dev profile, Windows 11 x86_64. That hash is in the solidcore mirror; tangency-labs/solidcraft main moved on (21f5b7a6c at the time of writing) and does not know it.
- Measured: the test `fillet_of_box_edge_is_a_cylinder_face` in `W:\Claude\solidcore-head\crates\solidcore-ops\tests\surfacecraft_repro.rs` fails with `SC_REPRO_FIXES` unset and `=1`. The test inlined below is that test with its helpers (`ok`, `record_of`, `decode`, `fixes`) removed; the inlined copy has not been compiled separately. Compile and run it once (a file in `crates/solidcore-ops/tests/`) before posting.
- The exact face-kind list printed by the assertion was given to me as "six planes and `cone(half_angle 0)`"; I did not have the raw line. Paste it from the run if you want the order exact.
- Duplicate searches (`gh issue list --repo tangency-labs/solidcraft --state all --search ...`, read only), all without a duplicate: "fillet cone half angle", "blend cylinder cone", "fillet cylinder reported as cone", "round of an edge Cone half_angle 0", "zero half angle cone", "cone half angle record", "Surface::Cone half_angle", "round of a straight edge is a cone", "fillet face kind cylinder", "record cylinder as cone". Related, not duplicates, cited in the text: #13815 (trims leave cones of semi-angle about 1e-13 where the face is a cylinder, open) and #17228 (a cone rebuild with radii within EFF_ZERO records a cylinder without checking the net, open).
- The record doc does bear on the report, and cuts both ways (read from the source, quoted in the text): a half angle below 1e-12 is allowed only on a face that carries its NURBS form, and it is documented as the label of a cone "narrower than that, its apex past 10^12 radii"; a cone of exactly 0 has no apex. So the doc does not say a zero half angle is the form of a cylinder, and it does not forbid it either: `Brep::validate` accepts this record (owner ruling item 8, option B, in `solidcore-record/src/brep/tests.rs`). Two kernel tests also assert the zero-half-angle cone as the round's form, so the issue is worded as a question, not as a defect claim.

--- text to post ---

## What happened

`ops::blend` with `Blend::Round` on one straight edge of a box gives a record whose round face is `Surface::Cone { half_angle: 0, .. }`, not `Surface::Cylinder`. The geometry is right; only the surface kind differs. A reader of the record that handles a cylinder has to treat a cone of half angle 0 as a cylinder too.

## Steps to reproduce (public API only)

As a test in `crates/solidcore-ops/tests/`, on solidcore f02d13e:

```rust
use solidcore_ops as ops;
use solidcore_record::brep::Surface;

#[test]
fn fillet_of_box_edge_is_a_cylinder_face() {
    let made = ops::make_box([0.0; 3], [20.0, 10.0, 5.0]).unwrap();
    let (rec, _) = made.to_record().unwrap();
    let body = ops::Body::from_record(&rec, false).unwrap().0;
    ops::blend(&body, ops::Blend::Round, &[0], &[2.0], &[], ops::Corners::Filled).unwrap();
    let (rec, _) = body.to_record().unwrap();
    let kinds: Vec<String> = rec
        .faces
        .iter()
        .map(|f| match &f.surface {
            Surface::Plane { .. } | Surface::PlaneThrough { .. } => "plane".to_string(),
            Surface::Cylinder { .. } => "cylinder".to_string(),
            Surface::Cone { half_angle, .. } => format!("cone(half_angle {half_angle})"),
            _ => "other".to_string(),
        })
        .collect();
    let cylinders = rec.faces.iter().filter(|f| matches!(f.surface, Surface::Cylinder { .. })).count();
    assert_eq!(cylinders, 1, "one cylindrical round expected, face kinds were {kinds:?}");
}
```

It fails the same way with every `UpstreamFixes` flag off (the default) and with every flag on (`ops::session::set_thread_fixes(solidcore::kernel::UpstreamFixes::NAMES.iter().copied())` first).

**Expected:** one `Surface::Cylinder` face (the round, radius 2, axis along the edge).
**Actual:** no cylinder face. The faces are six planes and one `Surface::Cone` with `half_angle` 0 (the assertion's list reads six `plane` and `cone(half_angle 0)`).

Version: solidcore f02d13e (dev profile), Windows 11 x86_64.

## Read from the source (not run)

- The record's own doc (`crates/solidcore-record/src/brep.rs:254`): "Its half angle is at least 10⁻¹² unless the face carries its NURBS form, which is then the face's geometry (a cone narrower than that, its apex past 10¹² radii)." A bare cone below 1e-12 is refused (`brep.rs:686`); a face label is accepted (`face_label`, `brep.rs:612`; test `degenerate_cone_and_torus_labels_are_valid_only_with_their_form`). So `validate` accepts this record, by the exemption written for a cone with its apex past 10¹² radii. A cone of half angle exactly 0 has no apex, and `Surface::Cylinder` is a kind of its own in the same enum.
- Which label a face gets is decided by `AnalyticSupport::of` (`crates/solidcore/src/surface/implicit.rs:1013`): `SurfaceKind::Cone(c) if c.cylinder` is a `Cylinder`; any other cone goes out as `Cone` with its semi-angle. `Cone::is_cylinder` (`surface/cone/p1.rs:1078`) is `self.cylinder || self.semi_angle_deg.abs() < EFF_ZERO`, and the kernel's own tests assert that the rounds of straight edges are cones of semi-angle 0 (`crates/solidcore-ops/tests/blend.rs` near line 258) for which `is_cylinder()` holds (`crates/solidcore-ops/tests/issue_3012_filleted_boxes_apart.rs:135`).

## Open

Whether a cone of semi-angle 0 is the intended record form for the round of a straight edge (then this is a request to say so in the record's doc), or the record should name it `Cylinder` where the kernel's own `is_cylinder()` holds. I don't suggest a fix. Related: #13815 (the same shape from trims, a cone of semi-angle about 1e-13 where the face is a cylinder) and #17228 (the cone rebuild that records a cylinder).

--- end ---
