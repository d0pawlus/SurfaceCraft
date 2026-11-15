Status: posted 2026-11-15 with the owner's consent: https://github.com/tangency-labs/solidcraft/issues/13797#issuecomment-6898964499 . Before posting: to be posted as a comment on tangency-labs/solidcraft#13797 ("STEP: move SolidCraft's exporter into solidcore", open, `help wanted`, `area:io`, free again since 2026-11-15); a question with evidence read from the head source (solidcore f02d13e, 2026-11-15), not measured: nothing was run. Posting needs the owner's consent.

# Draft 17: comment on #13797, no PCURVEs on analytic faces

## Notes for the owner (not posted)

- Replaces the "comment on #13797" option at the end of draft 13's "What head covers and the gap" section; draft 13 itself stays unposted. This comment is the question only, no new writer and no API change.
- #13797's body (read 2026-11-15): "... exact surfaces with exact PCURVEs, voids and sheets ... Done when every corpus body round-trips (import, export, re-import) with identical face counts and volume to 1e-6". Its comments: kaltenbrunnermarkus claimed it (no draft PR), brett19 proposed a takeover in four steps (signed-volume tests, the AP242 layer, faceted export, the round-trip gate; none names PCURVEs on analytic faces), a bot says the claim lapsed. Commit `c44866a7b` ("STEP export: SolidCraft's AP242 product layer moves into solidcore-step ... (#13797)", 2026-11-12) moved the product layer; the issue stays open.
- Every line number below was checked in `W:\Claude\solidcore-head` at f02d13e9ce38f45a8cd4b5a794fa09ae81d2221e (`git log -1`: "Boundary surface: three curves of one direction that meet end to end are refused ... (#17243)"). The writer's design comment says the omission is on purpose (line 559-560 and commit `7c4c2f4e2`, "the path an analytic face with a form already takes (no pcurves)"); the comment therefore asks whether it is meant to stay, it does not call it a bug.
- The comment names no application. The consumer is described in one line ("a reader that re-imports a sheet and builds its trims from the file").
- Not claimed, because not checked: whether any external reader needs the PCURVEs. Draft 13's measurement of the old importer (sphere and torus faces, `(uses, uses_projected) == (0, 4)`) is not repeated here: that importer is deleted at head.
- Duplicate check (`gh issue list --repo tangency-labs/solidcraft --state all --search "PCURVE"`, read only): related reader-side issues only (#13794 "STEP reader: ... the file's PCURVEs as an option", #13793, #13634, #13814, #12859), none about the writer omitting them.

--- text to post ---

A question about scope, with evidence **read from the source at solidcore f02d13e, not measured** (I ran nothing for this).

The issue text says "exact surfaces with exact PCURVEs". At head, the writer in `crates/solidcore-step/src/write/topology.rs` writes no PCURVE for an edge use on a face that carries an analytic form:

```rust
// BrepWriter::edge_geometry (line 605)
for u in &edge.uses {
    let face = face_of(b, u.face)?;
    if face.nurbs.is_some() && !as_form(face, b.tolerance) {   // line 614
        continue;
    }
    ...
```

- `as_form` (line 329) is true only for a ball patch, a blend or a singular torus (a face that is written as its NURBS form).
- Every cylinder, cone, sphere and ordinary torus face carries its NURBS form: the record validator requires it (`crates/solidcore-record/src/brep.rs:860-873`, "a cylinder, cone, sphere, ball patch, torus, revolution or blend without its NURBS form"). So each use on such a face is skipped at line 614.
- When every use of an edge is skipped, `edge_geometry` returns the bare 3-D curve (line 624, `if associated.is_empty()`). An edge between two analytic faces, and every edge of a one-face sheet on an analytic surface, is therefore written as a bare curve; an edge between an analytic face and a plane or a spline carries only that other face's PCURVE.
- The omission looks intended: `BrepWriter::surface` says "A face on its NURBS form is written as the analytic surface it is, with no pcurves (STEP's are in the angle)" (lines 559-560), and `crates/solidcore-step/tests/write_signed_volume.rs:10-11` says "where the writer gives none (a face on an analytic surface)".
- On the reading side, the file's own PCURVEs are not used by solidcore's reader either (`crates/solidcore-step/src/topology.rs:33`), so the round-trip gate in this issue would not see the difference.

Question: is writing the PCURVE of a use on an analytic face in STEP's parameters (`SURFACE_CURVE`, `SEAM_CURVE` where a face uses an edge twice, the 3-D curve as master) meant to be part of this issue, or is "no PCURVEs on analytic faces" the intended end state? It matters for a reader that re-imports a sheet and builds its trims from the file: with no PCURVE it has to invert the surface from the 3-D curve.

--- end ---
