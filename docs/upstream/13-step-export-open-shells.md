# Draft 13: STEP export of open shells (sheet bodies): NOT NEEDED at head, use `solidcore-step`

Update 2026-11-16 (P9-O8): the application writes sheets and solids with `solidcore_step::write::product::bodies` on the pin `c02c5c51c`; its own writer (`crates/io/src/step/{p21,mod,geom,topology,pcurve,solid,solid_check}.rs`) and `crates/kernel/src/pcurve.rs`'s STEP parameterisation are removed. Faces on analytic surfaces go without PCURVEs (draft 17, posted).

Status: draft, not filed, and probably not to be filed as a new issue. Checked against solidcore head `f02d13e9ce38f45a8cd4b5a794fa09ae81d2221e` (2026-11-15). Searched the head for: a STEP writer of open shells / `SHELL_BASED_SURFACE_MODEL` / `OPEN_SHELL` / `MANIFOLD_SURFACE_SHAPE_REPRESENTATION`, and for PCURVE / SEAM_CURVE writing. Found: `crates/solidcore-step/src/write/topology.rs` (`brep`, `layout`, `BrepWriter::shell`, `edge_geometry`) and `crates/solidcore-step/src/write/product.rs` (`bodies`, `products`, `solidcore_bodies`); tests in `crates/solidcore-step/tests/write_signed_volume.rs` (one-face sheets on cylinder, cone, sphere, torus; voids). Duplicates (`gh issue list --repo tangency-labs/solidcraft --state all`, read only): tangency-labs/solidcraft#13797 "STEP: move SolidCraft's exporter into solidcore" (open, free again since 2026-11-15; its text names "exact surfaces with exact PCURVEs, voids and sheets"), #13791 (delete step-import, step-brep, step-export, open), #13799 (the merge of the two pipelines, open). Maintainer's answer on tangency-labs/solidcraft#13797 (2026-11-16T03:33Z, https://github.com/tangency-labs/solidcraft/issues/13797): the reading of the writer matches the code; PCURVEs on analytic faces are part of the end state, but not of the move of the exporter itself; they come as their own pull request under tangency-labs/solidcraft#13799 right after the move, with a test that re-reads the written file building trims from its PCURVEs and compares them with the 3-D curves. State: announced by the maintainer, not merged. No reply needed.
Pin: `solidcore` rev `4f2da85c26f7d608808d94cffcf06ebe867ef61d` (2026-11-09).

## Result

The request this draft was written for (a writer of the open shells that the import side models as sheets) is already in solidcore, but not at the pin. `solidcore-step` arrived on 2026-11-10 (commit `5b9e153d2`, #13979), one day after the pin; `write/product.rs` followed on 2026-11-12 (`c44866a7b`, #15315). Use:

- `solidcore_step::write::product::bodies(Vec<ExportBody { name, record: &Brep, color, faces }>, &Header)`: STEP text (AP242) from schema-4 records, solids as `MANIFOLD_SOLID_BREP` / `BREP_WITH_VOIDS` in an `ADVANCED_BREP_SHAPE_REPRESENTATION`, sheets as one `SHELL_BASED_SURFACE_MODEL` of `OPEN_SHELL`s in a `MANIFOLD_SURFACE_SHAPE_REPRESENTATION`. Refusals are `ExportError::Body { name, reason }`.
- or `solidcore_step::write::topology::brep` with a `GeomWriter` for finer control.

The pinned `step-export` refuses open shells (`solid_shells`: "has an open shell; step-export writes closed solids", `BodyKind::ShellModel`: "is a shell model"); at head that is still so (`crates/step-export/src/lib.rs`, tests `rejects_open_manifold_shell`), and #13791 plans to delete the crate. Nothing to add there.

## What the application's writer produces (for comparison)

Not for posting. Written in the application (`crates/io/src/step/{mod,topology,pcurve,geom,p21}.rs`, adapted from SolidCraft's `crates/step` at fcd3186; the same origin as the writer moved into `solidcore-step`):

- AP214 product structure, millimetres, `MANIFOLD_SURFACE_SHAPE_REPRESENTATION` holding one `SHELL_BASED_SURFACE_MODEL` of `OPEN_SHELL`s of `ADVANCED_FACE`s (`FACE_OUTER_BOUND` / `FACE_BOUND`, `EDGE_LOOP` of `ORIENTED_EDGE`, `VERTEX_LOOP`), sides by face use.
- Every edge used by one or two faces is a `SURFACE_CURVE` (a `SEAM_CURVE` where one face on a cylinder, cone, sphere or torus uses it twice) with one `PCURVE` (a `DEFINITIONAL_REPRESENTATION` of a 2-D B-spline) per use; the 3-D curve is the master. A non-manifold edge (more than two uses) is written as the bare curve.
- Faces on a plane or a (rational) spline carry the record's own pcurves. Faces on a cylinder, cone, sphere or torus get pcurves in STEP's angle parameters, computed by the application's `crates/kernel/src/pcurve.rs`: exact iso-parameter lines. An analytic face with an edge that is not an iso-parameter curve (a slanted section) is refused, never approximated. A face with a NURBS form that is not one of those, a blend, a ball patch, a placed revolution, a wire shell: refused ("not supported yet").

## What head covers and the gap (read from the code, not run)

Head writes every item above, sheets included, and refuses what STEP cannot say exactly (`unsupported(..)` in `write/topology.rs`: wire and vertex shells, regions meeting across a shell, a torus with tube start and no NURBS form, a traced intersection curve).

One difference. A record face on a cylinder, cone, sphere or torus always carries its NURBS form (`solidcore-record/src/brep.rs`, validation "without its NURBS form"). `BrepWriter::edge_geometry` skips the pcurve of every use on such a face (`face.nurbs.is_some() && !as_form(..)`), and `BrepWriter::surface` says why: "STEP's are in the angle". So an edge between analytic faces is written as a bare curve, and `write_signed_volume.rs` states it too ("where the writer gives none (a face on an analytic surface)"). Pcurves are written for planes, splines, and for faces held as their NURBS form (ball patch, blend, singular torus). The application's writer gives analytic faces exact pcurves (hence the refusal of non-iso edges above); head gives none and therefore refuses nothing there.

Whether that matters is the owner's decision: a reader that needs PCURVEs to build the trims of an analytic face would project instead. If it matters, the smallest request is not a new writer but a comment on #13797: "write the pcurve of a use on an analytic face in STEP's parameters (iso-parameter lines in closed form, or the form's pcurve mapped to the angle); refuse an edge that is not one". No new API: `topology::brep` / `product::bodies` keep their signatures.

## The two open observations about the importer (not a claim of a kernel bug)

`docs/STATUS.md` records that the importer did not take the file's PCURVEs on sphere and torus faces. The importer meant is `step-brep` at the pin (`import_with_report_options` with `BuildOptions { file_pcurves: true }`), which is deleted at head (`ecd20d47c`, #14038). Its replacement documents that "the file's own PCURVE entities are not used" (`crates/solidcore-step/src/topology.rs`, module docs): pcurves are the closed forms of the 3-D edges or the kernel's own trims. So the observation does not carry over; it has to be measured again on the new reader, and the measurement would show "no file pcurve taken, on any face".

The observation at the pin, exact input: application test `analytic_patches_write_pcurves_that_lie_on_their_edges` (`crates/io/src/step/tests.rs`, module `patches`): a one-face sheet that is a box `u × v` of the sphere (resp. torus) bounded by exact circles and lines (the module's own description), written by `write_sheets`, 4 `PCURVE` entities, every pcurve within 1e-6 mm of its edge, then read by `step_brep::import_with_report_options` with file pcurves on. Result: one face, `fallback_faces == 0`, `(uses, uses_projected) == (0, 4)`. For cylinder and cone patches the same test gives `(uses, curved_uses) == (4, 4)`. The test's comment records that the pcurves pass the importer's own checks and that why the matching of its trim candidates (`create_uv_trim_curves`) fails for sphere and torus was not established.

## For the owner before anything is posted

1. Decide whether to post at all: the writer exists, and #13797 is the place if a pcurve request is wanted.
2. Moving the application to head means replacing `step_brep::brep_to_model` and `step_export::write` in `crates/kernel/src/step.rs` (`export_solids`), and `import_with_pcurves` (step-brep is gone); the PCURVE gates of P8-H1 (`gate_m3_sheet_step_roundtrip`) would have to be re-measured, since head writes none on analytic faces.
