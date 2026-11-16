# Draft 13: the sag refinement of solidcore-tess runs to its cap when a face's boundary stands off its surface

Status: filed 2026-11-15 as tangency-labs/solidcraft#17431 (Bug report template, label `bug`). Reproduced on solidcore main
`f02d13e9c` with a kernel-only test, `crates/solidcore-tess/tests/offset_boundary_refine.rs` on the branch
`repro/surfacecraft-tess-recognise` of d0pawlus/solidcore (`971b413`). No duplicate in tangency-labs/solidcore or tangency-labs/solidcraft
(searches: "MAX_REFINE", "sag refinement", "mesher refine cap", "tessellation 40000 triangles", "tessellation boundary off surface").

What the issue says: a cylinder face (r 4 mm, one face, valid record) whose boundary edges lie on the cylinder of radius 4 + d, edge
tolerance d. With d = 0 the mesh never exceeds 192 triangles down to chord 0.008; with d = 0.05 it goes from 128 to about 40 000 triangles
at chord 0.02, with d = 0.1 at chord 0.05, and stays at the cap (`MAX_REFINE = 20_000`) at every finer chord. The area of the synthetic
face stays within 1 %; the wrong areas seen on faces of real data (41.9 and 74.9 mm2 at neighbouring chords for a face of 41.8 mm2)
were not reproduced with a synthetic face and are said so in the issue.

In SurfaceCraft: the hybrid merge (`scan.to_solid`) leaves a group's boundary up to 0.1 to 0.17 mm off its primitive when the snap is refused;
`areas_agree` then rejects the group (PR #142). Nothing is loosened on our side.
