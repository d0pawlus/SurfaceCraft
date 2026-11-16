# Field tests through MCP

Sessions in which an agent drives the running desktop app over MCP (bridged mode) on real scans,
as a user would. Each finding names what was called, what was expected and what happened, so it
can become an issue or a pull request. Scans stay local (owner's files, never committed); only
counts and measurements are recorded here.

Status of a finding: **open**, **fixed in #n**, **wontfix (reason)**.

## 2026-11-16: water pump casting (8.1 M triangles)

Scan: closed casting of a water pump with impeller and internal passages, 265 x 221 x 121 mm,
8 146 012 triangles, 4 070 884 vertices. `scan.qa`: one component, no holes, no non-manifold
edges, no debris, noise σ 0.0036 mm, median edge 0.21 mm, no findings. Desktop app on Windows,
bridged MCP.

Runs (`scan.to_solid`, tolerance 0.2 mm, measured afterwards with `body.compare` at 0.2 mm):

| Call | Body | Faces | Within 0.2 mm | RMS | Max abs | Not facing |
|---|---|---|---|---|---|---|
| `method: auto` (defaults) | `solid` | 11 979 | 48.0 % | 1.30 mm | 5.0 mm (cap) | 620 856 of 2 894 725 |
| `method: freeform`, `max_patches: 60000` | `pump_freeform_60k` | 60 000 | 97.5 % | 0.078 mm | 3.06 mm | 30 189 of 4 040 696 |
| `method: hybrid`, `max_patches: 60000` | `pump_hybrid_60k` | 59 619 | 98.4 % | 0.060 mm | 1.83 mm | 26 991 of 4 043 894 |

STEP: `pump_freeform_60k` 136 MB, `pump_hybrid_60k` 1 121 MB (F-10). None of the three reaches the
99 % of a pass at 0.2 mm.

### F-1 (open, high): long commands lose their result in bridged mode

`scan_to_solid` called with `queued: true`, then `job_wait`: the job ends `failed` with
"the app did not answer", although the app finishes the command and the body appears in the
document. Happened on all three `scan.to_solid` calls of this scan. The `verdict`, `deviation`,
`attempts`, `issues` and `next_steps` of the result are lost, so the agent cannot follow the
documented workflow ("read `verdict`, follow `next_steps`") and has to rebuild the picture with
`doc.list` and `body.compare`.

Cause: `crates/ui-egui/src/control.rs` waits for the app's reply with a fixed
`recv_timeout(600 s)`. `queued: true` only moves the wait into an MCP job; the bridge itself still
gives up after ten minutes.

Expected: a queued command keeps running until it ends and `job_wait` returns its result, however
long it takes (or the app reports progress and the bridge waits while there is progress).

### F-2 (open, medium): the app does not answer anything while a long command runs

During the hybrid run, `doc.list` also failed with "the app did not answer". The bridge cannot
report status, list objects or cancel (`job_cancel`) while the UI thread runs a command. An
agent cannot tell "still computing" from "hung".

Expected: read-only commands and job status are served while a command runs, or the bridge
answers "busy: <command>, running for N s".

### F-3 (open, medium): `auto` on a large scan returns a wrap cut at the first-round cap

On 8.1 M triangles `auto` returned a freeform body of 11 979 faces with 48 % within tolerance
and 21 % of the measured points not facing the body. The larger-cap retry is skipped above 1.5 M
triangles (documented), so `auto` hands back a result that is clearly unusable when one call with
`max_patches: 60000` gives 97.5 %. Because of F-1 it is not known whether `next_steps` offered
that call.

Ideas: for scans above the retry limit start directly with a cap scaled to the scan (as the
`coarse_start` path does), or reduce the scan to the retry limit before the first wrap.

### F-4 (open, low): `body.compare` output too large for an MCP client

`body.compare` returns `per_face` for every face: 1.06 MB of JSON for 11 979 faces, 4.9 MB for
60 000 faces. Both exceeded the client's tool-result limit and had to be read from a saved file
with a script.

Expected: totals plus the N worst faces by default (a `worst` / `limit` parameter), the full list
on request.

### F-5 (open, low): `guide.read` of one topic is 49 KB

`guide_read topic: scan-to-solid` returns 49.6 KB of Markdown, more than the client shows inline
(saved to a file). The short path and "Issues and what to do" are what an agent needs first.

Idea: `section` parameter, or a short summary with the section headings and a second call per
section.

### F-6 (open, info): `scan.to_solid` changes the scan object between runs

Triangle count of the scan: 8 146 012 after import, 8 146 018 after the `auto` run, 8 146 014
after the freeform run (cleaning edits the scan in place). The regions of the `auto` run (104
regions) were gone after the freeform run. Documented for hole filling, but a user comparing runs
gets a slightly different scan each time; worth stating in the result (`scan_changed`) or
cleaning a copy.

### F-7 (open, high): the app stops answering after the hybrid run

Update, same day: about an hour after the call the app answered again and the body
`pump_hybrid_60k` was in the document. Between 14:05:50 and 14:07:50 the process used 168
CPU-seconds after an hour of near zero, i.e. the work that installs the result (adopting the
job's session, regions, primitives, the body) ran only once the app woke up; the owner had the
window in view at that time. This points at the second candidate below: the finished job waits for
a frame, and with no frame drawn (no user input, `wake()` not enough) nothing advances. To confirm:
run a long command over MCP with the window in the background and without touching it.

Original observation:

`scan_to_solid` with `method: hybrid`, `max_patches: 60000` (queued) ended with "the app did not
answer" after the bridge's 600 s (F-1). After that every call, `doc.list` included, waits the full
600 s and fails, while the app process is idle: 0.1 CPU-seconds in 20 s, about 1 CPU-second in
ten minutes (14 809 CPU-seconds in all, working set 1.8 GB, 53 threads, window not minimised and
`Responding`, one established bridge connection). So the UI thread is not running a command, yet
`poll_control` does not take the request.

Candidates, from `crates/ui-egui/src/jobs.rs`: `busy` stays `Some` although the job's thread has
ended (a result neither `Finished` nor `Lost`), or no frame is drawn, so `poll_control` never runs
(it only runs between frames, and `wake()` asks for one). Needs the app's status line and a stack
dump of the UI thread to tell. Expected: a job that ended in any way frees the app, and the bridge
can always say what the app is doing (F-2).

### F-9 (open, low): hybrid leaves its working objects in the document

After the hybrid run the document holds 104 new regions and 6 primitives besides the body, and
the earlier bodies were hidden. Merged faces: 60 000 patches became 59 619 faces, so about 400
patches went into analytic faces (the result with `analytic` was lost, F-1). For a user the
document is cluttered; an option to drop the working objects, or a group for them, would help.

### F-10 (open, medium): hybrid STEP is eight times larger than freeform

`export.step` of `pump_hybrid_60k` (59 619 faces): 1 121 451 809 bytes. Of `pump_freeform_60k`
(60 000 faces): 136 261 874 bytes.

Entity counts of the two files: `CARTESIAN_POINT` 537 822 (freeform) against 16 038 042 (hybrid).
In the hybrid file every one of its 238 506 `B_SPLINE_CURVE_WITH_KNOTS` (the 3D curves of the
edges and the pcurves alike) is degree 1 with 65 control points, a polyline; in the freeform file
the edges are cubic Béziers with 4 control points. So the hybrid body's edges, all of them and
not only those of merged faces, come out of the merge as sampled polylines. Besides the size, a
polyline edge is no longer tangent-continuous, which a CAD user will see on fillets and when
offsetting. Analytic faces in the file: 31 `PLANE`, 12 `CYLINDRICAL_SURFACE`, 4
`CONICAL_SURFACE`, 47 of 59 619.

Expected: the edges the merge did not touch keep their cubic curves; a moved border edge is
re-fitted as a cubic (the guide says Bézier, least squares).

### Manual attempt (same scan, no `scan.to_solid`)

Steps: fit primitives to the large regions, `align.datum` on the two flange planes (55.07 mm
apart) and the inlet axis, sections and sketches, extrude and revolve, `body.compare`. Result:
only the inlet pipe could be built by hand (revolve of a typed profile, cut of the bore on its own
axis): 78.6 % of its nearby scan points within 0.2 mm, RMS 0.26 mm, the misses at the end bead
whose shape was guessed and where the pipe meets the housing. The housing could not be built at
all; the blockers are F-11 to F-15.

### F-11 (open, high): `align.datum` puts the origin 68 m away

`align.datum` with primary a plane, secondary an axis lying in a plane parallel to it (the inlet
axis, `n · d` = 0.0003) and tertiary a point: the result moved the mesh by 67 921.9 mm; the bounding
box went to x = -68 051 … -67 773. Y and Z came out right (the axis at y = 0, the plane at z = 0).
The X origin looks taken from the intersection of the secondary axis with the primary plane,
which runs off to infinity when the axis is parallel to the plane, instead of from the tertiary
point. Worked around with `mesh.transform` (`translate` x + 67 922.605). Expected: an axis parallel
to the primary plane fixes Y (and is projected), the tertiary point fixes X.

### F-12 (open, medium): aligning a scan leaves its primitives and bodies behind

`align.datum` (and `mesh.transform`) moves the mesh only (`also` takes meshes). The nine
primitives fitted before the alignment stayed where they were, drawn across the viewport far
from the part, and had to be fitted again. The camera was not refitted either. Expected: an
option to move the objects derived from the scan with it, or a warning that they are now stale.

### F-13 (open, high): an extrude cannot pick profiles, and a failed one does not say which

`sketch.fit_chains` at z = 27.5 gave 14 closed profiles (outer outline, cores, holes), at z = 45
55 profiles (mostly small circles). `feature.extrude` takes every closed profile of a sketch and
failed both times with "kernel operation `sweep::extrude (hole)` failed: the result is not a
closed solid" (`sweep::extrude` at z = 45), without naming the profile. There is no parameter to
extrude one profile or a chosen set, and no command to delete entities from a sketch. Expected:
`profiles` (indices) on extrude and revolve, and the failing profile in the error.

### F-14 (open, high for agents): no read-out of sketch or section geometry

`inspect_design` gives a sketch as "120 entities, 4 constraints"; `sketch.fit_chains` returns
per-chain statistics; `section.stack` returns area, perimeter, size and fitted circles but no
polyline. An agent therefore cannot see a section outline, cannot copy part of it into its own
sketch, and cannot check which loop is which. Expected: a `sketch.get` (entities with their
coordinates, profiles with their loops) and polylines from `section.stack` on request
(`points: true`, decimated).

### F-15 (open, medium): `sketch.from_section` and `sketch.fit_chains` disagree

Same plane (z = 27.5, tolerance 0.2): `from_section` gave one sketch of 278 entities with 1 closed
profile, `fit_chains` 120 entities with 14 closed profiles (25 chains, 11 shorter than four
tolerances and dropped). A user would not know which to trust; the `from_section` profile count
suggests its chains were not closed.

### F-16 (open, low): small API inconsistencies met on the way

- `body.compare` names the scan `scan`, `body.refit_features` names it `mesh`.
- `primitive.info` has no extent (bounding box or parameter range of the region on the
  primitive), so the length of a cylinder had to be found with a stack of 18 sections.
- `sketch.fit_chains` lists every dropped chain in full (39 KB for the z = 45 section, 9 empty
  entries).
- `body.plan_regions` on the casting skipped 6 of 7 primitives (slot profile, boss or step, local
  plane): the primitives route does not apply to a casting, which is right, but `scan.to_solid`
  is then the only route an agent has.

### F-8 (open, info): 60 000 patches are not enough for a complex casting

The maximum cap gives 97.5 % within 0.2 mm, under the 99 % of a pass; there is no further step
except loosening the tolerance. The scan's own noise is 0.0036 mm, so the shortfall is shape, not
noise: thin vanes of the impeller and the internal passages. A cap scaled to the scan's area or a
per-part wrap (`scan.split` of the internal parts) would be needed.
