# Request: put the boundary of a group of patches on a primitive, and make an analytic face of the group (draft)

Status: **draft only.** Nothing has been filed at tangency-labs/solidcore (filing needs the owner's consent).
Pin: `solidcore` rev `4f2da85c26f7d608808d94cffcf06ebe867ef61d`.

`scan.to_solid` with `method: hybrid` (P9-O6b) replaces each group of freeform patches that lies on a primitive
by one analytic face. What solidcore gave for it, and what had to be written in the application
(`crates/kernel/src/merge.rs`, `merge/snap.rs`), is below. The application would drop its code for every item
the kernel gets.

## What solidcore already gave (used)

- `SurfaceEval::project` / `point` on the analytic kinds: the closest point of a plane, cylinder, cone and
  sphere and its angle and height, in closed form. This replaced the application's distance of a point from a
  primitive and its parameter inversion.
- `step_brep::build_from_topology` on a one-face `step_import::model::Body`: the trims (pcurves) of the face in
  its own parameters, the surface's NURBS form and parameter box, the seam and the orientation of the loops,
  from 3-D edge curves on a plane or cylinder. Plane and cylinder faces come out labelled.
- `eval::form::surface`: the exact NURBS forms of the analytic kinds.
- `nurbs::crv_fit::p2::approx_lst_sq`: the least-squares fit of a Bézier edge through points put on a primitive.

## What has no kernel entry (still written in the application)

| Function | Computes | Why no kernel entry | Closest kernel function |
|---|---|---|---|
| `merge::through`, `merge::pcurve` (cone and sphere faces only) | the trims of a face on a partial cone or sphere in the parameters of the surface's NURBS form | `build_from_topology` gives a partial cone or sphere as an unlabelled rational spline (no `nurbs`, no analytic label), which the record validator then takes as a spline; the label is kept only for the kernel's own full forms | `step_brep::build_from_topology`, `trimming_tools::create_uv_trim_curves` |
| `merge::snap` (`moved_net`, `applied`) | moves a shared edge of a spline network onto a surface: re-fits the edge's Bézier curve and replaces the matching border of the control net of every patch that uses it, and moves the vertex | no edit of a patch network by an edge or a vertex; the record is edited directly | `approx_lst_sq` for the curve only |
| `merge::samples::split_round` | cuts a group that goes round the axis of a cylinder or cone into sectors by angle | the faces of a full revolution need a seam edge, which the merge does not make; the kernel's builder makes one only for a face it builds from a loop whose edges it may split | `step_brep` seam insertion on import |

A request that would remove the last two: an operation "make one analytic face of these faces of a body, whose
common surface is the primitive S, within tolerance t" on a `Brep` (boundary kept shared with the neighbouring
faces, boundary curves re-fitted onto S, neighbours updated, a seam added where the face goes round S).
