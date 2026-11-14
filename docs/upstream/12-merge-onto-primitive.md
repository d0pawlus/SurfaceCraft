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

Marked `APP-GEOMETRY:` (so `cargo xtask app-geometry` names them): `crates/kernel/src/merge.rs`,
`crates/kernel/src/merge/place.rs`, `crates/kernel/src/merge/snap.rs`.

| Function | Computes | Why no kernel entry | Closest kernel function |
|---|---|---|---|
| `merge::through`, `merge::pcurve`, `merge::loop_points`, `merge::face_loops`, `merge::validate_loops`, `merge::polygon_area`, `merge::inside` (cone and sphere faces only) | the trims of a face on a partial cone or sphere in the parameters of the surface's NURBS form, and the winding of its loops (outer counter-clockwise, holes clockwise and inside it) | `build_from_topology` gives a partial cone or sphere as an unlabelled rational spline (no `nurbs`, no analytic label), which the record validator then takes as a spline; the label is kept only for the kernel's own full forms | `step_brep::build_from_topology`, `trimming_tools::create_uv_trim_curves`, `fix_loop_orientation` |
| `place::seam_direction`, `place::padded_box`, `place::check_reach`, `place::finish`, the frames of `place::{plane, around, sphere}` and the `flip` choice | where the seam of a cylinder, cone or sphere goes (the widest gap in angle of the boundary samples), the parameter box and domain of the face in the NURBS form, and which side of the surface the patches face | the builder chooses the seam and box itself for plane and cylinder (used there); nothing returns them for a cone or sphere with the label kept, and the side is a question about the patches, not the surface | `step_brep` seam choice on import, `eval::form::surface` |
| `snap::moved_net`, `snap::applied`, `snap::plan`, `snap::follow` | moves a shared edge of a spline network onto a surface: replaces the matching border of the control net of every patch that uses the edge, moves the vertex, and the edges that end there | no edit of a patch network by an edge or a vertex; the record is edited directly | `approx_lst_sq` for the curve only (used) |
| `samples::split_round` (not marked: classification, not construction) | cuts a group that goes round the axis of a cylinder or cone into sectors by angle | the faces of a full revolution need a seam edge, which the merge does not make | `step_brep` seam insertion on import |

A request that would remove the last rows: an operation "make one analytic face of these faces of a body, whose
common surface is the primitive S, within tolerance t" on a `Brep` (boundary kept shared with the neighbouring
faces, boundary curves re-fitted onto S, neighbours updated, a seam added where the face goes round S).
