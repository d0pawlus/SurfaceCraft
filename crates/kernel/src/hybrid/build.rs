//! Building the solid of a hybrid model from cuts, free cuts and the plan of the regions.

use super::cutters::*;
use super::patch::*;
use super::vector::*;
use super::*;

/// Builds the solid. `lo`/`hi` bound the stock (it must contain the part, with room to spare);
/// planes and full-revolution shapes are applied in that order, then fillets, then freeform
/// cuts. Cuts the kernel cannot honour are reported as skipped; the solid is verified closed.
pub fn solid_from_cuts(lo: V, hi: V, cuts: &[Cut], free: &[FreeCut]) -> Result<Built> {
    solid_from_plan(lo, hi, cuts, free, &[], &[])
}

/// [`solid_from_cuts`] plus local tools (plan P3b, Z2): after the global cuts, each tool of
/// [`crate::local`] (blind hole, pocket, through slot) is subtracted from the stock.
pub fn solid_from_cuts_and_tools(
    lo: V,
    hi: V,
    cuts: &[Cut],
    free: &[FreeCut],
    tools: &[crate::local::LocalTool],
) -> Result<Built> {
    solid_from_plan(lo, hi, cuts, free, tools, &[])
}

/// The whole hybrid build (plan P3b, Z1 to Z3), in the order of the plan: the global cuts (planes,
/// full revolutions), the local subtractions (`tools`), the local additions (`adds`: bosses and
/// domes), then the blends of the partial cylinders, cones, tori and spheres among `cuts`
/// (fillets of one radius in one kernel call, largest first, so that the kernel makes the corner
/// patch where three meet; a plane-to-plane fillet is made before the revolutions, the rest after
/// the additions), and last the freeform cuts.
pub fn solid_from_plan(
    lo: V,
    hi: V,
    cuts: &[Cut],
    free: &[FreeCut],
    tools: &[crate::local::LocalTool],
    adds: &[crate::local::LocalAdd],
) -> Result<Built> {
    if !finite_all(&lo) || !finite_all(&hi) || (0..3).any(|i| hi[i] <= lo[i]) {
        return Err(KernelError::Argument("stock bounds must be finite with hi > lo".into()));
    }
    for c in cuts {
        check_shape(&c.shape)?;
    }
    if cuts.is_empty() && free.is_empty() {
        return Err(KernelError::Argument("nothing to build from".into()));
    }
    if tools.len() > 256 || adds.len() > 256 {
        return Err(KernelError::Argument("at most 256 local tools".into()));
    }
    let half = 4.0 * norm(sub(hi, lo)).max(1.0);
    let stock = ops::make_box(lo, sub(hi, lo)).map_err(|e| op_err("make_box", e))?;
    let mut body = stock;
    let mut status: Vec<CutStatus> = vec![CutStatus::Skipped("not applied".into()); cuts.len()];
    // 1. planes
    for (i, c) in cuts.iter().enumerate() {
        if let Shape::Plane { point, normal } = &c.shape {
            let n = unit(*normal).ok_or_else(|| KernelError::Argument("plane normal is zero".into()))?;
            body = half_space(&body, *point, n, half)?;
            if let Some(s) = status.get_mut(i) {
                *s = CutStatus::Applied("halfspace");
            }
        }
    }
    // 2. fillets of the edges the planes make (rounds that need a boss or a pocket wait)
    body = crate::blends::blend_pass(body, cuts, &mut status, false)?;
    // 3. full-revolution shapes
    for (i, c) in cuts.iter().enumerate() {
        match &c.shape {
            Shape::Plane { .. } => continue,
            // a partial cylinder, cone, torus or sphere is a blend (below)
            Shape::Cylinder { .. } | Shape::Cone { .. } | Shape::Torus { .. } | Shape::Sphere { .. } if !c.full => {
                if matches!(c.shape, Shape::Cone { .. })
                    && !c.convex
                    && let Some(s) = status.get_mut(i)
                {
                    *s = CutStatus::Skipped("a partial concave cone is not supported yet".into());
                }
                continue;
            }
            _ => {}
        }
        let tool = shape_solid(&c.shape, lo, hi)?;
        let (op, how) =
            if c.convex { (ops::BooleanOp::Intersect, "intersect") } else { (ops::BooleanOp::Subtract, "subtract") };
        body = ops::boolean::boolean(body, tool, op)
            .map_err(|e| op_err("boolean", format!("{e} ({how} {:?})", c.shape)))?
            .ok_or_else(|| KernelError::Op { op: "boolean", message: "the cut removed everything".into() })?;
        if let Some(s) = status.get_mut(i) {
            *s = CutStatus::Applied(how);
        }
    }
    // 3b. local tools: blind holes, pockets, slots
    let mut tstatus = Vec::with_capacity(tools.len());
    for t in tools {
        let (b, r) = crate::local::subtract_tool(body, t, lo, hi);
        body = b;
        tstatus.push(match r {
            Ok(crate::local::Applied::Done(how)) => CutStatus::Applied(how),
            Ok(crate::local::Applied::Partly(why)) => CutStatus::Skipped(why),
            Err(e) => CutStatus::Skipped(e.to_string()),
        });
    }
    // 3c. local additions: bosses and domes
    let mut astatus = Vec::with_capacity(adds.len());
    for t in adds {
        let (b, r) = crate::local::unite_add(body, t);
        body = b;
        astatus.push(match r {
            Ok(crate::local::Applied::Done(how)) => CutStatus::Applied(how),
            Ok(crate::local::Applied::Partly(why)) => CutStatus::Skipped(why),
            Err(e) => CutStatus::Skipped(e.to_string()),
        });
    }
    // 3d. the remaining blends: rounds and chamfers of the edges the pockets and bosses made
    body = crate::blends::blend_pass(body, cuts, &mut status, true)?;
    // 4. freeform tools
    let mut fstatus = Vec::with_capacity(free.len());
    for f in free {
        body = cut_by_spline_body(&body, &f.surface, f.outward)?;
        fstatus.push(CutStatus::Applied("split"));
    }
    if !body.is_manifold_solid().map_err(|e| op_err("is_manifold_solid", e))? {
        return Err(KernelError::Op {
            op: "solid_from_cuts",
            message: "the result is not a closed manifold solid".into(),
        });
    }
    let n = body.solid_count().map_err(|e| op_err("solid_count", e))?;
    if n != 1 {
        return Err(KernelError::Op {
            op: "solid_from_cuts",
            message: format!("the result has {n} solids, expected 1"),
        });
    }
    Ok(Built { brep: encode(&body)?, primitives: status, freeform: fstatus, tools: tstatus, adds: astatus })
}
