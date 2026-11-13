//! AutoSurface of an unstructured layout: repair and measure, refine until within the tolerance, then store the surface.
use super::super::{AutoBlock, AutoIn, AutoOut, CmdError, CmdResult};
use super::field::*;
use super::layout::*;
use crate::Session;
use std::sync::Arc;
use surfacecraft_doc::{ObjectId, QuadObject, SurfaceObject};
use surfacecraft_fit::boundary::Outline;
use surfacecraft_fit::orient::{repair_patches, same_side_normal};
use surfacecraft_fit::patch::{Deviation, PatchSet, deviation, edge_normal_jumps};
use surfacecraft_fit::retopo::QuadMesh;
use surfacecraft_geom::{KdTree, Point3, Vec3};
use surfacecraft_kernel::nurbs::{is_closed, sheet_from_bezier_network};
use surfacecraft_kernel::record::brep::Brep;

/// Refinements of an AutoSurface layout allowed before it is accepted as it is.
const MAX_AUTO_REFINEMENTS: usize = 3;

/// The scan an AutoSurface layout is fitted to, with the lookup tree for the reference normals.
struct AutoContext<'a> {
    q: &'a QuadObject,
    i: &'a AutoIn,
    tol: f64,
    max_patches: usize,
    scan: LayoutScan,
    tree: KdTree,
    outline: Outline,
}

/// The patches of a layout after repair, with their deviation from the scan.
struct Measured {
    radius: f64,
    set: PatchSet,
    dev: Deviation,
    repaired: usize,
    unverified: usize,
    /// Patches still flipped after the repair (0: otherwise the fit fails).
    flipped: usize,
}

/// Facts about the accepted fit that the report needs besides the surface.
struct Accepted {
    closed: bool,
    signed_volume: Option<f64>,
    refinements: usize,
    repaired: usize,
    unsnapped: usize,
    splits_rejected: usize,
    folded: usize,
    flipped_patches: usize,
}

/// The scan normal at a point, the reference of the orientation checks. It is reversed when the
/// layout has been turned around as a whole.
fn scan_reference<'a>(
    tree: &'a KdTree,
    normals: &'a [Vec3],
    flipped: bool,
) -> impl Fn(&Point3, &Vec3) -> Option<Vec3> + 'a {
    let sign = if flipped { -1.0 } else { 1.0 };
    move |p: &Point3, hint: &Vec3| same_side_normal(tree, normals, sign, p, hint)
}

/// Turns the patches of `layout` that run against the scan around, and measures the fit.
fn repair_and_measure(layout: &mut QuadMesh, scan: &LayoutScan, tree: &KdTree, flipped: bool) -> CmdResult<Measured> {
    let radius = layout_radius(layout, scan.spacing);
    let reference = scan_reference(tree, &scan.point_normals, flipped);
    let (set, report) = repair_patches(layout, &scan.scan, radius, &reference);
    if report.failed > 0 {
        return Err(CmdError::Failed(format!(
            "{} patches stay flipped against the scan after repair; retopologise again (fewer quads) or clean the region",
            report.failed
        )));
    }
    if set.nets.len() != layout.faces.len() {
        return Err(CmdError::Failed("the layout has quads with missing edges".into()));
    }
    let dev = deviation(&set, &scan.pts, 12);
    Ok(Measured { radius, set, dev, repaired: report.repaired, unverified: report.unverified, flipped: report.failed })
}

/// Turns every quad of `layout` around, together with its normal.
fn reverse_layout(layout: &mut QuadMesh) {
    for f in &mut layout.faces {
        f.swap(1, 3);
    }
    for n in &mut layout.normals {
        *n = -*n;
    }
}

/// The control nets of the patches of `set`, as the kernel builds them.
fn bezier_nets(set: &PatchSet) -> Vec<Vec<Vec<[f64; 3]>>> {
    set.nets.iter().map(|n| n.iter().map(|r| r.to_vec()).collect()).collect()
}

/// Adds the surface of an accepted fit (and its solid when closed) and returns their ids and the
/// number of patches.
fn store_auto_surface(
    s: &mut Session,
    q: &QuadObject,
    i: &AutoIn,
    sheet: Brep,
    dev: &Deviation,
    closed: bool,
) -> (ObjectId, Option<ObjectId>, usize) {
    let display = surfacecraft_kernel::tess::tessellate(&sheet, None).unwrap_or_default();
    let name = i.name.clone().unwrap_or_else(|| "autosurface".into());
    let patches = sheet.faces.len();
    let solid = if closed && i.make_solid.unwrap_or(true) {
        Some(s.doc.add_body(&format!("{name}_solid"), sheet.clone()))
    } else {
        None
    };
    let sid = s.doc.add_surface(SurfaceObject {
        id: ObjectId(0),
        name,
        sheet: Arc::new(sheet),
        region: q.region,
        control: [4, 4],
        degree: 3,
        rms: dev.rms,
        max: dev.max,
        display: Arc::new(display),
        parent: None,
        visible: true,
    });
    (sid, solid, patches)
}

/// The AutoSurface report of an accepted fit; stores its surface first.
fn finish_auto(s: &mut Session, ctx: &AutoContext<'_>, sheet: Brep, step: &Measured, accepted: &Accepted) -> AutoOut {
    let jumps = edge_normal_jumps(&step.set);
    let jmax = jumps.iter().copied().fold(0.0, f64::max).to_degrees();
    let jmean = (jumps.iter().sum::<f64>() / jumps.len().max(1) as f64).to_degrees();
    let within = step.dev.max <= ctx.tol;
    let mut notes: Vec<String> = Vec::new();
    if !within {
        notes.push(format!(
            "max deviation {:.4} mm exceeds {} mm after {} refinements; raise max_patches or retopologise finer",
            step.dev.max, ctx.tol, accepted.refinements
        ));
    }
    if step.unverified > 0 {
        notes.push(format!(
            "{} patches could not be checked against the scan orientation (no scan samples on their side); inspect them",
            step.unverified
        ));
    }
    if accepted.splits_rejected > 0 {
        notes.push(
            "a split of the refinement would have broken a proper quad and was rejected; the layout was not refined further"
                .to_string(),
        );
    }
    if accepted.unsnapped > 0 {
        notes.push(format!(
            "{} boundary loop splits could not be snapped back onto the region outline (kept as they were)",
            accepted.unsnapped
        ));
    }
    let warning = (!notes.is_empty()).then(|| notes.join("; "));
    let (sid, solid, patches) = store_auto_surface(s, ctx.q, ctx.i, sheet, &step.dev, accepted.closed);
    AutoOut {
        blocks: vec![AutoBlock { id: sid, patches, rms: step.dev.rms, max: step.dev.max }],
        max: step.dev.max,
        within_tolerance: within,
        warning,
        normal_jump_max_deg: Some(jmax),
        normal_jump_mean_deg: Some(jmean),
        closed: Some(accepted.closed),
        volume: accepted.signed_volume.map(f64::abs),
        solid,
        refinements: Some(accepted.refinements),
        uncovered: Some(step.dev.uncovered),
        folded: Some(accepted.folded),
        repaired_patches: Some(accepted.repaired),
        flipped_patches: Some(accepted.flipped_patches),
        unverified_patches: Some(step.unverified),
        outline_loops_unsnapped: Some(accepted.unsnapped),
        splits_rejected: Some(accepted.splits_rejected),
    }
}

/// Refines `layout` until its patches are within the tolerance or it can be split no further,
/// then stores the surface.
fn refine_and_fit(s: &mut Session, ctx: &AutoContext<'_>, mut layout: QuadMesh) -> CmdResult<AutoOut> {
    let mut refinements = 0usize;
    let mut flipped = false;
    let mut repaired = 0usize;
    let mut unsnapped = 0usize;
    let mut splits_rejected = 0usize;
    loop {
        let step = repair_and_measure(&mut layout, &ctx.scan, &ctx.tree, flipped)?;
        repaired += step.repaired;
        let can_split =
            layout.faces.len() * 4 <= ctx.max_patches && refinements < MAX_AUTO_REFINEMENTS && splits_rejected == 0;
        if step.dev.max > ctx.tol && can_split {
            let split = split_conformed(&layout, &ctx.scan.scan, &ctx.outline, step.radius);
            if split.rejected {
                // a rejected split stops the refinement: the layout stays as it is
                splits_rejected += 1;
                continue;
            }
            unsnapped += split.report.unsnapped();
            layout = split.layout;
            refinements += 1;
            continue;
        }
        let sheet = sheet_from_bezier_network(&bezier_nets(&step.set), &step.set.corners)?;
        let closed = is_closed(&sheet);
        let signed_volume = if closed { surfacecraft_kernel::solid::volume(&sheet).ok() } else { None };
        if signed_volume.is_some_and(|v| v < 0.0) && !flipped {
            // the layout runs against its normals: turn every quad around and build again
            reverse_layout(&mut layout);
            flipped = true;
            continue;
        }
        let accepted = Accepted {
            closed,
            signed_volume,
            refinements,
            repaired,
            unsnapped,
            splits_rejected,
            folded: layout.folded(),
            flipped_patches: step.flipped,
        };
        return Ok(finish_auto(s, ctx, sheet, &step, &accepted));
    }
}

/// AutoSurface of an unstructured layout: bicubic patches, refined until within `tol`.
pub(in crate::cmd::quad) fn auto_unstructured(
    s: &mut Session,
    q: &QuadObject,
    tol: f64,
    i: &AutoIn,
) -> CmdResult<AutoOut> {
    check_layout(q)?;
    let (pts, point_normals, spacing) = scan_for(s, q)?;
    if pts.len() < 20 {
        return Err(CmdError::Failed("the region has too few points".into()));
    }
    let max_patches = i.max_patches.unwrap_or(6000).clamp(1, MAX_LAYOUT_QUADS);
    let layout = layout_of(q);
    if layout.faces.len() > max_patches {
        return Err(CmdError::Params(format!("the layout has more than max_patches = {max_patches} quads")));
    }
    let scan = LayoutScan::from_parts(pts, point_normals, spacing);
    let tree = KdTree::build(&scan.pts);
    let outline = if i.conform_boundary.unwrap_or(false) { outline_for(s, q)? } else { Outline::default() };
    let ctx = AutoContext { q, i, tol, max_patches, scan, tree, outline };
    refine_and_fit(s, &ctx, layout)
}
