//! The hybrid method of `scan.to_solid` (P9-O6): one closed body in which the regions a primitive
//! fits are exact analytic faces (plane, cylinder, cone, sphere, torus) and the rest of the scan is a
//! network of freeform patches, judged against the scan like every other method.
//!
//! The construction is the trim-a-stock one of the hybrid model (plan P3), with the freeform wrap as
//! the stock instead of a box: the primitives are fitted and planned as for `body.from_regions`; a
//! copy of the scan is prepared whose primitive regions lie a little beyond (or, for an addition,
//! short of) their primitives (`snap`); the copy is rebuilt as a clean surface (`mesh.remesh`) and
//! wrapped in patches (`surface.wrap`); and the kernel trims that stock with the primitives
//! (`hybrid::cut_stock`), so the faces on the primitives are exactly the primitives and the patches
//! stay where no primitive fits. A cut the kernel refuses leaves its region as patches, at most the
//! offset from the scan. Anything else that goes wrong ends the attempt with an issue code and no
//! body; the document is as it was.
use super::freeform::{self, NetworkFacts, issues_for, verdict_of, wrap_mesh};
use super::regions::{candidates, fit_regions};
use super::{
    DeviationSummary, Issue, LeftOut, MAX_LISTED_FACES, MAX_LISTED_LEFT_OUT, Method, Modelled, NextStep, ScanSummary,
    Settings, ToSolidIn, ToSolidOut, Verdict, apply_roles, issue, message_of, region_limits, relate, step, summary,
    triangle_count,
};
use crate::Session;
use crate::cmd::hybrid::{FaceOf, collect_items, item_faces, snap_axes, square_deg};
use crate::cmd::mesh_remesh::{RemeshIn, remesh_cmd};
use crate::cmd::plan::{self, Item, RoleKind};
use crate::cmd::qa::{CompareIn, compare};
use crate::cmd::{CmdError, ObjRef, cancelled, is_cancelled};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use std::collections::HashMap;
use std::sync::Arc;
use surfacecraft_doc::ObjectId;
use surfacecraft_kernel::hybrid::{self, Built, Cut};
use surfacecraft_kernel::local::{LocalAdd, LocalTool};
use surfacecraft_kernel::record::brep::Brep;
use surfacecraft_kernel::tess::tessellate_topo;

#[path = "pipeline_hybrid_snap.rs"]
mod snap;
use snap::{SnapRegion, primitive_of, snapped};

/// How far beyond its primitive the prepared scan lies, as a share of the tolerance. The cut only has
/// to clear the fit error of the stock; a cut the kernel refuses leaves the region this far from the
/// scan, which stays inside the tolerance.
const OFFSET_SHARE_OF_TOLERANCE: f64 = 0.4;
/// The voxel of the rebuild, as a share of the tolerance (the default of `mesh.remesh` is 1.5): finer,
/// because the hybrid body is judged against the scan like a plain wrap, and a coarse voxel rounds
/// the sharp edges the patches next to the primitives follow.
const REMESH_VOXEL_SHARE: f64 = 0.75;
/// The surface of the rebuilt scan has about this many quads at most: the voxel grows with the area.
const REMESH_QUAD_BUDGET: f64 = 1_500_000.0;
/// The most patches the stock may have. The kernel's booleans on a network are slow and fragile as
/// it grows (measured: a few seconds at 400 patches, 15 s at 2 000, refusals above); a network that
/// would be larger is not cut.
pub(super) const STOCK_MAX_PATCHES: usize = 4_000;
/// Chord of the tessellation the area of the faces is measured on, as a share of the tolerance.
const AREA_CHORD_SHARE: f64 = 1.0;

/// How much of the body is exact geometry.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct AnalyticSummary {
    /// Faces on a plane, cylinder, cone, sphere or torus.
    pub faces: usize,
    /// All faces of the body.
    pub total_faces: usize,
    /// Share of the body's surface area on those faces (0..1).
    pub area_share: f64,
}

/// Why an attempt stopped.
enum Halt {
    Cancelled(CmdError),
    Failed { code: &'static str, message: String },
}

type Step<T> = Result<T, Halt>;

fn halt(code: &'static str, message: impl Into<String>) -> Halt {
    Halt::Failed { code, message: message.into() }
}

/// The halt for a command's error: a cancellation stays one, anything else is `code`.
fn failed_at(code: &'static str) -> impl Fn(CmdError) -> Halt {
    move |e| if is_cancelled(&e) { Halt::Cancelled(e) } else { halt(code, message_of(&e)) }
}

fn check_cancel(s: &Session) -> Step<()> {
    if s.cancel.is_cancelled() { Err(Halt::Cancelled(cancelled())) } else { Ok(()) }
}

/// The primitives fitted to the regions of the scan.
struct Fitted {
    modelled: Vec<Modelled>,
    left: Vec<LeftOut>,
    unassigned: usize,
    relations: usize,
}

fn fit_primitives(s: &mut Session, i: &ToSolidIn, set: &Settings, report: &ScanSummary) -> Step<Fitted> {
    let (list, unassigned) = candidates(s, set.scan);
    let total = triangle_count(s, set.scan);
    let (min_triangles, rms_limit) = region_limits(set, total, report);
    let (modelled, left) = fit_regions(s, &list, min_triangles, rms_limit);
    if modelled.is_empty() {
        return Err(halt("hybrid_no_primitives", "no region of the scan could be modelled as a primitive"));
    }
    let relations = if i.constrain { relate(s, &modelled, report.noise_sigma) } else { 0 };
    Ok(Fitted { modelled, left, unassigned, relations })
}

/// What the planner made of the primitives: the cuts, tools and additions for the kernel.
struct Planned {
    items: Vec<Item>,
    plan: plan::Plan,
    cuts: Vec<Cut>,
}

fn plan_primitives(s: &Session, modelled: &[Modelled]) -> Step<Planned> {
    let refs: Vec<ObjRef> = modelled.iter().map(|m| ObjRef::Id(m.primitive.0)).collect();
    let lift = failed_at("hybrid_failed");
    let (items, _, scan) = collect_items(s, &refs, &mut HashMap::new(), |_| {}).map_err(&lift)?;
    let plan = plan::plan(&items, &scan, square_deg(None).map_err(&lift)?);
    let mut cuts = plan.cuts.clone();
    snap_axes(&mut cuts);
    Ok(Planned { items, plan, cuts })
}

/// The regions that become analytic faces, as the prepared scan sees them.
fn snap_regions(s: &Session, p: &Planned) -> Vec<SnapRegion> {
    let mut out = Vec::new();
    for (k, item) in p.items.iter().enumerate() {
        let Some(role) = p.plan.roles.get(k) else { continue };
        let (shape, is_addition) = match role.kind {
            RoleKind::Global => {
                let Some(cut) = p.plan.cut_members.iter().position(|m| m.contains(&k)).and_then(|c| p.cuts.get(c))
                else {
                    continue;
                };
                (cut.shape.clone(), false)
            }
            RoleKind::LocalSubtract => (role.shape.clone(), false),
            RoleKind::LocalAdd => (role.shape.clone(), true),
            // a blend is not cut into a patch stock, and a skipped region has no primitive
            RoleKind::Blend | RoleKind::Skipped => continue,
        };
        let triangles =
            s.doc.primitive(item.id).and_then(|o| o.region).and_then(|r| s.doc.region(r)).map(|r| r.triangles.to_vec());
        if let Some(triangles) = triangles {
            out.push(SnapRegion { triangles, primitive: primitive_of(&shape), convex: item.cut.convex, is_addition });
        }
    }
    out
}

/// The stock: a body wrapped over the prepared scan.
struct Stock {
    body: ObjectId,
    patches: usize,
    capped: bool,
}

/// Rebuilds the prepared scan as a clean closed surface (`mesh.remesh`); a scan the remesh refuses
/// ends the attempt.
fn remesh_prepared(s: &mut Session, set: &Settings, prepared: ObjectId) -> Step<ObjectId> {
    let area = s.doc.mesh(prepared).map_or(0.0, |m| m.mesh.area());
    let voxel = (REMESH_VOXEL_SHARE * set.tolerance).max((area / REMESH_QUAD_BUDGET).sqrt());
    let input = RemeshIn {
        object: ObjRef::Id(prepared.0),
        tolerance: Some(set.tolerance),
        voxel: Some(voxel),
        min_feature: None,
        max_hole: None,
        min_component_fraction: None,
        name: Some(format!("{}-remesh", set.name)),
    };
    match remesh_cmd(s, input) {
        Ok(r) => Ok(r.id),
        Err(e) if is_cancelled(&e) => Err(Halt::Cancelled(e)),
        Err(e) => Err(halt(
            "hybrid_not_closed",
            format!("the scan cannot be rebuilt as a closed surface: {}", message_of(&e)),
        )),
    }
}

/// Prepares the scan, rebuilds it as a clean surface and wraps it. The two meshes made on the way
/// are removed again.
fn make_stock(s: &mut Session, set: &Settings, planned: &Planned) -> Step<Stock> {
    let scan = s.doc.mesh(set.scan).map(|m| m.mesh.clone()).ok_or_else(|| halt("hybrid_failed", "the scan is gone"))?;
    let delta = OFFSET_SHARE_OF_TOLERANCE * set.tolerance;
    let prepared = snapped(&scan, &snap_regions(s, planned), delta);
    let prepared_id = s.doc.add_mesh(&format!("{}-prepared", set.name), prepared);
    let remeshed = remesh_prepared(s, set, prepared_id)?;
    check_cancel(s)?;
    let cap = set.max_patches.unwrap_or(STOCK_MAX_PATCHES).min(STOCK_MAX_PATCHES);
    let wrapped = wrap_mesh(s, set, remeshed, Some(cap)).map_err(failed_at("hybrid_stock_failed"));
    s.doc.remove(prepared_id);
    s.doc.remove(remeshed);
    let w = wrapped?;
    if !w.closed {
        return Err(halt(
            "hybrid_stock_open",
            "the patch network over the scan is not closed, so it cannot be trimmed",
        ));
    }
    Ok(Stock { body: w.body.id, patches: w.patches, capped: w.capped })
}

/// Trims the stock with the cuts, tools and additions of the plan.
fn trim_stock(s: &Session, stock: &Stock, planned: &Planned) -> Step<Built> {
    let brep =
        s.doc.body(stock.body).map(|b| b.brep.clone()).ok_or_else(|| halt("hybrid_failed", "the stock is gone"))?;
    let tools: Vec<LocalTool> = planned.plan.tools.iter().map(|t| t.tool.clone()).collect();
    let adds: Vec<LocalAdd> = planned.plan.adds.iter().map(|a| a.add.clone()).collect();
    hybrid::cut_stock(&brep, &planned.cuts, &tools, &adds).map_err(|e| halt("hybrid_cut_failed", e.to_string()))
}

/// Analytic faces and their share of the surface area, from a tessellation that knows its faces.
fn analytic_summary(brep: &Brep, tolerance: f64) -> Option<AnalyticSummary> {
    let analytic = hybrid::analytic_faces(brep);
    let topo = tessellate_topo(brep, Some(AREA_CHORD_SHARE * tolerance)).ok()?;
    if topo.tri_face.len() != topo.mesh.triangle_count() {
        return None;
    }
    let (mut on, mut all) = (0.0, 0.0);
    for (t, &face) in topo.tri_face.iter().enumerate() {
        let Some([a, b, c]) = topo.mesh.triangle(t) else { continue };
        let area = 0.5 * (b - a).cross(&(c - a)).norm();
        all += area;
        if analytic.binary_search(&(face as usize)).is_ok() {
            on += area;
        }
    }
    let share = if all > 0.0 { (on / all).clamp(0.0, 1.0) } else { 0.0 };
    Some(AnalyticSummary { faces: analytic.len(), total_faces: brep.faces.len(), area_share: share })
}

/// Everything the result is made of once the body is built and measured.
struct Measured {
    deviation: DeviationSummary,
    worst_faces: Vec<surfacecraft_deviation::faces::FaceDeviation>,
    analytic: AnalyticSummary,
    faces_of: Vec<FaceOf>,
    body: crate::cmd::body::BodyOut,
}

/// Puts the trimmed solid in the stock's body and measures it against the scan.
fn measure(s: &mut Session, set: &Settings, stock: &Stock, built: Built, planned: &Planned) -> Step<Measured> {
    let brep = Arc::new(built.brep.clone());
    let analytic = analytic_summary(&brep, set.tolerance)
        .ok_or_else(|| halt("hybrid_failed", "the faces could not be measured"))?;
    if analytic.faces == 0 {
        return Err(halt(
            "hybrid_no_analytic_faces",
            "the trimmed body has no face on a primitive (no cut reached the stock)",
        ));
    }
    let diag = 0.02 * set.tolerance.max(1.0) + 1e-3;
    let faces_of = item_faces(&planned.items, &planned.plan, &planned.cuts, &built, diag);
    let slot = s.doc.body_mut(stock.body).ok_or_else(|| halt("hybrid_failed", "the stock is gone"))?;
    slot.brep = brep;
    let cmp = compare(
        s,
        CompareIn {
            body: ObjRef::Id(stock.body.0),
            scan: ObjRef::Id(set.scan.0),
            tolerance: Some(set.tolerance),
            max_distance: None,
            chord: None,
            facing: true,
        },
    )
    .map_err(failed_at("hybrid_failed"))?;
    let vertices = s.doc.mesh(set.scan).map_or(0, |m| m.mesh.vertex_count());
    let deviation = summary(&cmp, vertices);
    let worst_faces =
        cmp.per_face.into_iter().filter(|f| f.count > 0 && f.within < 1.0).take(MAX_LISTED_FACES).collect();
    let body = crate::cmd::body::body_out(s, stock.body).map_err(failed_at("hybrid_failed"))?;
    Ok(Measured { deviation, worst_faces, analytic, faces_of, body })
}

/// The next steps of a hybrid result.
fn next_steps(m: &Measured, issues: &[Issue], set: &Settings) -> Vec<NextStep> {
    if issues.is_empty() {
        let why = "the solid passed: export it (pass `path` when files are allowed, else read `data`)";
        return vec![step("export.step", json!({"objects": [m.body.id]}), why)];
    }
    let mut steps = Vec::new();
    if issues.iter().any(|i| i.code == "out_of_tolerance") {
        let why = "moves the analytic faces of the solid onto the scan (planes shift, radii change)";
        steps.push(step("body.refit_faces", json!({"body": m.body.id, "mesh": set.scan}), why));
    }
    let why = "what each verdict and issue means and how to go on by hand";
    steps.push(step("guide.read", json!({"topic": "scan-to-solid"}), why));
    steps
}

/// The hybrid attempt without the clean-up of a failure.
fn build(s: &mut Session, i: &ToSolidIn, set: &Settings, report: &ScanSummary) -> Step<ToSolidOut> {
    let mut fitted = fit_primitives(s, i, set, report)?;
    check_cancel(s)?;
    let planned = plan_primitives(s, &fitted.modelled)?;
    let stock = make_stock(s, set, &planned)?;
    check_cancel(s)?;
    let built = trim_stock(s, &stock, &planned)?;
    let measured = measure(s, set, &stock, built, &planned)?;
    apply_roles(&mut fitted.modelled, &measured.faces_of);
    let facts = NetworkFacts { closed: true, patches: stock.patches, capped: stock.capped };
    let issues = issues_for(&facts, &measured.deviation, set, report);
    let next = next_steps(&measured, &issues, set);
    let verdict = verdict_of(&issues, &measured.deviation);
    let mut out =
        freeform::out(set, None, report.clone(), verdict, None, Some(measured.deviation.clone()), issues, next);
    out.method = Method::Hybrid;
    out.body = Some(measured.body);
    out.patches = Some(stock.patches);
    out.worst_faces = measured.worst_faces;
    out.analytic = Some(measured.analytic);
    out.relations = fitted.relations;
    out.left_out_regions = fitted.left.len();
    out.left_out_triangles = fitted.left.iter().map(|l| l.triangles).sum();
    out.left_out = fitted.left.into_iter().take(MAX_LISTED_LEFT_OUT).collect();
    out.unassigned_triangles = fitted.unassigned;
    out.modelled = fitted.modelled;
    Ok(out)
}

/// The result of an attempt that ended in `code`: no body, and nothing in the document.
fn failure(set: &Settings, report: ScanSummary, code: &str, message: &str) -> ToSolidOut {
    let why = "what each verdict and issue means and how to go on by hand";
    let next = vec![
        step(
            "scan.to_solid",
            json!({"scan": set.scan, "method": "freeform", "tolerance": set.tolerance}),
            "wraps the whole scan in patches instead",
        ),
        step("guide.read", json!({"topic": "scan-to-solid"}), why),
    ];
    let issues = vec![issue(code, format!("no hybrid body was built: {message}"))];
    let mut out = freeform::out(set, None, report, Verdict::Fail, None, None, issues, next);
    out.method = Method::Hybrid;
    out
}

/// The hybrid method: the regions of the scan (segmented by the caller) become primitives where they
/// fit, the rest patches, in one closed body. A failure other than a cancellation is a result with
/// an issue code and the document as it was.
pub(super) fn hybrid_solid(
    s: &mut Session,
    i: &ToSolidIn,
    set: &Settings,
    report: ScanSummary,
) -> Result<ToSolidOut, CmdError> {
    let before = s.doc.clone();
    match build(s, i, set, &report) {
        Ok(out) => Ok(out),
        Err(Halt::Cancelled(e)) => {
            s.doc = before;
            Err(e)
        }
        Err(Halt::Failed { code, message }) => {
            s.doc = before;
            Ok(failure(set, report, code, &message))
        }
    }
}
