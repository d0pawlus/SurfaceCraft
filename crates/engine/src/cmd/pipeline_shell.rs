//! The shell method of `scan.to_solid`: an open, one-sided scan (a panel, a casting scanned from the
//! top) is thickened into a closed shell on the mesh side (`mesh.thicken`) and the shell is wrapped in
//! a patch network (`surface.wrap`), because the kernel makes no solid of a sheet of many faces.
//!
//! The deviation and the verdict are measured against the scan, never against the shell: the scan's
//! points are compared with the face of the body they lie on (`Thickened::scan_face`), which is the
//! scan itself for `thickness` of either sign and the scan moved half the thickness along its
//! normals for `symmetric` (the scan then lies in the middle of the wall, where it has no face).
use super::freeform::{self, CapRetry};
use super::{
    DeviationSummary, Issue, MIN_COVERAGE, Method, Settings, ToSolidIn, ToSolidOut, Verdict, issue, message_of, step,
    summary,
};
use crate::Session;
use crate::cmd::mesh::thicken::command_error;
use crate::cmd::qa::{CompareIn, QaIn, compare_points, scan_qa};
use crate::cmd::wrap::{WrapIn, WrapOut, wrap};
use crate::cmd::{CmdError, CmdResult, ObjRef};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};
use serde_json::json;
use surfacecraft_doc::ObjectId;
use surfacecraft_geom::Mesh;
use surfacecraft_geom::shell::{ThickenError, ThickenParams, Thickened, thicken};

/// A wall thinner than requested at more than this share of the vertices is an issue (a few
/// reduced vertices in the noise of a scan are not).
const REDUCED_ISSUE_SHARE: f64 = 0.01;

/// What the shell method did.
#[derive(Clone, Debug, Deserialize, Serialize, JsonSchema)]
pub struct ThickenedInfo {
    /// Wall thickness asked for (mm).
    pub thickness: f64,
    /// Half of it on each side of the scan.
    pub symmetric: bool,
    /// Vertices whose offset was reduced because the surface curves more tightly than the thickness.
    pub clamped_vertices: usize,
    /// Largest reduction of the thickness at a vertex (mm).
    pub largest_reduction: f64,
    /// Boundary loops of the scan stitched to their offset copies (the rim, windows).
    pub boundary_loops: usize,
    /// Triangles of the thickened mesh.
    pub triangles: usize,
    /// The thickened mesh, when `keep_thickened` kept it in the document.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub mesh: Option<ObjectId>,
}

/// Rejects the combinations of `thickness` with other parameters that mean nothing.
pub(super) fn check(i: &ToSolidIn) -> CmdResult<()> {
    let Some(t) = i.thickness else {
        if i.symmetric || i.keep_thickened {
            return Err(CmdError::Params("`symmetric` and `keep_thickened` need `thickness`".into()));
        }
        return Ok(());
    };
    if !(t.is_finite() && t != 0.0) {
        return Err(CmdError::Params("thickness must be a finite number other than 0 (mm)".into()));
    }
    if i.method == Method::Primitives {
        return Err(CmdError::Params(
            "thickness makes a shell from an open scan with method `freeform` (or `auto`): it has no meaning with method `primitives`".into(),
        ));
    }
    if i.reuse_regions {
        return Err(CmdError::Params(
            "thickness has no meaning with `reuse_regions` (the scan is not segmented)".into(),
        ));
    }
    Ok(())
}

/// With a `thickness`, the largest boundary loop of the scan is its rim and must stay open: holes
/// are filled only up to one edge less than it. Without a thickness `i` is returned as it is.
pub(super) fn keep_rim_open(s: &Session, mut i: ToSolidIn, scan: ObjectId) -> CmdResult<ToSolidIn> {
    if i.thickness.is_none() {
        return Ok(i);
    }
    let qa = scan_qa(s, QaIn { object: ObjRef::Id(scan.0) })?;
    if let Some(rim) = qa.hole_list.first() {
        let wanted = i.fill_holes_max_edges.unwrap_or(super::FILL_HOLES_MAX_EDGES);
        i.fill_holes_max_edges = Some(wanted.min(rim.vertices.saturating_sub(1)));
    }
    Ok(i)
}

fn info(shell: &Thickened, i: &ToSolidIn, kept: Option<ObjectId>) -> ThickenedInfo {
    ThickenedInfo {
        thickness: i.thickness.unwrap_or(0.0),
        symmetric: i.symmetric,
        clamped_vertices: shell.report.clamped_vertices,
        largest_reduction: shell.report.largest_reduction,
        boundary_loops: shell.report.boundary_loops,
        triangles: shell.mesh.triangle_count(),
        mesh: kept,
    }
}

fn wrap_shell(s: &mut Session, set: &Settings, mesh: ObjectId, max_patches: Option<usize>) -> CmdResult<WrapOut> {
    wrap(
        s,
        WrapIn {
            mesh: ObjRef::Id(mesh.0),
            tolerance: Some(set.tolerance),
            min_within: Some(set.min_within),
            max_patches,
            refine: None,
            crease_deg: None,
            continuity: None,
            name: Some(set.name.clone()),
        },
    )
}

/// The result of a run whose scan could not be thickened: a verdict, not an error.
fn refused(set: &Settings, report: super::ScanSummary, e: &ThickenError, i: &ToSolidIn) -> ToSolidOut {
    let issues = vec![issue("no_solid", format!("the scan could not be thickened: {e}"))];
    let mut next = Vec::new();
    if let ThickenError::Folds { suggested, .. } = e {
        let smaller = (suggested * 100.0).floor() / 100.0;
        if smaller > 0.0 {
            let params = json!({"scan": set.scan, "thickness": smaller, "symmetric": i.symmetric});
            next.push(step("scan.to_solid", params, "a wall the surface can carry (a thinner one folds less)"));
        }
    }
    next.push(step(
        "guide.read",
        json!({"topic": "scan-to-solid"}),
        "what each verdict and issue means and how to go on by hand",
    ));
    freeform::out(set, None, report, Verdict::Fail, None, None, issues, next)
}

/// Measures the shell's body against the scan, as its points on `face` (the face the scan lies on).
fn measure(s: &Session, set: &Settings, face: &Mesh, w: &WrapOut) -> CmdResult<DeviationSummary> {
    let cmp = compare_points(
        s,
        w.body.id,
        set.scan,
        face,
        &CompareIn {
            body: ObjRef::Id(w.body.id.0),
            scan: ObjRef::Id(set.scan.0),
            tolerance: Some(set.tolerance),
            max_distance: None,
            chord: None,
            facing: true,
        },
    )?;
    Ok(summary(&cmp, face.vertex_count()))
}

/// Issues of the run: those of the wrapped shell, and a wall that is not what was asked for.
fn issues_of(
    w: &WrapOut,
    d: &DeviationSummary,
    set: &Settings,
    report: &super::ScanSummary,
    t: &ThickenedInfo,
    vertices: usize,
) -> Vec<Issue> {
    let mut list = freeform::issues_of(w, d, set, report);
    if t.clamped_vertices as f64 > REDUCED_ISSUE_SHARE * vertices as f64 {
        let msg = format!(
            "the wall is thinner than {} mm at {} of {vertices} vertices (down by up to {:.2} mm): the surface curves more tightly than the thickness there",
            t.thickness.abs(),
            t.clamped_vertices,
            t.largest_reduction
        );
        list.push(issue("thickness_reduced", msg));
    }
    list
}

/// Thickens the cleaned scan, wraps the shell and judges the result against the scan.
pub(super) fn shell_solid(
    s: &mut Session,
    set: &Settings,
    i: &ToSolidIn,
    report: super::ScanSummary,
) -> CmdResult<ToSolidOut> {
    let (thickness, symmetric) = (i.thickness.unwrap_or(0.0), i.symmetric);
    let scan = s.doc.mesh(set.scan).ok_or_else(|| CmdError::Params(format!("no mesh {}", set.scan)))?;
    let (name, mesh) = (scan.name.clone(), scan.mesh.clone());
    let shell = match thicken(&mesh, &ThickenParams { thickness, symmetric }) {
        Ok(t) => t,
        Err(ThickenError::Closed) => {
            return Err(CmdError::Params(
                "thickness was given, but the scan is closed (it has no open boundary after cleaning): there is nothing to thicken; leave `thickness` out to model it as a solid".into(),
            ));
        }
        Err(e @ (ThickenError::BadThickness(_) | ThickenError::TooThick { .. })) => return Err(command_error(&e)),
        Err(e) => return Ok(refused(set, report, &e, i)),
    };
    let thick_id = s.doc.add_mesh(&format!("{name} thickened"), shell.mesh.clone());
    let result = wrap_and_judge(s, set, i, report, &shell, thick_id);
    let kept = i.keep_thickened.then_some(thick_id);
    if kept.is_none() {
        s.doc.remove(thick_id);
    }
    result.map(|mut out| {
        out.thickened = Some(info(&shell, i, kept));
        out
    })
}

fn wrap_and_judge(
    s: &mut Session,
    set: &Settings,
    i: &ToSolidIn,
    report: super::ScanSummary,
    shell: &Thickened,
    thick_id: ObjectId,
) -> CmdResult<ToSolidOut> {
    let first = match wrap_shell(s, set, thick_id, set.max_patches) {
        Ok(w) => w,
        Err(e) => {
            let why = format!("the thickened scan could not be wrapped in patches: {}", message_of(&e));
            let issues = vec![issue("no_solid", why)];
            let next = vec![step(
                "guide.read",
                json!({"topic": "scan-to-solid"}),
                "what each verdict and issue means and how to go on by hand",
            )];
            return Ok(freeform::out(set, None, report, Verdict::Fail, None, None, issues, next));
        }
    };
    let (w, retry): (WrapOut, Option<CapRetry>) =
        freeform::retry_larger(s, set, first, |s, cap| wrap_shell(s, set, thick_id, Some(cap)));
    let face = shell.scan_face();
    let d = measure(s, set, &face, &w)?;
    let t = info(shell, i, None);
    // the rim stays open on purpose and the shell is closed over it: holes left are not a defect of the solid
    let judged = super::ScanSummary { holes_left: 0, ..report.clone() };
    let issues = issues_of(&w, &d, set, &judged, &t, face.vertex_count());
    let next = freeform::next_steps(&w, &issues, set, retry.is_some());
    let verdict = if issues.is_empty() {
        Verdict::Pass
    } else if d.coverage >= MIN_COVERAGE {
        Verdict::Review
    } else {
        Verdict::Fail
    };
    Ok(freeform::out(set, retry, report, verdict, Some(&w), Some(d), issues, next))
}
