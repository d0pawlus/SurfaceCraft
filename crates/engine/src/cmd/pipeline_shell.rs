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
    DeviationSummary, Issue, MIN_COVERAGE, Method, NextStep, Settings, ToSolidIn, ToSolidOut, Verdict, issue,
    message_of, step, summary,
};
use crate::Session;
use crate::cmd::mesh::thicken::{ShellCounts, command_error, crossing_issues};
use crate::cmd::mesh_edit::{FillIn, fill};
use crate::cmd::qa::{CompareIn, compare_points};
use crate::cmd::wrap::{PATCH_LIMIT, WrapIn, WrapOut, wrap};
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
    /// Vertices whose offset was bounded up front by the distance to another part of the scan close to them along the surface (a bead, a crease): the offset sheet then stays on its side of the scanned sheet.
    pub limited_vertices: usize,
    /// Vertices of the scan moved (by at most one median edge) to remove crossings of the scan with itself (a fold of the sheet): the thickened face differs from the scan there by that much.
    pub repaired_vertices: usize,
    /// The largest distance a vertex of the scan was moved by that repair (mm). Deviation, coverage and the verdict are measured against the scan as it was; a repair larger than the tolerance, or at more than 0.5 % of the vertices, raises the issue `scan_repaired`.
    pub repair_largest_shift: f64,
    /// Largest reduction of the thickness at a vertex (mm).
    pub largest_reduction: f64,
    /// Median reduction of the thickness over the reduced vertices (mm).
    pub median_reduction: f64,
    /// Vertices of triangles that folded before the relaxation; the other reduced vertices were tapered.
    pub folded_vertices: usize,
    /// Boundary loops of the scan stitched to their offset copies (the rim, windows).
    pub boundary_loops: usize,
    /// Loops stitched besides the rim: real holes of the scan show as windows through the wall.
    pub windows: usize,
    /// Edges of the rim (the longest boundary loop).
    pub rim_edges: usize,
    /// Thinnest wall found (mm), sampled as the distance between the two sheets.
    pub min_wall: f64,
    /// Median wall found (mm).
    pub median_wall: f64,
    /// Vertices on a local crossing (neighbouring offset triangles, the flanks of a crease, the two sheets of a wall, the strip along the rim) that the relaxation could not remove even at the floor of the offset (a tenth of the thickness): sharp creases. More than 0.1 % of the vertices is refused.
    pub unresolved_vertices: usize,
    /// Triangles removed from the scan before the offset (fins on edges of three or more triangles, duplicates).
    pub removed_non_manifold: usize,
    /// Triangles of the scan turned around to make the orientation consistent.
    pub flipped_triangles: usize,
    /// Always true: an even sample of the thickened mesh was tested for triangles that cross one they share no vertex with (`self_intersection_samples` tested).
    pub self_intersection_checked: bool,
    /// Triangles of the sample tested.
    pub self_intersection_samples: usize,
    /// Of them, triangles that cross only triangles close to them along the scan (a fold of the offset; the relaxation removes these, so 0 unless `unresolved_vertices` is not).
    pub self_intersections_local: usize,
    /// Of them, triangles that cross a triangle far from them along the scan: the offset sheet meets another part of the shell (a channel or gap narrower than the wall). Only these raise the issue `self_intersection`.
    pub self_intersections: usize,
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
    if i.components.is_some() || i.part_share.is_some() {
        return Err(CmdError::Params(
            "thickness models the scan as one open shell: `components` and `part_share` do not apply (run `scan.split` first and thicken a part)".into(),
        ));
    }
    Ok(())
}

/// Cleans the scan, keeping its rim open, and thickens it. The rim is the largest boundary loop of the
/// CLEANED mesh (debris is gone by then): holes are filled up to the limit but never the rim.
pub(super) fn run(s: &mut Session, set: &Settings, i: &ToSolidIn) -> CmdResult<ToSolidOut> {
    let limit = i.fill_holes_max_edges.unwrap_or(super::FILL_HOLES_MAX_EDGES);
    let cleaned_only = ToSolidIn { fill_holes_max_edges: Some(0), ..i.clone() };
    let mut report = super::prepare(s, &cleaned_only, set.scan)?;
    let parts = s.doc.mesh(set.scan).map_or(1, |m| super::prepare::coequal_parts(&m.mesh));
    if parts > 1 {
        return Err(CmdError::Params(format!(
            "the scan has {parts} co-equal parts and thickness models one open shell: run `scan.split` first and thicken a part (`rebuild` does not apply to a thickened scan)"
        )));
    }
    let rim = s
        .doc
        .mesh(set.scan)
        .map_or(0, |m| surfacecraft_geom::edit::boundary_loops(&m.mesh).iter().map(Vec::len).max().unwrap_or(0));
    let fill_up_to = limit.min(rim.saturating_sub(1));
    if rim > 0 && fill_up_to > 0 {
        let edit = fill(s, FillIn { object: ObjRef::Id(set.scan.0), max_edges: Some(fill_up_to), smooth: Some(true) })?;
        report.holes_filled += edit.holes_filled.unwrap_or(0);
        report.holes_left = edit.holes_left.unwrap_or(report.holes_left);
        report.triangles = edit.triangles;
    }
    shell_solid(s, set, i, report)
}

fn info(shell: &Thickened, i: &ToSolidIn, kept: Option<ObjectId>) -> ThickenedInfo {
    ThickenedInfo {
        thickness: i.thickness.unwrap_or(0.0),
        symmetric: i.symmetric,
        clamped_vertices: shell.report.clamped_vertices,
        limited_vertices: shell.report.limited_vertices,
        repaired_vertices: shell.report.repaired_vertices,
        repair_largest_shift: shell.report.repair_largest_shift,
        largest_reduction: shell.report.largest_reduction,
        median_reduction: shell.report.median_reduction,
        folded_vertices: shell.report.folded_vertices,
        boundary_loops: shell.report.boundary_loops,
        windows: shell.report.boundary_loops.saturating_sub(1),
        rim_edges: shell.report.longest_loop_edges,
        min_wall: shell.report.min_wall,
        median_wall: shell.report.median_wall,
        unresolved_vertices: shell.report.unresolved_vertices,
        removed_non_manifold: shell.report.removed_non_manifold,
        flipped_triangles: shell.report.flipped_triangles,
        self_intersection_checked: true,
        self_intersection_samples: shell.report.crossing_samples,
        self_intersections_local: shell.report.crossing_local,
        self_intersections: shell.report.crossing_triangles,
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

/// The issue code of a refusal: crossings that stay at the floor of the offset have their own, so that
/// the reason is told from a scan that folds or cannot be wrapped.
fn refusal_code(e: &ThickenError) -> &'static str {
    match e {
        ThickenError::UnresolvedCrossings { .. } => "unresolved_crossings",
        _ => "no_solid",
    }
}

/// The result of a run whose scan could not be thickened: a verdict, not an error.
fn refused(set: &Settings, report: super::ScanSummary, e: &ThickenError, i: &ToSolidIn) -> ToSolidOut {
    let issues = vec![issue(refusal_code(e), format!("the scan could not be thickened: {e}"))];
    let mut next = Vec::new();
    if let ThickenError::Folds { suggested, .. } = e {
        // the sign of the thickness says where the material is: the suggestion keeps it
        let sign = if i.thickness.is_some_and(|t| t < 0.0) { -1.0 } else { 1.0 };
        let smaller = sign * (suggested * 100.0).floor() / 100.0;
        if smaller != 0.0 {
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
            "the wall is thinner than {} mm at {} of {vertices} vertices (reduced by {:.2} mm at most, by {:.2} mm at the median; the sampled wall is {:.3} mm at the thinnest, {:.3} mm at the median): the surface curves more tightly than the thickness there",
            t.thickness.abs(),
            t.clamped_vertices,
            t.largest_reduction,
            t.median_reduction,
            t.min_wall,
            t.median_wall
        );
        list.push(issue("thickness_reduced", msg));
    }
    let counts = ShellCounts {
        local: t.self_intersections_local,
        far: t.self_intersections,
        unresolved: t.unresolved_vertices,
        samples: t.self_intersection_samples,
        triangles: t.triangles,
        repaired: t.repaired_vertices,
        largest_shift: t.repair_largest_shift,
        vertices,
    };
    list.extend(crossing_issues(&counts, set.tolerance));
    if t.windows > 0 {
        let msg = format!(
            "{} boundary loops besides the rim ({} edges) were stitched through the wall as windows: a real hole of the scan shows as a window; fill it with a larger `fill_holes_max_edges` unless the part has the window",
            t.windows, t.rim_edges
        );
        list.push(issue("windows_stitched", msg));
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
        freeform::retry_larger_on(s, set, thick_id, first, |s, cap| wrap_shell(s, set, thick_id, Some(cap)))?;
    let face = shell.scan_face();
    let d = measure(s, set, &face, &w)?;
    let t = info(shell, i, None);
    // the rim stays open on purpose and the shell is closed over it: holes left are not a defect of the solid
    let judged = super::ScanSummary { holes_left: 0, ..report.clone() };
    let issues = issues_of(&w, &d, set, &judged, &t, face.vertex_count());
    let tried = retry.as_ref().is_some_and(|r| r.reason != freeform::RetryReason::BudgetSpent);
    let next = next_steps(&w, &issues, set, tried, i, &t);
    let verdict = if issues.is_empty() {
        Verdict::Pass
    } else if d.coverage >= MIN_COVERAGE {
        Verdict::Review
    } else {
        Verdict::Fail
    };
    Ok(freeform::out(set, retry, report, verdict, Some(&w), Some(d), issues, next))
}

/// The next steps of a thickened run: those of the freeform method that make sense for a shell, and
/// the ones of the thickness itself. The freeform ones that name the open scan (`surface.wrap` of the
/// scan, a repeat that drops or re-offers the thickness) are left out.
fn next_steps(
    w: &WrapOut,
    issues: &[Issue],
    set: &Settings,
    retried: bool,
    i: &ToSolidIn,
    t: &ThickenedInfo,
) -> Vec<NextStep> {
    let has = |code: &str| issues.iter().any(|x| x.code == code);
    let steps: Vec<NextStep> = freeform::next_steps(w, issues, set, retried)
        .into_iter()
        .filter(|st| st.command != "surface.wrap" && st.command != "mesh.fill_holes" && st.command != "scan.to_solid")
        .collect();
    let again = |extra: serde_json::Value| {
        let mut p =
            json!({"scan": set.scan, "thickness": t.thickness, "symmetric": i.symmetric, "tolerance": set.tolerance});
        if let (Some(base), Some(add)) = (p.as_object_mut(), extra.as_object()) {
            base.extend(add.clone());
        }
        p
    };
    let mut front = Vec::new();
    if has("thickness_reduced") {
        let carried = t.thickness.abs() - t.median_reduction;
        let thinner = (if carried > 0.0 { carried } else { 0.5 * t.thickness.abs() } * 100.0).floor() / 100.0;
        let params = again(json!({"thickness": thinner.copysign(t.thickness)}));
        front.push(step("scan.to_solid", params, "a thinner wall that the curvature of the surface can carry"));
    }
    if has("out_of_tolerance") && w.cap_used < PATCH_LIMIT {
        let params = again(json!({"max_patches": PATCH_LIMIT, "keep_thickened": true}));
        front.push(step("scan.to_solid", params, "the thickened shell needs more patches than the limit: wrap it again with the largest limit and keep the thickened mesh"));
    }
    if has("windows_stitched") {
        let params = again(json!({"fill_holes_max_edges": t.rim_edges.saturating_sub(1)}));
        front.push(step(
            "scan.to_solid",
            params,
            "fills every hole of the scan but the rim, so none is stitched as a window",
        ));
    }
    front.extend(steps);
    front
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crossings_that_stay_at_the_floor_are_refused_with_a_code_of_their_own() {
        // Arrange
        let crossings = ThickenError::UnresolvedCrossings {
            vertices: 380,
            total: 170_000,
            share_percent: 0.22,
            limit_percent: 0.1,
        };

        // Act and assert
        assert_eq!(refusal_code(&crossings), "unresolved_crossings");
        assert_eq!(refusal_code(&ThickenError::Unresolved(5)), "no_solid");
        let text = crossings.to_string();
        assert!(text.contains("380 of 170000") && text.contains("mesh.clean") && text.contains("0.1 %"), "{text}");
    }
}
