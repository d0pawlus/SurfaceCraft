//! The freeform method of `scan.to_solid`: the whole scan wrapped in a patch network
//! (`surface.wrap`) and measured against the scan, for parts no set of primitives describes.
use super::{
    DeviationSummary, MAX_FILL_EDGES, MIN_COVERAGE, Method, NextStep, ScanSummary, Settings, ToSolidOut, Verdict,
    coverage_issue, issue, message_of, step, summary,
};
use crate::Session;
use crate::cmd::qa::{CompareIn, compare};
use crate::cmd::wrap::{PATCH_LIMIT, WrapIn, WrapOut, robust_diagonal, scale_warning, wrap};
use crate::cmd::{CmdResult, ObjRef};
use serde_json::json;

/// What keeps a wrapped scan from a pass.
fn issues_of(w: &WrapOut, d: &DeviationSummary, set: &Settings, report: &ScanSummary) -> Vec<super::Issue> {
    let mut list = Vec::new();
    if !w.closed {
        let msg = "the patch network is not closed (the scan has open boundaries): the result is a sheet, not a solid";
        list.push(issue("open_surface", msg.to_string()));
    }
    list.extend(coverage_issue(d, "the patches"));
    if d.count > 0 && d.within < set.min_within {
        let msg = format!(
            "{:.2} % of the measured points are within {} mm ({:.0} % required); max {:.3} mm, RMS {:.3} mm; {} patches{}",
            100.0 * d.within,
            set.tolerance,
            100.0 * set.min_within,
            d.max_abs,
            d.rms,
            w.patches,
            if w.capped { ", stopped by the patch limit" } else { "" }
        );
        list.push(issue("out_of_tolerance", msg));
    }
    if !list.is_empty() && report.holes_left > 0 {
        list.push(issue("scan_defects", format!("{} holes of the scan were too large to fill", report.holes_left)));
    }
    if !list.is_empty() && report.non_manifold_edges > 0 {
        let msg = format!("{} edges are shared by three or more triangles", report.non_manifold_edges);
        list.push(issue("non_manifold_edges", msg));
    }
    list
}

fn next_steps(w: &WrapOut, issues: &[super::Issue], set: &Settings) -> Vec<NextStep> {
    let has = |code: &str| issues.iter().any(|i| i.code == code);
    if issues.is_empty() {
        let why = "the solid passed: export it (pass `path` when files are allowed, else read `data`)";
        return vec![step("export.step", json!({"objects": [w.body.id]}), why)];
    }
    let mut steps = Vec::new();
    if has("scan_defects") {
        let why = "fills the holes that were too large, so the patch network can close";
        steps.push(step("mesh.fill_holes", json!({"object": set.scan, "max_edges": MAX_FILL_EDGES}), why));
        let again = json!({"scan": set.scan, "method": "freeform", "tolerance": set.tolerance});
        steps.push(step("scan.to_solid", again, "wraps the repaired scan"));
    }
    if has("out_of_tolerance") && w.patches < PATCH_LIMIT {
        let why = "a finer network: more and smaller patches (a second body; delete the first)";
        let params = json!({"mesh": set.scan, "tolerance": set.tolerance, "max_patches": PATCH_LIMIT});
        steps.push(step("surface.wrap", params, why));
    }
    if has("open_surface") && !has("scan_defects") {
        let why = "turns the sheet into a solid of constant thickness when the part is a shell (give `thickness`)";
        steps.push(step("guide.read", json!({"topic": "fit-surfaces"}), why));
    }
    steps.push(step(
        "guide.read",
        json!({"topic": "scan-to-solid"}),
        "what each verdict and issue means and how to go on by hand",
    ));
    steps
}

/// The result of a run in which nothing could be wrapped.
fn failed(set: &Settings, report: ScanSummary, why: &str) -> ToSolidOut {
    let issues = vec![issue("no_solid", format!("the scan could not be wrapped in patches: {why}"))];
    let next = vec![step(
        "guide.read",
        json!({"topic": "scan-to-solid"}),
        "what each verdict and issue means and how to go on by hand",
    )];
    out(set, report, Verdict::Fail, None, None, issues, next)
}

fn out(
    set: &Settings,
    report: ScanSummary,
    verdict: Verdict,
    w: Option<&WrapOut>,
    deviation: Option<DeviationSummary>,
    issues: Vec<super::Issue>,
    next_steps: Vec<NextStep>,
) -> ToSolidOut {
    ToSolidOut {
        verdict,
        method: Method::Freeform,
        scan: set.scan,
        body: w.map(|w| w.body.clone()),
        patches: w.map(|w| w.patches),
        tolerance: set.tolerance,
        deviation,
        worst_faces: Vec::new(),
        scan_report: report,
        modelled: Vec::new(),
        left_out: Vec::new(),
        left_out_regions: 0,
        left_out_triangles: 0,
        unassigned_triangles: 0,
        relations: 0,
        issues,
        next_steps,
    }
}

/// Wraps the scan and judges the result.
pub(super) fn freeform_solid(s: &mut Session, set: &Settings, report: ScanSummary) -> CmdResult<ToSolidOut> {
    let params = WrapIn {
        mesh: ObjRef::Id(set.scan.0),
        tolerance: Some(set.tolerance),
        min_within: Some(set.min_within),
        max_patches: None,
        refine: None,
        crease_deg: None,
        continuity: None,
        name: Some(set.name.clone()),
    };
    let w = match wrap(s, params) {
        Ok(w) => w,
        Err(e) => return Ok(failed(set, report, message_of(&e))),
    };
    let cmp = compare(
        s,
        CompareIn {
            body: ObjRef::Id(w.body.id.0),
            scan: ObjRef::Id(set.scan.0),
            tolerance: Some(set.tolerance),
            max_distance: None,
            chord: None,
            facing: true,
        },
    )?;
    let vertices = s.doc.mesh(set.scan).map_or(0, |m| m.mesh.vertex_count());
    let d = summary(&cmp, vertices);
    let mut issues = issues_of(&w, &d, set, &report);
    let diagonal = s.doc.mesh(set.scan).map_or(f64::NAN, |m| robust_diagonal(&m.mesh));
    if let Some(msg) = scale_warning(set.tolerance, diagonal) {
        issues.insert(0, issue("tolerance_exceeds_part", msg));
    }
    let next = next_steps(&w, &issues, set);
    let verdict = if issues.is_empty() {
        Verdict::Pass
    } else if d.coverage >= MIN_COVERAGE {
        Verdict::Review
    } else {
        Verdict::Fail
    };
    Ok(out(set, report, verdict, Some(&w), Some(d), issues, next))
}
